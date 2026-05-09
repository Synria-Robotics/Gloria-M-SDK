from __future__ import annotations

import logging
import struct
import time
from typing import Dict, Optional

from .actuator import Actuator
from .protocol_mit import pack_f32, pack_mit_command, unpack_f32, unpack_mit_feedback
from .serial_can_adapter import CanPacket, SerialCanAdapter
from .types import ControlMode, Limits

if False:  # TYPE_CHECKING — avoid circular import
    from .transport import ICanTransport

_log = logging.getLogger(__name__)


def _u32_to_bytes_le(value: int) -> bytes:
    if not (0 <= int(value) <= 0xFFFFFFFF):
        raise ValueError("u32 out of range")
    return struct.pack("<I", int(value))


def _is_u32_param(rid: int) -> bool:
    # Consistent with original firmware: some RIDs use uint32, others use float32.
    return (7 <= rid <= 10) or (13 <= rid <= 16) or (35 <= rid <= 36)


class CanController:
    """
    High-level controller: manages actuator registration, command dispatch,
    feedback parsing and parameter read/write.
    """

    def __init__(self, adapter: "SerialCanAdapter | ICanTransport"):
        self._adapter = adapter
        self._by_can_id: Dict[int, Actuator] = {}

    def register(self, actuator: Actuator) -> None:
        # Compatibility: packets may arrive with either command_id or feedback_id.
        self._by_can_id[actuator.command_id] = actuator
        self._by_can_id[actuator.feedback_id] = actuator
        _log.debug("Registered actuator %r (cmd_id=0x%03X fb_id=0x%03X)",
                   actuator.name, actuator.command_id, actuator.feedback_id)

    def poll(self) -> None:
        for pkt in self._adapter.read_packets():
            self._handle_packet(pkt)

    def _handle_packet(self, pkt: CanPacket) -> None:
        if pkt.cmd != 0x11:
            return

        # Parameter read/write reply (data[2] == 0x33 / 0x55)
        if len(pkt.data) == 8 and pkt.data[2] in (0x33, 0x55):
            rid = pkt.data[3]
            if _is_u32_param(rid):
                value = struct.unpack("<I", pkt.data[4:8])[0]
            else:
                value = unpack_f32(pkt.data[4:8])
            act = self._by_can_id.get(pkt.can_id)
            if act is not None:
                act.params[int(rid)] = value
            return

        # State feedback (MIT-style packing; used uniformly across all control modes)
        act = self._by_can_id.get(pkt.can_id)
        if act is None and pkt.can_id == 0x00 and len(pkt.data) == 8:
            # Firmware compatibility: when CAN ID is 0, the motor ID is encoded in the low 4 bits of data[0].
            derived_id = pkt.data[0] & 0x0F
            act = self._by_can_id.get(int(derived_id))
        if act is None:
            return
        fb = unpack_mit_feedback(pkt.data, limits=act.limits)
        act.update_state(position=fb.position, velocity=fb.velocity, torque=fb.torque)

    # ---------------------------
    # Basic commands
    def enable(self, act: Actuator) -> None:
        _log.info("Enabling actuator %r (cmd_id=0x%03X)", act.name, act.command_id)
        self._control_cmd(act, 0xFC)
        time.sleep(0.1)
        self.poll()

    def disable(self, act: Actuator) -> None:
        _log.info("Disabling actuator %r (cmd_id=0x%03X)", act.name, act.command_id)
        self._control_cmd(act, 0xFD)
        time.sleep(0.01)

    def set_zero(self, act: Actuator) -> None:
        _log.warning("set_zero called on %r — this permanently resets the angle origin.",
                     act.name)
        self._control_cmd(act, 0xFE)
        time.sleep(0.1)
        self.poll()

    def _control_cmd(self, act: Actuator, cmd: int) -> None:
        data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd & 0xFF])
        self._adapter.send(act.command_id, data)

    # ---------------------------
    # Mode / parameters
    def set_control_mode(self, act: Actuator, mode: ControlMode, *, retries: int = 10, retry_s: float = 0.05) -> bool:
        from .registers import Variable
        rid = int(Variable.CTRL_MODE)
        # Clear cached value so we don't return a stale match from a previous call.
        act.params.pop(rid, None)
        self.write_param_u32(act, rid, int(mode))

        deadline = time.time() + retries * retry_s
        while time.time() < deadline:
            time.sleep(retry_s)
            self.poll()
            if rid in act.params and int(act.params[rid]) == int(mode):
                _log.info("Control mode confirmed: %s on %r", mode.name, act.name)
                return True
        _log.warning("Mode switch to %s timed out on %r (retries=%d, retry_s=%.3f)",
                     mode.name, act.name, retries, retry_s)
        return False

    def refresh_state(self, act: Actuator) -> None:
        # 0x7FF broadcast request
        can_id_l = act.command_id & 0xFF
        can_id_h = (act.command_id >> 8) & 0xFF
        data = bytes([can_id_l, can_id_h, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00])
        self._adapter.send(0x7FF, data)
        self.poll()

    def write_param_u32(self, act: Actuator, rid: int, value: int) -> None:
        can_id_l = act.command_id & 0xFF
        can_id_h = (act.command_id >> 8) & 0xFF
        data = bytes([can_id_l, can_id_h, 0x55, rid & 0xFF]) + _u32_to_bytes_le(value)
        self._adapter.send(0x7FF, data)

    def write_param_f32(self, act: Actuator, rid: int, value: float) -> None:
        """Write a float32 parameter to the motor."""
        can_id_l = act.command_id & 0xFF
        can_id_h = (act.command_id >> 8) & 0xFF
        data = bytes([can_id_l, can_id_h, 0x55, rid & 0xFF]) + pack_f32(value)
        self._adapter.send(0x7FF, data)

    def save_params(self, act: Actuator) -> None:
        """Persist all motor parameters to non-volatile storage (0xAA command)."""
        can_id_l = act.command_id & 0xFF
        can_id_h = (act.command_id >> 8) & 0xFF
        self._adapter.send(0x7FF, bytes([can_id_l, can_id_h, 0xAA, 0x00, 0, 0, 0, 0]))

    def apply_limits_and_save(self, act: Actuator, limits: Limits) -> None:
        """Write MIT scaling limits (PMAX/VMAX/TMAX) to the motor and save to flash."""
        from .registers import Variable
        self.write_param_f32(act, int(Variable.PMAX), limits.pmax)
        self.write_param_f32(act, int(Variable.VMAX), limits.vmax)
        self.write_param_f32(act, int(Variable.TMAX), limits.tmax)
        self.save_params(act)

    def read_param(self, act: Actuator, rid: int, *, timeout_s: float = 0.05) -> Optional[float]:
        """
        Send a parameter read request and return the reply value, or None on timeout.

        The cached value in act.params[rid] is cleared before the request is sent so
        that stale data is never returned.
        """
        can_id_l = act.command_id & 0xFF
        can_id_h = (act.command_id >> 8) & 0xFF
        act.params.pop(int(rid), None)
        self._adapter.send(0x7FF, bytes([can_id_l, can_id_h, 0x33, rid & 0xFF, 0, 0, 0, 0]))
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self.poll()
            if int(rid) in act.params:
                value = act.params[int(rid)]
                _log.debug("read_param rid=%d → %s on %r", rid, value, act.name)
                return value
            time.sleep(0.002)
        _log.warning(
            "read_param timed out (rid=%d, timeout=%.3fs) on %r — "
            "check CAN wiring and motor power",
            rid, timeout_s, act.name,
        )
        return None


    def send_mit(
        self,
        act: Actuator,
        *,
        kp: float,
        kd: float,
        q: float,
        dq: float,
        tau: float,
        poll: bool = True,
    ) -> None:
        q = act.clamp_position(q)
        payload = pack_mit_command(kp=kp, kd=kd, q=q, dq=dq, tau=tau, limits=act.limits)
        self._adapter.send(act.command_id, payload)
        if poll:
            self.poll()

    def send_pos_vel(self, act: Actuator, *, position: float, velocity: float, poll: bool = True) -> None:
        """
        PV mode: position + velocity (float32, float32).

        Compatibility: transmits on CAN ID = 0x100 + slave_id to match the original firmware example.
        """
        position = act.clamp_position(position)
        can_id = 0x100 + (act.command_id & 0x7FF)
        payload = pack_f32(position) + pack_f32(velocity)
        self._adapter.send(can_id, payload)
        if poll:
            self.poll()

