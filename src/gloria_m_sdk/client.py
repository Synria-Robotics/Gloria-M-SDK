from __future__ import annotations

import logging
import time
from typing import Optional

from .exceptions import GloriaConnectionError, GloriaModeError
from .protocol import (
    pack_control_command,
    pack_mit_command,
    pack_param_read,
    pack_param_save,
    pack_param_write_f32,
    pack_param_write_u32,
    pack_pos_vel_command,
    pack_state_request,
    parse_param_reply,
    unpack_mit_feedback,
)
from .registers import Variable
from .serial_can_adapter import CanPacket, SerialCanAdapter
from .types import ActuatorState, ControlMode, Limits, PositionRange

_log = logging.getLogger(__name__)

_BROADCAST_ID = 0x7FF
_DEFAULT_LIMITS = Limits(pmax=3.14, vmax=10.0, tmax=12.0)


class MotorClient:
    """Internal motor-level client for the Gloria-M gripper.

    The class owns connection management, motor lifecycle commands, motion
    commands, parameter read/write, and state caching. Protocol byte packing is
    delegated to :mod:`gloria_m_sdk.protocol`; serial framing is delegated to
    :class:`gloria_m_sdk.serial_can_adapter.SerialCanAdapter`.
    """

    def __init__(
        self,
        port: str,
        *,
        baudrate: int = 921_600,
        command_id: int = 0x01,
        feedback_id: int = 0x101,
        limits: Optional[Limits] = None,
        safe_position: Optional[PositionRange] = None,
        timeout: float = 0.5,
        _transport: Optional[object] = None,
    ) -> None:
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._command_id = int(command_id)
        self._feedback_id = int(feedback_id)
        self._limits = limits if limits is not None else _DEFAULT_LIMITS
        self._safe_position = safe_position
        self._transport_override = _transport

        self._adapter: Optional[object] = None
        self._state = ActuatorState()
        self._params: dict[int, int | float] = {}

    # ------------------------------------------------------------------
    # Connection management

    def connect(self, *, apply_limits: bool = True) -> None:
        import serial as _serial

        if self._adapter is not None:
            return

        if self._transport_override is not None:
            _log.debug("Using injected CAN transport")
            self._adapter = self._transport_override
        else:
            _log.info("Opening serial port %r at %d baud", self._port, self._baudrate)
            try:
                self._adapter = SerialCanAdapter(
                    self._port, baudrate=self._baudrate, timeout=self._timeout
                )
            except _serial.SerialException as exc:
                self._adapter = None
                raise GloriaConnectionError(
                    f"Cannot open port {self._port!r}: {exc}"
                ) from exc

        if apply_limits:
            self.apply_limits(self._limits)

        _log.info(
            "MotorClient connected (port=%r, cmd_id=0x%03X, fb_id=0x%03X)",
            self._port,
            self._command_id,
            self._feedback_id,
        )

    def disconnect(self) -> None:
        if self._adapter is not None:
            _log.info("Disconnecting MotorClient (port=%r)", self._port)
            if hasattr(self._adapter, "close"):
                self._adapter.close()  # type: ignore[union-attr]
            self._adapter = None

    @property
    def is_connected(self) -> bool:
        return bool(self._adapter is not None and self._adapter.is_open)

    def __enter__(self) -> "MotorClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    # ------------------------------------------------------------------
    # State and mode

    @property
    def state(self) -> ActuatorState:
        return self._state

    @property
    def current_mode(self) -> Optional[ControlMode]:
        cached = self._params.get(int(Variable.CTRL_MODE))
        if cached is None:
            return None
        try:
            return ControlMode(int(cached))
        except ValueError:
            return None

    # ------------------------------------------------------------------
    # Motor lifecycle

    def enable(self) -> None:
        _log.info("Enabling gripper (cmd_id=0x%03X)", self._command_id)
        self._send(self._command_id, pack_control_command(0xFC))
        time.sleep(0.1)
        self.poll()

    def disable(self) -> None:
        _log.info("Disabling gripper (cmd_id=0x%03X)", self._command_id)
        self._send(self._command_id, pack_control_command(0xFD))
        time.sleep(0.01)

    def set_zero(self) -> None:
        _log.warning("set_zero called; this permanently resets the angle origin")
        self._send(self._command_id, pack_control_command(0xFE))
        time.sleep(0.1)
        self.poll()

    def set_mode(
        self,
        mode: ControlMode,
        *,
        retries: int = 10,
        retry_s: float = 0.05,
    ) -> None:
        rid = int(Variable.CTRL_MODE)
        self._params.pop(rid, None)
        self.write_param_u32(rid, int(mode))

        deadline = time.time() + retries * retry_s
        while time.time() < deadline:
            time.sleep(retry_s)
            self.poll()
            if rid in self._params and int(self._params[rid]) == int(mode):
                _log.info("Control mode confirmed: %s", mode.name)
                return

        raise GloriaModeError(
            f"Motor did not confirm mode switch to {mode.name}. "
            "Check CAN wiring and motor power."
        )

    def refresh(self) -> None:
        self._send(_BROADCAST_ID, pack_state_request(self._command_id))
        self.poll()

    def poll(self) -> None:
        adapter = self._require_connected()
        for pkt in adapter.read_packets():
            self._handle_packet(pkt)

    # ------------------------------------------------------------------
    # Motion commands

    def send_mit(
        self,
        *,
        kp: float,
        kd: float,
        q: float,
        dq: float,
        tau: float,
        poll: bool = True,
    ) -> None:
        self._require_mode(ControlMode.MIT)
        payload = pack_mit_command(
            kp=kp,
            kd=kd,
            q=self._clamp_position(q),
            dq=dq,
            tau=tau,
            limits=self._limits,
        )
        self._send(self._command_id, payload)
        if poll:
            self.poll()

    def send_pos_vel(self, *, position: float, velocity: float, poll: bool = True) -> None:
        self._require_mode(ControlMode.POS_VEL)
        can_id = 0x100 + (self._command_id & 0x7FF)
        payload = pack_pos_vel_command(self._clamp_position(position), velocity)
        self._send(can_id, payload)
        if poll:
            self.poll()

    # ------------------------------------------------------------------
    # Parameters

    def read_param(self, rid: int | Variable, *, timeout_s: float = 0.05) -> Optional[int | float]:
        rid_int = int(rid)
        self._params.pop(rid_int, None)
        self._send(_BROADCAST_ID, pack_param_read(self._command_id, rid_int))

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self.poll()
            if rid_int in self._params:
                return self._params[rid_int]
            time.sleep(0.002)

        _log.warning(
            "read_param timed out (rid=%d, timeout=%.3fs); check CAN wiring and motor power",
            rid_int,
            timeout_s,
        )
        return None

    def write_param_f32(self, rid: int | Variable, value: float) -> None:
        self._send(_BROADCAST_ID, pack_param_write_f32(self._command_id, int(rid), value))

    def write_param_u32(self, rid: int | Variable, value: int) -> None:
        self._send(_BROADCAST_ID, pack_param_write_u32(self._command_id, int(rid), value))

    def save_params(self) -> None:
        self._send(_BROADCAST_ID, pack_param_save(self._command_id))

    def apply_limits(self, limits: Limits) -> None:
        self._limits = limits
        self.write_param_f32(Variable.PMAX, limits.pmax)
        self.write_param_f32(Variable.VMAX, limits.vmax)
        self.write_param_f32(Variable.TMAX, limits.tmax)
        self.save_params()

    # ------------------------------------------------------------------
    # Internals

    def _require_connected(self):
        if self._adapter is None:
            raise GloriaConnectionError("Not connected. Call MotorClient.connect() first.")
        return self._adapter

    def _require_mode(self, expected: ControlMode) -> None:
        actual = self.current_mode
        if actual is None:
            raise GloriaModeError(
                f"No control mode has been confirmed yet. "
                f"Call gripper.set_mode(ControlMode.{expected.name}) before sending motion commands."
            )
        if actual != expected:
            raise GloriaModeError(
                f"Wrong control mode: motor is in {actual.name}, but this command "
                f"requires {expected.name}. Call gripper.set_mode(ControlMode.{expected.name}) first."
            )

    def _send(self, can_id: int, data8: bytes) -> None:
        self._require_connected().send(can_id, data8)

    def _clamp_position(self, value: float) -> float:
        if self._safe_position is None:
            return float(value)
        return self._safe_position.clamp(float(value))

    def _handle_packet(self, pkt: CanPacket) -> None:
        if pkt.cmd != 0x11:
            return

        param = parse_param_reply(pkt.data)
        if param is not None:
            rid, value = param
            if self._packet_matches_gripper(pkt):
                self._params[int(rid)] = value
            return

        if not self._packet_matches_gripper(pkt):
            return

        fb = unpack_mit_feedback(pkt.data, limits=self._limits)
        self._state.update(position=fb.position, velocity=fb.velocity, torque=fb.torque)

    def _packet_matches_gripper(self, pkt: CanPacket) -> bool:
        if pkt.can_id in (self._command_id, self._feedback_id):
            return True
        if pkt.can_id == 0x00 and len(pkt.data) == 8:
            return (pkt.data[0] & 0x0F) == (self._command_id & 0x0F)
        return False
