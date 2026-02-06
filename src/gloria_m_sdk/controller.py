from __future__ import annotations

import struct
import time
from typing import Dict, Optional

from .actuator import Actuator
from .protocol_mit import MitFeedback, pack_f32, pack_mit_command, unpack_f32, unpack_mit_feedback
from .serial_can_adapter import CanPacket, SerialCanAdapter
from .types import ControlMode


def _u32_to_bytes_le(value: int) -> bytes:
    if not (0 <= int(value) <= 0xFFFFFFFF):
        raise ValueError("u32 out of range")
    return struct.pack("<I", int(value))


def _is_u32_param(rid: int) -> bool:
    # 与原始例程一致：某些 RID 用 uint32 传输，其它用 float32。
    return (7 <= rid <= 10) or (13 <= rid <= 16) or (35 <= rid <= 36)


class CanController:
    """
    高层控制器：管理执行器注册、发送命令、解包反馈与参数读写。
    """

    def __init__(self, adapter: SerialCanAdapter):
        self._adapter = adapter
        self._by_can_id: Dict[int, Actuator] = {}

    def register(self, actuator: Actuator) -> None:
        # 兼容：既可能用 command_id 回包，也可能用 feedback_id 回包。
        self._by_can_id[actuator.command_id] = actuator
        self._by_can_id[actuator.feedback_id] = actuator

    def poll(self) -> None:
        for pkt in self._adapter.read_packets():
            self._handle_packet(pkt)

    def _handle_packet(self, pkt: CanPacket) -> None:
        if pkt.cmd != 0x11:
            return

        # 参数读写回包（data[2] == 0x33/0x55）
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

        # 状态回包（MIT 风格打包，原例程对所有模式统一用这套解包）
        act = self._by_can_id.get(pkt.can_id)
        if act is None and pkt.can_id == 0x00 and len(pkt.data) == 8:
            # 兼容某些固件：CANID=0 时，把 ID 塞在 data[0] 的低 4 bit。
            derived_id = pkt.data[0] & 0x0F
            act = self._by_can_id.get(int(derived_id))
        if act is None:
            return
        fb = unpack_mit_feedback(pkt.data, limits=act.limits)
        act.update_state(position=fb.position, velocity=fb.velocity, torque=fb.torque)

    # ---------------------------
    # 基础命令
    def enable(self, act: Actuator) -> None:
        self._control_cmd(act, 0xFC)
        time.sleep(0.1)
        self.poll()

    def disable(self, act: Actuator) -> None:
        self._control_cmd(act, 0xFD)
        time.sleep(0.01)

    def set_zero(self, act: Actuator) -> None:
        self._control_cmd(act, 0xFE)
        time.sleep(0.1)
        self.poll()

    def _control_cmd(self, act: Actuator, cmd: int) -> None:
        data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd & 0xFF])
        self._adapter.send(act.command_id, data)

    # ---------------------------
    # 模式/参数
    def set_control_mode(self, act: Actuator, mode: ControlMode, *, retries: int = 10, retry_s: float = 0.05) -> bool:
        RID_CTRL_MODE = 10
        self.write_param_u32(act, RID_CTRL_MODE, int(mode))

        deadline = time.time() + retries * retry_s
        while time.time() < deadline:
            time.sleep(retry_s)
            self.poll()
            if int(RID_CTRL_MODE) in act.params and int(act.params[RID_CTRL_MODE]) == int(mode):
                return True
        return False

    def refresh_state(self, act: Actuator) -> None:
        # 0x7FF 广播请求
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

    # ---------------------------
    # 控制模式命令
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
        PV 模式：位置 + 速度（float32, float32）

        兼容原始例程：发送 CANID = 0x100 + slave_id
        """
        position = act.clamp_position(position)
        can_id = 0x100 + (act.command_id & 0x7FF)
        payload = pack_f32(position) + pack_f32(velocity)
        self._adapter.send(can_id, payload)
        if poll:
            self.poll()

