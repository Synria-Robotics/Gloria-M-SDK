from __future__ import annotations

import struct

from .serial_can_adapter import SerialCanAdapter
from .types import Limits


def apply_limits_and_save(adapter: SerialCanAdapter, *, motor_id: int, limits: Limits) -> None:
    """
    Write MIT scaling limits to the motor parameters and send a save-parameters command.

    - PMAX (RID=21) float32
    - VMAX (RID=22) float32
    - TMAX (RID=23) float32
    - SAVE (0xAA)
    """
    can_id_l = motor_id & 0xFF
    can_id_h = (motor_id >> 8) & 0xFF

    adapter.send(0x7FF, bytes([can_id_l, can_id_h, 0x55, 21]) + struct.pack("<f", float(limits.pmax)))
    adapter.send(0x7FF, bytes([can_id_l, can_id_h, 0x55, 22]) + struct.pack("<f", float(limits.vmax)))
    adapter.send(0x7FF, bytes([can_id_l, can_id_h, 0x55, 23]) + struct.pack("<f", float(limits.tmax)))

    adapter.send(0x7FF, bytes([can_id_l, can_id_h, 0xAA, 0x00, 0, 0, 0, 0]))

