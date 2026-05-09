from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Tuple

from .types import Limits


def _clamp(x: float, x_min: float, x_max: float) -> float:
    if x < x_min:
        return x_min
    if x > x_max:
        return x_max
    return x


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    x = _clamp(x, x_min, x_max)
    span = x_max - x_min
    if span <= 0:
        raise ValueError("x_max must be greater than x_min")
    data_norm = (x - x_min) / span
    return int(data_norm * ((1 << bits) - 1))


def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    data_norm = float(x) / float((1 << bits) - 1)
    return data_norm * span + x_min


def pack_f32(value: float) -> bytes:
    return struct.pack("<f", float(value))


def unpack_f32(data4: bytes) -> float:
    return struct.unpack("<f", data4)[0]


def pack_mit_command(
    *,
    kp: float,
    kd: float,
    q: float,
    dq: float,
    tau: float,
    limits: Limits,
) -> bytes:
    """
    Pack an MIT control frame into 8 bytes:
    - q:   16 bit
    - dq:  12 bit
    - kp:  12 bit (0..500)
    - kd:  12 bit (0..5)
    - tau: 12 bit
    """
    kp_uint = float_to_uint(kp, 0.0, 500.0, 12)
    kd_uint = float_to_uint(kd, 0.0, 5.0, 12)

    q_uint = float_to_uint(q, -limits.pmax, limits.pmax, 16)
    dq_uint = float_to_uint(dq, -limits.vmax, limits.vmax, 12)
    tau_uint = float_to_uint(tau, -limits.tmax, limits.tmax, 12)

    b0 = (q_uint >> 8) & 0xFF
    b1 = q_uint & 0xFF
    b2 = (dq_uint >> 4) & 0xFF
    b3 = ((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F)
    b4 = kp_uint & 0xFF
    b5 = (kd_uint >> 4) & 0xFF
    b6 = ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F)
    b7 = tau_uint & 0xFF
    return bytes([b0, b1, b2, b3, b4, b5, b6, b7])


@dataclass(frozen=True)
class MitFeedback:
    position: float
    velocity: float
    torque: float


def unpack_mit_feedback(data8: bytes, *, limits: Limits) -> MitFeedback:
    """
    Unpack an MIT feedback frame:
    - q_uint:   data[1:3]
    - dq_uint:  data[3] | high-4-bit(data[4])
    - tau_uint: low-4-bit(data[4]) | data[5]
    """
    if len(data8) != 8:
        raise ValueError("MIT feedback must be exactly 8 bytes")
    q_uint = (data8[1] << 8) | data8[2]
    dq_uint = (data8[3] << 4) | (data8[4] >> 4)
    tau_uint = ((data8[4] & 0x0F) << 8) | data8[5]

    q = uint_to_float(q_uint, -limits.pmax, limits.pmax, 16)
    dq = uint_to_float(dq_uint, -limits.vmax, limits.vmax, 12)
    tau = uint_to_float(tau_uint, -limits.tmax, limits.tmax, 12)
    return MitFeedback(position=q, velocity=dq, torque=tau)

