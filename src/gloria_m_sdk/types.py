from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class ControlMode(IntEnum):
    """控制模式枚举（用于写入 CTRL_MODE 参数）。"""

    MIT = 1
    POS_VEL = 2
    VEL = 3
    TORQUE_POS = 4


@dataclass(frozen=True)
class Limits:
    """MIT 模式打包/解包的标定范围（用于数值缩放）。"""

    pmax: float
    vmax: float
    tmax: float


@dataclass(frozen=True)
class PositionRange:
    """额外的安全位置范围（弧度制）。"""

    min: float
    max: float

    def clamp(self, value: float) -> float:
        if value < self.min:
            return self.min
        if value > self.max:
            return self.max
        return value

