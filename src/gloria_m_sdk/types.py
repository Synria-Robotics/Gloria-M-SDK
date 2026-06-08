from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import time


class ControlMode(IntEnum):
    """Control mode enum (written to the CTRL_MODE parameter)."""

    MIT = 1
    POS_VEL = 2
    VEL = 3
    TORQUE_POS = 4


@dataclass(frozen=True)
class Limits:
    """Calibrated ranges for MIT-mode packing/unpacking (used for value scaling)."""

    pmax: float
    vmax: float
    tmax: float


@dataclass(frozen=True)
class PositionRange:
    """Additional safe position range (in radians)."""

    min: float
    max: float

    def clamp(self, value: float) -> float:
        if value < self.min:
            return self.min
        if value > self.max:
            return self.max
        return value


@dataclass
class ActuatorState:
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    updated_at: float = 0.0

    def update(self, *, position: float, velocity: float, torque: float) -> None:
        self.position = float(position)
        self.velocity = float(velocity)
        self.torque = float(torque)
        self.updated_at = time.time()
