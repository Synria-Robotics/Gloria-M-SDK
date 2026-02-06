from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Dict, Optional

from .types import Limits, PositionRange


@dataclass
class ActuatorState:
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    updated_at: float = 0.0


@dataclass
class Actuator:
    """
    一个通用执行器配置。

    - command_id: 用于 MIT/使能/失能 等命令的 CAN ID（例如 0x01）
    - feedback_id: 反馈帧解析所使用的 CAN ID（例如 0x101）
    """

    name: str
    command_id: int
    feedback_id: int
    limits: Limits
    safe_position: Optional[PositionRange] = None

    state: ActuatorState = field(default_factory=ActuatorState)
    params: Dict[int, float] = field(default_factory=dict)

    def clamp_position(self, q: float) -> float:
        if self.safe_position is None:
            return q
        return self.safe_position.clamp(q)

    def update_state(self, *, position: float, velocity: float, torque: float) -> None:
        self.state.position = float(position)
        self.state.velocity = float(velocity)
        self.state.torque = float(torque)
        self.state.updated_at = time.time()

