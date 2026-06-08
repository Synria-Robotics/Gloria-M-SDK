"""
Gloria-M SDK (Python)

Recommended entry point:

    from gloria_m_sdk import GloriaGripper

    with GloriaGripper.from_config("demos/gripper_control.toml") as gripper:
        gripper.move_to(1000)
        gripper.move_to(500)
        gripper.move_to(0)
        gripper.move_to(1000)
"""

from .constants import MIT_SAFE_Q_MAX, MIT_SAFE_Q_MIN
from .exceptions import (
    GloriaCommunicationError,
    GloriaConfigError,
    GloriaConnectionError,
    GloriaModeError,
    GloriaSdkError,
)
from .gripper_control import (
    GripperControlConfig,
    GripperControlState,
    GripperControlStatus,
    GripperController,
    MotorHealth,
)
from .gripper_api import (
    CanIdScanHit,
    DEFAULT_GRIPPER_CONFIG,
    DiagnosticResult,
    GloriaGripper,
    GripperConfig,
    GripperConnectionConfig,
    GripperLoopConfig,
    MotorSnapshot,
    PvMoveConfig,
    PvMoveStatus,
    format_motor_snapshot,
    load_gripper_config,
    print_motor_snapshot,
    resolve_gripper_port,
)
from .registers import Variable
from .serial_can_adapter import CanPacket, SerialCanAdapter
from .transport import FakeCanAdapter, ICanTransport
from .types import ActuatorState, ControlMode, Limits, PositionRange

__all__ = [
    "GloriaGripper",
    "GloriaSdkError",
    "GloriaConnectionError",
    "GloriaCommunicationError",
    "GloriaConfigError",
    "GloriaModeError",
    "GripperControlConfig",
    "GripperControlState",
    "GripperControlStatus",
    "GripperController",
    "GripperConfig",
    "GripperConnectionConfig",
    "GripperLoopConfig",
    "PvMoveConfig",
    "PvMoveStatus",
    "DiagnosticResult",
    "MotorSnapshot",
    "format_motor_snapshot",
    "print_motor_snapshot",
    "CanIdScanHit",
    "DEFAULT_GRIPPER_CONFIG",
    "load_gripper_config",
    "resolve_gripper_port",
    "MotorHealth",
    "ActuatorState",
    "ControlMode",
    "Limits",
    "PositionRange",
    "SerialCanAdapter",
    "CanPacket",
    "Variable",
    "MIT_SAFE_Q_MAX",
    "MIT_SAFE_Q_MIN",
    "ICanTransport",
    "FakeCanAdapter",
]
