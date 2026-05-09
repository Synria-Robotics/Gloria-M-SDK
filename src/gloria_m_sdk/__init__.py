"""
Gloria-M SDK (Python)
Synria Robotics: Last updated May 9, 2026

Recommended entry point
-----------------------
    from gloria_m_sdk import GloriaGripper, ControlMode

    with GloriaGripper("COM5") as g:
        g.motor.set_mode(ControlMode.POS_VEL)
        g.motor.enable()
        g.motion.send_pos_vel(position=2.5, velocity=1.0)
        print(g.state.position)

Layer overview
--------------
    GloriaGripper (client.py)          ← facade / recommended entry point
      ├─ MotorAPI  (api/motor_api.py)  ← enable / disable / set_mode / poll
      ├─ MotionAPI (api/motion_api.py) ← send_mit / send_pos_vel
      └─ ParamAPI  (api/param_api.py)  ← read / write / save / apply_limits
           │
           ▼
    CanController (controller.py)      ← command dispatch, feedback parsing
           │
           ▼
    protocol_mit.py                    ← MIT bit-packing, float32 codec
           │
           ▼
    SerialCanAdapter (serial_can_adapter.py)  ← transport: serial framing
"""

# --- Facade (recommended for new code) ---
from .client import GloriaGripper

# --- Domain APIs ---
from .api import MotorAPI, MotionAPI, ParamAPI

# --- Exceptions ---
from .exceptions import (
    GloriaSdkError,
    GloriaConnectionError,
    GloriaCommunicationError,
    GloriaConfigError,
    GloriaModeError,
)

# --- Types / models ---
from .types import ControlMode, Limits, PositionRange
from .actuator import Actuator, ActuatorState

# --- Lower-level (power users / existing demos) ---
from .serial_can_adapter import SerialCanAdapter
from .controller import CanController
from .param_config import apply_limits_and_save  # legacy; prefer ctrl.apply_limits_and_save
from .constants import MIT_SAFE_Q_MAX, MIT_SAFE_Q_MIN
from .registers import Variable
from .gripper_baseline import BaselinePoint, TorqueBaseline

__all__ = [
    # Facade
    "GloriaGripper",
    # Domain APIs
    "MotorAPI",
    "MotionAPI",
    "ParamAPI",
    # Exceptions
    "GloriaSdkError",
    "GloriaConnectionError",
    "GloriaCommunicationError",
    "GloriaConfigError",
    "GloriaModeError",
    # Types
    "Actuator",
    "ActuatorState",
    "ControlMode",
    "Limits",
    "PositionRange",
    # Lower-level / power users
    "CanController",
    "SerialCanAdapter",
    "apply_limits_and_save",
    # Data
    "BaselinePoint",
    "TorqueBaseline",
    "Variable",
    # Constants
    "MIT_SAFE_Q_MAX",
    "MIT_SAFE_Q_MIN",
]

