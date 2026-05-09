from __future__ import annotations

# Safe position range for the gripper [MIT_SAFE_Q_MIN, MIT_SAFE_Q_MAX] (rad).
# Convention: 0.0 = fully open; increasing in the positive direction moves toward closed; max ~2.7 rad.
MIT_SAFE_Q_MIN: float = 0.0
MIT_SAFE_Q_MAX: float = 2.7
