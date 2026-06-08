# Copyright (c) 2026 Synria Robotics Co., Ltd.
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
04 - Switch motor control mode

User-facing pattern:

    gripper = GloriaGripper.from_config("demos/gripper_control.toml")
    gripper.connect(mode=None, enable=False)
    gripper.set_mode(ControlMode.MIT)

This demo switches the motor control mode only. It does not enable the motor
and does not command open, close, or grip motion.
"""

from __future__ import annotations

import argparse
from gloria_m_sdk import DEFAULT_GRIPPER_CONFIG, ControlMode, GloriaGripper, GloriaSdkError, print_motor_snapshot


def main(args: argparse.Namespace) -> int:
    gripper = None
    try:
        gripper = GloriaGripper.from_config(args.config, port=args.port or None)
        print(f"[config] loaded {args.config}")
        print(f"[port] {gripper.port}")
        gripper.connect(mode=None, enable=False, apply_limits=False)
        before = gripper.snapshot()
        print_motor_snapshot("[before]", before)

        mode = ControlMode.MIT if args.mode == "mit" else ControlMode.POS_VEL
        print(f"[api] gripper.set_mode(ControlMode.{mode.name})")
        after = gripper.set_mode(mode)
        print_motor_snapshot("[after]", after)

        current = gripper.current_mode
        current_text = "unknown" if current is None else current.name
        print(f"[mode] current_mode={current_text}")
        print("[done]")
        return 0

    except GloriaSdkError as exc:
        print(f"[error] {exc}")
        return 2
    finally:
        if gripper is not None:
            gripper.disconnect(disable=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Switch motor control mode")
    parser.add_argument("--config", default=DEFAULT_GRIPPER_CONFIG, help="path to TOML config file")
    parser.add_argument("--port", default="", help="serial port override; defaults to config connection.port")
    parser.add_argument("--mode", choices=("mit", "pv"), default="mit", help="target mode")

    raise SystemExit(main(parser.parse_args()))
