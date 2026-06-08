# Copyright (c) 2026 Synria Robotics Co., Ltd.
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
03 - Set current motor position as zero

User-facing pattern:

    gripper = GloriaGripper.from_config("demos/gripper_control.toml")
    gripper.connect(mode=ControlMode.MIT, enable=True)
    print(gripper.snapshot())
    input("Press Enter to set zero...")
    gripper.set_zero()

Setting zero changes the motor's angle origin. Run this only when the gripper is
physically placed at the intended zero position.
"""

from __future__ import annotations

import argparse
from gloria_m_sdk import ControlMode, DEFAULT_GRIPPER_CONFIG, GloriaGripper, GloriaSdkError, print_motor_snapshot


def main(args: argparse.Namespace) -> int:
    gripper = None
    try:
        gripper = GloriaGripper.from_config(args.config, port=args.port or None)
        print(f"[config] loaded {args.config}")
        print(f"[port] {gripper.port}")
        print("[api] gripper.connect(mode=ControlMode.MIT, enable=True)")
        gripper.connect(mode=ControlMode.MIT, enable=True, apply_limits=False)

        before = gripper.refresh()
        print_motor_snapshot("[current]", before, gripper=gripper)

        input("[confirm] Press Enter to set current position as zero, or Ctrl+C to cancel...")

        print("[api] gripper.set_zero()")
        after = gripper.set_zero()
        print_motor_snapshot("[after]", after, gripper=gripper)
        print("[done]")
        return 0

    except KeyboardInterrupt:
        print("\n[cancel] set_zero was not sent")
        return 130
    except GloriaSdkError as exc:
        print(f"[error] {exc}")
        return 2
    finally:
        if gripper is not None:
            gripper.disconnect(disable=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Set current motor position as zero")
    parser.add_argument("--config", default=DEFAULT_GRIPPER_CONFIG, help="path to TOML config file")
    parser.add_argument("--port", default="", help="serial port override; defaults to config connection.port")

    raise SystemExit(main(parser.parse_args()))
