# Copyright (c) 2026 Synria Robotics Co., Ltd.
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
01 - Enable / disable motor

User-facing pattern:

    gripper = GloriaGripper.from_config("demos/gripper_control.toml")
    gripper.enable_for(2.0)
    gripper.disable()

This demo is for checking motor enable/disable only. It does not command open,
close, or grip motion.
"""

from __future__ import annotations

import argparse

from gloria_m_sdk import DEFAULT_GRIPPER_CONFIG, ControlMode, GloriaGripper, GloriaSdkError, print_motor_snapshot


def print_status(phase: str, snapshot) -> None:
    print_motor_snapshot(f"[{phase}]", snapshot)


def main(args: argparse.Namespace) -> int:
    try:
        gripper = GloriaGripper.from_config(
            args.config,
            port=args.port or None,
            motor_status_callback=print_status,
        )
        print(f"[config] loaded {args.config}")
        print(f"[port] {gripper.port}")

        if args.action == "disable":
            print("[api] gripper.disable()")
            try:
                gripper.connect(mode=None, enable=False, apply_limits=False, refresh=False)
                gripper.disable()
            finally:
                gripper.disconnect(disable=False)
            print("[done]")
            return 0

        mode = {"mit": ControlMode.MIT, "pv": ControlMode.POS_VEL, "none": None}[args.mode]
        mode_text = "none" if mode is None else mode.name
        print(f"[api] gripper.enable_for({args.duration_s:.2f}, mode={mode_text})")
        gripper.enable_for(args.duration_s, mode=mode)
        print("[done]")
        return 0
    except KeyboardInterrupt:
        print("\n[ctrl+c] exiting")
        return 130
    except GloriaSdkError as exc:
        print(f"[error] {exc}")
        return 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Enable motor briefly, then disable it")
    parser.add_argument("--config", default=DEFAULT_GRIPPER_CONFIG, help="path to TOML config file")
    parser.add_argument("--port", default="", help="serial port override; defaults to config connection.port")
    parser.add_argument(
        "--action",
        choices=("enable", "disable"),
        default="enable",
        help="enable briefly, or only send disable",
    )
    parser.add_argument("--duration-s", type=float, default=2.0, help="how long to keep the motor enabled [s]")
    parser.add_argument("--mode", choices=("mit", "pv", "none"), default="mit", help="optional mode before enable")

    raise SystemExit(main(parser.parse_args()))
