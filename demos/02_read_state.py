# Copyright (c) 2026 Synria Robotics Co., Ltd.
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
02 - Read gripper position and state

User-facing pattern:

    gripper = GloriaGripper.from_config("demos/gripper_control.toml")
    gripper.connect(mode=None, enable=False)
    state = gripper.refresh()

This demo reads the latest motor feedback. By default it does not enable the
motor or command motion. If your motor only reports feedback after control
frames, run with --enable; the demo will disable automatically on exit.
"""

from __future__ import annotations

import argparse
import time

from gloria_m_sdk import ControlMode, DEFAULT_GRIPPER_CONFIG, GloriaGripper, GloriaSdkError, print_motor_snapshot


def main(args: argparse.Namespace) -> int:
    gripper = None
    try:
        samples = 1 if args.single or args.once else args.samples
        period_s = 1.0 / max(1.0, args.fps)
        gripper = GloriaGripper.from_config(args.config, port=args.port or None)
        print(f"[config] loaded {args.config}")
        print(f"[port] {gripper.port}")

        if args.enable:
            print("[api] gripper.connect(mode=ControlMode.MIT, enable=True)")
            gripper.connect(mode=ControlMode.MIT, enable=True, apply_limits=False)
        else:
            print("[api] gripper.connect(mode=None, enable=False)")
            gripper.connect(mode=None, enable=False, apply_limits=False, refresh=False)

        count = 0
        hinted_no_feedback = False
        while samples is None or count < max(1, samples):
            count += 1
            snapshot = gripper.refresh()
            total = "..." if samples is None else str(max(1, samples))
            print_motor_snapshot(f"[state {count}/{total}]", snapshot, gripper=gripper)
            if not snapshot.has_feedback and not hinted_no_feedback:
                print("[hint] no feedback received; try: python demos/02_read_state.py --enable --single")
                hinted_no_feedback = True
            if samples is None or count < max(1, samples):
                time.sleep(max(0.0, period_s))

        print("[done]")
        return 0
    except KeyboardInterrupt:
        print("\n[ctrl+c] exiting")
        return 130
    except GloriaSdkError as exc:
        print(f"[error] {exc}")
        return 2
    finally:
        if gripper is not None:
            gripper.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Read gripper position, velocity, torque, and feedback status")

    parser.add_argument("--config", default=DEFAULT_GRIPPER_CONFIG, help="path to TOML config file")
    parser.add_argument("--port", type=str, default="", help="serial port override; defaults to config connection.port")
    parser.add_argument("--enable", action="store_true", help="enable in MIT mode while reading feedback")
    parser.add_argument("--single", action="store_true", help="print state once; default is continuous print")
    parser.add_argument("--once", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--samples", type=int, help="number of state samples to print; default is continuous")
    parser.add_argument("--fps", type=float, default=10.0, help="target frames per second for continuous mode")

    raise SystemExit(main(parser.parse_args()))
