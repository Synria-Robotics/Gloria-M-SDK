# Copyright (c) 2026 Synria Robotics Co., Ltd.
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
07 - PV gripper open, gentle close, hold

User-facing pattern:

    gripper = GloriaGripper.from_config("demos/gripper_control.toml")
    gripper.connect(mode=ControlMode.POS_VEL)
    gripper.pv_open()
    gripper.pv_close()
    gripper.pv_hold_closed(2.0)

PV mode is useful for simple position/velocity motion tests. For real object
gripping with contact detection and stall protection, use the MIT API demo.
"""

from __future__ import annotations

import argparse

from gloria_m_sdk import ControlMode, DEFAULT_GRIPPER_CONFIG, GloriaGripper, GloriaSdkError, PvMoveStatus, print_motor_snapshot


def _print_pv_status(phase: str, status: PvMoveStatus) -> None:
    done = " settled" if status.settled else " timeout" if status.timed_out else ""
    print(
        f"[{phase}] t={status.elapsed_s:5.2f}s target={status.target:+.3f} "
        f"q={status.position:+.3f} dq={status.velocity:+.3f} tau={status.torque:+.3f}{done}"
    )


def main(args: argparse.Namespace) -> int:
    try:
        gripper = GloriaGripper.from_config(
            args.config,
            port=args.port or None,
            open_velocity=args.open_vel,
            close_velocity=args.close_vel,
            pv_status_callback=_print_pv_status,
        )
        print(f"[port] {gripper.port}")
        try:
            gripper.connect(mode=ControlMode.POS_VEL)
            snap = gripper.snapshot()
            print_motor_snapshot("[init]", snap)

            print("[api] gripper.pv_open()")
            gripper.pv_open()

            print("[api] gripper.pv_close()")
            gripper.pv_close()

            print(f"[api] gripper.pv_hold_closed({args.hold_s:.2f})")
            gripper.pv_hold_closed(args.hold_s)

            print("[done]")
            return 0
        finally:
            gripper.disconnect()

    except KeyboardInterrupt:
        print("\n[ctrl+c] exiting")
        return 130
    except GloriaSdkError as exc:
        print(f"[error] {exc}")
        return 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PV mode gripper open, gentle close, hold")
    parser.add_argument("--config", default=DEFAULT_GRIPPER_CONFIG, help="path to TOML config file")
    parser.add_argument("--port", default="", help="serial port override; defaults to config connection.port")
    parser.add_argument("--open-vel", type=float, default=1.0, help="opening velocity [rad/s]")
    parser.add_argument("--close-vel", type=float, default=0.3, help="closing velocity [rad/s]")
    parser.add_argument("--hold-s", type=float, default=2.0, help="closed-position hold duration [s]")

    raise SystemExit(main(parser.parse_args()))
