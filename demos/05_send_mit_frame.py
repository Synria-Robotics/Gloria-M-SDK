# Copyright (c) 2026 Synria Robotics Co., Ltd.
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
05 - Send MIT control frame

User-facing pattern:

    gripper = GloriaGripper.from_config("demos/gripper_control.toml")
    for value in (1000, 500, 0, 500, 1000):
        gripper.send_mit_for(1.0, kp=20.0, kd=1.0, q=gripper.motor_position(value), dq=0.0, tau=0.0)

This demo is for understanding the raw MIT frame fields:
    torque = kp * position_error + kd * velocity_error + tau

Positions use user units: 1000 is open, 0 is closed.
"""

from __future__ import annotations

import argparse

from gloria_m_sdk import DEFAULT_GRIPPER_CONFIG, GloriaGripper, GloriaSdkError, print_motor_snapshot


def print_status(phase: str, snapshot) -> None:
    print_motor_snapshot(f"[{phase}]", snapshot)


def _parse_positions(value: str) -> list[float]:
    return [float(part.strip()) for part in value.split(",") if part.strip()]


def main(args: argparse.Namespace) -> int:
    try:
        gripper = GloriaGripper.from_config(
            args.config,
            port=args.port or None,
            motor_status_callback=print_status,
        )
        print(f"[config] loaded {args.config}")
        print(f"[port] {gripper.port}")
        for value in args.positions:
            q = gripper.motor_position(value)
            print(
                "[api] gripper.send_mit_for("
                f"{args.duration_s:.2f}, value={value:.0f}, q={q:.3f}, "
                f"kp={args.kp}, kd={args.kd}, dq=0.0, tau=0.0)"
            )
            gripper.send_mit_for(
                args.duration_s,
                kp=args.kp,
                kd=args.kd,
                q=q,
                dq=0.0,
                tau=0.0,
                period_s=args.period_s,
            )
        print("[done]")
        return 0
    except KeyboardInterrupt:
        print("\n[ctrl+c] exiting")
        return 130
    except GloriaSdkError as exc:
        print(f"[error] {exc}")
        return 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send MIT control frame")
    parser.add_argument("--config", default=DEFAULT_GRIPPER_CONFIG, help="path to TOML config file")
    parser.add_argument("--port", default="", help="serial port override; defaults to config connection.port")
    parser.add_argument("--duration-s", type=float, default=1.0, help="how long to keep sending the MIT frame [s]")
    parser.add_argument("--period-s", type=float, default=0.01, help="send period [s]")
    parser.add_argument("--positions", type=_parse_positions, default=(1000, 500, 0, 500, 1000), help="comma-separated user positions, 1000=open and 0=closed")
    parser.add_argument("--kp", type=float, default=20.0, help="MIT position gain")
    parser.add_argument("--kd", type=float, default=1.0, help="MIT damping gain")

    raise SystemExit(main(parser.parse_args()))
