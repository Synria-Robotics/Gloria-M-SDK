# Copyright (c) 2026 Synria Robotics Co., Ltd.
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
08 - MIT gripper user example

User-facing pattern:

    with GloriaGripper.from_config("demos/gripper_control.toml") as gripper:
        gripper.move_to(1000)
        gripper.move_to(0)

User units:
    1000 = fully open
    0    = close with stall/contact protection
"""

from __future__ import annotations

import argparse
from dataclasses import replace

from gloria_m_sdk import DEFAULT_GRIPPER_CONFIG, GloriaGripper, GloriaSdkError


def _apply_force_level(gripper: GloriaGripper, level: int) -> float:
    force_level = max(1, min(10, int(level)))
    cfg = gripper.controller.config
    closing_direction = 1.0 if cfg.close_limit > cfg.open_pos else -1.0
    hold_tau = closing_direction * (0.075 * force_level)
    gripper.controller.config = replace(cfg, hold_tau_ff=hold_tau)
    return hold_tau


def _apply_gentle_open(gripper: GloriaGripper) -> None:
    cfg = gripper.controller.config
    gripper.controller.config = replace(
        cfg,
        open_kp=10.0,
        open_kd=2.0,
        open_position_tolerance=0.06,
        open_velocity_tolerance=0.10,
        open_timeout_s=15.0,
    )


def main(args: argparse.Namespace) -> int:
    try:
        with GloriaGripper.from_config(args.config, port=args.port or None) as gripper:
            print(f"[start] port={gripper.port} force_level={args.force_level}/10 stall_protection=on")
            _apply_gentle_open(gripper)
            hold_tau = _apply_force_level(gripper, args.force_level)
            print(f"[force] hold_tau_ff={hold_tau:+.3f}")

            print("[api] gripper.move_to(1000)  # gentle open")
            gripper.move_to(1000)

            print("[api] gripper.move_to(0, stall_protection=True)  # hold until Ctrl+C")
            gripper.move_to(0, stall_protection=True)

    except KeyboardInterrupt:
        print("\n[ctrl+c] exiting")
        return 130
    except GloriaSdkError as exc:
        print(f"[error] {exc}")
        return 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the MIT gripper user example")
    parser.add_argument("--config", default=DEFAULT_GRIPPER_CONFIG, help="path to TOML config file")
    parser.add_argument("--port", default="", help="serial port override; defaults to config connection.port")
    parser.add_argument("--force-level", type=int, choices=range(1, 11), default=4, help="holding force level, 1 to 10")

    raise SystemExit(main(parser.parse_args()))
