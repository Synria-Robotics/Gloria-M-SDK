# Copyright (c) 2026 Synria Robotics Co., Ltd.
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
00 - Read-only CAN diagnostic

User-facing pattern:

    gripper = GloriaGripper.from_config("demos/gripper_control.toml")
    result = gripper.check_connection()

This demo does not enable the motor, switch mode, write parameters, save flash,
or move the gripper.
"""

from __future__ import annotations

import argparse
from gloria_m_sdk import (
    ControlMode,
    DEFAULT_GRIPPER_CONFIG,
    DiagnosticResult,
    GloriaGripper,
    GloriaSdkError,
)


def _mode_text(value: int | float | None) -> str:
    if value is None:
        return "no reply"
    try:
        return f"{int(value)} ({ControlMode(int(value)).name})"
    except ValueError:
        return str(value)


def _parse_id_range(value: str) -> list[int]:
    ids: list[int] = []
    for part in value.split(","):
        text = part.strip()
        if not text:
            continue
        if "-" in text:
            start_text, end_text = text.split("-", 1)
            start = int(start_text, 0)
            end = int(end_text, 0)
            if end < start:
                raise ValueError(f"invalid descending id range: {text}")
            ids.extend(range(start, end + 1))
        else:
            ids.append(int(text, 0))
    if not ids:
        raise ValueError("empty id range")
    return ids


def _print_ports() -> int:
    ports = GloriaGripper.list_ports()
    if not ports:
        print("[ports] no serial ports found")
        return 0
    print("[ports] available serial ports:")
    for device, description in ports:
        print(f"  {device} | {description}")
    return 0


def _print_result(result: DiagnosticResult) -> int:
    print(f"[can] CTRL_MODE={_mode_text(result.ctrl_mode)}")
    print(f"[can] PMAX={result.pmax} VMAX={result.vmax} TMAX={result.tmax}")
    if result.state.has_feedback:
        print(
            f"[can] state q={result.state.position:+.3f} "
            f"dq={result.state.velocity:+.3f} tau={result.state.torque:+.3f}"
        )
    else:
        print("[can] state = no feedback")

    if result.motor_replied:
        print("[result] CAN communication looks OK")
        return 0

    print("[result] serial opened, but motor did not reply on CAN")
    print("[hint] check motor power, CANH/CANL wiring, common GND, termination, CAN ID, and CAN baud rate")
    return 2


def main(args: argparse.Namespace) -> int:
    if args.list_ports:
        return _print_ports()

    try:
        gripper = GloriaGripper.from_config(args.config, port=args.port)
        print(f"[config] loaded {args.config}")
        print(
            f"[serial] port={gripper.port} baudrate={gripper.config.connection.baudrate}"
        )
        print(
            f"[can] command_id=0x{gripper.config.connection.command_id:03X} "
            f"feedback_id=0x{gripper.config.connection.feedback_id:03X}"
        )

        if args.scan_ids:
            ids = _parse_id_range(args.scan_ids)
            print(f"[scan] scanning {len(ids)} command IDs")
            hits = gripper.scan_ids(ids, read_timeout_s=args.timeout)
            if not hits:
                print("[scan] no motor replied")
                return 2
            for hit in hits:
                print(
                    f"[scan] id=0x{hit.command_id:03X} fb=0x{hit.feedback_id:03X} "
                    f"CTRL_MODE={_mode_text(hit.ctrl_mode)}"
                )
            return 0

        return _print_result(gripper.check_connection(read_timeout_s=args.timeout))

    except GloriaSdkError as exc:
        print(f"[error] {exc}")
        return 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Read-only CAN communication diagnostic")
    parser.add_argument("--config", default=DEFAULT_GRIPPER_CONFIG, help="path to TOML config file")
    parser.add_argument("--port", default="", help="serial port override; defaults to config connection.port")
    parser.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    parser.add_argument("--timeout", type=float, default=0.2, help="parameter read timeout [s]")
    parser.add_argument("--scan-ids", help="read-only scan command IDs, e.g. '1-10' or '0x01,0x02,0x10'")

    raise SystemExit(main(parser.parse_args()))
