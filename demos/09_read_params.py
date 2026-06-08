# Copyright (c) 2026 Synria Robotics Co., Ltd.
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
09 - Read motor parameters

User-facing pattern:

    gripper = GloriaGripper.from_config("demos/gripper_control.toml")
    gripper.connect(mode=None, enable=False)
    pmax = gripper.read_param("PMAX")

This demo reads motor parameters only. It does not enable the motor, switch mode,
write parameters, save flash, or move the gripper.
"""

from __future__ import annotations

import argparse

from gloria_m_sdk import DEFAULT_GRIPPER_CONFIG, GloriaGripper, GloriaSdkError, Variable


DEFAULT_PARAMS = (
    Variable.MST_ID,
    Variable.ESC_ID,
    Variable.CTRL_MODE,
    Variable.PMAX,
    Variable.VMAX,
    Variable.TMAX,
    Variable.can_br,
)


def _parse_param(value: str) -> int:
    text = value.strip()
    if not text:
        raise argparse.ArgumentTypeError("parameter cannot be empty")
    for variable in Variable:
        if variable.name.lower() == text.lower():
            return int(variable)
    try:
        return int(text, 0)
    except ValueError as exc:
        names = ", ".join(variable.name for variable in DEFAULT_PARAMS)
        raise argparse.ArgumentTypeError(f"unknown parameter '{value}'. Common names: {names}") from exc


def _param_name(rid: int) -> str:
    try:
        return Variable(int(rid)).name
    except ValueError:
        return str(int(rid))


def _format_value(value: int | float | None) -> str:
    if value is None:
        return "no reply"
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def main(args: argparse.Namespace) -> int:
    gripper = None
    try:
        gripper = GloriaGripper.from_config(args.config, port=args.port or None)
        print(f"[config] loaded {args.config}")
        print(f"[port] {gripper.port}")
        gripper.connect(mode=None, enable=False, apply_limits=False, refresh=False)

        params = [int(variable) for variable in Variable] if args.all else list(args.params or DEFAULT_PARAMS)
        print(f"[api] gripper.read_param(...) x {len(params)}")

        replied = 0
        for rid in params:
            value = gripper.read_param(rid, timeout_s=args.timeout)
            if value is not None:
                replied += 1
            print(f"[param] {_param_name(rid):<10} rid={rid:>3} value={_format_value(value)}")

        if replied == 0:
            print("[result] no parameter reply")
            return 2
        print("[done]")
        return 0

    except GloriaSdkError as exc:
        print(f"[error] {exc}")
        return 1
    finally:
        if gripper is not None:
            gripper.disconnect(disable=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Read motor parameters")
    parser.add_argument("--config", default=DEFAULT_GRIPPER_CONFIG, help="path to TOML config file")
    parser.add_argument("--port", default="", help="serial port override; defaults to config connection.port")
    parser.add_argument("--param", dest="params", action="append", type=_parse_param, help="parameter name or register ID")
    parser.add_argument("--all", action="store_true", help="read all known motor parameters")
    parser.add_argument("--timeout", type=float, default=0.2, help="parameter read timeout [s]")

    raise SystemExit(main(parser.parse_args()))
