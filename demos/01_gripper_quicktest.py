# Copyright (c) 2026 Synria Robotics Co., Ltd.
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
01 – PV mode gripper reciprocating cycle quick test (Gripper Quick Test)

Description:
    The gripper repeatedly moves between open position (open_q) and close
    position (close_q) to quickly verify that the motor PV (position +
    velocity) control mode is working correctly.

How it works:
    In PV mode, each cycle sends a target position and movement velocity
    command to the motor.  The motor firmware handles trajectory planning
    internally.  Once the feedback position is within the settle threshold
    for the required dwell time, the direction switches automatically.

Usage:
    python 01_gripper_quicktest.py [--port COM5] [--id 0x01] [--close-q -2.7] [--velocity 1.0]

Exit:
    Press Ctrl+C to exit; the motor will be disabled automatically.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_SRC_DIR = os.path.join(_REPO_ROOT, "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from gloria_m_sdk import (
    Actuator,
    CanController,
    ControlMode,
    Limits,
    PositionRange,
    SerialCanAdapter,
    apply_limits_and_save,
)


def _parse_int(value: str) -> int:
    return int(value, 0)


def main() -> int:
    ap = argparse.ArgumentParser(description="PV mode gripper reciprocating cycle")
    ap.add_argument("--port", default="COM5", help="serial port")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--id", type=_parse_int, default="0x01", help="motor command CAN ID")
    ap.add_argument("--fb-id", type=_parse_int, default="0x101", help="motor feedback CAN ID")
    ap.add_argument("--open-q", type=float, default=2.5, help="open position [rad]")
    ap.add_argument("--close-q", type=float, default=0.0, help="close position [rad]")
    ap.add_argument("--velocity", type=float, default=1.0, help="movement velocity [rad/s]")
    ap.add_argument("--loop-sleep", type=float, default=0.01, help="control loop sleep time [s]")
    ap.add_argument("--print-hz", type=float, default=5.0, help="print frequency [Hz]")
    ap.add_argument("--settle-threshold", type=float, default=0.1, help="settle threshold [rad]")
    ap.add_argument("--settle-time", type=float, default=0.3, help="dwell time after settling [s]")
    ap.add_argument("--timeout", type=float, default=8.0, help="max time per segment [s]; auto-switch on timeout")
    args = ap.parse_args()

    safe_q = PositionRange(min=min(args.open_q, args.close_q), max=max(args.open_q, args.close_q))
    limits = Limits(pmax=3.14, vmax=10.0, tmax=12.0)

    act = Actuator(
        name="gripper_cycle",
        command_id=args.id,
        feedback_id=args.fb_id,
        limits=limits,
        safe_position=safe_q,
    )

    with SerialCanAdapter(args.port, baudrate=args.baud, timeout=0.5) as adapter:
        apply_limits_and_save(adapter, motor_id=args.id, limits=limits)

        ctrl = CanController(adapter)
        ctrl.register(act)

        ok = ctrl.set_control_mode(act, ControlMode.POS_VEL)
        print(f"[mode] set PV: {ok}")
        ctrl.enable(act)
        print("[enable] ok")

        ctrl.refresh_state(act)
        print(f"[init] position = {act.state.position:.3f} rad")

        cycle = 0
        last_print_at = 0.0

        try:
            while True:
                if cycle % 2 == 0:
                    target = float(args.close_q)
                    direction = "closing"
                else:
                    target = float(args.open_q)
                    direction = "opening"

                print(f"\n--- Segment {cycle + 1}: {direction} → {target:.2f} rad ---")
                seg_start = time.perf_counter()
                settled_at = None

                while True:
                    ctrl.send_pos_vel(act, position=target, velocity=args.velocity, poll=True)

                    elapsed = time.perf_counter() - seg_start
                    pos_err = abs(act.state.position - target)

                    if (time.time() - last_print_at) >= 1.0 / max(1.0, float(args.print_hz)):
                        last_print_at = time.time()
                        print(
                            f"  t={elapsed:5.2f}s | "
                            f"target={target:+.3f} q_fb={act.state.position:+.3f} | "
                            f"vel={act.state.velocity:+.3f} tau={act.state.torque:+.3f}"
                        )

                    if pos_err < args.settle_threshold:
                        if settled_at is None:
                            settled_at = time.perf_counter()
                        elif (time.perf_counter() - settled_at) >= args.settle_time:
                            print(f"  Settled! (error {pos_err:.3f} rad)")
                            break
                    else:
                        settled_at = None

                    # Timeout guard: switch direction even if target not reached
                    if elapsed > args.timeout:
                        print(f"  Timeout switch (q_fb={act.state.position:+.3f}, error {pos_err:.3f} rad)")
                        break

                    time.sleep(float(args.loop_sleep))

                cycle += 1

        except KeyboardInterrupt:
            print("\n[ctrl+c] exiting")
        finally:
            ctrl.disable(act)
            print("[disable] ok")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
