# Copyright (c) 2026 Synria Robotics Co., Ltd.
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
02 – PV mode gripper gentle close (PV Gentle Close)

Description:
    First opens the gripper to the open position, then slowly closes to the
    target position using PV (position + velocity) mode.  The low closing
    speed produces a smooth, gentle motion suitable for picking up delicate
    objects.

How it works:
    In PV mode, each control cycle sends a target position and a speed limit
    to the motor.  The motor firmware handles trajectory planning and speed
    limiting internally.  The opening phase uses a higher speed (open_vel);
    the closing phase uses a lower speed (close_vel) for gentle approach.
    After reaching the target the gripper holds for the configured hold time
    before exiting.

Usage:
    python 02_pv_control.py [--port COM5] [--id 0x01] [--close-q -2.0] [--close-vel 0.3]

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
)


def _parse_int(value: str) -> int:
    return int(value, 0)


def main() -> int:
    ap = argparse.ArgumentParser(description="PV mode gripper gentle close")
    ap.add_argument("--port", default="COM5", help="serial port")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--id", type=_parse_int, default="0x01", help="motor command CAN ID")
    ap.add_argument("--fb-id", type=_parse_int, default="0x101", help="motor feedback CAN ID")
    ap.add_argument("--open-q", type=float, default=2.5, help="open position [rad]")
    ap.add_argument("--close-q", type=float, default=0.0, help="close position [rad]")
    ap.add_argument("--open-vel", type=float, default=1.0, help="opening velocity [rad/s]")
    ap.add_argument("--close-vel", type=float, default=0.3, help="closing velocity [rad/s]; lower = gentler")
    ap.add_argument("--loop-sleep", type=float, default=0.01, help="control loop sleep time [s]")
    ap.add_argument("--print-hz", type=float, default=5.0, help="print frequency [Hz]")
    ap.add_argument("--settle-threshold", type=float, default=0.15, help="settle threshold [rad]")
    ap.add_argument("--settle-time", type=float, default=0.5, help="dwell time after settling [s]")
    ap.add_argument("--timeout", type=float, default=20.0, help="max time per segment [s]")
    ap.add_argument("--hold-time", type=float, default=2.0, help="grip hold duration after closing [s]")
    args = ap.parse_args()

    safe_q = PositionRange(min=min(args.open_q, args.close_q), max=max(args.open_q, args.close_q))
    limits = Limits(pmax=3.14, vmax=10.0, tmax=12.0)

    act = Actuator(
        name="gripper_pv_gentle",
        command_id=args.id,
        feedback_id=args.fb_id,
        limits=limits,
        safe_position=safe_q,
    )

    with SerialCanAdapter(args.port, baudrate=args.baud, timeout=0.5) as adapter:
        ctrl = CanController(adapter)
        ctrl.register(act)

        # Write PMAX/VMAX/TMAX limits and save to flash.
        # The motor will clamp all position and velocity commands to these
        # ranges at the firmware level, providing a hardware safety net.
        ctrl.apply_limits_and_save(act, limits)

        # Switch to PV mode.  A False return means the motor didn't respond —
        # the motor may still switch correctly but we warn the user.
        ok = ctrl.set_control_mode(act, ControlMode.POS_VEL)
        print(f"[mode] set PV: {ok}")
        if not ok:
            print("[warn] mode switch not confirmed, motor may not have responded, retrying...")
        ctrl.enable(act)
        print("[enable] ok")

        # Read the initial position before issuing any move commands.
        ctrl.refresh_state(act)
        print(f"[init] position = {act.state.position:.3f} rad")

        try:
            # ---- Phase 1: open to open_q ----
            open_q = act.clamp_position(args.open_q)
            print(f"\n=== Phase 1: open to {open_q:.2f} rad ===")
            _move_to(ctrl, act, target=open_q, velocity=args.open_vel,
                     loop_sleep=args.loop_sleep, print_hz=args.print_hz,
                     settle_threshold=args.settle_threshold, settle_time=args.settle_time,
                     timeout=args.timeout)
            print(f"\n  Open complete, current position = {act.state.position:.3f} rad")
            time.sleep(0.5)

            # ---- Phase 2: gentle close to close_q ----
            close_q = act.clamp_position(args.close_q)
            print(f"\n=== Phase 2: gentle close to {close_q:.2f} rad ===")
            _move_to(ctrl, act, target=close_q, velocity=args.close_vel,
                     loop_sleep=args.loop_sleep, print_hz=args.print_hz,
                     settle_threshold=args.settle_threshold, settle_time=args.settle_time,
                     timeout=args.timeout)
            print(f"\n  Close complete, current position = {act.state.position:.3f} rad")

            # Hold position: keep sending the closed position with velocity=0
            # so the motor holds stiffly rather than going limp.
            print(f"  Holding grip for {args.hold_time:.1f} s ...")
            hold_end = time.perf_counter() + args.hold_time
            while time.perf_counter() < hold_end:
                ctrl.send_pos_vel(act, position=close_q, velocity=0.0, poll=True)
                time.sleep(args.loop_sleep)

            print("  Done!")

        except KeyboardInterrupt:
            print("\n[ctrl+c] exiting")
        finally:
            # Always de-energize the motor on exit.
            ctrl.disable(act)
            print("[disable] ok")

    return 0


def _move_to(
    ctrl: CanController,
    act: Actuator,
    *,
    target: float,
    velocity: float,
    loop_sleep: float,
    print_hz: float,
    settle_threshold: float,
    settle_time: float,
    timeout: float,
) -> None:
    """Block until the motor reaches *target* within *settle_threshold* [rad].

    Each iteration sends a PV command with the given *target* and *velocity*.
    The motor's firmware handles internal trajectory planning; this function
    only monitors the feedback position and decides when to stop.

    Settle logic
    ------------
    The motor is considered "settled" when the position error stays below
    *settle_threshold* for at least *settle_time* seconds in a row.  A single
    overshoot resets the settle timer.

    Timeout
    -------
    If the position does not settle within *timeout* seconds the function
    returns early.  This handles cases where a mechanical stop prevents the
    motor from reaching the commanded angle (e.g. object grasped).

    Parameters
    ----------
    ctrl:             Active CanController.
    act:              Actuator whose state is updated by each PV send.
    target:           Target position [rad].
    velocity:         Speed limit [rad/s]; lower = slower and gentler.
    loop_sleep:       Sleep between iterations [s] (e.g. 0.01 s ≈ 100 Hz).
    print_hz:         Console status print rate [Hz].
    settle_threshold: Position error below which the motor is "at target" [rad].
    settle_time:      How long the error must stay below threshold [s].
    timeout:          Maximum segment duration before early return [s].
    """
    seg_start = time.perf_counter()
    settled_at: float | None = None
    last_print_at = 0.0
    print_interval = 1.0 / max(1.0, print_hz)

    while True:
        ctrl.send_pos_vel(act, position=target, velocity=velocity, poll=True)

        now = time.perf_counter()
        elapsed = now - seg_start
        pos_err = abs(act.state.position - target)

        if (now - last_print_at) >= print_interval:
            last_print_at = now
            print(
                f"  t={elapsed:5.2f}s | "
                f"target={target:+.3f} q_fb={act.state.position:+.3f} | "
                f"vel={act.state.velocity:+.3f} tau={act.state.torque:+.3f}"
            )

        if pos_err < settle_threshold:
            if settled_at is None:
                settled_at = now
            elif (now - settled_at) >= settle_time:
                print(f"  Settled! (error {pos_err:.3f} rad)")
                break
        else:
            settled_at = None

        if elapsed > timeout:
            print(f"  Timeout (q_fb={act.state.position:+.3f}, error {pos_err:.3f} rad)")
            break

        time.sleep(loop_sleep)


if __name__ == "__main__":
    raise SystemExit(main())
