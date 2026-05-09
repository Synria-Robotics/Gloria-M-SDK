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
    python 01_gripper_quicktest.py [--port auto] [--id 0x01] [--close-q -2.7] [--vel 1.0]

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
    ControlMode,
    GloriaGripper,
    Limits,
    PositionRange,
)


def _parse_int(value: str) -> int:
    return int(value, 0)


def _resolve_port(port: str) -> str:
    """Return *port* unchanged, or auto-detect a single available COM port."""
    if port and port.lower() != "auto":
        return port
    from serial.tools import list_ports

    found = list(list_ports.comports())
    if not found:
        raise SystemExit("[port] No serial ports found. Plug in the serial-to-CAN adapter.")
    if len(found) > 1:
        names = ", ".join(p.device for p in found)
        raise SystemExit(f"[port] Multiple serial ports found ({names}); pass --port explicitly.")
    print(f"[port] auto-detected {found[0].device} ({found[0].description})")
    return found[0].device


def main() -> int:
    ap = argparse.ArgumentParser(description="PV mode gripper reciprocating cycle")
    ap.add_argument("--port", default="auto", help="serial port; 'auto' picks the only available one")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--id", type=_parse_int, default="0x01", help="motor command CAN ID")
    ap.add_argument("--fb-id", type=_parse_int, default="0x101", help="motor feedback CAN ID")
    ap.add_argument("--open-q", type=float, default=2.5, help="open position [rad]")
    ap.add_argument("--close-q", type=float, default=0.0, help="close position [rad]")
    ap.add_argument("--vel", type=float, default=1.0, help="movement velocity [rad/s]")
    ap.add_argument("--loop-sleep", type=float, default=0.01, help="control loop sleep time [s]")
    ap.add_argument("--print-hz", type=float, default=5.0, help="print frequency [Hz]")
    ap.add_argument("--settle-threshold", type=float, default=0.1, help="settle threshold [rad]")
    ap.add_argument("--settle-time", type=float, default=0.3, help="dwell time after settling [s]")
    ap.add_argument("--timeout", type=float, default=8.0, help="max time per segment [s]; auto-switch on timeout")
    args = ap.parse_args()
    args.port = _resolve_port(args.port)

    safe_q = PositionRange(min=min(args.open_q, args.close_q), max=max(args.open_q, args.close_q))
    limits = Limits(pmax=3.14, vmax=10.0, tmax=12.0)

    with GloriaGripper(
        args.port,
        baudrate=args.baud,
        command_id=args.id,
        feedback_id=args.fb_id,
        limits=limits,
        safe_position=safe_q,
    ) as g:
        # apply_limits=True (default) writes PMAX/VMAX/TMAX to flash so the
        # motor clamps commands at the firmware level even if user code errs.
        g.motor.set_mode(ControlMode.POS_VEL)
        print("[mode] PV: ok")
        g.motor.enable()
        print("[enable] ok")

        # Fetch the current position so the first segment starts from a
        # known state rather than from zero.
        g.motor.refresh()
        print(f"[init] position = {g.state.position:.3f} rad")

        cycle = 0
        last_print_at = 0.0

        try:
            while True:
                # Alternate between close and open targets each cycle.
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
                    # Send PV command every loop iteration.  The motor
                    # firmware smooths the trajectory internally.
                    g.motion.send_pos_vel(position=target, velocity=args.vel, poll=True)

                    elapsed = time.perf_counter() - seg_start
                    pos_err = abs(g.state.position - target)

                    # Rate-limited console output so the terminal doesn't flood.
                    if (time.time() - last_print_at) >= 1.0 / max(1.0, float(args.print_hz)):
                        last_print_at = time.time()
                        print(
                            f"  t={elapsed:5.2f}s | "
                            f"target={target:+.3f} q_fb={g.state.position:+.3f} | "
                            f"vel={g.state.velocity:+.3f} tau={g.state.torque:+.3f}"
                        )

                    # Settle detection: position must stay within the threshold
                    # for the full settle_time before the segment is considered done.
                    if pos_err < args.settle_threshold:
                        if settled_at is None:
                            settled_at = time.perf_counter()
                        elif (time.perf_counter() - settled_at) >= args.settle_time:
                            print(f"  Settled! (error {pos_err:.3f} rad)")
                            break
                    else:
                        settled_at = None

                    # Timeout guard: switch direction even if target not reached
                    # (e.g. gripper hit a mechanical stop before the target angle).
                    if elapsed > args.timeout:
                        print(f"  Timeout switch (q_fb={g.state.position:+.3f}, error {pos_err:.3f} rad)")
                        break

                    time.sleep(float(args.loop_sleep))

                cycle += 1

        except KeyboardInterrupt:
            print("\n[ctrl+c] exiting")
        finally:
            # Always de-energize the motor on exit, even if an exception occurred.
            g.motor.disable()
            print("[disable] ok")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
