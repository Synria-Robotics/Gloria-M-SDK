# Copyright (c) 2026 Synria Robotics Co., Ltd.
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
03 – MIT mode linkage gripper force control (MIT Linkage Force Control)

Description:
    Demonstrates fingertip-force-controlled gripping on a linkage-driven
    gripper using MIT torque/impedance control mode.  Because the fingertip
    force and motor torque are not in a simple 1-to-1 relationship, a
    piecewise-linear "equivalent moment-arm vs. motor angle" curve is used
    to estimate the transmission ratio at each position:

        fingertip_force ≈ closing_torque / equivalent_moment_arm(q)

    An outer PI force-control loop adjusts the torque command during the hold
    phase to track the target grip force.

Control phases:
    1. Opening   — Apply a positive torque to push the gripper to its upper limit.
    2. Approaching — Apply a negative torque to close toward the object.
    3. Contact   — Detect via estimated grip force, velocity stall, or close limit.
    4. Hold      — PI force control maintains the target grip force [N].
    5. Release   — Apply positive torque to re-open; repeat for the next cycle.

Calibration note:
    The default moment-arm profile (--radius-profile) is a linear placeholder.
    Replace it with actual values measured from your linkage mechanism:

        python 03_mit_linkage_force_control.py --radius-profile "0.003:10,1.4:14,2.77:18"
        #  angle_rad:radius_mm pairs, at least two points, ascending angle order

Usage:
    python 03_mit_linkage_force_control.py [--port auto] [--id 0x01]
        [--target-force 15] [--hold-seconds 5] [--cycles 1]

Exit:
    Press Ctrl+C to exit; the motor will be disabled automatically.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

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
    TorqueBaseline,
    Variable,
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


def _clamp(value: float, low: float, high: float) -> float:
    if value < low:
        return low
    if value > high:
        return high
    return value


def _is_open_enough(position: float, open_q: float, epsilon: float) -> bool:
    """
    Return True if the gripper is already in the "open enough" region.
    Passes as soon as the position is close to or past open_q, to avoid
    getting stuck in opening/releasing due to zero-point drift or
    mechanical upper-limit error.
    """

    margin = max(0.04, float(epsilon) * 2.0)
    return float(position) >= float(open_q) - margin


@dataclass(frozen=True)
class LinkageForceProfile:
    """
    Piecewise-linear mapping from motor angle q [rad] to equivalent moment arm [m].

    If the linkage calibration is accurate enough, radius_at(q) approximates
    the local transmission ratio at the current position, allowing motor torque
    to be converted to fingertip force more accurately than a fixed moment arm.
    """

    points: Tuple[Tuple[float, float], ...]

    @classmethod
    def from_text(cls, text: str) -> "LinkageForceProfile":
        raw_points: List[Tuple[float, float]] = []
        for item in text.split(","):
            item = item.strip()
            if not item:
                continue
            if ":" not in item:
                raise ValueError(
                    "Moment-arm profile format must be q:radius_mm, e.g. 0.003:10,1.4:14,2.77:18"
                )
            q_text, radius_mm_text = item.split(":", 1)
            q = float(q_text.strip())
            radius_mm = float(radius_mm_text.strip())
            if radius_mm <= 0.0:
                raise ValueError("Radius in moment-arm profile must be > 0 mm")
            raw_points.append((q, radius_mm / 1000.0))

        if len(raw_points) < 2:
            raise ValueError("Moment-arm profile requires at least two points")

        points = tuple(sorted(raw_points, key=lambda item: item[0]))
        return cls(points=points)

    def radius_at(self, q: float) -> float:
        pts = self.points
        if q <= pts[0][0]:
            return pts[0][1]
        if q >= pts[-1][0]:
            return pts[-1][1]

        for idx in range(1, len(pts)):
            q0, r0 = pts[idx - 1]
            q1, r1 = pts[idx]
            if q <= q1:
                if abs(q1 - q0) <= 1e-9:
                    return r1
                ratio = (q - q0) / (q1 - q0)
                return r0 + (r1 - r0) * ratio
        return pts[-1][1]

    def force_from_feedback(self, q: float, feedback_tau: float) -> float:
        closing_torque = max(0.0, -float(feedback_tau))
        return closing_torque / max(self.radius_at(q), 1e-6)

    def closing_tau_from_force(self, q: float, force_n: float) -> float:
        return max(0.0, float(force_n)) * self.radius_at(q)


def _default_radius_profile() -> str:
    return "0.003:10,1.4:14,2.77:18"


def _build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="MIT linkage gripper force control demo with contact detection and hold phase"
    )
    ap.add_argument("--port", default="auto", help="serial port; 'auto' picks the only available one (e.g. COM8 or /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=921600, help="serial baud rate")
    ap.add_argument("--id", type=_parse_int, default="0x01", help="motor command CAN ID")
    ap.add_argument("--fb-id", type=_parse_int, default="0x101", help="motor feedback CAN ID")

    ap.add_argument("--open-q", type=float, default=2.77, help="gripper maximum open position [rad]")
    ap.add_argument(
        "--close-q",
        type=float,
        default=0.003,
        help="gripper closed position [rad]",
    )
    ap.add_argument(
        "--radius-profile",
        default=_default_radius_profile(),
        help="linkage equivalent moment-arm profile, format: q:radius_mm",
    )
    ap.add_argument(
        "--baseline-csv",
        default=".\\demos\\baseline\\close_baseline_4310.csv",
        help="no-load torque baseline CSV",
    )

    ap.add_argument("--target-force", type=float, default=15.0, help="target gripping force [N]")
    ap.add_argument("--open-tau", type=float, default=2.5, help="opening-direction torque, must be positive [Nm]")
    ap.add_argument("--close-tau", type=float, default=-2.5, help="closing-direction torque, must be negative [Nm]")
    ap.add_argument(
        "--contact-force",
        type=float,
        default=10.0,
        help="contact detection force threshold [N]; with baseline loaded this is the incremental gripping force threshold",
    )
    ap.add_argument(
        "--abort-force",
        type=float,
        default=500.0,
        help="force protection threshold [N]; with baseline loaded this is the incremental gripping force limit",
    )
    ap.add_argument(
        "--max-hold-tau",
        type=float,
        default=25,
        help="maximum closing torque allowed during the hold phase [Nm]",
    )

    ap.add_argument("--approach-kd", type=float, default=0.8, help="MIT torque control damping kd for the approach phase")
    ap.add_argument("--hold-kd", type=float, default=0.5, help="MIT torque control damping kd for the hold phase")
    ap.add_argument("--release-kd", type=float, default=0.8, help="MIT torque control damping kd for the release phase")

    ap.add_argument("--force-kp", type=float, default=0.6, help="outer force-control loop kp")
    ap.add_argument("--force-ki", type=float, default=0.4, help="outer force-control loop ki")
    ap.add_argument(
        "--integral-limit",
        type=float,
        default=20.0,
        help="force-control integral clamp [N·s]",
    )
    ap.add_argument("--hold-seconds", type=float, default=5.0, help="grip hold duration [s]")
    ap.add_argument("--cycles", type=int, default=1, help="number of cycles to run; 0 = run indefinitely")

    ap.add_argument(
        "--contact-vel-threshold",
        type=float,
        default=0.08,
        help="stall detection velocity threshold [rad/s]",
    )
    ap.add_argument(
        "--contact-min-travel",
        type=float,
        default=0.05,
        help="minimum travel required before stall contact detection is triggered [rad]",
    )
    ap.add_argument(
        "--contact-error-threshold",
        type=float,
        default=0.05,
        help="reserved; unused in current pure-torque mode [rad]",
    )
    ap.add_argument(
        "--position-epsilon",
        type=float,
        default=0.02,
        help="position tolerance for phase transitions [rad]",
    )
    ap.add_argument("--loop-sleep", type=float, default=0.002, help="control loop sleep time [s]")
    ap.add_argument("--print-hz", type=float, default=10.0, help="status print frequency [Hz]")
    ap.add_argument(
        "--start-phase",
        choices=("auto", "opening", "approaching"),
        default="approaching",
        help="starting phase: auto = detect automatically, approaching = start from approach phase directly",
    )
    ap.add_argument("--hold-q", type=float, default=None, help="target position locked during the hold phase [rad]; defaults to close-q when position-only is set")
    ap.add_argument("--hold-kp", type=float, default=25.0, help="position hold stiffness kp; only active when --hold-q is set")
    ap.add_argument("--position-only", action="store_true", help="move to the specified close position and hold; defaults to close-q when --hold-q is not set")
    return ap


def main() -> int:
    args = _build_arg_parser().parse_args()
    args.port = _resolve_port(args.port)

    # Backwards compatibility for old default values.
    # Tests showed that -1.25 Nm is sufficient for stable closing;
    # if the user hasn't explicitly overridden the old defaults, silently upgrade to a more practical value.
    if abs(float(args.open_tau) - 0.25) <= 1e-9 or abs(float(args.open_tau) - 1.25) <= 1e-9:
        args.open_tau = 1.60
    if abs(float(args.close_tau) - (-0.25)) <= 1e-9 or abs(float(args.close_tau) - (-1.25)) <= 1e-9:
        args.close_tau = -1.60

    raw_open_q = float(args.open_q)
    raw_close_q = float(args.close_q)
    safe_q = PositionRange(min=min(raw_open_q, raw_close_q), max=max(raw_open_q, raw_close_q))
    limits = Limits(pmax=3.14, vmax=10.0, tmax=12.0)
    linkage = LinkageForceProfile.from_text(args.radius_profile)
    # Load the no-load torque baseline so we can subtract free-running friction
    # from the feedback torque and get a better estimate of actual grip force.
    baseline = TorqueBaseline.from_csv(args.baseline_csv) if args.baseline_csv else None
    if baseline is not None:
        print(f"[baseline] loaded: {args.baseline_csv} ({len(baseline.points)} points)")

    act = Actuator(
        name="linkage_gripper",
        command_id=args.id,
        feedback_id=args.fb_id,
        limits=limits,
        safe_position=safe_q,
    )

    open_q = act.clamp_position(raw_open_q)
    close_q = act.clamp_position(raw_close_q)
    hold_q = None if args.hold_q is None else act.clamp_position(float(args.hold_q))
    position_only = bool(args.position_only)
    hold_q_cmd = close_q if (position_only and hold_q is None) else hold_q
    open_tau = float(args.open_tau)
    close_tau = float(args.close_tau)
    if close_q > open_q:
        raise ValueError("This demo assumes decreasing angle = closing direction; ensure close-q <= open-q")
    if open_tau <= 0.0:
        raise ValueError("open-tau must be positive (positive torque = opening direction)")
    if close_tau >= 0.0:
        raise ValueError("close-tau must be negative (negative torque = closing direction)")

    if position_only:
        # In position-only mode the demo moves to close-q (or hold-q) and
        # holds there without force control.  Disable all force-related
        # thresholds so they don't interfere.
        args.target_force = 0.0
        args.max_hold_tau = 0.0
        args.hold_seconds = float("inf")
        args.abort_force = float("inf")

    with SerialCanAdapter(args.port, baudrate=args.baud, timeout=0.5) as adapter:
        ctrl = CanController(adapter)
        ctrl.register(act)

        # Write PMAX/VMAX/TMAX limits and persist to flash.
        # This guarantees the motor's firmware-level clamping is active even
        # if the outer force-control loop outputs a large tau command.
        ctrl.apply_limits_and_save(act, limits)

        # Switch to MIT mode; the motor must confirm this before we enable.
        ok = ctrl.set_control_mode(act, ControlMode.MIT)
        print(f"[mode] set MIT: {ok}")
        ctrl.enable(act)
        print("[enable] ok")

        ctrl.refresh_state(act)
        print(
            f"[init] q={act.state.position:+.3f} rad | open_q={open_q:+.3f} | "
            f"close_q={close_q:+.3f} | hold_q={hold_q_cmd if hold_q_cmd is not None else 'auto'} | "
            f"open_tau={open_tau:+.3f} | close_tau={close_tau:+.3f}"
        )

        force_integral = 0.0
        cycles_done = 0

        # Determine the starting phase: if the gripper is already open enough
        # we can skip the opening phase and go straight to approaching.
        initial_q = float(act.state.position)
        if args.start_phase == "opening":
            phase = "opening"
        elif args.start_phase == "approaching":
            phase = "approaching"
        else:  # "auto"
            phase = "approaching" if _is_open_enough(initial_q, open_q, args.position_epsilon) else "opening"
        phase_started_at = time.perf_counter()
        phase_started_q = initial_q
        print(f"[phase] start in {phase} | q={initial_q:+.3f} | start_phase={args.start_phase}")
        last_loop_mono: Optional[float] = None
        last_print_at = 0.0

        try:
            while True:
                now = time.perf_counter()
                dt = 0.0 if last_loop_mono is None else max(1e-4, now - last_loop_mono)
                last_loop_mono = now

                # ---- Per-loop state estimates ----
                # Raw force estimate: closing torque / linkage moment arm
                estimated_force = linkage.force_from_feedback(act.state.position, act.state.torque)
                radius_mm = linkage.radius_at(act.state.position) * 1000.0
                # Baseline-corrected estimates: subtract the no-load friction torque
                # so the "effective force" represents actual grip force only.
                baseline_tau = baseline.tau_at(act.state.position) if baseline is not None else 0.0
                delta_tau = (
                    baseline.closing_delta_tau(act.state.position, act.state.torque)
                    if baseline is not None
                    else max(0.0, -float(act.state.torque))
                )
                effective_force = delta_tau / max(linkage.radius_at(act.state.position), 1e-6)

                tau_cmd = 0.0
                kp = 0.0
                kd = args.approach_kd
                q_target = act.state.position
                phase_age = now - phase_started_at

                if phase == "opening":
                    tau_cmd = open_tau
                    q_target = act.state.position
                    kd = args.release_kd

                    reached_open = _is_open_enough(act.state.position, open_q, args.position_epsilon)
                    opening_stuck = (
                        phase_age >= 0.6
                        and abs(float(act.state.position) - float(phase_started_q))
                        <= max(0.02, float(args.position_epsilon))
                    )
                    if (reached_open and phase_age >= 0.05) or opening_stuck:
                        if args.cycles > 0 and cycles_done >= args.cycles:
                            break
                        phase = "approaching"
                        phase_started_at = now
                        phase_started_q = float(act.state.position)
                        force_integral = 0.0
                        if opening_stuck and not reached_open:
                            print(
                                f"[warn] opening phase displacement too small, switching to approaching directly | "
                                f"q={act.state.position:+.3f}"
                            )
                        print(f"[phase] approaching, grip #{cycles_done + 1}")

                elif phase == "approaching":
                    if position_only:
                        tau_cmd = 0.0
                        q_target = float(hold_q_cmd if hold_q_cmd is not None else close_q)
                        kp = float(args.hold_kp)
                        kd = args.approach_kd
                    else:
                        tau_cmd = close_tau
                        q_target = act.state.position
                        kd = args.approach_kd

                    moved_distance = max(0.0, float(phase_started_q) - float(act.state.position))
                    stalled = (
                        moved_distance >= args.contact_min_travel
                        and
                        abs(act.state.velocity) <= args.contact_vel_threshold
                        and phase_age >= 0.15
                    )
                    reached_close_limit = act.state.position <= close_q + args.position_epsilon
                    reached_hold_q = (
                        hold_q_cmd is not None
                        and abs(float(act.state.position) - float(hold_q_cmd)) <= float(args.position_epsilon)
                    )
                    touched_object = (
                        (not position_only)
                        and moved_distance >= args.contact_min_travel
                        and effective_force >= args.contact_force
                    )

                    if reached_hold_q or touched_object or stalled or reached_close_limit:
                        if position_only and not reached_hold_q:
                            hold_q_cmd = float(act.state.position)
                        elif not position_only:
                            if reached_close_limit and not touched_object and not stalled:
                                hold_q_cmd = float(close_q if hold_q is None else hold_q)
                            else:
                                hold_q_cmd = hold_q
                        force_integral = 0.0
                        phase = "holding"
                        phase_started_at = now
                        phase_started_q = float(act.state.position)

                        if reached_hold_q:
                            reason = "hold_q"
                        elif touched_object:
                            reason = "force"
                        elif stalled:
                            reason = "stall"
                        else:
                            reason = "close_limit"
                        print(
                            f"[phase] holding, reason={reason}, current position={act.state.position:+.3f} "
                            f"effective force={effective_force:.2f}N"
                        )

                elif phase == "holding":
                    if position_only:
                        # Position-only hold: stiff spring-damper at hold_q, no torque feedforward.
                        if hold_q_cmd is None:
                            hold_q_cmd = float(close_q)
                        if effective_force >= float(args.abort_force):
                            hold_q_cmd = float(act.state.position)
                            print(f"[protect] force too high, clamp hold_q to current q={hold_q_cmd:+.3f}")
                        q_target = float(hold_q_cmd)
                        kp = float(args.hold_kp)
                        kd = args.hold_kd
                        tau_cmd = 0.0
                    else:
                        # Force-control hold: outer PI loop adjusts closing torque
                        # to maintain target_force at the fingertip.
                        q_target = float(act.state.position) if hold_q_cmd is None else float(hold_q_cmd)
                        kp = 0.0 if hold_q_cmd is None else float(args.hold_kp)
                        kd = args.hold_kd

                    # PI force controller: error = target - measured effective force
                    force_error = float(args.target_force) - effective_force
                    force_integral = _clamp(
                        force_integral + force_error * dt,
                        -float(args.integral_limit),
                        float(args.integral_limit),
                    )
                    commanded_force = max(
                        0.0,
                        float(args.target_force)
                        + float(args.force_kp) * force_error
                        + float(args.force_ki) * force_integral,
                    )
                    # Convert commanded force to closing torque via the linkage model,
                    # then cap at max_hold_tau to protect the gripper.
                    tau_cmd = -min(
                        float(args.max_hold_tau),
                        linkage.closing_tau_from_force(act.state.position, commanded_force),
                    )

                    # Over-force protection: if effective force exceeds the abort
                    # threshold, release immediately regardless of hold_seconds.
                    if effective_force >= float(args.abort_force):
                        phase = "releasing"
                        phase_started_at = now
                        phase_started_q = float(act.state.position)
                        print(f"[protect] over-force release triggered, effective force={effective_force:.2f}N")
                    elif phase_age >= float(args.hold_seconds):
                        # Hold duration expired — move to release phase.
                        phase = "releasing"
                        phase_started_at = now
                        phase_started_q = float(act.state.position)
                        print(f"[phase] releasing, grip #{cycles_done + 1} complete")

                elif phase == "releasing":
                    tau_cmd = open_tau
                    q_target = act.state.position
                    kd = args.release_kd

                    reached_open = _is_open_enough(act.state.position, open_q, args.position_epsilon)
                    if reached_open and phase_age >= 0.05:
                        cycles_done += 1
                        phase = "opening"
                        phase_started_at = now
                        phase_started_q = float(act.state.position)
                        print(f"[cycle] {cycles_done} complete")

                else:
                    raise RuntimeError(f"unknown phase: {phase}")

                ctrl.send_mit(
                    act,
                    kp=float(kp),
                    kd=float(kd),
                    q=float(q_target),
                    dq=0.0,
                    tau=float(tau_cmd),
                    poll=True,
                )

                print_period = 1.0 / max(1.0, float(args.print_hz))
                wall_now = time.time()
                if wall_now - last_print_at >= print_period:
                    last_print_at = wall_now
                    p_m = ctrl.read_param(act, int(Variable.p_m))
                    xout = ctrl.read_param(act, int(Variable.xout))
                    p_m_text = "nan" if p_m is None else f"{p_m:+.3f}"
                    xout_text = "nan" if xout is None else f"{xout:+.3f}"
                    print(
                        f"phase={phase:10s} cycle={cycles_done:02d} | "
                        f"q={act.state.position:+.3f} q_ref={q_target:+.3f} dq={act.state.velocity:+.3f} | "
                        f"p_m={p_m_text} xout={xout_text} | "
                        f"tau_cmd={tau_cmd:+.3f} tau_fb={act.state.torque:+.3f} | "
                        f"force_est={estimated_force:6.2f}N eff_force={effective_force:6.2f}N | "
                        f"tau_free={baseline_tau:+.3f} delta_tau={delta_tau:+.3f} | "
                        f"radius={radius_mm:5.1f}mm"
                    )

                time.sleep(float(args.loop_sleep))

        except KeyboardInterrupt:
            print("\n[ctrl+c] exit requested")
        finally:
            # Always de-energize the motor before closing the serial port.
            ctrl.disable(act)
            print("[disable] ok")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
