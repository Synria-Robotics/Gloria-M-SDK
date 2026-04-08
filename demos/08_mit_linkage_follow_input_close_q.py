from __future__ import annotations

"""
MIT linkage gripper controller that follows another motor position.

This script keeps the core close/hold logic from `06_mit_linkage_force_control`,
but removes the demo-style timed auto-open cycle.

Motor `--input-id/--input-fb-id` is treated as the dynamic source of `close_q`.
Its position is read continuously, mapped, and then used as the gripper target.
"""

import argparse
import os
import struct
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_SRC_DIR = os.path.join(_REPO_ROOT, "src")
_DEFAULT_BASELINE_CSV = os.path.join(
    _REPO_ROOT,
    "demos",
    "output",
    "close_baseline_20260407_114136_binned.csv",
)
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
    apply_limits_and_save,
)


def _parse_int(value: str) -> int:
    return int(value, 0)


def _clamp(value: float, low: float, high: float) -> float:
    if value < low:
        return low
    if value > high:
        return high
    return value


def _read_float_param_once(
    adapter: SerialCanAdapter,
    *,
    target_id: int,
    rid: int,
    timeout_s: float = 0.03,
) -> Optional[float]:
    can_id_l = int(target_id) & 0xFF
    can_id_h = (int(target_id) >> 8) & 0xFF
    adapter.send(0x7FF, bytes([can_id_l, can_id_h, 0x33, int(rid) & 0xFF, 0, 0, 0, 0]))

    deadline = time.time() + float(timeout_s)
    expect_ids = {int(target_id), int(0x100 + int(target_id)), 0x00}
    while time.time() < deadline:
        for pkt in adapter.read_packets():
            if len(pkt.data) != 8:
                continue
            if int(pkt.can_id) not in expect_ids and not (
                pkt.data[0] == can_id_l and pkt.data[1] == can_id_h
            ):
                continue
            if pkt.data[2] not in (0x33, 0x55):
                continue
            if int(pkt.data[3]) != (int(rid) & 0xFF):
                continue
            return float(struct.unpack("<f", pkt.data[4:8])[0])
        time.sleep(0.002)
    return None


def _read_input_position(
    adapter: SerialCanAdapter,
    ctrl: CanController,
    input_motor: Actuator,
    *,
    input_id: int,
) -> Tuple[Optional[float], str]:
    ctrl.refresh_state(input_motor)

    p_m = _read_float_param_once(adapter, target_id=input_id, rid=int(Variable.p_m), timeout_s=0.02)
    if p_m is not None:
        return float(p_m), "p_m"

    xout = _read_float_param_once(adapter, target_id=input_id, rid=int(Variable.xout), timeout_s=0.02)
    if xout is not None:
        return float(xout), "xout"

    if float(input_motor.state.updated_at) > 0.0:
        return float(input_motor.state.position), "state"

    return None, "none"


@dataclass(frozen=True)
class LinkageForceProfile:
    points: Tuple[Tuple[float, float], ...]

    @classmethod
    def from_text(cls, text: str) -> "LinkageForceProfile":
        raw_points: List[Tuple[float, float]] = []
        for item in text.split(","):
            item = item.strip()
            if not item:
                continue
            if ":" not in item:
                raise ValueError("radius profile must be q:radius_mm, for example 0.003:10,1.4:14,2.77:18")
            q_text, radius_mm_text = item.split(":", 1)
            q = float(q_text.strip())
            radius_mm = float(radius_mm_text.strip())
            if radius_mm <= 0.0:
                raise ValueError("radius profile values must be > 0 mm")
            raw_points.append((q, radius_mm / 1000.0))

        if len(raw_points) < 2:
            raise ValueError("radius profile needs at least two points")

        return cls(points=tuple(sorted(raw_points, key=lambda item: item[0])))

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
                span = q1 - q0
                if abs(span) <= 1e-9:
                    return r1
                ratio = (q - q0) / span
                return r0 + (r1 - r0) * ratio
        return pts[-1][1]

    def force_from_feedback(self, q: float, feedback_tau: float) -> float:
        closing_torque = max(0.0, -float(feedback_tau))
        return closing_torque / max(self.radius_at(q), 1e-6)

    def closing_tau_from_force(self, q: float, force_n: float) -> float:
        return max(0.0, float(force_n)) * self.radius_at(q)


def _build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="MIT 连杆夹爪跟随脚本：读取另一台电机的位置，作为动态 close_q 输入",
    )
    ap.add_argument("--port", default="COM12", help="串口号，例如 COM12")
    ap.add_argument("--baud", type=int, default=921600, help="串口波特率")

    ap.add_argument("--id", type=_parse_int, default="0x07", help="夹爪电机命令 CAN ID")
    ap.add_argument("--fb-id", type=_parse_int, default="0x207", help="夹爪电机反馈 CAN ID")
    ap.add_argument("--input-id", type=_parse_int, default="0x01", help="输入电机命令 CAN ID")
    ap.add_argument("--input-fb-id", type=_parse_int, default="0x201", help="输入电机反馈 CAN ID")

    ap.add_argument("--open-q", type=float, default=2.77, help="夹爪最大张开位置 [rad]")
    ap.add_argument("--close-q-min", type=float, default=0.02, help="夹爪允许的最小闭合位置 [rad]")
    ap.add_argument("--input-scale", type=float, default=1.0, help="输入电机位置到 close_q 的比例系数")
    ap.add_argument("--input-offset", type=float, default=0.0, help="输入电机位置到 close_q 的偏移量 [rad]")
    ap.add_argument("--input-poll-s", type=float, default=0.01, help="输入电机刷新周期 [s]")
    ap.add_argument("--input-timeout-s", type=float, default=0.5, help="输入电机超时保护 [s]")

    ap.add_argument("--radius-profile", default="0.02:10,1.4:14,2.77:18", help="连杆等效力臂曲线 q:radius_mm")
    ap.add_argument("--baseline-csv", default=_DEFAULT_BASELINE_CSV, help="空载扭矩基线 CSV")
    ap.add_argument("--target-force", type=float, default=10.0, help="接触后目标夹持力 [N]")
    ap.add_argument("--contact-force", type=float, default=28.0, help="接触判定阈值 [N]")
    ap.add_argument("--open-tau", type=float, default=1.25, help="张开方向扭矩，必须为正 [Nm]")
    ap.add_argument("--close-tau", type=float, default=-9.5, help="闭合方向扭矩，必须为负 [Nm]")
    ap.add_argument("--max-hold-tau", type=float, default=20.0, help="保持阶段允许的最大闭合扭矩 [Nm]")
    ap.add_argument("--approach-kd", type=float, default=2, help="闭合阶段阻尼 kd")
    ap.add_argument("--close-near-window", type=float, default=0.25, help="接近目标位置时开始减小闭合扭矩的距离窗口 [rad]")
    ap.add_argument("--close-near-scale", type=float, default=0.35, help="接近目标位置时的最小闭合扭矩比例")
    ap.add_argument("--hold-kd", type=float, default=0.5, help="保持阶段阻尼 kd")
    ap.add_argument("--release-kd", type=float, default=0.8, help="张开阶段阻尼 kd")
    ap.add_argument("--hold-kp", type=float, default=25.0, help="位置保持刚度 kp")
    ap.add_argument("--force-kp", type=float, default=0.6, help="外环力控 kp")
    ap.add_argument("--force-ki", type=float, default=0.4, help="外环力控 ki")
    ap.add_argument("--integral-limit", type=float, default=20.0, help="力控积分限幅 [N*s]")
    ap.add_argument("--contact-confirm-seconds", type=float, default=0.02, help="接触判定确认时间，避免抖动误判 [s]")
    ap.add_argument("--contact-release-force", type=float, default=0.02, help="接触释放阈值 [N]，<=0 时自动取 contact-force 的较低值")
    ap.add_argument("--contact-release-seconds", type=float, default=0.08, help="接触丢失确认时间，避免瞬时掉力误判 [s]")
    ap.add_argument("--force-filter-tau", type=float, default=0.03, help="用于接触判定的力估计低通时间常数 [s]")
    ap.add_argument("--no-contact-hold-seconds", type=float, default=0, help="保持阶段无接触力时的超时退出时间，<=0 表示关闭 [s]")
    ap.add_argument("--contact-vel-threshold", type=float, default=0.08, help="堵转判定速度阈值 [rad/s]")
    ap.add_argument("--contact-min-travel", type=float, default=0.05, help="允许触发接触判定前的最小运动量 [rad]")
    ap.add_argument("--position-epsilon", type=float, default=0.02, help="目标位置容差 [rad]")
    ap.add_argument("--loop-sleep", type=float, default=0.002, help="控制循环休眠时间 [s]")
    ap.add_argument("--print-hz", type=float, default=10.0, help="状态打印频率 [Hz]")
    return ap


def _map_input_to_close_q(raw_q: float, *, scale: float, offset: float, low: float, high: float) -> float:
    return _clamp(float(raw_q) * float(scale) + float(offset), float(low), float(high))


def main() -> int:
    args = _build_arg_parser().parse_args()

    open_q_raw = float(args.open_q)
    close_q_min_raw = float(args.close_q_min)
    if close_q_min_raw > open_q_raw:
        raise ValueError("close-q-min must be <= open-q")
    if float(args.open_tau) <= 0.0:
        raise ValueError("open-tau must be positive")
    if float(args.close_tau) >= 0.0:
        raise ValueError("close-tau must be negative")

    contact_force_enter = float(args.contact_force)
    if contact_force_enter <= 0.0:
        raise ValueError("contact-force must be > 0")
    abort_force = max(float(args.target_force), contact_force_enter) + 5.0

    contact_force_release = float(args.contact_release_force)
    if contact_force_release <= 0.0:
        contact_force_release = min(contact_force_enter * 0.6, max(0.1, contact_force_enter - 1.0))
    contact_force_release = _clamp(contact_force_release, 0.05, contact_force_enter)

    if float(args.target_force) < contact_force_enter:
        print(
            f"[warn] target-force({float(args.target_force):.2f}N) < contact-force({contact_force_enter:.2f}N), "
            "接触判定可能会晚于目标夹持力"
        )

    limits = Limits(pmax=3.14, vmax=10.0, tmax=12.0)
    gripper_safe = PositionRange(min=close_q_min_raw, max=open_q_raw)
    linkage = LinkageForceProfile.from_text(args.radius_profile)
    baseline = TorqueBaseline.from_csv(args.baseline_csv) if args.baseline_csv else None

    gripper = Actuator(
        name="linkage_gripper",
        command_id=int(args.id),
        feedback_id=int(args.fb_id),
        limits=limits,
        safe_position=gripper_safe,
    )
    input_motor = Actuator(
        name="close_q_input",
        command_id=int(args.input_id),
        feedback_id=int(args.input_fb_id),
        limits=limits,
        safe_position=None,
    )

    with SerialCanAdapter(args.port, baudrate=args.baud, timeout=0.5) as adapter:
        apply_limits_and_save(adapter, motor_id=int(args.id), limits=limits)

        ctrl = CanController(adapter)
        ctrl.register(gripper)
        ctrl.register(input_motor)

        ok = ctrl.set_control_mode(gripper, ControlMode.MIT)
        print(f"[mode] gripper set MIT: {ok}")
        ctrl.enable(gripper)
        print("[enable] gripper ok")

        input_ok = ctrl.set_control_mode(input_motor, ControlMode.MIT)
        print(f"[mode] input motor set MIT: {input_ok}")
        ctrl.enable(input_motor)
        print("[enable] input motor ok")

        ctrl.refresh_state(gripper)
        ctrl.refresh_state(input_motor)
        ctrl.send_mit(
            input_motor,
            kp=0.0,
            kd=0.0,
            q=float(input_motor.state.position),
            dq=0.0,
            tau=0.0,
            poll=False,
        )
        print("[input] sent neutral MIT command")

        initial_input_raw_q, initial_input_source = _read_input_position(
            adapter,
            ctrl,
            input_motor,
            input_id=int(args.input_id),
        )
        if initial_input_raw_q is None:
            initial_input_raw_q = float(input_motor.state.position)
            initial_input_source = "fallback"

        follow_close_q = _map_input_to_close_q(
            initial_input_raw_q,
            scale=args.input_scale,
            offset=args.input_offset,
            low=close_q_min_raw,
            high=open_q_raw,
        )

        if baseline is not None:
            print(f"[baseline] loaded: {args.baseline_csv} ({len(baseline.points)} points)")
        print(
            f"[contact] enter>={contact_force_enter:.2f}N | release<={contact_force_release:.2f}N "
            f"for {float(args.contact_release_seconds):.2f}s | filter_tau={float(args.force_filter_tau):.3f}s"
        )
        print(f"[protect] abort_force={abort_force:.2f}N")

        phase = "holding"
        if gripper.state.position > follow_close_q + float(args.position_epsilon):
            phase = "approaching"
        elif gripper.state.position < follow_close_q - float(args.position_epsilon):
            phase = "opening"

        hold_target_q = follow_close_q if phase == "holding" else None
        contact_latched = False
        contact_candidate_started_at: Optional[float] = None
        contact_release_started_at: Optional[float] = None
        no_contact_started_at: Optional[float] = time.perf_counter() if phase == "holding" else None
        phase_started_at = time.perf_counter()
        phase_started_q = float(gripper.state.position)
        force_integral = 0.0
        filtered_effective_force: Optional[float] = None
        last_loop_mono: Optional[float] = None
        last_print_at = 0.0
        last_input_poll = 0.0
        last_input_q_at = time.time()
        last_input_source = initial_input_source
        last_raw_input_q = float(initial_input_raw_q)
        last_follow_close_q = follow_close_q
        input_lost_warned = False

        print(
            f"[init] gripper_q={gripper.state.position:+.3f} | input_q={initial_input_raw_q:+.3f} "
            f"({initial_input_source}) | close_q_in={follow_close_q:+.3f} | phase={phase}"
        )

        try:
            while True:
                now = time.perf_counter()
                dt = 0.0 if last_loop_mono is None else max(1e-4, now - last_loop_mono)
                last_loop_mono = now

                if (now - last_input_poll) >= float(args.input_poll_s):
                    input_q_now, input_source = _read_input_position(
                        adapter,
                        ctrl,
                        input_motor,
                        input_id=int(args.input_id),
                    )
                    last_input_poll = now
                else:
                    input_q_now = None
                    input_source = "cached"

                if input_q_now is not None:
                    raw_input_q = float(input_q_now)
                    last_input_q_at = time.time()
                    last_input_source = input_source
                    last_raw_input_q = raw_input_q
                else:
                    raw_input_q = float(last_raw_input_q)
                    if input_source != "cached":
                        last_input_source = input_source

                input_age = time.time() - float(last_input_q_at)
                input_valid = bool(last_input_q_at) and input_age <= float(args.input_timeout_s)

                if input_valid:
                    follow_close_q = _map_input_to_close_q(
                        raw_input_q,
                        scale=args.input_scale,
                        offset=args.input_offset,
                        low=close_q_min_raw,
                        high=open_q_raw,
                    )
                    last_follow_close_q = follow_close_q
                    if input_lost_warned:
                        print(f"[input] recovered | raw_q={raw_input_q:+.3f} close_q_in={follow_close_q:+.3f}")
                        input_lost_warned = False
                else:
                    follow_close_q = last_follow_close_q
                    if not input_lost_warned:
                        print(f"[warn] input motor timeout, hold last close_q={follow_close_q:+.3f}")
                        input_lost_warned = True

                gripper_q = float(gripper.state.position)
                gripper_dq = float(gripper.state.velocity)
                gripper_tau = float(gripper.state.torque)

                estimated_force = linkage.force_from_feedback(gripper_q, gripper_tau)
                baseline_tau = baseline.tau_at(gripper_q) if baseline is not None else 0.0
                delta_tau = (
                    baseline.closing_delta_tau(gripper_q, gripper_tau)
                    if baseline is not None
                    else max(0.0, -gripper_tau)
                )
                effective_force = delta_tau / max(linkage.radius_at(gripper_q), 1e-6)
                if filtered_effective_force is None:
                    filtered_effective_force = effective_force
                else:
                    filter_tau = max(0.0, float(args.force_filter_tau))
                    if filter_tau <= 1e-6:
                        force_alpha = 1.0
                    else:
                        force_alpha = _clamp(dt / (filter_tau + dt), 0.0, 1.0)
                    filtered_effective_force = filtered_effective_force + force_alpha * (
                        effective_force - filtered_effective_force
                    )
                contact_force_signal = filtered_effective_force
                radius_mm = linkage.radius_at(gripper_q) * 1000.0

                tau_cmd = 0.0
                kp = 0.0
                kd = float(args.hold_kd)
                q_target = gripper_q

                if not input_valid:
                    phase = "input_lost"
                    hold_target_q = gripper_q
                    contact_latched = True
                    contact_candidate_started_at = None
                    contact_release_started_at = None
                    no_contact_started_at = None
                    force_integral = 0.0
                    q_target = gripper_q
                    kp = float(args.hold_kp)
                    kd = float(args.hold_kd)
                    tau_cmd = 0.0
                else:
                    need_open = gripper_q < follow_close_q - float(args.position_epsilon)
                    need_close = gripper_q > follow_close_q + float(args.position_epsilon)

                    if phase == "idle":
                        if need_open:
                            phase = "opening"
                            phase_started_at = now
                            phase_started_q = gripper_q
                            force_integral = 0.0
                            hold_target_q = None
                            contact_latched = False
                            contact_candidate_started_at = None
                            contact_release_started_at = None
                            no_contact_started_at = None
                            print(f"[phase] idle -> opening | close_q_in={follow_close_q:+.3f}")
                        elif need_close:
                            phase = "approaching"
                            phase_started_at = now
                            phase_started_q = gripper_q
                            force_integral = 0.0
                            hold_target_q = None
                            contact_latched = False
                            contact_candidate_started_at = None
                            contact_release_started_at = None
                            no_contact_started_at = None
                            print(f"[phase] idle -> approaching | close_q_in={follow_close_q:+.3f}")
                    elif phase == "opening":
                        if need_close:
                            phase = "approaching"
                            phase_started_at = now
                            phase_started_q = gripper_q
                            force_integral = 0.0
                            hold_target_q = None
                            contact_latched = False
                            contact_candidate_started_at = None
                            contact_release_started_at = None
                            no_contact_started_at = None
                            print(f"[phase] opening -> approaching | close_q_in={follow_close_q:+.3f}")
                    elif phase == "approaching":
                        if need_open:
                            phase = "opening"
                            phase_started_at = now
                            phase_started_q = gripper_q
                            force_integral = 0.0
                            hold_target_q = None
                            contact_latched = False
                            contact_candidate_started_at = None
                            contact_release_started_at = None
                            no_contact_started_at = None
                            print(f"[phase] approaching -> opening | close_q_in={follow_close_q:+.3f}")
                    elif phase == "holding":
                        if need_open:
                            phase = "opening"
                            phase_started_at = now
                            phase_started_q = gripper_q
                            force_integral = 0.0
                            hold_target_q = None
                            contact_latched = False
                            contact_candidate_started_at = None
                            contact_release_started_at = None
                            no_contact_started_at = None
                            print(f"[phase] holding -> opening | close_q_in={follow_close_q:+.3f}")
                        elif need_close and not contact_latched:
                            phase = "approaching"
                            phase_started_at = now
                            phase_started_q = gripper_q
                            force_integral = 0.0
                            hold_target_q = None
                            contact_candidate_started_at = None
                            contact_release_started_at = None
                            no_contact_started_at = None
                            print(f"[phase] holding -> approaching | close_q_in={follow_close_q:+.3f}")
                    elif phase == "input_lost":
                        phase = "holding"
                        phase_started_at = now
                        phase_started_q = gripper_q
                        force_integral = 0.0
                        hold_target_q = follow_close_q
                        contact_latched = False
                        contact_candidate_started_at = None
                        contact_release_started_at = None
                        no_contact_started_at = now
                        print(f"[phase] input_lost -> holding | close_q_in={follow_close_q:+.3f}")

                    phase_age = now - phase_started_at

                    if phase == "idle":
                        tau_cmd = 0.0
                        q_target = gripper_q
                        kp = 0.0
                        kd = float(args.hold_kd)
                        hold_target_q = None
                        contact_release_started_at = None
                        force_integral = 0.0

                    elif phase == "opening":
                        tau_cmd = float(args.open_tau)
                        q_target = follow_close_q
                        kp = float(args.hold_kp)
                        kd = float(args.release_kd)
                        contact_latched = False
                        hold_target_q = None
                        contact_candidate_started_at = None
                        contact_release_started_at = None
                        no_contact_started_at = None
                        force_integral = 0.0

                        if gripper_q >= follow_close_q - float(args.position_epsilon):
                            phase = "holding"
                            phase_started_at = now
                            phase_started_q = gripper_q
                            hold_target_q = follow_close_q
                            contact_candidate_started_at = None
                            contact_release_started_at = None
                            no_contact_started_at = now
                            print(f"[phase] opening -> holding | q={gripper_q:+.3f} close_q_in={follow_close_q:+.3f}")

                    elif phase == "approaching":
                        close_error = max(0.0, gripper_q - follow_close_q)
                        close_near_window = max(float(args.close_near_window), float(args.position_epsilon))
                        close_near_scale = _clamp(float(args.close_near_scale), 0.0, 1.0)
                        if close_error >= close_near_window:
                            close_scale = 1.0
                        else:
                            close_scale = close_near_scale + (1.0 - close_near_scale) * (close_error / close_near_window)

                        tau_cmd = float(args.close_tau) * close_scale
                        q_target = gripper_q
                        kd = float(args.approach_kd)

                        moved_distance = max(0.0, float(phase_started_q) - gripper_q)
                        stalled = (
                            moved_distance >= float(args.contact_min_travel)
                            and abs(gripper_dq) <= float(args.contact_vel_threshold)
                            and phase_age >= 0.15
                        )
                        reached_target = gripper_q <= follow_close_q + float(args.position_epsilon)
                        force_ready = (
                            moved_distance >= float(args.contact_min_travel)
                            and contact_force_signal >= contact_force_enter
                        )
                        if force_ready:
                            if contact_candidate_started_at is None:
                                contact_candidate_started_at = now
                        else:
                            contact_candidate_started_at = None
                        touched_object = (
                            force_ready
                            and contact_candidate_started_at is not None
                            and (now - contact_candidate_started_at) >= float(args.contact_confirm_seconds)
                        )

                        if touched_object or stalled or reached_target:
                            phase = "holding"
                            phase_started_at = now
                            phase_started_q = gripper_q
                            force_integral = 0.0
                            if reached_target and not touched_object and not stalled:
                                hold_target_q = follow_close_q
                                contact_latched = False
                                contact_candidate_started_at = None
                                contact_release_started_at = None
                                no_contact_started_at = now
                                reason = "close_q_in"
                            else:
                                hold_target_q = gripper_q
                                contact_latched = True
                                contact_candidate_started_at = None
                                contact_release_started_at = None
                                no_contact_started_at = None
                                reason = "force" if touched_object else "stall"
                            print(
                                f"[phase] approaching -> holding | reason={reason} q={gripper_q:+.3f} "
                                f"close_q_in={follow_close_q:+.3f} eff_force={effective_force:.2f}N "
                                f"eff_filt={contact_force_signal:.2f}N"
                            )

                    elif phase == "holding":
                        if contact_latched:
                            q_target = gripper_q if hold_target_q is None else float(hold_target_q)
                            kp = float(args.hold_kp)
                        else:
                            q_target = gripper_q
                            kp = 0.0
                            contact_release_started_at = None
                        kd = float(args.hold_kd)

                        if contact_latched:
                            if contact_force_signal <= contact_force_release:
                                if contact_release_started_at is None:
                                    contact_release_started_at = now
                            else:
                                contact_release_started_at = None

                            contact_present = not (
                                contact_release_started_at is not None
                                and (now - contact_release_started_at) >= float(args.contact_release_seconds)
                            )
                        else:
                            contact_present = contact_force_signal >= contact_force_enter

                        if contact_present:
                            no_contact_started_at = None
                        elif no_contact_started_at is None:
                            no_contact_started_at = now

                        if contact_latched:
                            if contact_present:
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
                                tau_cmd = -min(
                                    float(args.max_hold_tau),
                                    linkage.closing_tau_from_force(gripper_q, commanded_force),
                                )
                            else:
                                force_integral = 0.0
                                tau_cmd = 0.0
                        else:
                            if contact_force_signal >= contact_force_enter:
                                if contact_candidate_started_at is None:
                                    contact_candidate_started_at = now
                            else:
                                contact_candidate_started_at = None

                            if (
                                contact_candidate_started_at is not None
                                and (now - contact_candidate_started_at) >= float(args.contact_confirm_seconds)
                            ):
                                hold_target_q = gripper_q
                                q_target = gripper_q
                                contact_latched = True
                                kp = float(args.hold_kp)
                                no_contact_started_at = None
                                force_integral = 0.0
                                phase_started_at = now
                                phase_started_q = gripper_q
                                contact_release_started_at = None
                                print(
                                    f"[phase] holding(no-contact) -> holding(contact) | "
                                    f"q={gripper_q:+.3f} eff_force={effective_force:.2f}N "
                                    f"eff_filt={contact_force_signal:.2f}N"
                                )
                            force_integral = 0.0
                            tau_cmd = 0.0

                        if (
                            float(args.no_contact_hold_seconds) > 0.0
                            and no_contact_started_at is not None
                            and (now - no_contact_started_at) >= float(args.no_contact_hold_seconds)
                            and not contact_present
                        ):
                            no_contact_duration = now - no_contact_started_at
                            phase = "idle"
                            phase_started_at = now
                            phase_started_q = gripper_q
                            hold_target_q = None
                            contact_latched = False
                            contact_candidate_started_at = None
                            contact_release_started_at = None
                            no_contact_started_at = None
                            force_integral = 0.0
                            q_target = gripper_q
                            kp = 0.0
                            tau_cmd = 0.0
                            print(
                                f"[phase] holding -> idle | no contact for {no_contact_duration:.2f}s "
                                f"close_q_in={follow_close_q:+.3f}"
                            )

                        if effective_force >= abort_force:
                            hold_target_q = gripper_q
                            q_target = gripper_q
                            tau_cmd = 0.0
                            kp = float(args.hold_kp)
                            contact_latched = True
                            contact_release_started_at = None
                            no_contact_started_at = None
                            force_integral = 0.0
                            print(f"[protect] over-force, lock current q={gripper_q:+.3f} eff_force={effective_force:.2f}N")

                    else:
                        raise RuntimeError(f"unknown phase: {phase}")

                ctrl.send_mit(
                    gripper,
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
                    p_m = _read_float_param_once(adapter, target_id=int(args.id), rid=int(Variable.p_m))
                    xout = _read_float_param_once(adapter, target_id=int(args.id), rid=int(Variable.xout))
                    p_m_text = "nan" if p_m is None else f"{p_m:+.3f}"
                    xout_text = "nan" if xout is None else f"{xout:+.3f}"
                    contact_text = "yes" if contact_latched else "no"
                    print(
                        f"phase={phase:10s} | "
                        f"in_q={raw_input_q:+.3f} src={last_input_source:7s} close_q_in={follow_close_q:+.3f} | "
                        f"q={gripper_q:+.3f} q_ref={q_target:+.3f} dq={gripper_dq:+.3f} | "
                        f"p_m={p_m_text} xout={xout_text} | "
                        f"tau_cmd={tau_cmd:+.3f} tau_fb={gripper_tau:+.3f} | "
                        f"force_est={estimated_force:6.2f}N eff_force={effective_force:6.2f}N "
                        f"eff_filt={contact_force_signal:6.2f}N | "
                        f"tau_free={baseline_tau:+.3f} delta_tau={delta_tau:+.3f} | "
                        f"radius={radius_mm:5.1f}mm contact={contact_text}"
                    )

                time.sleep(float(args.loop_sleep))

        except KeyboardInterrupt:
            print("\n[ctrl+c] exit")
        finally:
            ctrl.disable(input_motor)
            print("[disable] input motor ok")
            ctrl.disable(gripper)
            print("[disable] gripper ok")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
