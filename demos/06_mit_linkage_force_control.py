from __future__ import annotations

"""
MIT 连杆夹爪力控示例。

这个示例面向“连杆驱动夹爪”场景。由于指尖力和电机力矩不是简单一一对应，
这里用一个“随电机角度变化的等效力臂曲线”近似描述传动比：

    指尖力 ~= 闭合方向力矩 / 等效力臂

控制流程如下：
1. 先用正扭矩把夹爪张开到上限附近。
2. 再用负扭矩朝闭合方向接近物体。
3. 通过估算夹持力、速度停滞或到达闭合极限判定接触。
4. 进入保持阶段，用 MIT 的扭矩控制维持目标夹持力。
5. 达到保持时间后，再用正扭矩自动释放。

默认的力臂曲线只是占位参数，实际使用时请替换成你的连杆机构标定数据。
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


def _slew(current: float, target: float, max_rate: float, dt: float) -> float:
    if dt <= 0.0:
        return current
    if max_rate <= 0.0:
        return target
    max_step = max_rate * dt
    err = target - current
    if err > max_step:
        return current + max_step
    if err < -max_step:
        return current - max_step
    return target


def _is_open_enough(position: float, open_q: float, epsilon: float) -> bool:
    """
    判断夹爪是否已经处在“足够张开”的区域。
    只要位置已经接近 open_q 或超过 open_q，就允许进入下一阶段，
    避免因为零点偏差或机械上限误差卡死在 opening/releasing。
    """

    margin = max(0.04, float(epsilon) * 2.0)
    return float(position) >= float(open_q) - margin


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


@dataclass(frozen=True)
class LinkageForceProfile:
    """
    电机角度 q [rad] 到等效力臂 [m] 的分段线性映射。

    如果连杆标定足够准确，radius_at(q) 就可以近似当前位置下的局部传动比，
    从而把电机力矩更合理地换算成指尖力，而不是简单用固定力臂。
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
                    "力臂曲线格式必须是 q:radius_mm，例如 0.003:10,1.4:14,2.77:18"
                )
            q_text, radius_mm_text = item.split(":", 1)
            q = float(q_text.strip())
            radius_mm = float(radius_mm_text.strip())
            if radius_mm <= 0.0:
                raise ValueError("力臂曲线中的半径必须大于 0 mm")
            raw_points.append((q, radius_mm / 1000.0))

        if len(raw_points) < 2:
            raise ValueError("力臂曲线至少需要两个点")

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
        description="MIT 连杆夹爪力控示例，包含接触判定与力保持"
    )
    ap.add_argument("--port", default="COM12", help="串口号，例如 COM8 或 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600, help="串口波特率")
    ap.add_argument("--id", type=_parse_int, default="0x07", help="电机命令 CAN ID")
    ap.add_argument("--fb-id", type=_parse_int, default="0x207", help="电机反馈 CAN ID")

    ap.add_argument("--open-q", type=float, default=2.77, help="夹爪最大张开位置 [rad]")
    ap.add_argument(
        "--close-q",
        type=float,
        default=0.003,
        help="夹爪闭合位置 [rad]",
    )
    ap.add_argument(
        "--radius-profile",
        default=_default_radius_profile(),
        help="连杆等效力臂曲线，格式为 q:radius_mm",
    )
    ap.add_argument(
        "--baseline-csv",
        default="",
        help="空载扭矩基线 CSV，建议使用 07_mit_close_test.py 生成的 *_binned.csv",
    )

    ap.add_argument("--target-force", type=float, default=15.0, help="目标夹持力 [N]")
    ap.add_argument("--open-tau", type=float, default=0.25, help="张开方向扭矩，必须为正 [Nm]")
    ap.add_argument("--close-tau", type=float, default=-0.25, help="闭合方向扭矩，必须为负 [Nm]")
    ap.add_argument(
        "--contact-force",
        type=float,
        default=10.0,
        help="接触判定力阈值 [N]；加载基线后表示增量夹持力阈值",
    )
    ap.add_argument(
        "--abort-force",
        type=float,
        default=65.0,
        help="力保护阈值 [N]；加载基线后表示增量夹持力保护阈值",
    )
    ap.add_argument(
        "--max-hold-tau",
        type=float,
        default=20,
        help="保持阶段允许的最大闭合力矩 [Nm]",
    )

    ap.add_argument("--approach-kd", type=float, default=0.8, help="接近阶段 MIT 扭矩控制阻尼 kd")
    ap.add_argument("--hold-kd", type=float, default=0.5, help="保持阶段 MIT 扭矩控制阻尼 kd")
    ap.add_argument("--release-kd", type=float, default=0.8, help="释放阶段 MIT 扭矩控制阻尼 kd")

    ap.add_argument("--force-kp", type=float, default=0.6, help="外环力控 kp")
    ap.add_argument("--force-ki", type=float, default=0.4, help="外环力控 ki")
    ap.add_argument(
        "--integral-limit",
        type=float,
        default=20.0,
        help="力控积分限幅 [N*s]",
    )
    ap.add_argument("--hold-seconds", type=float, default=3.0, help="保持夹持时间 [s]")
    ap.add_argument("--cycles", type=int, default=1, help="执行循环次数，0 表示一直循环")

    ap.add_argument(
        "--contact-vel-threshold",
        type=float,
        default=0.08,
        help="堵转判定速度阈值 [rad/s]",
    )
    ap.add_argument(
        "--contact-min-travel",
        type=float,
        default=0.05,
        help="允许触发堵转接触判定前的最小运动量 [rad]",
    )
    ap.add_argument(
        "--contact-error-threshold",
        type=float,
        default=0.05,
        help="保留参数，当前纯扭矩模式下未使用 [rad]",
    )
    ap.add_argument(
        "--position-epsilon",
        type=float,
        default=0.02,
        help="阶段切换位置容差 [rad]",
    )
    ap.add_argument("--loop-sleep", type=float, default=0.002, help="控制循环休眠时间 [s]")
    ap.add_argument("--print-hz", type=float, default=10.0, help="状态打印频率 [Hz]")
    ap.add_argument(
        "--start-phase",
        choices=("auto", "opening", "approaching"),
        default="approaching",
        help="起始阶段：auto 自动判断，approaching 表示直接从闭合接近开始",
    )
    ap.add_argument("--hold-q", type=float, default=None, help="进入保持阶段后锁定的目标位置 [rad]；未设置时 position-only 使用 close-q")
    ap.add_argument("--hold-kp", type=float, default=25.0, help="位置保持刚度 kp，仅在设置 --hold-q 后生效")
    ap.add_argument("--position-only", action="store_true", help="只运动到设定闭合位置并保持；未设置 hold-q 时默认使用 close-q")
    return ap


def main() -> int:
    args = _build_arg_parser().parse_args()

    # 兼容旧默认值。你前面的实测表明 -1.25 Nm 可以稳定闭合，
    # 如果用户没有显式覆盖旧默认值，就自动提升到更接近实机可用的量级。
    if abs(float(args.open_tau) - 0.25) <= 1e-9 or abs(float(args.open_tau) - 1.25) <= 1e-9:
        args.open_tau = 1.60
    if abs(float(args.close_tau) - (-0.25)) <= 1e-9 or abs(float(args.close_tau) - (-1.25)) <= 1e-9:
        args.close_tau = -1.60

    raw_open_q = float(args.open_q)
    raw_close_q = float(args.close_q)
    safe_q = PositionRange(min=min(raw_open_q, raw_close_q), max=max(raw_open_q, raw_close_q))
    limits = Limits(pmax=3.14, vmax=10.0, tmax=12.0)
    linkage = LinkageForceProfile.from_text(args.radius_profile)
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
        raise ValueError("当前示例默认按“角度减小为闭合方向”设计，请保证 close-q <= open-q")
    if open_tau <= 0.0:
        raise ValueError("open-tau 必须为正，正扭矩表示张开")
    if close_tau >= 0.0:
        raise ValueError("close-tau 必须为负，负扭矩表示闭合")

    if position_only:
        args.target_force = 0.0
        args.max_hold_tau = 0.0
        args.hold_seconds = float("inf")
        args.abort_force = float("inf")

    with SerialCanAdapter(args.port, baudrate=args.baud, timeout=0.5) as adapter:
        apply_limits_and_save(adapter, motor_id=args.id, limits=limits)

        ctrl = CanController(adapter)
        ctrl.register(act)

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

        initial_q = float(act.state.position)
        if args.start_phase == "opening":
            phase = "opening"
        elif args.start_phase == "approaching":
            phase = "approaching"
        else:
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

                estimated_force = linkage.force_from_feedback(act.state.position, act.state.torque)
                radius_mm = linkage.radius_at(act.state.position) * 1000.0
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
                                f"[warn] opening 阶段位移过小，直接切换到 approaching | "
                                f"q={act.state.position:+.3f}"
                            )
                        print(f"[阶段] approaching，第 {cycles_done + 1} 次夹持")

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
                            f"[阶段] holding，原因={reason}，当前位置={act.state.position:+.3f} "
                            f"有效估算力={effective_force:.2f}N"
                        )

                elif phase == "holding":
                    if position_only:
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
                        q_target = float(act.state.position) if hold_q_cmd is None else float(hold_q_cmd)
                        kp = 0.0 if hold_q_cmd is None else float(args.hold_kp)
                        kd = args.hold_kd

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
                        linkage.closing_tau_from_force(act.state.position, commanded_force),
                    )

                    if effective_force >= float(args.abort_force):
                        phase = "releasing"
                        phase_started_at = now
                        phase_started_q = float(act.state.position)
                        print(f"[保护] 触发过力释放，有效估算力={effective_force:.2f}N")
                    elif phase_age >= float(args.hold_seconds):
                        phase = "releasing"
                        phase_started_at = now
                        phase_started_q = float(act.state.position)
                        print(f"[阶段] releasing，第 {cycles_done + 1} 次夹持结束")

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
                        print(f"[循环] 已完成 {cycles_done} 次")

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
                    p_m = _read_float_param_once(adapter, target_id=args.id, rid=int(Variable.p_m))
                    xout = _read_float_param_once(adapter, target_id=args.id, rid=int(Variable.xout))
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
            print("\n[ctrl+c] 收到退出请求")
        finally:
            ctrl.disable(act)
            print("[disable] ok")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
