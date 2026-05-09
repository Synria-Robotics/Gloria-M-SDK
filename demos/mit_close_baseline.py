# Copyright (c) 2026 Synria Robotics Co., Ltd.
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
MIT mode gripper no-load closing baseline calibration

Purpose:
    Record the motor's torque output during a free (no-load) close so that
    subsequent force-control demos can subtract this friction baseline and
    obtain a more accurate estimate of actual grip force.

How it works:
    1. Send a constant MIT closing torque command (--close-tau) in a tight loop.
    2. Sample position, velocity, and feedback torque at every iteration.
    3. Stop when the gripper reaches close-q, the estimated force exceeds
       --stop-force, or --timeout seconds elapse.
    4. Write two CSV files to --save-dir:
       - ``close_baseline_raw.csv``   — every raw sample (for inspection).
       - ``close_baseline_<prefix>.csv`` — samples binned by position and
         averaged, suitable for loading with TorqueBaseline.from_csv().

Calibration workflow:
    1. Run this script with the gripper in air (no object), note the output CSV.
    2. Pass that CSV to demo 03 via --baseline-csv to enable friction compensation.
    3. Re-run after any mechanical change (e.g. re-greasing, part replacement).

Usage:
    python mit_close_baseline.py [--port auto] [--id 0x01] [--close-tau -1.25]

Exit:
    Press Ctrl+C to exit early; partial data will still be saved.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

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


def _resolve_port(port: str) -> str:
    """返回 *port* 原值，或在只有一个可用串口时自动选中。"""
    if port and port.lower() != "auto":
        return port
    from serial.tools import list_ports

    found = list(list_ports.comports())
    if not found:
        raise SystemExit("[port] 未检测到串口，请插入串口转 CAN 适配器。")
    if len(found) > 1:
        names = ", ".join(p.device for p in found)
        raise SystemExit(f"[port] 检测到多个串口 ({names})，请使用 --port 明确指定。")
    print(f"[port] 自动选择 {found[0].device} ({found[0].description})")
    return found[0].device


@dataclass(frozen=True)
class CloseSample:
    elapsed_s: float
    position_rad: float
    velocity_rad_s: float
    tau_cmd_nm: float
    tau_fb_nm: float
    force_est_n: float


def _write_raw_csv(path: Path, samples: List[CloseSample]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "elapsed_s",
                "position_rad",
                "velocity_rad_s",
                "tau_cmd_nm",
                "tau_fb_nm",
                "force_est_n",
            ]
        )
        for sample in samples:
            writer.writerow(
                [
                    f"{sample.elapsed_s:.6f}",
                    f"{sample.position_rad:.6f}",
                    f"{sample.velocity_rad_s:.6f}",
                    f"{sample.tau_cmd_nm:.6f}",
                    f"{sample.tau_fb_nm:.6f}",
                    f"{sample.force_est_n:.6f}",
                ]
            )


def _write_binned_baseline_csv(path: Path, samples: List[CloseSample], bin_width_rad: float) -> None:
    buckets: Dict[int, List[CloseSample]] = {}
    for sample in samples:
        bucket_id = int(round(sample.position_rad / bin_width_rad))
        buckets.setdefault(bucket_id, []).append(sample)

    rows = []
    for bucket_id, bucket_samples in buckets.items():
        count = len(bucket_samples)
        q_mean = sum(s.position_rad for s in bucket_samples) / count
        dq_mean = sum(s.velocity_rad_s for s in bucket_samples) / count
        tau_fb_mean = sum(s.tau_fb_nm for s in bucket_samples) / count
        force_est_mean = sum(s.force_est_n for s in bucket_samples) / count
        rows.append((q_mean, dq_mean, tau_fb_mean, force_est_mean, count))

    rows.sort(key=lambda item: item[0], reverse=True)

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "position_mean_rad",
                "velocity_mean_rad_s",
                "tau_fb_mean_nm",
                "force_est_mean_n",
                "sample_count",
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    f"{row[0]:.6f}",
                    f"{row[1]:.6f}",
                    f"{row[2]:.6f}",
                    f"{row[3]:.6f}",
                    row[4],
                ]
            )


def main() -> int:
    ap = argparse.ArgumentParser(description="MIT 模式夹爪闭合测试 / 空载基线标定")
    ap.add_argument("--port", default="auto", help="串口号，auto 表示自动选择唯一串口")
    ap.add_argument("--baud", type=int, default=921600, help="串口波特率")
    ap.add_argument("--id", type=_parse_int, default="0x01", help="电机命令 CAN ID")
    ap.add_argument("--fb-id", type=_parse_int, default="0x201", help="电机反馈 CAN ID")

    ap.add_argument("--open-q", type=float, default=2.77, help="夹爪最大张开位置 [rad]")
    ap.add_argument("--close-q", type=float, default=0.003, help="夹爪闭合位置 [rad]")
    ap.add_argument("--close-tau", type=float, default=-1.25, help="闭合方向扭矩，必须为负 [Nm]")
    ap.add_argument("--kd", type=float, default=0.8, help="MIT 扭矩控制阻尼 kd")

    ap.add_argument("--stop-force", type=float, default=0.0, help="估算夹持力达到该值后停止，0 表示禁用 [N]")
    ap.add_argument("--radius-mm", type=float, default=12.0, help="用于估算夹持力的等效力臂 [mm]")
    ap.add_argument("--timeout", type=float, default=5.0, help="最长闭合测试时间 [s]")
    ap.add_argument("--position-epsilon", type=float, default=0.02, help="闭合位置容差 [rad]")
    ap.add_argument("--bin-width", type=float, default=0.05, help="基线分桶宽度 [rad]")
    ap.add_argument("--save-dir", default="demos/baseline", help="采样结果输出目录")
    ap.add_argument("--save-prefix", default="close_baseline", help="输出文件名前缀")
    ap.add_argument("--no-save", action="store_true", help="只测试，不保存 CSV")
    ap.add_argument("--loop-sleep", type=float, default=0.002, help="控制循环休眠时间 [s]")
    ap.add_argument("--print-hz", type=float, default=10.0, help="状态打印频率 [Hz]")
    args = ap.parse_args()
    args.port = _resolve_port(args.port)

    if args.close_tau >= 0.0:
        raise ValueError("close-tau 必须为负，负扭矩表示闭合")
    if args.radius_mm <= 0.0:
        raise ValueError("radius-mm 必须大于 0")
    if args.bin_width <= 0.0:
        raise ValueError("bin-width 必须大于 0")

    safe_q = PositionRange(min=min(args.open_q, args.close_q), max=max(args.open_q, args.close_q))
    limits = Limits(pmax=3.14, vmax=10.0, tmax=12.0)

    act = Actuator(
        name="gripper_close_test",
        command_id=args.id,
        feedback_id=args.fb_id,
        limits=limits,
        safe_position=safe_q,
    )

    close_q = act.clamp_position(float(args.close_q))
    radius_m = float(args.radius_mm) / 1000.0

    with SerialCanAdapter(args.port, baudrate=args.baud, timeout=0.5) as adapter:
        ctrl = CanController(adapter)
        ctrl.register(act)
        # Write safety limits and persist to flash before the baseline run.
        ctrl.apply_limits_and_save(act, limits)

        ok = ctrl.set_control_mode(act, ControlMode.MIT)
        print(f"[mode] set MIT: {ok}")
        ctrl.enable(act)
        print("[enable] ok")

        # Read the initial position before issuing any closing torque.
        ctrl.refresh_state(act)
        start_mono = time.perf_counter()
        last_print_at = 0.0
        stop_reason = "unknown"
        samples: List[CloseSample] = []

        try:
            while True:
                now = time.perf_counter()
                elapsed = now - start_mono

                ctrl.send_mit(
                    act,
                    kp=0.0,
                    kd=float(args.kd),
                    q=float(act.state.position),
                    dq=0.0,
                    tau=float(args.close_tau),
                    poll=True,
                )

                estimated_force = max(0.0, -float(act.state.torque)) / radius_m
                samples.append(
                    CloseSample(
                        elapsed_s=elapsed,
                        position_rad=float(act.state.position),
                        velocity_rad_s=float(act.state.velocity),
                        tau_cmd_nm=float(args.close_tau),
                        tau_fb_nm=float(act.state.torque),
                        force_est_n=float(estimated_force),
                    )
                )
                reached_close = act.state.position <= close_q + float(args.position_epsilon)
                reached_force = float(args.stop_force) > 0.0 and estimated_force >= float(args.stop_force)
                reached_timeout = elapsed >= float(args.timeout)

                if (time.time() - last_print_at) >= 1.0 / max(1.0, float(args.print_hz)):
                    last_print_at = time.time()
                    print(
                        f"elapsed={elapsed:5.2f}s | "
                        f"q={act.state.position:+.3f} dq={act.state.velocity:+.3f} | "
                        f"tau_cmd={args.close_tau:+.3f} tau_fb={act.state.torque:+.3f} | "
                        f"force_est={estimated_force:5.2f}N"
                    )

                if reached_close:
                    stop_reason = "close_position"
                    print(f"[stop] 到达闭合位置附近: q={act.state.position:+.3f}")
                    break
                if reached_force:
                    stop_reason = "force_limit"
                    print(f"[stop] 达到估算力阈值: force_est={estimated_force:.2f}N")
                    break
                if reached_timeout:
                    stop_reason = "timeout"
                    print(f"[stop] 达到超时时间: {elapsed:.2f}s")
                    break

                time.sleep(float(args.loop_sleep))

        except KeyboardInterrupt:
            stop_reason = "keyboard_interrupt"
            print("\n[ctrl+c] 收到退出请求")
        finally:
            ctrl.disable(act)
            print("[disable] ok")

        if samples:
            q_min = min(sample.position_rad for sample in samples)
            q_max = max(sample.position_rad for sample in samples)
            tau_min = min(sample.tau_fb_nm for sample in samples)
            tau_max = max(sample.tau_fb_nm for sample in samples)
            print(
                f"[summary] samples={len(samples)} stop_reason={stop_reason} "
                f"q_range=[{q_min:.3f}, {q_max:.3f}] tau_fb_range=[{tau_min:.3f}, {tau_max:.3f}]"
            )

        if samples and not args.no_save:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            save_dir = Path(args.save_dir)
            save_dir.mkdir(parents=True, exist_ok=True)

            raw_path = save_dir / f"{args.save_prefix}_{timestamp}_raw.csv"
            baseline_path = save_dir / f"{args.save_prefix}_{timestamp}_binned.csv"

            _write_raw_csv(raw_path, samples)
            _write_binned_baseline_csv(baseline_path, samples, float(args.bin_width))

            print(f"[save] raw csv: {raw_path}")
            print(f"[save] binned baseline csv: {baseline_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
