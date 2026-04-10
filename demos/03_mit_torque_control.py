from __future__ import annotations

"""
MIT 模式力控示例

目的：
- demo 同时打印电机回传的状态：位置 q、速度 dq、力矩 tau（反馈）

说明：
- 本 demo 仍然会给一个保持位置 hold_q
- 在此基础上叠加一个正弦力矩 tau_ff，让你观察反馈力矩的变化

注意：
- 力矩上限由 Limits.tmax 决定（默认是 12），超过会被打包时限幅
- 夹爪安全位置范围固定为 [MIT_SAFE_Q_MIN, MIT_SAFE_Q_MAX]，超出会被 clamp
"""

import argparse
import math
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
    MIT_SAFE_Q_MAX,
    MIT_SAFE_Q_MIN,
    PositionRange,
    SerialCanAdapter,
    apply_limits_and_save,
)


def _parse_int(s: str) -> int:
    return int(s, 0)


def main() -> int:
    ap = argparse.ArgumentParser(description="MIT 模式力控示例：使用额外力矩参数并打印反馈")
    ap.add_argument("--port", default="COM12", help="串口号。Windows 例如 COM8；Linux 例如 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600, help="波特率，默认 921600")
    ap.add_argument("--id", type=_parse_int, default="0x01", help="电机ID（MIT命令ID），默认 0x01")
    ap.add_argument("--fb-id", type=_parse_int, default="0x101", help="反馈ID，默认 0x101")
    ap.add_argument("--hold-q", type=float, default=1.2, help="保持位置(弧度)，默认 1.2（会自动 clamp 到安全范围）")
    ap.add_argument("--tau-amp", type=float, default=2.0, help="力矩正弦幅值，默认 2.0（单位与反馈 tau 一致）")
    ap.add_argument("--tau-freq", type=float, default=0.5, help="力矩正弦频率(Hz)，默认 0.5")
    # run-seconds = 0 表示一直运行，按 Ctrl+C 退出
    ap.add_argument("--run-seconds", type=float, default=0.0, help="运行时长（秒）。0 表示无限循环，Ctrl+C 退出")
    # 位置斜坡限速：避免一开始从“张开”突然冲到 hold_q
    ap.add_argument("--slew-rate", type=float, default=0.8, help="位置斜坡限速（rad/s），默认 0.8")
    # 力矩幅值渐入时间：避免一上来就满幅 tau_ff
    ap.add_argument("--tau-ramp-seconds", type=float, default=1.0, help="力矩渐入时间（秒），默认 1.0")
    args = ap.parse_args()

    # 固定你的夹爪安全范围：0.0 张开，正方向趋向闭合
    safe_q = PositionRange(min=MIT_SAFE_Q_MIN, max=MIT_SAFE_Q_MAX)
    # 固定你的 MIT 缩放上限（PMAX/VMAX/TMAX）
    limits = Limits(pmax=3.14, vmax=10.0, tmax=12.0)

    act = Actuator(
        name="gripper",
        command_id=args.id,
        feedback_id=args.fb_id,
        limits=limits,
        safe_position=safe_q,
    )

    with SerialCanAdapter(args.port, baudrate=args.baud, timeout=0.5) as adapter:
        apply_limits_and_save(adapter, motor_id=args.id, limits=limits)

        ctrl = CanController(adapter)
        ctrl.register(act)

        # 切换到 MIT 模式
        ok = ctrl.set_control_mode(act, ControlMode.MIT)
        print(f"[mode] set MIT: {ok}")

        # 使能
        ctrl.enable(act)
        print("[enable] ok")

        # 先刷新一次状态，用于初始化“当前期望位置”
        # 如果固件不支持 refresh 或没回包，act.state 可能还是 0，此时也安全（0=张开）
        ctrl.refresh_state(act)

        start_wall = time.time()
        start_mono = time.perf_counter()
        last_print = 0.0
        last_mono = start_mono

        # q_cmd：真正下发给电机的目标位置（带斜坡限速）
        q_cmd = float(act.state.position)
        # 目标保持位置（做一次 clamp，保证安全范围内）
        q_target = act.clamp_position(float(args.hold_q))
        try:
            while True:
                now_mono = time.perf_counter()
                t = now_mono - start_mono
                dt = now_mono - last_mono
                last_mono = now_mono

                # 可选：运行指定时长后退出（默认 0 = 不退出）
                if args.run_seconds and (time.time() - start_wall) > float(args.run_seconds):
                    break

                # 位置软启动：用斜坡限速逐步逼近 q_target，避免开头猛冲
                if dt > 0:
                    max_step = float(args.slew_rate) * float(dt)
                    err = q_target - q_cmd
                    if err > max_step:
                        q_cmd += max_step
                    elif err < -max_step:
                        q_cmd -= max_step
                    else:
                        q_cmd = q_target

                # 力矩幅值渐入（ramp）：0 -> 1
                ramp_T = max(0.0, float(args.tau_ramp_seconds))
                if ramp_T <= 1e-6:
                    ramp = 1.0
                else:
                    ramp = min(1.0, max(0.0, t / ramp_T))

                # 正弦扫力矩（叠加在位置保持上）
                tau_ff = (
                    ramp
                    * float(args.tau_amp)
                    * math.sin(2.0 * math.pi * float(args.tau_freq) * t)
                )

                # MIT 五参数：
                # - q 设为一个固定保持位置（夹爪夹在某个开度）
                # - dq 设 0
                # - tau 设为正弦前馈，观察反馈 tau 的响应
                ctrl.send_mit(
                    act,
                    kp=40.0,
                    kd=1.0,
                    q=q_cmd,
                    dq=0.0,
                    tau=tau_ff,
                    poll=True,
                )

                time.sleep(0.002)
                if (time.time() - last_print) >= 0.05:
                    last_print = time.time()
                    # 这里打印：
                    # - tau_ff：我们给进去的前馈力矩
                    # - q/dq/tau：电机当前反馈
                    print(
                        f"q_target={q_target:+.3f} q_cmd={q_cmd:+.3f} | "
                        f"tau_ff={tau_ff:+.3f} (ramp={ramp:.2f}) | "
                        f"q={act.state.position:+.3f} rad, "
                        f"dq={act.state.velocity:+.3f} rad/s, "
                        f"tau={act.state.torque:+.3f}"
                    )
        except KeyboardInterrupt:
            print("\n[ctrl+c] exit requested")
        finally:
            # 退出前失能
            ctrl.disable(act)
            print("[disable] ok")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

