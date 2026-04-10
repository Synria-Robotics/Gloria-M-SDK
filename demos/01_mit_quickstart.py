from __future__ import annotations

"""
MIT 往复开合示例

你只需要知道两件事就能跑：
- 默认的电机CAN ID：0x01
- 默认的反馈 ID：0x101

并且你的夹爪定义为：
- 位置单位为弧度(rad)
- 0.0 表示夹爪“完全张开”
- 沿正方向增大趋向“闭合”，安全范围默认为 [MIT_SAFE_Q_MIN, MIT_SAFE_Q_MAX]

本示例会在安全范围内做一个“往返开合”的位置轨迹，并打印：
- 期望位置 q_des
- 实际反馈位置 q
- 实际反馈速度 dq
- 实际反馈力矩 tau

注意：
- 发送控制帧后，电机才回一帧状态。
  本 demo 默认 poll=True，即每次发送后都会读取并解析回包，更新 act.state。
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
    ap = argparse.ArgumentParser(description="MIT 往复运动示例（夹爪：0 开，正方向趋向闭合）")
    ap.add_argument("--port", default="COM12", help="串口号。Windows 例如 COM8；Linux 例如 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600, help="波特率，默认 921600")
    # 这里的 id 是电机的 CAN ID
    ap.add_argument("--id", type=_parse_int, default="0x01", help="电机ID，默认 0x01")
    # 反馈帧的 CAN ID
    ap.add_argument("--fb-id", type=_parse_int, default="0x101", help="反馈ID，默认 0x101")
    # run-seconds = 0 表示一直运行，按 Ctrl+C 退出
    ap.add_argument("--run-seconds", type=float, default=0.0, help="运行时长（秒）。0 表示无限循环，Ctrl+C 退出")
    # 一个开合往返周期（秒）
    ap.add_argument("--sweep-period", type=float, default=6.0, help="往返开合周期（秒），默认 6.0")
    ap.add_argument("--slew-rate", type=float, default=0.8, help="位置斜坡限速（rad/s），默认 0.8")
    args = ap.parse_args()

    # 配置夹爪安全范围和缩放上限
    safe_q = PositionRange(min=MIT_SAFE_Q_MIN, max=MIT_SAFE_Q_MAX)
    limits = Limits(pmax=3.14, vmax=10.0, tmax=12.0)

    # 定义一个电机对象
    act = Actuator(
        name="gripper",
        command_id=args.id,
        feedback_id=args.fb_id,
        limits=limits,
        safe_position=safe_q,
    )

    # 打开串口并进入控制循环
    with SerialCanAdapter(args.port, baudrate=args.baud, timeout=0.5) as adapter:
        apply_limits_and_save(adapter, motor_id=args.id, limits=limits)

        ctrl = CanController(adapter)
        ctrl.register(act)

        # 设置电机控制模式为 MIT，失败显示 False
        ok = ctrl.set_control_mode(act, ControlMode.MIT)
        print(f"[mode] set MIT: {ok}")

        # 内部 sleep 0.1s 使能电机
        ctrl.enable(act)
        print("[enable] ok")

        # 初始化当前期望位置
        ctrl.refresh_state(act)

        start_wall = time.time()
        start_mono = time.perf_counter()
        last_print = 0.0
        last_mono = start_mono
        
        q_cmd = float(act.state.position)
        try:
            while True:
                now_mono = time.perf_counter()
                t = now_mono - start_mono
                dt = now_mono - last_mono
                last_mono = now_mono

                if args.run_seconds and (time.time() - start_wall) > float(args.run_seconds):
                    break

                # 生成一个基于 safe_q 的“往返”位置轨迹：u=1 时靠近张开(min)，u=0 时靠近闭合(max)
                period = max(0.1, float(args.sweep_period))
                u = (math.cos(2.0 * math.pi * (t / period)) * 0.5 + 0.5)  # u in [0,1]
                q_des = (1.0 - u) * safe_q.max + u * safe_q.min

                # max_step = slew_rate * dt
                if dt > 0:
                    max_step = float(args.slew_rate) * float(dt)
                    err = q_des - q_cmd
                    if err > max_step:
                        q_cmd += max_step
                    elif err < -max_step:
                        q_cmd -= max_step
                    else:
                        q_cmd = q_des

                # MIT 五参数控制：
                # - kp/kd：位置环/速度环增益
                # - q：期望位置(rad)
                # - dq：期望速度(rad/s)，这里设 0 表示“只做位置伺服”
                # - tau：额外力矩前馈（力控/助力），这里设 0
                ctrl.send_mit(
                    act,
                    kp=50.0,
                    kd=0.5,
                    q=q_cmd,
                    dq=0.0,
                    tau=0.0,
                    poll=True,
                )

                # 打印频率 20Hz
                time.sleep(0.002)
                if (time.time() - last_print) >= 0.05:
                    last_print = time.time()
                    print(
                        f"q_des={q_des:+.3f} q_cmd={q_cmd:+.3f} | "
                        f"q={act.state.position:+.3f} rad, "
                        f"dq={act.state.velocity:+.3f} rad/s, "
                        f"tau={act.state.torque:+.3f}"
                    )
        except KeyboardInterrupt:
            print("\n[ctrl+c] exit requested")
        finally:
            # 确保退出时失能
            ctrl.disable(act)
            print("[disable] ok")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

