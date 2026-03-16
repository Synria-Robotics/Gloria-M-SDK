from __future__ import annotations

"""
PV 模式位置精度示例（打印目标位置与当前位置误差）

PV mode：
- “目标位置 + 追踪速度”

本 demo 的做法：
- 对每个目标位置 q_des，持续发送一段时间（hold_seconds）
- 在这段时间里按一定频率打印误差 err = q_des - q_feedback
- 你可以通过增大 hold_seconds 或降低 vel 来观察收敛过程
"""

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
    MIT_SAFE_Q_MIN,
    PositionRange,
    SerialCanAdapter,
    apply_limits_and_save,
)


def _parse_int(s: str) -> int:
    return int(s, 0)


def _parse_floats_csv(s: str) -> list[float]:
    parts = [p.strip() for p in s.split(",") if p.strip()]
    return [float(p) for p in parts]


def main() -> int:
    ap = argparse.ArgumentParser(description="PV 模式位置精度示例：打印目标位置与当前位置误差")
    ap.add_argument("--port", default="COM12", help="串口号。Windows 例如 COM8；Linux 例如 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600, help="波特率，默认 921600")
    # 这里的 id 是电机“CAN ID”
    ap.add_argument("--id", type=_parse_int, default="0x01", help="电机ID，默认 0x01")
    ap.add_argument("--fb-id", type=_parse_int, default="0x101", help="反馈ID，默认 0x101")
    ap.add_argument(
        "--targets",
        type=_parse_floats_csv,
        default="0.0,-1.0,-2.5,0.0",
        help="目标位置列表(弧度,逗号分隔)，默认 0.0,-1.0,-2.5,0.0",
    )
    ap.add_argument("--vel", type=float, default=4.0, help="追踪速度(弧度/秒)，默认 4.0（越大越快，但可能超调/抖动）")
    ap.add_argument("--hold-seconds", type=float, default=2.0, help="每个目标保持/追踪时长（秒），太短会导致误差评估不准")
    ap.add_argument("--print-hz", type=float, default=10.0, help="打印频率（Hz）")
    args = ap.parse_args()

    # 设定电机位置范围和缩放上限
    safe_q = PositionRange(min=MIT_SAFE_Q_MIN, max=0.0)
    limits = Limits(pmax=3.14, vmax=10.0, tmax=12.0)

    # 创建电机对象
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

        # 切换到 PV 模式
        ok = ctrl.set_control_mode(act, ControlMode.POS_VEL)
        print(f"[mode] set POS_VEL: {ok}")

        # 使能
        ctrl.enable(act)
        print("[enable] ok")

        print_interval = 1.0 / float(args.print_hz)
        try:
            while True:
                for q_des in args.targets:
                    q_des = act.clamp_position(q_des)
                    t_end = time.time() + float(args.hold_seconds)
                    next_print = 0.0

                    while time.time() < t_end:
                        ctrl.send_pos_vel(act, position=q_des, velocity=float(args.vel), poll=True)
                        time.sleep(0.01)

                        now = time.time()
                        if now >= next_print:
                            next_print = now + print_interval
                            err = q_des - act.state.position
                            print(
                                f"q_des={q_des:+.3f} | "
                                f"q={act.state.position:+.3f} | "
                                f"err={err:+.3f} rad"
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

