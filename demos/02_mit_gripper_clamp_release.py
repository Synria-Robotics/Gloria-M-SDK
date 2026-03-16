from __future__ import annotations

"""
MIT 力矩“夹紧/松开”交互 demo

操作：
- 启动后默认先执行一次 b：慢慢回到张开位置 q=0.0
- 按键 a：MIT 模式下施加“夹紧方向”的固定小力矩（kp=kd=0）
- 按键 b：回到张开位置（不要太快），恢复 kp/kd，并把 tau 清零
- Ctrl+C：退出并自动失能
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


def _get_key_nonblocking() -> str | None:
    try:
        import msvcrt

        if msvcrt.kbhit():
            ch = msvcrt.getwch()
            return ch
        return None
    except Exception:
        return None


def main() -> int:
    ap = argparse.ArgumentParser(description="MIT 夹爪夹紧/松开交互 demo（按 a/b，Ctrl+C 退出）")
    ap.add_argument("--port", default="COM12", help="串口号。Windows 例如 COM8；Linux 例如 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600, help="波特率，默认 921600")
    ap.add_argument("--id", type=_parse_int, default="0x01", help="电机ID（MIT命令ID），默认 0x01")
    ap.add_argument("--fb-id", type=_parse_int, default="0x101", help="反馈ID，默认 0x101")
    args = ap.parse_args()

    safe_q = PositionRange(min=MIT_SAFE_Q_MIN, max=0.0)
    limits = Limits(pmax=3.14, vmax=10.0, tmax=12.0)

    act = Actuator(
        name="gripper",
        command_id=args.id,
        feedback_id=args.fb_id,
        limits=limits,
        safe_position=safe_q,
    )

    OPEN_Q = 0.0
    OPEN_KP = 25.0
    OPEN_KD = 0.8
    OPEN_SLEW_RATE = 0.6  # rad/s

    # 力矩前馈值：
    CLAMP_TAU = -0.5

    SEND_HZ = 200.0
    DT = 1.0 / SEND_HZ

    def step_open(q_cmd: float, target_q: float, dt: float) -> float:
        target_q = act.clamp_position(target_q)
        max_step = OPEN_SLEW_RATE * max(0.0, dt)
        err = target_q - q_cmd
        if err > max_step:
            return q_cmd + max_step
        if err < -max_step:
            return q_cmd - max_step
        return target_q

    with SerialCanAdapter(args.port, baudrate=args.baud, timeout=0.5) as adapter:
        apply_limits_and_save(adapter, motor_id=args.id, limits=limits)

        ctrl = CanController(adapter)
        ctrl.register(act)

        ok = ctrl.set_control_mode(act, ControlMode.MIT)
        print(f"[mode] set MIT: {ok}")
        ctrl.enable(act)
        print("[enable] ok")

        ctrl.refresh_state(act)
        q_cmd = float(act.state.position)

        target_q = OPEN_Q
        mode = "opening"  # 进入 demo 默认执行一次 b
        q_hold_for_torque = float(act.state.position)

        print("按键：a=夹紧(纯力矩)  b=松开到0.0  Ctrl+C=退出")

        last_print = 0.0
        t_prev = time.perf_counter()
        try:
            while True:
                t_now = time.perf_counter()
                dt = t_now - t_prev
                t_prev = t_now

                key = _get_key_nonblocking()
                if key is not None:
                    k = key.strip().lower()
                    if k == "a":
                        ctrl.refresh_state(act)
                        q_hold_for_torque = float(act.state.position)
                        mode = "clamp"
                        print("[mode] a: clamp torque")
                    elif k == "b":
                        # 每次从“夹紧/其它状态”切回张开时，都用当前反馈位置重置 q_cmd。
                        ctrl.refresh_state(act)
                        q_cmd = float(act.state.position)
                        mode = "opening"
                        target_q = OPEN_Q
                        print("[mode] b: open to 0.0")

                if mode == "clamp":
                    ctrl.send_mit(
                        act,
                        kp=0.0,
                        kd=0.0,
                        q=q_hold_for_torque,
                        dq=0.0,
                        tau=CLAMP_TAU,
                        poll=True,
                    )
                else:
                    q_cmd = step_open(q_cmd, target_q, dt)
                    ctrl.send_mit(
                        act,
                        kp=OPEN_KP,
                        kd=OPEN_KD,
                        q=q_cmd,
                        dq=0.0,
                        tau=0.0,
                        poll=True,
                    )
                    if abs(q_cmd - act.clamp_position(target_q)) < 0.02:
                        mode = "open_hold"

                if (time.time() - last_print) >= 0.1:
                    last_print = time.time()
                    print(
                        f"mode={mode:9s} | "
                        f"q={act.state.position:+.3f} dq={act.state.velocity:+.3f} tau={act.state.torque:+.3f}"
                    )

                time.sleep(DT)
        except KeyboardInterrupt:
            print("\n[ctrl+c] exit requested")
        finally:
            ctrl.disable(act)
            print("[disable] ok")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

