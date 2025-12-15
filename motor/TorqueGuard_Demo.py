import time
import serial
from CAN import *
from XuanYa_Comm import MotorComm, gloria_max

"""
Torque 守护示例 (增强版)
------------------------
集成以下策略减少“卡顿/停-冲”现象:
1. 双阈值滞后: torque_limit_enter / torque_limit_exit，避免频繁进出保护。
2. 扭矩滤波: 指数平滑 tau_filtered 降低尖峰影响。
3. 状态机: NORMAL / PROTECT / RECOVERY 三态。
   - PROTECT: 软保持 + 缓慢卸载向安全中位。
   - RECOVERY: ramp_time 内插值回轨迹，速度与位置平滑过渡。
4. 冻结增益: 保护期固定 kp = kp_protect，不自动递增；恢复结束再继续增益爬升。
5. 速度裁剪: 保护期速度设为 0，阻断条件触发时不硬性跳大位移。
6. 动态频率: 保护/恢复期采用更低发送频率，减小机械冲击。
7. 统计扩展: 记录各状态驻留时间、进入次数、恢复次数、阻断方向。
8. 日志统一: kp 重置日志与实际值一致，并输出 ramp 进度与滤波扭矩。

运行:
    python TorqueGuard_Demo.py
Ctrl+C 退出。
可调参数请见 main() 顶部。
"""

def should_block_command(q_current: float, dq_current: float, tau_current: float,
                         q_set: float, dq_set: float, torque_limit: float) -> bool:
    """判断是否阻断控制指令。
    返回 True 表示阻断 (不发送), False 表示允许。
    """
    # 未超阈值直接放行
    if abs(tau_current) <= torque_limit:
        return False

    # 正向过载 (tau > +limit): 不允许进一步正向增加位置或更正的速度
    if tau_current > torque_limit:
        # 位置继续增大 (向更正) - 阻断
        if q_set > q_current:
            return True
        # 速度目标更正且高于当前速度 - 阻断
        if dq_set > 0 and dq_set > dq_current:
            return True
        return False  # 其它方向允许 (减速或回撤)

    # 负向过载 (tau < -limit): 不允许位置继续向负或速度更负
    if tau_current < -torque_limit:
        if q_set < q_current:
            return True
        if dq_set < 0 and dq_set < dq_current:
            return True
        return False

    return False


def main():
    # ---------------- 基础参数 ----------------
    demo_hz = 0.2            # 正弦轨迹频率
    tau_desired = 0.0        # MIT 力矩期望
    kd = 0.5

    # 控制增益
    kp = 1.0                 # 初始 kp
    kp_max = 20.0            # 最大 kp
    kp_step = 0.001          # 正常模式下爬升步长
    kp_protect = 3.0         # 保护模式固定 kp
    # 空载自适应增益限制
    load_threshold = 0.2     # |tau_filtered| < 此值视为空载
    kp_empty_max = 8.0       # 空载时 kp 最大值 (A方案调整)
    kd_empty = 0.8           # 空载加一点阻尼

    # 位置死区与轨迹滤波
    pos_deadband = 0.002     # rad 以内不发送位置变化
    alpha_traj = 0.25        # 轨迹指令滤波系数 (A方案提高响应)
    q_traj_filtered = None
    dq_traj_filtered = None
    deadband_hits = 0        # 死区命中统计

    # 扭矩阈值 (滞后)
    torque_limit_enter = 1.0   # 进入保护阈值
    torque_limit_exit  = 0.7   # 退出保护阈值 (稳态回落所需)

    # 扭矩滤波与导数估计
    alpha_tau = 0.25          # 滤波系数 (0<alpha<=1)，大则响应快，小则更平滑
    tau_filtered = 0.0
    tau_filtered_prev = 0.0
    tau_deriv = 0.0           # 扭矩变化率估计

    # 保护与恢复
    cooldown_time = 0.3       # 保护期最短持续时间
    ramp_time = 0.5           # 恢复期平滑插值时长
    protection_until = 0.0
    recovery_start = 0.0
    q_hold = None             # 进入保护时的保持位置

    # 发送频率统一到 200Hz
    frame_rate_hz = 200.0
    send_interval_normal = 1.0 / frame_rate_hz
    send_interval_protect = 1.0 / frame_rate_hz
    send_interval_recovery = 1.0 / frame_rate_hz
    current_send_interval = send_interval_normal

    # 统计
    stats_interval = 1.0
    last_stats_print = time.time()
    state_enter_times = {"NORMAL": time.time(), "PROTECT": None, "RECOVERY": None}
    state_durations = {"NORMAL": 0.0, "PROTECT": 0.0, "RECOVERY": 0.0}
    count_enter = {"PROTECT": 0, "RECOVERY": 0}
    blocked_pos = 0
    blocked_neg = 0
    unload_steps = 0
    amp_scale_sum = 0.0
    amp_scale_count = 0
    send_total = 0
    # 幅度二次滤波 & 位置步长限制
    amp_scale_filtered = 1.0
    max_step = 0.015         # 单循环最大位置变化 (rad) (A方案放宽追踪能力)
    q_cmd_prev = None        # 上一次发送的目标位置
    large_err_hits = 0       # 大误差次数统计
    large_err_threshold = 0.8  # rad 超过认为过大 (A方案减少误触发)

    # 状态机
    state = "NORMAL"  # 可取 NORMAL / PROTECT / RECOVERY

    # 初始化电机
    Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x101)
    serial_device = serial.Serial('COM3', 921600, timeout=0.5)
    MotorControl1 = MotorControl(serial_device)
    MotorControl1.addMotor(Motor1)
    MC = MotorComm()

    # 使能
    MotorControl1.disable(Motor1)
    time.sleep(0.5)
    MotorControl1.enable(Motor1)

    print(f"Torque Guard Demo Start. Enter={torque_limit_enter}Nm Exit={torque_limit_exit}Nm")
    t0 = time.time()

    try:
        while True:
            now = time.time()
            t = now - t0

            # ---------------- 轨迹规划 ----------------
            q_mid = gloria_max / 2.0
            q_amp = gloria_max / 2.0
            # 根据扭矩余量自适应缩放轨迹幅度 (越接近阈值缩得越小)
            # 余量 ratio = clamp( (torque_limit_enter - |tau_filtered|) / torque_limit_enter , 0, 1 )
            margin_ratio = max(0.0, min(1.0, (torque_limit_enter - abs(tau_filtered)) / torque_limit_enter))
            # 非线性缩放: 使用平方让高余量时接近1，低余量快速下降
            # A方案: 将指数2改为1.2，减少幅度过度压缩
            amp_scale = margin_ratio ** 1.2
            amp_scale_sum += amp_scale
            amp_scale_count += 1

            # 幅度二次滤波，平滑 amp_scale 防止突放大
            amp_scale_filtered = amp_scale_filtered + 0.2 * (amp_scale - amp_scale_filtered)

            q_traj = (q_amp * amp_scale_filtered) * cm.sin(2 * cm.pi * demo_hz * t) + q_mid
            dq_traj = 2 * cm.pi * demo_hz * ((q_amp * amp_scale_filtered) * cm.cos(2 * cm.pi * demo_hz * t))

            # 初始化滤波
            if q_traj_filtered is None:
                q_traj_filtered = q_traj
                dq_traj_filtered = dq_traj
            else:
                q_traj_filtered = q_traj_filtered + alpha_traj * (q_traj - q_traj_filtered)
                dq_traj_filtered = dq_traj_filtered + alpha_traj * (dq_traj - dq_traj_filtered)

            # ---------------- 当前状态 ----------------
            q_cur = MC.getPosRad(Motor1)
            dq_cur = MC.getVelRad(Motor1)
            tau_cur = MC.getTorque(Motor1)
            # 扭矩滤波与导数
            tau_filtered = alpha_tau * tau_cur + (1 - alpha_tau) * tau_filtered
            tau_deriv = (tau_filtered - tau_filtered_prev) / max(1e-6, current_send_interval)
            tau_filtered_prev = tau_filtered

            # ---------------- 状态机判定 ----------------
            if state == "NORMAL":
                # 空载判定
                is_empty_load = abs(tau_filtered) < load_threshold
                if is_empty_load:
                    # 限制 kp 上升并增加阻尼
                    if kp > kp_empty_max:
                        kp -= kp_step * 5  # 快速降回
                    elif kp < kp_empty_max:
                        kp += kp_step  # 缓慢靠近
                    kd_eff = kd_empty
                else:
                    # 正常模式增益缓慢爬升
                    if kp < kp_max:
                        kp += kp_step
                    kd_eff = kd
                # 进入保护条件: 滤波扭矩超过进入阈值 & 阻断条件成立
                if abs(tau_filtered) > torque_limit_enter and should_block_command(q_cur, dq_cur, tau_filtered, q_traj, dq_traj, torque_limit_enter):
                    state = "PROTECT"
                    count_enter["PROTECT"] += 1
                    protection_until = now + cooldown_time
                    q_hold = q_cur
                    kp = kp_protect
                    state_enter_times["PROTECT"] = now
                    print(f"[ENTER PROTECT] tauFilt={tau_filtered:.2f}Nm kp->{kp:.2f} cooldown={cooldown_time:.2f}s")
                    # 切换发送频率
                    current_send_interval = send_interval_protect  # 200Hz

            elif state == "PROTECT":
                # 自适应卸载：根据扭矩方向与大小决定微调步长
                safe_mid = q_mid
                # 基础回中位分量
                base_delta = LIMIT_MIN_MAX(safe_mid - q_cur, -0.01, 0.01)
                # 额外反向卸载分量，依据扭矩符号 (tau_filtered 正 -> 向负方向微移)
                unload_gain = 0.005  # 卸载步长基准
                # 根据扭矩幅值与变化率放大 (若扭矩仍在上升则加大卸载力度)
                rise_factor = 1.0
                if abs(tau_deriv) > 0.0:
                    rise_factor += LIMIT_MIN_MAX(tau_deriv * 0.02, -0.5, 2.0)  # 扭矩上升时增加
                mag_factor = min(3.0, 1.0 + (abs(tau_filtered) - torque_limit_enter) * 0.5) if abs(tau_filtered) > torque_limit_enter else 1.0
                unload_delta = 0.0
                if tau_filtered > torque_limit_enter:
                    unload_delta = -unload_gain * rise_factor * mag_factor
                elif tau_filtered < -torque_limit_enter:
                    unload_delta = unload_gain * rise_factor * mag_factor

                # 合成微调 (限制总幅度)
                delta = LIMIT_MIN_MAX(base_delta + unload_delta, -0.02, 0.02)
                q_cmd = q_cur + delta
                dq_cmd = 0.0
                unload_steps += 1
                MotorControl1.controlMIT(Motor1, kp, kd, q_cmd, dq_cmd, 0.0)

                # 判断是否可进入恢复：冷却到期 & 滤波扭矩低于退出阈值
                if now >= protection_until and abs(tau_filtered) < torque_limit_exit:
                    state = "RECOVERY"
                    count_enter["RECOVERY"] += 1
                    recovery_start = now
                    state_enter_times["RECOVERY"] = now
                    print(f"[ENTER RECOVERY] tauFilt={tau_filtered:.2f}Nm ramp_time={ramp_time:.2f}s")
                    current_send_interval = send_interval_recovery  # 200Hz

            elif state == "RECOVERY":
                # 平滑插值回轨迹
                blend = min(1.0, (now - recovery_start) / ramp_time)
                q_cmd = (1 - blend) * q_hold + blend * q_traj
                dq_cmd = blend * dq_traj  # 速度渐进
                MotorControl1.controlMIT(Motor1, kp, kd, q_cmd, dq_cmd, tau_desired)
                if blend >= 1.0:
                    # 恢复完成
                    state = "NORMAL"
                    state_enter_times["NORMAL"] = now
                    print(f"[BACK TO NORMAL] tauFilt={tau_filtered:.2f}Nm q resumed kp={kp:.2f}")
                    current_send_interval = send_interval_normal  # 200Hz

            # ---------------- NORMAL 模式下发送轨迹 ----------------
            if state == "NORMAL":
                # 应用死区 (位置与速度一起判断) 减少空载抖动
                # 位置步长限制 (slew rate) 基于滤波后的轨迹
                if q_cmd_prev is None:
                    q_cmd_prev = q_cur
                # 期望位置增量
                desired_step = q_traj_filtered - q_cmd_prev
                limited_step = LIMIT_MIN_MAX(desired_step, -max_step, max_step)
                q_slew = q_cmd_prev + limited_step
                q_cmd_prev = q_slew

                if abs(q_slew - q_cur) > large_err_threshold:
                    large_err_hits += 1
                    # 大误差时临时降低 kp 软化追赶
                    kp_eff_large = min(kp, 5.0)
                else:
                    kp_eff_large = kp

                if abs(q_slew - q_cur) < pos_deadband and abs(dq_traj_filtered) < 0.1:
                    q_cmd = q_cur
                    dq_cmd = 0.0
                    deadband_hits += 1
                else:
                    q_cmd = q_slew
                    dq_cmd = dq_traj_filtered
                MotorControl1.controlMIT(Motor1, kp_eff_large, kd_eff, q_cmd, dq_cmd, tau_desired)
                send_total += 1
                if send_total % 200 == 0:  # 降低打印频率避免刷屏
                    print(f"[NORMAL] kp={kp:.2f} kpEff={kp_eff_large:.2f} kdEff={kd_eff:.2f} empty={is_empty_load} tauFilt={tau_filtered:.2f} deriv={tau_deriv:.2f} ampScaleF={amp_scale_filtered:.2f} qCmd={q_cmd:.3f}")

            # ---------------- 统计与状态驻留时间 ----------------
            if state_enter_times["PROTECT"]:
                if state == "PROTECT":
                    state_durations["PROTECT"] += current_send_interval
            if state_enter_times["RECOVERY"]:
                if state == "RECOVERY":
                    state_durations["RECOVERY"] += current_send_interval
            if state == "NORMAL":
                state_durations["NORMAL"] += current_send_interval

            if now - last_stats_print >= stats_interval:
                last_stats_print = now
                avg_amp_scale = amp_scale_sum / max(1, amp_scale_count)
                print(
                    f"[STATS] state={state} kp={kp:.2f} tauFilt={tau_filtered:.2f} empty={(abs(tau_filtered)<load_threshold)} deadHits={deadband_hits} largeErrHits={large_err_hits} dTau={tau_deriv:.2f} enterP={count_enter['PROTECT']} enterR={count_enter['RECOVERY']} "
                    f"durN={state_durations['NORMAL']:.2f}s durP={state_durations['PROTECT']:.2f}s durR={state_durations['RECOVERY']:.2f}s unloadSteps={unload_steps} avgAmpScale={avg_amp_scale:.2f}")

            time.sleep(current_send_interval)

    except KeyboardInterrupt:
        serial_device.close()
        print("Exit Torque Guard Demo")


if __name__ == "__main__":
    main()
