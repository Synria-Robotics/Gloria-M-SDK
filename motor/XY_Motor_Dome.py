import math as cm
from DM_CAN import *
import serial
import time
from XuanYa_Comm import *

def main(demoType=True):
    MotorControl1.disable(Motor1)
    time.sleep(1)
    MotorControl1.enable(Motor1)

    MC = MotorComm()
    kp = 1.0
    hz = 0.2
    duration = 5.0

    maxvel = 0.0
    while True:
        if demoType:
            time.sleep(0.001)
            t = time.time()
            q = gloria_max/2*cm.sin(2*cm.pi*hz*t) + gloria_max/2      # one 1.25s for 5s 2.97 max
            dq = 2*cm.pi*hz*(gloria_max/2*cm.cos(2*cm.pi*hz*t) + gloria_max/2)
            # q = cm.sin(2*cm.pi*hz*t)
            # dq = 2*cm.pi*hz*cm.cos(2*cm.pi*hz*t)
            if kp < 20.0:
                kp += 0.001
            MotorControl1.controlMIT(Motor1, kp, 1.0, q, dq, 0.0)
            print(f"pAngle={MC.getAngle100(Motor1)}\tvel={MC.getVel(Motor1)}°/s\tTorque={MC.getTorque(Motor1)}Nm")
        else:
            print(f" ")
            print(f"{gloria_max}")
            start_time = time.time()
            while time.time() - start_time < duration:
                MotorControl1.controlMIT(Motor1, 20.0, 1.0, gloria_max, 0.0, 0.0)
                # MotorControl1.controlMIT(Motor1, 1.0, 1.0, q*4, dq_from_q(q, hz), 0.0)
                # print(f"pos={MC.getAngle(Motor1)}°, rad={MC.getPosRad(Motor1)}rad")
                # print(f"vel={MC.getVel(Motor1)}°/s, velRad={MC.getVelRad(Motor1)}rad/s")
                # print(f"T={MC.getTorque(Motor1)}Nm, TMos={MC.getTMos(Motor1)}°C, TRoto={MC.getTRoto(Motor1)}°C")
                # print(f"pAngle={MC.getAngle100(Motor1)}\tvel={MC.getVel(Motor1)}°/s\tTorque={MC.getTorque(Motor1)}Nm")
                if MC.getVel(Motor1) > maxvel:
                    maxvel = MC.getVel(Motor1)
                print(f"maxvel={maxvel}°/s")
            print(f" ")
            print(f"{0.0}")
            start_time = time.time()
            while time.time() - start_time < duration:
                MotorControl1.controlMIT(Motor1, 20.0, 1.0, 0.0, 0.0, 0.0)
                # MotorControl1.controlMIT(Motor1, 1.0, 1.0, 0.0, dq_from_q(0.0, hz), 0.0)
                # print(f"pos={MC.getAngle(Motor1)}°, rad={MC.getPosRad(Motor1)}rad")
                # print(f"vel={MC.getVel(Motor1)}°/s, velRad={MC.getVelRad(Motor1)}rad/s")
                # print(f"T={MC.getTorque(Motor1)}Nm, TMos={MC.getTMos(Motor1)}°C, TRoto={MC.getTRoto(Motor1)}°C")
                # print(f"pAngle={MC.getAngle100(Motor1)}\tvel={MC.getVel(Motor1)}°/s\tTorque={MC.getTorque(Motor1)}Nm")
                if MC.getVel(Motor1) > maxvel:
                    maxvel = MC.getVel(Motor1)
                print(f"maxvel={maxvel}°/s")

if __name__ == "__main__":
    try:
        Motor1=Motor(DM_Motor_Type.DM4310,0x02,0x202)
        serial_device = serial.Serial('COM3', 921600, timeout=0.5)
        MotorControl1=MotorControl(serial_device)
        MotorControl1.addMotor(Motor1)

        main(False)
    except KeyboardInterrupt:
        serial_device.close()
        print("exit")

'''

def dq_from_q(q, hz, prev_dq=None, A=1.485, q0=1.485):
    # 归一化x
    s = (q - q0) / A
    # 数值安全
    if s > 1.0: s = 1.0
    if s < -1.0: s = -1.0
    # 余弦绝对值
    cos_abs = math.sqrt(max(0.0, 1.0 - s*s))
    # 符号判定：若无历史则默认正；否则沿用符号
    if prev_dq is None:
        cos_sign = 1.0
    else:
        cos_sign = 1.0 if prev_dq >= 0 else -1.0
    ω = 2*math.pi*hz
    dq = ω*(A*(cos_sign*cos_abs) + q0)
    return dq
if MotorControl1.switchControlMode(Motor1,Control_Type.POS_VEL):
    print("switch POS_VEL success")
print("sub_ver:",MotorControl1.read_motor_param(Motor1,DM_variable.sub_ver))
print("Gr:",MotorControl1.read_motor_param(Motor1,DM_variable.Gr))

print("PMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.PMAX))
print("MST_ID:",MotorControl1.read_motor_param(Motor1,DM_variable.MST_ID))
print("VMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.VMAX))
print("TMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.TMAX))

MotorControl1.save_motor_param(Motor1)
'''
