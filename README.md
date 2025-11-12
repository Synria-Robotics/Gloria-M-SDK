# Gloria-M-SDK

Gloria-M-SDK 是用于控制达妙（DM）系列电机的开发工具包，支持电机参数配置、控制模式切换、状态读取等功能，适用于基于CAN通信的电机控制系统开发。

## 依赖环境
- Python 3.x
- numpy
- pyserial

## 安装方法
1. 克隆仓库到本地
```bash
git clone https://github.com/your-username/Gloria-M-SDK.git
cd Gloria-M-SDK
```
2. 安装依赖包
```bash
pip install -r requirements.txt
```

## 快速开始
以下示例展示如何初始化电机、使能并进行基本控制：
```python
import serial
from DM_CAN import Motor, MotorControl, DM_Motor_Type
from XuanYa_Comm import MotorComm
import time
import numpy as np  # 补充缺失的numpy导入

# 初始化串口（根据实际端口修改）
serial_device = serial.Serial('COM3', 115200, timeout=0.1)

# 创建电机对象（电机类型、从机ID、主机ID）
motor = Motor(
    MotorType=DM_Motor_Type.DM4310,
    SlaveID=1,
    MasterID=100
)

# 创建电机控制对象
motor_control = MotorControl(serial_device)
motor_control.addMotor(motor)

# 初始化通信对象
mc = MotorComm()

# 失能后重新使能电机
motor_control.disable(motor)
time.sleep(1)
motor_control.enable(motor)

# MIT模式控制（位置环+速度+力矩环）
try:
    while True:
        # 设定期望位置、速度、力矩（示例：正弦轨迹）
        t = time.time()
        q = 1.5 * (1 + np.sin(2 * np.pi * 0.5 * t))  # 位置（弧度）
        dq = 1.5 * np.pi * np.cos(2 * np.pi * 0.5 * t)  # 速度（弧度/秒）
        tau = 0.0  # 力矩（Nm）
        
        # 发送控制命令（kp=20, kd=1）
        motor_control.controlMIT(motor, 20.0, 1.0, q, dq, tau)
        
        # 打印电机状态
        print(f"位置: {mc.getAngle(motor):.2f}°  速度: {mc.getVel(motor):.2f}°/s  力矩: {mc.getTorque(motor):.2f}Nm")
        time.sleep(0.001)
except KeyboardInterrupt:
    # 退出时失能电机
    motor_control.disable(motor)
    serial_device.close()
```

## 基本用法
1. 电机参数配置
```python
# 修改电机参数（RID为参数ID，data为参数值）
success = motor_control.change_motor_param(motor, RID=0x01, data=100)
if success:
    print("参数修改成功")

# 读取电机参数
param_value = motor_control.read_motor_param(motor, RID=0x01)
print(f"参数值: {param_value}")

# 保存参数到Flash
motor_control.save_motor_param(motor)
```

2. 控制模式切换
```python
from DM_CAN import Control_Type

# 切换到MIT控制模式
motor_control.switchControlMode(motor, Control_Type.MIT)

# 切换到位置-速度控制模式
motor_control.switchControlMode(motor, Control_Type.POS_VEL)
```

3. 状态读取
```python
# 获取位置（弧度）
pos_rad = mc.getPosRad(motor)

# 获取角度（度）
angle = mc.getAngle(motor)

# 获取速度（度/秒）
vel = mc.getVel(motor)

# 获取温度
t_mos = mc.getTMos(motor)  # MOS管温度
t_roto = mc.getTRoto(motor)  # 转子温度
```

## 注意事项
- 电机使能前建议等待 1-2 秒，确保硬件初始化完成
- 旧版本固件电机使能需使用enable_old方法并指定控制模式
- 通信前需确保 CAN 端口配置正确（波特率、端口号）
- 修改关键参数（如限位）后建议重启电机生效
- 长时间运行需定期调用refresh_motor_status更新电机状态
