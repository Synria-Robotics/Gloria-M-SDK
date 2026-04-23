# Gloria-M SDK

> 面向 Gloria-M 系列夹爪执行器的 Python SDK，用于通过串口转 CAN 适配器进行电机控制。

[English](README.md) | [简体中文](README.zh-CN.md)

版权所有 (c) 2026 Synria Robotics Co., Ltd.  
官网：https://synriarobotics.ai

## 功能特性

- 通过串口转 CAN 适配器与 Gloria-M 系列电机通信
- 支持 **MIT 模式**（kp/kd/扭矩前馈控制）和 **PV 模式**（位置 + 速度控制）
- 提供参数读写、电机使能和失能等基础控制能力
- 内置 MIT 协议打包/解包，以及反馈状态解析

## 项目结构

```
Gloria-M-SDK/
|-- src/gloria_m_sdk/       # SDK 核心库
|   |-- __init__.py         # 包入口，导出公开 API
|   |-- actuator.py         # 执行器抽象
|   |-- controller.py       # 高层控制器：命令下发、反馈解析
|   |-- protocol_mit.py     # MIT 协议打包/解包
|   |-- serial_can_adapter.py  # 串口转 CAN 适配器
|   |-- param_config.py     # 参数写入与保存
|   |-- registers.py        # 寄存器定义（RID 枚举）
|   |-- types.py            # 数据类型（Limits、ControlMode 等）
|   |-- constants.py        # 常量定义
|   `-- gripper_baseline.py # 夹爪扭矩基线
|-- demos/                  # 示例脚本
|   |-- 01_gripper_quicktest.py  # PV 模式往复运动测试
|   |-- 02_pv_control.py        # PV 模式柔顺闭合
|   |-- 03_mit_linkage_force_control.py  # MIT 连杆夹爪力控
|   `-- output/                 # 基线数据 CSV 输出目录
|-- pyproject.toml
|-- requirements.txt
|-- README.md
`-- README.zh-CN.md
```

## 环境要求

- Python >= 3.11
- 已连接到 COM 口的串口转 CAN 适配器
- Gloria-M 系列电机

## 安装

```bash
pip install -r requirements.txt
```

也可以使用可编辑/开发模式安装：

```bash
pip install -e .
```

## 示例程序

### 01_gripper_quicktest.py - PV 模式往复运动测试

夹爪会在打开位置和闭合位置之间反复运动，用于快速验证 PV 控制模式是否正常工作。

```bash
python demos/01_gripper_quicktest.py --port COM5 --id 0x01 --close-q 0.0 --open-q 2.5 --velocity 1.0
```

### 02_pv_control.py - PV 模式柔顺闭合

先打开到 2.5 rad，再以较低速度在 PV 模式下闭合到 0 rad。适合柔和夹取较脆弱的物体。

```bash
python demos/02_pv_control.py --port COM5 --open-q 2.5 --close-q 0.0 --close-vel 0.3
```

### 03_mit_linkage_force_control.py - MIT 连杆夹爪力控

基于 MIT 扭矩控制实现“接近 - 接触 - 保持 - 释放”流程，并通过可配置的力臂曲线估算指尖夹持力。

```bash
python demos/03_mit_linkage_force_control.py --port COM5 --open-q 2.77 --close-q 0.003 --target-force 15
```

使用 4340 强力版本夹爪

```bash
python demos/03_mit_linkage_force_control.py --port COM5 --baseline-csv ".\demos\output\close_baseline_4340.csv" --target-force 30 --contact-force 60
```

**MIT 控制公式：**

$$\tau_{out} = k_p \cdot (q_{target} - q_{fb}) + k_d \cdot (dq_{target} - dq_{fb}) + \tau_{ff}$$

## 常用参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--port` | COM5 | 串口号 |
| `--baud` | 921600 | 串口波特率 |
| `--id` | 0x01 | 电机命令 CAN ID |
| `--fb-id` | 0x101 | 电机反馈 CAN ID |
| `--open-q` | 2.5 | 打开位置 [rad] |
| `--close-q` | 0.0 | 闭合位置 [rad] |
| `--baseline-csv` | ".\\demos\\output\\close_baseline_4310.csv" | 云犀夹爪的基线负载文件，默认是4310的参数。4340版本需手动改用close_baseline_4340.csv |
| `--target-force` | 15 | 目标夹持力 |
| `--contact-force` | 10 | 接触检测力阈值，使用4340版本夹爪时需改为60 |

## 许可证

详见 [LICENSE](LICENSE) 文件。
