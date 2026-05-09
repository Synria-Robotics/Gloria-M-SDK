# Gloria-M SDK

> 面向 Gloria-M 系列夹爪执行器的 Python SDK，用于通过串口转 CAN 适配器进行电机控制。

[English](README.md) | [简体中文](README.zh-CN.md)

版权所有 (c) 2026 Synria Robotics Co., Ltd.  
官网：https://synriarobotics.ai  
仓库：https://github.com/Synria-Robotics/Gloria-M-SDK/tree/main

## 功能特性

- 通过串口转 CAN 适配器与 Gloria-M 系列电机通信
- 支持 **MIT 模式**（kp/kd/扭矩前馈控制）和 **PV 模式**（位置 + 速度控制）
- 提供参数读写、电机使能和失能等基础控制能力
- 内置 MIT 协议打包/解包，以及反馈状态解析- **传输层抽象**（`ICanTransport`）— 替换串口后端无需修改任何其他代码
- **`FakeCanAdapter`** — 纯内存实现，无需硬件即可运行全量单元测试
- **结构化日志**（`logging`）— 连接、模式切换、参数读取超时均会输出日志记录
## 项目结构

```
Gloria-M-SDK/
|-- src/gloria_m_sdk/       # SDK 核心库
|   |-- __init__.py         # 包入口，导出公开 API
|   |-- client.py           # 门面层：GloriaGripper（推荐入口）
|   |-- exceptions.py       # 异常体系（GloriaSdkError 及子类）
|   |-- transport.py        # ICanTransport 协议 + FakeCanAdapter（测试捆）
|   |-- api/                # API 层：按领域拆分的子 API
|   |   |-- __init__.py
|   |   |-- base.py         # BaseAPI（共享控制器访问）
|   |   |-- motor_api.py    # MotorAPI：使能/失能/模式/归零/读取
|   |   |-- motion_api.py   # MotionAPI：send_mit / send_pos_vel
|   |   `-- param_api.py    # ParamAPI：读写寄存器、保存、应用限制
|   |-- actuator.py         # Actuator 和 ActuatorState 数据模型
|   |-- controller.py       # CanController（命令下发、反馈解析）
|   |-- protocol_mit.py     # MIT 协议打包/解包
|   |-- serial_can_adapter.py  # 串口转 CAN 传输层
|   |-- param_config.py     # 遗留辅助函数（建议改用 ctrl.apply_limits_and_save）
|   |-- registers.py        # 寄存器定义（Variable 枚举）
|   |-- types.py            # 数据类型（Limits、ControlMode 等）
|   |-- constants.py        # 常量定义
|   `-- gripper_baseline.py # 夹爪扭矩基线
|-- tests/                  # Pytest 测试套件（无需硬件）
|   |-- conftest.py         # 公共 fixture（FakeCanAdapter 支撑的 gripper）
|   |-- test_protocol_mit.py# MIT 位打包 round-trip 测试
|   |-- test_baseline.py    # TorqueBaseline 加载与插分测试
|   \-- test_client.py      # GloriaGripper 门面集成测试
|-- demos/                  # 示例脚本
|   |-- 01_gripper_quicktest.py  # PV 模式往复运动测试
|   |-- 02_pv_control.py        # PV 模式柔顺闭合
|   |-- 03_mit_linkage_force_control.py  # MIT 连杆夹爪力控
|   |-- mit_close_baseline.py   # MIT 空载闭合基线采集
|   `-- baseline/               # 基线数据 CSV 输出目录
|-- CHANGELOG.md
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

一并安装测试依赖（pytest）：

```bash
pip install -e ".[dev]"
```

## SDK 分层设计

SDK 采用严格五层架构。上位机应用仅需与最上面两层（门面层和 API 层）交互。

```
用户代码
    │
    ▼
┌───────────────────────────────────────┐
│  门面层   GloriaGripper  (client.py)   │  ← 推荐入口
│  .motor / .motion / .params             │
│  .state / .current_mode / .is_connected │
└─────────────────┬─────────────────────┘
                  │
                  ▼
┌───────────────────────────────────────┐
│  API 层   MotorAPI / MotionAPI          │  api/
│           ParamAPI                       │
└─────────────────┬─────────────────────┘
                  │
                  ▼
┌───────────────────────────────────────┐
│  控制层   CanController                │  controller.py
│  命令下发 · 反馈解析                    │
└─────────────────┬─────────────────────┘
                  │
                  ▼
┌───────────────────────────────────────┐
│  协议层   protocol_mit.py              │
│  MIT 位打包 · float32 编解码            │
└─────────────────┬─────────────────────┘
                  │
                  ▼
┌───────────────────────────────────────┐
│  传输层   SerialCanAdapter             │  serial_can_adapter.py
│  串口帧封装 · 原始收发                │
└───────────────────────────────────────┘

横切层（各层均可引用）：
  exceptions.py       — GloriaSdkError 异常体系
  types.py            — Limits、ControlMode、PositionRange
  actuator.py         — Actuator、ActuatorState
  registers.py        — Variable（RID 枚举）
  gripper_baseline.py — TorqueBaseline
```

## 快速开始

```python
from gloria_m_sdk import GloriaGripper, ControlMode

with GloriaGripper("COM5") as g:  # 将 COM5 替换为实际串口号
    g.motor.set_mode(ControlMode.POS_VEL)
    g.motor.enable()
    g.motor.refresh()
    print(f"位置 = {g.state.position:.3f} rad")

    # 移动到开仓位置
    g.motion.send_pos_vel(position=2.5, velocity=1.0)
```

### GloriaGripper 构造参数

```python
GloriaGripper(
    port,                    # 如 "COM5" 或 "/dev/ttyUSB0"；可用 'auto' 自动检测
    *,
    baudrate=921_600,
    command_id=0x01,         # 电机命令 CAN ID
    feedback_id=0x101,       # 电机反馈 CAN ID
    limits=None,             # Limits(pmax, vmax, tmax)，默认 (3.14, 10, 12)
    safe_position=None,      # PositionRange(min, max) — 位置限幅
    baseline_csv=None,       # 空载扭矩基线 CSV 路径
    timeout=0.5,             # 串口读超时 [s]
    _transport=None,         # 测试锆：传入 FakeCanAdapter 替代实际串口
)
```

### GloriaGripper 属性

| 属性 | 类型 | 说明 |
|------|------|------|
| `state` | `ActuatorState` | 最新反馈快照（位置、速度、扭矩） |
| `current_mode` | `ControlMode \| None` | 电机最后确认的控制模式；`set_mode()` 前为 `None` |
| `is_connected` | `bool` | `True` 表示串口已打开 |

### GloriaGripper.motor — MotorAPI

| 方法 | 说明 |
|------|------|
| `enable()` | 发送使能命令 |
| `disable()` | 发送失能命令 |
| `set_zero()` | 将当前位置设为零点 |
| `set_mode(mode)` | 切换控制模式；失败则抛出 `GloriaModeError` |
| `refresh()` | 广播请求状态并更新 `gripper.state` |
| `poll()` | 解析待处理 RX 包，更新执行器状态 |

### GloriaGripper.motion — MotionAPI

| 方法 | 说明 |
|------|------|
| `send_mit(*, kp, kd, q, dq, tau)` | 发送 MIT 扭矩控制帧 |
| `send_pos_vel(*, position, velocity)` | 发送 PV 位置+速度帧 |

### GloriaGripper.params — ParamAPI

| 方法 | 说明 |
|------|------|
| `read(rid, *, timeout_s)` | 读取寄存器；超时返回 `None` |
| `write_f32(rid, value)` | 写入 float32 寄存器 |
| `write_u32(rid, value)` | 写入 uint32 寄存器 |
| `save()` | 将参数持久化到 Flash |
| `apply_limits(limits)` | 写入 PMAX/VMAX/TMAX 并保存 |

### 异常体系

```python
GloriaSdkError               # 基类，一网打尽
├── GloriaConnectionError     # 串口打不开
├── GloriaCommunicationError  # 超时 / 帧格式错误
├── GloriaConfigError         # 参数越界
└── GloriaModeError           # 模式切换未确认
```

### 底层访问（高级用户）

`CanController` 和 `SerialCanAdapter` 仍然导出并在示例脚本中直接使用。

| 符号 | 说明 |
|------|------|
| `CanController` | 直接命令下发 / 反馈解析 |
| `SerialCanAdapter` | 原始串口转 CAN 传输 |
| `ICanTransport` | 自定义传输层的结构型协议 |
| `FakeCanAdapter` | 纯内存传输捆，无硬件可测试 |
| `Variable` | 寄存器 ID 枚举（RID） |
| `TorqueBaseline` | 空载扭矩基线，用于力估算 |

## 无硬件测试

`FakeCanAdapter` 是 `SerialCanAdapter` 的纯内存替代品。通过 `_transport` 参数注入，无需连接任何硬件即可运行全量 SDK 逻辑：

```python
from gloria_m_sdk import FakeCanAdapter, GloriaGripper, ControlMode
from gloria_m_sdk.registers import Variable

fake = FakeCanAdapter()
# 模拟电机在 set_mode() 后回复 CTRL_MODE = 2（POS_VEL）
fake.queue_param_reply(can_id=0x101, rid=int(Variable.CTRL_MODE),
                       value=int(ControlMode.POS_VEL), is_u32=True)

with GloriaGripper("任意端口", _transport=fake) as g:
    g.motor.set_mode(ControlMode.POS_VEL)
    assert g.current_mode == ControlMode.POS_VEL
```

运行内置测试套件（60 项测试，约 1 秒，无需硬件）：

```bash
pytest tests/ -v
```

## 开启日志

SDK 通过标准 `logging` 模块输出日志记录，开启方式：

```python
import logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(name)s %(levelname)s %(message)s",
)
```

| 级别 | 触发事件 |
|------|---------|
| `INFO` | 连接、断开、使能、失能、模式确认、限制应用 |
| `WARNING` | 模式切换超时、`read_param` 超时、`set_zero`（永久改变零点） |
| `DEBUG` | 每一条 CAN 帧收发 |

## 示例
### 01_gripper_quicktest.py - PV 模式往复运动测试
夹爪会在打开位置和闭合位置之间反复运动，用于快速验证 PV 控制模式是否正常工作。

```bash
python demos/01_gripper_quicktest.py --port auto --id 0x01 --close-q 0.0 --open-q 2.5 --vel 1.0
```

### 02_pv_control.py - PV 模式柔顺闭合

先打开到 2.5 rad，再以较低速度在 PV 模式下闭合到 0 rad。适合柔和夹取较脆弱的物体。

```bash
python demos/02_pv_control.py --port auto --open-q 2.5 --close-q 0.0 --close-vel 0.3
```

### 03_mit_linkage_force_control.py - MIT 连杆夹爪力控

基于 MIT 扭矩控制实现“接近 - 接触 - 保持 - 释放”流程，并通过可配置的力臂曲线估算指尖夹持力。
如果没有指定 baseline 文件则默认选择 close_baseline_4310.csv

```bash
python demos/03_mit_linkage_force_control.py --port auto --open-q 2.77 --close-q 0.003 --target-force 15
```

使用 4340 强力版本夹爪

```bash
python demos/03_mit_linkage_force_control.py --port auto --baseline-csv ".\demos\baseline\close_baseline_4340.csv" --target-force 30 --contact-force 60
```

**MIT 控制公式：**

$$\tau_{out} = k_p \cdot (q_{target} - q_{fb}) + k_d \cdot (dq_{target} - dq_{fb}) + \tau_{ff}$$

### mit_close_baseline.py - MIT 空载闭合基线采集

该脚本用于在 MIT 模式下以固定负扭矩让夹爪空载闭合，记录闭合过程中的位置、速度、反馈扭矩和估算夹持力。输出的基线文件可作为 `03_mit_linkage_force_control.py` 的 `--baseline-csv` 输入，用于扣除夹爪自身摩擦、机构阻力等空载负载。

建议在没有夹持物的情况下运行：

```bash
python demos/mit_close_baseline.py --port auto --close-tau -1.25
```

常用参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--port` | auto | 串口号 |
| `--baud` | 921600 | 串口波特率 |
| `--id` | 0x01 | 电机命令 CAN ID |
| `--fb-id` | 0x201 | 电机反馈 CAN ID |
| `--open-q` | 2.77 | 夹爪最大张开位置 [rad] |
| `--close-q` | 0.003 | 夹爪闭合位置 [rad] |
| `--close-tau` | -1.25 | 闭合方向扭矩 [N·m]，必须为负值 |
| `--kd` | 0.8 | MIT 扭矩控制阻尼项 |
| `--stop-force` | 0.0 | 估算夹持力达到该阈值后停止，0 表示禁用 [N] |
| `--radius-mm` | 12.0 | 用于估算夹持力的等效力臂 [mm] |
| `--timeout` | 5.0 | 最长采集时间 [s] |
| `--position-epsilon` | 0.02 | 判定到达闭合位置的容差 [rad] |
| `--bin-width` | 0.05 | 按位置生成基线曲线时的分桶宽度 [rad] |
| `--save-dir` | demos/baseline | CSV 输出目录 |
| `--save-prefix` | close_baseline | CSV 文件名前缀 |
| `--no-save` | false | 只运行测试，不保存 CSV |

运行完成后默认生成两个 CSV 文件：

```text
demos/baseline/{save_prefix}_{timestamp}_raw.csv
demos/baseline/{save_prefix}_{timestamp}_binned.csv
```

原始采样 CSV 每一行对应控制循环中的一次采样：

| 字段 | 说明 |
|------|------|
| `elapsed_s` | 从本次测试开始到当前采样点的时间 [s] |
| `position_rad` | 当前电机位置反馈 [rad] |
| `velocity_rad_s` | 当前电机速度反馈 [rad/s] |
| `tau_cmd_nm` | 当前发送的 MIT 扭矩命令 [N·m] |
| `tau_fb_nm` | 电机反馈扭矩 [N·m] |
| `force_est_n` | 根据反馈扭矩估算的夹持力 [N]，计算方式为 `max(0, -tau_fb_nm) / (radius_mm / 1000)` |

分桶基线 CSV 会把位置相近的原始采样点归为一组，并对每组求平均，适合后续作为基线曲线使用：

| 字段 | 说明 |
|------|------|
| `position_mean_rad` | 当前位置分桶内的平均位置 [rad] |
| `velocity_mean_rad_s` | 当前位置分桶内的平均速度 [rad/s] |
| `tau_fb_mean_nm` | 当前位置分桶内的平均反馈扭矩 [N·m] |
| `force_est_mean_n` | 当前位置分桶内的平均估算夹持力 [N] |
| `sample_count` | 当前位置分桶内包含的原始采样点数量 |

## 常用参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--port` | auto | 串口号 |
| `--baud` | 921600 | 串口波特率 |
| `--id` | 0x01 | 电机命令 CAN ID |
| `--fb-id` | 0x101 | 电机反馈 CAN ID |
| `--open-q` | 2.5 | 打开位置 [rad] |
| `--close-q` | 0.0 | 闭合位置 [rad] |
| `--baseline-csv` | ".\\demos\\baseline\\close_baseline_4310.csv" | 云犀夹爪的基线负载文件，默认是4310的参数。4340版本需手动改用close_baseline_4340.csv |
| `--target-force` | 15 | 目标夹持力 |
| `--contact-force` | 10 | 接触检测力阈值，使用4340版本夹爪时需改为60 |

## 许可证

详见 [LICENSE](LICENSE) 文件。
