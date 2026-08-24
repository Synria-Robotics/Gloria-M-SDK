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
- 内置 MIT 协议打包/解包，以及反馈状态解析
- **传输层抽象**（`ICanTransport`）— 替换串口后端无需修改任何其他代码
- **`FakeCanAdapter`** — 纯内存实现，无需硬件即可运行全量单元测试
- **结构化日志**（`logging`）— 连接、模式切换、参数读取超时均会输出日志记录
## 项目结构

```
Gloria-M-SDK/
|-- src/gloria_m_sdk/       # SDK 核心库
|   |-- __init__.py         # 包入口，导出公开 API
|   |-- gripper_api.py      # 公开 API：GloriaGripper
|   |-- client.py           # 内部电机客户端
|   |-- gripper_control.py  # MIT 夹爪状态机：软闭合、接触检测、防堵转
|   |-- protocol.py         # 达妙协议打包/解析
|   |-- serial_can_adapter.py  # 串口转 CAN 传输层
|   |-- exceptions.py       # 异常体系（GloriaSdkError 及子类）
|   |-- transport.py        # ICanTransport 协议 + FakeCanAdapter（测试捆）
|   |-- registers.py        # 寄存器定义（Variable 枚举）
|   |-- types.py            # 数据类型（Limits、ControlMode 等）
|   `-- constants.py        # 常量定义
|-- tests/                  # Pytest 测试套件（无需硬件）
|   |-- conftest.py         # 公共 fixture（FakeCanAdapter 支撑的 gripper）
|   |-- test_gripper_api.py # 公开 GloriaGripper API 测试
|   |-- test_gripper_control.py # 夹爪状态机测试
|   |-- test_protocol.py    # 协议打包/解析测试
|   \-- test_client.py      # 内部电机客户端测试
|-- demos/                  # 示例脚本
|   |-- 00_check_connection.py # 只读检查配置/串口/CAN
|   |-- 01_enable_disable.py   # 使能/失能生命周期检查
|   |-- 02_read_state.py       # 读取当前位置/速度/力矩
|   |-- 03_set_zero.py         # 将当前位置设置为电机零点
|   |-- 04_switch_mode.py      # 切换 MIT/PV 控制模式
|   |-- 05_send_mit_frame.py   # 发送原始 MIT 控制帧
|   |-- 06_send_pv_frame.py    # 发送原始 PV 位置+速度帧
|   |-- 07_pv_gripper.py       # PV 打开/闭合/保持运动
|   |-- 08_mit_gripper.py      # MIT 软闭合夹爪控制
|   |-- 09_read_params.py      # 读取电机参数
|   `-- gripper_control.toml   # 唯一维护的夹爪配置
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

SDK 只暴露一个用户 API：`GloriaGripper`。夹爪运动、诊断、使能/失能、切模式、原始 MIT/PV 帧和 PV 运动都从这个对象调用；协议封包和串口/CAN 传输留在内部。

```
用户代码
    │
    ▼
┌───────────────────────────────────────┐
│  API  GloriaGripper                    │
│  move / grip / 诊断 / MIT / PV          │
└─────────────────┬─────────────────────┘
                  │
                  ▼
┌───────────────────────────────────────┐
│  协议层   protocol.py                  │
│  MIT / PV / 参数帧 / 反馈解析           │
└─────────────────┬─────────────────────┘
                  │
                  ▼
┌───────────────────────────────────────┐
│  传输层   SerialCanAdapter             │  serial_can_adapter.py
│  串口帧封装 · 原始收发                │
└───────────────────────────────────────┘

横切层（各层均可引用）：
  exceptions.py       — GloriaSdkError 异常体系
  types.py            — Limits、ControlMode、PositionRange、ActuatorState
  registers.py        — Variable（RID 枚举）
```

## 快速开始

```python
from gloria_m_sdk import GloriaGripper

with GloriaGripper.from_config("demos/gripper_control.toml") as gripper:
    gripper.move_to(1000)
    gripper.move_to(0)
```

带防堵转保护的夹取流程：

```python
from gloria_m_sdk import GloriaGripper

with GloriaGripper.from_config("demos/gripper_control.toml") as gripper:
    gripper.move_to(1000)
    gripper.move_to(0, stall_protection=True)
```

### GloriaGripper 方法

| 方法 | 说明 |
|------|------|
| `from_config(path)` | 从 TOML 配置创建 API 对象 |
| `move_to(value)` | 用户单位控制：`1000=打开`，`0=带保护闭合` |
| `open()` | 用 MIT 位置控制打开夹爪 |
| `close()` | 软闭合搜索；接触后不再追完全闭合位置 |
| `hold(duration_s=1.0)` | 在接触位置保持 |
| `release()` | 打开/释放夹爪 |
| `check_connection()` | 只读串口/CAN 诊断 |
| `scan_ids(ids)` | 只读扫描 CAN ID |
| `connect(mode=..., enable=...)` | 打开传输，读取 PMAX 作为本次 MIT 量程，可选切模式和使能 |
| `disconnect()` | 必要时失能并关闭传输 |
| `enable()` | 发送使能命令 |
| `disable()` | 发送失能命令 |
| `set_zero()` | 将当前位置设为零点 |
| `set_mode(mode)` | 切换控制模式；失败则抛出 `GloriaModeError` |
| `sync_pmax_from_motor()` | 读取 PMAX 并仅更新当前 SDK 实例；不写电机 Flash |
| `send_mit_for(...)` | 切到 MIT、使能、发送原始 MIT 帧，然后失能 |
| `send_pv_for(...)` | 切到 PV、使能、发送原始 PV 帧，然后失能 |
| `pv_open()` / `pv_close()` / `pv_hold_closed()` | 简单 PV 夹爪运动方法 |

`GloriaGripper.connect()` 默认会在解析反馈或发送 MIT 命令前读取真机
PMAX。真机未回复时保留配置中的 `[limits].pmax` 作为回退。该同步操作
只读；使用 `apply_limits=True` 明确向电机写入配置量程时不会执行同步。

### MIT 夹爪防堵转控制

`GloriaGripper` 内部使用 `GripperController` 状态机。闭合时不会用大 `Kp` 追完全闭合位置，而是低速搜索；检测到接触后记录当前位置，并切换到小力保持。

```python
from gloria_m_sdk import GloriaGripper

with GloriaGripper.from_config("demos/gripper_control.toml") as gripper:
    status = gripper.close()
    print(status.contact_detected, status.contact_position)
```

如果硬件后端能读取电流、温度或错误码，可以通过 `health_reader` 注入：

```python
from gloria_m_sdk import MotorHealth

gripper = GloriaGripper.from_config(
    "demos/gripper_control.toml",
    health_reader=lambda: MotorHealth(current_a=2.1, temperature_c=42.0, error_code=0),
)
```

### 异常体系

```python
GloriaSdkError               # 基类，一网打尽
├── GloriaConnectionError     # 串口打不开
├── GloriaCommunicationError  # 超时 / 帧格式错误
├── GloriaConfigError         # 参数越界
└── GloriaModeError           # 模式切换未确认
```

### 底层访问

| 符号 | 说明 |
|------|------|
| `SerialCanAdapter` | 原始串口转 CAN 传输 |
| `ICanTransport` | 自定义传输层的结构型协议 |
| `FakeCanAdapter` | 纯内存传输捆，无硬件可测试 |
| `Variable` | 寄存器 ID 枚举（RID） |

## 无硬件测试

`FakeCanAdapter` 是 `SerialCanAdapter` 的纯内存替代品。测试套件把它注入内部电机客户端，无需连接任何硬件即可运行协议逻辑：

```python
from gloria_m_sdk import FakeCanAdapter, ControlMode
from gloria_m_sdk.client import MotorClient
from gloria_m_sdk.registers import Variable

fake = FakeCanAdapter()
# 模拟电机在 set_mode() 后回复 CTRL_MODE = 2（POS_VEL）
fake.queue_param_reply(can_id=0x101, rid=int(Variable.CTRL_MODE),
                       value=int(ControlMode.POS_VEL), is_u32=True)

with MotorClient("任意端口", _transport=fake) as g:
    g.set_mode(ControlMode.POS_VEL)
    assert g.current_mode == ControlMode.POS_VEL
```

运行内置测试套件（无需硬件）：

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

所有 demo 默认读取 `demos/gripper_control.toml`。只有需要覆盖串口时才传 `--port`；如果配置里的 `connection.port` 是 `auto`，SDK 会自动选择最像 USB-CAN 的串口。

### 00_check_connection.py - 检查配置、串口和 CAN

使用 `GloriaGripper.check_connection()` 检查串口/CAN 通信。不使能电机、不切模式、不写参数、不保存 Flash、不运动夹爪。

```bash
python demos/00_check_connection.py --list-ports
python demos/00_check_connection.py
python demos/00_check_connection.py --scan-ids 1-10
```

### 01_enable_disable.py - 使能 / 失能电机

使用 `GloriaGripper.enable_for()` 连接电机、可选切模式、短时间使能，并在退出时确保失能。这个 demo 不执行打开、闭合或夹取动作。

```bash
python demos/01_enable_disable.py
python demos/01_enable_disable.py --duration-s 2.0 --mode mit
python demos/01_enable_disable.py --action disable
```

### 02_read_state.py - 读取当前位置和状态

使用 `GloriaGripper.refresh()` 持续打印当前位置、速度、力矩，以及是否收到反馈。默认不使能电机。

```bash
python demos/02_read_state.py
python demos/02_read_state.py --single
python demos/02_read_state.py --enable --single
python demos/02_read_state.py --enable --samples 10
```

### 03_set_zero.py - 将当前位置设置为零点

使用 `GloriaGripper.set_zero()` 先打印当前位置，再等待按回车确认后发送置零命令。这个操作会永久改变电机角度零点。

```bash
python demos/03_set_zero.py
```

### 04_switch_mode.py - 切换控制模式

使用 `GloriaGripper.set_mode()` 在 MIT 和 PV 模式之间切换。不使能电机，也不发送运动命令。

```bash
python demos/04_switch_mode.py --mode mit
python demos/04_switch_mode.py --mode pv
```

### 05_send_mit_frame.py - 发送 MIT 控制帧

使用 `GloriaGripper.send_mit_for()` 切到 MIT 模式、使能、发送原始 MIT 控制帧，然后失能。位置使用用户单位：`1000=打开`，`0=闭合`。

```bash
python demos/05_send_mit_frame.py
python demos/05_send_mit_frame.py --positions 1000,500,0,500,1000
```

### 06_send_pv_frame.py - 发送 PV 位置+速度帧

使用 `GloriaGripper.send_pv_for()` 切到 PV 模式、使能、发送原始 PV 位置/速度帧，然后失能。位置使用用户单位：`1000=打开`，`0=闭合`。

```bash
python demos/06_send_pv_frame.py
python demos/06_send_pv_frame.py --positions 1000,500,0,500,1000 --velocity 0.6 --duration-s 5.0
```

### 07_pv_gripper.py - PV 打开 / 闭合 / 保持

使用 `GloriaGripper` 的 PV 方法打开、低速闭合，并在配置的闭合位置保持。需要接触检测和防堵转时，用 MIT API demo。

```bash
python demos/07_pv_gripper.py
python demos/07_pv_gripper.py --close-vel 0.3 --hold-s 2.0
```

### 08_mit_gripper.py - MIT 夹爪用户示例

展示推荐给用户的真实调用方式：先打开到 `1000`，再调用 `move_to(0, stall_protection=True)`。通过配置创建的 API 对象里，这一个闭合命令会自动持续保持，直到用户按 Ctrl+C 退出。

```bash
python demos/08_mit_gripper.py
python demos/08_mit_gripper.py --force-level 4
python demos/08_mit_gripper.py --port /dev/cu.usbmodem00000000050C1
```

### 09_read_params.py - 读取电机参数

使用 `GloriaGripper.read_param()` 按寄存器名称或寄存器 ID 读取电机参数。不使能电机、不切模式、不写参数、不保存 Flash、不运动夹爪。

```bash
python demos/09_read_params.py
python demos/09_read_params.py --param CTRL_MODE --param PMAX --param VMAX --param TMAX
python demos/09_read_params.py --all
```

## 许可证

详见 [LICENSE](LICENSE) 文件。
