# Gloria-M SDK

> Python SDK for serial-to-CAN motor control, targeting the Gloria-M series gripper actuators.

[English](README.md) | [简体中文](README.zh-CN.md)

Copyright (c) 2026 Synria Robotics Co., Ltd.  
Website: https://synriarobotics.ai  
Repository: https://github.com/Synria-Robotics/Gloria-M-SDK/tree/main

## Features

- Communicates with Gloria-M series motors through a serial-to-CAN adapter
- Supports **MIT mode** (kp/kd/torque feedforward control) and **PV mode** (position + velocity)
- Provides parameter read/write and enable/disable primitives
- Built-in MIT protocol packing/unpacking and feedback state parsing
- **Transport abstraction** (`ICanTransport`) — swap the serial backend without touching any other code
- **`FakeCanAdapter`** — full in-memory stub for hardware-free unit testing
- **Structured logging** — connect/disconnect, mode switches, and parameter timeouts all emit log records

## Project Structure

```
Gloria-M-SDK/
|-- src/gloria_m_sdk/       # SDK core library
|   |-- __init__.py         # Package entry point; exports public API
|   |-- gripper_api.py      # Public API: GloriaGripper
|   |-- client.py           # Internal motor client
|   |-- gripper_control.py  # MIT gripper state machine: soft close, contact, stall protection
|   |-- protocol.py         # Damiao protocol packing/parsing
|   |-- serial_can_adapter.py  # Serial-to-CAN transport layer
|   |-- exceptions.py       # Exception hierarchy (GloriaSdkError and subclasses)
|   |-- transport.py        # ICanTransport protocol + FakeCanAdapter (testing stub)
|   |-- registers.py        # Register definitions (Variable enum)
|   |-- types.py            # Data types (Limits, ControlMode, PositionRange)
|   `-- constants.py        # Constant definitions
|-- tests/                  # Pytest test suite (no hardware required)
|   |-- conftest.py         # Shared fixtures (FakeCanAdapter-backed gripper)
|   |-- test_gripper_api.py # Public GloriaGripper API tests
|   |-- test_gripper_control.py # Gripper state-machine tests
|   |-- test_protocol.py    # Protocol bit-packing/parsing tests
|   `-- test_client.py      # Internal motor client tests
|-- demos/                  # Example scripts
|   |-- 00_check_connection.py # Read-only config/serial/CAN check
|   |-- 01_enable_disable.py   # Enable/disable motor lifecycle check
|   |-- 02_read_state.py       # Read current position/velocity/torque
|   |-- 03_set_zero.py         # Set current motor position as zero
|   |-- 04_switch_mode.py      # Switch MIT/PV control mode
|   |-- 05_send_mit_frame.py   # Send raw MIT control frame
|   |-- 06_send_pv_frame.py    # Send raw PV position + velocity frame
|   |-- 07_pv_gripper.py       # PV open/close/hold motion
|   |-- 08_mit_gripper.py      # MIT soft-close gripper control
|   |-- 09_read_params.py      # Read motor parameters
|   `-- gripper_control.toml   # Single maintained gripper config
|-- CHANGELOG.md
|-- pyproject.toml
|-- requirements.txt
|-- README.md
`-- README.zh-CN.md
```

## Requirements

- Python >= 3.11
- Serial-to-CAN adapter connected to a COM port
- Gloria-M series motor

## Installation

```bash
pip install -r requirements.txt
```

Or install in editable/development mode:

```bash
pip install -e .
```

To also install test dependencies (pytest):

```bash
pip install -e ".[dev]"
```

## SDK Layers

The SDK exposes one user-facing API: `GloriaGripper`. It owns normal gripper
motion, diagnostics, enable/disable, mode switching, raw MIT/PV frame helpers,
and PV motion helpers. Protocol packing and serial/CAN transport stay internal.

```
User code
    │
    ▼
┌─────────────────────────────────────────┐
│  API  GloriaGripper                     │
│  move / grip / diagnose / MIT / PV      │
└──────────────────┬──────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────┐
│  Protocol   protocol.py                 │
│  MIT / PV / parameter frames / feedback │
└──────────────────┬──────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────┐
│  Transport  SerialCanAdapter            │  serial_can_adapter.py
│  serial framing · raw TX/RX             │
└─────────────────────────────────────────┘

Cross-cutting (any layer may import):
  exceptions.py  — GloriaSdkError hierarchy
  types.py       — Limits, ControlMode, PositionRange, ActuatorState
  registers.py   — Variable (RID enum)
```

## Quick Start

```python
from gloria_m_sdk import GloriaGripper

with GloriaGripper.from_config("demos/gripper_control.toml") as gripper:
    gripper.move_to(1000)
    gripper.move_to(0)
```

For protected gripping:

```python
from gloria_m_sdk import GloriaGripper

with GloriaGripper.from_config("demos/gripper_control.toml") as gripper:
    gripper.move_to(1000)
    gripper.move_to(0, stall_protection=True)
```

### GloriaGripper methods

| Method | Description |
|--------|-------------|
| `from_config(path)` | Create API object from TOML config |
| `move_to(value)` | Move in user units: `1000=open`, `0=protected close` |
| `open()` | Open gripper using MIT position control |
| `close()` | Soft-close search; stops chasing close limit after contact |
| `hold(duration_s=1.0)` | Hold at detected contact position |
| `release()` | Open/release the gripper |
| `check_connection()` | Read-only serial/CAN diagnostic |
| `scan_ids(ids)` | Read-only CAN ID scan |
| `connect(mode=..., enable=...)` | Open transport, read PMAX for runtime MIT scaling, optionally switch mode and enable |
| `disconnect()` | Disable if needed and close transport |
| `enable()` | Send enable command |
| `disable()` | Send disable command |
| `set_zero()` | Set current position as zero |
| `set_mode(mode)` | Switch control mode; raises `GloriaModeError` on failure |
| `sync_pmax_from_motor()` | Read PMAX and update only this SDK instance; never writes motor Flash |
| `send_mit_for(...)` | Switch to MIT, enable, send raw MIT frames, then disable |
| `send_pv_for(...)` | Switch to PV, enable, send raw PV frames, then disable |
| `pv_open()` / `pv_close()` / `pv_hold_closed()` | Simple PV gripper motion helpers |

By default, `GloriaGripper.connect()` reads the motor's PMAX before decoding
feedback or sending MIT commands. If the motor does not reply, the configured
`[limits].pmax` value is retained as a fallback. This synchronization is
read-only and is skipped when `apply_limits=True` because that mode explicitly
writes the configured limits to the motor.

### MIT Gripper Stall Protection

`GloriaGripper` uses `GripperController` internally. During close it does not
drive a high-`Kp` target at the fully closed position. It performs a soft
search, records the contact position, then holds with low stiffness and
optional feedforward torque.

```python
from gloria_m_sdk import GloriaGripper

with GloriaGripper.from_config("demos/gripper_control.toml") as gripper:
    status = gripper.close()
    print(status.contact_detected, status.contact_position)
```

If your hardware backend can read current, temperature, or motor error codes,
inject them through `health_reader`:

```python
from gloria_m_sdk import MotorHealth

gripper = GloriaGripper.from_config(
    "demos/gripper_control.toml",
    health_reader=lambda: MotorHealth(current_a=2.1, temperature_c=42.0, error_code=0),
)
```

### Exception hierarchy

```python
GloriaSdkError          # base — catch-all
├── GloriaConnectionError   # serial port cannot be opened
├── GloriaCommunicationError# timeout / malformed packet
├── GloriaConfigError       # invalid parameter value
└── GloriaModeError         # mode switch not confirmed
```

### Low-level access

| Symbol | Description |
|--------|-------------|
| `SerialCanAdapter` | Raw serial-to-CAN transport |
| `ICanTransport` | Structural protocol for custom transport backends |
| `FakeCanAdapter` | In-memory transport stub for hardware-free testing |
| `Variable` | Register ID enum (RID) |

## Testing without hardware

`FakeCanAdapter` is an in-memory drop-in for `SerialCanAdapter`. The test suite
injects it into the internal motor client so protocol logic can run without a
physical motor or serial port:

```python
from gloria_m_sdk import FakeCanAdapter, ControlMode
from gloria_m_sdk.client import MotorClient
from gloria_m_sdk.registers import Variable

fake = FakeCanAdapter()
# Simulate the motor echoing CTRL_MODE = 2 (POS_VEL) after set_mode()
fake.queue_param_reply(can_id=0x101, rid=int(Variable.CTRL_MODE),
                       value=int(ControlMode.POS_VEL), is_u32=True)

with MotorClient("unused", _transport=fake) as g:
    g.set_mode(ControlMode.POS_VEL)
    assert g.current_mode == ControlMode.POS_VEL
```

Run the built-in test suite (no hardware):

```bash
pytest tests/ -v
```

## Enabling logging

The SDK emits log records at `INFO` / `WARNING` / `DEBUG` through the
standard `logging` module. Enable them with:

```python
import logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(name)s %(levelname)s %(message)s",
)
```

| Level | Events |
|-------|--------|
| `INFO` | connect, disconnect, enable, disable, mode confirmed, limits applied |
| `WARNING` | mode switch timeout, `read_param` timeout, `set_zero` (destructive) |
| `DEBUG` | every CAN frame TX/RX |

## Demos

All demos read `demos/gripper_control.toml` by default. Pass `--port` only when
you want to override `connection.port`; if the configured port is `auto`, the
SDK selects the most likely USB-CAN serial port.

### 00_check_connection.py - check config, serial, and CAN

Uses `GloriaGripper.check_connection()` to check serial/CAN communication without enabling
the motor, switching mode, writing parameters, saving flash, or moving the
gripper.

```bash
python demos/00_check_connection.py --list-ports
python demos/00_check_connection.py
python demos/00_check_connection.py --scan-ids 1-10
```

### 01_enable_disable.py - enable / disable motor

Uses `GloriaGripper.enable_for()` to connect, optionally switch mode, enable briefly,
and always disable on exit. It does not command open, close, or grip motion.

```bash
python demos/01_enable_disable.py
python demos/01_enable_disable.py --duration-s 2.0 --mode mit
python demos/01_enable_disable.py --action disable
```

### 02_read_state.py - read current position and state

Uses `GloriaGripper.refresh()` to continuously print current position, velocity,
torque, and whether feedback was received. By default it does not enable the
motor.

```bash
python demos/02_read_state.py
python demos/02_read_state.py --single
python demos/02_read_state.py --enable --single
python demos/02_read_state.py --enable --samples 10
```

### 03_set_zero.py - set current position as zero

Uses `GloriaGripper.set_zero()` to print the current position first, then waits
for Enter before sending the motor zeroing command. This permanently changes
the motor's angle origin.

```bash
python demos/03_set_zero.py
```

### 04_switch_mode.py - switch control mode

Uses `GloriaGripper.set_mode()` to switch the motor between MIT and PV modes. It does
not enable the motor and does not command motion.

```bash
python demos/04_switch_mode.py --mode mit
python demos/04_switch_mode.py --mode pv
```

### 05_send_mit_frame.py - send MIT control frame

Uses `GloriaGripper.send_mit_for()` to switch to MIT mode, enable, send raw MIT control
frames, then disable. Positions use user units: `1000=open`, `0=closed`.

```bash
python demos/05_send_mit_frame.py
python demos/05_send_mit_frame.py --positions 1000,500,0,500,1000
```

### 06_send_pv_frame.py - send PV position + velocity frame

Uses `GloriaGripper.send_pv_for()` to switch to PV mode, enable, send raw PV
position/velocity frames, then disable. Positions use user units:
`1000=open`, `0=closed`.

```bash
python demos/06_send_pv_frame.py
python demos/06_send_pv_frame.py --positions 1000,500,0,500,1000 --velocity 0.6 --duration-s 5.0
```

### 07_pv_gripper.py - PV open / close / hold

Uses `GloriaGripper` PV helpers to open, gently close, then hold the configured close
position. For contact detection and stall protection, use the MIT API demo.

```bash
python demos/07_pv_gripper.py
python demos/07_pv_gripper.py --close-vel 0.3 --hold-s 2.0
```

### 08_mit_gripper.py - MIT gripper user example

Shows the intended user-facing API: open to `1000`, then call
`move_to(0, stall_protection=True)`. With the config-created API object, that
single close command keeps holding until the user presses Ctrl+C.

```bash
python demos/08_mit_gripper.py
python demos/08_mit_gripper.py --force-level 4
python demos/08_mit_gripper.py --port /dev/cu.usbmodem00000000050C1
```

### 09_read_params.py - read motor parameters

Uses `GloriaGripper.read_param()` to read motor parameters by register name or
register ID. It does not enable the motor, switch mode, write parameters, save
flash, or move the gripper.

```bash
python demos/09_read_params.py
python demos/09_read_params.py --param CTRL_MODE --param PMAX --param VMAX --param TMAX
python demos/09_read_params.py --all
```

## License

See the [LICENSE](LICENSE) file.
