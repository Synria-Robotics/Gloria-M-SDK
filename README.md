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
|   |-- client.py           # Facade: GloriaGripper (recommended entry point)
|   |-- exceptions.py       # Exception hierarchy (GloriaSdkError and subclasses)
|   |-- transport.py        # ICanTransport protocol + FakeCanAdapter (testing stub)
|   |-- api/                # API layer: domain-specific sub-APIs
|   |   |-- __init__.py
|   |   |-- base.py         # BaseAPI (shared controller access)
|   |   |-- motor_api.py    # MotorAPI: enable/disable/mode/zero/poll
|   |   |-- motion_api.py   # MotionAPI: send_mit / send_pos_vel
|   |   `-- param_api.py    # ParamAPI: read/write registers, save, apply_limits
|   |-- actuator.py         # Actuator and ActuatorState data models
|   |-- controller.py       # CanController (command dispatch, feedback parsing)
|   |-- protocol_mit.py     # MIT protocol packing/unpacking
|   |-- serial_can_adapter.py  # Serial-to-CAN transport layer
|   |-- param_config.py     # Legacy helper (use ctrl.apply_limits_and_save instead)
|   |-- registers.py        # Register definitions (Variable enum)
|   |-- types.py            # Data types (Limits, ControlMode, PositionRange)
|   |-- constants.py        # Constant definitions
|   `-- gripper_baseline.py # Gripper torque baseline
|-- tests/                  # Pytest test suite (no hardware required)
|   |-- conftest.py         # Shared fixtures (FakeCanAdapter-backed gripper)
|   |-- test_protocol_mit.py# MIT bit-packing round-trip tests
|   |-- test_baseline.py    # TorqueBaseline CSV loading and interpolation tests
|   `-- test_client.py      # GloriaGripper facade integration tests
|-- demos/                  # Example scripts
|   |-- 01_gripper_quicktest.py  # PV mode reciprocating cycle test
|   |-- 02_pv_control.py        # PV mode gentle close
|   |-- 03_mit_linkage_force_control.py  # MIT linkage gripper force control
|   |-- mit_close_baseline.py   # MIT no-load close baseline capture
|   `-- baseline/               # Baseline data CSV output
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

The SDK uses a strict five-layer architecture. Upper-computer applications interact exclusively with the **Facade** and **API** layers.

```
User code
    │
    ▼
┌─────────────────────────────────────────┐
│  Facade    GloriaGripper  (client.py)   │  ← Recommended entry point
│  .motor / .motion / .params             │
│  .state / .current_mode / .is_connected │
└──────────────────┬──────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────┐
│  API layer  MotorAPI / MotionAPI        │  api/
│             ParamAPI                    │
└──────────────────┬──────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────┐
│  Controller  CanController              │  controller.py
│  command dispatch · feedback parsing    │
└──────────────────┬──────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────┐
│  Protocol   protocol_mit.py             │
│  MIT bit-packing · float32 codec        │
└──────────────────┬──────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────┐
│  Transport  SerialCanAdapter            │  serial_can_adapter.py
│  serial framing · raw TX/RX             │
└─────────────────────────────────────────┘

Cross-cutting (any layer may import):
  exceptions.py  — GloriaSdkError hierarchy
  types.py       — Limits, ControlMode, PositionRange
  actuator.py    — Actuator, ActuatorState
  registers.py   — Variable (RID enum)
  gripper_baseline.py — TorqueBaseline
```

## Quick Start

```python
from gloria_m_sdk import GloriaGripper, ControlMode

with GloriaGripper("COM5") as g:  # replace COM5 with your actual port
    g.motor.set_mode(ControlMode.POS_VEL)
    g.motor.enable()
    g.motor.refresh()
    print(f"position = {g.state.position:.3f} rad")

    # Move to open position
    g.motion.send_pos_vel(position=2.5, velocity=1.0)
```

### GloriaGripper constructor

```python
GloriaGripper(
    port,                    # e.g. "COM5" or "/dev/ttyUSB0"; use 'auto' to auto-detect
    *,
    baudrate=921_600,
    command_id=0x01,         # CAN ID for commands
    feedback_id=0x101,       # CAN ID for feedback
    limits=None,             # Limits(pmax, vmax, tmax); defaults to (3.14, 10, 12)
    safe_position=None,      # PositionRange(min, max) — position clamp
    baseline_csv=None,       # path to no-load torque baseline CSV
    timeout=0.5,             # serial read timeout [s]
    _transport=None,         # testing hook: inject a FakeCanAdapter instead of opening a port
)
```

### GloriaGripper properties

| Property | Type | Description |
|----------|------|-------------|
| `state` | `ActuatorState` | Latest feedback snapshot (position, velocity, torque) |
| `current_mode` | `ControlMode \| None` | Control mode last confirmed by the motor; `None` before `set_mode()` |
| `is_connected` | `bool` | `True` if the serial port is open |

### GloriaGripper.motor — MotorAPI

| Method | Description |
|--------|-------------|
| `enable()` | Send enable command |
| `disable()` | Send disable command |
| `set_zero()` | Set current position as zero |
| `set_mode(mode)` | Switch control mode; raises `GloriaModeError` on failure |
| `refresh()` | Request state via broadcast and update `gripper.state` |
| `poll()` | Parse pending RX packets, update state |

### GloriaGripper.motion — MotionAPI

| Method | Description |
|--------|-------------|
| `send_mit(*, kp, kd, q, dq, tau)` | MIT torque-control frame |
| `send_pos_vel(*, position, velocity)` | PV position + velocity frame |

### GloriaGripper.params — ParamAPI

| Method | Description |
|--------|-------------|
| `read(rid, *, timeout_s)` | Read register; returns `float` or `None` |
| `write_f32(rid, value)` | Write float32 register |
| `write_u32(rid, value)` | Write uint32 register |
| `save()` | Persist parameters to flash |
| `apply_limits(limits)` | Write PMAX/VMAX/TMAX and save |

### Exception hierarchy

```python
GloriaSdkError          # base — catch-all
├── GloriaConnectionError   # serial port cannot be opened
├── GloriaCommunicationError# timeout / malformed packet
├── GloriaConfigError       # invalid parameter value
└── GloriaModeError         # mode switch not confirmed
```

### Low-level access (power users)

The underlying `CanController` and `SerialCanAdapter` are still exported and
used directly in the demo scripts.

| Symbol | Description |
|--------|-------------|
| `CanController` | Direct motor command / feedback parsing |
| `SerialCanAdapter` | Raw serial-to-CAN transport |
| `ICanTransport` | Structural protocol for custom transport backends |
| `FakeCanAdapter` | In-memory transport stub for hardware-free testing |
| `Variable` | Register ID enum (RID) |
| `TorqueBaseline` | No-load torque baseline for force estimation |

## Testing without hardware

`FakeCanAdapter` is an in-memory drop-in for `SerialCanAdapter`. Inject it
via the `_transport` parameter to run the full SDK logic without a physical
motor or serial port:

```python
from gloria_m_sdk import FakeCanAdapter, GloriaGripper, ControlMode
from gloria_m_sdk.registers import Variable

fake = FakeCanAdapter()
# Simulate the motor echoing CTRL_MODE = 2 (POS_VEL) after set_mode()
fake.queue_param_reply(can_id=0x101, rid=int(Variable.CTRL_MODE),
                       value=int(ControlMode.POS_VEL), is_u32=True)

with GloriaGripper("unused", _transport=fake) as g:
    g.motor.set_mode(ControlMode.POS_VEL)
    assert g.current_mode == ControlMode.POS_VEL
```

Run the built-in test suite (60 tests, ~1 s, no hardware):

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

### 01_gripper_quicktest.py - PV mode reciprocating cycle test

The gripper repeatedly moves between open and close positions to quickly verify that the PV control mode is working correctly.

```bash
python demos/01_gripper_quicktest.py --port auto --id 0x01 --close-q 0.0 --open-q 2.5 --vel 1.0
```

### 02_pv_control.py - PV mode gentle close

Opens to position 2.5, then closes to position 0 at a low speed in PV mode. Suitable for gently gripping delicate objects.

```bash
python demos/02_pv_control.py --port auto --open-q 2.5 --close-q 0.0 --close-vel 0.3
```

### 03_mit_linkage_force_control.py - MIT linkage gripper force control

Approach-contact-hold-release cycle using MIT torque control with a configurable moment-arm profile for accurate fingertip force estimation.

```bash
python demos/03_mit_linkage_force_control.py --port auto --open-q 2.77 --close-q 0.003 --target-force 15
```

For the 4340 high-force gripper version:

```bash
python demos/03_mit_linkage_force_control.py --port auto --baseline-csv ".\demos\baseline\close_baseline_4340.csv" --target-force 30 --contact-force 60
```

**MIT control formula:**

$$\tau_{out} = k_p \cdot (q_{target} - q_{fb}) + k_d \cdot (dq_{target} - dq_{fb}) + \tau_{ff}$$

### mit_close_baseline.py - MIT no-load close baseline capture

This script closes the gripper with a fixed negative torque in MIT mode while no object is being held. It records position, velocity, feedback torque, and estimated gripping force. The generated binned baseline CSV can be used as `--baseline-csv` for `03_mit_linkage_force_control.py` to compensate for no-load friction and linkage resistance.

Run it with the gripper unloaded:

```bash
python demos/mit_close_baseline.py --port auto --close-tau -1.25
```

Common options:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--port` | auto | Serial port |
| `--baud` | 921600 | Serial baud rate |
| `--id` | 0x01 | Motor command CAN ID |
| `--fb-id` | 0x201 | Motor feedback CAN ID |
| `--open-q` | 2.77 | Fully open gripper position [rad] |
| `--close-q` | 0.003 | Closed gripper position [rad] |
| `--close-tau` | -1.25 | Closing torque [N·m]; must be negative |
| `--kd` | 0.8 | MIT damping term |
| `--stop-force` | 0.0 | Stop when estimated force reaches this threshold; 0 disables it [N] |
| `--radius-mm` | 12.0 | Effective moment arm used for force estimation [mm] |
| `--timeout` | 5.0 | Maximum capture time [s] |
| `--position-epsilon` | 0.02 | Closed-position tolerance [rad] |
| `--bin-width` | 0.05 | Position bin width for the baseline curve [rad] |
| `--save-dir` | demos/baseline | CSV output directory |
| `--save-prefix` | close_baseline | CSV filename prefix |
| `--no-save` | false | Run the test without saving CSV files |

By default, the script writes two CSV files:

```text
demos/baseline/{save_prefix}_{timestamp}_raw.csv
demos/baseline/{save_prefix}_{timestamp}_binned.csv
```

The raw CSV contains one row per control-loop sample:

| Field | Description |
|-------|-------------|
| `elapsed_s` | Time since the start of the test [s] |
| `position_rad` | Motor position feedback [rad] |
| `velocity_rad_s` | Motor velocity feedback [rad/s] |
| `tau_cmd_nm` | MIT torque command sent to the motor [N·m] |
| `tau_fb_nm` | Motor feedback torque [N·m] |
| `force_est_n` | Estimated gripping force [N], calculated as `max(0, -tau_fb_nm) / (radius_mm / 1000)` |

The binned baseline CSV groups raw samples with nearby positions and averages each group for use as a baseline curve:

| Field | Description |
|-------|-------------|
| `position_mean_rad` | Mean position within this position bin [rad] |
| `velocity_mean_rad_s` | Mean velocity within this position bin [rad/s] |
| `tau_fb_mean_nm` | Mean feedback torque within this position bin [N·m] |
| `force_est_mean_n` | Mean estimated gripping force within this position bin [N] |
| `sample_count` | Number of raw samples in this position bin |

## Common Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--port` | auto | Serial port |
| `--baud` | 921600 | Serial baud rate |
| `--id` | 0x01 | Motor command CAN ID |
| `--fb-id` | 0x101 | Motor feedback CAN ID |
| `--open-q` | 2.5 | Open position [rad] |
| `--close-q` | 0.0 | Close position [rad] |
| `--baseline-csv` | ".\\demos\\baseline\\close_baseline_4310.csv" | No-load baseline file for the gripper. Defaults to the 4310 profile; use `close_baseline_4340.csv` manually for the 4340 version |
| `--target-force` | 15 | Target gripping force [N] |
| `--contact-force` | 10 | Contact detection force threshold [N]; use 60 for the 4340 gripper version |

## License

See the [LICENSE](LICENSE) file.
