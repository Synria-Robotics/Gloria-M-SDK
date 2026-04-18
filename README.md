# Gloria-M SDK

> Python SDK for serial-to-CAN motor control, targeting the Gloria-M series gripper actuators.

Copyright (c) 2026 Synria Robotics Co., Ltd.  
Website: https://synriarobotics.ai

## Features

- Communicates with Gloria-M series motors through a serial-to-CAN adapter
- Supports **MIT mode** (kp/kd/τ torque control) and **PV mode** (position + velocity)
- Provides parameter read/write and enable/disable primitives
- Built-in MIT protocol packing/unpacking and feedback state parsing

## Project Structure

```
Gloria-M-SDK-1.0.3/
├── src/gloria_m_sdk/       # SDK core library
│   ├── __init__.py         # Package entry point; exports public API
│   ├── actuator.py         # Actuator abstraction
│   ├── controller.py       # High-level controller (command dispatch, feedback parsing)
│   ├── protocol_mit.py     # MIT protocol packing/unpacking
│   ├── serial_can_adapter.py  # Serial-to-CAN adapter
│   ├── param_config.py     # Parameter write and save
│   ├── registers.py        # Register definitions (RID enum)
│   ├── types.py            # Data types (Limits, ControlMode, etc.)
│   ├── constants.py        # Constant definitions
│   └── gripper_baseline.py # Gripper torque baseline
├── demos/                  # Example scripts
│   ├── 01_gripper_quicktest.py  # PV mode reciprocating cycle test
│   ├── 02_pv_control.py        # PV mode gentle close
│   ├── 03_mit_linkage_force_control.py  # MIT linkage gripper force control
│   └── output/                 # Baseline data CSV output
├── pyproject.toml
├── requirements.txt
└── README.md
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

## Demos

### 01_gripper_quicktest.py — PV mode reciprocating cycle test

The gripper repeatedly moves between open and close positions to quickly verify that the PV control mode is working correctly.

```bash
python demos/01_gripper_quicktest.py --port COM5 --id 0x01 --close-q 0.0 --open-q 2.5 --velocity 1.0
```

### 02_pv_control.py — PV mode gentle close

Opens to position 2.5, then closes to position 0 at a low speed in PV mode. Suitable for gently gripping delicate objects.

```bash
python demos/02_pv_control.py --port COM5 --open-q 2.5 --close-q 0.0 --close-vel 0.3
```

### 03_mit_linkage_force_control.py — MIT linkage gripper force control

Approach-contact-hold-release cycle using MIT torque control with a configurable moment-arm profile for accurate fingertip force estimation.

```bash
python demos/03_mit_linkage_force_control.py --port COM5 --open-q 2.77 --close-q 0.003 --target-force 15
```

**MIT control formula:**

$$\tau_{out} = k_p \cdot (q_{target} - q_{fb}) + k_d \cdot (dq_{target} - dq_{fb}) + \tau_{ff}$$

## Common Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--port` | COM5 | Serial port |
| `--baud` | 921600 | Serial baud rate |
| `--id` | 0x01 | Motor command CAN ID |
| `--fb-id` | 0x101 | Motor feedback CAN ID |
| `--open-q` | 2.5 | Open position [rad] |
| `--close-q` | 0.0 | Close position [rad] |

## License

See the [LICENSE](LICENSE) file.
