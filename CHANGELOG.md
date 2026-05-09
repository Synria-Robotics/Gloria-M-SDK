# Changelog

All notable changes to the Gloria-M SDK are documented here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).
Versions follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [1.0.3] — 2026-05-09

### Added

- **Transport abstraction** (`transport.py`): introduced the `ICanTransport`
  structural Protocol and `FakeCanAdapter` in-memory stub, enabling complete
  hardware-free unit testing of SDK logic.
- **Dependency injection hook** (`GloriaGripper._transport`): pass any
  `ICanTransport`-compatible object to `GloriaGripper(port, _transport=...)` to
  replace the serial-to-CAN adapter without modifying any other code.
- **Logging** (`logging.getLogger(__name__)` in `client.py`, `controller.py`,
  `serial_can_adapter.py`): key lifecycle events (connect, disconnect,
  enable, disable, mode switch, parameter timeouts) are now emitted at INFO /
  WARNING / DEBUG levels.  Enable with:
  ```python
  import logging
  logging.basicConfig(level=logging.INFO)
  ```
- **Test suite** (`tests/`): pytest-based tests covering protocol bit-packing
  round-trips (`test_protocol_mit.py`), baseline interpolation
  (`test_baseline.py`), and full facade integration via `FakeCanAdapter`
  (`test_client.py`).  All tests run without hardware.
- `FakeCanAdapter` exported from top-level `gloria_m_sdk` package.
- `ICanTransport` exported from top-level `gloria_m_sdk` package.

### Changed

- `demos/01_gripper_quicktest.py` and `demos/02_pv_control.py` refactored to
  use the `GloriaGripper` facade instead of directly constructing
  `SerialCanAdapter` + `CanController`. New users now see the recommended
  entry-point pattern in all examples.
- `CanController.__init__` type hint widened from `SerialCanAdapter` to
  accept any `ICanTransport`-compatible object.
- `GloriaGripper.disconnect()` now uses `hasattr` guard so custom transport
  objects that omit `close()` do not raise.
- `controller.set_zero` now emits a `WARNING`-level log to remind callers
  that the operation permanently resets the angle origin.
- `controller.read_param` now emits a `WARNING`-level log on timeout with
  the register ID, timeout value, and actuator name, so intermittent CAN
  faults are visible without raising an exception.

---

## [1.0.2] — 2026-04-20

### Added

- `GloriaModeError` — raised by `MotorAPI.set_mode` when the motor does not
  echo back the expected mode within the retry window.
- `_require_mode` guard in `BaseAPI` — `send_mit` / `send_pos_vel` now raise
  `GloriaModeError` immediately if the motor is in the wrong control mode,
  instead of silently sending an ill-formed command.
- `GloriaGripper.current_mode` property — returns the last confirmed
  `ControlMode` or `None` before any mode has been set.
- Detailed docstrings and usage examples on all public exception classes.

### Changed

- `ParamAPI.apply_limits` now calls `CanController.apply_limits_and_save`
  (previously it issued the writes directly).
- `CanController.read_param` clears the cached register value before sending
  the read request, preventing stale data from being returned.

### Fixed

- `_extract_frames` in `SerialCanAdapter`: unbounded remainder buffer growth
  when persistent garbage input arrived on the serial port. Remainder is now
  capped to at most `_RX_FRAME_LEN - 1` bytes.

---

## [1.0.1] — 2026-03-15

### Added

- `GloriaGripper` facade (`client.py`) — five-layer architecture with
  `MotorAPI`, `MotionAPI`, and `ParamAPI` sub-objects.
- `BaseAPI.connect()` guard — all sub-API methods raise
  `GloriaConnectionError` before `GloriaGripper.connect()` succeeds.
- `PositionRange.clamp()` — positions sent via `send_mit` / `send_pos_vel`
  are silently clamped to the configured safe range.
- Chinese README (`README.zh-CN.md`) with full API reference tables.

### Changed

- `apply_limits_and_save` moved from `param_config.py` into
  `CanController.apply_limits_and_save`; the old function is kept as a
  deprecated shim for backward compatibility.

---

## [1.0.0] — 2026-02-01

### Added

- Initial public release.
- MIT protocol bit-packing / unpacking (`protocol_mit.py`).
- Serial-to-CAN framing (`serial_can_adapter.py`).
- `CanController` with enable / disable / set_zero / PV / MIT commands.
- `TorqueBaseline` CSV loader and piecewise-linear interpolator.
- Demo scripts: `01_gripper_quicktest.py`, `02_pv_control.py`,
  `03_mit_linkage_force_control.py`, `mit_close_baseline.py`.
- Exception hierarchy: `GloriaSdkError`, `GloriaConnectionError`,
  `GloriaCommunicationError`, `GloriaConfigError`.
