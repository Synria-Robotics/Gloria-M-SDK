from __future__ import annotations

"""
GloriaGripper — facade entry point for the Gloria-M gripper SDK.

Architecture (mirrors the Electronic-Skin-ML tactile SDK pattern):

    User code
        │
        ▼
    ┌─────────────────────────────────────┐
    │  Facade   GloriaGripper            │  client.py
    │  .motor / .motion / .params        │
    └────────────────┬────────────────────┘
                     │
                     ▼
    ┌─────────────────────────────────────┐
    │  API layer  MotorAPI / MotionAPI   │  api/
    │             ParamAPI               │
    └────────────────┬────────────────────┘
                     │
                     ▼
    ┌─────────────────────────────────────┐
    │  Controller  CanController          │  controller.py
    │  (command dispatch, feedback parse) │
    └────────────────┬────────────────────┘
                     │
                     ▼
    ┌─────────────────────────────────────┐
    │  Protocol   protocol_mit.py         │
    │  MIT bit-packing, float32 codec     │
    └────────────────┬────────────────────┘
                     │
                     ▼
    ┌─────────────────────────────────────┐
    │  Transport  SerialCanAdapter        │  serial_can_adapter.py
    │  serial framing, raw TX/RX          │
    └─────────────────────────────────────┘

    Cross-cutting (any layer may import):
      exceptions.py  — GloriaSdkError hierarchy
      types.py       — Limits, ControlMode, PositionRange
      actuator.py    — Actuator, ActuatorState
      registers.py   — Variable (RID enum)
      gripper_baseline.py — TorqueBaseline
"""

from typing import Optional

from .actuator import Actuator, ActuatorState
from .api import MotorAPI, MotionAPI, ParamAPI
from .controller import CanController
from .gripper_baseline import TorqueBaseline
from .serial_can_adapter import SerialCanAdapter
from .types import ControlMode, Limits, PositionRange  # noqa: F401 — re-exported for users

import logging
_log = logging.getLogger(__name__)

_DEFAULT_LIMITS = Limits(pmax=3.14, vmax=10.0, tmax=12.0)


class GloriaGripper:
    """Facade entry point for the Gloria-M gripper SDK.

    All motor interaction goes through three domain sub-APIs:

    - :attr:`motor`  — enable / disable / set_zero / set_mode / poll
    - :attr:`motion` — send_mit / send_pos_vel
    - :attr:`params` — read / write_f32 / write_u32 / save / apply_limits

    The current actuator state is available via :attr:`state`.

    Quick start::

        from gloria_m_sdk import GloriaGripper, ControlMode

        with GloriaGripper("COM5") as g:
            g.motor.set_mode(ControlMode.POS_VEL)
            g.motor.enable()
            g.motor.refresh()
            print(g.state.position)

            g.motion.send_pos_vel(position=2.5, velocity=1.0)

    Parameters
    ----------
    port:
        Serial port name, e.g. ``"COM5"`` or ``"/dev/ttyUSB0"``.
    baudrate:
        Serial baud rate (default 921 600).
    command_id:
        CAN ID for commands sent to the motor (default ``0x01``).
    feedback_id:
        CAN ID of feedback frames from the motor (default ``0x101``).
    limits:
        MIT scaling limits (PMAX / VMAX / TMAX).  Defaults to
        ``Limits(pmax=3.14, vmax=10.0, tmax=12.0)``.
    safe_position:
        Optional position clamp applied to every command.
    baseline_csv:
        Path to a no-load torque baseline CSV.  When provided the baseline is
        loaded at construction time and available as :attr:`baseline`.
    timeout:
        Serial read timeout in seconds (default 0.5).
    """

    def __init__(
        self,
        port: str,
        *,
        baudrate: int = 921_600,
        command_id: int = 0x01,
        feedback_id: int = 0x101,
        limits: Optional[Limits] = None,
        safe_position: Optional[PositionRange] = None,
        baseline_csv: Optional[str] = None,
        timeout: float = 0.5,
        _transport: Optional[object] = None,
    ) -> None:
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._limits = limits if limits is not None else _DEFAULT_LIMITS
        self._transport_override = _transport  # testing hook — not part of public API

        self._act = Actuator(
            name="gripper",
            command_id=command_id,
            feedback_id=feedback_id,
            limits=self._limits,
            safe_position=safe_position,
        )

        self._adapter: Optional[object] = None
        self._ctrl: Optional[CanController] = None

        # Optional torque baseline
        self.baseline: Optional[TorqueBaseline] = None
        if baseline_csv:
            self.baseline = TorqueBaseline.from_csv(baseline_csv)

        # Domain API objects — usable after connect()
        self.motor = MotorAPI(self)
        self.motion = MotionAPI(self)
        self.params = ParamAPI(self)

    # ------------------------------------------------------------------
    # Connection management

    def connect(self, *, apply_limits: bool = True) -> None:
        """Open the serial port and prepare the motor.

        Parameters
        ----------
        apply_limits:
            When True (default) write PMAX / VMAX / TMAX to the motor and
            save to flash immediately after connecting.

        Raises
        ------
        GloriaConnectionError
            If the serial port cannot be opened.
        """
        import serial as _serial

        from .exceptions import GloriaConnectionError

        if self._adapter is not None:
            return  # already connected

        if self._transport_override is not None:
            _log.debug("Using injected transport (hardware-free / test mode)")
            self._adapter = self._transport_override
        else:
            _log.info("Opening serial port %r at %d baud", self._port, self._baudrate)
            try:
                self._adapter = SerialCanAdapter(
                    self._port, baudrate=self._baudrate, timeout=self._timeout
                )
            except _serial.SerialException as exc:
                self._adapter = None
                raise GloriaConnectionError(
                    f"Cannot open port {self._port!r}: {exc}"
                ) from exc

        self._ctrl = CanController(self._adapter)
        self._ctrl.register(self._act)

        if apply_limits:
            _log.info("Applying limits: pmax=%.3f vmax=%.3f tmax=%.3f",
                      self._limits.pmax, self._limits.vmax, self._limits.tmax)
            self._ctrl.apply_limits_and_save(self._act, self._limits)

        _log.info("GloriaGripper connected (port=%r, cmd_id=0x%03X, fb_id=0x%03X)",
                  self._port, self._act.command_id, self._act.feedback_id)

    def disconnect(self) -> None:
        """Close the serial port and release all resources.

        After this call :attr:`is_connected` returns ``False`` and all
        sub-API method calls will raise
        :class:`~gloria_m_sdk.exceptions.GloriaConnectionError`.

        .. note::
            ``disconnect()`` does **not** send a disable command to the motor.
            If the motor is currently energized, it will continue to hold its
            last commanded position until the firmware watchdog times out and
            cuts power.  For a clean shutdown, always call
            ``gripper.motor.disable()`` before disconnecting.
        """
        if self._adapter is not None:
            _log.info("Disconnecting GloriaGripper (port=%r)", self._port)
            if hasattr(self._adapter, "close"):
                self._adapter.close()  # type: ignore[union-attr]
            self._adapter = None
        self._ctrl = None

    @property
    def is_connected(self) -> bool:
        """``True`` if the serial port is currently open and ready.

        This checks the underlying serial port's ``is_open`` flag.  It does
        **not** send a ping to the motor — a ``True`` return only means the
        host-side port is open, not that the motor is alive on the bus.

        Use :meth:`motor.refresh` to confirm the motor is responding.
        """
        return (
            self._adapter is not None
            and self._adapter.is_open
        )

    def __enter__(self) -> "GloriaGripper":
        """Connect on entry; used with the ``with`` statement."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """Disconnect on exit (even if an exception was raised).

        For a clean shutdown, disable the motor before the ``with`` block exits::

            with GloriaGripper("COM5") as g:
                g.motor.enable()
                try:
                    g.motion.send_pos_vel(position=2.5, velocity=1.0)
                finally:
                    g.motor.disable()  # de-energize before disconnect
        """
        self.disconnect()

    # ------------------------------------------------------------------
    # State shortcut

    @property
    def current_mode(self) -> Optional[ControlMode]:
        """Control mode last confirmed by the motor, or ``None`` if not set.

        This reflects the mode echoed back by the motor after a successful
        :meth:`motor.set_mode` call.  It returns ``None`` before any mode has
        been confirmed.

        Use this to inspect the active mode without sending any CAN traffic::

            print(g.current_mode)           # ControlMode.POS_VEL or None
            assert g.current_mode == ControlMode.MIT
        """
        from .registers import Variable

        rid = int(Variable.CTRL_MODE)
        cached = self._act.params.get(rid)
        if cached is None:
            return None
        try:
            return ControlMode(int(cached))
        except ValueError:
            return None

    @property
    def state(self) -> ActuatorState:
        """Latest actuator state snapshot (position, velocity, torque).

        Fields
        ------
        position : float
            Motor shaft angle [rad], relative to the zero set by
            :meth:`motor.set_zero`.
        velocity : float
            Motor shaft angular velocity [rad/s].
        torque : float
            Estimated motor output torque [N·m].

        The state is updated whenever a feedback frame is parsed.  This
        happens automatically after any command sent with ``poll=True``
        (the default), or explicitly by calling :meth:`motor.refresh` or
        :meth:`motor.poll`.

        The returned :class:`~gloria_m_sdk.actuator.ActuatorState` is a
        snapshot; successive reads may return different values if a new
        feedback packet has arrived in the meantime.
        """
        return self._act.state
