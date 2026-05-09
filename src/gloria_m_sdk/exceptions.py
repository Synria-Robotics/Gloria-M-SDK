from __future__ import annotations


class GloriaSdkError(Exception):
    """Base exception for all Gloria-M SDK errors.

    Catch this class to handle any SDK-level error in a single except block::

        from gloria_m_sdk import GloriaGripper, GloriaSdkError

        try:
            with GloriaGripper("COM5") as g:
                g.motor.enable()
        except GloriaSdkError as e:
            print(f"SDK error: {e}")

    Sub-classes provide finer-grained error categories for more specific
    handling when needed.
    """


class GloriaConnectionError(GloriaSdkError):
    """Raised when the serial port cannot be opened or is lost mid-session.

    Common causes:
    - Wrong port name (e.g. ``"COM5"`` when the adapter is on ``"COM3"``).
    - Serial adapter not plugged in, or driver not installed.
    - Port already held open by another process (e.g. a terminal program).
    - The USB cable was unplugged while the session was active.

    How to handle::

        from gloria_m_sdk.exceptions import GloriaConnectionError

        try:
            gripper.connect()
        except GloriaConnectionError as e:
            print(f"Check cable and port: {e}")
    """


class GloriaCommunicationError(GloriaSdkError):
    """Raised when a CAN frame is malformed or a reply times out.

    Common causes:
    - CAN bus wiring fault (wrong pins, missing termination resistor).
    - Motor powered off while the host is sending commands.
    - Baud rate mismatch between adapter and motor firmware.
    - Serial buffer overflow due to an overly tight control loop.

    This exception is currently reserved for future use; the controller layer
    returns ``None`` or ``False`` on timeout rather than raising.  Catching it
    is still recommended for forward compatibility.
    """


class GloriaConfigError(GloriaSdkError):
    """Raised when a configuration value is invalid or out of the motor's range.

    Common causes:
    - ``Limits.pmax`` / ``vmax`` / ``tmax`` exceed the hardware specification.
    - A register ID passed to ``ParamAPI.read`` or ``write_f32`` does not exist.

    Check the :class:`~gloria_m_sdk.registers.Variable` enum for the list of
    valid register IDs.
    """


class GloriaModeError(GloriaSdkError):
    """Raised when the motor does not confirm a control-mode switch.

    Raised by :meth:`~gloria_m_sdk.api.motor_api.MotorAPI.set_mode` when the
    motor fails to echo back the expected mode within the retry window.

    Common causes:
    - Motor is not yet powered or still booting.
    - CAN bus fault (bad wiring, missing 120 Ω termination).
    - Wrong CAN ID — the command is not reaching the motor.

    Typical fix: check CAN wiring, confirm 24 V power is stable, then retry.
    """
