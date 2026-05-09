from __future__ import annotations

from typing import TYPE_CHECKING

from .base import BaseAPI
from ..types import ControlMode

if TYPE_CHECKING:
    from ..client import GloriaGripper


class MotorAPI(BaseAPI):
    """Motor lifecycle, mode and state commands.

    Accessed via ``gripper.motor``.

    Typical startup sequence
    ------------------------
    ::

        g.motor.set_mode(ControlMode.POS_VEL)   # 1. pick a control mode
        g.motor.enable()                          # 2. energize the motor
        g.motor.refresh()                         # 3. read initial state
        print(g.state.position)                   # 4. use the state

    Notes
    -----
    - ``set_mode()`` must be called before ``enable()``.  Calling ``enable()``
      without a prior ``set_mode()`` leaves the motor in whatever mode it was
      last saved to flash.
    - All methods require an active connection; they raise
      :class:`~gloria_m_sdk.exceptions.GloriaConnectionError` otherwise.
    """

    def enable(self) -> None:
        """Energize the motor and enter the active control state.

        Sends the enable command over CAN and waits for the motor to echo back
        a live feedback frame.  After this call the motor will actively resist
        disturbances according to the current control mode and parameters.

        Call this **after** :meth:`set_mode` and before any motion command.
        Calling enable on an already-enabled motor is safe (no-op on the
        motor side).
        """
        self._ctrl.enable(self._act)

    def disable(self) -> None:
        """De-energize the motor — it goes limp immediately.

        Sends the disable command.  The motor stops applying torque and the
        gripper will open or close under gravity / spring force.

        Always call this in a ``finally`` block so the motor is de-energized
        even if an exception occurs::

            try:
                g.motor.enable()
                g.motion.send_pos_vel(position=2.5, velocity=1.0)
            finally:
                g.motor.disable()
        """
        self._ctrl.disable(self._act)

    def set_zero(self) -> None:
        """Set the current motor position as the new zero reference (in-place).

        **Warning**: this permanently changes the motor's angle origin.  All
        subsequent position readings and commands will be relative to the new
        zero.  This is normally used once during mechanical assembly to align
        the encoder index with the mechanical home position.

        The new zero is stored in the motor's volatile registers; call
        ``gripper.params.save()`` afterwards if you want it to survive a
        power cycle.
        """
        self._ctrl.set_zero(self._act)

    def set_mode(self, mode: ControlMode) -> None:
        """Switch the motor's control mode and wait for confirmation.

        Parameters
        ----------
        mode:
            The desired control mode.  Common values:

            - ``ControlMode.MIT``     — MIT torque/impedance control (kp, kd, tau).
              Used for force-controlled gripping.
            - ``ControlMode.POS_VEL`` — Position + velocity target (PV mode).
              Used for smooth positional moves.

        Raises
        ------
        GloriaModeError
            If the motor does not echo back the expected mode within the retry
            window (typically 3 attempts, ~150 ms total).  Check CAN wiring
            and motor power if this is raised unexpectedly.

        Notes
        -----
        - Mode switches take ~50–100 ms because the controller retries and
          waits for a feedback frame with the new mode field.
        - Always call ``set_mode()`` before ``enable()`` when starting a
          session, to ensure the motor is in a known state.
        """
        from ..exceptions import GloriaModeError

        ok = self._ctrl.set_control_mode(self._act, mode)
        if not ok:
            raise GloriaModeError(
                f"Motor did not confirm mode switch to {mode.name}. "
                "Check CAN wiring and motor power."
            )

    def refresh(self) -> None:
        """Broadcast a state-request frame and update :attr:`GloriaGripper.state`.

        Sends a dedicated "read state" CAN frame and blocks until a feedback
        packet is received (or the serial timeout expires).  This is the
        preferred way to obtain an up-to-date position / velocity / torque
        reading **without** moving the motor.

        Use :meth:`poll` instead when the motor is already receiving periodic
        motion commands and you just want to drain the RX buffer.
        """
        self._ctrl.refresh_state(self._act)

    def poll(self) -> None:
        """Parse all pending RX packets and update the actuator state.

        Does not send any new command — it only reads whatever feedback frames
        have already arrived in the serial buffer.

        Use this in a tight loop alongside motion commands when you want the
        latest state without the overhead of an extra request frame::

            while running:
                g.motion.send_pos_vel(position=target, velocity=1.0, poll=False)
                g.motor.poll()          # drain RX, update g.state
                print(g.state.position)

        For an initial state read (before any commands), use :meth:`refresh`
        instead, which sends a request frame and guarantees a response.
        """
        self._ctrl.poll()
