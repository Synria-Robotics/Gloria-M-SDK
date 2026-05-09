from __future__ import annotations

from typing import TYPE_CHECKING

from .base import BaseAPI
from ..types import ControlMode

if TYPE_CHECKING:
    from ..client import GloriaGripper


class MotionAPI(BaseAPI):
    """Low-level motion commands: MIT torque/impedance control and PV position-velocity.

    Accessed via ``gripper.motion``.

    Choosing a control mode
    -----------------------
    **PV (position-velocity)** — ``send_pos_vel``

    Easiest to use.  Send a target position and a speed limit; the motor
    firmware handles trajectory planning internally.  Ideal for pick-and-place
    moves where you want the gripper to reach a specific angle.

    **MIT (torque/impedance)** — ``send_mit``

    Gives direct control over the motor's torque output.  Each frame specifies
    a spring-damper impedance law plus a torque feedforward::

        output_torque = kp*(q_target - q) + kd*(dq_target - dq) + tau

    Setting ``kp=0, kd=0.8`` and a constant ``tau`` approximates pure torque
    control with velocity damping, which is useful for contact and gripping tasks.

    Both modes require a prior ``gripper.motor.set_mode(...)`` call.

    Notes
    -----
    - All methods raise :class:`~gloria_m_sdk.exceptions.GloriaConnectionError`
      if the gripper is not connected.
    - ``poll=True`` (default) tells the controller to read back a feedback frame
      immediately after each command.  Set ``poll=False`` only when you need
      maximum command throughput and update state via a separate ``gripper.motor.poll()``.
    """

    def send_mit(
        self,
        *,
        kp: float,
        kd: float,
        q: float,
        dq: float,
        tau: float,
        poll: bool = True,
    ) -> None:
        """Send one MIT torque/impedance control frame.

        The motor computes its output torque each cycle as::

            output = kp * (q - q_feedback) + kd * (dq - dq_feedback) + tau

        Parameters
        ----------
        kp:
            Position stiffness [N·m/rad].  Range ``[0, 500]``.
            Set to ``0`` for pure torque control; increase to add spring-like
            position tracking on top of the torque feedforward.
        kd:
            Velocity damping [N·m·s/rad].  Range ``[0, 5]``.
            A value of ``0.5``–``1.0`` provides smooth damping during closing.
            Higher values increase resistance to fast motion.
        q:
            Target position [rad].  Acts as the spring equilibrium when ``kp > 0``.
            Ignored (has no effect) when ``kp = 0``.
        dq:
            Target velocity [rad/s].  Acts as the damper setpoint.
            Set to ``0`` to damp out velocity (pure braking behaviour).
        tau:
            Torque feedforward [N·m].  Positive = opening direction,
            negative = closing direction.  This is the dominant term for
            force-controlled gripping (e.g. ``tau = -1.5`` for a 1.5 N·m
            closing torque).
        poll:
            If ``True`` (default), parse feedback immediately after sending.
            The updated state is then available via ``gripper.state``.

        Example — pure torque close with damping::

            g.motion.send_mit(kp=0.0, kd=0.8, q=0.0, dq=0.0, tau=-1.5)

        Example — spring-damper position hold (stiff)::

            g.motion.send_mit(kp=100.0, kd=1.0, q=target_rad, dq=0.0, tau=0.0)
        """
        ctrl = self._ctrl  # raises GloriaConnectionError if not connected
        self._require_mode(ControlMode.MIT)
        ctrl.send_mit(
            self._act, kp=kp, kd=kd, q=q, dq=dq, tau=tau, poll=poll
        )

    def send_pos_vel(
        self, *, position: float, velocity: float, poll: bool = True
    ) -> None:
        """Send one PV (position + velocity) frame.

        The motor firmware plans a trajectory to reach *position* at up to
        *velocity* rad/s.  Sending this command repeatedly (at ~100–500 Hz)
        with an updated position implements smooth streaming motion.

        Parameters
        ----------
        position:
            Target position [rad].  The value is clamped to the
            ``safe_position`` range configured in ``GloriaGripper`` before
            transmission, so over-range values are safe to send.
        velocity:
            Movement speed limit [rad/s].  The motor will not exceed this speed
            while tracking the target.  Pass ``0.0`` to hold the current
            position in place (the motor becomes a stiff position controller).
        poll:
            If ``True`` (default), parse feedback immediately after sending.
            The updated state is then available via ``gripper.state``.

        Example — move and hold::

            g.motion.send_pos_vel(position=2.5, velocity=1.0)  # move to 2.5 rad
            g.motion.send_pos_vel(position=2.5, velocity=0.0)  # hold at 2.5 rad
        """
        ctrl = self._ctrl  # raises GloriaConnectionError if not connected
        self._require_mode(ControlMode.POS_VEL)
        ctrl.send_pos_vel(
            self._act, position=position, velocity=velocity, poll=poll
        )
