from __future__ import annotations

from typing import TYPE_CHECKING

from ..types import ControlMode

if TYPE_CHECKING:
    from ..client import GloriaGripper
    from ..controller import CanController
    from ..actuator import Actuator


class BaseAPI:
    """Shared base for all domain API classes (MotorAPI / MotionAPI / ParamAPI).

    Responsibilities
    ----------------
    - Holds a back-reference to the parent :class:`~gloria_m_sdk.client.GloriaGripper`
      facade so that all sub-APIs share the same controller and actuator objects.
    - Provides ``_ctrl`` and ``_act`` convenience properties that sub-classes use
      instead of accessing the gripper internals directly.
    - Guards against use-before-connect: ``_ctrl`` raises
      :class:`~gloria_m_sdk.exceptions.GloriaConnectionError` if
      :meth:`~gloria_m_sdk.client.GloriaGripper.connect` has not been called yet.

    Lifecycle
    ---------
    ``BaseAPI`` objects are created inside ``GloriaGripper.__init__`` and are
    therefore available immediately — but all methods that call ``_ctrl`` will
    raise until ``connect()`` succeeds.  This design lets callers keep a
    reference to ``gripper.motor`` / ``gripper.motion`` / ``gripper.params``
    without having to call ``connect()`` first::

        g = GloriaGripper("COM5")
        motor_api = g.motor          # OK — object exists
        motor_api.enable()           # raises GloriaConnectionError
        g.connect()
        motor_api.enable()           # OK — controller is now live
    """

    def __init__(self, gripper: "GloriaGripper") -> None:
        self._gripper = gripper

    @property
    def _ctrl(self) -> "CanController":
        """Return the active CanController, raising if not connected."""
        from ..exceptions import GloriaConnectionError

        ctrl = self._gripper._ctrl
        if ctrl is None:
            raise GloriaConnectionError(
                "Not connected. Call GloriaGripper.connect() first."
            )
        return ctrl

    @property
    def _act(self) -> "Actuator":
        """Return the Actuator data model associated with this gripper."""
        return self._gripper._act

    def _require_mode(self, expected: ControlMode) -> None:
        """Raise :class:`~gloria_m_sdk.exceptions.GloriaModeError` if the motor
        is not currently in *expected* control mode.

        The check is based on the mode value last confirmed by the motor via
        :meth:`~gloria_m_sdk.api.motor_api.MotorAPI.set_mode`.  If no mode has
        been confirmed yet (e.g. ``set_mode()`` was never called), the error
        message tells the caller exactly which call to make.
        """
        from ..exceptions import GloriaModeError
        from ..registers import Variable

        rid = int(Variable.CTRL_MODE)
        cached = self._act.params.get(rid)

        if cached is None:
            raise GloriaModeError(
                f"No control mode has been confirmed yet. "
                f"Call gripper.motor.set_mode(ControlMode.{expected.name}) before "
                f"sending motion commands."
            )

        if int(cached) != int(expected):
            try:
                actual_name = ControlMode(int(cached)).name
            except ValueError:
                actual_name = str(cached)
            raise GloriaModeError(
                f"Wrong control mode: motor is in {actual_name}, but this command "
                f"requires {expected.name}. "
                f"Call gripper.motor.set_mode(ControlMode.{expected.name}) first."
            )
