from __future__ import annotations

from typing import Optional, TYPE_CHECKING

from .base import BaseAPI
from ..types import Limits

if TYPE_CHECKING:
    from ..client import GloriaGripper


class ParamAPI(BaseAPI):
    """Motor register read/write and parameter persistence.

    Accessed via ``gripper.params``.

    Register IDs
    ------------
    Use values from the :class:`~gloria_m_sdk.registers.Variable` enum for
    all register ID arguments.  Always cast to ``int``::

        from gloria_m_sdk import Variable

        rid = int(Variable.PMAX)   # position limit register

    Parameter persistence
    ---------------------
    Writes via :meth:`write_f32` / :meth:`write_u32` are volatile — they take
    effect immediately but are lost on power-cycle.  Call :meth:`save` to
    commit all current register values to the motor's non-volatile flash.

    Flash has a finite write endurance (~100 000 cycles).  Avoid calling
    :meth:`save` in a tight control loop; call it once at startup after
    applying configuration.

    Typical startup sequence
    ------------------------
    ::

        from gloria_m_sdk import GloriaGripper, Limits

        with GloriaGripper("COM5") as g:
            # apply_limits is called automatically in connect()
            # equivalent manual sequence:
            g.params.apply_limits(Limits(pmax=3.14, vmax=10.0, tmax=12.0))

            # read back PMAX to verify the write
            val = g.params.read(int(Variable.PMAX))
            print(f"PMAX = {val:.3f} rad")
    """

    def read(self, rid: int, *, timeout_s: float = 0.05) -> Optional[float]:
        """Read a motor register by ID and return its value.

        Sends a register-read request over CAN and blocks until a reply
        arrives or *timeout_s* seconds elapse.

        Parameters
        ----------
        rid:
            Register ID.  Use ``int(Variable.xxx)`` from
            :class:`~gloria_m_sdk.registers.Variable`.
        timeout_s:
            How long to wait for the motor's reply [s].  The default (50 ms)
            is sufficient for a healthy CAN bus.  Increase to 0.1 s if you
            see spurious ``None`` returns.

        Returns
        -------
        float
            The decoded register value, or ``None`` if no reply arrived within
            *timeout_s*.  A ``None`` return usually indicates a wiring fault
            or a wrong register ID.
        """
        return self._ctrl.read_param(self._act, rid, timeout_s=timeout_s)

    def write_f32(self, rid: int, value: float) -> None:
        """Write a float32 value to a motor register (volatile).

        The change takes effect immediately but is lost on power-cycle.
        Call :meth:`save` afterwards to persist to flash.

        Parameters
        ----------
        rid:   Register ID — use ``int(Variable.xxx)``.
        value: New register value (float32 precision).
        """
        self._ctrl.write_param_f32(self._act, rid, value)

    def write_u32(self, rid: int, value: int) -> None:
        """Write a uint32 value to a motor register (volatile).

        The change takes effect immediately but is lost on power-cycle.
        Call :meth:`save` afterwards to persist to flash.

        Parameters
        ----------
        rid:   Register ID — use ``int(Variable.xxx)``.
        value: New register value (unsigned 32-bit integer).
        """
        self._ctrl.write_param_u32(self._act, rid, value)

    def save(self) -> None:
        """Persist all current register values to non-volatile flash (0xAA command).

        This makes the current configuration survive a power cycle.

        **Flash wear warning**: flash endurance is typically ~100 000 write
        cycles.  Call this once at startup, not in a control loop.
        """
        self._ctrl.save_params(self._act)

    def apply_limits(self, limits: Limits) -> None:
        """Write PMAX, VMAX, TMAX to the motor and immediately save to flash.

        This is a convenience wrapper for the standard startup configuration
        sequence.  Equivalent to::

            g.params.write_f32(int(Variable.PMAX), limits.pmax)
            g.params.write_f32(int(Variable.VMAX), limits.vmax)
            g.params.write_f32(int(Variable.TMAX), limits.tmax)
            g.params.save()

        Parameters
        ----------
        limits:
            A :class:`~gloria_m_sdk.types.Limits` dataclass with ``pmax``
            [rad], ``vmax`` [rad/s], and ``tmax`` [N·m] fields.

        Notes
        -----
        - This is called automatically by :meth:`~gloria_m_sdk.client.GloriaGripper.connect`
          when ``apply_limits=True`` (the default).
        - You only need to call this manually when you want to change limits
          after the initial connect, or when ``apply_limits=False`` was passed
          to ``connect()``.
        """
        self._ctrl.apply_limits_and_save(self._act, limits)
