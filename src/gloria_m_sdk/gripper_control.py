from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import time
from typing import Callable, Optional

from .client import MotorClient


class GripperControlState(str, Enum):
    IDLE = "IDLE"
    OPENING = "OPENING"
    CLOSE_SEARCH = "CLOSE_SEARCH"
    CONTACT_DETECTED = "CONTACT_DETECTED"
    HOLDING = "HOLDING"
    RELEASING = "RELEASING"
    ERROR = "ERROR"


@dataclass(frozen=True)
class MotorHealth:
    """Optional safety data from hardware-specific integrations.

    The current SDK feedback contains position, velocity, and torque. Current,
    temperature, and error code are backend/firmware dependent, so callers can
    inject them through ``GripperController(health_reader=...)`` when available.
    """

    current_a: Optional[float] = None
    temperature_c: Optional[float] = None
    error_code: Optional[int] = None


@dataclass(frozen=True)
class GripperControlConfig:
    """Tunable parameters for MIT-mode gripper control.

    ``open_pos`` and ``close_limit`` define the mechanical travel. The closing
    direction is inferred from them, so both increasing-angle and decreasing-
    angle grippers are supported.
    """

    open_pos: float
    close_limit: float

    # Opening/releasing: target is reachable, so normal MIT position control is OK.
    open_kp: float = 80.0
    open_kd: float = 1.0
    open_tau_ff: float = 0.0
    open_position_tolerance: float = 0.03
    open_velocity_tolerance: float = 0.05
    open_timeout_s: float = 5.0
    release_timeout_s: float = 5.0

    # Closing search: soft motion. Do not chase close_limit with high stiffness.
    search_kp: float = 0.0
    search_kd: float = 0.8
    search_tau_ff: float = 0.0
    close_speed: float = 0.25
    search_step: float = 0.03
    close_limit_tolerance: float = 0.02
    close_timeout_s: float = 5.0

    # Contact detection: low velocity + raised torque, sustained briefly.
    contact_velocity_threshold: float = 0.05
    contact_torque_threshold: float = 0.35
    contact_confirm_s: float = 0.15

    # Holding: target is contact_pos, not close_limit.
    hold_kp: float = 8.0
    hold_kd: float = 0.6
    hold_tau_ff: float = 0.0

    # Safety: thresholds are conservative defaults; tune per mechanism.
    max_abs_torque: Optional[float] = 6.0
    max_abs_current: Optional[float] = None
    max_temperature_c: Optional[float] = None
    stall_velocity_threshold: float = 0.03
    stall_torque_threshold: float = 3.0
    stall_timeout_s: float = 0.3
    position_margin: float = 0.1

    # Neutral command sent after entering ERROR to avoid continued closing force.
    error_kd: float = 0.8

    def __post_init__(self) -> None:
        if self.open_pos == self.close_limit:
            raise ValueError("open_pos and close_limit must differ")
        if self.close_speed <= 0:
            raise ValueError("close_speed must be positive")
        if self.search_step <= 0:
            raise ValueError("search_step must be positive")
        if self.contact_confirm_s < 0:
            raise ValueError("contact_confirm_s must be non-negative")
        if self.stall_timeout_s < 0:
            raise ValueError("stall_timeout_s must be non-negative")


@dataclass(frozen=True)
class GripperControlStatus:
    state: GripperControlState
    position: float
    velocity: float
    torque: float
    contact_position: Optional[float]
    contact_detected: bool
    error_reason: Optional[str]


class GripperController:
    """State-machine gripper controller built on MIT mode.

    This controller deliberately avoids the unsafe pattern of commanding a high
    ``Kp`` target at the fully closed position. Closing is a soft search; once
    contact is detected, the controller records the current position and holds
    around that contact point with low stiffness and optional feedforward torque.
    """

    def __init__(
        self,
        motor: MotorClient,
        config: GripperControlConfig,
        *,
        health_reader: Optional[Callable[[], MotorHealth]] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.motor = motor
        self.config = config
        self.health_reader = health_reader
        self._clock = clock

        self.state = GripperControlState.IDLE
        self.contact_position: Optional[float] = None
        self.contact_detected = False
        self.error_reason: Optional[str] = None

        self._state_started_at = self._clock()
        self._contact_since: Optional[float] = None
        self._stall_since: Optional[float] = None

    # ------------------------------------------------------------------
    # State transitions

    def begin_open(self) -> None:
        self._transition(GripperControlState.OPENING)
        self.contact_position = None
        self.contact_detected = False

    def begin_close(self) -> None:
        self._transition(GripperControlState.CLOSE_SEARCH)
        self.contact_position = None
        self.contact_detected = False

    def begin_release(self) -> None:
        self._transition(GripperControlState.RELEASING)

    def clear_error(self) -> None:
        self.error_reason = None
        self._transition(GripperControlState.IDLE)

    def status(self) -> GripperControlStatus:
        s = self.motor.state
        return GripperControlStatus(
            state=self.state,
            position=s.position,
            velocity=s.velocity,
            torque=s.torque,
            contact_position=self.contact_position,
            contact_detected=self.contact_detected,
            error_reason=self.error_reason,
        )

    # ------------------------------------------------------------------
    # Control loop

    def step(self) -> GripperControlStatus:
        """Run one state-machine tick and send at most one MIT command."""

        if self.state == GripperControlState.ERROR:
            self._send_neutral()
            return self.status()

        safety_reason = self._safety_reason()
        if safety_reason is not None:
            self._enter_error(safety_reason)
            self._send_neutral()
            return self.status()

        if self.state == GripperControlState.IDLE:
            return self.status()
        if self.state == GripperControlState.OPENING:
            self._step_opening(GripperControlState.IDLE, self.config.open_timeout_s)
        elif self.state == GripperControlState.RELEASING:
            self._step_opening(GripperControlState.IDLE, self.config.release_timeout_s)
        elif self.state == GripperControlState.CLOSE_SEARCH:
            self._step_close_search()
        elif self.state == GripperControlState.CONTACT_DETECTED:
            self._transition(GripperControlState.HOLDING)
            self._step_holding()
        elif self.state == GripperControlState.HOLDING:
            self._step_holding()
        else:
            self._enter_error(f"unknown state: {self.state}")

        safety_reason = self._safety_reason()
        if safety_reason is not None:
            self._enter_error(safety_reason)
            self._send_neutral()
        return self.status()

    def run_until(
        self,
        target_state: GripperControlState,
        *,
        loop_s: float = 0.01,
        timeout_s: Optional[float] = None,
    ) -> GripperControlStatus:
        """Blocking helper for scripts; production code can call ``step`` itself."""

        deadline = None if timeout_s is None else self._clock() + timeout_s
        while self.state not in (target_state, GripperControlState.ERROR):
            if deadline is not None and self._clock() >= deadline:
                self._enter_error(f"run_until timeout waiting for {target_state.value}")
                break
            self.step()
            time.sleep(loop_s)
        return self.status()

    # ------------------------------------------------------------------
    # State handlers

    def _step_opening(self, done_state: GripperControlState, timeout_s: float) -> None:
        c = self.config
        self.motor.send_mit(
            kp=c.open_kp,
            kd=c.open_kd,
            q=c.open_pos,
            dq=0.0,
            tau=c.open_tau_ff,
            poll=True,
        )
        s = self.motor.state
        reached = (
            abs(s.position - c.open_pos) <= c.open_position_tolerance
            and abs(s.velocity) <= c.open_velocity_tolerance
        )
        if reached:
            self._transition(done_state)
        elif self._state_age_s() >= timeout_s:
            self._enter_error(f"{self.state.value} timeout")

    def _step_close_search(self) -> None:
        c = self.config
        s = self.motor.state
        q_target = self._next_search_target(s.position)
        dq_target = self._close_direction * abs(c.close_speed)

        self.motor.send_mit(
            kp=c.search_kp,
            kd=c.search_kd,
            q=q_target,
            dq=dq_target,
            tau=c.search_tau_ff,
            poll=True,
        )

        if self._at_close_limit(self.motor.state.position):
            self.contact_position = self.motor.state.position
            self.contact_detected = False
            self._transition(GripperControlState.HOLDING)
            return

        if self._contact_condition():
            if self._contact_since is None:
                self._contact_since = self._clock()
            if self._clock() - self._contact_since >= c.contact_confirm_s:
                self.contact_position = self.motor.state.position
                self.contact_detected = True
                self._transition(GripperControlState.CONTACT_DETECTED)
                return
        else:
            self._contact_since = None

        if self._state_age_s() >= c.close_timeout_s:
            self._enter_error("close search timeout")

    def _step_holding(self) -> None:
        c = self.config
        q_hold = self.contact_position
        if q_hold is None:
            q_hold = self.motor.state.position
            self.contact_position = q_hold

        self.motor.send_mit(
            kp=c.hold_kp,
            kd=c.hold_kd,
            q=q_hold,
            dq=0.0,
            tau=c.hold_tau_ff,
            poll=True,
        )

    # ------------------------------------------------------------------
    # Detection and safety

    @property
    def _close_direction(self) -> float:
        return 1.0 if self.config.close_limit > self.config.open_pos else -1.0

    def _next_search_target(self, position: float) -> float:
        c = self.config
        target = float(position) + self._close_direction * c.search_step
        if self._close_direction > 0:
            return min(target, c.close_limit)
        return max(target, c.close_limit)

    def _at_close_limit(self, position: float) -> bool:
        distance = (self.config.close_limit - float(position)) * self._close_direction
        return distance <= self.config.close_limit_tolerance

    def _contact_condition(self) -> bool:
        s = self.motor.state
        if self._at_close_limit(s.position):
            return False
        return (
            abs(s.velocity) <= self.config.contact_velocity_threshold
            and abs(s.torque) >= self.config.contact_torque_threshold
        )

    def _safety_reason(self) -> Optional[str]:
        c = self.config
        s = self.motor.state

        lo = min(c.open_pos, c.close_limit) - c.position_margin
        hi = max(c.open_pos, c.close_limit) + c.position_margin
        if s.position < lo or s.position > hi:
            return f"position out of range: {s.position:.3f}"

        if c.max_abs_torque is not None and abs(s.torque) > c.max_abs_torque:
            return f"torque limit exceeded: {s.torque:.3f}"

        health = self.health_reader() if self.health_reader is not None else None
        if health is not None:
            if c.max_abs_current is not None and health.current_a is not None:
                if abs(health.current_a) > c.max_abs_current:
                    return f"current limit exceeded: {health.current_a:.3f}A"
            if c.max_temperature_c is not None and health.temperature_c is not None:
                if health.temperature_c > c.max_temperature_c:
                    return f"temperature limit exceeded: {health.temperature_c:.1f}C"
            if health.error_code not in (None, 0):
                return f"motor error code: {health.error_code}"

        if self.state == GripperControlState.CLOSE_SEARCH:
            stalled = (
                abs(s.velocity) <= c.stall_velocity_threshold
                and abs(s.torque) >= c.stall_torque_threshold
            )
            if stalled:
                if self._stall_since is None:
                    self._stall_since = self._clock()
                elif self._clock() - self._stall_since >= c.stall_timeout_s:
                    return "stall protection triggered"
            else:
                self._stall_since = None

        return None

    # ------------------------------------------------------------------
    # Helpers

    def _send_neutral(self) -> None:
        self.motor.send_mit(
            kp=0.0,
            kd=self.config.error_kd,
            q=self.motor.state.position,
            dq=0.0,
            tau=0.0,
            poll=True,
        )

    def _transition(self, state: GripperControlState) -> None:
        self.state = state
        self._state_started_at = self._clock()
        self._contact_since = None
        self._stall_since = None

    def _enter_error(self, reason: str) -> None:
        self.error_reason = reason
        self._transition(GripperControlState.ERROR)

    def _state_age_s(self) -> float:
        return self._clock() - self._state_started_at
