from __future__ import annotations

from dataclasses import dataclass, fields
import os
from pathlib import Path
import time
import tomllib
from typing import Any, Callable, Optional

from .client import MotorClient as _MotorClient
from .exceptions import GloriaSdkError
from .gripper_control import (
    GripperControlConfig,
    GripperControlState,
    GripperControlStatus,
    GripperController,
    MotorHealth,
)
from .types import ControlMode, Limits, PositionRange

DEFAULT_GRIPPER_CONFIG = os.path.join("demos", "gripper_control.toml")
StatusCallback = Callable[[str, GripperControlStatus], None]
PvStatusCallback = Callable[[str, "PvMoveStatus"], None]
MotorStatusCallback = Callable[[str, "MotorSnapshot"], None]


@dataclass(frozen=True)
class GripperConnectionConfig:
    port: str = "auto"
    baudrate: int = 921_600
    command_id: int = 0x01
    feedback_id: int = 0x101
    timeout: float = 0.5


@dataclass(frozen=True)
class GripperLoopConfig:
    period_s: float = 0.01
    print_hz: float = 10.0
    hold_s: float = 2.0
    open_first: bool = True
    return_to_initial: bool = True


@dataclass(frozen=True)
class GripperConfig:
    connection: GripperConnectionConfig
    limits: Limits
    control: GripperControlConfig
    loop: GripperLoopConfig


@dataclass(frozen=True)
class MotorSnapshot:
    position: float
    velocity: float
    torque: float
    has_feedback: bool


def format_motor_snapshot(
    prefix: str,
    snapshot: MotorSnapshot,
    *,
    gripper: Optional["GloriaGripper"] = None,
) -> str:
    feedback = "yes" if snapshot.has_feedback else "no"
    pos = ""
    if gripper is not None:
        pos_text = f"{gripper.position_value(snapshot.position):6.1f}/1000" if snapshot.has_feedback else "   n/a"
        pos = f" pos={pos_text}"
    return (
        f"{prefix}{pos} q={snapshot.position:+.3f} "
        f"dq={snapshot.velocity:+.3f} tau={snapshot.torque:+.3f} feedback={feedback}"
    )


def print_motor_snapshot(
    prefix: str,
    snapshot: MotorSnapshot,
    *,
    gripper: Optional["GloriaGripper"] = None,
) -> None:
    print(format_motor_snapshot(prefix, snapshot, gripper=gripper))


@dataclass(frozen=True)
class DiagnosticResult:
    ctrl_mode: int | float | None
    pmax: int | float | None
    vmax: int | float | None
    tmax: int | float | None
    state: MotorSnapshot

    @property
    def motor_replied(self) -> bool:
        return any(value is not None for value in (self.ctrl_mode, self.pmax, self.vmax, self.tmax)) or self.state.has_feedback


@dataclass(frozen=True)
class CanIdScanHit:
    command_id: int
    feedback_id: int
    ctrl_mode: int | float


@dataclass(frozen=True)
class PvMoveConfig:
    open_pos: float
    close_pos: float
    open_velocity: float = 1.0
    close_velocity: float = 0.3
    settle_threshold: float = 0.1
    settle_time_s: float = 0.3
    timeout_s: float = 8.0
    period_s: float = 0.01


@dataclass(frozen=True)
class PvMoveStatus:
    phase: str
    target: float
    position: float
    velocity: float
    torque: float
    elapsed_s: float
    settled: bool
    timed_out: bool


class GloriaGripper:
    """Single user-facing gripper API.

    Normal users should create this object with ``from_config()`` and then call
    methods on it. The lower motor client remains an internal implementation
    detail used for protocol and transport operations.
    """

    def __init__(
        self,
        motor: _MotorClient,
        controller: GripperController,
        *,
        loop: Optional[GripperLoopConfig] = None,
        pv_config: Optional[PvMoveConfig] = None,
        config: Optional[GripperConfig] = None,
        status_callback: Optional[StatusCallback] = None,
        motor_status_callback: Optional[MotorStatusCallback] = None,
        pv_status_callback: Optional[PvStatusCallback] = None,
        hold_on_stall_protection: bool = False,
    ) -> None:
        self.motor = motor
        self.controller = controller
        self.loop = loop if loop is not None else GripperLoopConfig()
        self.pv_config = pv_config
        self.config = config
        self.status_callback = status_callback
        self.motor_status_callback = motor_status_callback
        self.pv_status_callback = pv_status_callback
        self.hold_on_stall_protection = hold_on_stall_protection
        self._connected = False

    @classmethod
    def from_config(
        cls,
        path: str | os.PathLike[str] = DEFAULT_GRIPPER_CONFIG,
        *,
        port: Optional[str] = None,
        status_callback: Optional[StatusCallback] = None,
        motor_status_callback: Optional[MotorStatusCallback] = None,
        pv_status_callback: Optional[PvStatusCallback] = None,
        open_velocity: float = 1.0,
        close_velocity: float = 0.3,
        hold_on_stall_protection: bool = True,
        health_reader: Optional[Callable[[], MotorHealth]] = None,
    ) -> "GloriaGripper":
        cfg = load_gripper_config(path)
        resolved_port = _resolve_port(port if port is not None else cfg.connection.port)
        safe_q = PositionRange(
            min=min(cfg.control.open_pos, cfg.control.close_limit),
            max=max(cfg.control.open_pos, cfg.control.close_limit),
        )
        motor = _MotorClient(
            resolved_port,
            baudrate=cfg.connection.baudrate,
            command_id=cfg.connection.command_id,
            feedback_id=cfg.connection.feedback_id,
            limits=cfg.limits,
            safe_position=safe_q,
            timeout=cfg.connection.timeout,
        )
        controller = GripperController(
            motor,
            cfg.control,
            health_reader=health_reader,
        )
        pv_config = PvMoveConfig(
            open_pos=cfg.control.open_pos,
            close_pos=cfg.control.close_limit,
            open_velocity=open_velocity,
            close_velocity=close_velocity,
            period_s=cfg.loop.period_s,
        )
        return cls(
            motor,
            controller,
            loop=cfg.loop,
            pv_config=pv_config,
            config=cfg,
            status_callback=status_callback,
            motor_status_callback=motor_status_callback,
            pv_status_callback=pv_status_callback,
            hold_on_stall_protection=hold_on_stall_protection,
        )

    def connect(
        self,
        *,
        mode: Optional[ControlMode] = ControlMode.MIT,
        enable: bool = True,
        apply_limits: bool = False,
        refresh: bool = True,
    ) -> GripperControlStatus:
        self.motor.connect(apply_limits=apply_limits)
        if mode is not None:
            self.motor.set_mode(mode)
        if enable:
            self.motor.enable()
        if refresh:
            self.motor.refresh()
        self._connected = True
        status = self.status()
        self._emit("connect", status)
        return status

    def disconnect(self, *, disable: bool = True) -> None:
        if disable and self.motor.is_connected:
            self.motor.disable()
        self.motor.disconnect()
        self._connected = False

    def __enter__(self) -> "GloriaGripper":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    def status(self) -> GripperControlStatus:
        return self.controller.status()

    @property
    def current_mode(self) -> Optional[ControlMode]:
        return self.motor.current_mode

    @property
    def port(self) -> str:
        return str(getattr(self.motor, "_port"))

    def snapshot(self) -> MotorSnapshot:
        return MotorSnapshot(
            position=self.motor.state.position,
            velocity=self.motor.state.velocity,
            torque=self.motor.state.torque,
            has_feedback=self.motor.state.updated_at > 0,
        )

    def refresh(self) -> MotorSnapshot:
        self.motor.refresh()
        snapshot = self.snapshot()
        self._emit_motor("refresh", snapshot)
        return snapshot

    def poll(self) -> MotorSnapshot:
        self.motor.poll()
        snapshot = self.snapshot()
        self._emit_motor("poll", snapshot)
        return snapshot

    def position_value(self, position: Optional[float] = None) -> float:
        """Return gripper position in user units where 0 is closed and 1000 is open."""

        q = self.motor.state.position if position is None else float(position)
        c = self.controller.config
        span = c.open_pos - c.close_limit
        if span == 0:
            raise GloriaSdkError("open_pos and close_limit must differ")
        value = (q - c.close_limit) / span * 1000.0
        return max(0.0, min(1000.0, value))

    def motor_position(self, value: float) -> float:
        """Convert user units where 0 is closed and 1000 is open into motor position."""

        return self._user_value_to_motor_position(value)

    def set_mode(self, mode: ControlMode) -> MotorSnapshot:
        self.motor.set_mode(mode)
        self.motor.refresh()
        snapshot = self.snapshot()
        self._emit_motor(f"mode:{mode.name}", snapshot)
        return snapshot

    def enable(self) -> MotorSnapshot:
        self.motor.enable()
        self.motor.refresh()
        snapshot = self.snapshot()
        self._emit_motor("enable", snapshot)
        return snapshot

    def disable(self) -> MotorSnapshot:
        self.motor.disable()
        snapshot = self.snapshot()
        self._emit_motor("disable", snapshot)
        return snapshot

    def set_zero(self) -> MotorSnapshot:
        self.motor.set_zero()
        self.motor.refresh()
        snapshot = self.snapshot()
        self._emit_motor("set_zero", snapshot)
        return snapshot

    def enable_for(
        self,
        duration_s: float,
        *,
        mode: Optional[ControlMode] = ControlMode.MIT,
        neutral_mit: bool = True,
    ) -> MotorSnapshot:
        self.connect(mode=mode, enable=False, apply_limits=False, refresh=False)
        try:
            snapshot = self.enable()
            end_at = time.monotonic() + duration_s
            while time.monotonic() < end_at:
                if neutral_mit and mode == ControlMode.MIT:
                    self.motor.send_mit(
                        kp=0.0,
                        kd=0.8,
                        q=self.motor.state.position,
                        dq=0.0,
                        tau=0.0,
                        poll=True,
                    )
                else:
                    self.motor.refresh()
                snapshot = self.snapshot()
                self._emit_motor("enabled", snapshot)
                time.sleep(0.05)
            return snapshot
        finally:
            self.disconnect()

    def send_mit_frame(
        self,
        *,
        kp: float = 0.0,
        kd: float = 0.8,
        q: Optional[float] = None,
        dq: float = 0.0,
        tau: float = 0.0,
        poll: bool = True,
    ) -> MotorSnapshot:
        q_cmd = self.motor.state.position if q is None else q
        self.motor.send_mit(kp=kp, kd=kd, q=q_cmd, dq=dq, tau=tau, poll=poll)
        snapshot = self.snapshot()
        self._emit_motor("mit", snapshot)
        return snapshot

    def send_mit_for(
        self,
        duration_s: float,
        *,
        kp: float = 0.0,
        kd: float = 0.8,
        q: Optional[float] = None,
        dq: float = 0.0,
        tau: float = 0.0,
        period_s: float = 0.01,
    ) -> MotorSnapshot:
        self.connect(mode=ControlMode.MIT, enable=True, apply_limits=False, refresh=False)
        try:
            snapshot = self.send_mit_frame(kp=kp, kd=kd, q=q, dq=dq, tau=tau)
            end_at = time.monotonic() + duration_s
            while time.monotonic() < end_at:
                snapshot = self.send_mit_frame(kp=kp, kd=kd, q=q, dq=dq, tau=tau)
                time.sleep(period_s)
            return snapshot
        finally:
            self.disconnect()

    def send_pv_frame(
        self,
        *,
        position: float,
        velocity: float,
        poll: bool = True,
    ) -> MotorSnapshot:
        self.motor.send_pos_vel(position=position, velocity=velocity, poll=poll)
        snapshot = self.snapshot()
        self._emit_motor("pv", snapshot)
        return snapshot

    def send_pv_for(
        self,
        duration_s: float,
        *,
        position: float,
        velocity: float,
        period_s: float = 0.01,
    ) -> MotorSnapshot:
        self.connect(mode=ControlMode.POS_VEL, enable=True, apply_limits=False, refresh=False)
        try:
            snapshot = self.send_pv_frame(position=position, velocity=velocity)
            end_at = time.monotonic() + duration_s
            while time.monotonic() < end_at:
                snapshot = self.send_pv_frame(position=position, velocity=velocity)
                time.sleep(period_s)
            return snapshot
        finally:
            self.disconnect()

    def pv_open(self) -> PvMoveStatus:
        cfg = self._require_pv_config()
        return self.pv_move_to("open", cfg.open_pos, cfg.open_velocity)

    def pv_close(self) -> PvMoveStatus:
        cfg = self._require_pv_config()
        return self.pv_move_to("close", cfg.close_pos, cfg.close_velocity)

    def pv_hold_closed(self, duration_s: float) -> PvMoveStatus:
        cfg = self._require_pv_config()
        return self.pv_hold(cfg.close_pos, duration_s, phase="hold")

    def pv_move_to(
        self,
        phase: str,
        target: float,
        velocity: float,
        *,
        timeout_s: Optional[float] = None,
    ) -> PvMoveStatus:
        cfg = self._require_pv_config()
        timeout = cfg.timeout_s if timeout_s is None else timeout_s
        started_at = time.monotonic()
        settled_at: Optional[float] = None
        status = self._pv_status(phase, target, started_at, settled=False, timed_out=False)

        while True:
            self.motor.send_pos_vel(position=target, velocity=velocity, poll=True)
            now = time.monotonic()
            elapsed = now - started_at
            error = abs(self.motor.state.position - target)

            if error < cfg.settle_threshold:
                if settled_at is None:
                    settled_at = now
                elif now - settled_at >= cfg.settle_time_s:
                    status = self._pv_status(phase, target, started_at, settled=True, timed_out=False)
                    self._emit_pv(status)
                    return status
            else:
                settled_at = None

            if elapsed > timeout:
                status = self._pv_status(phase, target, started_at, settled=False, timed_out=True)
                self._emit_pv(status)
                return status

            status = self._pv_status(phase, target, started_at, settled=False, timed_out=False)
            self._emit_pv(status)
            time.sleep(cfg.period_s)

    def pv_hold(self, target: float, duration_s: float, *, phase: str = "hold") -> PvMoveStatus:
        cfg = self._require_pv_config()
        started_at = time.monotonic()
        status = self._pv_status(phase, target, started_at, settled=False, timed_out=False)
        while time.monotonic() - started_at < duration_s:
            self.motor.send_pos_vel(position=target, velocity=0.0, poll=True)
            status = self._pv_status(phase, target, started_at, settled=False, timed_out=False)
            self._emit_pv(status)
            time.sleep(cfg.period_s)
        return status

    def check_connection(self, *, read_timeout_s: float = 0.2) -> DiagnosticResult:
        self.motor.connect(apply_limits=False)
        try:
            ctrl_mode = self.motor.read_param(10, timeout_s=read_timeout_s)
            pmax = self.motor.read_param(21, timeout_s=read_timeout_s)
            vmax = self.motor.read_param(22, timeout_s=read_timeout_s)
            tmax = self.motor.read_param(23, timeout_s=read_timeout_s)
            self.motor.refresh()
            return DiagnosticResult(
                ctrl_mode=ctrl_mode,
                pmax=pmax,
                vmax=vmax,
                tmax=tmax,
                state=self.snapshot(),
            )
        finally:
            self.motor.disconnect()

    def scan_ids(self, ids: list[int], *, read_timeout_s: float = 0.2) -> list[CanIdScanHit]:
        if self.config is None:
            raise GloriaSdkError("scan_ids requires a gripper created with from_config()")
        hits: list[CanIdScanHit] = []
        for command_id in ids:
            feedback_id = 0x100 + (command_id & 0x7FF)
            motor = self._make_motor_for_ids(command_id, feedback_id)
            try:
                motor.connect(apply_limits=False)
                value = motor.read_param(10, timeout_s=read_timeout_s)
                if value is not None:
                    hits.append(CanIdScanHit(command_id=command_id, feedback_id=feedback_id, ctrl_mode=value))
            finally:
                motor.disconnect()
        return hits

    @staticmethod
    def list_ports() -> list[tuple[str, str]]:
        from serial.tools import list_ports

        return [(p.device, p.description) for p in list_ports.comports()]

    def _emit_motor(self, phase: str, snapshot: MotorSnapshot) -> None:
        if self.motor_status_callback is not None:
            self.motor_status_callback(phase, snapshot)

    def _require_pv_config(self) -> PvMoveConfig:
        if self.pv_config is None:
            raise GloriaSdkError("PV motion requires a gripper created with from_config()")
        return self.pv_config

    def _pv_status(self, phase: str, target: float, started_at: float, *, settled: bool, timed_out: bool) -> PvMoveStatus:
        s = self.motor.state
        return PvMoveStatus(
            phase=phase,
            target=target,
            position=s.position,
            velocity=s.velocity,
            torque=s.torque,
            elapsed_s=time.monotonic() - started_at,
            settled=settled,
            timed_out=timed_out,
        )

    def _emit_pv(self, status: PvMoveStatus) -> None:
        if self.pv_status_callback is not None:
            self.pv_status_callback(status.phase, status)

    def _make_motor_for_ids(self, command_id: int, feedback_id: int) -> _MotorClient:
        if self.config is None:
            raise GloriaSdkError("scan_ids requires a gripper created with from_config()")
        safe_q = PositionRange(
            min=min(self.config.control.open_pos, self.config.control.close_limit),
            max=max(self.config.control.open_pos, self.config.control.close_limit),
        )
        return _MotorClient(
            getattr(self.motor, "_port"),
            baudrate=self.config.connection.baudrate,
            command_id=command_id,
            feedback_id=feedback_id,
            limits=self.config.limits,
            safe_position=safe_q,
            timeout=self.config.connection.timeout,
        )

    def open(self) -> GripperControlStatus:
        self.controller.begin_open()
        return self._run_until({GripperControlState.IDLE}, phase="open")

    def close(self) -> GripperControlStatus:
        self.controller.begin_close()
        return self._run_until({GripperControlState.HOLDING}, phase="close")

    def hold(self, duration_s: Optional[float] = None) -> GripperControlStatus:
        end_at = time.monotonic() + (self.loop.hold_s if duration_s is None else duration_s)
        status = self.status()
        while time.monotonic() < end_at and self.controller.state != GripperControlState.ERROR:
            status = self.controller.step()
            self._emit("hold", status)
            time.sleep(self.loop.period_s)
        self._raise_if_error(status)
        return status

    def release(self) -> GripperControlStatus:
        self.controller.begin_release()
        return self._run_until({GripperControlState.IDLE}, phase="release")

    def move_to(self, value: float, *, stall_protection: bool = True) -> GripperControlStatus:
        """Move gripper using user units where 1000 is open and 0 is closed.

        Positive intermediate values use normal MIT position control. A command
        of 0 uses the soft-close state machine by default, so the gripper does
        not keep pushing against an object or mechanical stop. Objects created
        with ``from_config()`` keep sending holding frames after contact until
        the caller interrupts or disconnects.
        """

        value = max(0.0, min(1000.0, float(value)))
        if value <= 0.0 and stall_protection:
            status = self.close()
            while self.hold_on_stall_protection:
                status = self.hold(1.0)
            return status
        if value >= 1000.0:
            return self.open()

        target = self._user_value_to_motor_position(value)
        return self._move_to_motor_position(target)

    def grip(
        self,
        *,
        open_first: Optional[bool] = None,
        hold_s: Optional[float] = None,
        return_to_initial: Optional[bool] = None,
    ) -> GripperControlStatus:
        do_open = self.loop.open_first if open_first is None else open_first
        do_return = self.loop.return_to_initial if return_to_initial is None else return_to_initial

        if do_open:
            self.open()
        status = self.close()
        status = self.hold(hold_s)
        if do_return:
            status = self.release()
        return status

    def _run_until(
        self,
        done_states: set[GripperControlState],
        *,
        phase: str,
    ) -> GripperControlStatus:
        status = self.status()
        while self.controller.state not in done_states and self.controller.state != GripperControlState.ERROR:
            status = self.controller.step()
            self._emit(phase, status)
            time.sleep(self.loop.period_s)
        self._raise_if_error(status)
        return status

    def _emit(self, phase: str, status: GripperControlStatus) -> None:
        if self.status_callback is not None:
            self.status_callback(phase, status)

    @staticmethod
    def _raise_if_error(status: GripperControlStatus) -> None:
        if status.state == GripperControlState.ERROR:
            reason = status.error_reason or "unknown error"
            raise GloriaSdkError(f"gripper control error: {reason}")

    def _user_value_to_motor_position(self, value: float) -> float:
        c = self.controller.config
        ratio = max(0.0, min(1000.0, float(value))) / 1000.0
        return c.close_limit + (c.open_pos - c.close_limit) * ratio

    def _move_to_motor_position(self, target: float) -> GripperControlStatus:
        c = self.controller.config
        started_at = time.monotonic()
        status = self.status()

        while True:
            self.motor.send_mit(
                kp=c.open_kp,
                kd=c.open_kd,
                q=target,
                dq=0.0,
                tau=c.open_tau_ff,
                poll=True,
            )
            status = self.status()
            reached = (
                abs(self.motor.state.position - target) <= c.open_position_tolerance
                and abs(self.motor.state.velocity) <= c.open_velocity_tolerance
            )
            if reached:
                return status
            if time.monotonic() - started_at >= c.open_timeout_s:
                raise GloriaSdkError(f"move_to timeout: target={target:.3f}")
            time.sleep(self.loop.period_s)


def load_gripper_config(path: str | os.PathLike[str] = DEFAULT_GRIPPER_CONFIG) -> GripperConfig:
    data = _load_toml(Path(path))
    connection = data.get("connection", {})
    loop = data.get("loop", {})

    control_raw = dict(data.get("control", {}))
    allowed = {field.name for field in fields(GripperControlConfig)}
    unknown = sorted(set(control_raw) - allowed)
    if unknown:
        raise ValueError(f"unknown [control] keys: {', '.join(unknown)}")

    return GripperConfig(
        connection=GripperConnectionConfig(
            port=str(connection.get("port", "auto")),
            baudrate=int(connection.get("baudrate", 921_600)),
            command_id=_parse_int(connection.get("command_id", "0x01")),
            feedback_id=_parse_int(connection.get("feedback_id", "0x101")),
            timeout=float(connection.get("timeout", 0.5)),
        ),
        limits=_build_limits(data),
        control=GripperControlConfig(**control_raw),
        loop=GripperLoopConfig(
            period_s=float(loop.get("period_s", 0.01)),
            print_hz=float(loop.get("print_hz", 10.0)),
            hold_s=float(loop.get("hold_s", 2.0)),
            open_first=bool(loop.get("open_first", True)),
            return_to_initial=bool(loop.get("return_to_initial", loop.get("release_after_hold", True))),
        ),
    )


def resolve_gripper_port(
    path: str | os.PathLike[str] = DEFAULT_GRIPPER_CONFIG,
    *,
    port: Optional[str] = None,
) -> str:
    cfg = load_gripper_config(path)
    return _resolve_port(port if port is not None else cfg.connection.port)


def _load_toml(path: Path) -> dict[str, Any]:
    with path.open("rb") as f:
        return tomllib.load(f)


def _parse_int(value: Any) -> int:
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        return int(value, 0)
    raise TypeError(f"expected int or int string, got {type(value).__name__}")


def _build_limits(data: dict[str, Any]) -> Limits:
    limits = data.get("limits", {})
    return Limits(
        pmax=float(limits.get("pmax", 3.14)),
        vmax=float(limits.get("vmax", 10.0)),
        tmax=float(limits.get("tmax", 12.0)),
    )


def _resolve_port(port: str | None) -> str:
    if port is not None and port.strip() and port.strip().lower() != "auto":
        return port.strip()
    from serial.tools import list_ports

    found = list(list_ports.comports())
    if not found:
        raise GloriaSdkError("No serial ports found. Plug in the USB-CAN adapter.")

    candidates = sorted(
        ((score, p) for p in found if (score := _port_score(p)) > 0),
        key=lambda item: item[0],
        reverse=True,
    )
    if candidates:
        if len(candidates) == 1 or candidates[0][0] > candidates[1][0]:
            return candidates[0][1].device
        names = ", ".join(f"{p.device} ({p.description})" for _, p in candidates)
        raise GloriaSdkError(f"Multiple USB-CAN-like serial ports found ({names}); pass --port or set connection.port.")

    if len(found) == 1:
        return found[0].device

    names = ", ".join(f"{p.device} ({p.description})" for p in found)
    raise GloriaSdkError(f"Multiple serial ports found ({names}); pass --port or set connection.port.")


def _port_score(port_info: object) -> int:
    device = str(getattr(port_info, "device", "")).lower()
    description = str(getattr(port_info, "description", "")).lower()
    manufacturer = str(getattr(port_info, "manufacturer", "")).lower()
    product = str(getattr(port_info, "product", "")).lower()
    text = " ".join((device, description, manufacturer, product))

    if "bluetooth" in text or "debug-console" in text:
        return 0

    score = 0
    if "usbmodem" in device or "ttyacm" in device or "ttyusb" in device:
        score += 100
    if "cdc" in text:
        score += 80
    if "usb" in text:
        score += 60
    if "can" in text:
        score += 40
    if "serial" in text:
        score += 10
    return score
