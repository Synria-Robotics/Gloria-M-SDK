from __future__ import annotations

from gloria_m_sdk import (
    ControlMode,
    FakeCanAdapter,
    GripperControlConfig,
    GripperControlState,
    GripperController,
    MotorHealth,
)
from gloria_m_sdk.client import MotorClient
from gloria_m_sdk.protocol import uint_to_float
from gloria_m_sdk.registers import Variable
from gloria_m_sdk.types import Limits


def _mit_q_from_command(data8: bytes, limits: Limits) -> float:
    q_uint = (data8[0] << 8) | data8[1]
    return uint_to_float(q_uint, -limits.pmax, limits.pmax, 16)


def _setup_mit_gripper(fake: FakeCanAdapter) -> MotorClient:
    g = MotorClient("unused", _transport=fake)
    g.connect(apply_limits=False)
    fake.queue_param_reply(
        can_id=0x101,
        rid=int(Variable.CTRL_MODE),
        value=int(ControlMode.MIT),
        is_u32=True,
    )
    g.set_mode(ControlMode.MIT)
    fake.clear()
    return g


def test_close_search_uses_incremental_target_not_close_limit(fake: FakeCanAdapter) -> None:
    g = _setup_mit_gripper(fake)
    limits = g.limits
    fake.queue_mit_feedback(can_id=0x101, position=2.45, velocity=-0.2, torque=0.05)
    g.refresh()
    fake.clear()

    cfg = GripperControlConfig(
        open_pos=2.7,
        close_limit=0.0,
        search_step=0.05,
        close_speed=0.2,
        max_abs_torque=None,
    )
    ctrl = GripperController(g, cfg)
    ctrl.begin_close()
    ctrl.step()

    can_id, data = fake.last_sent()
    assert can_id == 0x01
    q_target = _mit_q_from_command(data, limits)
    assert 2.35 < q_target < 2.45
    assert abs(q_target - cfg.close_limit) > 1.0


def test_contact_records_position_and_enters_holding(fake: FakeCanAdapter) -> None:
    g = _setup_mit_gripper(fake)
    fake.queue_mit_feedback(can_id=0x101, position=1.25, velocity=0.0, torque=-0.8)

    cfg = GripperControlConfig(
        open_pos=2.7,
        close_limit=0.0,
        contact_confirm_s=0.0,
        contact_torque_threshold=0.3,
        max_abs_torque=None,
    )
    ctrl = GripperController(g, cfg)
    ctrl.begin_close()

    status = ctrl.step()

    assert status.state == GripperControlState.CONTACT_DETECTED
    assert status.contact_detected is True
    assert status.contact_position is not None
    assert abs(status.contact_position - 1.25) < 0.02


def test_close_limit_is_not_reported_as_object_contact(fake: FakeCanAdapter) -> None:
    g = _setup_mit_gripper(fake)
    fake.queue_mit_feedback(can_id=0x101, position=0.01, velocity=0.0, torque=-0.8)

    cfg = GripperControlConfig(
        open_pos=2.7,
        close_limit=0.0,
        close_limit_tolerance=0.03,
        contact_confirm_s=0.0,
        contact_torque_threshold=0.3,
        max_abs_torque=None,
    )
    ctrl = GripperController(g, cfg)
    ctrl.begin_close()

    status = ctrl.step()

    assert status.state == GripperControlState.HOLDING
    assert status.contact_detected is False


def test_torque_safety_enters_error(fake: FakeCanAdapter) -> None:
    g = _setup_mit_gripper(fake)
    fake.queue_mit_feedback(can_id=0x101, position=1.5, velocity=0.0, torque=-5.0)

    cfg = GripperControlConfig(
        open_pos=2.7,
        close_limit=0.0,
        max_abs_torque=2.0,
    )
    ctrl = GripperController(g, cfg)
    ctrl.begin_close()

    status = ctrl.step()

    assert status.state == GripperControlState.ERROR
    assert status.error_reason is not None
    assert "torque limit" in status.error_reason


def test_health_reader_can_trigger_temperature_error(fake: FakeCanAdapter) -> None:
    g = _setup_mit_gripper(fake)

    cfg = GripperControlConfig(
        open_pos=2.7,
        close_limit=0.0,
        max_temperature_c=60.0,
    )
    ctrl = GripperController(
        g,
        cfg,
        health_reader=lambda: MotorHealth(temperature_c=72.0),
    )
    ctrl.begin_close()

    status = ctrl.step()

    assert status.state == GripperControlState.ERROR
    assert status.error_reason is not None
    assert "temperature" in status.error_reason
