from __future__ import annotations

from gloria_m_sdk import (
    ControlMode,
    FakeCanAdapter,
    GloriaGripper,
    GripperControlConfig,
    GripperControlState,
    GripperController,
    GripperLoopConfig,
    PvMoveConfig,
    load_gripper_config,
    resolve_gripper_port,
)
from gloria_m_sdk.client import MotorClient
from gloria_m_sdk.registers import Variable


def test_load_gripper_config_reads_single_toml() -> None:
    cfg = load_gripper_config("demos/gripper_control.toml")

    assert cfg.connection.command_id == 0x01
    assert cfg.connection.feedback_id == 0x101
    assert cfg.control.open_pos > cfg.control.close_limit


def test_resolve_gripper_port_prefers_manual_override() -> None:
    assert resolve_gripper_port("demos/gripper_control.toml", port="/dev/test-can") == "/dev/test-can"


def test_mit_gripper_api_close_hides_state_machine(fake: FakeCanAdapter) -> None:
    motor = MotorClient("unused", _transport=fake)
    controller = GripperController(
        motor,
        GripperControlConfig(
            open_pos=2.7,
            close_limit=0.0,
            contact_confirm_s=0.0,
            contact_torque_threshold=0.3,
            max_abs_torque=None,
        ),
    )
    api = GloriaGripper(
        motor,
        controller,
        loop=GripperLoopConfig(period_s=0.0, hold_s=0.0),
    )

    fake.queue_param_reply(
        can_id=0x101,
        rid=int(Variable.CTRL_MODE),
        value=int(ControlMode.MIT),
        is_u32=True,
    )
    fake.queue_mit_feedback(can_id=0x101, position=1.3, velocity=0.0, torque=-0.6)
    api.connect(apply_limits=False)

    fake.clear()
    fake.queue_mit_feedback(can_id=0x101, position=1.25, velocity=0.0, torque=-0.8)

    status = api.close()

    assert status.state == GripperControlState.HOLDING
    assert status.contact_detected is True
    assert status.contact_position is not None


def test_mit_gripper_api_move_to_uses_user_units_and_stall_protection(fake: FakeCanAdapter) -> None:
    motor = MotorClient("unused", _transport=fake)
    controller = GripperController(
        motor,
        GripperControlConfig(
            open_pos=2.0,
            close_limit=0.0,
            open_position_tolerance=0.02,
            open_velocity_tolerance=0.02,
            contact_confirm_s=0.0,
            contact_torque_threshold=0.3,
            max_abs_torque=None,
        ),
    )
    api = GloriaGripper(
        motor,
        controller,
        loop=GripperLoopConfig(period_s=0.0, hold_s=0.0),
    )
    fake.queue_param_reply(
        can_id=0x101,
        rid=int(Variable.CTRL_MODE),
        value=int(ControlMode.MIT),
        is_u32=True,
    )
    fake.queue_mit_feedback(can_id=0x101, position=2.0, velocity=0.0, torque=0.0)
    api.connect(apply_limits=False)

    fake.clear()
    fake.queue_mit_feedback(can_id=0x101, position=1.0, velocity=0.0, torque=0.0)
    halfway = api.move_to(500)
    assert abs(halfway.position - 1.0) < 0.02

    fake.queue_mit_feedback(can_id=0x101, position=0.8, velocity=0.0, torque=-0.8)
    closed = api.move_to(0)
    assert closed.contact_detected is True
    assert closed.contact_position is not None


def test_gloria_gripper_position_value_maps_motor_position_to_user_units(fake: FakeCanAdapter) -> None:
    motor = MotorClient("unused", _transport=fake)
    api = GloriaGripper(
        motor,
        GripperController(motor, GripperControlConfig(open_pos=2.0, close_limit=0.0)),
    )

    assert api.position_value(0.0) == 0.0
    assert api.position_value(1.0) == 500.0
    assert api.position_value(2.0) == 1000.0
    assert api.position_value(3.0) == 1000.0


def test_pv_gripper_api_move_to_hides_mode_and_loop(fake: FakeCanAdapter) -> None:
    motor = MotorClient("unused", _transport=fake)
    api = GloriaGripper(
        motor,
        GripperController(motor, GripperControlConfig(open_pos=2.7, close_limit=0.0)),
        pv_config=PvMoveConfig(
            open_pos=2.7,
            close_pos=0.0,
            settle_time_s=0.0,
            period_s=0.0,
        ),
    )

    fake.queue_param_reply(
        can_id=0x101,
        rid=int(Variable.CTRL_MODE),
        value=int(ControlMode.POS_VEL),
        is_u32=True,
    )
    fake.queue_mit_feedback(can_id=0x101, position=2.7, velocity=0.0, torque=0.0)
    api.connect(mode=ControlMode.POS_VEL, apply_limits=False)

    fake.clear()
    fake.queue_mit_feedback(can_id=0x101, position=0.0, velocity=0.0, torque=0.0)

    status = api.pv_close()

    assert status.settled is True
    assert status.timed_out is False
    assert abs(status.position - 0.0) < 0.02


def test_motor_lifecycle_api_enable_for_disables_afterward(fake: FakeCanAdapter) -> None:
    motor = MotorClient("unused", _transport=fake)
    api = GloriaGripper(motor, GripperController(motor, GripperControlConfig(open_pos=2.7, close_limit=0.0)))

    fake.queue_param_reply(
        can_id=0x101,
        rid=int(Variable.CTRL_MODE),
        value=int(ControlMode.MIT),
        is_u32=True,
    )
    fake.queue_param_reply(
        can_id=0x101,
        rid=int(Variable.CTRL_MODE),
        value=int(ControlMode.MIT),
        is_u32=True,
    )

    api.enable_for(0.0, mode=ControlMode.MIT)

    assert any(data[7] == 0xFC for _, data in fake.sent_frames)
    assert any(data[7] == 0xFD for _, data in fake.sent_frames)


def test_motor_lifecycle_api_set_zero_sends_zero_command(fake: FakeCanAdapter) -> None:
    motor = MotorClient("unused", _transport=fake)
    api = GloriaGripper(motor, GripperController(motor, GripperControlConfig(open_pos=2.7, close_limit=0.0)))
    api.connect(mode=None, enable=False, apply_limits=False, refresh=False)

    api.set_zero()

    assert any(data[7] == 0xFE for _, data in fake.sent_frames)


def test_motor_lifecycle_api_set_mode_updates_current_mode(fake: FakeCanAdapter) -> None:
    motor = MotorClient("unused", _transport=fake)
    api = GloriaGripper(motor, GripperController(motor, GripperControlConfig(open_pos=2.7, close_limit=0.0)))
    api.connect(mode=None, enable=False, apply_limits=False, refresh=False)
    fake.queue_param_reply(
        can_id=0x101,
        rid=int(Variable.CTRL_MODE),
        value=int(ControlMode.POS_VEL),
        is_u32=True,
    )

    api.set_mode(ControlMode.POS_VEL)

    assert motor.current_mode == ControlMode.POS_VEL


def test_motor_lifecycle_api_send_mit_for_sends_frame_and_disables(fake: FakeCanAdapter) -> None:
    motor = MotorClient("unused", _transport=fake)
    api = GloriaGripper(motor, GripperController(motor, GripperControlConfig(open_pos=2.7, close_limit=0.0)))
    fake.queue_mit_feedback(can_id=0x101, position=1.0, velocity=0.0, torque=0.0)
    fake.queue_param_reply(
        can_id=0x101,
        rid=int(Variable.CTRL_MODE),
        value=int(ControlMode.MIT),
        is_u32=True,
    )

    api.send_mit_for(0.0, kp=0.0, kd=0.8, q=None, dq=0.0, tau=0.0)

    assert any(data[7] == 0xFC for _, data in fake.sent_frames)
    assert any(can_id == 0x01 and data[7] not in (0xFC, 0xFD, 0xFE) for can_id, data in fake.sent_frames)
    assert any(data[7] == 0xFD for _, data in fake.sent_frames)


def test_motor_lifecycle_api_send_pv_for_sends_frame_and_disables(fake: FakeCanAdapter) -> None:
    motor = MotorClient("unused", _transport=fake)
    api = GloriaGripper(motor, GripperController(motor, GripperControlConfig(open_pos=2.7, close_limit=0.0)))
    fake.queue_param_reply(
        can_id=0x101,
        rid=int(Variable.CTRL_MODE),
        value=int(ControlMode.POS_VEL),
        is_u32=True,
    )

    api.send_pv_for(0.0, position=2.0, velocity=0.5)

    assert any(data[7] == 0xFC for _, data in fake.sent_frames)
    assert any(can_id == 0x101 for can_id, _ in fake.sent_frames)
    assert any(data[7] == 0xFD for _, data in fake.sent_frames)
