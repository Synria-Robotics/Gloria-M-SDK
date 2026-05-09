"""
Integration-style tests for GloriaGripper facade using FakeCanAdapter.

All tests run without hardware — the FakeCanAdapter simulates motor replies.
"""
from __future__ import annotations

import pytest

from gloria_m_sdk import (
    ControlMode,
    FakeCanAdapter,
    GloriaGripper,
    Limits,
    PositionRange,
)
from gloria_m_sdk.exceptions import GloriaConnectionError, GloriaModeError
from gloria_m_sdk.registers import Variable


# ---------------------------------------------------------------------------
# Connection management


class TestConnection:
    def test_connect_with_fake_transport(self, fake: FakeCanAdapter) -> None:
        g = GloriaGripper("unused", _transport=fake)
        assert not g.is_connected
        g.connect(apply_limits=False)
        assert g.is_connected

    def test_context_manager(self, fake: FakeCanAdapter) -> None:
        with GloriaGripper("unused", _transport=fake) as g:
            assert g.is_connected
        assert not g.is_connected

    def test_double_connect_is_idempotent(self, fake: FakeCanAdapter) -> None:
        g = GloriaGripper("unused", _transport=fake)
        g.connect(apply_limits=False)
        g.connect(apply_limits=False)  # should not raise
        assert g.is_connected

    def test_api_raises_before_connect(self, fake: FakeCanAdapter) -> None:
        g = GloriaGripper("unused", _transport=fake)
        with pytest.raises(GloriaConnectionError, match="connect"):
            g.motor.enable()

    def test_connect_apply_limits_sends_frames(self, fake: FakeCanAdapter) -> None:
        limits = Limits(pmax=3.14, vmax=10.0, tmax=12.0)
        with GloriaGripper("unused", _transport=fake, limits=limits) as g:
            pass  # apply_limits=True by default
        # expect write PMAX, VMAX, TMAX + save (4 frames to 0x7FF)
        assert len(fake.sent_frames) >= 4


# ---------------------------------------------------------------------------
# Motor API


class TestMotorAPI:
    def test_enable_sends_frame(self, gripper: GloriaGripper, fake: FakeCanAdapter) -> None:
        gripper.motor.enable()
        # Enable command: data[-1] == 0xFC, CAN ID == command_id (0x01)
        assert any(data[7] == 0xFC for _, data in fake.sent_frames)

    def test_disable_sends_frame(self, gripper: GloriaGripper, fake: FakeCanAdapter) -> None:
        gripper.motor.disable()
        assert any(data[7] == 0xFD for _, data in fake.sent_frames)

    def test_set_mode_succeeds_when_motor_echoes_back(
        self, gripper: GloriaGripper, fake: FakeCanAdapter
    ) -> None:
        # Queue the motor's echo: CTRL_MODE (RID=10) == 2 (POS_VEL)
        fake.queue_param_reply(can_id=0x101, rid=int(Variable.CTRL_MODE),
                               value=int(ControlMode.POS_VEL), is_u32=True)
        gripper.motor.set_mode(ControlMode.POS_VEL)
        assert gripper.current_mode == ControlMode.POS_VEL

    def test_set_mode_raises_when_no_echo(
        self, gripper: GloriaGripper, fake: FakeCanAdapter
    ) -> None:
        # No echo queued → GloriaModeError after retry window
        with pytest.raises(GloriaModeError, match="did not confirm"):
            gripper.motor.set_mode(ControlMode.POS_VEL)

    def test_refresh_sends_broadcast_frame(
        self, gripper: GloriaGripper, fake: FakeCanAdapter
    ) -> None:
        gripper.motor.refresh()
        # 0x7FF broadcast with 0xCC in data[2]
        assert any(cid == 0x7FF and data[2] == 0xCC for cid, data in fake.sent_frames)

    def test_refresh_updates_state(
        self, gripper: GloriaGripper, fake: FakeCanAdapter
    ) -> None:
        fake.queue_mit_feedback(can_id=0x101, position=1.5, velocity=0.0, torque=0.0)
        gripper.motor.refresh()
        assert abs(gripper.state.position - 1.5) < 0.02  # 1 LSB tolerance


# ---------------------------------------------------------------------------
# Motion API


class TestMotionAPI:
    def _setup_pv_mode(self, gripper: GloriaGripper, fake: FakeCanAdapter) -> None:
        fake.queue_param_reply(can_id=0x101, rid=int(Variable.CTRL_MODE),
                               value=int(ControlMode.POS_VEL), is_u32=True)
        gripper.motor.set_mode(ControlMode.POS_VEL)

    def _setup_mit_mode(self, gripper: GloriaGripper, fake: FakeCanAdapter) -> None:
        fake.queue_param_reply(can_id=0x101, rid=int(Variable.CTRL_MODE),
                               value=int(ControlMode.MIT), is_u32=True)
        gripper.motor.set_mode(ControlMode.MIT)

    def test_send_pos_vel_requires_pv_mode(
        self, gripper: GloriaGripper, fake: FakeCanAdapter
    ) -> None:
        """send_pos_vel must raise GloriaModeError if motor is in MIT mode."""
        self._setup_mit_mode(gripper, fake)
        with pytest.raises(GloriaModeError, match="POS_VEL"):
            gripper.motion.send_pos_vel(position=1.0, velocity=1.0)

    def test_send_pos_vel_sends_frame(
        self, gripper: GloriaGripper, fake: FakeCanAdapter
    ) -> None:
        self._setup_pv_mode(gripper, fake)
        fake.clear()
        gripper.motion.send_pos_vel(position=1.5, velocity=1.0, poll=False)
        # PV frame goes to 0x100 + command_id
        assert any(cid == 0x101 for cid, _ in fake.sent_frames)

    def test_send_mit_requires_mit_mode(
        self, gripper: GloriaGripper, fake: FakeCanAdapter
    ) -> None:
        self._setup_pv_mode(gripper, fake)
        with pytest.raises(GloriaModeError, match="MIT"):
            gripper.motion.send_mit(kp=0.0, kd=0.5, q=0.0, dq=0.0, tau=-1.0)

    def test_send_mit_sends_frame(
        self, gripper: GloriaGripper, fake: FakeCanAdapter
    ) -> None:
        self._setup_mit_mode(gripper, fake)
        fake.clear()
        gripper.motion.send_mit(kp=0.0, kd=0.5, q=0.0, dq=0.0, tau=-1.0, poll=False)
        assert len(fake.sent_frames) == 1

    def test_position_clamped_to_safe_range(self, fake: FakeCanAdapter) -> None:
        """Positions outside safe_position range are silently clamped."""
        safe = PositionRange(min=0.0, max=2.0)
        with GloriaGripper("unused", _transport=fake, safe_position=safe) as g:
            fake.queue_param_reply(can_id=0x101, rid=int(Variable.CTRL_MODE),
                                   value=int(ControlMode.POS_VEL), is_u32=True)
            g.motor.set_mode(ControlMode.POS_VEL)
            fake.clear()
            g.motion.send_pos_vel(position=99.0, velocity=1.0, poll=False)
        # Exactly one PV frame should have been sent; the value is clamped
        assert len(fake.sent_frames) == 1


# ---------------------------------------------------------------------------
# Param API


class TestParamAPI:
    def test_read_returns_value_when_motor_replies(
        self, gripper: GloriaGripper, fake: FakeCanAdapter
    ) -> None:
        rid = int(Variable.PMAX)
        fake.queue_param_reply(can_id=0x101, rid=rid, value=3.14)
        result = gripper.params.read(rid, timeout_s=0.1)
        assert result is not None
        assert abs(result - 3.14) < 1e-4

    def test_read_returns_none_on_timeout(
        self, gripper: GloriaGripper, fake: FakeCanAdapter
    ) -> None:
        result = gripper.params.read(int(Variable.PMAX), timeout_s=0.01)
        assert result is None

    def test_write_f32_sends_frame(
        self, gripper: GloriaGripper, fake: FakeCanAdapter
    ) -> None:
        fake.clear()
        gripper.params.write_f32(int(Variable.PMAX), 3.14)
        assert len(fake.sent_frames) == 1

    def test_save_sends_aa_command(
        self, gripper: GloriaGripper, fake: FakeCanAdapter
    ) -> None:
        fake.clear()
        gripper.params.save()
        # Save command: data[2] == 0xAA
        assert any(data[2] == 0xAA for _, data in fake.sent_frames)


# ---------------------------------------------------------------------------
# FakeCanAdapter self-tests


class TestFakeCanAdapter:
    def test_records_sent_frames(self, fake: FakeCanAdapter) -> None:
        fake.send(0x01, b"\x00" * 8)
        assert len(fake.sent_frames) == 1
        assert fake.sent_frames[0][0] == 0x01

    def test_bad_length_raises(self, fake: FakeCanAdapter) -> None:
        with pytest.raises(ValueError, match="8 bytes"):
            fake.send(0x01, b"\x00" * 7)

    def test_queue_and_drain(self, fake: FakeCanAdapter) -> None:
        from gloria_m_sdk.serial_can_adapter import CanPacket
        pkt = CanPacket(can_id=0x01, cmd=0x11, data=b"\x00" * 8)
        fake.queue_packet(pkt)
        received = fake.read_packets()
        assert len(received) == 1
        assert received[0].can_id == 0x01
        # Second read is empty
        assert fake.read_packets() == []

    def test_clear(self, fake: FakeCanAdapter) -> None:
        fake.send(0x01, b"\x00" * 8)
        fake.queue_mit_feedback(can_id=0x101, position=0.0, velocity=0.0, torque=0.0)
        fake.clear()
        assert fake.sent_frames == []
        assert fake.read_packets() == []

    def test_is_open(self, fake: FakeCanAdapter) -> None:
        assert fake.is_open is True

    def test_satisfies_protocol(self, fake: FakeCanAdapter) -> None:
        from gloria_m_sdk.transport import ICanTransport
        assert isinstance(fake, ICanTransport)
