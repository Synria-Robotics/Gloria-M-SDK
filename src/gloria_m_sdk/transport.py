from __future__ import annotations

"""
Transport abstraction layer — ICanTransport and FakeCanAdapter.

ICanTransport
-------------
A structural Protocol (PEP 544) that defines the five methods any CAN
transport backend must implement.  Both ``SerialCanAdapter`` (real hardware)
and ``FakeCanAdapter`` (in-memory stub) satisfy this interface without
inheriting from it.

FakeCanAdapter
--------------
An in-memory drop-in replacement for ``SerialCanAdapter`` used for testing
without physical hardware.  It records every outgoing frame and lets tests
inject pre-built packets that the controller will "receive"::

    from gloria_m_sdk.transport import FakeCanAdapter
    from gloria_m_sdk import GloriaGripper, ControlMode

    fake = FakeCanAdapter()
    # Tell the fake adapter what to answer when the controller polls:
    fake.queue_mit_feedback(can_id=0x101, position=1.57, velocity=0.0, torque=0.0)

    with GloriaGripper("unused", _transport=fake) as g:
        g.motor.refresh()
        assert abs(g.state.position - 1.57) < 0.01

Using FakeCanAdapter in GloriaGripper
--------------------------------------
Pass it via the private ``_transport`` constructor argument::

    g = GloriaGripper("unused", _transport=FakeCanAdapter())
    g.connect(apply_limits=False)
    # ... test without hardware ...

The ``_transport`` parameter is intentionally private (leading underscore) —
it is a testing hook and not part of the stable public API.
"""

import struct
from typing import Iterable, List, Optional, Protocol, Tuple, runtime_checkable

from .serial_can_adapter import CanPacket


# ---------------------------------------------------------------------------
# Protocol

@runtime_checkable
class ICanTransport(Protocol):
    """Structural interface for CAN transport backends.

    Any object that implements these five members can be used as the
    transport layer for :class:`~gloria_m_sdk.controller.CanController`.
    This enables dependency injection and hardware-free unit testing.

    Built-in implementations:

    - :class:`~gloria_m_sdk.serial_can_adapter.SerialCanAdapter` — real hardware
    - :class:`FakeCanAdapter` — in-memory stub for unit tests
    """

    @property
    def is_open(self) -> bool:
        """``True`` if the transport is ready to send and receive."""
        ...

    def send(self, can_id: int, data8: bytes) -> None:
        """Transmit one 8-byte CAN frame with the given *can_id*."""
        ...

    def read_packets(self) -> List[CanPacket]:
        """Return all currently buffered received packets (non-blocking)."""
        ...

    def read_packets_until(
        self,
        *,
        deadline_s: float,
        poll_interval_s: float = 0.0,
    ) -> Iterable[CanPacket]:
        """Yield packets until *deadline_s* (wall-clock seconds) is reached."""
        ...

    def close(self) -> None:
        """Release any underlying resources."""
        ...


# ---------------------------------------------------------------------------
# In-memory fake

class FakeCanAdapter:
    """In-memory CAN transport for unit tests — no hardware required.

    Outgoing frames are recorded in :attr:`sent_frames`.  Incoming packets
    are returned by :meth:`read_packets` in the order they were enqueued
    with the helper methods below.

    Quick-start example::

        from gloria_m_sdk.transport import FakeCanAdapter
        from gloria_m_sdk import GloriaGripper, ControlMode, Limits

        fake = FakeCanAdapter()
        # Simulate the motor echoing back CTRL_MODE = 2 (POS_VEL)
        fake.queue_param_reply(can_id=0x101, rid=10, value=2, is_u32=True)

        with GloriaGripper("unused", _transport=fake) as g:
            g.motor.set_mode(ControlMode.POS_VEL)
            # No physical port opened — works entirely in memory.

    Attributes
    ----------
    sent_frames:
        List of ``(can_id, data8)`` tuples recorded by each :meth:`send` call.
    """

    def __init__(self) -> None:
        self.sent_frames: List[Tuple[int, bytes]] = []
        self._queued: List[CanPacket] = []

    # ------------------------------------------------------------------
    # ICanTransport interface

    @property
    def is_open(self) -> bool:
        """Always ``True`` for the fake adapter."""
        return True

    def send(self, can_id: int, data8: bytes) -> None:
        """Record the outgoing frame without transmitting anything."""
        if len(data8) != 8:
            raise ValueError("CAN data must be exactly 8 bytes")
        self.sent_frames.append((can_id, bytes(data8)))

    def read_packets(self) -> List[CanPacket]:
        """Drain and return all queued packets."""
        packets, self._queued = list(self._queued), []
        return packets

    def read_packets_until(
        self,
        *,
        deadline_s: float,
        poll_interval_s: float = 0.0,
    ) -> Iterable[CanPacket]:
        """Return all queued packets immediately (ignores deadline)."""
        yield from self.read_packets()

    def close(self) -> None:
        """No-op."""

    def __enter__(self) -> "FakeCanAdapter":
        return self

    def __exit__(self, *_) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Low-level helper

    def queue_packet(self, packet: CanPacket) -> None:
        """Enqueue a raw :class:`~gloria_m_sdk.serial_can_adapter.CanPacket`."""
        self._queued.append(packet)

    # ------------------------------------------------------------------
    # High-level helpers

    def queue_mit_feedback(
        self,
        *,
        can_id: int,
        position: float,
        velocity: float,
        torque: float,
        limits: Optional["Limits"] = None,  # type: ignore[name-defined]
    ) -> None:
        """Enqueue a realistic MIT feedback frame.

        The values are bit-packed using the same MIT layout the real motor
        firmware produces.  The resulting
        :class:`~gloria_m_sdk.serial_can_adapter.CanPacket` (``cmd=0x11``)
        will be processed by :meth:`~gloria_m_sdk.controller.CanController.poll`
        and update the actuator state.

        Parameters
        ----------
        can_id:
            CAN ID to stamp on the packet.  Should match the actuator's
            ``feedback_id`` (default ``0x101``).
        position:
            Simulated motor position [rad].
        velocity:
            Simulated motor velocity [rad/s].
        torque:
            Simulated motor torque [N·m].
        limits:
            MIT scaling limits; defaults to ``Limits(3.14, 10.0, 12.0)``.
        """
        from .types import Limits as _Limits
        from .protocol_mit import float_to_uint

        lim = limits if limits is not None else _Limits(pmax=3.14, vmax=10.0, tmax=12.0)

        q_uint = float_to_uint(position, -lim.pmax, lim.pmax, 16)
        dq_uint = float_to_uint(velocity, -lim.vmax, lim.vmax, 12)
        tau_uint = float_to_uint(torque, -lim.tmax, lim.tmax, 12)

        # MIT feedback layout (see unpack_mit_feedback in protocol_mit.py):
        #   data[0]: motor ID byte (low byte of can_id)
        #   data[1]: q bits [15:8]
        #   data[2]: q bits [7:0]
        #   data[3]: dq bits [11:4]
        #   data[4]: dq bits [3:0] (high nibble) | tau bits [11:8] (low nibble)
        #   data[5]: tau bits [7:0]
        #   data[6:8]: unused (zero)
        data = bytes([
            can_id & 0xFF,
            (q_uint >> 8) & 0xFF,
            q_uint & 0xFF,
            (dq_uint >> 4) & 0xFF,
            ((dq_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F),
            tau_uint & 0xFF,
            0x00,
            0x00,
        ])
        self._queued.append(CanPacket(can_id=can_id, cmd=0x11, data=data))

    def queue_param_reply(
        self,
        *,
        can_id: int,
        rid: int,
        value: "float | int",
        is_u32: bool = False,
    ) -> None:
        """Enqueue a parameter read/write echo reply.

        Simulates the motor echoing back a register value after a read or
        write command.  Used to test :meth:`~gloria_m_sdk.api.param_api.ParamAPI.read`
        and :meth:`~gloria_m_sdk.api.motor_api.MotorAPI.set_mode`.

        Parameters
        ----------
        can_id:
            CAN ID to stamp on the packet (should match the actuator's
            ``feedback_id`` or ``command_id``).
        rid:
            Register ID being echoed.
        value:
            The value the "motor" is echoing back.
        is_u32:
            If ``True``, pack *value* as little-endian uint32; otherwise as
            little-endian float32.
        """
        if is_u32:
            val_bytes = struct.pack("<I", int(value))
        else:
            val_bytes = struct.pack("<f", float(value))
        # layout: data[0:2]=reserved, data[2]=0x33, data[3]=rid, data[4:8]=value
        data = bytes([0x00, 0x00, 0x33, rid & 0xFF]) + val_bytes
        self._queued.append(CanPacket(can_id=can_id, cmd=0x11, data=data))

    def clear(self) -> None:
        """Reset :attr:`sent_frames` and the queued-packet list."""
        self.sent_frames.clear()
        self._queued.clear()

    def last_sent(self) -> Optional[Tuple[int, bytes]]:
        """Return the most recently recorded frame, or ``None`` if nothing sent."""
        return self.sent_frames[-1] if self.sent_frames else None
