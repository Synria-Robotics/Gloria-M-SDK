from __future__ import annotations

from dataclasses import dataclass
import logging
import time
from typing import Iterable, List, Optional, Tuple

import serial

_log = logging.getLogger(__name__)


@dataclass(frozen=True)
class CanPacket:
    can_id: int
    cmd: int
    data: bytes  # 8 bytes


class SerialCanAdapter:
    """
    Minimal serial-to-CAN adapter layer.

    - TX frame: 30 bytes, starting with 0x55 0xAA
    - RX frame: 16 bytes, starting with 0xAA and ending with 0x55
    """

    _TX_TEMPLATE = bytearray(
        [
            0x55,
            0xAA,
            0x1E,
            0x03,
            0x01,
            0x00,
            0x00,
            0x00,
            0x0A,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,  # CANID low
            0x00,  # CANID high
            0x00,
            0x00,
            0x00,
            0x08,  # DLC
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ]
    )

    _RX_HEADER = 0xAA
    _RX_TAIL = 0x55
    _RX_FRAME_LEN = 16

    def __init__(self, port: str, baudrate: int = 921_600, timeout: float = 0.5):
        self._ser = serial.Serial(port, baudrate, timeout=timeout)
        self._rx_remainder = b""

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()

    @property
    def is_open(self) -> bool:
        """True if the underlying serial port is currently open."""
        return bool(self._ser and self._ser.is_open)

    def __enter__(self) -> "SerialCanAdapter":
        if not self._ser.is_open:
            self._ser.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def send(self, can_id: int, data8: bytes) -> None:
        if len(data8) != 8:
            raise ValueError("CAN data must be exactly 8 bytes")
        frame = bytearray(self._TX_TEMPLATE)
        frame[13] = can_id & 0xFF
        frame[14] = (can_id >> 8) & 0xFF
        frame[21:29] = data8
        _log.debug("TX can_id=0x%03X data=%s", can_id, data8.hex())
        self._ser.write(bytes(frame))

    def read_packets(self) -> List[CanPacket]:
        """
        Read the serial buffer and parse available CAN packets (may return an empty list).
        """
        data = self._rx_remainder + self._ser.read_all()
        frames, remainder = self._extract_frames(data)
        self._rx_remainder = remainder

        packets: List[CanPacket] = []
        for frame in frames:
            cmd = frame[1]
            can_id = (frame[6] << 24) | (frame[5] << 16) | (frame[4] << 8) | frame[3]
            payload = bytes(frame[7:15])
            pkt = CanPacket(can_id=can_id, cmd=cmd, data=payload)
            _log.debug("RX can_id=0x%03X cmd=0x%02X data=%s", can_id, cmd, payload.hex())
            packets.append(pkt)
        return packets

    def read_packets_until(
        self,
        *,
        deadline_s: float,
        poll_interval_s: float = 0.0,
    ) -> Iterable[CanPacket]:
        while time.time() < deadline_s:
            pkts = self.read_packets()
            for p in pkts:
                yield p
            if poll_interval_s > 0:
                time.sleep(poll_interval_s)

    def _extract_frames(self, data: bytes) -> Tuple[List[bytes], bytes]:
        frames: List[bytes] = []
        i = 0
        remainder_pos = 0
        n = len(data)
        fl = self._RX_FRAME_LEN
        while i <= n - fl:
            if data[i] == self._RX_HEADER and data[i + fl - 1] == self._RX_TAIL:
                frames.append(data[i : i + fl])
                i += fl
                remainder_pos = i
            else:
                i += 1
        remainder = data[remainder_pos:]
        # Cap the remainder to at most fl-1 bytes: any byte further back than
        # that has already been scanned and cannot start a valid complete frame.
        # Without this cap, persistent garbage input causes unbounded buffer growth.
        if len(remainder) >= fl:
            remainder = remainder[-(fl - 1):]
        return frames, remainder

