from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Iterable, List, Optional, Tuple

import serial


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
            packets.append(CanPacket(can_id=can_id, cmd=cmd, data=payload))
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
        return frames, data[remainder_pos:]

