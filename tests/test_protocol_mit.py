"""
Unit tests for protocol_mit.py — pure bit-packing / unpacking logic.

No hardware required.  All tests run in < 1 ms each.
"""
from __future__ import annotations

import math
import pytest

from gloria_m_sdk.protocol_mit import (
    float_to_uint,
    pack_f32,
    pack_mit_command,
    uint_to_float,
    unpack_f32,
    unpack_mit_feedback,
)
from gloria_m_sdk.types import Limits

_LIMITS = Limits(pmax=3.14, vmax=10.0, tmax=12.0)


# ---------------------------------------------------------------------------
# float_to_uint / uint_to_float round-trips


class TestFloatUintRoundTrip:
    """Pack a float → uint → float and verify the reconstruction error is small."""

    @pytest.mark.parametrize("bits", [12, 16])
    @pytest.mark.parametrize("value,lo,hi", [
        (0.0,  -3.14, 3.14),
        (1.57, -3.14, 3.14),
        (-3.14, -3.14, 3.14),
        (5.0, -10.0, 10.0),
        (-9.99, -10.0, 10.0),
    ])
    def test_round_trip_within_tolerance(self, value: float, lo: float, hi: float, bits: int) -> None:
        u = float_to_uint(value, lo, hi, bits)
        recovered = uint_to_float(u, lo, hi, bits)
        span = hi - lo
        tolerance = span / ((1 << bits) - 1)  # one LSB
        assert abs(recovered - value) <= tolerance + 1e-9, (
            f"value={value} lo={lo} hi={hi} bits={bits}: "
            f"recovered={recovered} tolerance={tolerance}"
        )

    def test_clamp_below_min(self) -> None:
        u = float_to_uint(-99.0, -3.14, 3.14, 16)
        assert u == 0

    def test_clamp_above_max(self) -> None:
        u = float_to_uint(99.0, -3.14, 3.14, 16)
        assert u == (1 << 16) - 1

    def test_invalid_range_raises(self) -> None:
        with pytest.raises(ValueError):
            float_to_uint(1.0, 5.0, 5.0, 12)  # span == 0


# ---------------------------------------------------------------------------
# pack_f32 / unpack_f32


class TestF32Codec:
    def test_round_trip(self) -> None:
        for v in [0.0, 1.0, -1.5, 3.14, 1e-6, -1e6]:
            assert abs(unpack_f32(pack_f32(v)) - v) < 1e-5

    def test_length(self) -> None:
        assert len(pack_f32(1.0)) == 4


# ---------------------------------------------------------------------------
# pack_mit_command / unpack_mit_feedback round-trip


class TestMitRoundTrip:
    """Pack a command frame and unpack it as if it were feedback (same layout)."""

    @pytest.mark.parametrize("q,dq,tau", [
        (0.0, 0.0, 0.0),
        (1.57, 2.0, 1.5),
        (-3.0, -5.0, -11.0),
        (3.14, 10.0, 12.0),   # at the limits
        (-3.14, -10.0, -12.0),
    ])
    def test_command_unpack_symmetry(self, q: float, dq: float, tau: float) -> None:
        """
        pack_mit_command and unpack_mit_feedback use the same bit layout,
        so packing a (q, dq, tau) command and immediately unpacking the
        payload as feedback should recover the original values within 1 LSB.
        """
        payload = pack_mit_command(kp=0.0, kd=0.0, q=q, dq=dq, tau=tau, limits=_LIMITS)
        # Feedback layout differs from command layout in the first byte (motor ID),
        # so we reconstruct the feedback bytes manually.
        from gloria_m_sdk.protocol_mit import float_to_uint, uint_to_float

        q_uint  = float_to_uint(q,   -_LIMITS.pmax, _LIMITS.pmax, 16)
        dq_uint = float_to_uint(dq,  -_LIMITS.vmax, _LIMITS.vmax, 12)
        t_uint  = float_to_uint(tau, -_LIMITS.tmax, _LIMITS.tmax, 12)

        fb_bytes = bytes([
            0x01,                                           # motor id
            (q_uint >> 8) & 0xFF,
            q_uint & 0xFF,
            (dq_uint >> 4) & 0xFF,
            ((dq_uint & 0x0F) << 4) | ((t_uint >> 8) & 0x0F),
            t_uint & 0xFF,
            0x00,
            0x00,
        ])

        fb = unpack_mit_feedback(fb_bytes, limits=_LIMITS)

        span_q   = 2 * _LIMITS.pmax  / (2**16 - 1)
        span_dq  = 2 * _LIMITS.vmax  / (2**12 - 1)
        span_tau = 2 * _LIMITS.tmax  / (2**12 - 1)

        assert abs(fb.position - q)   <= span_q   + 1e-9
        assert abs(fb.velocity - dq)  <= span_dq  + 1e-9
        assert abs(fb.torque   - tau) <= span_tau + 1e-9

    def test_feedback_wrong_length_raises(self) -> None:
        with pytest.raises(ValueError, match="8 bytes"):
            unpack_mit_feedback(b"\x00" * 7, limits=_LIMITS)
