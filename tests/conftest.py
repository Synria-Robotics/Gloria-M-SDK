"""
Shared pytest fixtures for Gloria-M SDK tests.

All fixtures here work without hardware — they use FakeCanAdapter.
"""
from __future__ import annotations

import sys
import os

# Allow running tests from the repo root without installing the package.
_SRC = os.path.join(os.path.dirname(__file__), "..", "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

import pytest

from gloria_m_sdk import ControlMode, FakeCanAdapter, GloriaGripper, Limits
from gloria_m_sdk.transport import FakeCanAdapter as _FakeCanAdapter


@pytest.fixture
def fake() -> FakeCanAdapter:
    """A fresh FakeCanAdapter with empty sent_frames and no queued packets."""
    return _FakeCanAdapter()


@pytest.fixture
def gripper(fake: FakeCanAdapter) -> GloriaGripper:
    """
    A connected GloriaGripper backed by a FakeCanAdapter.

    apply_limits=False is used so the fixture does not attempt to write
    parameters to the (non-existent) motor at startup.
    """
    g = GloriaGripper("unused", _transport=fake)
    g.connect(apply_limits=False)
    return g
