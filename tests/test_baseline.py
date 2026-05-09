"""
Unit tests for gripper_baseline.py — TorqueBaseline CSV loading and interpolation.

No hardware required.
"""
from __future__ import annotations

import csv
import io
import math
import pathlib
import tempfile

import pytest

from gloria_m_sdk.gripper_baseline import BaselinePoint, TorqueBaseline


# ---------------------------------------------------------------------------
# Helpers

def _write_csv(rows: list[dict], path: pathlib.Path, fieldnames: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


# ---------------------------------------------------------------------------
# from_csv


class TestFromCsv:
    def test_loads_position_rad_tau_fb_nm_columns(self, tmp_path: pathlib.Path) -> None:
        p = tmp_path / "baseline.csv"
        _write_csv(
            [
                {"position_rad": "0.0", "tau_fb_nm": "0.1"},
                {"position_rad": "1.0", "tau_fb_nm": "0.5"},
                {"position_rad": "2.0", "tau_fb_nm": "0.9"},
            ],
            p,
            ["position_rad", "tau_fb_nm"],
        )
        bl = TorqueBaseline.from_csv(p)
        assert len(bl.points) == 3
        assert bl.points[0].position_rad == pytest.approx(0.0)
        assert bl.points[2].tau_fb_nm    == pytest.approx(0.9)

    def test_loads_mean_columns(self, tmp_path: pathlib.Path) -> None:
        p = tmp_path / "baseline_mean.csv"
        _write_csv(
            [
                {"position_mean_rad": "0.5", "tau_fb_mean_nm": "-0.2"},
                {"position_mean_rad": "1.5", "tau_fb_mean_nm": "-0.6"},
            ],
            p,
            ["position_mean_rad", "tau_fb_mean_nm"],
        )
        bl = TorqueBaseline.from_csv(p)
        assert len(bl.points) == 2

    def test_sorts_by_position(self, tmp_path: pathlib.Path) -> None:
        p = tmp_path / "unsorted.csv"
        _write_csv(
            [
                {"position_rad": "2.0", "tau_fb_nm": "0.9"},
                {"position_rad": "0.0", "tau_fb_nm": "0.1"},
                {"position_rad": "1.0", "tau_fb_nm": "0.5"},
            ],
            p,
            ["position_rad", "tau_fb_nm"],
        )
        bl = TorqueBaseline.from_csv(p)
        positions = [pt.position_rad for pt in bl.points]
        assert positions == sorted(positions)

    def test_empty_csv_raises(self, tmp_path: pathlib.Path) -> None:
        p = tmp_path / "empty.csv"
        p.write_text("position_rad,tau_fb_nm\n")
        with pytest.raises(ValueError, match="empty"):
            TorqueBaseline.from_csv(p)

    def test_one_row_raises(self, tmp_path: pathlib.Path) -> None:
        p = tmp_path / "one_row.csv"
        _write_csv([{"position_rad": "0.0", "tau_fb_nm": "0.1"}], p, ["position_rad", "tau_fb_nm"])
        with pytest.raises(ValueError, match="at least two"):
            TorqueBaseline.from_csv(p)

    def test_unknown_columns_raises(self, tmp_path: pathlib.Path) -> None:
        p = tmp_path / "bad_cols.csv"
        _write_csv([{"x": "0", "y": "1"}, {"x": "1", "y": "2"}], p, ["x", "y"])
        with pytest.raises(ValueError, match="must contain"):
            TorqueBaseline.from_csv(p)


# ---------------------------------------------------------------------------
# tau_at (interpolation)


@pytest.fixture
def simple_baseline() -> TorqueBaseline:
    """Linear baseline: tau = -0.5 * q  over [0, 2]."""
    return TorqueBaseline(points=(
        BaselinePoint(position_rad=0.0, tau_fb_nm=0.0),
        BaselinePoint(position_rad=1.0, tau_fb_nm=-0.5),
        BaselinePoint(position_rad=2.0, tau_fb_nm=-1.0),
    ))


class TestTauAt:
    def test_exact_point(self, simple_baseline: TorqueBaseline) -> None:
        assert simple_baseline.tau_at(1.0) == pytest.approx(-0.5)

    def test_midpoint_interpolation(self, simple_baseline: TorqueBaseline) -> None:
        assert simple_baseline.tau_at(0.5) == pytest.approx(-0.25)

    def test_clamp_below_min(self, simple_baseline: TorqueBaseline) -> None:
        assert simple_baseline.tau_at(-10.0) == pytest.approx(0.0)

    def test_clamp_above_max(self, simple_baseline: TorqueBaseline) -> None:
        assert simple_baseline.tau_at(10.0) == pytest.approx(-1.0)


# ---------------------------------------------------------------------------
# closing_delta_tau


class TestClosingDeltaTau:
    def test_no_contact(self, simple_baseline: TorqueBaseline) -> None:
        """When feedback torque equals baseline, contact force is zero."""
        # At q=1.0, baseline tau = -0.5 Nm. Matching feedback → delta = 0.
        delta = simple_baseline.closing_delta_tau(1.0, -0.5)
        assert delta == pytest.approx(0.0)

    def test_contact_force_detected(self, simple_baseline: TorqueBaseline) -> None:
        """When feedback torque exceeds baseline by 0.3 Nm, delta = 0.3."""
        # Baseline at q=1.0 is -0.5 Nm (closing_tau = 0.5).
        # Feedback of -0.8 Nm → closing = 0.8 → delta = 0.3.
        delta = simple_baseline.closing_delta_tau(1.0, -0.8)
        assert delta == pytest.approx(0.3, abs=1e-9)

    def test_opening_direction_is_zero(self, simple_baseline: TorqueBaseline) -> None:
        """Positive (opening-direction) feedback never produces a contact force."""
        delta = simple_baseline.closing_delta_tau(0.5, +1.0)
        assert delta == pytest.approx(0.0)
