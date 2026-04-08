from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Tuple


@dataclass(frozen=True)
class BaselinePoint:
    position_rad: float
    tau_fb_nm: float


@dataclass(frozen=True)
class TorqueBaseline:
    """
    空载扭矩基线。

    基线文件通常来自 `07_mit_close_test.py` 生成的 `*_binned.csv`，
    用于根据当前位置 q 插值得到空载下的反馈力矩 `tau_free(q)`。
    """

    points: Tuple[BaselinePoint, ...]

    @classmethod
    def from_csv(cls, path: str | Path) -> "TorqueBaseline":
        csv_path = Path(path)
        with csv_path.open("r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            rows = list(reader)

        if not rows:
            raise ValueError(f"baseline csv is empty: {csv_path}")

        points: List[BaselinePoint] = []
        for row in rows:
            if "position_mean_rad" in row and "tau_fb_mean_nm" in row:
                q = float(row["position_mean_rad"])
                tau = float(row["tau_fb_mean_nm"])
            elif "position_rad" in row and "tau_fb_nm" in row:
                q = float(row["position_rad"])
                tau = float(row["tau_fb_nm"])
            else:
                raise ValueError(
                    "baseline csv must contain either "
                    "`position_mean_rad,tau_fb_mean_nm` or `position_rad,tau_fb_nm`"
                )
            points.append(BaselinePoint(position_rad=q, tau_fb_nm=tau))

        if len(points) < 2:
            raise ValueError("baseline csv needs at least two samples")

        points.sort(key=lambda item: item.position_rad)
        return cls(points=tuple(points))

    def tau_at(self, q: float) -> float:
        pts = self.points
        if q <= pts[0].position_rad:
            return pts[0].tau_fb_nm
        if q >= pts[-1].position_rad:
            return pts[-1].tau_fb_nm

        for idx in range(1, len(pts)):
            p0 = pts[idx - 1]
            p1 = pts[idx]
            if q <= p1.position_rad:
                span = p1.position_rad - p0.position_rad
                if abs(span) <= 1e-9:
                    return p1.tau_fb_nm
                ratio = (q - p0.position_rad) / span
                return p0.tau_fb_nm + (p1.tau_fb_nm - p0.tau_fb_nm) * ratio
        return pts[-1].tau_fb_nm

    def closing_tau_at(self, q: float) -> float:
        return max(0.0, -self.tau_at(q))

    def closing_delta_tau(self, q: float, tau_fb_nm: float) -> float:
        current = max(0.0, -float(tau_fb_nm))
        baseline = self.closing_tau_at(q)
        return max(0.0, current - baseline)
