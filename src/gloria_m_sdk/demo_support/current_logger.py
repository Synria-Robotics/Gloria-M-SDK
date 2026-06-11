from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
from typing import Optional, TextIO, Tuple


CurrentSample = Tuple[float, float]


class CurrentLogWriter:
    """Stream estimated-current samples and keep a bounded per-demo history."""

    def __init__(
        self,
        *,
        demo_name: str,
        log_root: Path,
        max_files: int = 5,
        flush_interval_s: float = 1.0,
    ) -> None:
        self.demo_name = demo_name
        self.log_dir = Path(log_root) / demo_name
        self.max_files = max(1, int(max_files))
        self.flush_interval_s = max(0.0, float(flush_interval_s))

        self._file: Optional[TextIO] = None
        self._writer: Optional[csv.writer] = None
        self._active_path: Optional[Path] = None
        self._started_at_mono: Optional[float] = None
        self._last_sample_mono: Optional[float] = None
        self._last_flush_mono: Optional[float] = None

        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.cleanup_active_files()

    @property
    def active_path(self) -> Optional[Path]:
        return self._active_path

    def append_sample(self, monotonic_s: float, current_a: float) -> Optional[Path]:
        now = float(monotonic_s)
        if self._file is None:
            self._open_active(now)

        if self._writer is None or self._started_at_mono is None:
            return None

        elapsed_s = max(0.0, now - self._started_at_mono)
        self._writer.writerow(
            [
                f"{elapsed_s:.6f}",
                f"{float(current_a):+.6f}",
                f"{elapsed_s:.6f}",
            ]
        )
        self._last_sample_mono = now

        if self._last_flush_mono is None or now - self._last_flush_mono >= self.flush_interval_s:
            self.flush()
            self._last_flush_mono = now
        return self._active_path

    def flush(self) -> None:
        if self._file is not None:
            self._file.flush()

    def finalize(self, *, ended_at: Optional[datetime] = None) -> Optional[Path]:
        if self._active_path is None or self._started_at_mono is None or self._last_sample_mono is None:
            self.close()
            return None

        self.close()

        end_time = ended_at or datetime.now()
        timestamp = end_time.strftime("%Y%m%d_%H%M%S")
        duration_s = max(0.0, self._last_sample_mono - self._started_at_mono)
        final_path = self.log_dir / f"{self.demo_name}_{timestamp}_{duration_s:.3f}s.csv"
        if final_path.exists():
            final_path.unlink()
        self._active_path.replace(final_path)
        self._active_path = None

        self._trim_final_logs()
        return final_path

    def close(self) -> None:
        if self._file is not None:
            self._file.flush()
            self._file.close()
        self._file = None
        self._writer = None

    def cleanup_active_files(self) -> None:
        for path in self.log_dir.glob(f"{self.demo_name}_active_*.csv"):
            path.unlink(missing_ok=True)

    def _open_active(self, monotonic_s: float) -> None:
        self._started_at_mono = float(monotonic_s)
        self._last_sample_mono = None
        self._last_flush_mono = None

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._active_path = self.log_dir / f"{self.demo_name}_active_{timestamp}.csv"
        if self._active_path.exists():
            self._active_path.unlink()

        self._file = self._active_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(["elapsed_s", "current_a", "recorded_duration_s"])
        self.flush()

    def _trim_final_logs(self) -> None:
        existing = self._existing_final_logs()
        while len(existing) > self.max_files:
            existing.pop(0).unlink(missing_ok=True)

    def _existing_final_logs(self) -> list[Path]:
        if not self.log_dir.exists():
            return []
        return sorted(
            (
                path
                for path in self.log_dir.glob(f"{self.demo_name}_*.csv")
                if f"{self.demo_name}_active_" not in path.name
            ),
            key=lambda path: (path.stat().st_mtime, path.name),
        )

