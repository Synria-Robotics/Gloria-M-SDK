from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Any, Deque, Optional, Tuple

DEFAULT_KT_NM_PER_A = 1.0


def estimate_current_from_torque(
    torque_nm: Optional[float],
    kt_nm_per_a: Optional[float],
) -> Tuple[Optional[float], Optional[float]]:
    """Estimate motor current from feedback torque and torque constant."""
    if torque_nm is None:
        return None, None
    if not math.isfinite(float(torque_nm)):
        return None, None

    kt = float(kt_nm_per_a) if LiveCurrentMonitor._is_valid_kt(kt_nm_per_a) else DEFAULT_KT_NM_PER_A
    signed_current = float(torque_nm) / kt
    return signed_current, abs(signed_current)


class LiveCurrentMonitor:
    """Small Tk window used by demos to display live estimated current curve."""

    def __init__(self, *, title: str, refresh_hz: float = 10.0) -> None:
        self.stop_event = threading.Event()
        self._lock = threading.Lock()
        self._refresh_ms = max(50, int(1000.0 / max(1.0, float(refresh_hz))))
        self._title = title
        self._root: Any = None
        self._canvas: Any = None
        self._header_var: Any = None
        self._status_var: Any = None
        self._current_var: Any = None
        self._range_var: Any = None
        self._history: Deque[Tuple[float, float]] = deque()
        self._window_s = 20.0
        self._started_at: Optional[float] = None
        self._sample: dict[str, Any] = {
            "status": "Starting",
            "current": None,
        }

    def update(self, **values: Any) -> None:
        with self._lock:
            self._sample.update(values)
            current = values.get("current")
            if "current" in values and self._is_valid_number(current):
                now = time.perf_counter()
                if self._started_at is None:
                    self._started_at = now
                self._history.append((now, float(current)))
                self._trim_history(now)

    def set_status(self, status: str, *, message: Optional[str] = None) -> None:
        values: dict[str, Any] = {"status": status}
        if message is not None:
            values["message"] = message
        self.update(**values)

    def should_stop(self) -> bool:
        return self.stop_event.is_set()

    def close(self) -> None:
        self.stop_event.set()
        root = self._root
        self._root = None
        self._canvas = None
        if root is not None:
            root.destroy()

    def run(self) -> None:
        import tkinter as tk
        from tkinter import ttk

        root = tk.Tk()
        self._root = root
        root.title(self._title)
        root.geometry("640x540+40+40")
        root.minsize(560, 480)
        root.protocol("WM_DELETE_WINDOW", self.close)

        frame = ttk.Frame(root, padding=16)
        frame.grid(row=0, column=0, sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(3, weight=1)

        self._header_var = tk.StringVar(value="24V Motor    Waiting")
        self._status_var = tk.StringVar(value="Waiting")
        ttk.Label(frame, textvariable=self._header_var, font=("", 14, "bold")).grid(
            row=0, column=0, sticky="w", pady=(0, 8)
        )

        self._current_var = tk.StringVar(value="Current: N/A")
        ttk.Label(frame, textvariable=self._current_var, font=("", 12)).grid(
            row=1, column=0, sticky="w", pady=(0, 4)
        )

        self._range_var = tk.StringVar(value="Range: N/A")
        ttk.Label(frame, textvariable=self._range_var, font=("", 11)).grid(
            row=2, column=0, sticky="w", pady=(0, 10)
        )

        self._canvas = tk.Canvas(frame, background="#ffffff", highlightthickness=1, highlightbackground="#c8c8c8")
        self._canvas.grid(row=3, column=0, sticky="nsew")

        self._refresh()
        root.mainloop()
        self.stop_event.set()

    def _refresh(self) -> None:
        if self._root is None or self.stop_event.is_set():
            return

        with self._lock:
            sample = dict(self._sample)
            history = list(self._history)

        current = sample.get("current")
        visible_points = self._visible_points(history)
        if self._current_var is not None:
            self._current_var.set(self._format_current(current))
        if self._range_var is not None:
            self._range_var.set(self._format_range(visible_points))
        if self._status_var is not None:
            status = self._display_status(sample.get("status"), current)
            self._status_var.set(status)
            if self._header_var is not None:
                self._header_var.set(f"24V Motor    {status}")
        self._draw_curve(visible_points)

        if not self.stop_event.is_set():
            self._root.after(self._refresh_ms, self._refresh)

    def _visible_points(self, history: list[Tuple[float, float]]) -> list[Tuple[float, float]]:
        now = time.perf_counter()
        return [(ts, val) for ts, val in history if now - ts <= self._window_s]

    def _trim_history(self, now: float) -> None:
        keep_s = self._window_s * 2.0
        while self._history and now - self._history[0][0] > keep_s:
            self._history.popleft()

    def _draw_curve(self, points: list[Tuple[float, float]]) -> None:
        if self._canvas is None:
            return

        canvas = self._canvas
        canvas.delete("all")
        width = max(1, int(canvas.winfo_width()))
        height = max(1, int(canvas.winfo_height()))
        plot_left = 70
        plot_top = 26
        plot_right = max(plot_left + 1, width - 22)
        plot_bottom = max(plot_top + 1, height - 125)

        canvas.create_rectangle(plot_left, plot_top, plot_right, plot_bottom, outline="#dddddd", fill="#ffffff")

        now = time.perf_counter()
        values = [value for _, value in points]
        limit = max(0.5, max((abs(value) for value in values), default=0.0) * 1.15)
        elapsed_end, elapsed_start = self._elapsed_window(now)
        self._draw_axes(canvas, plot_left, plot_top, plot_right, plot_bottom, limit, elapsed_start, elapsed_end)

        xy: list[float] = []
        started_at = self._started_at or now
        for ts, value in points:
            elapsed = max(0.0, ts - started_at)
            x = self._map_x(elapsed, elapsed_start, elapsed_end, plot_left, plot_right)
            y = self._map_y(value, -limit, limit, plot_top, plot_bottom)
            xy.extend([x, y])

        if not xy:
            canvas.create_text(
                (plot_left + plot_right) / 2,
                (plot_top + plot_bottom) / 2,
                text="Waiting for current data",
                fill="#777777",
            )
        elif len(xy) >= 4:
            canvas.create_line(*xy, fill="#0b6efd", width=2, smooth=True)
        else:
            canvas.create_oval(xy[0] - 2, xy[1] - 2, xy[0] + 2, xy[1] + 2, fill="#0b6efd", outline="")

    def _draw_axes(
        self,
        canvas: Any,
        plot_left: int,
        plot_top: int,
        plot_right: int,
        plot_bottom: int,
        limit: float,
        elapsed_start: float,
        elapsed_end: float,
    ) -> None:
        grid_color = "#eeeeee"
        axis_color = "#888888"
        text_color = "#555555"

        y_ticks = [limit, limit / 2.0, 0.0, -limit / 2.0, -limit]
        for value in y_ticks:
            y = self._map_y(value, -limit, limit, plot_top, plot_bottom)
            color = "#b7b7b7" if abs(value) <= 1e-12 else grid_color
            canvas.create_line(plot_left, y, plot_right, y, fill=color)
            canvas.create_text(
                plot_left - 8,
                y,
                text=f"{value:+.2f}",
                anchor="e",
                fill=text_color,
            )

        divisions = 4
        for index in range(divisions + 1):
            ratio = index / divisions
            x = plot_left + ratio * (plot_right - plot_left)
            elapsed = elapsed_start + ratio * (elapsed_end - elapsed_start)
            label = f"{elapsed:.1f}s" if elapsed_end < 10.0 else f"{elapsed:.0f}s"
            canvas.create_line(x, plot_top, x, plot_bottom, fill=grid_color)
            canvas.create_text(x, plot_bottom + 14, text=label, anchor="n", fill=text_color)

        canvas.create_line(plot_left, plot_top, plot_left, plot_bottom, fill=axis_color, width=1)
        canvas.create_line(plot_left, plot_bottom, plot_right, plot_bottom, fill=axis_color, width=1)
        canvas.create_text(plot_left, plot_top - 10, text="Current (A)", anchor="w", fill=text_color)
        canvas.create_text(plot_right - 6, plot_bottom - 12, text="Time (s)", anchor="e", fill=text_color)

    def _elapsed_window(self, now: float) -> Tuple[float, float]:
        if self._started_at is None:
            return self._window_s, 0.0
        elapsed_end = max(0.001, now - self._started_at)
        if elapsed_end <= self._window_s:
            return self._window_s, 0.0
        elapsed_start = elapsed_end - self._window_s
        return elapsed_end, elapsed_start

    @staticmethod
    def _map_x(value: float, low: float, high: float, left: int, right: int) -> float:
        ratio = (value - low) / max(1e-9, high - low)
        ratio = max(0.0, min(1.0, ratio))
        return left + ratio * (right - left)

    @staticmethod
    def _map_y(value: float, low: float, high: float, top: int, bottom: int) -> float:
        ratio = (value - low) / max(1e-9, high - low)
        return bottom - ratio * (bottom - top)

    @staticmethod
    def _format_current(value: Any) -> str:
        if not LiveCurrentMonitor._is_valid_number(value):
            return "Current: N/A"
        return f"Current: {float(value):+.3f} A"

    @staticmethod
    def _format_range(points: list[Tuple[float, float]]) -> str:
        if not points:
            return "Range: N/A"
        values = [value for _, value in points]
        return f"Range: {min(values):+.3f} to {max(values):+.3f} A"

    @staticmethod
    def _display_status(status: Any, current: Any) -> str:
        status_text = str(status or "")
        if status_text in {"Error", "Stopped"}:
            return status_text
        if LiveCurrentMonitor._is_valid_number(current):
            return "Normal"
        return "Waiting"

    @staticmethod
    def _is_valid_kt(value: Any) -> bool:
        return LiveCurrentMonitor._is_valid_number(value) and abs(float(value)) > 1e-9

    @staticmethod
    def _is_valid_number(value: Any) -> bool:
        if value is None:
            return False
        try:
            return math.isfinite(float(value))
        except (TypeError, ValueError):
            return False


def update_monitor_from_state(
    monitor: LiveCurrentMonitor,
    state: Any,
    *,
    kt_value: Optional[float],
    status: Optional[str] = None,
    phase: Optional[str] = None,
    message: Optional[str] = None,
) -> None:
    current, current_abs = estimate_current_from_torque(state.torque, kt_value)
    values = {
        "current": current,
    }
    if status is not None:
        values["status"] = status
    if phase is not None:
        values["phase"] = phase
    if message is not None:
        values["message"] = message
    monitor.update(**values)
