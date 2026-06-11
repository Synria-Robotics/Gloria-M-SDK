from __future__ import annotations

import threading
import traceback
from argparse import Namespace
from pathlib import Path
from typing import Callable

from .live_current_monitor import LiveCurrentMonitor


def repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def parse_int(value: str) -> int:
    return int(value, 0)


def resolve_port(port: str) -> str:
    """Return *port* unchanged, or auto-detect a single available COM port."""
    if port and port.lower() != "auto":
        return port
    from serial.tools import list_ports

    found = list(list_ports.comports())
    if not found:
        raise SystemExit("[port] No serial ports found. Plug in the serial-to-CAN adapter.")
    if len(found) > 1:
        names = ", ".join(p.device for p in found)
        raise SystemExit(f"[port] Multiple serial ports found ({names}); pass --port explicitly.")
    print(f"[port] auto-detected {found[0].device} ({found[0].description})")
    return found[0].device


def run_demo_with_monitor(
    args: Namespace,
    *,
    demo_name: str,
    title: str,
    thread_name: str,
    run_demo: Callable[[Namespace, LiveCurrentMonitor], None],
) -> int:
    monitor = LiveCurrentMonitor(
        title=title,
        refresh_hz=args.monitor_hz,
        demo_name=demo_name,
        log_root=repo_root() / "log",
    )
    result = {"code": 0}

    def worker() -> None:
        try:
            run_demo(args, monitor)
        except Exception as exc:
            result["code"] = 1
            monitor.set_status("Error", message=str(exc))
            traceback.print_exc()

    thread = threading.Thread(target=worker, name=thread_name)
    thread.start()
    try:
        monitor.run()
    except KeyboardInterrupt:
        monitor.close()
    thread.join()

    log_path = monitor.save_current_log()
    if log_path is not None:
        print(f"[log] current csv: {log_path}")
    else:
        print("[log] no valid current samples captured")
    return int(result["code"])

