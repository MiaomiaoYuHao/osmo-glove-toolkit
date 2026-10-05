#!/usr/bin/env python3
"""Synthetic test of data-preview row tracing."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import time
import tkinter as tk
from pathlib import Path

APP = Path(__file__).resolve().parent
sys.path.insert(0, str(APP))
spec = importlib.util.spec_from_file_location("preview_app", APP / "纯数据预览.pyw")
preview = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preview)


def main() -> int:
    root = tk.Tk()
    root.withdraw()
    app = preview.MonitorApp(root, auto_connect=False, auto_trace=False)
    test_root = APP / "diagnostics"
    test_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="preview_trace_test_", dir=test_root) as temp:
        app.trace.start_session(temp, metadata={"synthetic": True})
        app._handle_row({
            "host_time": time.time(),
            "index": 9,
            "finger": "index",
            "finger_id": 4,
            "link": 1,
            "sensor_id": 1,
            "kind": "QUAT",
            "mag_x": None,
            "mag_y": None,
            "mag_z": None,
            "quat_x": 0.0,
            "quat_y": 0.0,
            "quat_z": 0.0,
            "quat_w": 1.0,
            "quat_accuracy": 131.0,
            "packet": {"quat": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0, "accuracy": 131.0}},
            "_trace": {
                "trace_id": 1,
                "connection_id": 1,
                "sequence": 1,
                "rx_wall_time_ns": time.time_ns(),
                "rx_monotonic_ns": time.monotonic_ns(),
                "packet_len": 32,
            },
        })
        session = app.trace.stop_session("synthetic_test")
        assert session is not None
        import json
        stats = json.loads((session / "manifest.json").read_text(encoding="utf-8"))["stats"]
        assert stats["analysis_rows"] == 1, stats
        assert stats["stage_records"] == 1, stats
        print("PREVIEW_TRACE_PIPELINE_PASS=True")
        print("SESSION=", session)
        print("STATS=", stats)
    app.on_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())