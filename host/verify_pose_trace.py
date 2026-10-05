#!/usr/bin/env python3
"""Synthetic end-to-end test of pose processing + trace recording."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import time
import tkinter as tk
from pathlib import Path

APP = Path(__file__).resolve().parent
sys.path.insert(0, str(APP))
spec = importlib.util.spec_from_file_location("pose_app", APP / "姿态测试上位机.pyw")
pose = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pose)


def row(trace_id: int, kind: str, **values):
    base = {
        "host_time": time.time(),
        "index": trace_id,
        "finger": "index",
        "finger_id": 4,
        "link": 1,
        "sensor_id": values.pop("sensor_id"),
        "kind": kind,
        "_trace": {
            "trace_id": trace_id,
            "connection_id": 1,
            "sequence": trace_id,
            "rx_wall_time_ns": time.time_ns(),
            "rx_monotonic_ns": time.monotonic_ns(),
            "packet_len": 32,
        },
    }
    base.update(values)
    return base


def main() -> int:
    root = tk.Tk()
    root.withdraw()
    app = pose.PoseMonitor(root, auto_connect=False, auto_trace=False)
    root = APP / "diagnostics"
    root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="pose_trace_test_", dir=root) as temp:
        app.trace.start_session(temp, metadata={"synthetic": True})
        quat_row = row(
            1,
            "QUAT",
            sensor_id=1,
            mag_x=None,
            mag_y=None,
            mag_z=None,
            quat_x=0.0,
            quat_y=0.0,
            quat_z=0.0,
            quat_w=1.0,
            quat_accuracy=131.0,
            packet={"quat": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0, "accuracy": 131.0}},
        )
        app._handle_quat(quat_row)
        secondary = row(
            2,
            "MAG",
            sensor_id=21,
            mag_x=-40.0,
            mag_y=12.0,
            mag_z=18.0,
            quat_x=None,
            quat_y=None,
            quat_z=None,
            quat_w=None,
            quat_accuracy=None,
            packet={"mag": {"x": -40.0, "y": 12.0, "z": 18.0}},
        )
        primary = row(
            3,
            "MAG",
            sensor_id=1,
            mag_x=-50.0,
            mag_y=10.0,
            mag_z=20.0,
            quat_x=None,
            quat_y=None,
            quat_z=None,
            quat_w=None,
            quat_accuracy=None,
            packet={"mag": {"x": -50.0, "y": 10.0, "z": 20.0}},
        )
        app._handle_mag(secondary)
        app._handle_mag(primary)
        app._draw_compass()
        app._pipeline_flush(time.monotonic() + 2.0)
        stats = app.trace.stats()
        session = app.trace.stop_session("synthetic_test")
        assert session is not None
        assert stats["analysis_rows"] >= 2, stats
        assert stats["stage_records"] >= 2, stats
        assert (session / "analysis.csv").exists(), session
        print("POSE_TRACE_PIPELINE_PASS=True")
        print("SESSION=", session)
        print("STATS=", stats)
    app.on_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())