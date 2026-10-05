#!/usr/bin/env python3
"""Verify real-time diagnostic gate enter/exit events."""

from __future__ import annotations

import importlib.util
import json
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


def row(trace_id, accuracy, mono_ns):
    return {
        "host_time": time.time(),
        "index": trace_id,
        "finger": "index",
        "finger_id": 4,
        "link": 1,
        "sensor_id": 1,
        "kind": "QUAT",
        "mag_x": None, "mag_y": None, "mag_z": None,
        "quat_x": 0.0, "quat_y": 0.0, "quat_z": 0.0, "quat_w": 1.0,
        "quat_accuracy": accuracy,
        "packet": {"quat": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0, "accuracy": accuracy}},
        "_trace": {
            "trace_id": trace_id, "connection_id": 1, "sequence": trace_id,
            "rx_wall_time_ns": time.time_ns(), "rx_monotonic_ns": mono_ns, "packet_len": 32,
        },
    }


def main():
    root = tk.Tk()
    root.withdraw()
    app = pose.PoseMonitor(root, auto_connect=False, auto_trace=False)
    test_root = APP / "diagnostics"
    with tempfile.TemporaryDirectory(prefix="gate_transition_test_", dir=test_root) as temp:
        session = app.trace.start_session(temp, metadata={"synthetic": True})
        app._handle_quat(row(1, 131.0, 1_000_000_000))
        app._handle_quat(row(2, 413.9801, 1_010_000_000))
        app._handle_quat(row(3, 131.0, 1_020_000_000))
        app.trace.stop_session("gate_test")
        events = [json.loads(line) for line in (session / "gate_events.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
        phases = [(event["gate"], event["phase"]) for event in events]
        assert ("norm_gate", "enter") in phases, phases
        assert ("norm_gate", "exit") in phases, phases
        norm_enter = next(event for event in events if event["gate"] == "norm_gate" and event["phase"] == "enter")
        norm_exit = next(event for event in events if event["gate"] == "norm_gate" and event["phase"] == "exit")
        assert abs(norm_exit["duration_ms"] - 10.0) < 1e-6, (norm_enter, norm_exit)
        print("GATE_TRANSITION_EVENTS_PASS=True")
        print("GATE_EVENTS=", len(events))
        print("PHASES=", phases)
    app.on_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
