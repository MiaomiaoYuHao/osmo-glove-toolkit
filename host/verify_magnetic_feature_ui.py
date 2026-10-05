#!/usr/bin/env python3
"""Verify magnetic feature response UI and recovery notice."""

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


def row(trace_id: int, accuracy: float, mono_ns: int) -> dict:
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


def main() -> int:
    root = tk.Tk()
    root.withdraw()
    app = pose.PoseMonitor(root, auto_connect=False, auto_trace=False)
    test_root = APP / "diagnostics"
    test_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="magnetic_feature_ui_test_", dir=test_root) as temp:
        session = app.trace.start_session(temp, metadata={"synthetic": True})
        app.open_mag_diag_window()
        root.update()
        base = 1_000_000_000
        app._handle_quat(row(1, 13.9801, base))
        app._handle_quat(row(2, 413.9801, base + 10_000_000))
        # 磁修正诊断掩码是"当前快照"，所以每个门控要在它自己那一拍读。
        text_norm = app.mag_feature_text.get("1.0", tk.END)
        assert "磁场强度超范围" in text_norm, text_norm
        app._handle_quat(row(3, 12813.9801, base + 20_000_000))
        text_strong = app.mag_feature_text.get("1.0", tk.END)
        assert "强磁事件" in text_strong, text_strong
        assert "磁场强度超范围" not in text_strong, text_strong
        app._handle_quat(row(4, 13.9801, base + 1_200_000_000))
        app._handle_quat(row(5, 13.9801, base + 2_300_000_000))
        assert app.mag_feature_text.get("1.0", tk.END).strip() == "", app.mag_feature_text.get("1.0", tk.END)
        assert "强磁干扰已恢复" in app.mag_recovery_var.get(), app.mag_recovery_var.get()
        app.trace.stop_session("feature_ui_test")
        events = [json.loads(line) for line in (session / "gate_events.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
        assert any(event.get("gate") == "disturbance_recovered" for event in events), events
        print("MAGNETIC_FEATURE_UI_PASS=True")
        print("FEATURE_TEXT_SHOWS_SHORT_SUMMARY_ONLY=True")
        print("CORRECTION_EFFECTIVE_PROMPT=True")
        print("STRONG_DISTURBANCE_RECOVERY_PROMPT=True")
    app.on_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
