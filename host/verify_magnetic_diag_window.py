#!/usr/bin/env python3
"""Verify active-only magnetic diagnostic window and recovery prompt."""

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


def row(trace_id: int, accuracy: float, mono_ns: int) -> dict:
    return {
        "host_time": time.time(), "index": trace_id, "finger": "index", "finger_id": 4,
        "link": 1, "sensor_id": 1, "kind": "QUAT", "mag_x": None, "mag_y": None, "mag_z": None,
        "quat_x": 0.0, "quat_y": 0.0, "quat_z": 0.0, "quat_w": 1.0,
        "quat_accuracy": accuracy,
        "packet": {"quat": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0, "accuracy": accuracy}},
        "_trace": {"trace_id": trace_id, "connection_id": 1, "sequence": trace_id,
                   "rx_wall_time_ns": time.time_ns(), "rx_monotonic_ns": mono_ns, "packet_len": 32},
    }


def main() -> int:
    root = tk.Tk()
    root.withdraw()
    app = pose.PoseMonitor(root, auto_connect=False, auto_trace=False)
    test_root = APP / "diagnostics"
    with tempfile.TemporaryDirectory(prefix="mag_diag_window_test_", dir=test_root) as temp:
        app.trace.start_session(temp, metadata={"synthetic": True})
        app.open_mag_diag_window()
        root.update()
        base = 1_000_000_000
        app._handle_quat(row(1, 413.9801, base))
        app._handle_quat(row(2, 12813.9801, base + 10_000_000))
        root.update()
        app.trace.stop_session("diag_window_test")
        active = app.mag_feature_text.get("1.0", tk.END)
        assert "强磁事件" in active, active
        assert "磁场强度超范围" not in active, active
        assert "磁场空间梯度异常" not in active, active
        app._handle_quat(row(3, 13.9801, base + 1_100_000_000))
        app._handle_quat(row(4, 13.9801, base + 2_200_000_000))
        root.update()
        final = app.mag_feature_text.get("1.0", tk.END)
        assert final.strip() == "", final
        assert "强磁干扰已恢复" in app.mag_recovery_var.get(), app.mag_recovery_var.get()
        print("MAGNETIC_DIAG_WINDOW_PASS=True")
        print("ACTIVE_ONLY_LIST=True")
        print("EMPTY_AFTER_FEATURE_EXIT=True")
        print("RECOVERY_NOTICE=True")
    app.on_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())