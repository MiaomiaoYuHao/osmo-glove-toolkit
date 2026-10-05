#!/usr/bin/env python3
"""Offline acceptance test for the trace recorder format."""

from __future__ import annotations

import tempfile
from pathlib import Path

from trace_recorder import TraceRecorder
from trace_session import summarize_session, verify_session


def main() -> int:
    root = Path(__file__).resolve().parent / "diagnostics"
    root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="trace_selftest_", dir=root) as temp:
        recorder = TraceRecorder("trace_selftest", __file__)
        session = recorder.start_session(temp, metadata={"synthetic": True})
        raw = b"\x01\x02\x03\x04"
        recorder.record_frame(
            trace_id=1,
            connection_id=1,
            sequence=1,
            rx_wall_time_ns=1_800_000_000_000_000_000,
            rx_monotonic_ns=123_456,
            raw_packet=raw,
            decoded={"index": 7, "sensorId": 1, "mag": {"x": 1.0, "y": 2.0, "z": 3.0}},
            decode_error=None,
            kind="MAG",
        )
        recorder.record_serial_bytes(
            connection_id=1,
            data=raw + b"\x00",
            first_rx_monotonic_ns=123_000,
            last_rx_monotonic_ns=123_999,
        )
        recorder.record_firmware_line(
            connection_id=1,
            line="GQ f4 l1 id13 acc0 raw 1 2 3 4 yawc -100",
            parsed={"type": "gq", "raw": "GQ", "fields": {"sensor_id": 13}},
            rx_wall_time_ns=1_800_000_000_000_000_001,
            rx_monotonic_ns=123_500,
        )
        recorder.record_gate_event({"source": "accuracy", "phase": "enter", "gate": "norm_gate", "new_value": 1})
        recorder.record_stage("synthetic_stage", {"trace_id": 1, "value": 3.0})
        recorder.record_analysis({"trace_id": 1, "kind": "MAG", "accepted": True, "mag_x": 1.0})
        recorder.record_system({"event": "synthetic_system", "healthy": True})
        recorder.record_event("synthetic", "test_event", {"ok": True})
        recorder.stop_session("selftest")
        verification = verify_session(session)
        summary = summarize_session(session)
        assert verification["verified"], verification
        assert summary["counts"]["frames"] == 1, summary
        assert summary["counts"]["firmware_lines"] == 1, summary
        assert summary["counts"]["gate_events"] == 1, summary
        assert (session / "raw_frames.bin").read_bytes().startswith(b"OSMOTRC1")
        assert (session / "serial_stream.bin").read_bytes() == raw + b"\x00"
        assert verification["serial_stream_bytes"] == len(raw) + 1, verification
        print("TRACE_RECORDER_SELFTEST_PASS=True")
        print("TRACE_DIR=", session)
        print("VERIFY_ISSUES=", verification["issues"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())