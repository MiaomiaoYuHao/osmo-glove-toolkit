#!/usr/bin/env python3
"""End-to-end test: raw COBS frames -> recorded trace -> offline replay."""

from __future__ import annotations

import json
import sys
import tempfile
import time
from pathlib import Path

from cobs import cobs

APP = Path(__file__).resolve().parent
sys.path.insert(0, str(APP))

import read_osmo_glove as glove  # noqa: E402
from trace_recorder import TraceRecorder  # noqa: E402
from trace_replay import replay_pose  # noqa: E402


def make_quat(index: int, x: float, y: float, z: float, w: float):
    message = glove.bpb.Data(index=index, link=1, sensor_id=1)
    message.finger = glove.bpb.Finger.index
    message.quat = glove.bpb.Quat(x=x, y=y, z=z, w=w, accuracy=131.0)
    return cobs.encode(bytes(message))


def make_mag(index: int, x: float, y: float, z: float):
    message = glove.bpb.Data(index=index, link=1, sensor_id=1)
    message.finger = glove.bpb.Finger.index
    message.mag = glove.bpb.Mag(x=x, y=y, z=z)
    return cobs.encode(bytes(message))


def main() -> int:
    root = APP / "diagnostics"
    root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="replay_test_", dir=root) as temp:
        source_recorder = TraceRecorder("replay_source_test", __file__)
        source_dir = source_recorder.start_session(temp, metadata={"synthetic": True})
        base_ns = 10_000_000_000
        payloads = [
            ("QUAT", make_quat(1, 0.0, 0.0, 0.0, 1.0)),
            ("MAG", make_mag(2, -50.0, 10.0, 20.0)),
            ("QUAT", make_quat(3, 0.1, 0.0, 0.0, 0.9949874371)),
            ("MAG", make_mag(4, -49.0, 11.0, 19.0)),
        ]
        for sequence, (kind, raw) in enumerate(payloads, 1):
            message, error = glove.decode_packet_detailed(raw)
            assert message is not None, error
            row = glove.message_to_row(message)
            rx_ns = base_ns + sequence * 10_000_000
            source_recorder.record_serial_bytes(
                connection_id=1,
                data=raw + b"\x00",
                first_rx_monotonic_ns=rx_ns,
                last_rx_monotonic_ns=rx_ns + 1,
            )
            source_recorder.record_frame(
                trace_id=sequence,
                connection_id=1,
                sequence=sequence,
                rx_wall_time_ns=time.time_ns(),
                rx_monotonic_ns=rx_ns,
                raw_packet=raw,
                decoded=row.get("packet"),
                decode_error=None,
                kind=kind,
            )
        source_recorder.record_firmware_line(
            connection_id=1,
            line="GQ f4 l1 id13 acc0 raw 1 2 3 4 yawc -100",
            parsed={"type": "gq", "raw": "GQ", "fields": {"sensor_id": 13, "quat_x": 0.0, "quat_y": 0.0, "quat_z": 0.0, "quat_w": 1.0, "accuracy_raw": 0, "yaw_corr_deg": -1.0}},
            rx_wall_time_ns=time.time_ns(),
            rx_monotonic_ns=base_ns - 1,
        )
        source_recorder.stop_session("synthetic_source_complete")

        replay_dir, summary = replay_pose(source_dir, output_parent=Path(temp), speed=0.0)
        assert summary["processed"] == len(payloads), summary
        assert summary["decode_errors"] == 0, summary
        manifest = json.loads((replay_dir / "manifest.json").read_text(encoding="utf-8"))
        stats = manifest["stats"]
        assert stats["analysis_rows"] == len(payloads), stats
        assert stats["stage_records"] == len(payloads), stats
        assert stats["serial_bytes"] == sum(len(raw) + 1 for _kind, raw in payloads), stats
        assert stats["firmware_lines"] == 1, stats
        print("TRACE_REPLAY_E2E_PASS=True")
        print("SOURCE=", source_dir)
        print("REPLAY=", replay_dir)
        print("SUMMARY=", summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())