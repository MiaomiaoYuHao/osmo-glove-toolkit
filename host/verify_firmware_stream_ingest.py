#!/usr/bin/env python3
"""Feed a real FIX14 serial byte stream through the host text ingestion path."""

from __future__ import annotations

import json
import queue
import tempfile
from pathlib import Path

from serial_trace_reader import TracedSerialReader
from trace_recorder import TraceRecorder

APP = Path(__file__).resolve().parent
SOURCE = APP / "diagnostics" / "real_smoke_traces" / "20260921_155154_pose_monitor_trace" / "serial_stream.bin"


def main() -> int:
    data = SOURCE.read_bytes()
    root = APP / "diagnostics"
    with tempfile.TemporaryDirectory(prefix="firmware_ingest_test_", dir=root) as temp:
        recorder = TraceRecorder("firmware_ingest_test", __file__)
        session = recorder.start_session(temp, metadata={"source": str(SOURCE)})
        output: queue.Queue = queue.Queue()
        reader = object.__new__(TracedSerialReader)
        reader.connection_id = 1
        reader.trace = recorder
        reader.output = output
        reader._text_pending = bytearray()
        reader._text_saw_cr = False
        for value in data:
            reader._feed_text(bytes((value,)))
        reader._flush_text()
        recorder.stop_session("ingest_complete")
        manifest = json.loads((session / "manifest.json").read_text(encoding="utf-8"))
        stats = manifest["stats"]
        emitted = []
        while not output.empty():
            event, payload = output.get_nowait()
            if event == "firmware_line":
                emitted.append(payload["parsed"]["type"])
        assert stats["firmware_lines"] >= 4, stats
        assert stats["firmware_parsed"] >= 4, stats
        assert emitted.count("gq") >= 2, emitted
        assert emitted.count("gy") >= 2, emitted
        print("FIRMWARE_STREAM_INGEST_PASS=True")
        print("FIRMWARE_LINES=", stats["firmware_lines"])
        print("FIRMWARE_PARSED=", stats["firmware_parsed"])
        print("PARSED_TYPES=", emitted)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
