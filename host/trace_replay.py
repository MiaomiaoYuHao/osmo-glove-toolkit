#!/usr/bin/env python3
"""Replay an OSMO raw trace through the same host algorithms.

Main commands:
  python trace_replay.py list-pose SOURCE_TRACE
  python trace_replay.py replay-pose SOURCE_TRACE --output PARENT_DIR
  python trace_replay.py decode SOURCE_TRACE --limit 20

`replay-pose` creates a new trace session containing the replayed raw frames,
decoded protobuf, algorithm stages and final analysis rows.  Timestamps from
the source trace are reused for motion/magnetometer dt, so ``--speed 0`` is a
deterministic as-fast-as-possible offline run.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import re
import struct
import sys
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

APP_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(APP_DIR))

import read_osmo_glove as glove  # noqa: E402
from firmware_diagnostics import parse_firmware_line
from replay_protocol import kind_for_sensor
from trace_recorder import TraceRecorder  # noqa: E402

RAW_MAGIC = b"OSMOTRC1"
RAW_RECORD = struct.Struct("<QQI")


@dataclass(frozen=True)
class ReplayFrame:
    source_trace_id: int
    rx_monotonic_ns: int
    payload: bytes


def iter_raw_frames(session_dir: str | Path) -> Iterator[ReplayFrame]:
    path = Path(session_dir) / "raw_frames.bin"
    data = path.read_bytes()
    if not data.startswith(RAW_MAGIC):
        raise RuntimeError(f"{path} is not an OSMOTRC1 raw stream")
    offset = len(RAW_MAGIC)
    while offset < len(data):
        if offset + RAW_RECORD.size > len(data):
            raise RuntimeError(f"{path}: truncated record header at {offset}")
        trace_id, rx_monotonic_ns, length = RAW_RECORD.unpack_from(data, offset)
        offset += RAW_RECORD.size
        end = offset + length
        if end > len(data):
            raise RuntimeError(f"{path}: truncated payload for trace_id={trace_id}")
        yield ReplayFrame(int(trace_id), int(rx_monotonic_ns), data[offset:end])
        offset = end


def iter_serial_chunks(session_dir: str | Path) -> Iterator[tuple[bytes, int | None, int | None]]:
    """Yield exact serial byte chunks, including trailing/non-framed bytes."""
    session = Path(session_dir)
    stream_path = session / "serial_stream.bin"
    chunk_path = session / "serial_chunks.jsonl"
    if stream_path.exists() and chunk_path.exists():
        stream = stream_path.read_bytes()
        for line in chunk_path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            record = json.loads(line)
            offset = int(record.get("offset", 0))
            length = int(record.get("length", 0))
            payload = stream[offset:offset + length]
            yield payload, record.get("first_rx_monotonic_ns"), record.get("last_rx_monotonic_ns")
        return
    for frame in iter_raw_frames(session):
        yield frame.payload + b"\x00", frame.rx_monotonic_ns, frame.rx_monotonic_ns


def iter_firmware_diagnostics(session_dir: str | Path) -> list[dict[str, Any]]:
    session = Path(session_dir)
    path = session / "firmware_diagnostics.jsonl"
    if path.exists():
        records: list[dict[str, Any]] = []
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                records.append(json.loads(line))
        records.sort(key=lambda item: int(item.get("rx_monotonic_ns") or 0))
        return records

    # Backward compatibility for traces captured before FIX14 text parsing.
    stream_path = session / "serial_stream.bin"
    if not stream_path.exists():
        return []
    stream = stream_path.read_bytes()
    chunks: list[dict[str, Any]] = []
    chunk_path = session / "serial_chunks.jsonl"
    if chunk_path.exists():
        chunks = [json.loads(line) for line in chunk_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    records = []
    start = 0
    search_from = 0
    while True:
        end = stream.find(b"\r\n", search_from)
        if end < 0:
            break
        raw = stream[start:end]
        printable = re.findall(r"[ -~]+", raw.decode("ascii", "replace"))
        text = printable[-1].strip() if printable else ""
        if text:
            end_offset = end + 2
            timestamp = None
            for chunk in chunks:
                offset = int(chunk.get("offset") or 0)
                length = int(chunk.get("length") or 0)
                if offset <= end_offset <= offset + length:
                    timestamp = chunk.get("last_rx_monotonic_ns") or chunk.get("first_rx_monotonic_ns")
                    break
            records.append({
                "line": text,
                "parsed_type": parse_firmware_line(text)["type"],
                "rx_monotonic_ns": int(timestamp or 0),
                "rx_wall_time_ns": None,
                "backfilled_from_serial_stream": True,
            })
        start = end + 2
        search_from = start
    records.sort(key=lambda item: int(item.get("rx_monotonic_ns") or 0))
    return records


def iter_decoded_rows(session_dir: str | Path) -> Iterator[tuple[ReplayFrame, dict[str, Any] | None, str | None]]:
    for frame in iter_raw_frames(session_dir):
        message, error = glove.decode_packet_detailed(frame.payload)
        if message is None:
            yield frame, None, error or "decode_failed"
            continue
        try:
            row = glove.message_to_row(message)
        except Exception as exc:
            yield frame, None, f"row_conversion_error: {type(exc).__name__}: {exc}"
            continue
        replay_kind = kind_for_sensor(row.get("sensor_id"))
        if replay_kind is not None:
            row["kind"] = replay_kind
        elif row.get("mag_x") is not None:
            row["kind"] = "MAG_META" if int(row.get("sensor_id") or 0) > 20 else "MAG"
        elif row.get("quat_x") is not None:
            row["kind"] = "QUAT"
        else:
            row["kind"] = "UNKNOWN"
        yield frame, row, None


def _load_pose_module() -> Any:
    spec = importlib.util.spec_from_file_location("pose_app", APP_DIR / "姿态测试上位机.pyw")
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load pose application module")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def replay_pose(
    session_dir: str | Path,
    *,
    output_parent: str | Path | None = None,
    speed: float = 0.0,
    sensor: int | None = None,
    limit: int | None = None,
) -> tuple[Path, dict[str, Any]]:
    source_dir = Path(session_dir).resolve()
    parent = Path(output_parent).resolve() if output_parent else source_dir.parent
    parent.mkdir(parents=True, exist_ok=True)

    pose = _load_pose_module()
    root = tk.Tk()
    root.withdraw()
    app = pose.PoseMonitor(root, auto_connect=False, auto_trace=False)
    app.trace = TraceRecorder("pose_replay", __file__)
    if sensor is not None:
        app.sensor_var.set(str(sensor))

    source_manifest = {}
    manifest_path = source_dir / "manifest.json"
    if manifest_path.exists():
        source_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    replay_session = app.trace.start_session(parent, metadata={
        "source_trace_dir": str(source_dir),
        "source_session_id": source_manifest.get("session_id"),
        "source_manifest": source_manifest,
        "speed": speed,
        "sensor": sensor,
        "limit": limit,
        "deterministic": speed <= 0,
    })

    for serial_chunk, first_rx_ns, last_rx_ns in iter_serial_chunks(source_dir):
        app.trace.record_serial_bytes(
            connection_id=1,
            data=serial_chunk,
            first_rx_monotonic_ns=first_rx_ns,
            last_rx_monotonic_ns=last_rx_ns,
        )

    diagnostics = iter_firmware_diagnostics(source_dir)
    diagnostic_index = 0
    previous_source_ns: int | None = None
    processed = 0
    decoded_rows = 0
    decode_errors = 0
    kinds: dict[str, int] = {}
    try:
        for replay_frame in iter_raw_frames(source_dir):
            if limit is not None and processed >= limit:
                break
            rx_monotonic_ns = replay_frame.rx_monotonic_ns or time.monotonic_ns()
            while diagnostic_index < len(diagnostics):
                diagnostic = diagnostics[diagnostic_index]
                diagnostic_ns = int(diagnostic.get("rx_monotonic_ns") or 0)
                if diagnostic_ns > rx_monotonic_ns:
                    break
                diagnostic_line = str(diagnostic.get("line") or "")
                parsed = parse_firmware_line(diagnostic_line)
                app.trace.record_firmware_line(
                    connection_id=1,
                    line=diagnostic_line,
                    parsed=parsed,
                    rx_wall_time_ns=int(diagnostic.get("rx_wall_time_ns") or time.time_ns()),
                    rx_monotonic_ns=diagnostic_ns,
                )
                app._handle_firmware_line({
                    "line": diagnostic_line,
                    "parsed": parsed,
                    "connection_id": 1,
                    "rx_wall_time_ns": diagnostic.get("rx_wall_time_ns"),
                    "rx_monotonic_ns": diagnostic_ns,
                })
                diagnostic_index += 1

            message, decode_error, protobuf_payload = glove.decode_packet_with_payload(replay_frame.payload)
            new_trace_id = app.trace.next_trace_id()
            replay_now_ns = time.monotonic_ns()
            wall_time_ns = time.time_ns()
            kind = "DECODE_ERROR"
            row = None
            if message is not None:
                try:
                    row = glove.message_to_row(message)
                except Exception as exc:
                    decode_error = f"row_conversion_error: {type(exc).__name__}: {exc}"
                    row = None
            if row is not None:
                replay_kind = kind_for_sensor(row.get("sensor_id"))
                if replay_kind is not None:
                    kind = replay_kind
                elif row.get("mag_x") is not None:
                    kind = "MAG_META" if int(row.get("sensor_id") or 0) > 20 else "MAG"
                elif row.get("quat_x") is not None:
                    kind = "QUAT"
                else:
                    kind = "UNKNOWN"
                row["kind"] = kind
                row["_trace"] = {
                    "trace_id": new_trace_id,
                    "connection_id": 1,
                    "sequence": processed + 1,
                    "rx_wall_time_ns": wall_time_ns,
                    "rx_monotonic_ns": rx_monotonic_ns,
                    "packet_len": len(replay_frame.payload),
                    "source_trace_id": replay_frame.source_trace_id,
                    "replay": True,
                    "replay_monotonic_ns": replay_now_ns,
                }
                decoded_rows += 1
                kinds[kind] = kinds.get(kind, 0) + 1
            else:
                kinds["DECODE_ERROR"] = kinds.get("DECODE_ERROR", 0) + 1
                decode_errors += 1

            app.trace.record_frame(
                trace_id=new_trace_id,
                connection_id=1,
                sequence=processed + 1,
                rx_wall_time_ns=wall_time_ns,
                rx_monotonic_ns=rx_monotonic_ns,
                raw_packet=replay_frame.payload,
                decoded=row.get("packet") if row is not None else None,
                protobuf_payload=protobuf_payload,
                decode_error=decode_error,
                kind=kind,
                source_trace_id=replay_frame.source_trace_id,
                replay=True,
            )
            if row is not None:
                if replay_kind is not None:
                    # Data-only replay frames are intentionally not fed into the
                    # live pose pipeline; the offline firmware replayer consumes them.
                    pass
                elif kind == "QUAT":
                    app._handle_quat(row)  # noqa: SLF001 - deliberate same-algorithm replay
                elif kind in ("MAG", "MAG_META"):
                    app._handle_mag(row)  # noqa: SLF001 - deliberate same-algorithm replay
                else:
                    app._record_stage("replay_unclassified", row, accepted=False)
            processed += 1

            if speed > 0 and previous_source_ns is not None:
                delta_s = max(0.0, (rx_monotonic_ns - previous_source_ns) / 1e9)
                time.sleep(delta_s / speed)
            previous_source_ns = rx_monotonic_ns

        while diagnostic_index < len(diagnostics):
            diagnostic = diagnostics[diagnostic_index]
            diagnostic_line = str(diagnostic.get("line") or "")
            parsed = parse_firmware_line(diagnostic_line)
            app.trace.record_firmware_line(
                connection_id=1,
                line=diagnostic_line,
                parsed=parsed,
                rx_wall_time_ns=int(diagnostic.get("rx_wall_time_ns") or time.time_ns()),
                rx_monotonic_ns=int(diagnostic.get("rx_monotonic_ns") or 0),
            )
            app._handle_firmware_line({
                "line": diagnostic_line,
                "parsed": parsed,
                "connection_id": 1,
                "rx_wall_time_ns": diagnostic.get("rx_wall_time_ns"),
                "rx_monotonic_ns": diagnostic.get("rx_monotonic_ns"),
            })
            diagnostic_index += 1

        app._pipeline_flush(time.monotonic() + 2.0)
        app.trace.record_system({
            "event": "replay_finished",
            "processed": processed,
            "decoded_rows": decoded_rows,
            "decode_errors": decode_errors,
            "kinds": kinds,
            "speed": speed,
        })
    finally:
        app.trace.stop_session("replay_complete")
        try:
            root.destroy()
        except Exception:
            pass

    return replay_session, {
        "source_trace_dir": str(source_dir),
        "output_trace_dir": str(replay_session),
        "processed": processed,
        "decoded_rows": decoded_rows,
        "decode_errors": decode_errors,
        "kinds": kinds,
    }


def _decode_listing(session_dir: Path, limit: int | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for index, (frame, row, error) in enumerate(iter_decoded_rows(session_dir), 1):
        if limit is not None and index > limit:
            break
        rows.append({
            "source_trace_id": frame.source_trace_id,
            "rx_monotonic_ns": frame.rx_monotonic_ns,
            "payload_len": len(frame.payload),
            "payload_hex": frame.payload.hex(),
            "kind": row.get("kind") if row else "DECODE_ERROR",
            "sensor_id": row.get("sensor_id") if row else None,
            "index": row.get("index") if row else None,
            "decode_error": error,
        })
    return rows


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Replay an OSMO host trace session.")
    sub = parser.add_subparsers(dest="command", required=True)

    p_list = sub.add_parser("list-pose", help="list raw frames and decoded envelope fields")
    p_list.add_argument("session_dir", type=Path)
    p_list.add_argument("--limit", type=int)

    p_replay = sub.add_parser("replay-pose", help="replay through the real pose/mag algorithms")
    p_replay.add_argument("session_dir", type=Path)
    p_replay.add_argument("--output", type=Path, help="parent directory for the replay trace")
    p_replay.add_argument("--speed", type=float, default=0.0, help="0=fast, 1=real-time, 2=2x")
    p_replay.add_argument("--sensor", type=int, help="select one sensor id during replay")
    p_replay.add_argument("--limit", type=int)

    p_decode = sub.add_parser("decode", help="decode and print raw frame summaries")
    p_decode.add_argument("session_dir", type=Path)
    p_decode.add_argument("--limit", type=int, default=20)

    args = parser.parse_args(argv)
    if args.command in ("list-pose", "decode"):
        session_dir = args.session_dir.resolve()
        rows = _decode_listing(session_dir, args.limit)
        print(json.dumps(rows, ensure_ascii=False, indent=2))
        return 0

    session_dir = args.session_dir.resolve()
    output, summary = replay_pose(
        session_dir,
        output_parent=args.output,
        speed=max(0.0, float(args.speed)),
        sensor=args.sensor,
        limit=args.limit,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"REPLAY_TRACE={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())