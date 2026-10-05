#!/usr/bin/env python3
"""Utilities for verifying and summarizing OSMO host trace sessions."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import struct
import sys
from collections import Counter
from pathlib import Path
from typing import Any

RAW_MAGIC = b"OSMOTRC1"
RAW_RECORD = struct.Struct("<QQI")


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, 1):
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise RuntimeError(f"{path.name}:{line_no}: invalid JSON: {exc}") from exc
    return rows


def _read_raw(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    if not path.exists():
        return records
    data = path.read_bytes()
    if not data.startswith(RAW_MAGIC):
        raise RuntimeError(f"{path.name}: missing {RAW_MAGIC!r} magic")
    offset = len(RAW_MAGIC)
    while offset < len(data):
        if offset + RAW_RECORD.size > len(data):
            raise RuntimeError(f"{path.name}: truncated record header at offset {offset}")
        trace_id, rx_monotonic_ns, length = RAW_RECORD.unpack_from(data, offset)
        offset += RAW_RECORD.size
        end = offset + length
        if end > len(data):
            raise RuntimeError(
                f"{path.name}: truncated payload trace_id={trace_id} "
                f"declared={length} remaining={len(data) - offset}"
            )
        payload = data[offset:end]
        offset = end
        records.append({
            "trace_id": trace_id,
            "rx_monotonic_ns": rx_monotonic_ns,
            "length": length,
            "payload": payload,
        })
    return records


def _load_manifest(session_dir: Path) -> dict[str, Any]:
    path = session_dir / "manifest.json"
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def verify_session(session_dir: Path) -> dict[str, Any]:
    frames = _read_jsonl(session_dir / "frames.jsonl")
    raw_records = _read_raw(session_dir / "raw_frames.bin")
    issues: list[str] = []
    if len(frames) != len(raw_records):
        issues.append(f"frame count mismatch: frames={len(frames)} raw={len(raw_records)}")
    frame_ids = [int(row["trace_id"]) for row in frames if row.get("trace_id") is not None]
    raw_ids = [int(row["trace_id"]) for row in raw_records]
    if len(frame_ids) != len(set(frame_ids)):
        issues.append("duplicate trace_id in frames.jsonl")
    if len(raw_ids) != len(set(raw_ids)):
        issues.append("duplicate trace_id in raw_frames.bin")
    frame_by_id = {int(row.get("trace_id")): row for row in frames if row.get("trace_id") is not None}
    for raw in raw_records:
        trace_id = int(raw["trace_id"])
        frame = frame_by_id.get(trace_id)
        if frame is None:
            issues.append(f"raw trace_id {trace_id} has no frames.jsonl row")
            continue
        if int(frame.get("packet_len", -1)) != int(raw["length"]):
            issues.append(f"trace_id {trace_id}: packet_len mismatch")
        if int(frame.get("rx_monotonic_ns", -1)) != int(raw["rx_monotonic_ns"]):
            issues.append(f"trace_id {trace_id}: rx_monotonic_ns mismatch")
        expected_hash = frame.get("packet_sha256")
        if expected_hash:
            actual_hash = hashlib.sha256(raw["payload"]).hexdigest()
            if actual_hash != expected_hash:
                issues.append(f"trace_id {trace_id}: payload sha256 mismatch")
    firmware_diagnostics = _read_jsonl(session_dir / "firmware_diagnostics.jsonl")
    gate_events = _read_jsonl(session_dir / "gate_events.jsonl")
    serial_chunks = _read_jsonl(session_dir / "serial_chunks.jsonl")
    serial_stream_path = session_dir / "serial_stream.bin"
    serial_stream_data = serial_stream_path.read_bytes() if serial_stream_path.exists() else b""
    serial_stream_bytes = len(serial_stream_data)
    expected_serial_offset = 0
    for chunk in serial_chunks:
        offset = int(chunk.get("offset", -1))
        length = int(chunk.get("length", -1))
        if offset < 0 or length < 0 or offset + length > serial_stream_bytes:
            issues.append(f"invalid serial chunk offset={offset} length={length}")
            continue
        if offset != expected_serial_offset:
            issues.append(f"serial chunk gap/overlap: expected={expected_serial_offset} actual={offset}")
        expected_serial_offset = offset + length
        payload = serial_stream_data[offset:offset + length]
        expected_hash = chunk.get("sha256")
        if expected_hash and hashlib.sha256(payload).hexdigest() != expected_hash:
            issues.append(f"serial chunk offset={offset} length={length}: sha256 mismatch")
    if expected_serial_offset != serial_stream_bytes:
        issues.append(f"serial stream trailing bytes not indexed: indexed={expected_serial_offset} total={serial_stream_bytes}")
    report = {
        "session_dir": str(session_dir.resolve()),
        "manifest": _load_manifest(session_dir),
        "frame_count": len(frames),
        "raw_record_count": len(raw_records),
        "raw_payload_bytes": sum(int(row["length"]) for row in raw_records),
        "serial_stream_bytes": serial_stream_bytes,
        "serial_chunk_count": len(serial_chunks),
        "firmware_line_count": len(firmware_diagnostics),
        "gate_event_count": len(gate_events),
        "issues": issues,
        "verified": not issues,
    }
    return report


def summarize_session(session_dir: Path) -> dict[str, Any]:
    frames = _read_jsonl(session_dir / "frames.jsonl")
    stages = _read_jsonl(session_dir / "stages.jsonl")
    events = _read_jsonl(session_dir / "events.jsonl")
    firmware = _read_jsonl(session_dir / "firmware_diagnostics.jsonl")
    gate_events = _read_jsonl(session_dir / "gate_events.jsonl")
    system = _read_jsonl(session_dir / "system.jsonl")
    analysis_path = session_dir / "analysis.csv"
    firmware_diagnostics = _read_jsonl(session_dir / "firmware_diagnostics.jsonl")
    gate_events = _read_jsonl(session_dir / "gate_events.jsonl")
    serial_chunks = _read_jsonl(session_dir / "serial_chunks.jsonl")
    serial_stream_path = session_dir / "serial_stream.bin"
    serial_stream_bytes = serial_stream_path.stat().st_size if serial_stream_path.exists() else 0
    analysis_rows = 0
    if analysis_path.exists():
        with analysis_path.open("r", encoding="utf-8", newline="") as handle:
            analysis_rows = max(0, sum(1 for _ in handle) - 1)
    kinds = Counter(str(row.get("kind")) for row in frames)
    firmware_types = Counter(str(row.get("parsed_type")) for row in firmware)
    decoded_ok = sum(1 for row in frames if row.get("decode_ok"))
    return {
        "session_dir": str(session_dir.resolve()),
        "manifest": _load_manifest(session_dir),
        "counts": {
            "frames": len(frames),
            "decoded_ok": decoded_ok,
            "decode_errors": len(frames) - decoded_ok,
            "raw_kinds": dict(kinds),
            "stages": len(stages),
            "analysis_rows": analysis_rows,
            "serial_stream_bytes": serial_stream_bytes,
            "serial_chunk_count": len(serial_chunks),
            "events": len(events),
            "firmware_lines": len(firmware),
            "firmware_parsed": sum(1 for row in firmware if row.get("parsed_type") != "text"),
            "firmware_types": dict(firmware_types),
            "gate_events": len(gate_events),
            "system_records": len(system),
        },
        "last_frame": frames[-1] if frames else None,
        "last_system": system[-1] if system else None,
    }


def export_analysis_csv(session_dir: Path, output: Path | None = None) -> Path:
    source = session_dir / "analysis.csv"
    if not source.exists():
        raise RuntimeError(f"missing {source}")
    target = output or (session_dir / "analysis_export.csv")
    with source.open("r", encoding="utf-8", newline="") as src, target.open("w", encoding="utf-8", newline="") as dst:
        reader = csv.reader(src)
        writer = csv.writer(dst)
        for row in reader:
            writer.writerow(row)
    return target


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Inspect an OSMO host trace session.")
    parser.add_argument("command", choices=("summary", "verify", "export-csv"))
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--output", type=Path, help="output CSV for export-csv")
    args = parser.parse_args(argv)

    session_dir = args.session_dir.resolve()
    if not session_dir.is_dir():
        print(f"ERROR: not a directory: {session_dir}", file=sys.stderr)
        return 2
    try:
        if args.command == "summary":
            result = summarize_session(session_dir)
        elif args.command == "verify":
            result = verify_session(session_dir)
        else:
            result = {"output": str(export_analysis_csv(session_dir, args.output).resolve())}
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2, default=str))
    return 0 if result.get("verified", True) is not False else 3


if __name__ == "__main__":
    raise SystemExit(main())