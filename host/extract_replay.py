#!/usr/bin/env python3
"""Extract data-only replay inputs and snapshots from an OSMO trace.

Usage:
    python extract_replay.py SESSION_DIR [--output OUT_DIR]

The source trace must contain frames produced by the REPLAYBIN firmware
(``replay_protocol.py`` sensor IDs 240..249).  Existing traces are still
readable; they simply contain no replay frames.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from typing import Any

import read_osmo_glove as glove
import replay_protocol as rp
from trace_replay import iter_raw_frames

RAW_FIELDS = [
    "kind", "sensor_id", "link", "index", "rx_monotonic_ns",
    "device_time_s", "device_ticks", "v0", "v1", "v2", "v3", "v4",
]


def _rows(session: Path):
    for frame in iter_raw_frames(session):
        message, error = glove.decode_packet_detailed(frame.payload)
        if message is None:
            continue
        try:
            row = glove.message_to_row(message)
        except Exception:
            continue
        row["_trace"] = {"rx_monotonic_ns": frame.rx_monotonic_ns}
        yield row


def extract(session: Path, output: Path) -> dict[str, Any]:
    output.mkdir(parents=True, exist_ok=True)
    snapshots = output / "snapshots"
    snapshots.mkdir(exist_ok=True)

    raw_rows: list[dict[str, Any]] = []
    diagnostic_rows: dict[str, list[Any]] = defaultdict(list)
    event_rows: list[dict[str, Any]] = []
    blob_rows: list[dict[str, Any]] = []

    for row in _rows(session):
        decoded = rp.decode_replay_row(row)
        if decoded is None:
            continue
        if decoded.kind.startswith("RAW_"):
            raw_rows.append({
                "kind": decoded.kind,
                "sensor_id": decoded.sensor_id,
                "link": decoded.link,
                "index": decoded.index,
                "rx_monotonic_ns": (row.get("_trace") or {}).get("rx_monotonic_ns"),
                "device_time_s": decoded.device_time_s,
                "device_ticks": (
                    int(round(decoded.device_time_s / 1.5625e-5))
                    if decoded.device_time_s is not None else None
                ),
                **{f"v{i}": (decoded.values[i] if i < len(decoded.values) else None)
                   for i in range(5)},
            })
        elif decoded.kind in ("CAL_BLOB", "FUSION_STATE", "CONFIG"):
            blob_rows.append(row)
        elif decoded.kind in ("YF", "YAW", "YAW2", "HEALTH"):
            diagnostic_rows[decoded.kind].append(decoded)
        elif decoded.kind == "EVENT":
            event = rp.decode_event(decoded)
            event["rx_monotonic_ns"] = (row.get("_trace") or {}).get("rx_monotonic_ns")
            event_rows.append(event)

    raw_csv = output / "replay_raw_inputs.csv"
    with raw_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=RAW_FIELDS)
        writer.writeheader()
        writer.writerows(raw_rows)

    for kind, frames in diagnostic_rows.items():
        assembled = rp.assemble_diagnostic(frames)
        fields = {
            "YF": rp.YF_FIELDS, "YAW": rp.YAW_FIELDS,
            "YAW2": rp.YAW2_FIELDS, "HEALTH": rp.HEALTH_FIELDS,
        }[kind]
        csv_path = output / f"replay_{kind.lower()}.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(assembled)

    if event_rows:
        event_path = output / "replay_events.csv"
        with event_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=["event_code", "event", "tick_ms", "a", "b", "link", "rx_monotonic_ns"])
            writer.writeheader()
            writer.writerows(event_rows)

    clock_events = [event for event in event_rows if event.get("event") == "clock_sync"]
    clock_points = []
    for event in clock_events:
        rx_ns = event.get("rx_monotonic_ns")
        if rx_ns is None:
            continue
        bhi_ticks = (int(event.get("b") or 0) << 32) | (int(event.get("a") or 0) & 0xFFFFFFFF)
        clock_points.append({
            "rx_monotonic_ns": int(rx_ns),
            "host_monotonic_s": int(rx_ns) / 1e9,
            "hal_tick_ms": int(event.get("tick_ms") or 0),
            "bhi_ticks": bhi_ticks,
            "bhi_time_s": bhi_ticks * 1.5625e-5,
        })
    clock_map: dict[str, Any] = {"points": clock_points}
    if len(clock_points) >= 2:
        xs = [point["hal_tick_ms"] / 1000.0 for point in clock_points]
        ys = [point["host_monotonic_s"] for point in clock_points]
        x_mean = sum(xs) / len(xs)
        y_mean = sum(ys) / len(ys)
        denom = sum((x - x_mean) ** 2 for x in xs)
        slope = (sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denom) if denom > 0 else None
        intercept = (y_mean - slope * x_mean) if slope is not None else None
        clock_map.update({
            "slope": slope,
            "intercept": intercept,
            "offset_at_first_s": clock_points[0]["host_monotonic_s"] - clock_points[0]["hal_tick_ms"] / 1000.0,
        })
    (output / "clock_map.json").write_text(
        json.dumps(clock_map, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    blobs = rp.assemble_blobs(blob_rows)
    config: dict[str, Any] | None = None
    for (kind, link), blob in blobs.items():
        (snapshots / f"{kind}_link{link}.bin").write_bytes(blob)
        if kind == "CONFIG":
            try:
                config = rp.parse_config(blob)
            except Exception as exc:
                config = {"error": str(exc)}

    summary = {
        "session": str(session),
        "output": str(output),
        "raw_rows": len(raw_rows),
        "diagnostics": {key: len(value) for key, value in diagnostic_rows.items()},
        "events": len(event_rows),
        "clock_points": len(clock_points),
        "clock_map": str(output / "clock_map.json"),
        "snapshots": {f"{key[0]}_link{key[1]}": len(value) for key, value in blobs.items()},
        "config": config,
        "raw_csv": str(raw_csv),
    }
    (output / "replay_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Extract replay inputs from an OSMO trace.")
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    session = args.session_dir.resolve()
    output = args.output.resolve() if args.output else session / "replay_extract"
    summary = extract(session, output)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
