#!/usr/bin/env python3
"""Validate a REPLAYBIN extract for missing/duplicated input samples."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any


def _read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _int(value: str | None, default: int = 0) -> int:
    try:
        return int(value or default)
    except (TypeError, ValueError):
        return default


def verify(work: Path) -> dict[str, Any]:
    work = work.resolve()
    raw = _read_csv(work / "replay_raw_inputs.csv")
    health = _read_csv(work / "replay_health.csv")
    events = _read_csv(work / "replay_events.csv")

    groups: dict[tuple[int, str], list[dict[str, str]]] = defaultdict(list)
    for row in raw:
        groups[(_int(row.get("link")), str(row.get("kind") or "UNKNOWN"))].append(row)

    streams: dict[str, Any] = {}
    for (link, kind), rows in sorted(groups.items()):
        ticks = [_int(row.get("device_ticks")) for row in rows]
        deltas = [b - a for a, b in zip(ticks, ticks[1:]) if b >= a]
        positive = [d for d in deltas if d > 0]
        median = statistics.median(positive) if positive else None
        gaps = [d for d in positive if median is not None and d > 3.0 * median]
        duplicates = sum(1 for d in deltas if d == 0)
        streams[f"link{link}:{kind}"] = {
            "samples": len(rows),
            "median_tick_delta": median,
            "max_tick_delta": max(positive) if positive else None,
            "gaps_gt_3x_median": len(gaps),
            "duplicate_timestamps": duplicates,
        }

    last_health = health[-1] if health else {}
    issues: list[str] = []
    for key, item in streams.items():
        if item["gaps_gt_3x_median"]:
            issues.append(f"{key}: {item['gaps_gt_3x_median']} gaps > 3x median")
        if item["duplicate_timestamps"]:
            issues.append(f"{key}: {item['duplicate_timestamps']} duplicate timestamps")
    for key in ("cdc_errors", "fifo_overflows", "meta_errors", "sensor_errors"):
        try:
            value = float(last_health.get(key) or 0.0)
        except (TypeError, ValueError):
            value = 0.0
        if value:
            issues.append(f"firmware health {key}={value:g}")

    report = {
        "work_dir": str(work),
        "streams": streams,
        "events": len(events),
        "health_samples": len(health),
        "last_health": last_health,
        "issues": issues,
        "ok": not issues,
    }
    (work / "replay_integrity.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify replay input completeness.")
    parser.add_argument("work_dir", type=Path)
    args = parser.parse_args()
    report = verify(args.work_dir)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
