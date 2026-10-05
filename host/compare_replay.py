#!/usr/bin/env python3
"""Compare recorded firmware YAW state with the offline replay output."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _f(row: dict[str, str], key: str) -> float | None:
    try:
        return float(row.get(key) or "")
    except (TypeError, ValueError):
        return None


def compare(work: Path, tolerance_ms: float = 100.0) -> dict[str, Any]:
    original_path = work / "replay_yaw.csv"
    replay_path = work / "replay_engine_output.csv"
    if not original_path.exists() or not replay_path.exists():
        raise RuntimeError("need replay_yaw.csv and replay_engine_output.csv")
    original = _read_csv(original_path)
    replay = _read_csv(replay_path)
    replay_t = []
    for row in replay:
        ticks = _f(row, "ts_ticks")
        if ticks is not None:
            replay_t.append((ticks * 1.5625e-5 * 1000.0, row))
    replay_t.sort(key=lambda item: item[0])

    out_rows: list[dict[str, Any]] = []
    yaw_errors: list[float] = []
    datum_errors: list[float] = []
    for row in original:
        t = _f(row, "t_ms")
        rec_yaw = _f(row, "yaw_corr_deg")
        rec_datum = _f(row, "datum_deg")
        if t is None or rec_yaw is None or not replay_t:
            continue
        nearest_t, nearest = min(replay_t, key=lambda item: abs(item[0] - t))
        dt_ms = nearest_t - t
        if abs(dt_ms) > tolerance_ms:
            continue
        rep_yaw = _f(nearest, "yaw_corr_deg")
        rep_datum = _f(nearest, "datum_deg")
        if rep_yaw is None or rep_datum is None:
            continue
        yaw_delta = rep_yaw - rec_yaw
        datum_delta = rep_datum - rec_datum
        yaw_errors.append(abs(yaw_delta))
        datum_errors.append(abs(datum_delta))
        out_rows.append({
            "rec_t_ms": t,
            "replay_t_ms": nearest_t,
            "dt_ms": dt_ms,
            "rec_yaw_corr_deg": rec_yaw,
            "replay_yaw_corr_deg": rep_yaw,
            "yaw_delta_deg": yaw_delta,
            "rec_datum_deg": rec_datum,
            "replay_datum_deg": rep_datum,
            "datum_delta_deg": datum_delta,
        })

    out_path = work / "replay_compare.csv"
    with out_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(out_rows[0]) if out_rows else [
            "rec_t_ms", "replay_t_ms", "dt_ms", "rec_yaw_corr_deg",
            "replay_yaw_corr_deg", "yaw_delta_deg", "rec_datum_deg",
            "replay_datum_deg", "datum_delta_deg",
        ])
        writer.writeheader()
        writer.writerows(out_rows)

    def stats(values: list[float]) -> dict[str, float | None]:
        if not values:
            return {"max": None, "mean": None, "p95": None}
        ordered = sorted(values)
        return {
            "max": max(values),
            "mean": sum(values) / len(values),
            "p95": ordered[min(len(ordered) - 1, int(math.ceil(0.95 * len(ordered))) - 1)],
        }

    summary = {
        "matched_rows": len(out_rows),
        "yaw_abs_error_deg": stats(yaw_errors),
        "datum_abs_error_deg": stats(datum_errors),
        "compare_csv": str(out_path),
    }
    (work / "replay_compare.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare firmware and offline replay YAW traces.")
    parser.add_argument("work_dir", type=Path, help="extract_replay output directory")
    parser.add_argument("--tolerance-ms", type=float, default=100.0)
    args = parser.parse_args()
    print(json.dumps(compare(args.work_dir.resolve(), args.tolerance_ms), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
