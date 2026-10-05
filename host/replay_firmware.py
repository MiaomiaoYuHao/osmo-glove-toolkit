#!/usr/bin/env python3
"""Run the exact mag_fusion algorithm offline from a recording.

Pipeline:
  recording -> extract_replay.py -> replay_raw_inputs.csv + snapshots
            -> replay_engine.exe -> replay_engine_output.csv

The engine loads the recorded ``mag_fusion_t`` snapshot and calls the same
``mag_fusion.c`` functions in the recorded input order.  It does not import or
simulate the host pose algorithm.
"""

from __future__ import annotations

import argparse
import csv
import json
import struct
import subprocess
import sys
from pathlib import Path
from typing import Any

APP = Path(__file__).resolve().parent
sys.path.insert(0, str(APP))

import replay_protocol as rp  # noqa: E402
from extract_replay import extract  # noqa: E402

HEADER = struct.Struct("<8sIIII")
RECORD = struct.Struct("<B3xIQ5f")
KIND_TO_CODE = {
    "RAW_QUAT": 0,
    "RAW_MAG": 1,
    "RAW_GYRO": 2,
    "RAW_ACC": 3,
    "RAW_MAG2": 4,
}


def _load_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def run(session_dir: Path, *, link: int | None = None, output: Path | None = None) -> dict[str, Any]:
    session_dir = session_dir.resolve()
    work = output.resolve() if output else session_dir / "replay_firmware"
    raw_csv = work / "replay_raw_inputs.csv"
    if not raw_csv.exists():
        extract(session_dir, work)

    rows = _load_rows(raw_csv)
    if not rows:
        raise RuntimeError("replay_raw_inputs.csv is empty; this is not a REPLAYBIN recording")
    links = sorted({int(row.get("link") or 0) for row in rows})
    selected_link = links[0] if link is None else int(link)
    rows = [row for row in rows if int(row.get("link") or 0) == selected_link]
    if not rows:
        raise RuntimeError(f"no replay frames for link {selected_link}")

    state_path = work / "snapshots" / f"FUSION_STATE_link{selected_link}.bin"
    if not state_path.exists():
        raise RuntimeError(f"missing {state_path}; request the D snapshot before recording")
    state = state_path.read_bytes()

    config_path = work / "snapshots" / f"CONFIG_link{selected_link}.bin"
    config: dict[str, Any] | None = None
    if config_path.exists():
        try:
            config = rp.parse_config(config_path.read_bytes())
        except Exception as exc:
            config = {"error": str(exc)}
        if isinstance(config, dict) and config.get("size_mag_fusion") not in (None, len(state)):
            raise RuntimeError(
                f"state size mismatch: snapshot={len(state)} config={config.get('size_mag_fusion')}"
            )

    input_path = work / "replay_input.bin"
    output_path = work / "replay_engine_output.csv"
    with input_path.open("wb") as handle:
        handle.write(HEADER.pack(b"MFREPLAY", 1, len(state), selected_link, 0))
        handle.write(state)
        written = 0
        for row in rows:
            kind = KIND_TO_CODE.get(str(row.get("kind")))
            if kind is None:
                continue
            ticks = row.get("device_ticks")
            if ticks in (None, ""):
                seconds = row.get("device_time_s")
                ticks = int(round(float(seconds) / 1.5625e-5)) if seconds not in (None, "") else 0
            else:
                ticks = int(ticks)
            values = [float(row.get(f"v{i}") or 0.0) for i in range(5)]
            handle.write(RECORD.pack(kind, int(row.get("index") or 0), ticks, *values))
            written += 1

    engine = APP / "replay_engine.exe"
    if not engine.exists():
        subprocess.run(
            ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(APP / "build_replay_engine.ps1")],
            check=True,
        )
    subprocess.run([str(engine), str(input_path), str(output_path)], check=True)
    summary = {
        "session": str(session_dir),
        "link": selected_link,
        "records": written,
        "state_bytes": len(state),
        "input": str(input_path),
        "output": str(output_path),
        "config": config,
    }
    (work / "replay_firmware_manifest.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay mag_fusion offline from a recording.")
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--link", type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    summary = run(args.session_dir, link=args.link, output=args.output)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
