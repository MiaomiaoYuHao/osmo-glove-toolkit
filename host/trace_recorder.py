#!/usr/bin/env python3
"""Thread-safe, replay-oriented trace recorder for the OSMO host tools.

The recorder deliberately uses only the Python standard library.  A trace
session keeps the exact COBS payloads in a compact binary stream and stores
decoded frames, processing stages, UI/system events and a flat analysis CSV as
JSON-lines/CSV files.  All records are correlated by ``trace_id``.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import platform
import queue
import socket
import struct
import threading
import time
import uuid
from collections import deque
from collections.abc import Mapping
from enum import Enum
from pathlib import Path
from typing import Any

TRACE_SCHEMA_VERSION = 1
RAW_MAGIC = b"OSMOTRC1"
RAW_RECORD = struct.Struct("<QQI")  # trace_id, rx_monotonic_ns, payload length

ANALYSIS_FIELDS = [
    # correlation / transport
    "trace_id", "connection_id", "sequence", "rx_wall_time_ns", "rx_wall_time",
    "rx_monotonic_ns", "rx_monotonic_s", "processing_monotonic_ns",
    "processing_latency_ms", "record_kind", "packet_len", "packet_sha256",
    "decode_ok", "decode_error", "source",
    # protobuf envelope
    "index", "finger", "finger_id", "link", "sensor_id", "kind",
    "mag_seconds", "mag_nanoseconds", "mag_device_time_s",
    "quat_seconds", "quat_nanoseconds", "quat_device_time_s",
    # raw protobuf values
    "mag_x_raw", "mag_y_raw", "mag_z_raw",
    "quat_x_raw", "quat_y_raw", "quat_z_raw", "quat_w_raw", "quat_accuracy_raw",
    # quaternion processing
    "quat_norm", "quat_w_norm", "quat_x_norm", "quat_y_norm", "quat_z_norm",
    "ref_quat_w", "ref_quat_x", "ref_quat_y", "ref_quat_z",
    "relative_quat_w", "relative_quat_x", "relative_quat_y", "relative_quat_z",
    "yaw_deg", "relative_yaw_deg",
    "motion_delta_deg", "motion_dt_s", "motion_rate_deg_s", "motion_median_deg_s",
    "acc_status", "acc_cal_result", "acc_diag", "acc_weight", "acc_fault",
    "acc_status_text", "acc_cal_text",
    # magnetometer processing
    "mag_scale", "mag_unit_scale",
    "mag_x", "mag_y", "mag_z",
    "mag_world_x", "mag_world_y", "mag_world_z",
    "paired_sensor_id", "paired_seen",
    "mag_secondary_x", "mag_secondary_y", "mag_secondary_z",
    "mag_common_x", "mag_common_y", "mag_common_z",
    "mag_diff_x", "mag_diff_y", "mag_diff_z",
    "mag_diff_baseline_x", "mag_diff_baseline_y", "mag_diff_baseline_z",
    "mag_diff_dynamic_x", "mag_diff_dynamic_y", "mag_diff_dynamic_z",
    "mag_diff_norm_raw", "mag_diff_norm_lp_prev", "mag_diff_norm_lp",
    "mag_norm_raw", "mag_norm_lp_prev", "mag_norm_lp",
    "mag_dt_s", "mag_alpha",
    "mag_state_before", "mag_state_after", "mag_state_since", "mag_decision",
    "mag_ref_x", "mag_ref_y", "mag_ref_z", "mag_ref_norm", "mag_current_norm",
    "mag_dot", "mag_angle_deg", "mag_dmag_rel", "mag_baseline_set",
    "mag_quality", "mag_magnitude_ut", "mag_noise_ut", "mag_jump_ut",
    # yaw / compass diagnostics
    "mag_yaw_deg", "output_yaw_deg", "yaw_error_deg",
    "pointer_zero_deg", "pointer_heading_lp_deg", "pointer_heading_delta_deg",
    "pointer_rebase_needed", "pointer_stable_since",
    # host / UI state
    "accepted", "drop_reason", "queue_depth", "ui_axis_mode",
    "ui_sensor_filter", "ui_mag_scale", "details_json",
]


def _json_safe(value: Any) -> Any:
    """Return a JSON-safe value without turning NaN/Inf into invalid JSON.

    快路径按 type() 恒等判断，先挡住绝大多数标量再谈容器 —— 这段每个录制
    项目都要跑一遍，用 typing.Mapping 做 isinstance 会走 typing 的
    __instancecheck__ 慢路径，实测能把写入线程吃满并把 UI 拖到掉帧。
    """
    t = type(value)
    if t is float:
        return value if math.isfinite(value) else None
    if t is str or t is int or t is bool or value is None:
        return value
    if t is dict:
        return {str(k): _json_safe(v) for k, v in value.items()}
    if t is list or t is tuple:
        return [_json_safe(v) for v in value]
    # ---- 以下为少见的子类 / 容器 ----
    if t is bytes or t is bytearray:
        return bytes(value).hex()
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (str, int, bool)):
        return value
    if isinstance(value, Mapping):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set, frozenset, deque)):
        return [_json_safe(v) for v in value]
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Enum):
        return _json_safe(value.value)
    if hasattr(value, "value") and not isinstance(value, (str, int, float, bool)):
        return _json_safe(getattr(value, "value"))
    return value


def _json_line(payload: Mapping[str, Any]) -> str:
    return json.dumps(
        _json_safe(dict(payload)),
        ensure_ascii=False,
        separators=(",", ":"),
        allow_nan=False,
    ) + "\n"


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _source_sha256(path: str | Path | None) -> str | None:
    if not path:
        return None
    try:
        return _sha256_bytes(Path(path).read_bytes())
    except Exception:
        return None


def _host_file_inventory(source_path: Path | None) -> dict[str, Any]:
    root = source_path.parent if source_path else Path(__file__).resolve().parent
    names = [
        "姿态测试上位机.pyw", "纯数据预览.pyw", "read_osmo_glove.py",
        "serial_trace_reader.py", "trace_recorder.py", "trace_session.py",
        "trace_replay.py", "replay_protocol.py", "extract_replay.py",
        "replay_firmware.py", "compare_replay.py", "verify_replay.py",
        "replay_engine.c",
        "build_replay_engine.ps1", "replay_engine.exe", "REPLAY_PROTOCOL.md",
        "firmware_diagnostics.py", "firmware_profile.py",
    ]
    result: dict[str, Any] = {}
    for name in names:
        path = root / name
        if path.exists():
            result[name] = {
                "bytes": path.stat().st_size,
                "sha256": _sha256_bytes(path.read_bytes()),
            }
    return result


def _dependency_versions() -> dict[str, Any]:
    versions: dict[str, Any] = {}
    for module_name in ("serial", "cobs", "betterproto"):
        try:
            module = __import__(module_name)
            versions[module_name] = getattr(module, "__version__", "unknown")
        except Exception as exc:
            versions[module_name] = f"unavailable: {type(exc).__name__}: {exc}"
    return versions


def _artifact_inventory(session_dir: Path) -> dict[str, Any]:
    artifacts: dict[str, Any] = {}
    for path in sorted(session_dir.iterdir()):
        if not path.is_file() or path.name in ("manifest.json", "manifest.tmp"):
            continue
        try:
            artifacts[path.name] = {
                "bytes": path.stat().st_size,
                "sha256": _sha256_bytes(path.read_bytes()),
            }
        except Exception as exc:
            artifacts[path.name] = {"error": f"{type(exc).__name__}: {exc}"}
    return artifacts

class TraceRecorder:
    """Queue-backed trace writer shared by serial threads and the Tk UI thread."""

    def __init__(self, app_name: str, source_path: str | Path | None = None):
        self.app_name = app_name
        self.source_path = Path(source_path).resolve() if source_path else None
        self.source_sha256 = _source_sha256(self.source_path)
        self._state_lock = threading.RLock()
        self._id_lock = threading.Lock()
        self._next_id = 1
        self._recording = False
        self._session_dir: Path | None = None
        self._queue: queue.Queue[tuple[str, dict[str, Any]] | object] | None = None
        self._sentinel = object()
        self._writer: threading.Thread | None = None
        self._files: dict[str, Any] = {}
        self._analysis_writer: csv.DictWriter | None = None
        self._manifest: dict[str, Any] = {}
        self._stats: dict[str, int] = {}
        self._errors: deque[dict[str, Any]] = deque(maxlen=40)
        self._last_flush = 0.0
        self._drop_notice: dict[str, int] = {}

    def next_trace_id(self) -> int:
        with self._id_lock:
            trace_id = self._next_id
            self._next_id += 1
            return trace_id

    @property
    def is_recording(self) -> bool:
        with self._state_lock:
            return self._recording

    @property
    def session_dir(self) -> Path | None:
        with self._state_lock:
            return self._session_dir

    def pop_errors(self) -> list[dict[str, Any]]:
        with self._state_lock:
            errors = list(self._errors)
            self._errors.clear()
            return errors

    def stats(self) -> dict[str, Any]:
        with self._state_lock:
            stats = dict(self._stats)
            stats.update({
                "recording": self._recording,
                "session_dir": str(self._session_dir) if self._session_dir else None,
                "queue_depth": self._queue.qsize() if self._queue is not None else 0,
                "errors": len(self._errors),
            })
            return stats

    def start_session(
        self,
        parent_dir: str | Path,
        *,
        metadata: Mapping[str, Any] | None = None,
        name: str | None = None,
    ) -> Path:
        """Start a new trace session and return its directory."""
        with self._state_lock:
            if self._recording:
                raise RuntimeError("trace recording is already active")
            root = Path(parent_dir)
            root.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            stem = name or f"{stamp}_{self.app_name}_trace"
            session_dir = root / stem
            suffix = 1
            while session_dir.exists():
                suffix += 1
                session_dir = root / f"{stem}_{suffix:02d}"
            session_dir.mkdir(parents=True)

            self._session_dir = session_dir
            self._queue = queue.Queue(maxsize=50000)
            self._stats = {
                "queued": 0, "written": 0, "dropped": 0,
                "raw_frames": 0, "raw_bytes": 0,
                "decoded_frames": 0, "decode_errors": 0,
                "analysis_rows": 0, "stage_records": 0, "serial_bytes": 0, "serial_chunks": 0,
                "firmware_lines": 0, "firmware_parsed": 0, "gate_events": 0,
                "system_records": 0, "event_records": 0,
            }
            self._errors.clear()
            self._drop_notice.clear()
            self._files = {}
            self._open_files(session_dir)
            metadata_dict = _json_safe(dict(metadata or {}))
            self._manifest = {
                "schema": {
                    "name": "osmo-host-trace",
                    "version": TRACE_SCHEMA_VERSION,
                    "raw_format": "magic OSMOTRC1 + repeated <QQI> trace_id,rx_monotonic_ns,len + COBS payload",
                },
                "session_id": uuid.uuid4().hex,
                "created_wall_time": time.time(),
                "created_wall_time_text": time.strftime("%Y-%m-%d %H:%M:%S"),
                "created_monotonic_ns": time.monotonic_ns(),
                "app": self.app_name,
                "source_file": str(self.source_path) if self.source_path else None,
                "source_sha256": self.source_sha256,
                "host_files": _host_file_inventory(self.source_path),
                "dependencies": _dependency_versions(),
                "python": platform.python_version(),
                "platform": platform.platform(),
                "hostname": socket.gethostname(),
                "cwd": os.getcwd(),
                "metadata": metadata_dict,
                "files": {
                    "raw_frames": "raw_frames.bin",
                    "frames": "frames.jsonl",
                    "stages": "stages.jsonl",
                    "events": "events.jsonl",
                    "system": "system.jsonl",
                    "analysis": "analysis.csv",
                    "serial_stream": "serial_stream.bin",
                    "serial_chunks": "serial_chunks.jsonl",
                    "firmware_diagnostics": "firmware_diagnostics.jsonl",
                    "gate_events": "gate_events.jsonl",
                },
                "state": "recording",
            }
            self._write_manifest()
            self._recording = True
            self._writer = threading.Thread(
                target=self._writer_loop,
                name=f"{self.app_name}-trace-writer",
                daemon=True,
            )
            self._writer.start()
        self.record_event("recorder", "session_started", {
            "session_dir": str(session_dir),
            "schema_version": TRACE_SCHEMA_VERSION,
        })
        return session_dir

    def stop_session(self, reason: str = "user", timeout: float = 8.0) -> Path | None:
        """Flush and close the current trace session."""
        with self._state_lock:
            if not self._recording:
                return self._session_dir
            session_dir = self._session_dir
            writer = self._writer
            q = self._queue
        self.record_event("recorder", "session_stopping", {"reason": reason})
        with self._state_lock:
            self._recording = False
        if q is not None:
            try:
                q.put(self._sentinel, timeout=2.0)
            except queue.Full:
                self._stats["dropped"] = self._stats.get("dropped", 0) + 1
        if writer is not None:
            writer.join(timeout=max(0.0, timeout))
        with self._state_lock:
            if writer is not None and writer.is_alive():
                self._errors.append({
                    "wall_time": time.time(),
                    "error": "trace writer did not stop before timeout",
                })
            ended = time.time()
            if self._manifest:
                self._manifest["state"] = "closed" if writer is None or not writer.is_alive() else "closing_timeout"
                self._manifest["ended_wall_time"] = ended
                self._manifest["ended_wall_time_text"] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(ended))
                self._manifest["duration_s"] = ended - float(self._manifest.get("created_wall_time", ended))
                self._manifest["stats"] = dict(self._stats)
                self._manifest["drop_notice"] = dict(self._drop_notice)
                self._manifest["artifacts"] = _artifact_inventory(session_dir) if session_dir is not None else {}
                self._write_manifest()
            return session_dir

    def _open_files(self, session_dir: Path) -> None:
        self._files = {
            "raw": (session_dir / "raw_frames.bin").open("wb"),
            "frames": (session_dir / "frames.jsonl").open("w", encoding="utf-8", newline="\n"),
            "stages": (session_dir / "stages.jsonl").open("w", encoding="utf-8", newline="\n"),
            "events": (session_dir / "events.jsonl").open("w", encoding="utf-8", newline="\n"),
            "system": (session_dir / "system.jsonl").open("w", encoding="utf-8", newline="\n"),
            "analysis": (session_dir / "analysis.csv").open("w", encoding="utf-8", newline="", buffering=1),
            "serial_stream": (session_dir / "serial_stream.bin").open("wb"),
            "serial_chunks": (session_dir / "serial_chunks.jsonl").open("w", encoding="utf-8", newline="\n"),
            "firmware_diagnostics": (session_dir / "firmware_diagnostics.jsonl").open("w", encoding="utf-8", newline="\n"),
            "gate_events": (session_dir / "gate_events.jsonl").open("w", encoding="utf-8", newline="\n"),
        }
        self._files["raw"].write(RAW_MAGIC)
        self._analysis_writer = csv.DictWriter(
            self._files["analysis"],
            fieldnames=ANALYSIS_FIELDS,
            extrasaction="ignore",
            restval="",
        )
        self._analysis_writer.writeheader()

    def _write_manifest(self) -> None:
        with self._state_lock:
            session_dir = self._session_dir
            manifest = dict(self._manifest)
        if session_dir is None:
            return
        tmp = session_dir / "manifest.tmp"
        final = session_dir / "manifest.json"
        try:
            tmp.write_text(
                json.dumps(_json_safe(manifest), ensure_ascii=False, indent=2, allow_nan=False),
                encoding="utf-8",
            )
            tmp.replace(final)
        except Exception as exc:
            with self._state_lock:
                self._errors.append({"wall_time": time.time(), "error": f"manifest write failed: {exc}"})

    def _enqueue(self, kind: str, payload: dict[str, Any]) -> None:
        with self._state_lock:
            if not self._recording or self._queue is None:
                return
            q = self._queue
            self._stats["queued"] = self._stats.get("queued", 0) + 1
        try:
            q.put_nowait((kind, payload))
        except queue.Full:
            with self._state_lock:
                self._stats["dropped"] = self._stats.get("dropped", 0) + 1
                self._drop_notice[kind] = self._drop_notice.get(kind, 0) + 1

    def record_frame(
        self,
        *,
        trace_id: int,
        connection_id: int,
        sequence: int,
        rx_wall_time_ns: int,
        rx_monotonic_ns: int,
        raw_packet: bytes,
        decoded: Mapping[str, Any] | None,
        protobuf_payload: bytes | None = None,
        decode_error: str | None,
        kind: str,
        read_overflow: bool = False,
        source_trace_id: int | None = None,
        replay: bool = False,
    ) -> None:
        payload: dict[str, Any] = {
            "record_type": "frame",
            "trace_id": trace_id,
            "connection_id": connection_id,
            "sequence": sequence,
            "rx_wall_time_ns": rx_wall_time_ns,
            "rx_wall_time": rx_wall_time_ns / 1e9,
            "rx_monotonic_ns": rx_monotonic_ns,
            "packet_len": len(raw_packet),
            "packet_sha256": _sha256_bytes(raw_packet),
            "packet_hex": raw_packet.hex(),
            "protobuf_len": len(protobuf_payload) if protobuf_payload is not None else None,
            "protobuf_sha256": _sha256_bytes(protobuf_payload) if protobuf_payload is not None else None,
            "protobuf_hex": protobuf_payload.hex() if protobuf_payload is not None else None,
            "decode_ok": decoded is not None and decode_error is None,
            "decode_error": decode_error,
            "kind": kind,
            "read_overflow": bool(read_overflow),
            "source_trace_id": source_trace_id,
            "replay": bool(replay),
            "decoded": dict(decoded) if decoded is not None else None,
            "_raw_bytes": bytes(raw_packet),
        }
        self._enqueue("frame", payload)

    def record_serial_bytes(
        self,
        *,
        connection_id: int,
        data: bytes,
        first_rx_monotonic_ns: int | None = None,
        last_rx_monotonic_ns: int | None = None,
    ) -> None:
        if not data:
            return
        self._enqueue("serial_bytes", {
            "record_type": "serial_bytes",
            "connection_id": connection_id,
            "length": len(data),
            "first_rx_monotonic_ns": first_rx_monotonic_ns,
            "last_rx_monotonic_ns": last_rx_monotonic_ns,
            "sha256": _sha256_bytes(data),
            "_raw_bytes": bytes(data),
        })
    def record_firmware_line(
        self,
        *,
        connection_id: int,
        line: str,
        parsed: Mapping[str, Any],
        rx_wall_time_ns: int,
        rx_monotonic_ns: int,
    ) -> None:
        self._enqueue("firmware_diagnostic", {
            "record_type": "firmware_diagnostic",
            "connection_id": connection_id,
            "rx_wall_time_ns": rx_wall_time_ns,
            "rx_wall_time": rx_wall_time_ns / 1e9,
            "rx_monotonic_ns": rx_monotonic_ns,
            "line": line,
            "parsed_type": parsed.get("type", "text"),
            "parsed": dict(parsed),
        })
    def record_gate_event(self, payload: Mapping[str, Any]) -> None:
        self._enqueue("gate_event", {
            "record_type": "gate_event",
            "wall_time": time.time(),
            "monotonic_ns": time.monotonic_ns(),
            **dict(payload),
        })
    def record_stage(self, stage: str, payload: Mapping[str, Any]) -> None:
        self._enqueue("stage", {
            "record_type": "stage",
            "stage": stage,
            **dict(payload),
        })

    def record_analysis(self, payload: Mapping[str, Any]) -> None:
        self._enqueue("analysis", {
            "record_type": "analysis",
            **dict(payload),
        })

    def record_event(self, category: str, name: str, data: Mapping[str, Any] | None = None) -> None:
        self._enqueue("event", {
            "record_type": "event",
            "wall_time": time.time(),
            "monotonic_ns": time.monotonic_ns(),
            "category": category,
            "event": name,
            "data": _json_safe(dict(data or {})),
        })

    def record_command(self, command: bytes, intent: str, result: str, detail: str | None = None) -> None:
        self.record_event("command", "serial_write", {
            "command_hex": command.hex(),
            "command_text": command.decode("ascii", "replace"),
            "intent": intent,
            "result": result,
            "detail": detail,
        })

    def record_system(self, payload: Mapping[str, Any]) -> None:
        self._enqueue("system", {
            "record_type": "system",
            "wall_time": time.time(),
            "monotonic_ns": time.monotonic_ns(),
            **dict(payload),
        })

    def _writer_loop(self) -> None:
        try:
            while True:
                try:
                    item = self._queue.get(timeout=0.25) if self._queue is not None else self._sentinel
                except queue.Empty:
                    self._flush()
                    continue

                stop_after_batch = item is self._sentinel
                batch = [] if stop_after_batch else [item]
                if self._queue is not None:
                    while len(batch) < 500:
                        try:
                            next_item = self._queue.get_nowait()
                        except queue.Empty:
                            break
                        if next_item is self._sentinel:
                            stop_after_batch = True
                            break
                        batch.append(next_item)

                for kind, payload in batch:
                    self._write_item(kind, payload)
                    with self._state_lock:
                        self._stats["written"] = self._stats.get("written", 0) + 1
                self._flush()
                if stop_after_batch:
                    break
        except Exception as exc:
            with self._state_lock:
                self._errors.append({
                    "wall_time": time.time(),
                    "error": f"trace writer failed: {exc}",
                })
        finally:
            try:
                self._flush()
            except Exception:
                pass
            with self._state_lock:
                for handle in self._files.values():
                    try:
                        handle.close()
                    except Exception:
                        pass
                self._files.clear()
                self._analysis_writer = None
                self._writer = None

    def _write_item(self, kind: str, payload: dict[str, Any]) -> None:
        if kind == "frame":
            raw = payload.pop("_raw_bytes", b"")
            self._files["raw"].write(RAW_RECORD.pack(
                int(payload["trace_id"]),
                int(payload["rx_monotonic_ns"]),
                len(raw),
            ))
            self._files["raw"].write(raw)
            self._files["frames"].write(_json_line(payload))
            with self._state_lock:
                self._stats["raw_frames"] = self._stats.get("raw_frames", 0) + 1
                self._stats["raw_bytes"] = self._stats.get("raw_bytes", 0) + len(raw)
                if payload.get("decode_ok"):
                    self._stats["decoded_frames"] = self._stats.get("decoded_frames", 0) + 1
                else:
                    self._stats["decode_errors"] = self._stats.get("decode_errors", 0) + 1
            return
        if kind == "serial_bytes":
            raw = payload.pop("_raw_bytes", b"")
            stream = self._files["serial_stream"]
            offset = stream.tell()
            stream.write(raw)
            payload["offset"] = offset
            payload["hex"] = raw.hex()
            self._files["serial_chunks"].write(_json_line(payload))
            with self._state_lock:
                self._stats["serial_bytes"] = self._stats.get("serial_bytes", 0) + len(raw)
                self._stats["serial_chunks"] = self._stats.get("serial_chunks", 0) + 1
            return
        if kind == "analysis":
            if self._analysis_writer is not None:
                self._analysis_writer.writerow(_json_safe(payload))
            with self._state_lock:
                self._stats["analysis_rows"] = self._stats.get("analysis_rows", 0) + 1
            return
        if kind == "firmware_diagnostic":
            self._files["firmware_diagnostics"].write(_json_line(payload))
            with self._state_lock:
                self._stats["firmware_lines"] = self._stats.get("firmware_lines", 0) + 1
                if payload.get("parsed_type") != "text":
                    self._stats["firmware_parsed"] = self._stats.get("firmware_parsed", 0) + 1
            return
        if kind == "gate_event":
            self._files["gate_events"].write(_json_line(payload))
            with self._state_lock:
                self._stats["gate_events"] = self._stats.get("gate_events", 0) + 1
            return
        if kind == "stage":
            self._files["stages"].write(_json_line(payload))
            with self._state_lock:
                self._stats["stage_records"] = self._stats.get("stage_records", 0) + 1
            return
        if kind == "event":
            self._files["events"].write(_json_line(payload))
            with self._state_lock:
                self._stats["event_records"] = self._stats.get("event_records", 0) + 1
            return
        if kind == "system":
            self._files["system"].write(_json_line(payload))
            with self._state_lock:
                self._stats["system_records"] = self._stats.get("system_records", 0) + 1

    def _flush(self) -> None:
        now = time.monotonic()
        if now - self._last_flush < 0.25:
            return
        self._last_flush = now
        for handle in self._files.values():
            try:
                handle.flush()
            except Exception:
                pass


__all__ = ["TraceRecorder", "ANALYSIS_FIELDS", "TRACE_SCHEMA_VERSION"]