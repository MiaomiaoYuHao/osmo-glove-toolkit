#!/usr/bin/env python3
"""Data-only replay/debug protocol for the Bowie glove firmware.

The wire format is the existing COBS/protobuf transport.  New replay records
use synthetic ``sensor_id`` values and carry values only; all names, ordering
and scaling live in this file.  Keep this in lockstep with
``Core/Inc/glove/bhi360.h`` and the emitters in ``Core/Src/glove/*.c``.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Any, Iterable

PROTOCOL_VERSION = 2

REPLAY_QUAT_ID = 240
REPLAY_MAG_ID = 241
REPLAY_GYRO_ID = 242
REPLAY_ACC_ID = 243
REPLAY_MAG2_ID = 244
REPLAY_CAL_ID = 245
REPLAY_STATE_ID = 246
REPLAY_YF_ID = 247
REPLAY_YAW_ID = 248
REPLAY_YAW2_ID = 249
REPLAY_CONFIG_ID = 250
REPLAY_EVENT_ID = 251
REPLAY_HEALTH_ID = 252
REPLAY_CLOCK_ID = 253

REPLAY_IDS = {
    REPLAY_QUAT_ID: "RAW_QUAT",
    REPLAY_MAG_ID: "RAW_MAG",
    REPLAY_GYRO_ID: "RAW_GYRO",
    REPLAY_ACC_ID: "RAW_ACC",
    REPLAY_MAG2_ID: "RAW_MAG2",
    REPLAY_CAL_ID: "CAL_BLOB",
    REPLAY_STATE_ID: "FUSION_STATE",
    REPLAY_YF_ID: "YF",
    REPLAY_YAW_ID: "YAW",
    REPLAY_YAW2_ID: "YAW2",
    REPLAY_CONFIG_ID: "CONFIG",
    REPLAY_EVENT_ID: "EVENT",
    REPLAY_HEALTH_ID: "HEALTH",
    REPLAY_CLOCK_ID: "CLOCK",
}

YF_FIELDS = [
    "innov_deg", "err_deg", "out_yaw_deg", "gamerv_yaw_deg", "step_deg",
    "relax", "hold_s", "slew_dps", "trust", "weight", "dir_res_deg",
    "large", "snap", "clean", "kalman",
    "traw_deg", "target_deg", "yaw_corr_deg", "datum_deg", "blend_deg",
    "hard_event", "rejecting", "e_inst", "e_lp", "e_step",
    "e_norm", "e_dip", "gyro_activity", "safe_target_deg", "diag_flags",
]

YAW_FIELDS = [
    "t_ms", "innov_deg", "traw_deg", "target_deg", "out_yaw_deg", "err_deg",
    "relax", "hold_s", "slew_dps", "trust", "large", "snap", "clean",
    "nis_big", "kalman", "step_deg", "yaw_corr_deg", "datum_deg",
    "blend_deg", "weight", "status", "nis", "yawP", "R", "K", "sig",
    "horiz", "geo", "omega_dps", "dirlock_ms",
]

YAW2_FIELDS = [
    "t_ms", "dir_res_deg", "e_inst", "e_lp", "norm_now", "dip_now_deg",
    "ref_dip_deg", "ref_norm", "sigma", "coast", "stuck_s",
    "drift_mdeg_s", "grad", "grad_thr", "fe_n", "fe_obs", "fe_res",
    "fe_d_norm", "fe_B_norm", "cand_s", "diag_hold", "rejecting",
    "hard_event", "cand_valid", "yaw_valid", "yaw_blend_valid",
    "learn_state", "learn_opens", "cal_valid", "cal_active", "cal_model",
    "cal_revision", "quality", "accuracy", "map_valid", "align_valid",
    "ref_dip_valid", "gamerv_yaw_deg", "mag_raw_x", "mag_raw_y", "mag_raw_z",
    "dir_strength_eff",
]

HEALTH_FIELDS = [
    "tick_ms", "q_count", "mag_count", "gyro_count", "acc_count",
    "mag2_count", "cdc_errors", "event_seq", "fifo_overflows",
    "meta_errors", "sensor_errors", "hard_events", "reject_events",
    "recal_events", "cal_revision", "cal_valid", "cal_active",
    "learn_state", "learn_opens", "hard_event", "rejecting",
    "ref_dip_valid", "fe_field_valid", "last_bhi_ts_lo", "last_bhi_ts_hi",
    "fw_tag", "schema_version", "data_only", "mag_dup_drop", "reserved1",
]

EVENT_NAMES = {
    1: "snapshot_begin",
    2: "snapshot_end",
    5: "calibration_loaded",
    6: "calibration_stored",
    7: "calibration_erased",
    8: "hard_iron_start",
    9: "hard_iron_finish",
    10: "hard_iron_clear",
    11: "relearn",
    12: "lock",
    13: "rebaseline",
    14: "auto_recal_start",
    15: "auto_recal_success",
    16: "auto_recal_timeout",
    17: "calibration_revision",
    18: "hard_event_enter",
    19: "hard_event_exit",
    20: "reject_enter",
    21: "reject_exit",
    22: "learn_state_change",
    23: "reload_saved_calibration",
    24: "fifo_overflow",
    25: "sensor_error",
    26: "meta_error",
    27: "clock_sync",
    30: "bsx_initialized",
    31: "bsx_reset",
    32: "sensor_status",
    33: "sample_rate_changed",
    34: "sensor_framework",
}

CONFIG_HEADER_FIELDS = [
    "magic", "schema_version", "fw_tag", "build_crc",
    "size_mag_fusion", "size_mag_cal_blob", "layout_hash",
    "off_cal", "off_A", "off_fe", "off_safe_B", "off_wring",
    "off_yaw_corr", "off_mag_datum", "off_status", "off_yawd_innov",
]

# name, scale (wire integer -> physical value)
CONFIG_PARAM_FIELDS = [
    ("yaw_innov_max", 100.0), ("yaw_nis_max", 10.0),
    ("yaw_innov_exit", 100.0), ("yaw_nis_exit", 10.0),
    ("slew_rad_s", 1.0), ("slew_fast_rad_s", 1.0),
    ("hold_ms", 1.0), ("hold_fast_ms", 1.0),
    ("relax_lo_deg", 1.0), ("relax_hi_deg", 1.0),
    ("relax_max", 100.0), ("snap_deg", 1.0),
    ("fast_inst_max", 100.0), ("fast_lp_max", 100.0),
    ("trust_rise_ms", 1000.0), ("trust_fall_ms", 1000.0),
    ("slew_omega_max_rad_s", 1.0), ("dir_res_deg", 100.0),
    ("dir_lock_ms", 1.0), ("dir_omega_max_rad_s", 1.0),
    ("norm_tol", 100.0), ("norm_sigma", 10.0),
    ("dip_tol_deg", 100.0), ("dip_sigma", 10.0),
    ("step_sigma", 10.0), ("step_floor", 1000.0),
    ("step_omega_k", 1000.0), ("drift_sigma", 10.0),
    ("drift_floor_mdps", 57295.7795), ("drift_omega_k", 1000.0),
    ("drift_run", 1.0), ("event_enter", 100.0),
    ("reject_at", 100.0), ("recover_ms", 1000.0),
    ("e_tau_ms", 1000.0), ("stuck_recal_s", 1.0),
    ("cal_window_s", 1.0), ("cal_full_min", 1.0),
    ("cal_bins_full", 1.0), ("stab_static_ds", 10.0),
    ("stab_motion_ms", 1000.0), ("stab_hold", 1.0),
    ("stab_breakout", 1000.0), ("fe_min_n", 1.0),
    ("fe_obs_max", 100.0), ("fe_res_max", 100.0),
    ("fe_every", 1.0), ("data_only", 1.0),
]

CONFIG_STRUCT = struct.Struct("<16I48i")

SCHEMAS = {
    "RAW_QUAT": ("quat", ["x", "y", "z", "w", "accuracy"]),
    "RAW_MAG": ("mag", ["x", "y", "z"]),
    "RAW_GYRO": ("mag", ["x", "y", "z"]),
    "RAW_ACC": ("mag", ["x", "y", "z"]),
    "RAW_MAG2": ("mag", ["x", "y", "z"]),
    "YF": ("diag6", YF_FIELDS),
    "YAW": ("diag6", YAW_FIELDS),
    "YAW2": ("diag6", YAW2_FIELDS),
    "HEALTH": ("diag6", HEALTH_FIELDS),
    "EVENT": ("event", ["code", "tick_ms", "a", "b"]),
}


@dataclass(frozen=True)
class DecodedReplayFrame:
    sensor_id: int
    kind: str
    index: int | None
    link: int | None
    device_time_s: float | None
    values: tuple[int | float, ...]


def kind_for_sensor(sensor_id: int | None) -> str | None:
    if sensor_id is None:
        return None
    return REPLAY_IDS.get(int(sensor_id))


def is_replay_sensor(sensor_id: int | None) -> bool:
    return kind_for_sensor(sensor_id) is not None


def _float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _word(value: Any) -> int:
    return _int(value) & 0xFFFFFFFF


def _packet(row: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    packet = row.get("packet") or {}
    return (packet.get("mag") or {}), (packet.get("quat") or {})


def decode_replay_row(row: dict[str, Any]) -> DecodedReplayFrame | None:
    """Decode a ``message_to_row`` result without doing any GUI work."""
    sid = _int(row.get("sensor_id"), -1)
    kind = kind_for_sensor(sid)
    if kind is None:
        return None

    raw_index = row.get("index")
    link = row.get("link")
    index: int | None
    if kind in ("YF", "YAW", "YAW2", "HEALTH", "CAL_BLOB", "FUSION_STATE", "CONFIG"):
        index = _word(raw_index) if raw_index is not None else None
    else:
        index = _int(raw_index) if raw_index is not None else None
    mag, quat = _packet(row)

    if kind == "RAW_QUAT":
        values = tuple(_float(quat.get(k)) for k in ("x", "y", "z", "w", "accuracy"))
        seconds, nanoseconds = _int(quat.get("seconds")), _int(quat.get("nanoseconds"))
    elif kind in ("RAW_MAG", "RAW_GYRO", "RAW_ACC", "RAW_MAG2"):
        values = tuple(_float(mag.get(k)) for k in ("x", "y", "z"))
        seconds, nanoseconds = _int(mag.get("seconds")), _int(mag.get("nanoseconds"))
    elif kind in ("YF", "YAW", "YAW2", "HEALTH"):
        values = (
            _float(quat.get("x")), _float(quat.get("y")), _float(quat.get("z")),
            _float(quat.get("w")), _float(quat.get("accuracy")), _float(mag.get("x")),
        )
        seconds = nanoseconds = 0
    elif kind == "EVENT":
        values = (
            _int(mag.get("seconds")), _int(mag.get("nanoseconds")),
            _int(quat.get("seconds")), _int(quat.get("nanoseconds")),
        )
        seconds = nanoseconds = 0
    else:  # raw snapshot chunks
        values = (
            _word(mag.get("seconds")), _word(mag.get("nanoseconds")),
            _word(quat.get("seconds")), _word(quat.get("nanoseconds")),
        )
        seconds = nanoseconds = 0

    device_time_s = (seconds + nanoseconds * 1e-9) if (seconds or nanoseconds) else None
    return DecodedReplayFrame(
        sensor_id=sid,
        kind=kind,
        index=index,
        link=_int(link) if link is not None else None,
        device_time_s=device_time_s,
        values=values,
    )


def decode_event(frame: DecodedReplayFrame) -> dict[str, Any]:
    if frame.kind != "EVENT":
        raise ValueError("not an EVENT frame")
    code, tick_ms, a, b = (int(v) for v in frame.values)
    return {
        "event_code": code,
        "event": EVENT_NAMES.get(code, f"unknown_{code}"),
        "tick_ms": tick_ms,
        "a": a,
        "b": b,
        "link": frame.link,
    }


def decode_diag_frame(frame: DecodedReplayFrame) -> dict[str, float]:
    if frame.kind not in ("YF", "YAW", "YAW2", "HEALTH"):
        raise ValueError(f"not a diagnostic frame: {frame.kind}")
    fields = {
        "YF": YF_FIELDS, "YAW": YAW_FIELDS, "YAW2": YAW2_FIELDS,
        "HEALTH": HEALTH_FIELDS,
    }[frame.kind]
    chunk = int(frame.index or 0) & 0x0F
    if frame.kind == "YF":
        starts = (0, 6, 12, 15, 21, 27)
        start = starts[chunk] if chunk < len(starts) else chunk * 6
    else:
        start = chunk * 6
    result: dict[str, float] = {}
    for offset, value in enumerate(frame.values):
        position = start + offset
        if position < len(fields):
            result[fields[position]] = float(value)
    return result


def assemble_diagnostic(frames: Iterable[DecodedReplayFrame]) -> list[dict[str, float]]:
    """Assemble 6-float chunks into one row per (kind, sample_id, link)."""
    grouped: dict[tuple[str, int, int | None], dict[str, float]] = {}
    order: list[tuple[str, int, int | None]] = []
    for frame in frames:
        if frame.kind not in ("YF", "YAW", "YAW2", "HEALTH"):
            continue
        sample_id = (int(frame.index or 0) & 0xFFFFFFFF) >> 4
        key = (frame.kind, sample_id, frame.link)
        if key not in grouped:
            grouped[key] = {}
            order.append(key)
        grouped[key].update(decode_diag_frame(frame))
    return [grouped[key] for key in order]


def _rows_to_bytes(rows: list[dict[str, Any]]) -> bytes:
    out = bytearray()
    for row in rows:
        mag, quat = _packet(row)
        out.extend(struct.pack(
            "<IIII",
            _word(mag.get("seconds")), _word(mag.get("nanoseconds")),
            _word(quat.get("seconds")), _word(quat.get("nanoseconds")),
        ))
    return bytes(out)


def assemble_blobs(rows: list[dict[str, Any]]) -> dict[tuple[str, int], bytes]:
    """Reassemble 245/246/250 snapshots, one entry per (kind, link)."""
    headers: dict[tuple[str, int], dict[str, Any]] = {}
    chunks: dict[tuple[str, int], dict[int, dict[str, Any]]] = {}
    for row in rows:
        kind = kind_for_sensor(row.get("sensor_id"))
        if kind not in ("CAL_BLOB", "FUSION_STATE", "CONFIG"):
            continue
        try:
            link = int(row.get("link") or 0)
        except (TypeError, ValueError):
            link = 0
        key = (kind, link)
        index = _word(row.get("index"))
        if index == 0xFFFFFFFF:
            headers[key] = row
        else:
            chunks.setdefault(key, {})[index] = row

    result: dict[tuple[str, int], bytes] = {}
    for key, header in headers.items():
        mag, _quat = _packet(header)
        length = _word(mag.get("seconds"))
        data = bytearray()
        for index in sorted(chunks.get(key, {})):
            data.extend(_rows_to_bytes([chunks[key][index]]))
        result[key] = bytes(data[:length])
    return result


def parse_config(blob: bytes) -> dict[str, Any]:
    """Decode the packed replay_config_t snapshot."""
    if len(blob) != CONFIG_STRUCT.size:
        raise ValueError(f"CONFIG size {len(blob)} != {CONFIG_STRUCT.size}")
    unpacked = CONFIG_STRUCT.unpack(blob)
    header = dict(zip(CONFIG_HEADER_FIELDS, unpacked[:16]))
    params: dict[str, float] = {}
    for (name, scale), value in zip(CONFIG_PARAM_FIELDS, unpacked[16:]):
        params[name] = value / scale
    return {**header, "params": params}


__all__ = [
    "PROTOCOL_VERSION", "REPLAY_QUAT_ID", "REPLAY_MAG_ID", "REPLAY_GYRO_ID",
    "REPLAY_ACC_ID", "REPLAY_MAG2_ID", "REPLAY_CAL_ID", "REPLAY_STATE_ID",
    "REPLAY_YF_ID", "REPLAY_YAW_ID", "REPLAY_YAW2_ID", "REPLAY_CONFIG_ID",
    "REPLAY_EVENT_ID", "REPLAY_HEALTH_ID", "REPLAY_CLOCK_ID", "REPLAY_IDS",
    "YF_FIELDS", "YAW_FIELDS", "YAW2_FIELDS", "HEALTH_FIELDS",
    "CONFIG_HEADER_FIELDS", "CONFIG_PARAM_FIELDS",
    "DecodedReplayFrame", "kind_for_sensor", "is_replay_sensor",
    "decode_replay_row", "decode_event", "decode_diag_frame",
    "assemble_diagnostic", "assemble_blobs", "parse_config",
]
