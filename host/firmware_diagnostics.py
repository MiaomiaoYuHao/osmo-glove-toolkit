#!/usr/bin/env python3
"""Parsers for FIX14/FIX20 diagnostic text emitted on the OSMO CDC stream.

Firmware text shares the COBS/protobuf CDC endpoint with binary frames.  The
raw bytes are always retained separately; this module turns recognised FIX14/FIX20
lines into stable structured records and keeps the complete original line.
"""

from __future__ import annotations

import re
from typing import Any

Q_SCALE = 16384.0

SINFO_OK_RE = re.compile(
    r"^SINFO\s+l(?P<link>\d+)\s+id(?P<sensor_id>\d+)\s+"
    r"(?P<sensor_name>.+?)\s+drv(?P<driver_id>\d+)\s+"
    r"range(?P<range_raw>\d+)\s+res(?P<resolution_raw>\d+)\s+"
    r"maxr(?P<max_rate_x100>\d+)\s+minr(?P<min_rate_x100>\d+)$"
)
SINFO_ERR_RE = re.compile(
    r"^SINFO\s+l(?P<link>\d+)\s+id(?P<sensor_id>\d+)\s+"
    r"(?P<sensor_name>.+?)\s+err(?P<error>-?\d+)$"
)
GQ_RE = re.compile(
    r"^GQ\s+f(?P<finger>\d+)\s+l(?P<link>\d+)\s+id(?P<sensor_id>\d+)\s+"
    r"acc(?P<accuracy_raw>-?\d+)\s+raw\s+"
    r"(?P<qx_raw>-?\d+)\s+(?P<qy_raw>-?\d+)\s+"
    r"(?P<qz_raw>-?\d+)\s+(?P<qw_raw>-?\d+)\s+"
    r"yawc\s+(?P<yaw_corr_x100>-?\d+)$"
)
GY_RE = re.compile(
    r"^GY\s+f(?P<finger>\d+)\s+l(?P<link>\d+)\s+"
    r"P\s+(?P<px>-?\d+)\s+(?P<py>-?\d+)\s+(?P<pz>-?\d+)\s+"
    r"C\s+(?P<cx>-?\d+)\s+(?P<cy>-?\d+)\s+(?P<cz>-?\d+)\s+"
    r"CV(?P<corrected_valid>\d+)\s+"
    r"B\s+(?P<bx_x10>-?\d+)\s+(?P<by_x10>-?\d+)\s+(?P<bz_x10>-?\d+)\s+"
    r"ACT\s+(?P<activity_x10>-?\d+)\s+MH\s+(?P<motion_hold>\d+)$"
)
MAG_RE = re.compile(
    r"^MAG\s+f(?P<finger>\d+)\s+l(?P<link>\d+)\s+"
    r"RAW\s+(?P<raw_x>-?\d+)\s+(?P<raw_y>-?\d+)\s+(?P<raw_z>-?\d+)\s+\|\s+"
    r"CAL\s+(?P<cal_x_x100>-?\d+)\s+(?P<cal_y_x100>-?\d+)\s+(?P<cal_z_x100>-?\d+)\s+\|\s+"
    r"OFF\s+(?P<off_x_x100>-?\d+)\s+(?P<off_y_x100>-?\d+)\s+(?P<off_z_x100>-?\d+)\s+\|\s+"
    r"REV\s+(?P<revision>\d+)\s+FIX\s+(?P<fixes>\d+)\s+"
    r"OPENS\s+(?P<learning_opens>\d+)\s+LRN\s+(?P<learning_state>\d+)\s+\|\s+"
    r"DATUM\s+(?P<datum_deg_x100>-?\d+)\s+YAWC\s+(?P<yaw_corr_deg_x100>-?\d+)$"
)
MAG_STATUS_RE = re.compile(r"^\s*(?P<link>\d+)\s+(?P<status>NO_CAL|CAL|ACQ|LOCKED|DEGRADED|COAST)\s+")
MAG_STATUS_FIELDS = [
    "link", "status", "trust_pct", "radius1", "radius2", "model", "resid_pct",
    "align", "grad", "grad_threshold", "drift_mdeg_s", "dip_deg",
    "stuck_s", "fixes", "fe_n", "fe_obs_x1000", "fe_res_x1000",
    "fe_d_x1000", "fe_b_x1000", "fe_a_x1000", "geo_x1000",
    "e_step_x1000", "e_drift_x1000", "e_lp_x1000", "e_norm_x1000",
    "e_dip_x1000", "e_grad_x1000", "e_fleet_x1000", "weight_pct",
    "rejecting", "hard_event", "candidate_valid", "candidate_s_x100",
    "good_s_x100", "e_inst_x1000", "datum_deg", "yaw_corr_deg",
    "revision", "learning_state", "learning_opens",
]


def _int(value: str) -> int:
    return int(value)


def _pct(value: str) -> float:
    return float(value.rstrip("%")) / 100.0


def _parse_status_row(line: str) -> dict[str, Any] | None:
    match = MAG_STATUS_RE.match(line)
    if not match:
        return None
    parts = line.split()
    invalid_dip = False
    if "(?)" in parts:
        parts.remove("(?)")
        invalid_dip = True
    if len(parts) < (len(MAG_STATUS_FIELDS) - 1):
        return {
            "type": "mag_status_partial",
            "raw": line,
            "link": int(match.group("link")),
            "status": match.group("status"),
            "token_count": len(parts),
        }
    grad_token = parts[8]
    if "/" in grad_token:
        grad_text, threshold_text = grad_token.split("/", 1)
    else:
        grad_text, threshold_text = grad_token, "0"
    values: list[Any] = [
        int(parts[0]), parts[1], _pct(parts[2]), int(parts[3]), int(parts[4]),
        parts[5], _pct(parts[6]), parts[7], int(grad_text), int(threshold_text),
        int(parts[9]), int(parts[10]),
    ]
    tail = parts[11:]
    for index, token in enumerate(tail):
        field_index = 12 + index
        if field_index >= len(MAG_STATUS_FIELDS):
            break
        if field_index == 12:
            values.append(int(token.rstrip("s")))
        else:
            try:
                values.append(int(token))
            except ValueError:
                values.append(token)
    fields = dict(zip(MAG_STATUS_FIELDS, values))
    fields["dip_valid"] = not invalid_dip
    return {"type": "mag_status_row", "raw": line, "fields": fields}


def parse_firmware_line(line: str) -> dict[str, Any]:
    """Return a structured record for every non-empty printable firmware line."""
    raw = line.rstrip("\r\n")
    text = raw.strip()
    result: dict[str, Any] = {"type": "text", "raw": raw, "text": text}
    if not text:
        return result

    match = SINFO_OK_RE.match(text)
    if match:
        fields: dict[str, Any] = {key: _int(value) for key, value in match.groupdict().items() if key != "sensor_name"}
        fields["sensor_name"] = match.group("sensor_name")
        fields["max_rate_hz"] = fields["max_rate_x100"] / 100.0
        fields["min_rate_hz"] = fields["min_rate_x100"] / 100.0
        return {"type": "sinfo", "raw": raw, "fields": fields}

    match = SINFO_ERR_RE.match(text)
    if match:
        fields = {key: _int(value) for key, value in match.groupdict().items() if key != "sensor_name"}
        fields["sensor_name"] = match.group("sensor_name")
        return {"type": "sinfo_error", "raw": raw, "fields": fields}

    match = GQ_RE.match(text)
    if match:
        fields = {key: _int(value) for key, value in match.groupdict().items()}
        fields.update({
            "quat_x": fields["qx_raw"] / Q_SCALE,
            "quat_y": fields["qy_raw"] / Q_SCALE,
            "quat_z": fields["qz_raw"] / Q_SCALE,
            "quat_w": fields["qw_raw"] / Q_SCALE,
            "yaw_corr_deg": fields["yaw_corr_x100"] / 100.0,
        })
        return {"type": "gq", "raw": raw, "fields": fields}

    match = GY_RE.match(text)
    if match:
        fields = {key: _int(value) for key, value in match.groupdict().items()}
        fields.update({
            "passthrough": (fields["px"], fields["py"], fields["pz"]),
            "corrected": (fields["cx"], fields["cy"], fields["cz"]),
            "bias_x10": (fields["bx_x10"], fields["by_x10"], fields["bz_x10"]),
            "bias": (
                fields["bx_x10"] / 10.0,
                fields["by_x10"] / 10.0,
                fields["bz_x10"] / 10.0,
            ),
            "activity": fields["activity_x10"] / 10.0,
            "corrected_valid_bool": bool(fields["corrected_valid"]),
        })
        return {"type": "gy", "raw": raw, "fields": fields}

    match = MAG_RE.match(text)
    if match:
        fields = {key: _int(value) for key, value in match.groupdict().items()}
        fields.update({
            "raw": (fields["raw_x"], fields["raw_y"], fields["raw_z"]),
            "cal": (
                fields["cal_x_x100"] / 100.0,
                fields["cal_y_x100"] / 100.0,
                fields["cal_z_x100"] / 100.0,
            ),
            "offset": (
                fields["off_x_x100"] / 100.0,
                fields["off_y_x100"] / 100.0,
                fields["off_z_x100"] / 100.0,
            ),
            "datum_deg": fields["datum_deg_x100"] / 100.0,
            "yaw_corr_deg": fields["yaw_corr_deg_x100"] / 100.0,
        })
        return {"type": "mag_detail", "raw": raw, "fields": fields}

    status = _parse_status_row(text)
    if status:
        return status

    if "link" in text and "datum" in text and "yawc" in text:
        return {"type": "mag_status_header", "raw": raw}

    if text.startswith("[MAG]"):
        severity = "info"
        lowered = text.lower()
        if any(word in lowered for word in ("failed", "error", "timeout")):
            severity = "error"
        elif any(word in lowered for word in ("started", "opened", "learning")):
            severity = "event"
        elif any(word in lowered for word in ("stored", "loaded", "restored", "succeeded", "locked", "cleared", "finished")):
            severity = "state"
        return {"type": "mag_event", "raw": raw, "severity": severity, "message": text}

    if text.startswith("boot_from_ram"):
        try:
            value = int(text.split(":", 1)[1].strip())
        except Exception:
            value = None
        return {"type": "boot", "raw": raw, "message": text, "value": value}

    return result


__all__ = ["parse_firmware_line", "MAG_STATUS_FIELDS", "Q_SCALE"]