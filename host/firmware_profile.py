#!/usr/bin/env python3
"""Known firmware profile metadata copied from frozen build manifests."""

from __future__ import annotations

LATEST_KNOWN_FIRMWARE = {
    "name": "FIX46",
    "frozen_dir": r"THost\firmware\九轴姿态_9DoF_RV\FROZEN_FIX46_20260923",
    "hex_sha256": "1262C83E2C301745655B2359BFBE073E2A69E98BEFE0437B04B3EE4F3D679438",
    "bhi_diag_print": False,
    "mag_diag_print": True,
    "gyro_source": "BHY2_SENSOR_ID_GYRO corrected (id 13)",
    "protocol": "data-only sensor IDs 240..253; replay_protocol.py schema v2",
    "notes": (
        "yaw correction = 1-D Kalman (K = P/(P+R)). "
        "large-correction freeze disabled (MAG_YAW_HOLD_S = 0). "
        "direction-residual gate enabled with runtime-adjustable strength, "
        "set from the host slider via 'T DIR <0..100>'. "
        "The host parses live YF/YAW/YAW2/P/CONFIG data and evaluates every "
        "magnetic correction gate against the FIX30 limits."
    ),
}



def latest_known_firmware() -> dict[str, object]:
    return dict(LATEST_KNOWN_FIRMWARE)


__all__ = ["LATEST_KNOWN_FIRMWARE", "latest_known_firmware"]















