#!/usr/bin/env python3
"""Verify that the host's latest-known firmware profile matches the frozen image."""

from __future__ import annotations

import hashlib
from pathlib import Path

from firmware_profile import LATEST_KNOWN_FIRMWARE

APP = Path(__file__).resolve().parent
WORKSPACE = APP.parent


def main() -> int:
    profile = LATEST_KNOWN_FIRMWARE
    frozen = WORKSPACE / str(profile["frozen_dir"])
    assert frozen.is_dir(), frozen
    hexes = sorted(frozen.glob("*.hex"))
    assert len(hexes) == 1, hexes
    hex_path = hexes[0]
    actual = hashlib.sha256(hex_path.read_bytes()).hexdigest().upper()
    expected = str(profile["hex_sha256"]).upper()
    assert actual == expected, (actual, expected)
    assert profile["bhi_diag_print"] is False, profile
    assert "corrected" in str(profile["gyro_source"]).lower(), profile
    print("FIRMWARE_PROFILE_PASS=True")
    print("FIRMWARE_NAME=", profile["name"])
    print("FIRMWARE_HEX_SHA256=", actual)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())