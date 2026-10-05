#!/usr/bin/env python3
"""Parser checks for FIX14 serial diagnostics."""

from __future__ import annotations

from firmware_diagnostics import MAG_STATUS_FIELDS, parse_firmware_line


def main() -> int:
    gq = parse_firmware_line("GQ f4 l1 id13 acc0 raw 3551 -4557 15295 1053 yawc 2842")
    assert gq["type"] == "gq", gq
    assert gq["fields"]["sensor_id"] == 13, gq
    assert abs(gq["fields"]["quat_w"] - 1053 / 16384.0) < 1e-12, gq
    assert gq["fields"]["yaw_corr_deg"] == 28.42, gq

    gy = parse_firmware_line("GY f4 l1 P -28 -31 13 C 0 0 0 CV0 B -27 -106 45 ACT 268 MH 12")
    assert gy["type"] == "gy", gy
    assert gy["fields"]["corrected_valid"] == 0, gy
    assert gy["fields"]["bias"] == (-2.7, -10.6, 4.5), gy
    assert gy["fields"]["activity"] == 26.8, gy
    assert gy["fields"]["motion_hold"] == 12, gy

    mag = parse_firmware_line(
        "MAG f4 l1 RAW -100 20 30 | CAL -2500 500 750 | OFF -1000 100 200 "
        "| REV 3 FIX 4 OPENS 2 LRN 1 | DATUM 1234 YAWC -852"
    )
    assert mag["type"] == "mag_detail", mag
    assert mag["fields"]["cal"] == (-25.0, 5.0, 7.5), mag
    assert mag["fields"]["datum_deg"] == 12.34, mag
    assert mag["fields"]["yaw_corr_deg"] == -8.52, mag

    sinfo = parse_firmware_line("SINFO l1 id13 GameRotation drv3 range4 res16 maxr10000 minr100")
    assert sinfo["type"] == "sinfo", sinfo
    assert sinfo["fields"]["max_rate_hz"] == 100.0, sinfo
    assert sinfo["fields"]["min_rate_hz"] == 1.0, sinfo

    event = parse_firmware_line("[MAG] calibration stored (1 link(s))")
    assert event["type"] == "mag_event", event
    assert event["severity"] == "state", event

    status = parse_firmware_line(
        "1 LOCKED 95% 100 101 full 2% yes 12/30 5 45 0s 3 20 1000 10 5 4 3 2 1 0 0 0 0 0 0 95 0 0 1 100 500 2 45 -3 4 1 7"
    )
    assert status["type"] == "mag_status_row", status
    fields = status["fields"]
    assert fields["status"] == "LOCKED", status
    assert fields["trust_pct"] == 0.95, status
    assert fields["grad"] == 12 and fields["grad_threshold"] == 30, status
    assert fields["datum_deg"] == 45 and fields["yaw_corr_deg"] == -3, status
    assert fields["learning_opens"] == 7, status

    generic = parse_firmware_line("some other diagnostic text")
    assert generic["type"] == "text", generic
    assert generic["text"] == "some other diagnostic text", generic
    assert len(MAG_STATUS_FIELDS) == 40, len(MAG_STATUS_FIELDS)
    print("FIRMWARE_DIAGNOSTICS_PARSER_PASS=True")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())