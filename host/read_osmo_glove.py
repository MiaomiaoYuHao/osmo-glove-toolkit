#!/usr/bin/env python3
"""Read OSMO/Bowie glove protobuf frames from a Windows COM port.

The firmware sends COBS-framed protobuf messages over USB CDC. This script
auto-detects the glove by VID/PID 2833:B015 and prints or records decoded data.

Examples:
    python read_osmo_glove.py
    python read_osmo_glove.py --port COM3
    python read_osmo_glove.py --csv osmo_data.csv
    python read_osmo_glove.py --duration 10
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path
from typing import Any

import serial
from cobs import cobs
from serial.tools import list_ports

# Keep the generated betterproto module local so the host application is
# self-contained and does not depend on an absolute workspace path.
HOST_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(HOST_ROOT))

from utils import bowiepb as bpb  # noqa: E402


FINGER_NAMES = {
    0: "none",
    1: "pinky",
    2: "ring",
    3: "middle",
    4: "index",
    5: "thumb",
}


def find_glove_port() -> str:
    """Find the glove CDC serial port by USB VID/PID."""
    matches = [
        port
        for port in list_ports.comports()
        if port.vid == 0x2833 and port.pid == 0xB015
    ]
    if matches:
        return matches[0].device

    # Fallback for drivers that do not expose VID/PID correctly.
    for port in list_ports.comports():
        desc = f"{port.description or ''} {port.device or ''}".lower()
        if "bowie" in desc or "glove" in desc:
            return port.device

    raise RuntimeError(
        "OSMO/Bowie glove not found. Expected USB VID:PID 2833:B015. "
        "Connect the glove by USB-C, wait for initialization, and check Device Manager."
    )


def decode_packet_with_payload(packet: bytes) -> tuple[Any | None, str | None, bytes | None]:
    """Return ``(message, error, protobuf_payload)`` for one COBS frame."""
    if not packet:
        return None, "empty_packet", None
    try:
        payload = cobs.decode(packet)
    except cobs.DecodeError as exc:
        return None, f"cobs_decode_error: {exc}", None
    except Exception as exc:
        return None, f"cobs_exception: {type(exc).__name__}: {exc}", None

    message = bpb.Data()
    try:
        message.parse(payload)
    except Exception as exc:
        return None, f"protobuf_parse_error: {type(exc).__name__}: {exc}", payload
    return message, None, payload


def decode_packet_detailed(packet: bytes) -> tuple[Any | None, str | None]:
    """Decode one COBS frame and return ``(message, error)``."""
    message, error, _payload = decode_packet_with_payload(packet)
    return message, error


def decode_packet(packet: bytes) -> Any | None:
    """Decode one COBS frame, returning None for malformed frames."""
    message, _error = decode_packet_detailed(packet)
    return message


def value(data: dict[str, Any], snake: str, camel: str | None = None) -> Any:
    """Return either snake_case or camelCase key from betterproto to_dict()."""
    if camel is not None and camel in data:
        return data[camel]
    return data.get(snake)


def message_to_row(message: Any) -> dict[str, Any]:
    data = message.to_dict()
    mag = data.get("mag") or {}
    quat = data.get("quat") or {}
    has_mag = data.get("mag") is not None
    has_quat = data.get("quat") is not None
    raw_finger = data.get("finger", 0)
    if hasattr(raw_finger, "value"):
        finger = int(raw_finger.value)
    elif isinstance(raw_finger, str):
        finger = {name: value for value, name in FINGER_NAMES.items()}.get(raw_finger.lower(), 0)
    else:
        try:
            finger = int(raw_finger)
        except (TypeError, ValueError):
            finger = 0

    return {
        "host_time": time.time(),
        "index": data.get("index", 0),
        "finger": FINGER_NAMES.get(finger, f"unknown({finger})"),
        "finger_id": finger,
        "link": data.get("link", 0),
        "sensor_id": value(data, "sensor_id", "sensorId"),
        "mag_x": mag.get("x", 0.0) if has_mag else None,
        "mag_y": mag.get("y", 0.0) if has_mag else None,
        "mag_z": mag.get("z", 0.0) if has_mag else None,
        "quat_x": quat.get("x", 0.0) if has_quat else None,
        "quat_y": quat.get("y", 0.0) if has_quat else None,
        "quat_z": quat.get("z", 0.0) if has_quat else None,
        "quat_w": quat.get("w", 0.0) if has_quat else None,
        "quat_accuracy": quat.get("accuracy", 0.0) if has_quat else None,
        "packet": data,
    }


def open_serial(port: str) -> serial.Serial:
    ser = serial.Serial(port, baudrate=115200, timeout=0.2)
    ser.reset_input_buffer()
    return ser


def main() -> int:
    parser = argparse.ArgumentParser(description="Read OSMO/Bowie glove data.")
    parser.add_argument("--port", help="Serial port, for example COM3. Auto-detected when omitted.")
    parser.add_argument("--csv", help="Write decoded rows to this CSV file.")
    parser.add_argument("--duration", type=float, help="Stop after N seconds. Runs forever when omitted.")
    parser.add_argument("--raw", action="store_true", help="Print the full decoded protobuf dictionary.")
    args = parser.parse_args()

    try:
        port = args.port or find_glove_port()
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(f"Opening OSMO glove on {port} ...")
    try:
        ser = open_serial(port)
    except serial.SerialException as exc:
        print(f"ERROR: Cannot open {port}: {exc}", file=sys.stderr)
        return 1

    csv_file = None
    writer = None
    fieldnames = [
        "host_time",
        "index",
        "finger",
        "finger_id",
        "link",
        "sensor_id",
        "mag_x",
        "mag_y",
        "mag_z",
        "quat_x",
        "quat_y",
        "quat_z",
        "quat_w",
        "quat_accuracy",
    ]
    if args.csv:
        csv_file = open(args.csv, "w", newline="", encoding="utf-8")
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        print(f"Recording CSV: {Path(args.csv).resolve()}")

    started = time.monotonic()
    buffer = bytearray()
    frame_count = 0

    print("Streaming. Press Ctrl+C to stop.")
    try:
        while True:
            if args.duration is not None and time.monotonic() - started >= args.duration:
                break

            waiting = ser.in_waiting
            chunk = ser.read(waiting if waiting > 0 else 1)
            if not chunk:
                continue
            buffer.extend(chunk)

            while True:
                delimiter = buffer.find(b"\x00")
                if delimiter < 0:
                    break
                packet = bytes(buffer[:delimiter])
                del buffer[: delimiter + 1]
                message = decode_packet(packet)
                if message is None:
                    continue

                row = message_to_row(message)
                frame_count += 1

                if args.raw:
                    print(row["packet"])
                else:
                    print(
                        f"{row['index']:>7} "
                        f"{row['finger']:<6} link={row['link']} "
                        f"sensor_id={row['sensor_id']} "
                        f"mag=({row['mag_x']}, {row['mag_y']}, {row['mag_z']}) "
                        f"quat=({row['quat_x']}, {row['quat_y']}, {row['quat_z']}, {row['quat_w']})"
                    )

                if writer is not None:
                    writer.writerow(row)
                    csv_file.flush()
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.close()
        if csv_file is not None:
            csv_file.close()

    print(f"Decoded frames: {frame_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
