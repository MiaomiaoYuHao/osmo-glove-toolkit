#!/usr/bin/env python3
from __future__ import annotations
import statistics
import time
from collections import defaultdict
import serial
import read_osmo_glove as glove

port = glove.find_glove_port()
ser = serial.Serial(port, baudrate=115200, timeout=0.15)
buffer = bytearray()
samples = defaultdict(list)
latest = {}
diff_rows = []
start = time.monotonic()
try:
    while time.monotonic() - start < 8.0:
        chunk = ser.read(1)
        if not chunk:
            continue
        buffer.extend(chunk)
        while True:
            index = buffer.find(b"\x00")
            if index < 0:
                break
            packet = bytes(buffer[:index])
            del buffer[:index + 1]
            msg, error, _payload = glove.decode_packet_with_payload(packet)
            if msg is None:
                continue
            row = glove.message_to_row(msg)
            sid = row.get("sensor_id")
            if sid not in (13, 33) or row.get("mag_x") is None:
                continue
            value = (float(row["mag_x"]), float(row["mag_y"]), float(row["mag_z"]))
            samples[sid].append(value)
            latest[sid] = value
            if 13 in latest and 33 in latest:
                diff_rows.append(tuple(latest[13][axis] - latest[33][axis] for axis in range(3)))
finally:
    ser.close()

print("PORT", port)
for sid in (13, 33):
    rows = samples[sid]
    print("SENSOR", sid, "N", len(rows), "HZ", round(len(rows) / 8.0, 2))
    for axis, name in enumerate("xyz"):
        vals = [row[axis] for row in rows]
        print(" ", name, "mean", round(statistics.mean(vals), 3), "std", round(statistics.pstdev(vals), 3), "min", min(vals), "max", max(vals), "p2p", max(vals) - min(vals))
print("DIFF N", len(diff_rows))
for axis, name in enumerate("xyz"):
    vals = [row[axis] for row in diff_rows]
    print(" ", name, "mean", round(statistics.mean(vals), 3), "std", round(statistics.pstdev(vals), 3), "min", min(vals), "max", max(vals), "p2p", max(vals) - min(vals))
