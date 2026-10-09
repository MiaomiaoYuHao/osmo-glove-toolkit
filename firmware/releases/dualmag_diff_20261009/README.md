# Dual-magnet differential build (2026-10-09)

Built from the **known-good fixall base** (`backup_before_magnet_recovery_20260925`
sources) instead of the magnet-recovery line, which proved unstable on this board
(USB drop / re-init loops).

## What it is

The firmware itself is a **plain data pipe** for both magnetometers of one island:

| frame | sensor_id | content |
|---|---|---|
| quaternion | 13 | BSX Game Rotation Vector |
| primary magnetometer (0x14) | 13 | BSX `MAG` channel |
| secondary magnetometer (0x15) | **33** (= 13 + 20) | BSX `MAG_PASS_META` channel |

* The second BMM350 is read through the BHI360 BSX **META** channel (`BHY2_SENSOR_ID_MAG_PASS_META`),
  so **no SensorAPI / Soft Pass-Through** is used at runtime.
* The **differential is computed on the host**, not in the firmware
  (`3D力测试上位机.pyw` pairs `sid` with `sid + 20`).

## Host requirements

Use the host files in this release's repo revision:

* `host/read_osmo_glove.py` — exposes `mag_seconds` / `mag_nanoseconds` per frame
* `host/3D力测试上位机.pyw` — timestamp-aligned pairing, Kalman bias drift
  compensation, numeric grid + 3D surface heat map

## Measured

```
rate        ~3.8 kB/s
quat  (13)  ~50 Hz
mag   (33)  ~25 Hz     (secondary / META channel)
```

## Flash

```
STM32_Programmer_CLI.exe -c port=SWD freq=1000 mode=UR \
  -w BowieGlove_dualmag_diff.bin 0x08000000 -v -rst
```

**Cold boot required**: power-cycle the board after flashing (the BHI360 keeps state
across a warm reset and will not stream otherwise).

## Warning

With a single taxel connected, the heat map is a **synthesised field** (a rendering of
one vector), not a measured 2D distribution.
