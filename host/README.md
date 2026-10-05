# Host tools

This directory contains the Windows-side applications and shared protocol tools
for the OSMO/Bowie glove.

## Main applications

| Script | Purpose |
|---|---|
| `3D力测试上位机.pyw` | Dual-magnet differential 3D force display, calibration, plots, CSV |
| `姿态测试上位机.pyw` | 3D attitude, compass/yaw diagnostics, magnetic feature monitoring |
| `纯数据预览.pyw` | Raw MAG / QUAT / META decoder and preview |
| `force_3d_ui_launcher.pyw` | Single-instance launcher for the 3D force application |
| `magcal/magcal.py` | Offline magnetometer collect / robust fit / calibration upload |
| `magcal/magcal_view.pyw` | Live 24-cell spherical coverage monitor while recording |

## Shared modules

- `read_osmo_glove.py` - COBS/protobuf serial protocol decoder
- `replay_protocol.py` - binary replay schema and conversion helpers
- `trace_recorder.py` - structured trace recording
- `serial_trace_reader.py` - traced serial reader
- `firmware_diagnostics.py` - `GQ/GY/MAG/SINFO/P` diagnostics parsing
- `firmware_profile.py` - known firmware build profiles
- `runtime_metrics.py` - host runtime metrics
- `trace_session.py`, `trace_replay.py`, `extract_replay.py` - trace verification and replay
- `replay_firmware.py`, `replay_engine.c`, `build_replay_engine.ps1` - native replay engine
- `diagnose_mag_noise.py`, `profile_pose_ui.py` - diagnostic utilities
- `verify_*.py` - regression and trace checks

## Requirements

```powershell
python -m pip install -r ..\requirements.txt
```

## Run

```powershell
python "3D力测试上位机.pyw"
python "姿态测试上位机.pyw"
python "纯数据预览.pyw"
```

The offline magnetometer calibration CLI is in `magcal/`:

```powershell
python magcal\magcal.py --help
python magcal\test_fit.py
```

Windows launchers are available from the repository root:

```text
run_3d_force.bat
run_attitude.bat
run_raw_preview.bat
run_magcal_view.bat
```

## Documentation

- `docs/使用说明.md` - full Chinese host and firmware operating guide
- `docs/追踪录制说明.md` - trace recording and replay format
- `docs/REPLAY_PROTOCOL.md` - replay protocol details
- `docs/FIX14_信息覆盖清单.md` - diagnostics coverage
- `docs/FIX20_HOST_SYNC.md` - host synchronization notes
- `magcal/README.md` - magnetometer calibration workflow and parameter contract
- `THOST_MANIFEST.md` - how the original `THost` directory was curated into this repository
- `magcal/lab/README.md` - archived calibration experiments and their limitations

## Verification

```powershell
python "3D力测试上位机.pyw" --self-test
python "姿态测试上位机.pyw" --self-test
python verify_trace_recorder.py
python verify_preview_trace.py
python magcal\test_fit.py
```
