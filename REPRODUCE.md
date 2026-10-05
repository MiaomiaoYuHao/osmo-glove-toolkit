# Reproduction guide

This repository contains multiple firmware variants and Windows host tools.
The exact end-to-end reproducibility guarantee currently applies to the force
firmware because that build has a pinned upstream commit and a verified patch.

## 1. Host tools

Install dependencies:

~~~powershell
python -m pip install -r requirements.txt
~~~

Run self-tests:

~~~powershell
python "host\3D力测试上位机.pyw" --self-test
python "host\姿态测试上位机.pyw" --self-test
python "host\verify_trace_recorder.py"
python "host\verify_preview_trace.py"
~~~

The 3D force self-test must end with:

~~~text
FORCE_SELF_TEST_PASS=True
CANVAS_PROJECTION_OK=True
~~~

The attitude self-test must end with:

~~~text
POSE_SELF_TEST_PASS=True
YAW_MAPS_TO_SCREEN_VERTICAL_AXIS=True
~~~

## 2. Firmware variants

| Variant | Source | Release |
|---|---|---|
| Force / multi-magnet | firmware/force/BowieGlove | firmware/releases/force |
| Custom 9-DoF attitude | firmware/attitude/BowieGlove_Attitude | firmware/releases/attitude_9dof |
| Official NDOF single-magnet yaw | firmware/ndof | firmware/releases/attitude_ndof |
| Full compatible 6-DoF GAMERV | firmware/force/BowieGlove | firmware/releases/complete_6dof |

## 3. Force firmware reproduction

Pinned upstream commit: bfc7328

Run from the repository root:

~~~powershell
.\scripts\reproduce_all.ps1
~~~

The script:

1. runs the host self-test;
2. clones the upstream repository;
3. checks out bfc7328;
4. applies firmware/force/patches/BowieGlove-magnet-stability.patch;
5. repairs absolute build paths in the upstream Debug makefiles;
6. performs a clean build;
7. compares the rebuilt HEX/BIN with firmware/releases/force/SHA256SUMS.txt.

Verified output:

~~~text
REPRODUCTION_PASS=True
BIN 3476FCAC78B1C88F6B4AB33D474C0181AB600D00F02D6282B0F167B595F58DB9
HEX 89D6359A8F34E61A574F088E0C79FA7FE12AD9B68B232D6B33F8CB3C1C7841AB
~~~

## 4. Custom 9-DoF attitude firmware

Source: firmware/attitude/BowieGlove_Attitude

The source tree was copied without Debug, Release, object files, logs, backups,
or old binaries. It includes Core, Drivers, tinyusb, STM32CubeIDE project files,
linker scripts, and CMake helper files.

Prebuilt release:

~~~text
firmware\releases\attitude_9dof\BowieGlove_Attitude_FINALV1.hex
firmware\releases\attitude_9dof\BowieGlove_Attitude_FINALV1.bin
firmware\releases\attitude_9dof\SHA256SUMS.txt
~~~

Build on another machine by opening the STM32CubeIDE project or adapting the
included CMake and GNU Arm configuration to that machine. The original local
build logs and absolute toolchain paths were intentionally excluded.

## 5. Official NDOF variant

The original local note referenced a directory named
firmware/BowieGlove_Attitude_NDOF, but that directory was not present in the
workspace. The toolkit includes:

- firmware/ndof/vendor/Bosch_Shuttle3_BHI360_BMM350_Poll_bsxsam_ndof.fw.h
- firmware/ndof/官方NDOF说明.txt
- firmware/releases/attitude_ndof/BowieGlove_Attitude_NDOF.hex
- firmware/releases/attitude_ndof/BowieGlove_Attitude_NDOF.bin
- firmware/releases/attitude_ndof/SHA256SUMS.txt

Exact source-level reproduction for this variant is pending until that missing
source tree is restored.

## 6. Flash a release

Example:

~~~powershell
STM32_Programmer_CLI.exe `
  -c port=SWD freq=1000 mode=HOTPLUG `
  -w "firmware\releases\force\BowieGlove_magnet_recovery.bin" 0x08000000 `
  -v -rst
~~~

Verify checksums before flashing:

~~~powershell
Get-Content .\firmware\releases\force\SHA256SUMS.txt
Get-Content .\firmware\releases\attitude_9dof\SHA256SUMS.txt
Get-Content .\firmware\releases\attitude_ndof\SHA256SUMS.txt
Get-Content .\firmware\releases\complete_6dof\SHA256SUMS.txt
~~~

## 7. Verification boundary

The force firmware has been rebuilt from a fresh upstream clone and matched
byte for byte. The custom attitude source and all release images are included,
but a clean cross-machine rebuild for the attitude and NDOF variants still
requires the missing source/toolchain details to be restored.

## 8. License boundary

The upstream OSMO repository has no LICENSE file. The patches, generated
protocol modules, vendor firmware headers, and upstream-derived firmware are
not relicensed by the toolkit MIT license. See NOTICE.md.
