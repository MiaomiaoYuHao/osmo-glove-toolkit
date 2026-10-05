# How to reproduce this project

This repository is organized so another person can download the host source,
reconstruct the modified firmware from the pinned upstream commit, build it,
and verify the same output hashes.

## 1. Host application

Install Python 3.12 or newer and the dependencies:

```powershell
python -m pip install -r requirements.txt
```

Run the program:

```powershell
python "host\3D力测试上位机.pyw"
```

Run the built-in verification:

```powershell
python "host\3D力测试上位机.pyw" --self-test
```

Expected output ends with:

```text
FORCE_SELF_TEST_PASS=True
CANVAS_PROJECTION_OK=True
```

The host application uses the local `host/utils/bowiepb` module. There is no
absolute path to another computer's workspace.

## 2. Firmware source

The reproducible firmware target is:

- Upstream: `https://github.com/jessicayin/osmo_tactile_glove.git`
- Base commit: `bfc7328`
- Patch: `firmware/patches/BowieGlove-magnet-stability.patch`
- Modified-file overlay: `firmware/BowieGlove/`

### Automatic bootstrap

From the repository root:

```powershell
.\scripts\bootstrap_firmware.ps1 -Destination .\build\osmo_tactile_glove
```

This performs:

```text
git clone upstream
git checkout bfc7328
git apply firmware/patches/BowieGlove-magnet-stability.patch
```

### Manual bootstrap

```powershell
git clone https://github.com/jessicayin/osmo_tactile_glove.git
cd osmo_tactile_glove
git checkout bfc7328
git apply C:\path\to\osmo-magnet-3d-force\firmware\patches\BowieGlove-magnet-stability.patch
```

The patch contains the same modifications as the complete overlay in
`firmware/BowieGlove/`.

## 3. Build firmware

The repository includes the build helper:

```powershell
.\scripts\build_firmware.ps1 -RepoPath .\build\osmo_tactile_glove
```

On the machine used to prepare this release, the build used:

- STM32CubeIDE `1.14.1`
- GNU Arm toolchain `11.3.rel1`
- GNU make from the STM32CubeIDE installation
- Target: STM32F446RET6

The helper accepts custom toolchain locations:

```powershell
.\scripts\build_firmware.ps1 `
  -RepoPath .\build\osmo_tactile_glove `
  -GccBin "C:\path\to\arm-none-eabi\bin" `
  -MakeExe "C:\path\to\make.exe"
```

Expected local build result:

```text
text   225524
data     1160
bss     30888
```

Expected generated files:

```text
build\osmo_tactile_glove\firmware\BowieGlove\Debug\BowieGlove.hex
build\osmo_tactile_glove\firmware\BowieGlove\Debug\BowieGlove.bin
```

## 4. Flash the prebuilt firmware

Ready-to-flash release files are included:

```text
firmware\releases\BowieGlove_magnet_recovery.hex
firmware\releases\BowieGlove_magnet_recovery.bin
firmware\releases\SHA256SUMS.txt
```

Example STM32CubeProgrammer command:

```powershell
STM32_Programmer_CLI.exe `
  -c port=SWD freq=1000 mode=HOTPLUG `
  -w "firmware\releases\BowieGlove_magnet_recovery.bin" 0x08000000 `
  -v -rst
```

Verify the files before flashing:

```powershell
Get-FileHash .\firmware\releases\* -Algorithm SHA256
Get-Content .\firmware\releases\SHA256SUMS.txt
```

## 5. Hardware reproduction checklist

Required hardware:

- STM32F446RET6 board running the BowieGlove firmware
- ST-Link debug probe
- Two BHI360 magnetometer channels
- OSMO magnetic skin
- Windows PC with the Bowie CDC USB device `2833:B015`

Test sequence:

1. Flash `BowieGlove_magnet_recovery.hex` or `BowieGlove_magnet_recovery.bin`.
2. Start the host application.
3. Verify `frames` increases continuously.
4. Capture the dual-magnet zero with `Z`.
5. Apply a strong magnet or move a magnet rapidly near the sensor.
6. Confirm the host `frames` counter keeps increasing.
7. Confirm the 3D vector and three-axis graphs continue updating.
8. Remove the magnet and confirm normal force values return.

## 6. Known verification boundary

The host-side self-test and the firmware compilation have been verified. The
strong-magnet hardware regression must still be repeated on the physical
glove. Do not claim that hardware-in-the-loop validation passed until that
test is recorded.

## 7. Licensing boundary

The upstream OSMO repository currently has no LICENSE file. The patch,
generated `bowiepb` module, and any upstream-derived code are not relicensed by
this repository. The MIT license applies only to original host-side material
authored by `lina130`. See `NOTICE.md`.

## 8. One-command reproduction

On Windows, after installing Python and STM32CubeIDE, run from the repository root:

```powershell
.scriptseproduce_all.ps1
```

This script:

1. runs the host self-test;
2. clones the upstream firmware from GitHub;
3. checks out commit `bfc7328`;
4. applies the firmware patch;
5. fixes the upstream makefiles that contain the original authors' absolute paths;
6. performs a clean firmware build;
7. compares the rebuilt HEX/BIN SHA256 values with `firmware/releases/SHA256SUMS.txt`.

The expected final line is:

```text
REPRODUCTION_PASS=True
```

This exact flow was verified on 2026-10-05 against the public upstream repository.
The rebuilt files were byte-identical to the release files.
