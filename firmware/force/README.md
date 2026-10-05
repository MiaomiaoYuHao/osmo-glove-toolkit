# BowieGlove firmware patch

This directory contains both a complete modified-source overlay and a patch
against the upstream OSMO/Bowie firmware.

- Upstream: `https://github.com/jessicayin/osmo_tactile_glove.git`
- Base commit: `bfc7328`
- Patch: `patches/BowieGlove-magnet-stability.patch`
- Overlay: `BowieGlove/`
- Prebuilt release: `../releases/force/BowieGlove_magnet_recovery.hex` and `.bin`

## Apply the patch

```powershell
git clone https://github.com/jessicayin/osmo_tactile_glove.git
cd osmo_tactile_glove
git checkout bfc7328
git apply C:\path\to\BowieGlove-magnet-stability.patch
```

Or run from the project root:

```powershell
.\scripts\bootstrap_firmware.ps1 -Destination .\build\osmo_tactile_glove
```

## Main stability changes

- Recover BHI360 after `SENSOR_ERROR`, `FIFO_OVERFLOW`, and `RESET`.
- Prevent zero-progress loops in the FIFO parser.
- Preserve parser errors instead of overwriting them with a success result.
- Retry a failed BHI360 reinitialization after a 2-second cooldown.
- Retry I2C reads and recover the bus after repeated errors.
- Check CDC TX space before writing a frame.
- Maintain TinyUSB servicing from the main loop.
- Enable an independent watchdog.
- Increase the linker stack and move the large work buffer to static memory.

## Build

Use STM32CubeIDE, or run:

```powershell
.\scripts\build_firmware.ps1 -RepoPath .\build\osmo_tactile_glove
```

Expected Debug size:

```text
text   225524
data     1160
bss     30888
```

## Flash

```powershell
STM32_Programmer_CLI.exe `
  -c port=SWD freq=1000 mode=HOTPLUG `
  -w ..\releases\force\BowieGlove_magnet_recovery.bin 0x08000000 `
  -v -rst
```

The upstream repository has no LICENSE file. The patch and overlay are not
relicensed by the new MIT license in this repository. See `../../NOTICE.md`.