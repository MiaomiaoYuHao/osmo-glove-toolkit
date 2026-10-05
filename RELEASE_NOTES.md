# Release 0.1.0 - Magnet stability and 3D force host

## Contents

- Complete Python source for the OSMO/Bowie 3D force application.
- Hard/soft-iron ellipsoid calibration.
- Repeated-rubbing six-direction calibration.
- XY, XZ, YZ grids and vector projections.
- Serial reconnect and connection watchdog.
- BowieGlove firmware patch for magnetic-transient recovery.
- Prebuilt HEX and BIN firmware images.
- Bootstrap, build, flash, and full reproduction scripts.

## Firmware target

- Upstream repository: `jessicayin/osmo_tactile_glove`
- Base commit: `bfc7328`
- MCU: STM32F446RET6
- Toolchain used: GNU Arm `11.3.rel1`
- Build result: `text 225524`, `data 1160`, `bss 30888`

## Verification

The repository was tested by cloning the upstream repository from GitHub,
checking out `bfc7328`, applying
`firmware/patches/BowieGlove-magnet-stability.patch`, rebuilding from a clean
`Debug` directory, and comparing the output hashes.

```text
REPRODUCTION_PASS=True
3476FCAC78B1C88F6B4AB33D474C0181AB600D00F02D6282B0F167B595F58DB9  BowieGlove.bin
89D6359A8F34E61A574F088E0C79FA7FE12AD9B68B232D6B33F8CB3C1C7841AB  BowieGlove.hex
```

## Hardware status

Host-side tests pass and the firmware builds reproducibly. The strong-magnet
hardware regression still needs to be recorded on the physical glove.

## License

MIT applies only to original host-side material authored by `lina130`.
Upstream-derived files are excluded. See `LICENSE` and `NOTICE.md`.
