# Custom 9-DoF attitude firmware

Source overlay: `BowieGlove_Attitude/`

This is the custom 9-DoF attitude / magnetic-yaw variant. The source tree is
included without Debug, Release, object files, logs, backups, or old binaries.

## Main functionality

- BHI360 GAMERV supplies roll/pitch/relative yaw.
- BMM350 magnetometer data provides magnetic yaw correction.
- Dual-magnet gradient and direction gates suppress magnetic disturbances.
- Online hard/soft-iron calibration.
- Magnetic bias, rebase, rollback, and recovery logic.
- Runtime direction-gate tuning through the host application.
- Replay diagnostics for calibration and fusion regression testing.

## Build inputs

- STM32F446RETX
- STM32CubeIDE project files
- `Core/`, `Drivers/`, `tinyusb/`
- `BowieGlove.ioc`, linker scripts, and CMake helper files

The custom attitude source was prepared for local STM32CubeIDE / GNU Arm
workflows. Restore toolchain paths after cloning for a build on another PC.

## Release

The latest prebuilt release is:

`../releases/attitude_9dof/BowieGlove_Attitude_FINALV1.hex`

The checksum file is in the same directory.

## License

This firmware is derived from the upstream OSMO/Bowie project and includes
Bosch firmware headers. It is not relicensed by the toolkit MIT license.
See `../../NOTICE.md`.
