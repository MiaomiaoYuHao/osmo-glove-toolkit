# Firmware variants

This directory contains the source and release material for the glove firmware
variants.

## Directory map

```text
force/BowieGlove/                    force and multi-magnet firmware source overlay
force/patches/                       patch against upstream bfc7328
attitude/BowieGlove_Attitude/        custom 9-DoF / yaw-supervisor source
ndof/                                official NDOF variant notes and vendor header
releases/force/                      force firmware HEX/BIN + SHA256
releases/attitude_9dof/              custom 9-DoF release
releases/attitude_ndof/              official NDOF release
releases/complete_6dof/              full compatible 6-DoF release
```

## Force firmware

Base: upstream `jessicayin/osmo_tactile_glove` commit `bfc7328`.

Apply:

```powershell
git clone https://github.com/jessicayin/osmo_tactile_glove.git
cd osmo_tactile_glove
git checkout bfc7328
git apply C:\path\to\firmware\force\patches\BowieGlove-magnet-stability.patch
```

Build:

```powershell
.\scripts\bootstrap_firmware.ps1
.\scripts\build_firmware.ps1
```

## Custom 9-DoF attitude firmware

Source: `firmware/attitude/BowieGlove_Attitude`.

Key features:

- GAMERV 6-axis attitude plus magnetic-yaw correction
- two BMM350 magnetometers
- online hard/soft-iron calibration
- magnetic gradient gate and disturbance recovery
- runtime direction-gate tuning
- calibration persistence and rollback/recovery logic
- host diagnostics and replay support

Prebuilt final release: `firmware/releases/attitude_9dof`.

## Official Bosch NDOF variant

The local source directory referenced by the original notes
`firmware/BowieGlove_Attitude_NDOF` was not present. The vendor firmware header
and prebuilt NDOF image are included as release artifacts. If the missing source
tree is recovered later, add it under `firmware/ndof/BowieGlove_Attitude_NDOF`
and update the build notes.

## Full compatible 6-DoF variant

The full compatible release uses the force firmware source layout and is
provided as `firmware/releases/complete_6dof`.

## License

The upstream repository has no license file. These firmware sources, patches,
and vendor headers are not relicensed by the toolkit MIT license. See
`../NOTICE.md`.