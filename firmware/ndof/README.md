# Official Bosch NDOF single-magnet yaw variant

This directory documents the official Bosch NDOF variant.

Included:

- `vendor/Bosch_Shuttle3_BHI360_BMM350_Poll_bsxsam_ndof.fw.h`
- `官方NDOF说明.txt`
- prebuilt HEX/BIN under `../releases/attitude_ndof/`

The original local note pointed to a source directory named
`firmware/BowieGlove_Attitude_NDOF`, but that directory was not present in the
workspace when this toolkit was assembled. The prebuilt image is therefore
published, while exact source reconstruction for this variant is pending until
that source tree is restored.

Variant behavior:

- BHI360 supplies 6-axis IMU and Bosch BSX NDOF fusion.
- BMM350 #1 is the external magnetometer used by official NDOF for yaw.
- BMM350 #2 is not part of the native NDOF fusion in this variant.

## License

Bosch firmware headers and the upstream OSMO/Bowie source are not relicensed by
the toolkit MIT license. See `../../NOTICE.md`.
