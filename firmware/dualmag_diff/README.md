# Source overlay — dual-magnet differential build

Base: `backup_before_magnet_recovery_20260925` (the working fixall sources).

Files here are the complete modified copies. Two build notes that are NOT in these
sources but are required:

1. **Include-path fix.** The STM32CubeIDE makefiles under `Debug/**/*.mk` pointed at an
   absolute upstream path (`.../_osmo_glove_research/firmware/BowieGlove/Core/Inc/...`),
   which made `bhi360.c` compile against a DIFFERENT header revision than `glove.c`.
   The two translation units then disagreed on `struct mag_data_dev` layout -> memory
   corruption -> "works then dies / re-init loop". Repoint those `-I` paths at the local
   tree (`../Core/Inc/glove`, `../Core/Inc/usb`, `../Core/Inc/comms`,
   `../Core/Inc/comms/protobuf`, `../tinyusb/src`).
2. **`objects.list`** must contain `./Core/Src/glove/bhi360.o` entries consistent with
   `subdir.mk` (the linker consumes `objects.list`, not `OBJS`).
