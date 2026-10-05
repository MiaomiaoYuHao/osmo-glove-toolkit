# Firmware release images

Each variant has its own HEX/BIN files and SHA256SUMS.txt.

| Directory | Variant |
|---|---|
| force/ | 3D force / multi-magnet stability firmware |
| attitude_9dof/ | Custom 9-DoF attitude / yaw supervisor |
| attitude_ndof/ | Official Bosch NDOF single-magnet yaw |
| complete_6dof/ | Full compatible 6-DoF GAMERV |

Flash a BIN image at 0x08000000 using STM32CubeProgrammer. Verify SHA256 before flashing.
