# Release 0.2.0 - OSMO Glove Toolkit

## Contents

- Force / multi-magnet firmware source, patch, and prebuilt images.
- Custom 9-DoF attitude firmware source and FINALV1 images.
- Official Bosch NDOF single-magnet yaw images and vendor header.
- Full compatible 6-DoF GAMERV images.
- 3D force host, attitude host, raw preview, trace, diagnostics, and replay tools.
- SHA256 manifests for every firmware release directory.

## Verified

The force firmware reproduction was run from a fresh upstream clone at
commit bfc7328 and matched the release byte for byte:

~~~text
REPRODUCTION_PASS=True
BIN 3476FCAC78B1C88F6B4AB33D474C0181AB600D00F02D6282B0F167B595F58DB9
HEX 89D6359A8F34E61A574F088E0C79FA7FE12AD9B68B232D6B33F8CB3C1C7841AB
~~~

The custom attitude and NDOF sources/releases are included, but the missing
NDOF source directory and cross-machine attitude rebuild are still pending.

## Hardware status

The strong-magnet hardware-in-the-loop regression still needs to be recorded on
the physical glove. The code and downloadable release assets are complete.

## License

MIT applies only to original host-side material authored by lina130.
Upstream-derived and vendor firmware material is excluded. See LICENSE and
NOTICE.md.
