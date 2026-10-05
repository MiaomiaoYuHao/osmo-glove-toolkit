# Notice and attribution

This project contains original host-side code by `lina130` and a compatibility
patch for the upstream OSMO tactile glove project.

Upstream project:

- https://github.com/jessicayin/osmo_tactile_glove
- OSMO: Open-Source Tactile Glove for Human-to-Robot Skill Transfer
- Authors include Jessica Yin, Haozhi Qi, Youngsun Wi, Sayantan Kundu,
  Mike Lambeta, William Yang, Changhao Wang, Tingfan Wu, Jitendra Malik,
  and Tess Hellebrekers.

At the time this repository was prepared, the upstream repository did not
include a LICENSE file. A public repository is not automatically an open-source
license. Therefore:

- The MIT license in this repository applies only to original material
  authored by `lina130`.
- `host/utils/bowiepb/` is a generated protobuf module derived from the
  upstream OSMO/Bowie protocol. It is included only for interoperability and
  is not relicensed here.
- `firmware/patches/BowieGlove-magnet-stability.patch` is a diff against the
  upstream firmware and is not relicensed here.

If you plan to redistribute or relicense upstream-derived material, request
permission or a proper license from the upstream authors.

OSMO citation:

```bibtex
@article{yin2025osmo,
  title={OSMO: Open-Source Tactile Glove for Human-to-Robot Skill Transfer},
  author={Jessica Yin and Haozhi Qi and Youngsun Wi and Sayantan Kundu and Mike Lambeta and William Yang and Changhao Wang and Tingfan Wu and Jitendra Malik and Tess Hellebrekers},
  journal={arXiv:2512.08920},
  year={2025}
}
```