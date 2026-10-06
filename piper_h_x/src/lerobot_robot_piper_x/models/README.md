# Kinematic model provenance

The URDFs and LICENSE are copied unmodified from
https://github.com/agilexrobotics/agx_arm_urdf at revision
`f6642ce0d7872c686f29c99e9e10cd23d1d49313`:

- `piper_h/urdf/piper_h_description.urdf` -> `piper_h.urdf`
- `piper_x/urdf/piper_x_description.urdf` -> `piper_x.urdf`
- `LICENSE` (MIT; Copyright (c) 2026 aalicecc)

Only the serial revolute-joint origins, axes, and limits are used. Mesh references
are unused; this implementation performs no mesh collision checking. The chain
ends at `link6`, not the installed gripper's fingertips. Configure the actual TCP
translation/rotation separately. Joint firmware conventions and physical zeros
must be checked against this model on the installation. URDF limits are intersected
with the supplied follower limits; for example this model restricts J6 to ±120°
even if the firmware allows a larger range. No hardware settings are changed.
