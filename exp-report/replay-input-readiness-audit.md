# Replay-input readiness audit: DENSE, GLIDE, and Cassie

**Audit date:** 2026-10-31. This is a data/interface audit, not a simulation
run. Its purpose is to prevent a surrogate or partial trace from being labelled
a physical replay.

## DENSE EZGripper

The local public HDF5 files contain experiment groups with tactile arrays and
tactile timestamps for two sensors (for example
`.../sensor{1,2}/tactile_data_raw` and `tactile_timestamps`). They do not
contain a registered gripper trajectory/command, object 6-DoF pose, pad
geometry, or calibrated robot/object frame. Therefore the public release can
support tactile slip classification, but cannot initialize or score a physical
SAP--MuJoCo grasp replay.

In addition, the raw SAP MJCF import must remain rejected in strict mode: it
does not preserve the two tendons, tendon equality, and tendon actuator as
the stock MuJoCo model does. Accepting it as direct joint forces would create
a different mechanism rather than repairing the task.

## GLIDE plate transfer

`external/glide_data/lift_plate_mixed/meta/info.json` declares 30 episodes
at 30 Hz. Its recorded state/action vectors are the 14 robot joint and gripper
coordinates; the accompanying observations are videos. The release has no
plate/tomato 6-DoF pose stream, object CAD/mass/inertia, end-effector geometry,
camera-to-robot/object calibration, or force trace. Consequently the current
guarded plate carry remains a clearly labelled GPU surrogate. It cannot be
upgraded to a real-object replay without a new source package or hardware
calibration data.

## Cassie landing

The public `jumping_data/data/t_*.npy` files contain approximately 100 samples
from 0 to 0.0495 s. This supports the completed 50-ms open-loop impact replay,
but not a full landing/recovery metric. The missing post-impact horizon is a
data boundary, not a solver failure. The original Menagerie model also remains
unsuitable for the replay because SAP strict import cannot preserve its
equality/connect constraints and motor-actuator semantics. The reduced-tree
replay is therefore the valid bounded task.

## Status

No GPU rerun is scheduled for these three items: it cannot invent the missing
measurements or preserve unsupported mechanisms. Any future upgrade requires
one of: an author-provided registered data release, physical calibration/CAD
and measurement collection, or SAP support for the relevant MJCF semantics.
