# MuJoCo contact-stiffness sweep

All runs use the same Panda/cube scene and controller.  Only the MuJoCo
contact `solref="timeconst dampratio"` was changed.  A run is *numerically
stable* when no state diverges.  A *stable grasp at 6 s* additionally requires
cube centre `z > 0.10 m` at the final frame (not merely a transient peak).

## Main finding

The default `solref="0.02 1"` is stable numerically but does **not** grasp:
final cube z is `0.01994 m` and maximum penetration is `11.26 mm`.

Four scanned settings retain the cube at the final frame:

* `solref="0.008 1.25"`: final z `0.10617 m`, penetration `3.17 mm`.
* `solref="0.005 1.25"`: final z `0.15541 m`, penetration `3.34 mm`, but
  peak arm tracking error is large (`2.79 rad`).
* `solref="0.005 1.5"`: final z `0.13440 m`, penetration `1.70 mm`, peak
  arm tracking error `1.20 rad`.
* `solref="0.004 1"`: final z `0.11501 m`, penetration `1.21 mm`, peak arm
  tracking error `0.98 rad`.

For this uncalibrated adapter, `0.005 1.5` is the best lift-retention setting
in this sweep; `0.004 1` is the more conservative setting when minimizing
penetration and controller disturbance matters more than lift height.  Neither
yet matches SAP's roughly `0.19 m` final height, so this is a parameter window
for follow-up tuning, not an engine-only verdict.

The complete machine-readable results are in `metrics.json` and `summary.csv`.
