# Panda cube pick: SAP vs. MuJoCo

This is one matched *task* run: the same FR3 Panda URDF, `40 mm` cube at
`[0.35, -0.15, 0.02] m`, the same reference keyframes, gravity, and `mu=0.8`.
Robot self-collision is disabled in both scenes.  Both use computed-torque PD
intent with arm `Kp/Kd=100/20` and fingers `3000/80`; the MuJoCo adapter also
clips torques to the effort limits declared in the source URDF.

| Engine | Final cube centre z | Peak z | Lifted above 0.10 m | Max penetration |
|---|---:|---:|---|---:|
| SAP approx32 | 0.192151 m | 0.192151 m | yes | 2.260 mm |
| SAP drake preset | 0.193107 m | 0.193107 m | yes | 0.286 mm |
| MuJoCo 3.12 | 0.019943 m | 0.022437 m | no | 11.601 mm |

The MuJoCo run does not form a stable grasp: the cube rises only about `2.44
mm`, then remains on the ground.  This is a real result for this adapter, not
a claim that the solvers have an inherently identical controller: SAP uses a
Drake `InverseDynamicsController`, while MuJoCo uses its own inverse dynamics
with the same PD gains and URDF torque bounds.  The raw traces and videos are
therefore retained for inspection rather than treating this as a calibrated,
engine-only conclusion.

* `sap/` contains SAP metrics, traces and video.
* `mujoco/` contains MuJoCo metrics, trace, joint trace and video.
