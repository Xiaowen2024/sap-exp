# SOP generalizability rerun register

> **2026-11-04:** SAP results now require a passing `sap_preflight.py`; the fixed SAP build (branch `fix-gamma-readout`, not yet merged) is the only build that passes on CPU and GPU. See `exp-report/sap-force-readout-audit.md`.

This register is the execution inventory for replacing historical or
coverage-asymmetric artifacts. A task is not marked complete merely because a
previous result exists: it needs a pre-run common manifest, a fixed-physics
B=1/B=5 screen for both engines, and the declared R=20 frozen matrix.

| Regime | Task | Existing result | Required corrective run | Current state |
| --- | --- | --- | --- | --- |
| Static friction | Three-box push | historical common-looking sweep, but no pre-run common manifest/raw-repeat contract; not affected by SAP bugs A/B (displacement-based) | declare and run fresh B=1/B=5 selection plus force/mass/friction/three-`dt`/push-setup matrix | **strict-v3** manifest (dt factorial over retain/release); runners pending |
| Static friction | Simple pinch | SAP 17-cell rerun with fixed SAP: 340/340 (2026-11-04); v1 force-matched SAP run crashed | force-matched manifest, shared ideal labels, pad-surface transfer, R=20 | **force-matched-v2** manifest (dt factorial; MuJoCo tc ≥ 2·dt); runners need the fixed SAP build + preflight |
| Sustained contact | Three-plate stack | fixed-SAP rerun of the historical grid: 8/8 held-out 20/20 (2026-11-04) | fresh fixed-physics B=1/B=5 plus thickness/mass/radius/offset/three-`dt` matrix | **strict-v3** manifest (gate outcome, no labels); runners pending |
| Sustained contact | Cube-toss resting tail | fixed-SAP force trace: load-balanced, 0.0024% closure error (2026-11-04) | matched SAP/MuJoCo manifold force trace from the same frozen states, plus three-`dt`/state/setup matrix | MuJoCo matched trace pending |
| Dynamic impact | GAUGE ball bounce | artifact marked partial | common B=1/B=5 selection and release-height/lateral/mass/trajectory/three-`dt` matrix with a fixed sampler | manifest and rerun pending |
| Dynamic impact | ContactNets cube toss | non-`dt` artifact marked partial | common B=1/B=5 selection and speed/yaw/mass/friction/state/three-`dt` matrix | manifest and rerun pending |
| Dynamic friction | GAUGE slope slider | trajectory-only comparison; no matched force reference | common trajectory-only rerun is possible; force-fidelity claim remains unavailable without released force data | requires shared SAP runner and manifest |
| Dynamic friction | MIT Push | SAP-only reconstruction | implement matched MuJoCo replay, same held-out trials and synchronized pose/wrench windows | MuJoCo runner missing |
| Energy | Pendulum | dt is a B candidate in historical artifact | fixed-physics B=1/B=5 then amplitude/cycle/three-`dt` matrix | rerun pending |
| Energy | Newton cradle | no aligned cross-engine artifact | fixed-physics B=1/B=5 then amplitude/material/impact-offset/three-`dt` matrix | SAP runner missing |
| Composite | Round socket | SAP selection changes `dt`; clearance < 2 mm is geometrically infeasible in the CoACD asset (bore audit 2026-11-04) | **strict-v3** manifest: clearance cells `not run`; offset/tilt/height/surface/three-`dt` at 2 mm | rebuild socket geometry before any clearance claim; old strict-v2 queue on the server should not run |
| Composite | Cassie landing | public traces cover only about 50 ms and no SAP-equivalent replay | no full task rerun is valid until a common state/action horizon is available | input boundary, not runnable |
| Composite | DENSE/EZGripper | public task state/command/geometry registration absent; raw SAP import rejected | no aligned replay can be defined from released data | input boundary, not runnable |
| Composite | GLIDE transfer | object pose/CAD/calibration are absent | no aligned replay can be defined from released data | input boundary, not runnable |

For the three input-boundary rows, the corrective action is not to invent a
simulation. They remain explicitly unavailable unless the required data are
released or supplied. For GAUGE Slope, the trajectory result may be rerun, but
no run can create a missing physical force trace; the result must remain a
trajectory comparison rather than friction-force fidelity.
