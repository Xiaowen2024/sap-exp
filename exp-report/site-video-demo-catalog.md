# Website experiment video-demo catalog

## Scope and integrity rules

This catalog covers every experiment family presented at
[`xiaowen2024.github.io/sap-exp`](https://xiaowen2024.github.io/sap-exp/), plus
the linked rolling benchmark.  A demo is only called a cross-engine comparison
when its two panes show the same named task and the exact source settings are
recorded below.  A raw-mesh or capacity-truncated case may appear only as a
clearly labelled **representation/failure control**, never as an accuracy
winner.

The video bundle is being built in `exp-results/site-video-demos/`.  This file
is the source-of-record coverage matrix and will be completed alongside each
generated clip.

## Coverage matrix

| Website experiment | Intended representative demos | Current status |
|---|---|---|
| Rolling benchmark | native 25-ball task-level comparison | **complete** |
| Sub-Coulomb friction / creep | 4-N sub-threshold hold, matched compliance; optional MuJoCo NoSlip control | **complete — trace-driven replay** |
| Kinetic friction | free sliding, reference `v0 = 1 m/s` | **complete — trace-driven replay** |
| Force-matched simple pinch | near-threshold pass plus slip-boundary control | **complete — trace-driven boundary replay** |
| Dynamic impact / restitution | calibrated `e_eff=0.8` reference impact; coarse-dt bottleneck | **complete — trace-driven replay** |
| Repeated elastic bounce | energy-retention representative and dissipative bottleneck | **complete — trace-driven replay** |
| Square analytic cavity | centered insertion and 1-mm offset rigid-gap bottleneck | **complete — native/pose-replay comparison** |
| Round concave socket | shared-CoACD attempt plus raw-mesh invalid control | **complete — CoACD attempt; raw control retained separately** |
| Broad concave settling | sphere-in-plate or three-plate shared-CoACD settling | **complete — shared-CoACD stack demo** |
| Panda pick and transfer | native-controller successful task-level comparison; robustness/failure pair | **complete — success and 0.5-kg failure-mode demos** |

## Generated videos

### 01. Rolling — 25-ball native comparison

Video: [01_rolling_25ball_native.mp4](../exp-results/site-video-demos/01_rolling_25ball_native.mp4)

| Engine | Settings recorded in source metrics | Why selected |
|---|---|---|
| MuJoCo 3.12 | `dt=1 ms`, 4 s, Newton/`implicitfast`, 150-N xy force at 60°, 25 balls, MSE velocity error 0.00693 | representative calibrated/native MuJoCo task run |
| SAP/Warp CUDA | `dt=1 ms`, 4 s, Drake preset, pair friction ball--box 0.8 / box--ground 0.4, `ke=1e4 N/m`, `tau=30 ms`, MSE 0.360 | representative SAP CUDA task run; visibly illustrates the present rolling mismatch |

This is a **task-level native-setting demo**, not a claim that the two contact
parameter vectors are physically identical.

### 02. Panda pick-and-transfer — native successful demonstrations

Video: [02_panda_pick_native_success.mp4](../exp-results/site-video-demos/02_panda_pick_native_success.mp4)

| Engine | Settings recorded in source metrics | Why selected |
|---|---|---|
| MuJoCo | `dt=1 ms`; inverse-dynamics PD; arm gains 100/20, finger gains 3000/80; `solref="0.005 1.5"`, `solimp="0.9 0.95 0.001 0.5 2"` | successful baseline task execution |
| SAP/Warp | `dt=5 ms`; Drake preset; common computed-torque controller; arm gains 100/20, finger gains 3000/80 | successful native SAP task execution |

The controller and collision representations are intentionally engine-native;
the clip demonstrates task behavior, not a one-to-one solver-only comparison.

### 03. Kinetic friction — trace-driven comparison

Video: [03_kinetic_friction_trace_replay.mp4](../exp-results/site-video-demos/03_kinetic_friction_trace_replay.mp4)

This is an annotated **telemetry replay** rather than a renderer-native 3-D
clip. The block position, horizontal velocity, normal force and tangential
force in the animation are interpolated directly from the two recorded CSV
traces.

| Engine | Settings | Why selected |
|---|---|---|
| MuJoCo | free 1-kg block, `mu=0.5`, initial `vx=1 m/s`, `dt=0.5 ms` | reference-speed sliding trace |
| SAP/Warp | same task-level mass/friction/initial speed, Drake preset, `dt=0.5 ms` | same reference-speed sliding trace |

The video shows `vx(t)` and `rho = |Ft| / (mu Fn)` after release. It makes the
low-speed friction-regularization difference visible while retaining a direct
audit trail to the source CSVs.

### 04. Sub-Coulomb static friction — 4-N force hold

Video: [04_static_friction_hold_trace_replay.mp4](../exp-results/site-video-demos/04_static_friction_hold_trace_replay.mp4)

This is an annotated **telemetry replay** of the three-box force-hold traces.
It begins after the 4-N command has completed its ramp and shows the 5.5-s
hold window.  The source animation displays the middle--top relative slip and
the aggregate friction utilization `rho` in lockstep with the stack motion.

| Engine | Settings | Interpretation of this selection |
|---|---|---|
| MuJoCo | `dt=1 ms`, `mu=0.5`, elliptic cone, `impratio=10`, `NoSlip=3`; 4-N hold | native anti-slip control: tail creep 0.00347 um/s |
| SAP/Warp | `dt=0.5 ms`, Drake preset, pair `ke=1e4 N/m`, pair `tau=3 ms`, `mu=0.5`, `rigid_gap=0`; 4-N hold | stable-normal-compliance reference: tail creep 2.516 um/s, `rho=0.830` |

The target 4 N is below the ideal top-interface static limit of 4.905 N.
The MuJoCo pane is deliberately a **native best-control** (`NoSlip`) rather
than a matched-contact-model claim.  It demonstrates the special anti-slip
mechanism; the matched-compliance MuJoCo baseline remains documented in the
friction report and should be used for solver-formulation conclusions.

### 05. Force-matched simple pinch — threshold boundary

Video: [05_simple_pinch_force_matched_boundary.mp4](../exp-results/site-video-demos/05_simple_pinch_force_matched_boundary.mp4)

This telemetry replay uses the boundary mass `m=0.3262 kg`, at which the
measured grasp capacity is almost exactly the object weight.  Both traces pass
the normal-contact gate and have no SAP contact truncation; the comparison is
therefore valid for the measured-force grasp experiment.

| Engine | Settings | Observed result |
|---|---|---|
| MuJoCo | `dt=0.5 ms`, `mu=0.4`, 0.2-mm geometric compression/side, measured preload 3.931 N/side | fails: final lift follow 2.59 mm; mean lift `rho` 0.998 |
| SAP/Warp | `dt=0.5 ms`, Drake preset, pair `ke=1e4 N/m`, pair `tau=3 ms`, 0.001-mm geometric compression/side, measured preload 4.040 N/side, capacity 128 / no truncation | succeeds: final lift follow 57.96 mm; mean lift `rho` 0.980 |

The playback fraction is normalized separately over each engine's native
lift-phase duration (1.25 s for MuJoCo, 2.75 s for SAP); the in-video clock
prints both source times.  This makes the force-matched success-boundary
behavior visible without claiming that the two controllers execute the same
trajectory duration.

### 06. Dynamic impact / restitution — equal-timescale coarse-step control

Video: [06_impact_equal_timescale_coarse_dt.mp4](../exp-results/site-video-demos/06_impact_equal_timescale_coarse_dt.mp4)

This is a trace-driven replay of the frictionless, single-contact sphere--plane
impact. It is the stronger extension of the impact benchmark: both engines
were behavior-calibrated at `v_ref=0.5 m/s`, `dt_ref=0.025 ms` to the same
`e_eff=0.8` and reference contact duration near 30 ms, then parameters were
frozen. The video displays their common coarse transfer condition:
`v_impact=0.5 m/s`, `dt=0.5 ms` (`dt/t_c ≈ 0.0167`).

| Engine | Frozen native setting | At `dt=0.5 ms` |
|---|---|---|
| MuJoCo | Newton + `implicitfast`, direct `solref` stiffness 11027.6 `s^-2`, damping scale 0.0744171, constant `solimp=0.9`, `condim=1` | `e_eff=0.8027`, measured `t_c=30 ms` |
| SAP/Warp | Drake preset; pair `ke=1e4 N/m`, `tau=1.46094 ms` (equal shape values: `2e4 N/m`, `0.730469 ms`) | `e_eff=0.7452`, measured `t_c=30 ms` |

The plotted normal force is the telemetry used for the impulse check; it is
not inferred from geometric overlap.  This is a controlled restitution-transfer
demo, rather than an absolute-material ground-truth comparison.

### 07. Repeated elastic bounce — 49-ball energy retention

Video: [07_repeated_elastic_bounce_energy_replay.mp4](../exp-results/site-video-demos/07_repeated_elastic_bounce_energy_replay.mp4)

This telemetry replay uses the benchmark's recorded energy and representative
ball-height traces over the complete 20-s release.  It is a **native-setting
energy comparison**, not an equal-contact-timescale calibration.

| Engine | Selected setting | Selected outcome |
|---|---|---|
| MuJoCo | `dt=0.5 ms`, nominal direct-contact stiffness `1e5 N/m`, zero damping | first rebound-height ratio 0.99948; final energy ratio 1.00485 |
| SAP/Warp | CPU, `approx32` preset, `dt=0.5 ms`, `ke=1e4 N/m`, `tau=0` | first rebound-height ratio 0.98194; final energy ratio 0.83403 |

All 49 balls share the same initial condition in the benchmark; the animated
sphere is ball 0 while the energy plot is the system mechanical energy.  The
clip highlights energy behavior, not performance: the source SAP run was CPU
and substantially slower than the MuJoCo run.

### 08. Analytic square cavity — 1-mm offset insertion

Video: [08_square_cavity_offset_labelled.mp4](../exp-results/site-video-demos/08_square_cavity_offset_labelled.mp4)

The left pane is a fresh native MuJoCo rendering.  The right pane is an
offline rendering of the recorded SAP pose trace in the same analytic visual
geometry; the SAP dynamics are **not** re-solved by MuJoCo. Both show a 20 ×
20 × 50-mm peg released with 1-mm lateral offset into a primitive square
cavity with 2-mm one-sided clearance, gravity and `mu=0.8`.

| Engine | Settings | Outcome |
|---|---|---|
| MuJoCo | `dt=0.5 ms`, Newton + `implicitfast`, 100 iterations, `solref=(1 ms, 1)`, default `solimp=(.9,.95,.001,.5,2)` | enters and rests at `z≈35.000 mm`; peak primitive overlap 0.141 mm |
| SAP/Warp | Drake preset, `dt=0.5 ms`, shape `ke=1e4 N/m`, shape `tau=3 ms`, `rigid_gap=0.1 mm` | enters and rests at `z≈35.090 mm`; no truncation; peak `phi0` overlap 0.293 mm |

The 0.1-mm gap is intentional: at the original 1-mm `rigid_gap` (equal to
the remaining lateral clearance) SAP escaped/fell through. The clip therefore
shows the valid primitive-control setting, not the invalid gap=1-mm failure.

### 09. Round concave socket — shared-CoACD attempt

Video: [09_round_socket_coacd_attempt.mp4](../exp-results/site-video-demos/09_round_socket_coacd_attempt.mp4)

Both panes use the shared 32-piece CoACD collision representation for the
2-mm-clearance, 1-mm-offset, 1-degree-tilt round socket. The MuJoCo pane is
native; the SAP pane is an offline render of the recorded SAP CoACD pose trace
in the corresponding collision/visual geometry. The latter is not re-solved
in MuJoCo.

| Engine | Settings | Reading |
|---|---|---|
| MuJoCo | `dt=0.5 ms`, `implicitfast`, elliptic cone, `solref=1 ms`, radius 10 mm | chosen as a credible low-clearance CoACD-style demonstration; it is not the full 25-case matrix verdict |
| SAP/Warp | Drake preset, shape `ke=1e4 N/m`, shape `tau=10 ms`, `rigid_gap=1 mm`, `dt=0.5 ms`, radius 10 mm | formal matrix case: no entry, final radial offset 22.56 mm, zero truncation |

This clip documents the **current tested configuration**, not an engine-best
ranking: MuJoCo had a wider `solref/solimp` search, while SAP's formal CoACD
matrix used one native material point.  The existing raw-/watertight-mesh SAP
clips remain invalid representation controls because their cavity witness can
be nonphysical; they are intentionally not used as a solver-accuracy pane.

### 10. Broad concave settling — three-plate shared-CoACD stack

Video: [10_broad_concave_three_plate_coacd.mp4](../exp-results/site-video-demos/10_broad_concave_three_plate_coacd.mp4)

This is the broad concave three-plate settling task using the shared CoACD
representation. The MuJoCo pane is the existing native CoACD rendering. The
SAP pane reconstructs the logged vertical positions in the matching visual
scene; its formal trace did not include xy/orientation, so it is explicitly a
**z-trajectory replay**, not a complete pose renderer.

| Engine | Formal benchmark setting | Outcome at 3 s |
|---|---|---|
| MuJoCo | `dt=0.5 ms`, Newton + `implicitfast`, `solref=.008 1`, `solimp=.95 .99 .002`, shared CoACD | order preserved; max 111 contacts; tail vertical jitter 0.005/0.018/0.046 mm (bottom/middle/top) |
| SAP/Warp | Drake preset, `dt=0.5 ms`, shape `ke=1e5 N/m`, shape `tau=10 ms`, `rigid_gap=2 mm`, shared CoACD | order preserved; max 218 candidates, no truncation; tail vertical jitter 0.000002/0.00384/0.00430 mm |

The native MuJoCo video and formal MuJoCo metric row have close but not
pixel-identical rendering settings; therefore this clip is an explanatory
representation/settling demo, while the table above is the authoritative
metric configuration. Raw-mesh controls are excluded from force/penetration
claims because their concave witness semantics are invalid.

### 11. Panda pick-and-transfer — 0.5-kg failure modes

Video: [11_panda_pick_0p5kg_failure_modes.mp4](../exp-results/site-video-demos/11_panda_pick_0p5kg_failure_modes.mp4)

This is the matched task-level stress condition from the frame-corrected v2
evaluation: 0.5-kg cube, nominal trajectory, 15-N/side finger effort cap and
the frozen native settings. Both fail the project retention gate, but by
different mechanisms.

| Engine | Frozen setting | Failure telemetry |
|---|---|---|
| MuJoCo | `dt=1 ms`, elliptic cone, Newton + `implicitfast`, `solref=(3 ms,.75)`, arm 150/30, finger 3000/80 | bilateral normal-contact instability; finger duty 0, final cube `z=20.0 mm` |
| SAP/Warp | `dt=5 ms`, Drake preset, pair `ke=5e4 N/m`, pair `tau=1 ms`, native target drive 5000/200 | static slip with bilateral duty 1; max hand-relative slip 5.15 mm; no capacity truncation |

As with the success demo, controllers remain engine-native. The video is
useful for qualitative failure diagnosis, not a pure contact-solver ranking.

## Validation performed for generated clips

- Both source files existed locally and were decoded by `ffmpeg`.
- All outputs are H.264 and were decoded once with `ffprobe`. Native composed
  clips, the labelled square-cavity clip, and the telemetry replays all carry
  in-video engine labels. Native two-pane clips are 1440 × 550, while the
  telemetry replays are 1350 × 760.
- The rolling label was independently checked against both `metrics.json`
  files; it states `dt=1 ms` for both engines.

## 中文说明

这个 catalog 的原则是：只有同一个 task、并且参数可追溯时，才叫真正的
MuJoCo--SAP 对比视频。raw mesh、capacity overflow 或不同 collision
representation 的录像也可以保留，但只能明确叫 failure/representation
control，不能拿来宣称谁更准确。

目前网站上每个实验家族都已有至少一个代表性 demo；rolling、Panda success/
failure、round socket 与 broad concave 都包含原生或原生+pose-replay 画面。需要
逐帧可追溯的力、速度、能量或 slip 的案例使用已有 CSV 的 trace replay，并在上面
明确标出来；每个视频所用参数均已写入本表。
