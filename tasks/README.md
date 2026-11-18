# Tasks

Tasks are grouped by what they are compared against, following the latest task set
(Part V, 2026-11-18):

| Category | Folder | Reference |
| --- | --- | --- |
| Simulation only · atomic | [`simulation-only-atomic/`](simulation-only-atomic/) | analytic or protocol-defined expectations, one contact mechanism per task |
| Simulation only · composite | [`simulation-only-composite/`](simulation-only-composite/) | analytic statics; several mechanisms coupled (round socket only) |
| Real data · simple | [`real-data-simple/`](real-data-simple/) | released real trajectories (GAUGE, ContactNets, MIT Push) |
| Robot playback | [`robot-playback/`](robot-playback/) | recorded robot motion replayed in simulation (RoboDojo, RH20T) |

```
tasks/<category>/<task>/
  SOP.md                         task metrics and pass/fail rules
  *_common.py                    task logic shared by the engine runners
  mujoco/ mjwarp/ sap/ genesis/  one runner folder per simulator
  prep/  analysis/               builders; render, summarize and audit scripts
  assets/                        task geometry
tasks/<category>/shared/         runners that cover several tasks of the category
```

Family-level protocols (static friction, dynamic friction, dynamic impact, energy
conservation, sustained contact, composite, real playback) are in
[`protocol/families/`](../protocol/families/). Manifests are not stored per task:
runners and the frozen-B5 registries look them up by name in
[`protocol/manifests/`](../protocol/manifests/) and record their SHA-256 in every
result, so they stay in one folder and are never edited in place.

## Simulation only · atomic

| Task | SOP | Engine folders | Manifests |
| --- | --- | --- | --- |
| [Three-plate stack](simulation-only-atomic/three-plate-stack/) | [SOP](simulation-only-atomic/three-plate-stack/SOP.md) | mjwarp sap genesis | [`three_plate_stack_strict_v2`](../protocol/manifests/three_plate_stack_strict_v2.json), [`three_plate_stack_strict_v3`](../protocol/manifests/three_plate_stack_strict_v3.json), [`three_plate_stack_strict_v3p1`](../protocol/manifests/three_plate_stack_strict_v3p1.json) |
| [Three-box push](simulation-only-atomic/three-box-push/) | [SOP](simulation-only-atomic/three-box-push/SOP.md) | mujoco mjwarp sap genesis | [`three_box_push_strict_v2`](../protocol/manifests/three_box_push_strict_v2.json), [`three_box_push_strict_v3`](../protocol/manifests/three_box_push_strict_v3.json), [`three_box_push_strict_v4`](../protocol/manifests/three_box_push_strict_v4.json) |
| [Simple pinch](simulation-only-atomic/simple-pinch/) | [SOP](simulation-only-atomic/simple-pinch/SOP.md) | mjwarp sap genesis | [`simple_pinch_force_controlled_v3`](../protocol/manifests/simple_pinch_force_controlled_v3.json), [`simple_pinch_force_matched_v1`](../protocol/manifests/simple_pinch_force_matched_v1.json), [`simple_pinch_force_matched_v2`](../protocol/manifests/simple_pinch_force_matched_v2.json) |
| [Pendulum (idealised)](simulation-only-atomic/pendulum-ideal/) | [SOP](simulation-only-atomic/pendulum-ideal/SOP.md) | mujoco mjwarp genesis | [`pendulum_energy_v2`](../protocol/manifests/pendulum_energy_v2.json) |

[`shared/run_static_friction_v2.py`](simulation-only-atomic/shared/run_static_friction_v2.py) runs simple pinch and three-box push on
MuJoCo and MuJoCo Warp. The GAUGE pendulum, which uses real trajectories, is under real data.

## Simulation only · composite

| Task | SOP | Engine folders | Manifests |
| --- | --- | --- | --- |
| [Round socket (peg-in-hole)](simulation-only-composite/round-socket/) | [SOP](simulation-only-composite/round-socket/SOP.md) | mujoco mjwarp sap genesis | [`round_socket_aligned_frozen_v1`](../protocol/manifests/round_socket_aligned_frozen_v1.json), [`round_socket_clearance_2p0`](../protocol/manifests/round_socket_clearance_2p0.json), [`round_socket_internal_selection_preflight`](../protocol/manifests/round_socket_internal_selection_preflight.json), [`round_socket_selection_dt025`](../protocol/manifests/round_socket_selection_dt025.json), [`round_socket_strict_v2`](../protocol/manifests/round_socket_strict_v2.json), [`round_socket_strict_v3`](../protocol/manifests/round_socket_strict_v3.json), [`round_socket_strict_v4`](../protocol/manifests/round_socket_strict_v4.json), [`round_socket_strict_v5_exploratory`](../protocol/manifests/round_socket_strict_v5_exploratory.json), [`round_socket_strict_v6`](../protocol/manifests/round_socket_strict_v6.json), [`round_socket_strict_v7_draft`](../protocol/manifests/round_socket_strict_v7_draft.json), [`round_socket_strict_v8_draft`](../protocol/manifests/round_socket_strict_v8_draft.json), [`round_socket_strict_v9_draft`](../protocol/manifests/round_socket_strict_v9_draft.json) |

## Real data · simple

| Task | SOP | Engine folders | Manifests |
| --- | --- | --- | --- |
| [GAUGE ball bounce](real-data-simple/ball-bounce/) | [SOP](real-data-simple/ball-bounce/SOP.md) | mujoco mjwarp | [`gauge_ball_bounce_dynamic_impact_v2`](../protocol/manifests/gauge_ball_bounce_dynamic_impact_v2.json), [`gauge_ball_bounce_dynamic_impact_v3`](../protocol/manifests/gauge_ball_bounce_dynamic_impact_v3.json) |
| [ContactNets cube toss](real-data-simple/cube-toss/) | [SOP](real-data-simple/cube-toss/SOP.md) | mujoco mjwarp | [`contactnets_cube_toss_dynamic_impact_v2`](../protocol/manifests/contactnets_cube_toss_dynamic_impact_v2.json) |
| [Cube-toss resting tail](real-data-simple/cube-toss-tail/) | [SOP](real-data-simple/cube-toss-tail/SOP.md) | mujoco mjwarp genesis | [`contactnets_cube_tail_sustained_v2`](../protocol/manifests/contactnets_cube_tail_sustained_v2.json), [`contactnets_cube_tail_sustained_v3`](../protocol/manifests/contactnets_cube_tail_sustained_v3.json) |
| [MIT Push](real-data-simple/mit-push/) | [SOP](real-data-simple/mit-push/SOP.md) | mjwarp genesis | [`mit_push_dynamic_friction_v3`](../protocol/manifests/mit_push_dynamic_friction_v3.json) |
| [GAUGE slope slider](real-data-simple/slope-slider/) | [SOP](real-data-simple/slope-slider/SOP.md) | mjwarp genesis | [`gauge_slope_slider_dynamic_friction_v3`](../protocol/manifests/gauge_slope_slider_dynamic_friction_v3.json) |
| [GAUGE pendulum](real-data-simple/gauge-pendulum/) | — | mjwarp genesis | [`pendulum_gauge_real_v1`](../protocol/manifests/pendulum_gauge_real_v1.json), [`pendulum_gauge_v3`](../protocol/manifests/pendulum_gauge_v3.json) |

[`shared/`](real-data-simple/shared/) holds the runners common to ball bounce and cube toss
(`impact_v2_common.py`, the B=5 R=20 selection, and the Genesis impact runners) and
`run_dynamic_friction_v2.py` (MIT Push and slope slider).

## Robot playback

| Task | SOP | Engine folders | Manifests |
| --- | --- | --- | --- |
| [RH20T dish on rack](robot-playback/rh20t-dish-on-rack/) | [SOP](robot-playback/rh20t-dish-on-rack/SOP.md) | — | [`rh20t_dish_rack_playback_v1_draft`](../protocol/manifests/rh20t_dish_rack_playback_v1_draft.json), [`rh20t_dish_rack_playback_v1_pilot_c3`](../protocol/manifests/rh20t_dish_rack_playback_v1_pilot_c3.json) |
| [RoboDojo](robot-playback/robodojo/) — stack bowls, insert tubes, deposit coin, laptop insertion / pickup, sweep blocks | [SOPs](robot-playback/robodojo/sop/) | one package for all engines | — |

Both playback folders are Python packages. Run their modules with
`PYTHONPATH=tasks/robot-playback/robodojo python -m robodojo_playback.<module>` and
`PYTHONPATH=tasks/robot-playback/rh20t-dish-on-rack python -m real_playback.rp_run`.
The RH20T [runner SOP](robot-playback/rh20t-dish-on-rack/runner-sop.md) covers the
real-playback pipeline.

## Not in this repository

Many SAP runs use runners from the SAP-WARP-INTERNAL repository
(`local_experiments/`). Tasks outside the current set (Newton's cradle, soft-pad
pinch, Cassie, EZGripper, GLIDE, FMB and the pre-protocol studies) are not
published here yet; the few of their files that were already public are kept in
[`archive/`](../archive/).
