# Tasks

One folder per benchmark task, grouped by the protocol family it belongs to.
Each family folder holds the family SOP (`README.md`) and any runner that covers
every task in the family; each task folder holds its own SOP and code.

```
tasks/<family>/
  README.md                family SOP (scope, metrics shared by the tasks)
  run_<family>_*.py        runners that cover all tasks in the family
  <engine>/                family-level runners for one engine
  <task>/
    SOP.md                 task metrics and pass/fail rules
    *_common.py            task logic shared by the engine runners
    mujoco/ mjwarp/ sap/ genesis/   one runner folder per simulator
    prep/                  data, geometry and manifest builders
    analysis/              render, summarize, audit and comparison scripts
    assets/ configs/       task geometry and engine configs
```

Manifests are not stored per task: every runner and the frozen-B5 registries
look them up by name under [`protocol/manifests/`](../protocol/manifests/), and
their SHA-256 is part of each result, so they stay in one folder and unchanged.

## Index

| Family | Task | SOP | Engine folders | Other | Manifests |
| --- | --- | --- | --- | --- | --- |
| Static friction | [simple-pinch](static-friction/simple-pinch/) | [SOP](static-friction/simple-pinch/SOP.md) | mjwarp sap genesis | analysis | [`simple_pinch_force_controlled_v3`](../protocol/manifests/simple_pinch_force_controlled_v3.json), [`simple_pinch_force_matched_v1`](../protocol/manifests/simple_pinch_force_matched_v1.json), [`simple_pinch_force_matched_v2`](../protocol/manifests/simple_pinch_force_matched_v2.json) |
| Static friction | [three-box-push](static-friction/three-box-push/) | [SOP](static-friction/three-box-push/SOP.md) | mujoco mjwarp sap genesis | analysis | [`three_box_push_strict_v2`](../protocol/manifests/three_box_push_strict_v2.json), [`three_box_push_strict_v3`](../protocol/manifests/three_box_push_strict_v3.json), [`three_box_push_strict_v4`](../protocol/manifests/three_box_push_strict_v4.json) |
| Dynamic friction | [mit-push](dynamic-friction/mit-push/) | [SOP](dynamic-friction/mit-push/SOP.md) | mjwarp genesis | — | [`mit_push_dynamic_friction_v3`](../protocol/manifests/mit_push_dynamic_friction_v3.json) |
| Dynamic friction | [slope-slider](dynamic-friction/slope-slider/) | [SOP](dynamic-friction/slope-slider/SOP.md) | mjwarp genesis | analysis | [`gauge_slope_slider_dynamic_friction_v3`](../protocol/manifests/gauge_slope_slider_dynamic_friction_v3.json) |
| Dynamic impact | [ball-bounce](dynamic-impact/ball-bounce/) | [SOP](dynamic-impact/ball-bounce/SOP.md) | mujoco mjwarp | analysis | [`gauge_ball_bounce_dynamic_impact_v2`](../protocol/manifests/gauge_ball_bounce_dynamic_impact_v2.json), [`gauge_ball_bounce_dynamic_impact_v3`](../protocol/manifests/gauge_ball_bounce_dynamic_impact_v3.json) |
| Dynamic impact | [cassie-landing](dynamic-impact/cassie-landing/) | — | mujoco mjwarp | — | — |
| Dynamic impact | [cube-toss](dynamic-impact/cube-toss/) | [SOP](dynamic-impact/cube-toss/SOP.md) | mujoco mjwarp | prep analysis | [`contactnets_cube_toss_dynamic_impact_v2`](../protocol/manifests/contactnets_cube_toss_dynamic_impact_v2.json) |
| Energy conservation | [newtons-cradle](energy-conservation/newtons-cradle/) | [SOP](energy-conservation/newtons-cradle/SOP.md) | mjwarp | prep | [`newtons_cradle_energy_v2`](../protocol/manifests/newtons_cradle_energy_v2.json), [`newtons_cradle_energy_v3`](../protocol/manifests/newtons_cradle_energy_v3.json), [`newtons_cradle_gauge_real_v1`](../protocol/manifests/newtons_cradle_gauge_real_v1.json), [`newtons_cradle_gauge_real_v2`](../protocol/manifests/newtons_cradle_gauge_real_v2.json), [`newtons_cradle_gauge_v4`](../protocol/manifests/newtons_cradle_gauge_v4.json) |
| Energy conservation | [pendulum](energy-conservation/pendulum/) | [SOP](energy-conservation/pendulum/SOP.md) | mjwarp genesis | — | [`pendulum_energy_v2`](../protocol/manifests/pendulum_energy_v2.json), [`pendulum_gauge_real_v1`](../protocol/manifests/pendulum_gauge_real_v1.json), [`pendulum_gauge_v3`](../protocol/manifests/pendulum_gauge_v3.json) |
| Sustained contact | [cube-toss-tail](sustained-contact/cube-toss-tail/) | [SOP](sustained-contact/cube-toss-tail/SOP.md) | mujoco mjwarp genesis | — | [`contactnets_cube_tail_sustained_v2`](../protocol/manifests/contactnets_cube_tail_sustained_v2.json), [`contactnets_cube_tail_sustained_v3`](../protocol/manifests/contactnets_cube_tail_sustained_v3.json) |
| Sustained contact | [three-plate-stack](sustained-contact/three-plate-stack/) | [SOP](sustained-contact/three-plate-stack/SOP.md) | mjwarp sap genesis | analysis | [`three_plate_stack_strict_v2`](../protocol/manifests/three_plate_stack_strict_v2.json), [`three_plate_stack_strict_v3`](../protocol/manifests/three_plate_stack_strict_v3.json), [`three_plate_stack_strict_v3p1`](../protocol/manifests/three_plate_stack_strict_v3p1.json) |
| Composite tasks | [cassie-landing](composite/cassie-landing/) | [SOP](composite/cassie-landing/SOP.md) | mujoco sap | prep analysis configs | — |
| Composite tasks | [dense-ezgripper-grasp](composite/dense-ezgripper-grasp/) | [SOP](composite/dense-ezgripper-grasp/SOP.md) | mjwarp | configs | — |
| Composite tasks | [glide-tomato-plate-transfer](composite/glide-tomato-plate-transfer/) | [SOP](composite/glide-tomato-plate-transfer/SOP.md) | mjwarp | — | — |
| Composite tasks | [peg-in-hole](composite/peg-in-hole/) | [SOP](composite/peg-in-hole/SOP.md) | mujoco mjwarp sap genesis | prep analysis assets | [`round_socket_aligned_frozen_v1`](../protocol/manifests/round_socket_aligned_frozen_v1.json), [`round_socket_clearance_2p0`](../protocol/manifests/round_socket_clearance_2p0.json), [`round_socket_internal_selection_preflight`](../protocol/manifests/round_socket_internal_selection_preflight.json), [`round_socket_selection_dt025`](../protocol/manifests/round_socket_selection_dt025.json), [`round_socket_strict_v2`](../protocol/manifests/round_socket_strict_v2.json), [`round_socket_strict_v3`](../protocol/manifests/round_socket_strict_v3.json), [`round_socket_strict_v4`](../protocol/manifests/round_socket_strict_v4.json), [`round_socket_strict_v5_exploratory`](../protocol/manifests/round_socket_strict_v5_exploratory.json), [`round_socket_strict_v6`](../protocol/manifests/round_socket_strict_v6.json), [`round_socket_strict_v7_draft`](../protocol/manifests/round_socket_strict_v7_draft.json), [`round_socket_strict_v8_draft`](../protocol/manifests/round_socket_strict_v8_draft.json), [`round_socket_strict_v9_draft`](../protocol/manifests/round_socket_strict_v9_draft.json) |
| Real-robot playback | [rh20t-dish-on-rack](real-playback/rh20t-dish-on-rack/) | [SOP](real-playback/rh20t-dish-on-rack/SOP.md) | — | — | [`rh20t_dish_rack_playback_v1_draft`](../protocol/manifests/rh20t_dish_rack_playback_v1_draft.json), [`rh20t_dish_rack_playback_v1_pilot_c3`](../protocol/manifests/rh20t_dish_rack_playback_v1_pilot_c3.json) |

Engine folders list only per-task subfolders. Family-level runners also apply:
[`run_static_friction_v2.py`](static-friction/run_static_friction_v2.py) and
[`run_dynamic_friction_v2.py`](dynamic-friction/run_dynamic_friction_v2.py) (MuJoCo and MuJoCo Warp),
[`dynamic-impact/{mujoco,mjwarp,genesis}/`](dynamic-impact/) (ball bounce and cube toss),
[`energy-conservation/{mujoco,mjwarp}/`](energy-conservation/) and
[`run_sustained_contact_v1.py`](sustained-contact/run_sustained_contact_v1.py).
Many SAP runs use runners from the SAP-WARP-INTERNAL repository
(`local_experiments/`), which is not part of this repository.

## Other task groups

- [`robodojo/`](robodojo/) — RoboDojo trajectory playback (deposit coin, insert tubes,
  laptop insertion and pickup, stack bowls, sweep blocks). One package,
  [`robodojo_playback/`](robodojo/robodojo_playback/), serves all of them; SOPs are in
  [`robodojo/sop/`](robodojo/sop/). Run modules with
  `PYTHONPATH=tasks/robodojo python -m robodojo_playback.<module>`.
- [`real-playback/`](real-playback/) — real-robot playback: the RH20T package
  [`real_playback/`](real-playback/real_playback/) (`PYTHONPATH=tasks/real-playback python -m real_playback.rp_run`),
  the FMB pilot in [`fmb_playback/`](real-playback/fmb_playback/), and the
  [runner SOP](real-playback/runner-sop.md).
- [`soft-contact/`](soft-contact/) — soft-pad pinch study (MuJoCo, SAP, IPC); manifests
  `soft_pad_*`, `soft_pinch_*` and `ipc_graspsim_*`.
- [`early-studies/`](early-studies/) — studies that predate the B=1/B=5 protocol and have no
  task SOP: kinetic friction, sphere-plane impact, thin disc, square cavity and Panda manipulation.

Cross-task engine infrastructure (Genesis adapters, preflights and queues; the
MuJoCo 4.0 rerun; the SAP frozen-B5 rerun) is in [`engines/`](../engines/).
