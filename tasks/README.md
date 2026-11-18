# Tasks

Everything is organized by task. Tasks are grouped by what they are compared
against, following the latest task set (Part V, 2026-11-18):

| Category | Folder | Reference |
| --- | --- | --- |
| Simulation only · atomic | [`simulation-only-atomic/`](simulation-only-atomic/) | analytic or protocol-defined expectations, one contact mechanism per task |
| Simulation only · composite | [`simulation-only-composite/`](simulation-only-composite/) | analytic statics; several mechanisms coupled (round socket only) |
| Real data · simple | [`real-data-simple/`](real-data-simple/) | released real trajectories (GAUGE, ContactNets, MIT Push) |
| Robot playback | [`robot-playback/`](robot-playback/) | recorded robot motion replayed in simulation (RoboDojo, RH20T) |

Each task folder holds the documents a run of the latest version actually uses,
and the code that runs it:

```
tasks/<category>/<task>/
  task-metrics.md                the task's measurement criteria (defined earlier in sim_protocol/)
  manifests/                     the latest manifest: conditions, candidates, seeds, gates and labels
  *_common.py                    task logic shared by the engine runners
  mujoco/ mjwarp/ sap/ genesis/  the runner for each simulator
  prep/  analysis/               builders; render, summarize and audit scripts
  assets/                        task geometry
```

Only the manifest revision used by the latest runs (Part V, 2026-11-18) is kept;
where `task-metrics.md` and the manifest differ (for example three-box push: the
metrics file describes retain/eject, manifest strict-v4 scores retain/slip/tip),
the manifest is what the runners score.

A few runners serve more than one task (for example the impact runners take
`--task ball|cube`); they are in [`common/`](../common/) and listed per task below.
Family-level protocols (which family a task belongs to and the metrics shared
by the family) are in [`protocol/families/`](../protocol/families/). Runners find
manifests by name (`common.paths.find_manifest`); results record each manifest's
name and SHA-256, so manifests are never edited in place — declare a new revision.

## Simulation only · atomic

| Task | Metrics | Manifest | Engine folders | Shared runners in [`common/`](../common/) |
| --- | --- | --- | --- | --- |
| [Three-plate stack](simulation-only-atomic/three-plate-stack/) | [task-metrics](simulation-only-atomic/three-plate-stack/task-metrics.md) | [`three_plate_stack_strict_v3p1`](simulation-only-atomic/three-plate-stack/manifests/three_plate_stack_strict_v3p1.json) | mjwarp sap genesis | — |
| [Three-box push](simulation-only-atomic/three-box-push/) | [task-metrics](simulation-only-atomic/three-box-push/task-metrics.md) | [`three_box_push_strict_v4`](simulation-only-atomic/three-box-push/manifests/three_box_push_strict_v4.json) | mujoco mjwarp sap genesis | `run_static_friction_v2.py` |
| [Simple pinch](simulation-only-atomic/simple-pinch/) | [task-metrics](simulation-only-atomic/simple-pinch/task-metrics.md) | [`simple_pinch_force_controlled_v3`](simulation-only-atomic/simple-pinch/manifests/simple_pinch_force_controlled_v3.json) | mjwarp sap genesis | `run_static_friction_v2.py` |
| [Pendulum (idealised)](simulation-only-atomic/pendulum-ideal/) | [task-metrics](simulation-only-atomic/pendulum-ideal/task-metrics.md) | [`pendulum_energy_v2`](simulation-only-atomic/pendulum-ideal/manifests/pendulum_energy_v2.json) | mujoco mjwarp genesis | — |

The GAUGE pendulum, which uses real trajectories, is under real data.

## Simulation only · composite

| Task | Metrics | Manifest | Engine folders | Shared runners in [`common/`](../common/) |
| --- | --- | --- | --- | --- |
| [Round socket (peg-in-hole)](simulation-only-composite/round-socket/) | [task-metrics](simulation-only-composite/round-socket/task-metrics.md) | [`round_socket_strict_v9_draft`](simulation-only-composite/round-socket/manifests/round_socket_strict_v9_draft.json) | mujoco mjwarp sap genesis | — |

## Real data · simple

| Task | Metrics | Manifest | Engine folders | Shared runners in [`common/`](../common/) |
| --- | --- | --- | --- | --- |
| [GAUGE ball bounce](real-data-simple/ball-bounce/) | [task-metrics](real-data-simple/ball-bounce/task-metrics.md) | [`gauge_ball_bounce_dynamic_impact_v2`](real-data-simple/ball-bounce/manifests/gauge_ball_bounce_dynamic_impact_v2.json) / [`gauge_ball_bounce_dynamic_impact_v3`](real-data-simple/ball-bounce/manifests/gauge_ball_bounce_dynamic_impact_v3.json) | mujoco mjwarp | `impact_v2_common.py`, `run_impact_v2_mjwarp.py`, `run_impact_b5_r20_selection*.py`, `run_impact_genesis_{ipc,rigid,sap}.py` (`--task ball`) |
| [ContactNets cube toss](real-data-simple/cube-toss/) | [task-metrics](real-data-simple/cube-toss/task-metrics.md) | [`contactnets_cube_toss_dynamic_impact_v2`](real-data-simple/cube-toss/manifests/contactnets_cube_toss_dynamic_impact_v2.json) | mujoco mjwarp | same impact runners as ball bounce (`--task cube`) |
| [Cube-toss resting tail](real-data-simple/cube-toss-tail/) | [task-metrics](real-data-simple/cube-toss-tail/task-metrics.md) | [`contactnets_cube_tail_sustained_v3`](real-data-simple/cube-toss-tail/manifests/contactnets_cube_tail_sustained_v3.json) | mujoco mjwarp genesis | — |
| [MIT Push](real-data-simple/mit-push/) | [task-metrics](real-data-simple/mit-push/task-metrics.md) | [`mit_push_dynamic_friction_v3`](real-data-simple/mit-push/manifests/mit_push_dynamic_friction_v3.json) | mjwarp genesis | `run_dynamic_friction_v2.py` |
| [GAUGE slope slider](real-data-simple/slope-slider/) | [task-metrics](real-data-simple/slope-slider/task-metrics.md) | [`gauge_slope_slider_dynamic_friction_v3`](real-data-simple/slope-slider/manifests/gauge_slope_slider_dynamic_friction_v3.json) | mjwarp genesis | `run_dynamic_friction_v2.py` |
| [GAUGE pendulum](real-data-simple/gauge-pendulum/) | — | [`pendulum_gauge_real_v1`](real-data-simple/gauge-pendulum/manifests/pendulum_gauge_real_v1.json) | mjwarp genesis | — |

## Robot playback

| Task | Metrics | Manifest | Engine folders | Shared runners in [`common/`](../common/) |
| --- | --- | --- | --- | --- |
| [RH20T dish on rack](robot-playback/rh20t-dish-on-rack/) | [task-metrics](robot-playback/rh20t-dish-on-rack/task-metrics.md) | [`rh20t_dish_rack_playback_v1_pilot_c3`](robot-playback/rh20t-dish-on-rack/manifests/rh20t_dish_rack_playback_v1_pilot_c3.json) | — | package `real_playback/` (all engines) |
| [RoboDojo](robot-playback/robodojo/) — stack bowls, insert tubes, deposit coin, laptop insertion / pickup, sweep blocks | [task-metrics](robot-playback/robodojo/task-metrics/) | — | — | package `robodojo_playback/` (all engines) |

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
