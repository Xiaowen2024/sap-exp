# Tasks

Everything is organized by task. Tasks are grouped by what they are compared
against, following the latest task set (Part V, 2026-11-18):

| Category | Folder | Reference |
| --- | --- | --- |
| Simulation only · atomic | [`simulation-only-atomic/`](simulation-only-atomic/) | analytic or protocol-defined expectations, one contact mechanism per task |
| Simulation only · composite | [`simulation-only-composite/`](simulation-only-composite/) | analytic statics; several mechanisms coupled (round socket only) |
| Real data · simple | [`real-data-simple/`](real-data-simple/) | released real trajectories (GAUGE, ContactNets, MIT Push) |
| Robot playback | [`robot-playback/`](robot-playback/) | recorded robot motion replayed in simulation (RoboDojo, RH20T) |

Each task folder holds what is needed to run and score it:

```
tasks/<category>/<task>/
  SOP.md                         how the task is measured and judged (metrics, pass/fail rules)
  manifests/                     the declared conditions, candidates, seeds and gates for each revision
  *_common.py                    task logic shared by the engine runners
  mujoco/ mjwarp/ sap/ genesis/  the runner for each simulator
  prep/  analysis/               builders; render, summarize and audit scripts
  assets/                        task geometry
```

A few runners serve more than one task (for example the impact runners take
`--task ball|cube`); they are in [`common/`](../common/) and listed per task below.
Family-level protocols (which family a task belongs to and the metrics shared
by the family) are in [`protocol/families/`](../protocol/families/). Runners find
manifests by name (`common.paths.find_manifest`); results record each manifest's
name and SHA-256, so manifests are never edited in place — declare a new revision.

## Simulation only · atomic

| Task | SOP | Manifests | Engine folders | Shared runners in [`common/`](../common/) |
| --- | --- | --- | --- | --- |
| [Three-plate stack](simulation-only-atomic/three-plate-stack/) | [SOP](simulation-only-atomic/three-plate-stack/SOP.md) | [3](simulation-only-atomic/three-plate-stack/manifests/) | mjwarp sap genesis | — |
| [Three-box push](simulation-only-atomic/three-box-push/) | [SOP](simulation-only-atomic/three-box-push/SOP.md) | [3](simulation-only-atomic/three-box-push/manifests/) | mujoco mjwarp sap genesis | `run_static_friction_v2.py` |
| [Simple pinch](simulation-only-atomic/simple-pinch/) | [SOP](simulation-only-atomic/simple-pinch/SOP.md) | [3](simulation-only-atomic/simple-pinch/manifests/) | mjwarp sap genesis | `run_static_friction_v2.py` |
| [Pendulum (idealised)](simulation-only-atomic/pendulum-ideal/) | [SOP](simulation-only-atomic/pendulum-ideal/SOP.md) | [1](simulation-only-atomic/pendulum-ideal/manifests/) | mujoco mjwarp genesis | — |

The GAUGE pendulum, which uses real trajectories, is under real data.

## Simulation only · composite

| Task | SOP | Manifests | Engine folders | Shared runners in [`common/`](../common/) |
| --- | --- | --- | --- | --- |
| [Round socket (peg-in-hole)](simulation-only-composite/round-socket/) | [SOP](simulation-only-composite/round-socket/SOP.md) | [12](simulation-only-composite/round-socket/manifests/) | mujoco mjwarp sap genesis | — |

## Real data · simple

| Task | SOP | Manifests | Engine folders | Shared runners in [`common/`](../common/) |
| --- | --- | --- | --- | --- |
| [GAUGE ball bounce](real-data-simple/ball-bounce/) | [SOP](real-data-simple/ball-bounce/SOP.md) | [2](real-data-simple/ball-bounce/manifests/) | mujoco mjwarp | `impact_v2_common.py`, `run_impact_v2_mjwarp.py`, `run_impact_b5_r20_selection*.py`, `run_impact_genesis_{ipc,rigid,sap}.py` (`--task ball`) |
| [ContactNets cube toss](real-data-simple/cube-toss/) | [SOP](real-data-simple/cube-toss/SOP.md) | [1](real-data-simple/cube-toss/manifests/) | mujoco mjwarp | same impact runners as ball bounce (`--task cube`) |
| [Cube-toss resting tail](real-data-simple/cube-toss-tail/) | [SOP](real-data-simple/cube-toss-tail/SOP.md) | [2](real-data-simple/cube-toss-tail/manifests/) | mujoco mjwarp genesis | — |
| [MIT Push](real-data-simple/mit-push/) | [SOP](real-data-simple/mit-push/SOP.md) | [1](real-data-simple/mit-push/manifests/) | mjwarp genesis | `run_dynamic_friction_v2.py` |
| [GAUGE slope slider](real-data-simple/slope-slider/) | [SOP](real-data-simple/slope-slider/SOP.md) | [1](real-data-simple/slope-slider/manifests/) | mjwarp genesis | `run_dynamic_friction_v2.py` |
| [GAUGE pendulum](real-data-simple/gauge-pendulum/) | — | [2](real-data-simple/gauge-pendulum/manifests/) | mjwarp genesis | — |

## Robot playback

| Task | SOP | Manifests | Engine folders | Shared runners in [`common/`](../common/) |
| --- | --- | --- | --- | --- |
| [RH20T dish on rack](robot-playback/rh20t-dish-on-rack/) | [SOP](robot-playback/rh20t-dish-on-rack/SOP.md) | [2](robot-playback/rh20t-dish-on-rack/manifests/) | — | package `real_playback/` (all engines) |
| [RoboDojo](robot-playback/robodojo/) — stack bowls, insert tubes, deposit coin, laptop insertion / pickup, sweep blocks | [SOPs](robot-playback/robodojo/sop/) | — | — | package `robodojo_playback/` (all engines) |

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
