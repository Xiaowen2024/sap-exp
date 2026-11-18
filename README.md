# SAP/Warp × MuJoCo contact-rich simulation benchmark

This repository contains a controlled comparison of SAP/Warp and MuJoCo across
contact parameterization, friction, dynamic impact, collision representation,
and Panda manipulation tasks.

## Executive summary

There is no universal winner. In the tested configurations:

| Regime | Main observation | Practical reading |
| --- | --- | --- |
| Near-rigid contact | MuJoCo maintained stable multi-contact stacks and near-zero static force-balance error; SAP/Warp showed high-stiffness contact switching and jitter. | MuJoCo was the more consistent near-rigid reference. |
| Compliant friction | SAP/Warp stayed closer to the Coulomb limit in free sliding and reached the ideal breakaway threshold more closely in the stable ramp and pinch tests. | SAP/Warp is useful when interpretable compliant friction is the target, subject to normal-contact stability. |
| Impact and energy | MuJoCo transferred calibrated restitution more consistently across time steps and retained more energy in the elastic-bounce test. | MuJoCo was the safer dynamic baseline in these tests. |
| Geometry and manipulation | Broad concave settling was similar, but tight insertion and Panda task outcomes exposed different collision and controller–contact failure modes. | Collision representation and task configuration can dominate the apparent ranking. |

All statements above refer to the tested configurations, not engine-best
performance. The comparison does not treat MuJoCo's `solref`/`solimp` and
SAP/Warp's `ke`/`tau` as one-to-one physical equivalents; parameters are
interpreted through measured force, penetration, slip, energy, stability, and
task outcomes.

## Full report

The complete advisor-facing synthesis is a single-page, documentation-style
report with numbered sections, tables, and source links:

**[Open the full GitHub Pages report →](https://xiaowen2024.github.io/sap-exp/)**

The report follows the progression:

1. Executive summary
2. Friction
3. Dynamic contact
4. Collision representation and contact geometry
5. Panda manipulation
6. Overall comparison and contact parameter interpretation
7. Implications for robotics
8. Final conclusion
9. Sources

## Repository structure

- [`tasks/`](tasks/) — the current task set in four categories: simulation only · atomic,
  simulation only · composite (round socket), real data · simple, and robot playback.
  Each task folder holds its measurement criteria (`task-metrics.md`), the latest manifest, assets
  and one runner folder per simulator (`mujoco/`, `mjwarp/`, `sap/`, `genesis/`);
  see the [task index](tasks/README.md)
- [`common/`](common/) — code shared by several tasks: [`paths.py`](common/paths.py) (which every
  runner imports for `REPO_ROOT`, `find_manifest` and the shared-module import path), the
  Genesis adapters, and runners that serve more than one task
- [`protocol/`](protocol/) — cross-task SOPs, [family protocols](protocol/families/),
  Genesis [runner SOPs](protocol/runner-sops/) and rerun [registries](protocol/registries/)
- [`tools/`](tools/) — cross-task reruns and queues, comparisons, report generation,
  validation and maintenance
- [`docs/`](docs/) — source files for the full report website
- [`exp-report/`](exp-report/) — experiment-level reports and interpretations
- [`exp-results/`](exp-results/) — raw CSV/JSON traces, plots, videos, and scene files
- [`.github/workflows/pages.yml`](.github/workflows/pages.yml) — GitHub Pages deployment workflow

Scripts formerly under `scripts/`, `sim_protocol/` and `assets/` were moved on
2026-11-18; [`MOVED_PATHS.md`](MOVED_PATHS.md) maps each old path to its new one.
Scripts can be run from any directory, e.g.
`python tasks/simulation-only-atomic/simple-pinch/mjwarp/run_pinch_v3_mjwarp.py --help`;
`python tools/check_layout.py` checks the layout after adding or moving scripts.

## Comparison principles

The benchmark distinguishes three levels of evidence:

- **Observable-matched comparisons**, such as aggregate stiffness or contact time scale;
- **Native-configuration behavior**, where each simulator uses its own contact and solver controls; and
- **Task-level robustness**, where the controller, collision geometry, time step, and contact model interact.

Runtime is not interpreted from CPU MuJoCo versus CUDA SAP/Warp wall time. A
fair scalability comparison requires matched hardware, world count, contact
capacity, and measurement scope.
