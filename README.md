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
2. Main findings
3. Friction
4. Dynamic contact
5. Collision representation and contact geometry
6. Panda manipulation
7. Overall comparison
8. Implications for robotics
9. Final conclusion
10. Sources

## Repository structure

- [`docs/`](docs/) — source files for the full report website
- [`exp-report/`](exp-report/) — experiment-level reports and interpretations
- [`exp-results/`](exp-results/) — raw CSV/JSON traces, plots, videos, and scene files
- [`.github/workflows/pages.yml`](.github/workflows/pages.yml) — GitHub Pages deployment workflow

## Comparison principles

The benchmark distinguishes three levels of evidence:

- **Observable-matched comparisons**, such as aggregate stiffness or contact time scale;
- **Native-configuration behavior**, where each simulator uses its own contact and solver controls; and
- **Task-level robustness**, where the controller, collision geometry, time step, and contact model interact.

Runtime is not interpreted from CPU MuJoCo versus CUDA SAP/Warp wall time. A
fair scalability comparison requires matched hardware, world count, contact
capacity, and measurement scope.
