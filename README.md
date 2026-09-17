# SAP/Warp × MuJoCo contact-dynamics benchmark

This repository contains a controlled comparison of SAP/Warp and MuJoCo across
normal-contact calibration, stacking, friction, impact, rolling, collision
representation, and Panda manipulation tasks.

## Report website

The advisor-facing synthesis is available at:

**[Open the GitHub Pages report](https://xiaowen2024.github.io/sap-exp/)**

The site is generated from [docs/](docs/). The complete experiment reports,
raw CSV/JSON traces, scene files, and runners remain under
[exp-report/](exp-report/) and [exp-results/](exp-results/).

## Scope

The comparison does not claim a universal winner. It distinguishes:

- native-configuration behavior;
- observable-matched comparisons, such as measured normal force or contact
  timescale; and
- task-level robustness, where controllers and contact models interact.

In particular, MuJoCo and SAP/Warp parameters are not treated as
one-to-one physical equivalents: solref/solimp and ke/tau must be interpreted
through measured penetration, force, slip, energy, stability, and task
outcomes.
