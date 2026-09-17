# MuJoCo thin-discs contact experiments

## What was tested

These are existing MuJoCo-only diagnostics in
[`thin_disc_experiments`](../mujoco-sim/learn_mujoco/thin_disc_experiments/).
They use finite cylinders rather than planes, because a MuJoCo plane is
infinite, zero-thickness and static and therefore cannot be stacked.

- **Three-disc stack:** three free cylinders, each radius 101 mm, total
  thickness 4 mm and mass 0.333 kg. Initial centers are 30, 70 and 110 mm;
  small lateral offsets prevent a perfectly symmetric contact. Gravity is
  9.81 m/s²; `dt=0.5 ms`, `implicitfast`, Newton with 100 iterations unless
  stated otherwise.
- **Ball-on-disc impact:** the same 4 mm disc is dropped onto the table, while
  a 40 g, 20 mm-radius ball falls from 130 mm onto the disc. It isolates the
  impact/contact-compliance part of the problem from stack ordering.

All penetration values below are MuJoCo's raw `max(0, -contact.dist)` in mm;
they are not visual overlap estimates.

## Three-disc stack: baseline failure and contact tuning

The original baseline (`solref=8 ms`, standard `solimp`) is a useful negative
control. It finishes with the top and middle discs in the wrong vertical order,
including non-adjacent bottom--top contact. It is therefore not a credible
stack, even though it remains numerically finite.

| configuration | dt | solref | solimp | order preserved | largest peak penetration | largest final penetration | observation |
|---|---:|---:|---|---|---:|---:|---|
| baseline | 0.5 ms | 8 ms | `0.95 0.99 0.002 0.5 2` | no | 4.014 mm | 0.034 mm | discs reorder; invalid stack |
| solref 1 ms | 0.5 ms | 1 ms | standard | yes | 0.652 mm | 0.0011 mm | lowest peak among time-constant sweep |
| solref 2 ms | 0.5 ms | 2 ms | standard | yes | 1.063 mm | 0.0021 mm | stable, larger transient overlap |
| solref 4 ms | 0.5 ms | 4 ms | standard | yes | 2.061 mm | 0.0086 mm | stable, more compliant |
| tighter solimp | 0.5 ms | 2 ms | `0.99 0.999 0.0005 0.5 2` | yes | 1.056 mm | 0.00040 mm | lower resting overlap, similar impact peak |
| hard/fine | 0.25 ms | 1 ms | `0.99 0.999 0.0005 0.5 2` | yes | 0.355 mm | 0.00128 mm | lowest reported peak, twice the step rate |

“Largest” is the maximum over all disc--table and disc--disc pairs. The
configuration-specific source JSON files are in
[`outputs/`](../mujoco-sim/learn_mujoco/thin_disc_experiments/outputs/):
`thin_disc_stack_*_summary_20260829T084846Z.json` and
`thin_disc_stack_hard_fine_summary_20260829T084847Z.json`.

### Interpretation

1. This test is not just a penetration test: the 8 ms baseline reaches a
   visibly/physically wrong ordering. A small final contact distance alone
   would not catch that failure.
2. Reducing MuJoCo's positive-format `solref` time constant stiffens the
   contact response and lowers peak transient penetration in this fixed-
   timestep stack.
3. Tightening `solimp` principally improves the resting-contact overlap here;
   reducing both timestep and time constant improves the impact peak as well.
4. The reported tuning is task-specific. It does not establish a universal
   “best MuJoCo contact” setting, because it trades stiffness, timestep and
   computational cost.

## Ball-on-disc impact sensitivity

All these runs use `dt=0.5 ms`, standard `solimp`, and measure the ball--disc
pair. The impact happens at about 0.148--0.155 s.

| contact parameterization | peak penetration | final penetration | peak time |
|---|---:|---:|---:|
| positive `solref=1 ms, damping=1` | 0.3525 mm | 0.00060 mm | 0.1475 s |
| positive `solref=2 ms, damping=1` | 0.8743 mm | 0.00239 mm | 0.1490 s |
| positive `solref=4 ms, damping=1` | 1.9380 mm | 0.00955 mm | 0.1510 s |
| positive `solref=8 ms, damping=1` (baseline) | 4.0482 mm | 0.03816 mm | 0.1550 s |
| direct `solref=[-1e5, -632.456]` | 1.4965 mm | 0.00597 mm | 0.1500 s |
| direct `solref=[-5e5, -1414.214]` | 0.5451 mm | 0.00119 mm | 0.1480 s |
| direct `solref=[-1e6, -2000]` | 0.3525 mm | 0.00060 mm | 0.1475 s |

The negative `solref` form supplies direct stiffness/damping coefficients,
whereas the positive form is time-constant/damping-ratio parameterization.
The paired values above produce the expected trend: stiffer/faster contact
reduces peak overlap at this timestep.

## Reproduction

```sh
cd /Users/xiaowenyuan/Documents/SIM-EXP/mujoco-sim/learn_mujoco/thin_disc_experiments
../../.venv/bin/python run_thin_disc_stack.py --model thin_disc_stack_solref_1ms.xml --duration 2
../../.venv/bin/python run_thin_disc_ball_drop.py --model thin_disc_ball_drop_solref_1ms.xml
```

The scripts preserve per-contact peak penetration, its time, final contact
penetration, final pose and stack order in timestamped JSON outputs. Existing
visualizations can be reproduced with `render_thin_disc_stack.py` and
`render_ball_drop_peak_frame.py`.
