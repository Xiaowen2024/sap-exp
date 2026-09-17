# MuJoCo vs SAP: analytic square-cavity insertion

## Common task

This is the analytic square-cavity control task, not the unreliable circular
mesh socket. The socket consists of four fixed box walls and a bottom. The free
peg is a 20 x 20 x 50 mm box; cavity inner half-width is 12 mm, giving 2 mm
single-side clearance. It falls under gravity from 120 mm for 2 s with
`dt=0.5 ms`; its mass is set from density 1000 kg/m3 in both models. Two initial
horizontal offsets are tested: 0 and 1 mm.

MuJoCo uses its native `Newton` solver, `implicitfast` integrator, 100 solver
iterations, `solimp="0.9 0.95 0.001 0.5 2"`, and `solref="timeconst 1"`.
It records the maximum of `max(0, -contact.dist)` over peg--socket contacts;
this is the **unmargined geometric signed-distance overlap** that MuJoCo
reports. The 1 ms time constant is the smallest tested value and is also the
practical lower bound of `2*dt` for positive-format `solref`.

## MuJoCo results

| offset | solref timeconst | entered cavity | max geometric penetration (mm) | final geometric penetration (mm) | final center [x,y,z] (mm) | max contacts | CPU stepping time (s) |
|---:|---:|---|---:|---:|---|---:|---:|
| 0 mm | 1 ms | yes | 0.14099 | 0.00014 | [0.000, 0.000, 35.000] | 4 | 0.0332 |
| 0 mm | 2 ms | yes | 0.57975 | 0.00057 | [0.000, 0.000, 34.999] | 4 | 0.0323 |
| 0 mm | 4 ms | yes | 1.49676 | 0.00229 | [0.000, 0.000, 34.998] | 4 | 0.0324 |
| 0 mm | 10 ms | yes | 4.24215 | 0.01434 | [0.000, 0.000, 34.986] | 4 | 0.0320 |
| 1 mm | 1 ms | yes | 0.14099 | 0.00014 | [1.000, 0.000, 35.000] | 4 | 0.0314 |
| 1 mm | 2 ms | yes | 0.57975 | 0.00057 | [1.000, 0.000, 34.999] | 4 | 0.0321 |
| 1 mm | 4 ms | yes | 1.49676 | 0.00229 | [1.000, 0.000, 34.998] | 4 | 0.0320 |
| 1 mm | 10 ms | yes | 4.24215 | 0.01434 | [1.000, 0.000, 34.986] | 4 | 0.0322 |

The whole raw output is in
[`mujoco-square-cavity-diagnostics/results.csv`](../exp-results/peg-in-hole/mujoco-square-cavity-diagnostics/results.csv),
and the reproducible runner is
[`run_square_cavity_comparison.py`](../mujoco-sim/learn_mujoco/peg_in_hole_experiments/run_square_cavity_comparison.py).

## Same raw-distance diagnostic in SAP

I additionally reran SAP's centered cases and reconstructed the unadjusted
collision-pipeline witness separation as
`dot(world(p1)-world(p0), normal)` before each solve. For these analytic
box--box contacts, it agrees with SAP's recorded `phi0` to floating-point noise
(less than `1e-7 mm`): the two shapes have zero shape margin, and the
`rigid_gap` is a detection/activation distance rather than a subtraction from
this reported witness separation.

| SAP centered case | max raw witness overlap (mm) | max phi0 overlap (mm) | difference (mm) |
|---|---:|---:|---:|
| `ke=1e3`, `tau=10 ms` | 0.09802732 | 0.09802729 | 0.00000003 |
| `ke=1e4`, `tau=10 ms` | 0.00973419 | 0.00973418 | 0.00000000 |
| `ke=1e5`, `tau=10 ms` | 0.00162423 | 0.00162423 | 0.00000000 |

Raw SAP measurements are in
[`sap-square-cavity-raw-gap-gpu/results.csv`](../exp-results/peg-in-hole/sap-square-cavity-raw-gap-gpu/results.csv).
This removes the earlier ambiguity for the *centered analytic box case*: SAP's
reported peak values here are not merely a margin-shifted `phi0` artifact.

## Comparison with the existing SAP results

Qualitatively, MuJoCo is stable in both offset cases: a 1 mm offset is still
inside the 2 mm side clearance, so it should reach the bottom without touching a
side wall. SAP's initial `rigid_gap=1 mm` cases escaped and fell through. The
targeted control sweep below shows that this is not an unavoidable SAP
ke/tau-contact failure: it is sensitive to the SAP contact activation gap.

The reported penetration magnitudes are now comparable as each engine's raw
contact-pair signed overlap for this analytic box task, but they are **still not
a one-number general accuracy ranking**:

- MuJoCo's `contact.dist` is raw geometric signed distance.
- SAP's raw witness separation equals `phi0` for the *tested analytic box--box
  contacts*, as verified above. This equality does not automatically extend to
  concave mesh--mesh SDF contacts, where the witness itself can be invalid.
- The engines still use different time-stepping/contact formulations, and their
  contact values are sampled at their respective solve stages. A lower peak in
  one configuration is task evidence, not a universal solver ranking.

For example, under the SAP `ke=1e4, tau=10 ms, rigid_gap=1 mm` baseline the
centered case has peak raw overlap `0.00973 mm`; the MuJoCo 1 ms case has
`0.14099 mm`. Thus SAP has the lower measured peak in these particular native
configurations. That does **not** prove SAP has lower true geometric overlap for
all tasks, because it remains a comparison of differently parameterized native
contact formulations.

What is already defensible is:

1. **MuJoCo's peak raw overlap increases sharply as its contact is softened**
   (`solref` 1 to 10 ms: 0.141 to 4.242 mm), although all cases settle with
   small final overlap.
2. **With its initial 1 mm `rigid_gap`, SAP does not preserve the expected
   offset insertion; with a calibrated smaller gap it does.** This is a
   configuration sensitivity, not evidence that SAP categorically fails the
   physical task.

## SAP `rigid_gap` control: 1 mm offset

All cases here use SAP GPU, `ke=1e4 N/m`, `tau=10 ms`, the Drake preset and the
same task. Only `rigid_gap` changes. It is an activation/detection gap: it
controls how far before physical overlap the collision pipeline admits contact
candidates; it is not subtracted from the raw witness distance reported above.

| rigid_gap | entered cavity | max raw overlap (mm) | final active phi0 overlap (mm) | final center [x,y,z] (mm) | max contacts | outcome |
|---:|---|---:|---:|---|---:|---|
| 0 mm | yes | 0.29298 | 0.10020 | [1.000, 0.000, 35.002] | 4 | stable, but more compliant final contact |
| 0.1 mm | yes | 0.29298 | 0.00988 | [1.000, 0.000, 35.090] | 4 | stable |
| 0.5 mm | yes | 0.00973 | 0.00973 | [1.000, -0.000, 35.090] | 4 | stable; lowest tested peak |
| 1.0 mm | no | 0 | 0 | [-70.161, 5.979, -13465.216] | 12 | escaped / fell through |

The raw control data are in
[`sap-square-cavity-gap-diagnostics/results.csv`](../exp-results/peg-in-hole/sap-square-cavity-gap-diagnostics/results.csv).
Thus `rigid_gap=1 mm` is too large for this geometry: at a 1 mm initial offset,
the remaining lateral clearance is also 1 mm, so broad-phase/narrow-phase
candidate contacts can be introduced near the side wall before physical
interference. `0.1--0.5 mm` retains stable insertion. This resolves the
previous apparent contradiction with MuJoCo: the fair SAP comparison should use
a gap smaller than the minimum intended geometric clearance, not an arbitrary
1 mm activation distance.
