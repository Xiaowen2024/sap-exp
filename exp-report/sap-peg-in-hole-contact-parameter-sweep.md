# SAP peg-in-hole contact-parameter sweep

## Scope and measurement

This is a SAP Warp GPU experiment (`contact_preset_variant: drake`) with a free
peg dropped under gravity for 2 s at `dt = 0.5 ms`.  Both the wall and peg use
the same material, `mu = 0.8`, `rigid_gap = 1 mm`, and an independent capacity
of 2048 contacts; no run approached that capacity.

The initial square cavity has inner half-width 12 mm and the square peg has
half-width 10 mm: the nominal *single-side* clearance is 2 mm.  Two initial
horizontal offsets are tested: centered and 1 mm.  The square cavity uses five
analytic boxes (four walls plus bottom), so its box--box contact gaps are
meaningful.  This deliberately excludes the preliminary concave raw
mesh--mesh socket result, whose SDF/witness contact distance became unreliable
inside the cavity.

For every solver step I record SAP's contact-Jacobian normal gap `phi0` for
every active contact:

\[
  p_{\max}=\max_{t,i}\max(0,-\phi_{0,i}(t)).
\]

Thus positive `phi0` is separation and negative `phi0` is SAP's local
constraint penetration. `max_phi0_penetration_mm` below is the maximum of that
quantity over the entire trajectory. It is **not** a post-hoc mesh-distance
calculation.

`final_phi0_penetration_mm` is the final active contact's constraint
penetration. If a body has already fallen through and has no final contact, it
is zero; that means *no active contact*, not a good result. Therefore credibility
requires both low penetration **and** a sensible final state.

The `ke`--`tau` material parameters are SAP's normal stiffness and relaxation
time. For identical materials the familiar Kelvin--Voigt comparison quantity
is `kd = ke * tau`; that relation is reported only for interpretation, not as
an independently tuned input here.

## Results: analytic square cavity

Baseline for the `ke` sweep is `tau = 10 ms`; baseline for the `tau` sweep is
`ke = 1e4 N/m`. Wall time is GPU stepping time, excluding one-time process
startup but including all 4000 simulation steps.

| initial offset | varied setting | ke (N/m) | tau (ms) | implied kd (N s/m) | entered cavity | max phi0 penetration (mm) | final active penetration (mm) | final peg center [x,y,z] (mm) | max contacts | GPU step time (s) | assessment |
|---|---:|---:|---:|---:|---|---:|---:|---|---:|---:|---|
| 0 mm | ke=1e3 | 1e3 | 10 | 10 | yes | 0.09803 | 0.09803 | [0.000, 0.000, 35.002] | 4 | 17.254 | stable; compliant contact |
| 0 mm | ke=1e4 | 1e4 | 10 | 100 | yes | 0.00973 | 0.00973 | [0.000, 0.000, 35.090] | 4 | 16.841 | stable |
| 0 mm | ke=1e5 | 1e5 | 10 | 1000 | yes | 0.00162 | 0.00162 | [0.000, 0.000, 35.098] | 4 | 16.573 | stable; lowest tested penetration |
| 0 mm | tau=1 ms | 1e4 | 1 | 10 | yes | 0.04101 | 0.00981 | [0.000, 0.000, 35.090] | 4 | 15.670 | stable; larger transient penetration |
| 0 mm | tau=3 ms | 1e4 | 3 | 30 | yes | 0.00979 | 0.00979 | [0.000, 0.000, 35.090] | 4 | 16.610 | stable |
| 0 mm | tau=30 ms | 1e4 | 30 | 300 | yes | 0.00959 | 0.00959 | [0.000, 0.000, 35.090] | 4 | 23.274 | stable; slower |
| 1 mm | ke=1e3 | 1e3 | 10 | 10 | no | 0.08240 | 0 | [-130.625, -1.344, -13521.769] | 12 | 26.439 | failed: escaped/fell through |
| 1 mm | ke=1e4 | 1e4 | 10 | 100 | no | 0 | 0 | [-70.161, 5.979, -13465.216] | 12 | 23.096 | failed: escaped/fell through |
| 1 mm | ke=1e5 | 1e5 | 10 | 1000 | no | 0 | 0 | [-202.976, 6.558, -13732.304] | 9 | 25.821 | failed: escaped/fell through |
| 1 mm | tau=1 ms | 1e4 | 1 | 10 | no | 0.09255 | 0 | [-675.674, -0.244, -15363.944] | 9 | 21.567 | failed: escaped/fell through |
| 1 mm | tau=3 ms | 1e4 | 3 | 30 | no | 0.02047 | 0 | [-577.096, 1.414, -15143.630] | 11 | 23.596 | failed: escaped/fell through |
| 1 mm | tau=30 ms | 1e4 | 30 | 300 | no | 0 | 0 | [-596.024, -25.487, -14281.032] | 8 | 25.982 | failed: escaped/fell through |

Raw machine-readable measurements and each exact scene configuration are in
[`results.csv`](../exp-results/peg-in-hole/sap-cavity-diagnostics/results.csv)
and [`exp-results/peg-in-hole/sap-cavity-diagnostics/`](../exp-results/peg-in-hole/sap-cavity-diagnostics/).

## What the sweep does and does not show

1. **For the symmetric, four-contact settling case, stiffness directly changes
   the contact error.** At fixed 10 ms `tau`, increasing `ke` from `1e3` to
   `1e5 N/m` reduces measured maximum penetration from 0.0980 mm to 0.00162 mm
   (about 60x), while preserving the same expected resting height and four
   contacts. Relative to the 2 mm clearance, these are 4.9%, 0.49%, and 0.081%.

2. **`tau` mainly changes transients and cost in this centered setting.** At
   fixed `ke=1e4`, `tau=1 ms` produces a 0.0410 mm peak despite reaching the
   same final state; `tau >= 3 ms` gives about 0.0096--0.0098 mm. `tau=30 ms`
   costs 23.27 s versus 15.67--16.84 s for 1--10 ms. These are empirical
   results for this exact `dt`, preset, and geometry; they are not a universal
   monotonic tuning law.

3. **Low reported penetration is not alone a credibility metric.** All six
   1 mm-offset square cases eventually have a near-zero *final active*
   penetration because they lose contact, escape laterally, and fall below the
   bottom (final `z` around -13 to -15 m). Three even show `p_max=0`: that
   means the sampled constraints never observed a negative gap, not that the
   insertion was correct. The final state rejects every one of these cases.

4. **This isolates two distinct failure modes.** The centered analytic cavity
   has well-defined collision primitives and can be made locally accurate with
   `ke/tau`. The offset cavity introduces simultaneous side/bottom contact and
   fails for all tested material values. The earlier raw concave mesh and the
   32-piece convex decomposition also fail even when centered; their small
   `phi0` values cannot validate the trajectory because the collision
   representation itself is unsuitable.

## Credibility rule used here

There is no universal published threshold such as “10% penetration means valid.”
For this diagnostic, a run is called *locally credible* only when all of the
following hold:

- contact geometry is analytic/known-valid rather than the failed raw concave
  SDF representation;
- no contact-capacity truncation occurs;
- final pose is inside the cavity and physically plausible;
- the maximum `phi0` penetration is small relative to the 2 mm clearance.

Under that rule, all six centered analytic-square runs are locally credible
(with different compliance); none of the 1 mm-offset runs is. This is evidence
about SAP Warp's behavior on this insertion contact topology, not proof that
SAP is intrinsically worse than MuJoCo for every insertion task.

## Round-hole representation controls

The following six cases use the same material point as the nominal centered
square baseline: `ke=1e4 N/m`, `tau=10 ms`, `kd=100 N s/m`, `dt=0.5 ms`, 2 s
duration and 2 mm radial/single-side clearance. Unlike the square cases, these
are *not* parameter-sweep points; they test whether a circular concave socket
has a trustworthy collision representation.

| collision representation | initial offset | entered cavity | max phi0 penetration (mm) | final active penetration (mm) | max contacts | final peg center [x,y,z] (mm) | GPU step time (s) | result / credibility |
|---|---:|---|---:|---:|---:|---|---:|---|
| original non-watertight circular socket mesh | 0 mm | yes | 11.98520 | 0.00144 | 63 | [0.005, 0.003, 36.999] | 37.293 | invalid: deep internal SDF/witness overlap |
| original non-watertight circular socket mesh | 1 mm | no | 0.00839 | 0.00785 | 22 | [-23.039, -3.030, 49.972] | 33.905 | invalid: lateral ejection |
| repaired watertight circular socket mesh | 0 mm | yes | 11.98316 | 0.00141 | 64 | [0.004, 0.003, 36.999] | 40.545 | invalid: same deep internal overlap |
| repaired watertight circular socket mesh | 1 mm | no | 0.00399 | 0.00353 | 21 | [-23.192, 1.213, 49.975] | 40.807 | invalid: lateral ejection |
| 32-piece CoACD convex decomposition of circular socket | 0 mm | no | 0.01904 | 0.01557 | 14 | [-8.886, 1.144, 59.470] | 58.667 | invalid: ejected upward/laterally |
| 32-piece CoACD convex decomposition of circular socket | 1 mm | no | 0.06580 | 0.01153 | 8 | [-25.982, -5.677, 50.593] | 43.263 | invalid: ejected upward/laterally |

Raw machine-readable results for the original and repaired mesh are in
[`mesh-representation-diagnostics/results.csv`](../exp-results/peg-in-hole/sap-mesh-representation-diagnostics/results.csv).
The convex cases are included in the square diagnostic
[`results.csv`](../exp-results/peg-in-hole/sap-cavity-diagnostics/results.csv).

### Interpretation

- **Watertight repair did not fix the circular cavity.** The repair removes a
  detached two-face, zero-volume component; the remaining 510-face component
  is watertight and has consistent winding. Yet its centered maximum `phi0`
  remains 11.983 mm versus 11.985 mm for the original. The trajectories are
  also nearly identical. The defect is therefore not simply a hole in the OBJ.
- **The centered 12 mm number is a warning, not a measurement of physical
  peg--wall interpenetration.** A centered peg belongs in the circular empty
  cavity. Since the mesh--mesh SDF/witness pipeline reports a near-cavity-scale
  overlap while it is visually/internal-pose centered, its contact witness no
  longer represents the desired inner-wall separation. Calling the case a
  numerical “success” because `entered_cavity=true` would be misleading.
- **Small `phi0` is also insufficient.** In both 1 mm-offset mesh cases, the
  recorded maximum is only 0.004--0.008 mm, but the peg is propelled roughly
  23 mm laterally outside the socket. Hence the low number records the few
  contacts that SAP happened to form before escape, not an accurate insertion.
- **No case was contact-capacity limited.** The highest observed count is 64,
  far below the 2048 contact capacity. The failure is geometry/contact topology
  behavior, not contact truncation.

These controls reinforce why the analytic-square result should be interpreted
as a clean contact-solver diagnostic, rather than evidence that the original
raw circular mesh socket was correctly simulated.
