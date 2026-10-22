# SAP/Warp thin-disc native-contact sweep

## Scope

Three free analytic discs (202-mm diameter, 4-mm thickness) settle under
gravity for 2 s. This report is a within-SAP/Warp native-parameter study. It
does not assign a physical stiffness equivalence to MuJoCo `solref` or rank
the two engines.

All primary runs use the `drake` preset, shape `tau=3 ms`, `rigid_gap=.1 mm`,
and sweep shape `ke` and timestep. For equal shape materials, SAP combines
the shape parameters into pair parameters; therefore the labels below are
shape inputs, not directly universal pair-material constants.

## Metrics and pass criterion

`peak overlap` is maximum native `-phi0`; `COM error` is relative to the
flat-stack reference; tail jitter is peak-to-peak COM motion over the last
0.5 s. A run fails if it escapes or loses the intended layer order. A
correct-order run with a 2-s settling time was still moving when the trace
ended.

Raw CSVs:

- [primary `dt × ke`](../exp-results/thin-discs/sap-comprehensive-sweep/results_primary.csv)
- [`tau` sweep](../exp-results/thin-discs/sap-comprehensive-sweep/results_tau.csv)
- [`rigid_gap` sweep](../exp-results/thin-discs/sap-comprehensive-sweep/results_gap.csv)
- [preset ablation](../exp-results/thin-discs/sap-comprehensive-sweep/results_preset.csv)

## What each control changed

### `ke × dt`: stiffness and discretization are coupled

- At `ke=1e3 N/m`, every primary-grid point fails through wrong order or
  escape. `3e3 N/m` is also unstable at several timesteps.
- At `1e4 N/m`, the stack often keeps its order, but has 0.7–0.8 mm COM bias
  and usually does not settle by 2 s.
- At `1e5 N/m`, `dt=.25–.5 ms` settles in approximately 0.17 s with about
  0.36 mm peak overlap. At `dt=1–2 ms`, the order remains correct in this
  sweep but tail motion becomes larger; at 5 ms, it remains ordered but is
  still moving at 2 s.
- Increasing `ke` at fine timestep reduces peak overlap, but it does not
  monotonically minimize final COM error because the `rigid_gap` activation
  geometry is also present. Coarse steps dominate at 2–5 ms: peak overlap is
  roughly 1.1 mm at 2 ms and 3.8 mm at 5 ms even at high `ke`.

Thus the useful tested operating region is not “maximum `ke`”; it is a
joint `ke`/`dt` selection.

### `tau`: trajectory damping, not an independent stiffness knob

At `dt=.5 ms`, `rigid_gap=.1 mm`:

| shape `ke` (N/m) | `tau` (ms) | peak overlap (mm) | maximum tilt (deg) | settling (s) |
|---:|---:|---:|---:|---:|
| `1e4` | 0.5 → 30 | 3.458 → 0.991 | 0.807 → 0.054 | 2.0 → 0.463 |
| `1e5` | 0.5 → 30 | 0.800 → 0.170 | 0.576 → 0.006 | 2.0 → 0.204 |

Because the runner uses `kd=ke·tau`, increasing `tau` adds normal relaxation
/ damping. In this particular drop-and-settle trajectory it reduces peak
overlap and tilt. This is not a universal larger-`tau` recommendation: it
changes the transient contact model and could hurt a different task.

### `rigid_gap`: zero was the unstable outlier in this task

With `ke=1e5 N/m`, `tau=3 ms`, and `dt=.5 ms`, `rigid_gap=0` gives
0.887-mm peak overlap, 0.060-mm final COM error, and does not settle within
2 s. The tested 0.05, 0.1, and 0.5-mm settings all settle in 0.17 s with
about 0.33–0.36 mm peak overlap. This is task-local activation behavior; it
does not imply that a positive gap is always preferable.

### Preset: Drake was the lower-overlap sampled option

At `dt=.5 ms`, `tau=3 ms`, and `rigid_gap=.1 mm`, `drake` has 0.356 mm
peak overlap at `ke=1e5`, compared with 0.969 mm for `approx32/64`; all
three settle and preserve order. At `ke=1e6`, the same comparison is 0.166
mm versus 0.841 mm. The sampled presets are a solver/discretization choice,
not interchangeable physical materials.

## Reproduction

- Runner: [`run_thin_disc_stack_sap_sweep.py`](../sap-sim/scripts/run_thin_disc_stack_sap_sweep.py)
- Site summary figure: [`thin-disc-sap-sweep.svg`](../docs/assets/thin-disc-sap-sweep.svg)
