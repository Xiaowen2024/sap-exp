# SAP 10-box high-stiffness diagnostic

## Question

The primary stack sweep found that increasing `ke` from `1e5` to `1e6 N/m` increased absolute COM deviation from the zero-gap rigid reference. This diagnostic tests whether that was solver saturation or a contact-model/geometric-offset effect.

## Matrix

- Ten aligned 4 cm boxes, 3 s duration, `tau=3 ms`, `rigid_gap=0.1 mm`.
- `ke = 1e5, 1e6 N/m`; `dt = 0.25, 0.5, 1 ms`; preset = `drake`, `approx64`.
- Outputs: final COM, peak contact gap violation, final linear/angular speed, and signed layer gaps: ground--box 0 followed by box 0--1 through box 8--9.

`signed gap > 0` is separation; `signed gap < 0` is overlap.

## Result

At `dt=0.25 ms`, Drake gives:

| `ke` | COM deviation from zero-gap rigid reference | ground--box 0 gap | internal box--box gaps |
|---:|---:|---:|---:|
| `1e5` | +0.329 mm | -0.0314 mm | +0.0717 to +0.0969 mm |
| `1e6` | +0.433 mm | -0.0032 mm | +0.0958 to +0.0995 mm |

The same pattern holds at 0.5 and 1 ms and for `approx64`: increasing `ke` substantially reduces the actual bottom overlap, while internal bodies become separated by values approaching the configured `rigid_gap=0.1 mm`.

Final maximum linear speeds are `3e-6` to `5.5e-5 m/s`; angular speeds are about machine zero. The cases meet the experiment's settling threshold, though the nonzero linear residual should be retained in the raw data. This Warp build still reports internal solve/line-search iterations as `-1`, so iteration/optimality is not available from this API path.

## Conclusion

The original COM "regression" is primarily a **reference-model mismatch caused by `rigid_gap`**, not evidence that a 10x larger `ke` creates greater static compliance or proof of near-rigid saturation. The zero-gap rigid COM reference is inappropriate when SAP deliberately activates/maintains contact at a 0.1 mm skin offset. A rigid-equilibrium comparison must either account for this configured offset or repeat the diagnostic at `rigid_gap=0` and then assess its stability.

The diagnostic does not rule out additional SAP regularization effects; it rules out using this particular COM trend as their evidence.

## Artifacts

- Raw diagnostic data: `exp-results/box-stacking/sap-high-stiffness-diagnostic/results.csv`
- Runner: `sap-sim/scripts/run_box_stack_sweep.py`
