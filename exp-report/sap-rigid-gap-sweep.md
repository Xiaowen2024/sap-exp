# SAP `rigid_gap` sweep: high-stiffness 10-box stack

## Setup

Ten aligned 4 cm boxes; Drake preset; `tau=3 ms`; 3 s simulation. Sweep:

- `ke`: `1e5`, `1e6 N/m`
- `dt`: `0.25`, `0.5 ms`
- `rigid_gap`: `0`, `0.01`, `0.05`, `0.1 mm`

The rigid reference is a zero-gap stack with COM height 200 mm. A case is considered
settled only if its recorded settling time is below 3 s and its tail motion is negligible.

## Key results

| dt ms | ke N/m | rigid gap mm | COM deviation mm | peak overlap mm | tail jitter mm | final max v m/s | settled? |
|---:|---:|---:|---:|---:|---:|---:|---|
| 0.25 | 1e5 | 0 | 0.03197 | 0.2413 | 0.02112 | 0.01670 | no |
| 0.25 | 1e6 | 0 | 0.00082 | 0.2106 | 0.05677 | 0.02967 | no |
| 0.25 | 1e5 | 0.05 | 0.32912 | 0.1249 | 0 | 0.000025 | yes |
| 0.25 | 1e6 | 0.05 | 0.43329 | 0.0918 | 0 | 0.000058 | yes |
| 0.5 | 1e5 | 0 | 0.04530 | 0.3537 | 0.06429 | 0.02056 | no |
| 0.5 | 1e6 | 0 | 0.00614 | 0.3359 | 0.08918 | 0.02762 | no |
| 0.5 | 1e5 | 0.05 | 0.32910 | 0.2008 | 0 | 0.000024 | yes |
| 0.5 | 1e6 | 0.05 | 0.41682 | 0.1797 | 0 | 0.000028 | yes |

`rigid_gap=0.01 mm` also fails to settle in all four tested combinations; its
final max linear speed is `0.011–0.037 m/s`. `rigid_gap=0.1 mm` is practically
identical to 0.05 mm in static COM and settling, while reducing peak overlap.

## Interpretation

1. With **zero rigid gap**, raising `ke` from `1e5` to `1e6 N/m` reduces the
   zero-gap COM deviation from 0.032 to 0.00082 mm at 0.25 ms (and 0.045 to
   0.0061 mm at 0.5 ms). Thus the static stiffness direction is physically
   expected: higher `ke` does not create extra compliant compression.
2. Zero/small gap is not usable here as a static benchmark at 3 s: it has
   persistent motion and larger jitter. This is a stability/settling limitation,
   even though it is not classified as a NaN/topple failure.
3. At 0.05--0.1 mm, contacts rapidly settle, but their internal signed gaps are
   near 0.1 mm. The activation skin is applied by the pair/contact construction,
   so a 0.05 mm per-shape setting can yield an approximately 0.1 mm pairwise
   separation. This offset dominates the zero-gap COM deviation.
4. Therefore, `rigid_gap` is a real accuracy--stability trade-off for this SAP
   configuration. Comparing it to MuJoCo's zero-gap geometry requires either
   using the near-zero-gap result while reporting its non-settling, or using a
   gap-aware reference equilibrium for stable SAP settings.

## Raw result

`exp-results/box-stacking/sap-rigid-gap-sweep/results.csv`
