# MuJoCo peg-in-hole: clearance, softness, and virtual-tolerance results

## Scope

This report summarizes the previously completed MuJoCo-only experiment. SAP has not yet run the same scene or criteria.

Fixed conditions: blind socket inner radius 12 mm; initial lateral offset 1 mm; initial tilt 1 degree. Peg radii 10.0, 10.5, 11.0, 11.5, and 11.75 mm yield single-side clearances 2.0, 1.5, 1.0, 0.5, and 0.25 mm.

## Criteria

`entered_cavity` means final peg center satisfies both `z <= 45 mm` and radial offset `r_final <= 3 mm`.

`center_within_nominal_clearance` means `r_final <= c`, where `c` is the physical single-side clearance.

`mechanically_credible` is a stricter post-processing criterion, not a MuJoCo flag:

```text
entered_cavity
AND r_final <= c
AND max_penetration <= 0.1 c
AND final_penetration <= 0.1 c
```

The 10% threshold is intentionally conservative: an apparent insertion requiring penetration comparable to the physical clearance is treated as a numerical virtual pass, not a credible rigid assembly result.

## Aggregate result

| collision representation | cases | entered cavity | mechanically credible |
|---|---:|---:|---:|
| mesh_sdf | 25 | 22 | 0 |
| convex_decomposition | 25 | 7 | 2 |

## All clearance/softness cases

`solref` below is `[timeconst_s, dampratio]`; a larger time constant is a softer/slower contact response.

| mode | peg radius (mm) | clearance c (mm) | solref timeconst (ms) | entered | final radial (mm) | center <= c | max penetration (mm) | final penetration (mm) | 10% c (mm) | credible |
|---|---:|---:|---:|---|---:|---|---:|---:|---:|---|
| mesh_sdf | 10.00 | 2.00 | 1 | yes | 2.010 | no | 0.3246 | 0.0000 | 0.200 | no |
| mesh_sdf | 10.00 | 2.00 | 2 | yes | 1.991 | yes | 0.3605 | 0.0530 | 0.200 | no |
| mesh_sdf | 10.00 | 2.00 | 4 | yes | 1.997 | yes | 0.8414 | 0.0327 | 0.200 | no |
| mesh_sdf | 10.00 | 2.00 | 8 | yes | 2.060 | no | 1.6496 | 0.0339 | 0.200 | no |
| mesh_sdf | 10.00 | 2.00 | 16 | yes | 1.778 | yes | 3.5689 | 0.0764 | 0.200 | no |
| mesh_sdf | 10.50 | 1.50 | 1 | yes | 1.036 | yes | 0.2971 | 0.0010 | 0.150 | no |
| mesh_sdf | 10.50 | 1.50 | 2 | yes | 0.950 | yes | 0.3619 | 0.0324 | 0.150 | no |
| mesh_sdf | 10.50 | 1.50 | 4 | yes | 1.504 | no | 0.8524 | 0.0336 | 0.150 | no |
| mesh_sdf | 10.50 | 1.50 | 8 | yes | 1.691 | no | 1.5966 | 0.0519 | 0.150 | no |
| mesh_sdf | 10.50 | 1.50 | 16 | yes | 1.740 | no | 3.6610 | 0.1271 | 0.150 | no |
| mesh_sdf | 11.00 | 1.00 | 1 | yes | 0.745 | yes | 0.2885 | 0.0000 | 0.100 | no |
| mesh_sdf | 11.00 | 1.00 | 2 | yes | 1.175 | no | 0.3625 | 0.0075 | 0.100 | no |
| mesh_sdf | 11.00 | 1.00 | 4 | yes | 1.229 | no | 0.8389 | 0.0289 | 0.100 | no |
| mesh_sdf | 11.00 | 1.00 | 8 | yes | 1.149 | no | 1.5831 | 0.0328 | 0.100 | no |
| mesh_sdf | 11.00 | 1.00 | 16 | yes | 1.211 | no | 3.6820 | 0.1297 | 0.100 | no |
| mesh_sdf | 11.50 | 0.50 | 1 | no | 6.471 | no | 0.1886 | 0.0000 | 0.050 | no |
| mesh_sdf | 11.50 | 0.50 | 2 | yes | 0.735 | no | 0.1469 | 0.0000 | 0.050 | no |
| mesh_sdf | 11.50 | 0.50 | 4 | yes | 0.746 | no | 0.2409 | 0.0344 | 0.050 | no |
| mesh_sdf | 11.50 | 0.50 | 8 | yes | 0.680 | no | 1.0961 | 0.0349 | 0.050 | no |
| mesh_sdf | 11.50 | 0.50 | 16 | yes | 0.484 | yes | 3.6119 | 0.0781 | 0.050 | no |
| mesh_sdf | 11.75 | 0.25 | 1 | no | 23.684 | no | 0.1208 | 0.0082 | 0.025 | no |
| mesh_sdf | 11.75 | 0.25 | 2 | no | 5.915 | no | 0.1628 | 0.0211 | 0.025 | no |
| mesh_sdf | 11.75 | 0.25 | 4 | yes | 0.426 | no | 0.1993 | 0.0344 | 0.025 | no |
| mesh_sdf | 11.75 | 0.25 | 8 | yes | 0.466 | no | 0.3732 | 0.0347 | 0.025 | no |
| mesh_sdf | 11.75 | 0.25 | 16 | yes | 0.266 | no | 2.0748 | 0.0781 | 0.025 | no |
| convex_decomposition | 10.00 | 2.00 | 1 | yes | 0.612 | yes | 0.1918 | 0.0004 | 0.200 | yes |
| convex_decomposition | 10.00 | 2.00 | 2 | yes | 0.494 | yes | 0.1364 | 0.0004 | 0.200 | yes |
| convex_decomposition | 10.00 | 2.00 | 4 | yes | 0.498 | yes | 0.8916 | 0.0016 | 0.200 | no |
| convex_decomposition | 10.00 | 2.00 | 8 | yes | 0.339 | yes | 1.8480 | 0.0064 | 0.200 | no |
| convex_decomposition | 10.00 | 2.00 | 16 | yes | 0.523 | yes | 6.9058 | 0.0508 | 0.200 | no |
| convex_decomposition | 10.50 | 1.50 | 1 | no | 10.061 | no | 0.2779 | 0.0007 | 0.150 | no |
| convex_decomposition | 10.50 | 1.50 | 2 | no | 4.829 | no | 0.2779 | 0.0123 | 0.150 | no |
| convex_decomposition | 10.50 | 1.50 | 4 | no | 2.292 | no | 0.3050 | 0.0078 | 0.150 | no |
| convex_decomposition | 10.50 | 1.50 | 8 | yes | 0.372 | yes | 0.3650 | 0.0311 | 0.150 | no |
| convex_decomposition | 10.50 | 1.50 | 16 | yes | 0.099 | yes | 0.5594 | 0.1160 | 0.150 | no |
| convex_decomposition | 11.00 | 1.00 | 1 | no | 38.894 | no | 0.0562 | 0.0003 | 0.100 | no |
| convex_decomposition | 11.00 | 1.00 | 2 | no | 28.871 | no | 0.5897 | 0.0003 | 0.100 | no |
| convex_decomposition | 11.00 | 1.00 | 4 | no | 4.197 | no | 0.8936 | 0.2459 | 0.100 | no |
| convex_decomposition | 11.00 | 1.00 | 8 | no | 0.198 | yes | 0.9291 | 0.5169 | 0.100 | no |
| convex_decomposition | 11.00 | 1.00 | 16 | no | 0.163 | yes | 0.9408 | 0.4991 | 0.100 | no |
| convex_decomposition | 11.50 | 0.50 | 1 | no | 28.937 | no | 0.0806 | 0.0001 | 0.050 | no |
| convex_decomposition | 11.50 | 0.50 | 2 | no | 50.783 | no | 0.4611 | 0.0003 | 0.050 | no |
| convex_decomposition | 11.50 | 0.50 | 4 | no | 26.045 | no | 1.2297 | 0.0011 | 0.050 | no |
| convex_decomposition | 11.50 | 0.50 | 8 | no | 0.166 | yes | 1.4366 | 1.0046 | 0.050 | no |
| convex_decomposition | 11.50 | 0.50 | 16 | no | 0.141 | yes | 1.4529 | 1.0542 | 0.050 | no |
| convex_decomposition | 11.75 | 0.25 | 1 | no | 39.187 | no | 0.3460 | 0.0001 | 0.025 | no |
| convex_decomposition | 11.75 | 0.25 | 2 | no | 0.539 | no | 0.4553 | 0.0004 | 0.025 | no |
| convex_decomposition | 11.75 | 0.25 | 4 | no | 41.984 | no | 1.2259 | 0.0010 | 0.025 | no |
| convex_decomposition | 11.75 | 0.25 | 8 | no | 0.132 | yes | 1.6883 | 1.1939 | 0.025 | no |
| convex_decomposition | 11.75 | 0.25 | 16 | no | 0.136 | yes | 1.7077 | 1.2897 | 0.025 | no |

## Offset failure-boundary sweep

This is a separate sweep at 1 degree tilt and `solref=[0.004, 1]`. It used a looser success definition: `final z <= 45 mm` and `final radial <= 3 mm`; it did not apply the penetration credibility threshold above.

| mode | tested initial offsets that entered (mm) | observations |
|---|---|---|
| convex_decomposition | 0, 0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2 | monotonic in tested interval |
| mesh_sdf | 0, 0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2, 2.25, 2.5, 2.75, 3, 4.75 | non-monotonic success; do not interpret largest success as a physical boundary |

## Interpretation

- `mesh_sdf` reports 22/25 cavity entries but 0/25 credible insertions under the strict penetration criterion.
- `convex_decomposition` reports 7/25 cavity entries; only the 2 mm-clearance cases at 1 ms and 2 ms `solref` meet every credibility condition.
- Therefore neither apparent insertion depth nor final center position alone is enough to validate precision assembly. Collision representation and contact softness change the effective, numerical clearance.
