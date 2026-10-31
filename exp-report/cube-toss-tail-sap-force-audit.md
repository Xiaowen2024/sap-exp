# Cube-toss resting tail — SAP force telemetry audit

**Audit date:** 2026-10-31  
**Source:** `exp-results/protocol-v2-native-default/sap-internal-cube-toss-tail-b1-b5-r20/sap_cube_toss_tail_b1_b5_r20.json`  
**Scope:** offline audit only; no simulation was rerun.

## Result

The SAP tail artifact is valid for its **settling and drift** observables, but
its saved normal-force aggregate does **not** pass a force/load-closure audit.
It must not be used for a SAP normal-load or force-balance comparison.

Both configurations contain 32 released samples × R=20 = 640 finite,
settled worlds. Contact duty is 1.0 and the reported tail drift is finite.
However, the stored per-world normal-force summaries are strongly bimodal:
some settled worlds report approximately 3.6 N with small closure error,
while others report roughly 0.08–0.15 N and a 96.7–97.8% closure error.
The same final-height/settling condition cannot make those values a stable
support-load estimate.

| SAP configuration | Worlds | Aggregate load-balance error | Per-world median / p95 load-balance error | Per-world normal-force range | Decision |
| --- | ---: | ---: | ---: | ---: | --- |
| native default | 640 | 41.34% | 12.83% / 97.46% | 0.078–3.616 N | reject force aggregate |
| `ke=1e4, tau=0` | 640 | 30.13% | 2.02% / 97.37% | 0.065–3.621 N | reject force aggregate |

The matching per-world normal-force coefficient of variation is also unstable:
the native-default median/p95 is 0.384 / 6.197, and the `ke=1e4, tau=0`
median/p95 is 0.143 / 6.083. These are diagnostics of an invalid aggregation,
not evidence of intermittent physical loss of support: the saved contact duty
is 1.0 for every world.

## What is repaired by this audit

- The task is no longer labelled “force audit pending.” The available force
  observable has been evaluated and **fails** its closure/consistency audit.
- The settled-tail drift and finite-state observations remain reportable.
- SAP normal-force mean, coefficient of variation, and load-balance error are
  excluded from any SAP-versus-MuJoCo conclusion.

## What would be required for a future force conclusion

The current JSON stores only final aggregates, not per-contact impulses,
contact IDs, normals, signed gaps, or per-timestep support resultants. Those
inputs are required to determine whether this is a unit conversion, incorrect
row selection, or support-manifold aggregation bug. A corrected conclusion
therefore requires a new SAP Warp GPU run that records the complete active
support manifold and computes the resultant in a declared world frame. It is
not recoverable from the existing aggregate-only artifact.
