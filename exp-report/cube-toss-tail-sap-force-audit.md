# Cube-Toss resting tail — SAP B=5 force-manifold audit

**Audit date:** 2026-10-31  
**Artifact:** `exp-results/protocol-v2-native-default/sap-internal-cube-toss-tail-b5-force-manifold-r20/`  
**Execution:** GPU-only SAP Warp rerun; 32 held-out release states × R=20 = 640 worlds.

## Repaired telemetry definition

The legacy saved normal-force aggregate is retired.  It mixed a previous
contact solve with the current state and did not identify the support
manifold, so it could not support a force conclusion.

The replacement trace samples a state only after that same state has been
contact-solved. At every physics step it records the active cube--ground
contacts, their body IDs, world-frame signed impulse resultant divided by
`dt`, contact count, minimum gap, position, and velocity. The trace has no
contact-capacity truncation.

## Result

Frozen SAP B=5 is `ke=1e4, tau=0`, with `dt=1/148 s` and cube mass 0.37 kg.
All **640/640** worlds settle, are finite, and have cube--ground contact duty
**1.000**. The tail motion is quiet: mean drift is **3.48 µm/s**, linear RMS
speed **0.155 mm/s**, and angular RMS speed **0.0020 rad/s**.

The manifold trace also establishes that the problem is no longer an impulse
definition error. Each sampled state has four support contacts and zero
truncation. When SAP emits a support impulse, its normal resultant is tightly
centred on the expected static load:

| Measure | Result |
| --- | ---: |
| Expected load, `m g` | 3.6297 N |
| Positive-pulse mean normal resultant | 3.6297 N |
| Positive-pulse 1st–99th percentile | 3.6294–3.6300 N |
| Positive-pulse duty during settled tail | 0.6700 |
| Zero-resultant duty during settled tail | 0.3300 |
| Time-mean normal resultant | 2.4331 N |
| Time-mean load-closure error | 32.97% |
| Tail force p99 / maximum | 3.6298 / 3.6422 N |

## Reporting decision

The B=5 tail trajectory, settling, finite-state, support-contact continuity,
and bounded pulse scale are reportable. SAP's time-averaged normal-load,
force CV, and force-balance comparison are **not** reportable: this frozen
configuration represents support through intermittent `m g`-sized impulses
rather than a load-balanced per-step resultant. This is a property observed
by the corrected manifold telemetry, not a conclusion from the invalid legacy
aggregate.
