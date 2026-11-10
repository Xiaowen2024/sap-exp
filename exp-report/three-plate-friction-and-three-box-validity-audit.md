# Three-plate friction and Three-box validity correction

Revision declared 2026-11-09. Historical artifacts remain unchanged.

## Three-plate stack

The strict-v3 MuJoCo scene assigned sliding friction 0.8 to the plates but
left ground friction at its native value 1.0. MuJoCo 3.17.0 compiles the
ground–bottom-plate sliding coefficient as 1.0. The SAP ground inherits the
plate material's 0.8. Thus the historical 240/240 SAP and 121/240 MuJoCo
counts describe different ground friction and are not the final aligned
comparison.

The common `three_plate_stack_strict_v3p1.json` declares sliding friction 0.8
on both ground and plates. Both engines are rerun from the five-candidate
selection screen through the same 12 frozen evaluation cells, R=20 each.
Seeds, perturbations, gates, duration and candidates are unchanged.
MuJoCo capacity flags are accumulated at every physics step.

The historical MuJoCo selection pass counts were 0, 3, 10, 8, 8 out of 20,
not 10/20 for every candidate. A maximum of 10/20 establishes failure of
the declared candidate screen to produce consistently passing calibration;
it does not establish that candidate coverage was the cause.

Corrected MuJoCo run completed and was downloaded and validated:
**128/240 (53.33%)**, with frozen `tc=8 ms, dampratio=2, dmax=.99`.
Its five selection counts are **0, 2, 8, 10, 8 /20**. Every selection and
held-out run has zero recorded capacity-overflow worlds and zero overflow
flags. Peak active contacts over those runs: **231**.

The new SAP selection still chooses `ke=1e6, tau=3 ms`; its selection pass
counts are 0, 0, 20, 20, 20 /20. The restored server's new SAP output was
retrieved on 2026-11-10 and validated: **240/240 (100%)**. Both artifacts
match the friction-aligned manifest, all 12 held-out cells, R=20 raw rows,
the frozen candidate, and reconstructed totals. This completes the aligned
pair: SAP 240/240 versus MuJoCo Warp 128/240. The old 121/240 and new
128/240 are separate runs; their difference is not isolated causal evidence
of a friction effect, given GPU run-to-run variation and the different winner.

## Three-box push

The historical strict-v4 result is SAP 500/500 and MuJoCo 478/500 valid and
correct rollouts. MuJoCo has 20 finite rollouts that fail the pre-event
contact-duty gate, plus two rollouts with non-finite diagnostics. The two
non-finite cases cannot count as correct retain classifications. The old
classifier defaulted to `retain` when neither event comparison fired,
which also happens for NaN inputs.

The corrected classifier labels non-finite position or orientation histories
`invalid`, with `agrees=false`. It separately records finite-state validity,
contact-duty validity, and capacity validity per world. Regression checks
cover a NaN position history, a NaN orientation history, and an overflow in
only one world.

The old MuJoCo driver supplied a literal `truncated=0`; that is not evidence
that capacity was checked. A new run of the original v4 manifest accumulates
all contact/constraint capacity overflow bits at every physics step, saves
contact/constraint high-water marks, and records solver iteration warnings
separately from capacity overflow. Non-finite qpos/qvel is latched at every
step as well. Physical cases, seeds, outcome thresholds and the historical
frozen winner are unchanged. Historical B1/selection rows retain their source
provenance; two new selection-cell replays measure repeatability of that same
frozen configuration. A separate replication of the five-candidate screen
changed candidate ordering; that diagnostic is not used to retune this audit.

Historical conclusion: under this declared candidate budget and finite
evaluation horizon, SAP has 500/500 valid and correct outcomes; MuJoCo has
478/500. The other 22 MuJoCo rollouts fail numerical/contact validity and
are included in the 500 denominator. The phrase “500/500 classifications
correct” is withdrawn. Historical capacity telemetry is unavailable.

The capacity-instrumented frozen MuJoCo replay was retrieved on 2026-11-10
and passes the matrix/raw-row/provenance validation: **485/500 (97%) valid
and correct**. The original frozen configuration remains
`k=1e5, impratio=100, noslip=2`; its two selection-cell replays pass 40/40.
All 499 finite held-out rollouts have correct outcome labels. Fourteen
finite rollouts fail the pre-event contact-duty gate; one rollout is
non-finite and has class `invalid`. There are zero measured capacity-overflow
worlds. The non-finite rollout is included in the 500 denominator, never
counted as a correct retain outcome. This new replay does not retroactively
provide capacity telemetry for historical results, and the 478→485 change
is not a parameter-tuning improvement: settings and seeds were unchanged.

Verification: all three validity/capacity regression tests pass on the
evaluation GPU. The integration test deliberately allocates two contact
slots for a scene producing eight contacts; the audit detects the native
narrowphase overflow flag. Injected transient NaN and overflow flags remain
latched after subsequent values become finite/zero. Solver iteration warnings
are retained as diagnostics and are not mislabelled as capacity overflow.
