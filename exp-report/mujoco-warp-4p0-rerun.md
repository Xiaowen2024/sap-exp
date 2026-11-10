# MuJoCo Warp 4.0 rerun

Declared 2026-11-10. This report tracks execution; it is not a completed
4.0 result or a claim of improvement over 3.17.

## Scope and unchanged experiment contract

The seven tasks from the completed-result review are rerun with MuJoCo
4.0.0 **and MuJoCo Warp 4.0.0**. CPU model compilation is a compatibility
check, not a substitute for the CUDA evaluation. SAP results are preserved.

- Three-plate stack: strict-v3.1, corrected common ground/plate μ=0.8;
  12 held-out cells × R=20.
- Three-box push: strict-v4, corrected non-finite classification and actual
  per-step capacity audit; 25 held-out cells × R=20, plus the declared
  parameter sensitivity evaluation.
- ContactNets cube toss: dynamic-impact-v2; six held-out cells, 32 disjoint
  held-out trajectories × R=20 per cell, with the original trial eligibility
  and symmetric orientation metric.
- Simple pinch: force-controlled-v3; 19 held-out cells × R=20, with external
  force-based labels shared with the inertia-corrected SAP reference.
- GAUGE slope: dynamic-friction-v3; 35 held-out cells × R=20.
- MIT Push: dynamic-friction-v3; seven held-out cells × R=20, preserving
  the existing limits on independent trial coverage.
- Pendulum: energy-conservation-v2; nine held-out cells × R=20. The four
  declared MuJoCo integrator candidates remain four; no artificial fifth
  candidate is added to this contact-free task.

The registry is
[`mujoco_warp_4p0_rerun.json`](../sim_protocol/manifests/mujoco_warp_4p0_rerun.json).
It fixes each existing task manifest by SHA-256. All seven pass the common
manifest validator. Physical cases, candidates, gates, seeds, perturbations,
sampling and loss formulas are unchanged. Rerun B=1 and the entire original
candidate screen, select only on selection cases, then freeze the 4.0
winner for the existing held-out matrix. A 3.17 winner is not automatically
the 4.0 winner.

## Prerequisite completed

Both friction-aligned Three-plate artifacts were retrieved and validated:
SAP 240/240; MuJoCo Warp 3.17, 128/240. The capacity-instrumented Three-box
replay was also retrieved and validated: 485/500 valid and correct, with
14 finite contact-duty failures, one non-finite rollout and zero measured
capacity-overflow worlds. Historical files remain unchanged. See the
[correction report](three-plate-friction-and-three-box-validity-audit.md).

## Environment and execution

An independent GPU virtual environment is installed; the 3.17 environment
is retained. MuJoCo, mujoco-core and mujoco-warp are all 4.0.0. Warp remains
1.18.0+cu12; NumPy 2.4.6 and SciPy 1.17.1 match the previous GPU environment.
The execution bundle saves package versions, pip freeze, GPU/driver,
timestamped run state, input hashes, exact source/protocol snapshots and
separate logs per task.

The three GPU validity/capacity regression tests pass under 4.0, including
a real undersized contact buffer and a transient non-finite state. The
queue has started with Three-plate; the other six tasks are queued. No
completed 4.0 task result is claimed at this checkpoint.

The launcher is
[`run_mujoco4_rerun.py`](../scripts/protocol_v2/run_mujoco4_rerun.py).
It checks the corrected Three-plate pair before execution, requires CUDA,
checks the installed versions and stops on task failure or changed inputs.
It resumes only completed, structurally matching artifacts; a failed task
is inspected before retrying. Structural coverage checks do not certify
physical validity or numerical superiority.

Server restart can change the driver/runtime context. Compare recorded
environment files before attributing any difference solely to the MuJoCo
version; a fixed candidate and seed do not guarantee bitwise GPU replay.

## Result organization and publication

Each final task report keeps the existing three-part structure:

1. **Task setup:** physical setup, a clear schematic, important parameter
   values and metric/gate definitions.
2. **B=1 → B=5:** actual compiled defaults, B=1 score, selected 4.0 settings,
   specific changes, B=5 score, representative mechanism metrics and failure
   cases. Show representative candidates without turning all five into a
   separate website section.
3. **Generalizability:** first list the varied axes and exact values, then
   per-condition and overall results for the frozen setting. Keep failures
   in their declared denominator and distinguish contact/finite/capacity
   failures from wrong physical labels. Continuous-loss tasks report their
   defined loss rather than an invented binary accuracy.

Retain 3.17 artifacts and label the new results explicitly as 4.0. Update
the website only with audited completed results, keeping matching SAP
coverage and limits explicit. Do not relabel a preparation plan, GPU smoke
test or selection-only result as a completed generalizability comparison.
