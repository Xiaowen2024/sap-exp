#!/usr/bin/env python3
"""Generate the round-socket strict-v9 DRAFT manifest from the v8 generator.

The 64 conditions, their labels, seeds, thresholds and the outcome taxonomy are those of strict-v8-draft
(build_round_socket_v8_manifest.build(), unchanged; the script asserts this against the v8 file).  v9 changes how candidates are
declared and how results are selected and reported, after the v8 pilot review:

  1. candidates: each engine declares its own five configurations in its own parameters.  Hardness is no longer matched across
     engines (v8's penetration ladder is dropped as a definition; the SAP and MuJoCo values it produced are kept as declared values,
     so runs already made stay valid).  Genesis IPC is added with its own clause;
  2. selection rule: most correct ROLLOUTS over the selection conditions (v8 ranked "fewest unstable" first, so a candidate with 7 of 8
     rollouts correct lost to one with 0 of 8);
  3. one universal criterion for every condition of every block: (a) label result k / R with the verdict, (b) deviation from the
     rigid-body answer in fixed columns.  Deviations are reported only: no ranking, no weights, no single number;
  4. the main result is each engine's frozen B = 5 configuration on every block, reported per block; no cross-block total;
  5. supplementary tests (hardness of each candidate, reachable hardness versus time step, attribution, robustness) explain the main
     result and never change it.

Usage: python3 build_round_socket_v9_manifest.py [--out sim_protocol/manifests/round_socket_strict_v9_draft.json]
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

import build_round_socket_v8_manifest as v8

ROOT = Path(__file__).resolve().parents[2]
V8_FILE = ROOT / "sim_protocol/manifests/round_socket_strict_v8_draft.json"
HARDNESS_DT_MS = [0.125, 0.25, 0.5, 1.0, 2.0]
HARDNESS_MARKS_MM = [0.15, 0.3, 0.6, 1.2]

# ------------------------------------------------------------------------------------------------ rules
RULES = copy.deepcopy(v8.RULES)
del RULES["unit_of_analysis"]
del RULES["fidelity_metrics_reported_for_every_condition"]
RULES = {
    "universal_criterion": {
        "scope": "one criterion, defined once and applied identically to every condition of every block, every engine and every configuration "
                 "(selection candidates, frozen B = 5, native B = 1, supplementary runs).",
        "1_label_result": "labelled conditions: k = number of the R = 8 rollouts whose outcome matches the label (correct_rule). Both k / R and the "
                          "condition verdict are reported: correct (k = R), unstable (0 < k < R), wrong (k = 0). The outcome thresholds are wide on "
                          "purpose: the verdict says whether the outcome is right in kind, not how close the trajectory is to the rigid-body answer.",
        "2_deviation": "the distance of the trajectory from the rigid-body answer, in the fixed columns of deviation_columns, reported next to the "
                       "label result for every condition. REPORT ONLY in this revision: a deviation never changes a verdict, the columns are not "
                       "combined into one number, are not weighted and are not used to rank engines.",
        "rollouts": "R = 8 rollouts per condition (perturbation U(+-0.05 mm) perpendicular to the tilt plane, identical arrays for all engines). "
                    "Per condition each deviation column is given as mean, min and max over the rollouts (rates keep their sign).",
        "aggregation": "results are reported per block (E, W, R, I, C), split into same-clearance and unseen-clearance groups: labelled conditions "
                       "correct / unstable / wrong, the rollout total, and for each deviation column the median and the worst condition. No "
                       "cross-block total and no block weights are defined; how (or whether) to combine blocks is decided after the run and is a "
                       "new revision."},
    "deviation_columns": {
        "held_in_wedge / held_on_rim (rigid answer: stays at the release pose, at rest)": [
            "max displacement from the release pose over the run, mm (0)", "max tilt change, deg (0)",
            "vertical creep rate over the last 0.5 s, mm/s (0)", "tilt rate over the last 0.5 s, deg/s (0)",
            "strict-hold count: rollouts with displacement <= 0.5 mm and tilt change <= 0.5 deg, settled (R)"],
        "inserted (rigid answer: flat on the base inside the bore, at rest)": [
            "final centre height, mm (35.0)", "final tilt, deg (a member of final_state_set.tilt_deg)",
            "final radial offset beyond the clearance, um (0)", "vertical creep rate over the last 0.5 s, mm/s (0)",
            "control cells only: time from release to first base contact, ms (80.77)"],
        "tips_off_rim": ["none: the end state is not labelled; only the label result applies"],
        "unlabelled": ["the same columns are recorded without a reference; only the universal checks make a correctness statement"]},
    **RULES}
RULES["criteria_freeze"] = ("every threshold and every column above is fixed at declaration. Smoke tests may use the selection conditions only. If a "
                            "label is contested after the run, the condition is dropped for all engines, never relabelled. Any other change makes a "
                            "new revision.")

# ------------------------------------------------------------------------------------------------ candidates (declared values)
MJ_COMMON = "dampratio = 1; solimp = 0.95 0.95 0.001 0.5 2; integrator = implicitfast, cone = elliptic, solver = Newton"
CANDIDATES = {
    "principle": "each engine declares five configurations in its own parameters; candidate 1 is the engine's literal native default (B = 1). "
                 "Hardness is NOT matched across engines: how hard an engine's contact can be set at the task time step is a property of that "
                 "engine and shows in its results. The hardness of every candidate is measured and reported (run_plan.5_hardness, supplementary); "
                 "it does not define a candidate. Physical inputs (geometry, mass, inertia, friction, gravity, dt) are never candidates.",
    "sap_warp": {
        "declared_order": [
            {"index": 1, "id": "native_default"},
            {"index": 2, "id": "ke=25483,tau=1.5ms,gap=.1mm", "ke": 25482.967479793464, "tau_s": 0.0015, "gap_m": 0.0001},
            {"index": 3, "id": "ke=10746,tau=1.5ms,gap=.1mm", "ke": 10746.078283213175, "tau_s": 0.0015, "gap_m": 0.0001},
            {"index": 4, "id": "ke=57255,tau=1.5ms,gap=.1mm", "ke": 57254.878843583785, "tau_s": 0.0015, "gap_m": 0.0001},
            {"index": 5, "id": "ke=4531.6,tau=1.5ms,gap=.1mm", "ke": 4531.583637600818, "tau_s": 0.0015, "gap_m": 0.0001}],
        "origin": "the parameter values and order of the v8 pilot's five candidates, kept exactly so that every SAP run of the pilot stays valid"},
    "mujoco_warp": {
        "common_to_candidates_2_to_5": MJ_COMMON,
        "declared_order": [
            {"index": 1, "id": "native_default"},
            {"index": 2, "id": "tc=1.5ms", "solref_timeconst_s": 0.0015, "new_in_v9": True},
            {"index": 3, "id": "tc=2.021ms", "solref_timeconst_s": 0.0020211825789663187},
            {"index": 4, "id": "tc=1ms", "solref_timeconst_s": 0.001, "new_in_v9": True},
            {"index": 5, "id": "tc=4.171ms", "solref_timeconst_s": 0.00417060914342511}],
        "engine_limit": "timeconst < 2 dt is not allowed (MuJoCo's own limit). At a time step where a candidate violates it, the candidate is "
                        "recorded as not runnable and is not retuned (this affects candidates 2 and 4 in the dt = 1.0 ms robustness run).",
        "origin": "candidates 1, 3 and 5 are the v8 pilot's three runnable candidates, values and positions unchanged. Positions 2 and 4 were "
                  "unreachable penetration rungs in v8; they now hold two harder settings the engine can run at dt = 0.5 ms (1.0 ms = 2 dt is "
                  "the hardest it allows)."},
    "genesis_ipc": {
        "declared_order": [
            {"index": 1, "id": "native default", "note": "IPCCouplerOptions() defaults: d_hat 10 mm, eps_velocity 0.01 m/s, Newton velocity tolerance 0.05 m/s"},
            {"index": 2, "id": "d_hat=0.03mm,eps_v=1e-3,newton_tol=1e-4"},
            {"index": 3, "id": "d_hat=0.1mm,eps_v=1e-3,newton_tol=1e-4"},
            {"index": 4, "id": "d_hat=0.01mm,eps_v=1e-3,newton_tol=1e-4"},
            {"index": 5, "id": "d_hat=0.03mm,eps_v=1e-4,newton_tol=1e-5"}],
        "numerical_parameters": "d_hat (contact distance: the peg rests about 0.9 d_hat away from a surface), eps_velocity (friction smoothing "
                                "speed) and the Newton velocity tolerance (set to eps_velocity / 10). contact_resistance stays at its default.",
        "origin": "new in v9; no IPC run of this task exists"}}

IPC_CLAUSE = {
    "engine": "Genesis 1.4.6 + IPCCoupler (libuipc 0.0.29), ABD rigid bodies, CUDA. Runner: scripts/genesis/run_socket_v8_genesis_ipc.py",
    "what_is_shared": "conditions, labels, seeds, perturbations, time step, the universal criterion and all scoring code (pose-based) are those of "
                      "the other engines, unchanged.",
    "geometry": "same analytic shapes, mesh representation: the socket is one watertight non-convex mesh (outer 32-gon prism with the 32-gon "
                "blind bore; no convex pieces, no decomposition); the peg is a 128-gon prism whose vertices lie on the nominal cylinder (flats "
                "at most 3.6 um inside, below radial_tolerance_mm), with one vertex line in the tilt plane on each side. Inertia is the "
                "uniform-density inertia of the prism (the manifest's solid-cylinder inertia to within the faceting).",
    "start_gap": "IPC cannot start at zero distance. A pose the manifest releases in contact (blocks W, R and their I copies) is moved by a "
                 "rigid translation, tilt unchanged, until every contact is g0 = d_hat of the candidate apart: a rim rest is lifted g0; a "
                 "two-point wedge is translated in the tilt plane so that both wedge gaps equal g0. Poses released in the air (E, C) are "
                 "unchanged. The shift is recorded per condition and is part of the scored trajectory: displacement and drop are measured "
                 "from the shifted start, so the settling onto the contacts appears in the deviation columns.",
    "known_limits": [
        "the rest gap is not a penetration: the non-penetration check passes by construction and the selection tie-break 'lowest penetration' is "
        "0 for every candidate, so ties fall to declared order. The gap shows in the deviation columns (for example the final centre height of a "
        "seated peg) and is reported as rest gap per condition and as a fraction of the clearance",
        "with d_hat = 30 um the gap is about 9 % of the 0.3-mm clearance and about 27 % of the 0.1-mm clearance; results at 0.1 mm are flagged",
        "candidate 1 (d_hat 10 mm) is of the size of the bore radius; it is run as the literal B = 1 and is not expected to be meaningful",
        "the R = 8 rollouts of a condition share one IPC scene (one Newton solve) and are numerically coupled",
        "libuipc aborts the process on a diverged solve; the condition's rollouts are then non-finite and count as invalid"],
    "not_applicable": ["engine-reported contact depth and the spurious-contact count (reported as 0)",
                       "block R support-edge diagnostic (two steps after release the peg is still outside the barrier)",
                       "the MuJoCo attribution control"]}

# ------------------------------------------------------------------------------------------------ run plan
_cal = v8.LADDER["calibration_cell"]
PLAN = copy.deepcopy(v8.PLAN)
PLAN["roles"] = {
    "main_result": "phases 1 and 2. Each engine is evaluated with its own frozen B = 5 configuration (and its native default, B = 1) on the whole "
                   "frozen matrix, every block including I. All conclusions are drawn from this.",
    "supplementary": "phases 3, 4 and 5. They explain the main result (why a block came out as it did). They never change a candidate, the frozen "
                     "configuration, a verdict, a deviation or any block result, and they are reported in a separate section."}
del PLAN["1_selection"]["candidates_in_declared_order"]
PLAN["1_selection"]["candidates"] = "the five declared configurations of the engine, in the order of candidates.<engine>.declared_order"
PLAN["1_selection"]["selection_rule"] = (
    "most correct rollouts summed over the selection conditions (k / R per condition, so an unstable condition counts by its share); then fewest "
    "inadmissible rests; then lowest mean pose-based peak penetration over the selection conditions; then declared order. A candidate that is not "
    "runnable at the task time step is skipped. B = k is the same rule over the first k candidates (kept in the raw artifact); B = 1 is the "
    "native default, B = 5 the frozen configuration. Deviations are not used.")
PLAN["4_attribution_control"] = ("supplementary. MuJoCo only, selection conditions only: candidate 3's solref and solimp with the native integrator / "
                                 "cone / solver, to separate stiffness from solver options")
PLAN["5_hardness"] = {
    "purpose": "supplementary. States how hard each declared candidate is, how hard each engine's contact can be made at a given time step, and "
               "what time step an engine needs for a given hardness. Information only.",
    "cell": {**_cal, "note": "32-mm centred drop; not a scored condition"},
    "observable": "pose-based peak base penetration during the drop (mm). Damping check: after first base contact the peg stays within 0.05 mm of "
                  "its final height from 0.3 s onward.",
    "a_candidate_hardness": "every declared candidate of every engine at the task time step (0.5 ms): peak base penetration and stop time after "
                            "first contact.",
    "b_reachable_hardness_vs_time_step": {
        "dt_ms": HARDNESS_DT_MS,
        "method": "per engine and dt, bisect the engine's stiffness parameter for the smallest peak base penetration that passes the damping check "
                  "(SAP: ke, with tau the smallest of {1.5, 3, 6, 12, 24} ms that passes; MuJoCo: solref timeconst >= 2 dt, other options as "
                  "candidates 2-5)",
        "reported": ["per engine and dt: the smallest reachable peak base penetration",
                     f"per engine: the largest tested dt at which {', '.join(map(str, HARDNESS_MARKS_MM))} mm is reachable (or 'not reachable in the tested range')",
                     "wall time per simulated second of this cell at each dt (same GPU for all engines)"]},
    "genesis_ipc": "penetration is 0 by construction; reported instead: the rest gap above the base for each candidate, and wall time per "
                   "simulated second at each dt",
    "not_done": "no scored condition is re-run at another hardness or time step for this test"}

REPORTS = copy.deepcopy(v8.REPORTS)
REPORTS["0_selection_table"] = ("per engine: all five candidates on the selection conditions (k / R and verdict per condition, rollout total, "
                                "pose-based penetration), the frozen index for B = 1 .. 5 and what changed from B = 1.")
REPORTS["1_hardness_correctness_curve"] = ("supplementary, descriptive: outcome on the three selection conditions versus the pose-based penetration "
                                           "measured on those conditions, for the candidates of every engine on one axis.")
REPORTS["2_frozen_result"] = ("MAIN RESULT. B = 1 and frozen B = 5 on the frozen matrix, one table per block (E, W, R, I, C), same-clearance and "
                              "unseen-clearance groups: per condition k / R, verdict and the deviation columns; per block the counts correct / "
                              "unstable / wrong, the rollout total and, per deviation column, the median and the worst condition. Inadmissible rests "
                              "on every condition including the unlabelled ones. No cross-block total.")
REPORTS["4_invariance"] = ("I is reported in the main result like every other block (frozen B = 5 and B = 1). In addition, descriptively: the largest "
                           "gravity scale (1, 8, 64) and mass scale at which the outcome still equals the base cell's; whether the mirror image "
                           "gives the same outcome.")
REPORTS["6_robustness"] = "supplementary: dt = 0.25 / 0.5 / 1.0 ms and the repeat run on the robustness subset."
REPORTS["7_hardness"] = ("supplementary: the hardness of every declared candidate; reachable peak base penetration versus dt for every engine on one "
                         "axis; the dt each engine needs for 0.15 / 0.3 / 0.6 / 1.2 mm; cost per simulated second.")

NOT_COVERED = [s for s in v8.NOT_COVERED if not s.startswith("Blindness")] + [
    "Blindness: clearances 0.1 and 0.3 mm were first run in the v8 pilot, and the blocks were designed knowing v6's findings (SAP's rim support "
    "gap, MuJoCo's soft-contact behaviour). v9's candidate, selection and reporting rules were written after the v8 pilot results of SAP and "
    "MuJoCo had been seen (MuJoCo's 0.6-mm rung with 7 of 8 rim rollouts correct lost to the 1.2-mm rung with 0 of 8; both engines 10 / 10 on W "
    "with different creep), and MuJoCo's two new candidates were chosen after that pilot. Conditions, labels and thresholds are unchanged from "
    "v8. This is disclosed, not removable.",
    "No single score: this revision defines no cross-block total, no block weights and no ranking by deviation. 17 of the 46 labelled frozen "
    "conditions are rim-support cases (R 11, I 3, E 3), so any unweighted total would mostly measure that one mechanism.",
    "Hardness is not matched: engines are compared each at its own selected configuration. A comparison at equal hardness is not part of the "
    "main result; the supplementary hardness test only states what equal hardness would require."]

REUSE = {
    "rule": "a run made under strict-v8-draft is reused without re-running when the candidate parameters, condition, seed, time step and runner "
            "code are identical. The artifact keeps its v8 manifest sha256 and is re-tabulated under the v9 criterion (every v9 column is already "
            "in its per-rollout rows). Experiments already made are not changed.",
    "sap_warp": "all pilot runs: selection of the five candidates, frozen B = 5 (candidate 4; the v9 selection rule freezes the same one), frozen "
                "B = 1, robustness.",
    "mujoco_warp": "selection of candidates 1, 3 and 5; frozen B = 1; the attribution control. The pilot's frozen B = 5 and robustness runs used "
                   "candidate 5 and are reused only if the v9 selection freezes candidate 5.",
    "new_runs": ["MuJoCo Warp: selection of candidates 2 and 4 (2 x 3 conditions); then frozen B = 5 (61 conditions) and robustness (24) for the "
                 "v9 winner unless it is candidate 5. On the pilot's three candidates alone the v9 rule picks candidate 3 (23 of 24 rollouts).",
                 "Genesis IPC: everything (selection 15, frozen B = 5 61, frozen B = 1 61, robustness 24)",
                 "supplementary hardness test: all engines"]}

CHANGES = [
    "conditions, labels, seeds, thresholds, outcome taxonomy and universal checks: unchanged (identical to strict-v8-draft)",
    "candidates: declared per engine in engine parameters (section candidates). v8's regularisation ladder (candidates defined by matching a "
    "penetration target across engines) is removed as a definition; its drop test remains as a supplementary hardness measurement",
    "SAP candidates: the v8 pilot's five, values and order unchanged. MuJoCo candidates: the pilot's three kept in place, two new ones "
    "(timeconst 1.5 ms and 1.0 ms) in the two positions that were unreachable rungs. Genesis IPC added (candidates and engine_clauses.genesis_ipc)",
    "selection rule: 'most conditions correct, then fewest unstable' replaced by 'most correct rollouts'",
    "rules.unit_of_analysis and rules.fidelity_metrics_reported_for_every_condition replaced by rules.universal_criterion and "
    "rules.deviation_columns (label result k / R + verdict, and report-only deviations)",
    "main result = each engine's frozen B = 5 (and B = 1) on every block, per block, no cross-block total",
    "supplementary: phase 5 (candidate hardness, reachable hardness versus time step); robustness and the attribution control labelled supplementary",
    "runs already made under v8 are reused where identical (section reuse_of_v8_pilot_runs)"]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, default=ROOT / "sim_protocol/manifests/round_socket_strict_v9_draft.json")
    a = ap.parse_args(); cells, geom = v8.build()
    for c in cells:
        c["role"] = "selection" if c["id"] in v8.SELECTION_IDS else "frozen"
    old = json.loads(V8_FILE.read_text()); v8_sha = hashlib.sha256(V8_FILE.read_bytes()).hexdigest()
    assert json.loads(json.dumps(cells)) == old["conditions"], "conditions differ from the strict-v8-draft file"
    assert json.loads(json.dumps(geom)) == old["geometry_summary"]
    assert {k: v for k, v in RULES.items() if k in old["rules"] and k != "criteria_freeze"} == {k: old["rules"][k] for k in RULES if k in old["rules"] and k != "criteria_freeze"}
    robust = old["run_plan"]["3_robustness"]["conditions"]; PLAN["3_robustness"]["conditions"] = robust
    for eng in ("sap_warp", "mujoco_warp", "genesis_ipc"):
        assert [c["index"] for c in CANDIDATES[eng]["declared_order"]] == [1, 2, 3, 4, 5]
    runs = {k: v for k, v in old["cell_runs_per_engine"].items() if not k.startswith("total") and k != "mujoco_attribution_control"}
    runs["total_per_engine"] = sum(runs.values()); runs["mujoco_attribution_control"] = len(v8.SELECTION_IDS)
    runs["supplementary_hardness"] = f"one drop cell: 5 candidates per engine, plus a stiffness bisection at {len(HARDNESS_DT_MS)} time steps (no condition runs)"
    man = {"protocol": old["protocol"], "task": old["task"], "revision": "strict-v9-draft",
           "status": "DRAFT 2026-11-16: not declared. Same conditions and labels as strict-v8-draft; candidates, selection and reporting rules revised after the v8 pilot review; Genesis IPC added. No run has been made under this file.",
           "supersedes": f"strict-v8-draft (pilot run kept in exp-results/round-socket-v8-draft; manifest sha256 {v8_sha[:16]}...). strict-v6 results and the strict-v7 draft are kept.",
           "changes_from_v8": CHANGES,
           "generator": "scripts/protocol_v2/build_round_socket_v9_manifest.py (conditions and labels from build_round_socket_v8_manifest.py / peg_hole_statics.py)",
           "R": old["R"], "seed0": old["seed0"], "seed_rule": old["seed_rule"].replace("every engine, rung and repeat", "every engine, candidate and repeat"),
           "engines": {"default": old["engines"]["default"] + ["Genesis 1.4.6 + IPC (libuipc 0.0.29), under engine_clauses.genesis_ipc"],
                       "open_decision": old["engines"]["open_decision"]},
           "design_rule": old["design_rule"], "fixed": old["fixed"],
           "rules": RULES, "candidates": CANDIDATES, "engine_clauses": {"genesis_ipc": IPC_CLAUSE}, "run_plan": PLAN,
           "reuse_of_v8_pilot_runs": REUSE, "cell_runs_per_engine": runs, "reports": REPORTS, "not_covered": NOT_COVERED,
           "implementation_required": [
               "socket_v8_common.execute: take the candidates from the manifest's declared lists (no ladder calibration before selection); "
               "selection_key: correct rollouts first",
               "SAP and MuJoCo runners: build candidates from candidates.<engine>.declared_order; MuJoCo 'runnable' check per candidate and dt",
               "result tables: per-block main table with k / R, verdict and the deviation columns (all columns are already in the per-rollout rows)",
               "supplementary hardness test: candidate hardness and the stiffness bisection at each dt",
               "Genesis IPC runner (scripts/genesis/run_socket_v8_genesis_ipc.py): written, not yet tested; its selection key must follow this file"],
           "geometry_summary": geom, "counts": old["counts"], "conditions": cells}
    a.out.write_text(json.dumps(man, indent=1) + "\n"); print("wrote", a.out); print("v8 sha256", v8_sha[:16], "| conditions", len(cells), "| sha256", hashlib.sha256(a.out.read_bytes()).hexdigest()[:16])


if __name__ == "__main__":
    main()
