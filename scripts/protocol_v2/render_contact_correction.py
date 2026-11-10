"""Render the two corrected task cards from completed artifacts; print JSON for apply_patch."""
from __future__ import annotations

import html
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "exp-results/contact-correction-20261109"


def load(path):
    return json.loads(path.read_text())


def number(value, digits=3):
    return "undefined" if value is None else f"{value:.{digits}g}"


def percent(value):
    return number(None if value is None else 100 * value)


def params(candidate):
    return "<code>" + html.escape(candidate["id"]) + "</code>"


def plate_condition(cell):
    key = cell['id']
    if key.startswith('thickness'):
        return f'thickness {cell["thickness_mm"]:g} mm'
    if key.startswith('mass'):
        return f'mass {cell["mass_per_plate_kg"]:.3g} kg per plate'
    if key.startswith('radius'):
        return f'radius {cell["radius_mm"]:g} mm'
    if key.startswith('offset'):
        return f'interplate x offset {cell["interplate_x_offset_mm"]:g} mm'
    return f'dt={cell["dt_ms"]:g} ms; x offset={cell["interplate_x_offset_mm"]:g} mm'


def box_condition(cell):
    force = f'F/(mg)={cell["force_mg"]:g}' if 'force_mg' in cell else f'force / pushed-interface slip threshold={cell["force_ratio"]:g}'
    return (f'{cell["pushed_body"]} box, μ={cell["mu"]:g}, {force}<br>'
            f'm={cell["mass_kg"]:g} kg, half-width={cell["box_halfwidth_mm"]:g} mm, dt={cell["dt_ms"]:g} ms')


def table(headings, rows):
    return '<table class="metric-table"><thead><tr>' + ''.join(f'<th>{h}</th>' for h in headings) + '</tr></thead><tbody>' + ''.join('<tr>' + ''.join(f'<td>{c}</td>' for c in row) + '</tr>' for row in rows) + '</tbody></table>'


def bars(title, values):
    parts = []
    for i, (name, k, n) in enumerate(values):
        y = 76 + 78 * i
        parts.append(f'<text x="24" y="{y+19}">{name}</text><rect x="180" y="{y}" width="320" height="26" fill="#e7ecef"/><rect x="180" y="{y}" width="{320*k/n:.1f}" height="26" fill="#4a8d6b"/><text x="516" y="{y+19}">{k}/{n}</text>')
    return f'<svg viewBox="0 0 640 240" role="img" aria-label="{html.escape(title)}"><rect width="640" height="240" fill="#fff"/><g font-family="Arial" font-size="16"><text x="24" y="36" font-size="18" font-weight="700">{title}</text>{"".join(parts)}</g></svg>'


def first_figure(old_section):
    start = old_section.index('<figure class="figure">')
    return old_section[start:old_section.index('</figure>', start)+len('</figure>')]


def box_card(old):
    sap = load(ROOT / 'exp-results/aligned-generalizability-strict-v3/v4/sap_three_box_strict_v4.json')
    replay_path = BASE / 'three-box/mujoco/mjwarp_three_box_strict_v4.json'
    audited = replay_path.exists() and load(replay_path).get('complete', False)
    mj = load(replay_path) if audited else load(ROOT / 'exp-results/aligned-generalizability-strict-v3/v4/mjwarp_three_box_strict_v4.json')
    if not audited:
        # Presentation-only correction of known non-finite saved diagnostics.
        # The original historical artifact is never rewritten or given invented capacity data.
        for cell in mj['heldout']:
            for row in cell['rollouts']:
                row['finite'] = all(row.get(k) is not None for k in ('face_disp_final_mm','com_rel_disp_final_mm','tilt_max_deg','hold_face_creep_um_s'))
                row['contact_gate_pass'] = row['duty_pre_event'] >= .99
                if not row['finite']:
                    row['observed'], row['agrees'] = 'invalid', False
            cell['observed_counts'] = {k: sum(r['observed'] == k for r in cell['rollouts']) for k in ('retain','slip','tip','invalid')}
    datasets = [('SAP Warp', sap), ('MuJoCo Warp', mj)]
    total = lambda x: (sum(r['agree_and_valid_count'] for r in x['heldout']), 20*len(x['heldout']))
    core, summary = [], []
    for name, data in datasets:
        b1 = sum(r['agree_and_valid_count'] for r in data['B1'])
        selected = data.get('selection_replay', data['selection_candidates'][data['frozen_index']-1])
        b5 = sum(r['agree_and_valid_count'] for r in selected)
        retain = next(r for r in selected if r['expected'] == 'retain')
        core.append([name, f'literal native default: <strong>{b1}/40</strong>', f'{params(data["frozen_candidate"])}; <strong>{b5}/40</strong>', f'retain-cell hold creep: {number(retain["hold_face_creep_um_s_mean"])} µm/s'])
        k, n = total(data)
        qualifier = 'valid and correct' if name == 'SAP Warp' or audited else 'passed the recorded state/contact gates; capacity verification pending'
        summary.append([name, params(data['frozen_candidate']), f'<strong>{k}/{n} = {100*k/n:.1f}%</strong> {qualifier}'])
    raw = [r for c in mj['heldout'] for r in c['rollouts']]
    nonfinite = sum(not r['finite'] for r in raw)
    contact_fail = sum(r['finite'] and not r['contact_gate_pass'] and (not audited or r['capacity_ok']) for r in raw)
    capacity_fail = sum(not r['capacity_ok'] for r in raw) if audited else None
    finite = sum(r['finite'] for r in raw)
    classified = sum(r['finite'] and r['agrees'] for r in raw)
    if audited:
        cap = [c['capacity_audit'] for c in mj['heldout']]
        peak = max(c['max_active_contacts'] for c in cap)
        allocation = min(c['allocated_contacts'] for c in cap)
        audit_text = f'Capacity-overflow rollouts: {capacity_fail}. Every physics step is checked; peak active contacts {peak}, allocated shared slots {allocation}. Solver-iteration warnings are recorded separately.'
    else:
        audit_text = 'Historical capacity telemetry is unavailable: the driver supplied a literal zero. The corrected per-step audit and NaN classifier pass all three GPU regression tests. The full frozen-setting replay was launched, but its result is pending retrieval after the remote connection timed out. No zero-overflow conclusion is made from the historical artifact.'
    schematic = first_figure(old).replace('The pushed interface must stay stuck below, and break away above, its analytic Coulomb threshold.', 'A horizontal COM load can cause retain, slip or tip. The first observed event determines the class.')
    visual = bars('Frozen B=5: recorded-gate agreement', [(name, *total(data)) for name, data in datasets])
    failures = [[box_condition(c['cell']), f'{c["agree_and_valid_count"]}/20', str(c['observed_counts'].get('invalid', 0)), number(c['duty_pre_event_min'])] for c in mj['heldout'] if c['agree_and_valid_count'] < 20]
    failure_table = table(['MuJoCo held-out condition', 'Valid and correct' if audited else 'Passed recorded gates', 'Non-finite / invalid class', 'Minimum pre-event contact duty'], failures) if failures else '<p>No held-out failure was observed in this rerun.</p>'
    selection_note = 'The MuJoCo B=5 row is a new replay of the originally selected setting.' if audited else 'Both B=1 and B=5 selection rows below are historical; the instrumented replay output has not yet been retrieved.'
    replay_note = 'The original MuJoCo frozen B=5 setting was replayed with per-step capacity and finite-state checks.' if audited else 'A replay of the original MuJoCo frozen B=5 setting with per-step capacity and finite-state checks was launched; its output is pending retrieval.'
    return f'''      <section id="three-box" class="report-section task-card"><h2>3.1 Three-box push</h2>
<h3>1. Task setup</h3><p>Three identical boxes start in a centred vertical stack. A horizontal force acts at the middle or top box COM after 0.6 s of settling, ramps for 0.4 s and holds for 0.65 s. Baseline: 1 kg per box, 20-mm half-width, dt=0.5 ms. Labels include <strong>retain, slip and tip</strong>; analytic slip and tipping thresholds define cases with a margin from the competing event. Classification uses the first event: contact-face displacement ≥1 mm or box tilt &gt;10°. A non-finite state is <strong>invalid</strong> and never counts as correct.</p>
<div class="task-figures">{schematic}<figure class="figure">{visual}<figcaption>Frozen B=5 on 25 declared conditions × R=20. Invalid simulations remain in the 500-rollout denominator.</figcaption></figure></div>
{table(['Parameter or metric','Definition'], [['Physical sweep','box mass 0.5/1/2 kg; friction μ=0.3/0.5/0.7/0.9; box half-width 15/20/30 mm; middle/top loads; dt=0.25/0.5/1 ms. These are the declared 25 combinations, not a full Cartesian grid.'], ['Valid and correct','finite state, no capacity overflow, pre-event pushed-interface contact duty ≥99%, and class matches the analytic label'], ['Creep','slope of contact-face displacement over the final 0.4-s hold; the retain selection cell is reported separately from sliding cases']])}
<h3>2. B=1 → B=5 core evidence</h3><p>The original selection uses two excluded top-box conditions at μ=0.3: force ratios 0.7 (retain) and 1.3 (slip), R=20 each. Mass, geometry, force command and dt are fixed across all five numerical candidates. B=1 values below come from that historical selection. {selection_note} B=1 was not evaluated on the full 25-cell held-out grid.</p>
{table(['Engine','B=1 on selection','Selected B=5 on selection','Representative mechanism metric'], core)}
<p>SAP changes native contact stiffness/relaxation and friction regularisation. MuJoCo changes normal-contact stiffness, friction impedance and NoSlip correction. Each configuration is selected as a whole; these runs do not isolate a single parameter’s causal effect.</p>
<h3>3. Generalizability</h3><p>Both engines use the same strict-v4 manifest and R=20 perturbation scheme for 25 held-out conditions. SAP's existing finite result is retained. {replay_note} Numerical parameters are unchanged. A separate selection replication changed candidate ordering, illustrating run-to-run variation, and is not used to retune this audit.</p>
{table(['Engine','Frozen setting','Held-out result'], summary)}
<p><strong>MuJoCo validity breakdown:</strong> {nonfinite} rollouts have non-finite state/diagnostic evidence; {contact_fail} other rollouts fail contact continuity. Among {finite} rollouts with finite recorded diagnostics, {classified} receive the expected class. The old “500/500 classifications correct” wording is withdrawn; non-finite cases are invalid and remain in the 500 denominator. {audit_text}</p>
{failure_table}<p>The conclusion covers this finite-horizon grid and declared five-candidate budget. Artifact: <a href="https://github.com/Xiaowen2024/sap-exp/tree/main/exp-results/contact-correction-20261109">capacity-instrumented rerun and provenance</a>.</p></section>'''


def plate_card(old):
    sap_path = BASE / 'three-plate/sap/sap_three_plate_strict_v3.json'
    sap_current = sap_path.exists()
    sap = load(sap_path) if sap_current else load(ROOT / 'exp-results/aligned-generalizability-strict-v3/sap_three_plate_strict_v3.json')
    mj = load(BASE / 'three-plate/mujoco/mjwarp_three_plate_strict_v3.json')
    datasets = [('SAP Warp', sap), ('MuJoCo Warp', mj)]
    core, coverage, detail = [], [], []
    for name, data in datasets:
        b1 = data['B1']; b5 = data['selection_candidates'][data['frozen_index']-1]
        core.append([name, f'literal native default: <strong>{b1["pass_count"]}/20</strong>', f'{params(data["frozen_candidate"])}: <strong>{b5["pass_count"]}/20</strong>', f'overlap {number(b1["max_overlap_mm_mean"])} → {number(b5["max_overlap_mm_mean"])} mm; maximum load error {percent(b1["load_error_max_mean"])}% → {percent(b5["load_error_max_mean"])}%'])
        coverage.append([name, params(data['frozen_candidate']), f'<strong>{sum(c["pass_count"] for c in data["heldout"])}/240</strong>'])
        for c in data['heldout']:
            gates = '; '.join(f'{k}: {v}' for k, v in c['failed_gate_counts'].items() if v) or 'none'
            detail.append([name, plate_condition(c['cell']), f'{c["pass_count"]}/20', number(c['max_overlap_mm_mean']), percent(c['load_error_max_mean']), gates])
    schematic = first_figure(old)
    visual = bars('Friction-aligned frozen B=5 (μ=0.8)', [(name, sum(c['pass_count'] for c in data['heldout']), 240) for name, data in datasets])
    counts = ', '.join(str(r['pass_count']) for r in mj['selection_candidates'])
    run_status = 'Both fresh reruns are complete.' if sap_current else 'MuJoCo rerun is complete (128/240). SAP rows below are its prior μ=0.8 result (240/240); the new SAP rerun was launched, but its output is pending retrieval after the remote connection timed out. The fresh paired comparison is not yet certified complete.'
    return f'''      <section id="three-plate" class="report-section task-card"><h2>4.1 Three-plate stack</h2>
<h3>1. Task setup</h3><p>Three circular plates start at rest on a plane, without a drop phase. Baseline: thickness h=4 mm, radius=100 mm, mass=1/3 kg per plate, dt=0.5 ms. <strong>Ground and all plates have sliding friction μ=0.8 in both engines.</strong> Each run lasts 2 s with R=20 independent shared xy/yaw perturbations. The former strict-v3 MuJoCo result used ground μ=1.0 and is superseded for cross-engine comparison.</p><p><strong>Run completion:</strong> {run_status}</p>
<div class="task-figures">{schematic}<figure class="figure">{visual}<figcaption>MuJoCo: fresh five-candidate selection and 12 frozen cells after aligning ground friction. SAP: {"fresh rerun" if sap_current else "prior run at the same μ=0.8, labelled pending confirmation by the fresh rerun"}.</figcaption></figure></div>
{table(['Parameter or gate','Definition'], [['Physical sweep','thickness 0.5/4/5/20 mm; plate mass 1/6, 1/3, 2/3 kg; radius 75/100/125 mm; interplate x offset 0/0.5/1 mm; dt=0.25/0.5/1 ms'], ['Pass','all gates simultaneously: correct plate order, sampled maximum centre-height overlap ≤5% of h, maximum interface load error ≤1%, and settled final 0.2 s (every plate |v|&lt;0.01 m/s and |ω|&lt;0.05 rad/s), finite state and no capacity truncation'], ['Overlap gate at calibration','5% × 4 mm = 0.20 mm; the allowed overlap scales with thickness'], ['Normal load','interface expectations are 3mg, 2mg, mg from bottom to top; each load is averaged over the final 1 s']])}
<h3>2. B=1 → B=5 core evidence</h3><p>Five numerical candidates per engine are selected only on the baseline 4-mm stack. Physical friction and dt remain fixed. B=1 and B=5 below use the same 20 selection perturbations.</p>
{table(['Engine','B=1 selection','Frozen B=5 selection','Selection mean maximum overlap / load error'], core)}
<p>MuJoCo's five selection pass counts in this corrected run are {counts} out of 20. A low best count means this declared candidate screen did not yield consistently passing calibration; it does not identify the cause or establish an engine-wide limit.</p>
<h3>3. Generalizability</h3><p>Coverage is matched: the selected setting is frozen for 12 declared conditions × R=20 per engine. The baseline calibration condition is excluded. The offset_0p5 and dt_0p5 entries repeat the same physical setting with different perturbations, so there are 11 distinct physical settings. Axis sweeps are not a full Cartesian grid.</p>
{table(['Engine','Frozen setting','Passed all gates'], coverage)}
{table(['Engine','Changed condition (others baseline)','Pass','Mean max overlap (mm)','Mean max load error (%)','Failed gates (out of 20)'], detail)}
<p>Gate-failure counts may overlap. “undefined” means the aggregate contains a non-finite value. The overlap metric is based on sampled centre heights, not an all-step geometric penetration witness. Full per-rollout data, preflight and provenance: <a href="https://github.com/Xiaowen2024/sap-exp/tree/main/exp-results/contact-correction-20261109">friction-aligned rerun</a>.</p></section>'''


def main():
    doc = (ROOT / 'docs/mechanism-eval.html').read_text()
    patches = []
    for name, fn in [('three-box', box_card), ('three-plate', plate_card)]:
        start = doc.index(f'      <section id="{name}"')
        end = doc.index('</section>', start) + len('</section>')
        old = doc[start:end]
        patches.append({'id': name, 'old': old, 'new': fn(old)})
    print(json.dumps(patches))


if __name__ == '__main__':
    main()
