"""P3 phase execution, P4 aggregation and P6 coverage without implicit passes."""
from __future__ import annotations

from pathlib import Path
from collections import defaultdict
import itertools

from .io_utils import GateError, digest, read_json, write_json
from .manifest import validate
from .matrix import CELL_IDS, ENGINES, cell_result, episode_result, permitted_claim, select_candidate
from .runner import gpu_host_gate, integrate, load_playback, package_hash
from .scene import catalog


def drive_phase(manifest,engine,input_root,out,api_evidence):
    gpu_host_gate();validate(manifest,input_root)
    if not api_evidence['passed'] or api_evidence['engine']!=engine:
        raise GateError('analytic API probe must pass before P3a')
    if api_evidence['runner_sha256']!=package_hash():raise GateError('API probe source revision changed')
    output=Path(out);output.mkdir(parents=True,exist_ok=True);rows=[]
    for ep in manifest['episodes']['selection']+manifest['episodes']['held_out']:
        path=output/ep['id'];assets=catalog(ep['scene'],path/'assets')
        rows.append({'episode_id':ep['id'],**integrate(ep['scene'],ep,load_playback(input_root,ep),assets,
                                                        engine,manifest['execution']['candidates'][engine][0],
                                                        path,objects=False)})
    report={'phase':'P3a','engine':engine,'manifest_sha256':manifest['declaration_sha256'],
            'runner_sha256':package_hash(),'rows':rows,'passed':all(r['drive_gate'] for r in rows),
            'objects_present':False}
    write_json(output/'drive.json',report);return report


def freeze_selection(manifest,engine,results,out):
    validate(manifest)
    candidates=manifest['execution']['candidates'][engine];ids=[e['id'] for e in manifest['episodes']['selection']]
    grouped=defaultdict(dict);times=defaultdict(float)
    for result in results:
        j=result['job']
        if result['manifest_sha256']!=manifest['declaration_sha256'] or result['runner_sha256']!=package_hash():
            raise GateError('selection source/manifest revision differs')
        if j['engine']!=engine or j['split']!='selection' or j['cell_id']!='nominal':
            raise GateError('non-selection row in P4')
        ci=next(i for i,c in enumerate(candidates) if c==j['candidate'])
        ep=j['episode_id']
        if ep in grouped[ci]:raise GateError('duplicate selection cell')
        grouped[ci][ep]=result['rows']
        times[ci]+=sum(r.get('wall_s',0) for r in result['rows'])
    complete=all(set(grouped[i])==set(ids) for i in range(len(candidates)))
    if not complete:raise GateError('all declared candidates and R=20 selection cells required')
    rows=[{'candidate_index':i,'episodes':grouped[i],'wall_s':times[i],'preflight_passed':True} for i in range(len(candidates))]
    winner=select_candidate(rows,ids)
    report={'phase':'P4','engine':engine,'complete':True,'manifest_sha256':manifest['declaration_sha256'],
            'runner_sha256':package_hash(),'winner_index':winner,
            'winner':candidates[winner] if winner is not None else None,
            'status':'winner frozen' if winner is not None else 'no candidate passes',
            'candidates':[{'id':candidates[r['candidate_index']]['id'],
                           'episodes':{ep:cell_result(raw) for ep,raw in r['episodes'].items()},
                           'wall_s':r['wall_s']} for r in rows]}
    if Path(out).exists():raise GateError('P4 winner is immutable; use a new revision')
    write_json(out,report);return report


def summarize(manifest,results,out):
    validate(manifest);grouped=defaultdict(dict)
    for r in results:
        if r['manifest_sha256']!=manifest['declaration_sha256']:raise GateError('report mixes manifests')
        j=r['job']
        if j['split']!='held_out':continue
        key=(j['episode_id'],j['engine'],j['budget'])
        if j['cell_id'] in grouped[key]:raise GateError('duplicate held-out cell')
        grouped[key][j['cell_id']]=cell_result(r['rows'])
    reports=[]
    for ep in manifest['episodes']['held_out']:
        for budget in (1,5):
            verdicts={}
            for engine in ENGINES:
                cells={c:grouped[(ep['id'],engine,budget)].get(c,{'status':'not run','correct':None,'denominator':20}) for c in CELL_IDS}
                verdicts[engine]=episode_result(cells)
                reports.append({'episode_id':ep['id'],'real_label':ep['real_label'],'engine':engine,'budget':budget,
                                'verdict':verdicts[engine],'nominal':cells['nominal'],'cells':cells})
            for a,b in itertools.combinations(ENGINES,2):
                reports.append({'episode_id':ep['id'],'budget':budget,'pair':[a,b],
                                'claim':permitted_claim(verdicts[a],verdicts[b],ep['real_label'])})
    labels={ep['real_label'] for ep in manifest['episodes']['held_out']}
    report={'phase':'P6','manifest_sha256':manifest['declaration_sha256'],'reports':reports,
            'class_balance':'not discriminative for the absent class' if len(labels)<2 else 'both real classes covered',
            'axes':'one-at-a-time; no claim about simultaneous completion changes',
            'penetration_claim_independent_review':'required before publication'}
    write_json(out,report);return report
