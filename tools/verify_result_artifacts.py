import json, hashlib, sys
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import find_manifest  # noqa: E402
T={
'three-plate v3.1':('three_plate_stack_strict_v3p1.json',{'SAP':'exp-results/contact-correction-20261109/three-plate/sap/sap_three_plate_strict_v3.json','MJ3.17':'exp-results/contact-correction-20261109/three-plate/mujoco/mjwarp_three_plate_strict_v3.json','MJ4.0':'exp-results/mujoco-warp-4p0/three-plate/mjwarp_three_plate_strict_v3.json'}),
'three-box v4':('three_box_push_strict_v4.json',{'SAP':'exp-results/aligned-generalizability-strict-v3/v4/sap_three_box_strict_v4.json','MJ3.17':'exp-results/contact-correction-20261109/three-box/mujoco/mjwarp_three_box_strict_v4.json','MJ4.0':'exp-results/mujoco-warp-4p0/three-box/mjwarp_three_box_strict_v4.json'}),
'bounce v2':('gauge_ball_bounce_dynamic_impact_v2.json',{'SAP':'exp-results/dynamic-impact-v2/sap_ball_dynamic_impact_v2.json','MJ3.17':'exp-results/dynamic-impact-v2/mjwarp_ball_dynamic_impact_v2.json'}),
'bounce v3':('gauge_ball_bounce_dynamic_impact_v3.json',{'SAP':'exp-results/dynamic-impact-v3/sap_ball_dynamic_impact_v3.json','MJ3.17':'exp-results/dynamic-impact-v3/mjwarp_ball_dynamic_impact_v3.json'}),
'cube toss v2':('contactnets_cube_toss_dynamic_impact_v2.json',{'SAP':'exp-results/dynamic-impact-v2/sap_cube_dynamic_impact_v2.json','MJ3.17':'exp-results/dynamic-impact-v2/mjwarp_cube_dynamic_impact_v2.json','MJ4.0':'exp-results/mujoco-warp-4p0/cube-toss/mjwarp_cube_dynamic_impact_v2.json'}),
'cube tail v3':('contactnets_cube_tail_sustained_v3.json',{'SAP':'exp-results/cube-tail-v3/sap_cube_tail_v3.json','MJ3.17':'exp-results/cube-tail-v3/mjwarp_cube_tail_v3.json'}),
'simple pinch v3':('simple_pinch_force_controlled_v3.json',{'SAP':'exp-results/simple-pinch-v3/sap_simple_pinch_v3_inertia_fixed.json','MJ3.17':'exp-results/simple-pinch-v3/mjwarp_simple_pinch_v3.json','MJ4.0':'exp-results/mujoco-warp-4p0/simple-pinch/mjwarp_simple_pinch_v3.json'}),
'GAUGE slope v3':('gauge_slope_slider_dynamic_friction_v3.json',{'SAP':'exp-results/gauge-slope-v3/sap_gauge_slope_v3.json','MJ3.17':'exp-results/gauge-slope-v3/mjwarp_gauge_slope_v3.json','MJ4.0':'exp-results/mujoco-warp-4p0/gauge-slope/mjwarp_gauge_slope_v3.json'}),
'MIT Push v3':('mit_push_dynamic_friction_v3.json',{'SAP':'exp-results/mit-push-v3/sap_mit_push_v3.json','MJ3.17':'exp-results/mit-push-v3/mjwarp_mit_push_v3.json','MJ4.0':'exp-results/mujoco-warp-4p0/mit-push/mjwarp_mit_push_v3.json'}),
'pendulum v2':('pendulum_energy_v2.json',{'SAP':'exp-results/pendulum-v2/sap_pendulum_v2.json','MJ3.17':'exp-results/pendulum-v2/mjwarp_pendulum_v2.json','MJ4.0':'exp-results/mujoco-warp-4p0/pendulum/mjwarp_pendulum_v2.json'}),
'pendulum GAUGE v3':('pendulum_gauge_v3.json',{'SAP':'exp-results/gauge-pendulum-v3/sap_gauge_pendulum_v3.json'}),
'pendulum GAUGE v3.1':('pendulum_gauge_real_v1.json',{'MJ3.17':'exp-results/gauge-real-317-400-20261110/317/pendulum/mjwarp_gauge_pendulum_real.json','MJ4.0':'exp-results/gauge-real-317-400-20261110/400/pendulum/mjwarp_gauge_pendulum_real.json'}),
'cradle v3':('newtons_cradle_energy_v3.json',{'SAP':'exp-results/newtons-cradle-v3/sap_cradle_v3.json','MJ3.17':'exp-results/newtons-cradle-v3/mjwarp_cradle_v3.json'}),
'cradle GAUGE v4.1':('newtons_cradle_gauge_real_v1.json',{'MJ3.17':'exp-results/gauge-real-317-400-20261110/317/cradle/mjwarp_gauge_cradle_real.json','MJ4.0':'exp-results/gauge-real-317-400-20261110/400/cradle/mjwarp_gauge_cradle_real.json'}),
}
def cid(c): return c.get('id') if isinstance(c,dict) else c
def hid(h):
    for k in ('cell','condition'):
        v=h.get(k)
        if isinstance(v,dict) and 'id' in v: return v['id']
    return h.get('id')
for t,(mf,E) in T.items():
    raw=open(find_manifest(mf),'rb').read(); sha=hashlib.sha256(raw).hexdigest(); m=json.loads(raw)
    held_ids=[c['id'] for c in m['held_out_conditions']]; seeds={}
    print(f'===== {t}  [{mf}]')
    for e,f in E.items():
        d=json.load(open(f)); issues=[]
        if d.get('manifest_sha256')!=sha: issues.append(f"SHA MISMATCH {str(d.get('manifest_sha256'))[:12]} vs {sha[:12]}")
        key='sap_candidates' if e=='SAP' else 'mujoco_warp_candidates'
        want=m['candidate_rule'][key]; got=[cid((s[0] if isinstance(s,list) else s).get('candidate')) for s in d.get('selection_candidates',[])]
        if len(got)!=len(want): issues.append(f'n candidates {len(got)} vs manifest {len(want)}')
        H=d.get('heldout',[]); ids=[hid(h) for h in H]
        if not H: issues.append('NO HELD-OUT')
        elif ids!=held_ids: issues.append(f'held ids differ {sorted(set(held_ids)^set(ids))}')
        seeds[e]=[h.get('seed') for h in H]
        if e=='SAP':
            pf=d.get('preflight') or {}; dev=(pf.get('devices') or {}).get('cuda:0') or {}
            if not (pf.get('all_pass') and dev and all(v.get('pass') for v in dev.values()) and (pf.get('cpu_gpu') or {}).get('pass')): issues.append('SAP preflight not embedded/passing')
        dv=d.get('device')
        if dv and 'cuda' not in str(dv): issues.append(f'device {dv}')
        print(f"  {e:6s} {'OK' if not issues else 'ISSUES: '+'; '.join(issues)}")
    s=[v for v in seeds.values() if v and any(x is not None for x in v)]
    if len(s)>1: print('  per-cell seeds identical across engines:', all(x==s[0] for x in s))
