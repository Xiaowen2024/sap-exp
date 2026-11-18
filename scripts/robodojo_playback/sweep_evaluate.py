"""Offline existing-trace metrics for Sweep Blocks. No simulator import/step."""
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np,trimesh
from scipy.spatial import ConvexHull
from scipy.optimize import linprog
from scipy.spatial.transform import Rotation as R
from .common import load
from .laptop_evaluate import contact_metrics,rms

OBJECTS=['broom','broom_shovel','cube_green','cube_cyan','cube_yellow']

def geometry(scene,tr):
 nodes=scene['telemetry_nodes'];bs={b['name']:b for b in scene['bodies']};local={}
 for name in nodes:
  meshes=[trimesh.creation.box(extents=[1.4,1.1,.05])] if name=='table' else [trimesh.load(p,force='mesh') for p in scene['assets'][bs[name]['asset']]['pieces']]
  local[name]=[]
  for m in meshes:
   h=ConvexHull(m.vertices);eq=h.equations;_,j=np.unique(np.round(eq,9),axis=0,return_index=True);local[name].append((np.asarray(m.vertices)[h.vertices],eq[np.sort(j)]))
 pairs=[(a,b) for i,a in enumerate(nodes) for b in nodes[i+1:] if a in OBJECTS or b in OBJECTS];peaks={a+'/'+b:{'sampled_peak_upper_bound_m':0.,'lower_bound_witness_m':0.,'upper_peak_time_s':None,'witness_components':None} for a,b in pairs};dt=scene['dt'];eps=4e-7;checked=0
 for k in range(0,len(tr['time']),10):
  world={}
  for name,pcs in local.items():
   if name=='table':rot=np.eye(3);p=np.array([0.,-.05,.74])
   else:q=tr[name+'__pose'][k];rot=R.from_quat(q[[4,5,6,3]]).as_matrix();p=q[:3]
   world[name]=[]
   for v,e in pcs:
    v=v@rot.T+p;nn=e[:,:3]@rot.T;eq=np.c_[nn,e[:,3]-nn@p];world[name].append((v,eq,v.min(0),v.max(0)))
  for a,b in pairs:
   peak=peaks[a+'/'+b]
   for j,(av,ae,alo,ahi) in enumerate(world[a]):
    for l,(bv,be,blo,bhi) in enumerate(world[b]):
     if np.any(ahi<=blo) or np.any(bhi<=alo):continue
     upper=min(float(np.min(-ae[:,3]-np.min(bv@ae[:,:3].T,axis=0))),float(np.min(-be[:,3]-np.min(av@be[:,:3].T,axis=0))))+eps
     if upper<=max(eps,peak['sampled_peak_upper_bound_m']):continue
     ee=np.vstack([ae,be]);lp=linprog([0,0,0,-1.],A_ub=np.c_[ee[:,:3],np.ones(len(ee))],b_ub=-ee[:,3],bounds=[(None,None)]*4,method='highs');checked+=1
     if not lp.success:raise RuntimeError('intersection LP failed')
     radius=float(lp.x[3])
     if radius<=1e-8:continue
     lower=max(0.,2*radius-eps);assert lower<=upper+1e-7
     peak.update(sampled_peak_upper_bound_m=upper,lower_bound_witness_m=max(peak['lower_bound_witness_m'],lower),upper_peak_time_s=float(tr['time'][k]),witness_components=[j,l])
  if k%5000==0:print('offline geometry',k,len(tr['time']),'LP',checked,flush=True)
 return {'method':'common convex component-pair face-normal separation upper bounds; LP intersection tests; inscribed-ball lower witnesses at increasing upper maxima','sample_dt_s':.01,'samples':len(range(0,len(tr['time']),10)),'LP_tests':checked,'pairs':peaks,'note':'lower witness is a valid lower bound on sampled peak, not necessarily its tightest lower bound; upper bound is not exact union penetration; no continuous-time or wall-crossing audit','wall_crossing_audit':None,'local_wall_thickness_audit':None}

def settling(scene,tr):
 out={}
 for name in OBJECTS:
  out[name]={}
  for window in ['source_final_s','tail_final_s']:
   lo,hi=scene['phases'][window];mask=(tr['time']>=lo-1e-8)&(tr['time']<=hi+1e-8);t=tr['time'][mask];p=tr[name+'__pose'][mask];v=tr[name+'__velocity'][mask]
   out[name][window]={'window_s':[lo,hi],'samples':len(t),'linear_speed_rms_mm_s':rms(t,v[:,:3])*1000,'angular_speed_rms_rad_s':rms(t,v[:,3:]),'net_COM_drift_mm':float(np.linalg.norm(p[-1,:3]-p[0,:3])*1000),'linear_speed_peak_mm_s':float(np.linalg.norm(v[:,:3],axis=1).max()*1000)}
 return out

def main(root,runs,skip_geometry=False):
 root=Path(root);scene,_=load(root/'scene_v1');out=root/'metrics.json';allresults=json.loads(out.read_text())['results'] if out.exists() else {}
 for name in runs:
  run=root/'runs'/name;health=json.loads((run/'health.json').read_text());tr=np.load(run/'dense_trace.npz');contacts=np.load(run/'contact_pairs.npz');start=time.perf_counter();settle=settling(scene,tr);contact=contact_metrics(scene,contacts);g=None if skip_geometry else geometry(scene,tr)
  diag={}
  for tool in ['broom','broom_shovel']:
   z=tr[tool+'__pose'][:,2];diag[tool]={'max_COM_lift_m':float(z.max()-z[0]),'source_final_COM_m':tr[tool+'__pose'][int(round(scene['source_duration_s']/scene['dt'])),:3].tolist()}
  pan=tr['broom_shovel__pose'];cube={}
  for obj in OBJECTS[2:]:
   ps=tr[obj+'__pose'];rel=R.from_quat(pan[:,[4,5,6,3]]).inv().apply(ps[:,:3]-pan[:,:3]);pancom=np.array(scene['assets']['broom_shovel']['com']);rel+=pancom
   cube[obj]={'source_final_COM_in_pan_asset_frame_m':rel[int(round(scene['source_duration_s']/scene['dt']))].tolist(),'tail_final_COM_in_pan_asset_frame_m':rel[-1].tolist(),'source_final_COM_m':ps[int(round(scene['source_duration_s']/scene['dt'])),:3].tolist(),'net_world_displacement_m':float(np.linalg.norm(ps[-1,:3]-ps[0,:3]))}
  tm=contacts['time'];release=tm>=27.6;pn=list(contacts['pair_names']);force=contacts['normal_force_sum_N'];grip=max(float(force[release,j].max()) for j,p in enumerate(pn) if any(o in p for o in OBJECTS) and ('link7' in p or 'link8' in p))
  result={'name':name,'health':health,'geometry':g,'contact_continuity':contact,'post_release_settling':settle,'tool_motion_diagnostics':diag,'cube_relative_diagnostics':cube,'post_release_gripper_normal_force_peak_N':grip,'physical_fidelity':{'object_trajectory_error_m':None,'final_pose_error_m':None,'reason':'no admitted independent 3D reference with coverage and uncertainty; initial reconstruction is not trajectory truth'},'task_outcome':{'observed_sweep':None,'observed_retained_cube_count':None,'valid_success':None,'outcome_agreement':None,'reason':'pending recorded-state video review and geometry validity; no official policy reward gate'},'input_hashes':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [run/'dense_trace.npz',run/'contact_pairs.npz',run/'input_scene.json']},'offline_wall_s':time.perf_counter()-start}
  (run/'sop_metrics.json').write_text(json.dumps(result,indent=2));allresults[name]=result;print(name,'evaluation done','tools',diag,'cubes',cube,flush=True)
 out.write_text(json.dumps({'protocol':'robodojo-sweep-blocks-v1','R':1,'results':allresults},indent=2))
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--root',required=True);a.add_argument('--runs',nargs='+',required=True);a.add_argument('--skip-geometry',action='store_true');v=a.parse_args();main(v.root,v.runs,v.skip_geometry)
