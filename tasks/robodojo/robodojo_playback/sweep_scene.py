"""Freeze the shared full-flow scene after source-only reconstruction/audit."""
import argparse,json,hashlib,shutil
from pathlib import Path
import numpy as np,trimesh
from scipy.spatial.transform import Rotation as R,Slerp


def main(root,x5):
 root=Path(root);x5=Path(x5);out=root/'scene_v1';out.mkdir(exist_ok=True);fit=json.loads((root/'source_fit.json').read_text());template=json.loads((root/'x5_template_scene.json').read_text());assets={};bodies=[];adjustments={}
 for name,folder,source,mass in [('broom','broom_pieces','broom_0',.04),('broom_shovel','broom_shovel_regions_v2','broom_shovel_0',.15),('cube','', 'small_cube_0',.03)]:
  files=sorted((root/'geometry'/folder).glob('*.obj')) if folder else [root/'geometry/small_cube_0.obj'];pcs=[trimesh.load(p,force='mesh') for p in files];agg=trimesh.util.concatenate(pcs);prop=agg.mass_properties;com=prop.center_mass;I=prop.inertia*mass/prop.mass;paths=[]
  assert np.linalg.eigvalsh(I).min()>0
  for j,m in enumerate(pcs):
   assert len(m.vertices)<=64,(name,len(m.vertices));m.vertices-=com;p=out/name/f'piece_{j:03d}.obj';p.parent.mkdir(exist_ok=True);m.export(p);paths.append(str(p.relative_to(out)))
  assets[name]={'pieces':paths,'mass':mass,'com':com.tolist(),'inertia':I.tolist(),'source_mass':'official metadata','inertia_method':'common additive convex mass properties scaled to source mass; native cooked inertia unavailable'}
  for obj in ([name] if name!='cube' else ['cube_green','cube_cyan','cube_yellow']):
   f=fit[obj];rr=R.from_quat(np.array(f['q_wxyz'])[[1,2,3,0]]);origin=np.array(f['origin_m']);minimum=min(rr.apply(m.vertices+com)[:,2].min()+origin[2] for m in pcs);shift=max(0.,.7652-minimum);origin[2]+=shift;adjustments[obj]={'vertical_initial_clearance_adjustment_m':shift,'final_table_gap_m':minimum+shift-.765}
   bodies.append({'name':obj,'asset':name,'p':(origin+rr.apply(com)).tolist(),'q':f['q_wxyz'],'driven':False,'origin_offset':com.tolist()})
 for name in ['base','link7','link8']:
  assets[name]=template['assets'][name]
  for path in assets[name]['pieces']:
   dst=out/path;dst.parent.mkdir(exist_ok=True);shutil.copy(x5/path,dst)
 d=np.load(root/'source/episode/numeric.npz');hz=float(d['additional_info__frequency']);N=len(d['state__left_ee_poses']);dt=.001;duration=(N-1)/hz;steps=int(round((duration+1.5)/dt));steps+=(-steps)%10;t=np.arange(steps+1)*dt;src=np.arange(N)/hz;cl=np.minimum(t,src[-1]);target={'time':t}
 for side in ['left','right']:
  P=d['state__'+side+'_ee_poses'];pos=np.column_stack([np.interp(cl,src,P[:,j]) for j in range(3)]);rot=Slerp(src,R.from_quat(P[:,[4,5,6,3]]))(cl);q=rot.as_quat()[:,[3,0,1,2]];v=np.gradient(pos,dt,axis=0);w=np.vstack([(rot[1:]*rot[:-1].inv()).as_rotvec()/dt,[0,0,0]])
  target[side+'_base__pose']=np.c_[pos,q];target[side+'_base__velocity']=np.c_[v,w];cmd=-.01+.054*np.interp(cl,src,d['action__'+side+'_ee_joint_states'][:,0]);target['command_'+side]=cmd
  bodies.append(dict(name=side+'_base',asset='base',p=pos[0].tolist(),q=q[0].tolist(),driven=True,is_gripper=True))
  for old in template['bodies']:
   if old.get('parent')!=side+'_base':continue
   b=dict(old);initial=float(np.clip(cmd[0],0,.044));b['joint_initial']=initial;b['p']=(pos[0]+rot[0].apply(np.array(b['zero_p'])+np.array(b['axis'])*initial)).tolist();b['q']=q[0].tolist();bodies.append(b)
 scene=dict(kind='reduced_full_sweep_blocks',task='sweep_blocks',episode=0,dt=dt,gravity=[0,0,-9.81],mu=.3,table_top_z=.765,initial_gap_m=.0002,source_start_frame=0,source_end_frame=N-1,source_hz=hz,source_start_s=0.,source_duration_s=duration,tail_s=steps*dt-duration,assets=assets,bodies=bodies,static_bodies=[],articulated_grippers=True,contact_capacity=4096,explicit_finger_self_collision_exclusions=True,drive=template['drive'],finger_actuator=template['finger_actuator'],source_control_audit=template['source_control_audit'],telemetry_nodes=['broom','broom_shovel','cube_green','cube_cyan','cube_yellow','left_link7','left_link8','right_link7','right_link8','table'],phases={'left_broom_grasp_s':[3.,6.5],'right_broom_grasp_s':[7.2,26.7],'left_pan_grasp_s':[13.3,26.7],'source_final_s':[27.6,duration],'tail_final_s':[steps*dt-.5,steps*dt]},contact_windows=[['broom','left_link7','left_broom_grasp_s'],['broom','left_link8','left_broom_grasp_s'],['broom','right_link7','right_broom_grasp_s'],['broom','right_link8','right_broom_grasp_s'],['broom_shovel','left_link7','left_pan_grasp_s'],['broom_shovel','left_link8','left_pan_grasp_s']],reconstruction=fit,initial_clearance_adjustments=adjustments,deviations=['official simulation reference, not real-world ground truth','initial source-only fitted object poses and table support assumptions, not released object states','detached physical X5 grippers replace full robot arms; shared source-command PD and base servo','common rigid broom and pan convex regions; no deformable bristles; source also declares rigid tools but PhysX cooked hulls unavailable','common Coulomb mu=.3 on every pair; source metadata broom=.45, pan/cubes=.3; source per-material combine not ported; no torsional or rolling friction','all small-cube collision meshes measure 19.25 mm despite metadata size 35 mm; all share .03 kg mass; colours identify instances but source asset IDs unconfirmed','closed geometry <=64 vertices per convex piece, no engine vertex truncation','1.5 s appended hold only diagnostic; one episode, each candidate once'])
 (out/'scene.json').write_text(json.dumps(scene,indent=2));np.savez_compressed(out/'targets.npz',**target);(root/'scene_manifest.json').write_text(json.dumps({str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()},indent=2));print('frozen steps',steps,'pieces',{k:len(v['pieces']) for k,v in assets.items()},'initial adjustments',adjustments,flush=True)
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('root');a.add_argument('x5');v=a.parse_args();main(v.root,v.x5)
