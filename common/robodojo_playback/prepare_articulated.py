"""Rebuild the official X5 finger hulls, joint limits and command semantics."""
import argparse
import json
from pathlib import Path
import numpy as np
import trimesh
from pxr import Usd,UsdGeom
from scipy.spatial.transform import Rotation as R,Slerp


def main(root):
    root=Path(root);out=root/'articulated';out.mkdir(exist_ok=True)
    scene=json.loads((root/'prepared/scene.json').read_text());data=np.load(root/'stack_bowls/numeric.npz',allow_pickle=False)
    stage=Usd.Stage.Open(str(root/'ARX.usd'));assets={'bowl':scene['assets']['bowl']};assets['bowl']['pieces']=['../prepared/'+p for p in assets['bowl']['pieces']]
    for link in ('link7','link8'):
        body=stage.GetPrimAtPath('/X5A/'+link)
        prim=stage.GetPrimAtPath(f'/X5A/{link}/collisions/{link}/node_STL_BINARY_/mesh')
        v=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(),float)
        local=np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)*UsdGeom.Xformable(body).ComputeLocalToWorldTransform(0).GetInverse()).T
        v=v@local[:3,:3].T+local[:3,3]
        hull=trimesh.Trimesh(vertices=v,process=True).convex_hull;props=hull.mass_properties;com=props.center_mass
        mass=float(body.GetAttribute('physics:mass').Get());hull.vertices-=com
        folder=out/link;folder.mkdir(exist_ok=True);hull.export(folder/'hull.obj')
        assets[link]={'pieces':[link+'/hull.obj'],'mass':mass,'com':com.tolist(),
                      'inertia':(props.inertia*mass/props.mass).tolist(),'additive_volume_m3':float(props.volume),
                      'collision_representation':'one convex hull, as specified by the official robot USD'}
    mass,I=1.,.01;cube=trimesh.creation.box(extents=[np.sqrt(6*I/mass)]*3);cube.export(out/'base.obj')
    assets['base']={'pieces':['base.obj'],'mass':mass,'com':[0.,0.,0.],'inertia':(np.eye(3)*I).tolist(),'additive_volume_m3':float(cube.volume)}
    n=len(data['state__right_ee_poses']);source_t=np.arange(n)/25;times=np.arange(10561)*.001;clamped=np.minimum(times,source_t[-1]);targets={'time':times}
    bodies=[b for b in scene['bodies'] if b['asset']=='bowl']
    for side in ('left','right'):
        poses=data[f'state__{side}_ee_poses'];pos=np.column_stack([np.interp(clamped,source_t,poses[:,i]) for i in range(3)])
        rot=Slerp(source_t,R.from_quat(poses[:,[4,5,6,3]]))(clamped);q=rot.as_quat()[:,[3,0,1,2]]
        v=np.gradient(pos,.001,axis=0);w=(rot[1:]*rot[:-1].inv()).as_rotvec()/.001;w=np.vstack([w,w[-1]])
        name=side+'_base';targets[name+'__pose']=np.c_[pos,q];targets[name+'__velocity']=np.c_[v,w]
        # The export shifts action to the next target. Feed it to the physical
        # actuator; do not turn an out-of-limit command into a measured pose.
        cmd=-.01+.054*np.interp(clamped,source_t,data[f'action__{side}_ee_joint_states'][:,0]);targets['command_'+side]=cmd
        bodies.append({'name':name,'asset':'base','p':pos[0].tolist(),'q':q[0].tolist(),'driven':True,'is_gripper':True})
        for link,sign in (('link7',1),('link8',-1)):
            joint=stage.GetPrimAtPath('/X5A/joints/joint'+link[-1]);origin=np.asarray(joint.GetAttribute('physics:localPos0').Get(),float)
            zero=origin+np.asarray(assets[link]['com']);initial=float(np.clip(cmd[0],0.,.044));local=zero+[0,sign*initial,0]
            bodies.append({'name':side+'_'+link,'asset':link,'parent':name,'zero_p':zero.tolist(),'axis':[0.,float(sign),0.],
                           'joint_initial':initial,'joint_lower':0.,'joint_upper':.044,'p':(pos[0]+rot[0].apply(local)).tolist(),'q':q[0].tolist(),
                           'driven':False,'is_gripper':True})
    scene.update(assets=assets,bodies=bodies,articulated_grippers=True,contact_capacity=16384,
                 drive={'Kt':20000.,'Kr':20.,'force_cap_N':200.,'torque_cap_Nm':20.},
                 finger_actuator={'Kp':2300.,'Kd':100.,'force_cap_N':100.},
                 ipc_parameters={'d_hat':.0001,'eps_velocity':.001,'velocity_tol':.0001})
    scene['deviations']=[x for x in scene['deviations'] if not x.startswith('independent spring')]
    scene['deviations']+=['detached free gripper bases replace robot arms; same base servo in all engines',
                          'official gripper hulls, joint limits and actuator gains; inertia matched from common hull density',
                          'IPC C5 geometry-scale diagnostic; native d_hat=10mm initial state was not admitted; no equal-budget ranking']
    scene['source_control_audit']={'source_commit':'fbfbbcb3801397fa00dcffaef3ca64d97c3fe76f',
        'gripper_state_semantics':'current code writes prev_control target into both state and action; raw normalized command is not measured joint width',
        'source_file':'env/observation_manager/obs_manager.py:188-215','joint_limits_m':[0.,.044],
        'target_mapping_m':'-.01 + .054 * normalized command','actuator_source':'env/robot_manager/robot_config/x5.py',
        'official_actuator':{'stiffness':2300,'damping':100,'effort_limit_sim':100},
        'robot_gravity':'disabled per body, matching the official spawn config'}
    (out/'scene.json').write_text(json.dumps(scene,indent=2));np.savez_compressed(out/'targets.npz',**targets)
    print('prepared articulated scene',len(bodies),'bodies',flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('root');main(ap.parse_args().root)
