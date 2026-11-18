"""Offline phase diagnostics for the once-only parameter transfer test."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from .laptop_evaluate import longest_false


def main(root):
    root=Path(root);sc=json.loads((root/'scene_v1/scene.json').read_text());results={}
    for engine in ['mujoco_warp','sap']:
        path=root/'runs'/(engine+'_it1');tr=np.load(path/'dense_trace.npz');co=np.load(path/'contact_pairs.npz');h=json.loads((path/'health.json').read_text());t=tr['time'];ct=co['time'];names=list(co['pair_names']);p=tr['laptop__pose'];v=tr['laptop__velocity'];dt=sc['dt']
        def force(a,b):
            n=a+'/'+b if a+'/'+b in names else b+'/'+a;return co['normal_force_sum_N'][:,names.index(n)]
        body={}
        for side,window in [('left',sc['phases']['left_grasp_s']),('right',sc['phases']['grasp_s'])]:
            lo,hi=window;mask=(t>=lo)&(t<=hi);base=tr[side+'_base__pose'][mask];r=R.from_quat(base[:,[4,5,6,3]]);relative=r.inv().apply(p[mask,:3]-base[:,:3]);rr=r.inv()*R.from_quat(p[mask][:,[4,5,6,3]]);contact=(ct>=lo)&(ct<hi);both=(force('laptop',side+'_link7')>.01)&(force('laptop',side+'_link8')>.01)
            body[side]={'window_s':window,'max_COM_change_in_gripper_frame_mm':float(np.linalg.norm(relative-relative[0],axis=1).max()*1000),'max_relative_rotation_change_deg':float(np.rad2deg((rr*rr[0].inv()).magnitude()).max()),'two_finger_force_duty':float(both[contact].mean()),'longest_two_finger_force_loss_s':longest_false(both[contact],dt)}
        left=(force('laptop','left_link7')>.01)|(force('laptop','left_link8')>.01);right=(force('laptop','right_link7')>.01)|(force('laptop','right_link8')>.01);handoff=(ct>=7.6)&(ct<8.5);static=(t>=0)&(t<=2.0);fixtures={}
        for side in ['left','right']:
            for link in ['link7','link8']:
                f=force(side+'_'+link,'laptop_stand');fixtures[side+'_'+link]={'peak_normal_force_N':float(f.max()),'peak_time_s':float(ct[np.argmax(f)])}
        result={'engine':h['engine'],'finite':h['finite'],'steps':h['steps'],'expected_steps':h['expected_steps'],'max_lift_COM_m':float(p[:,2].max()-p[0,2]),'before_pickup_2s_COM_drift_mm':float(np.linalg.norm(p[static,:3]-p[0,:3],axis=1).max()*1000),'before_pickup_2s_z_range_mm':float(np.ptp(p[static,2])*1000),'grasp_phases':body,'handoff':{'window_s':[7.6,8.5],'longest_no_hand_normal_force_s':longest_false((left|right)[handoff],dt),'both_hands_force_duty':float((left&right)[handoff].mean())},'gripper_stand_contacts':fixtures,'maximum_object_speed_m_s':float(np.linalg.norm(v[:,:3],axis=1).max()),'COM_at_phases':{str(x):p[int(np.argmin(abs(t-x))),:3].tolist() for x in [0,3.2,7.6,8.5,12.96,float(t[-1])]},'outcome':None,'note':'phase changes are descriptive, not object-reference trajectory errors; inspect video and geometry before outcome judgement'}
        (path/'transfer_stages.json').write_text(json.dumps(result,indent=2));results[engine]=result;print(engine,'lift',result['max_lift_COM_m'],'grasp',body,'final',result['COM_at_phases'][str(float(t[-1]))],flush=True)
    (root/'transfer_stages.json').write_text(json.dumps(results,indent=2))

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--root',required=True);main(a.parse_args().root)
