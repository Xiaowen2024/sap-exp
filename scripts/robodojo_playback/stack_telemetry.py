"""Read-only, native-step telemetry for the Stack Bowls evaluation SOP.

Enabled only by ROBODOJO_STACK_EVAL=1. Contact candidates and force-bearing
contacts are kept separate. Robot motion is an input, never a fidelity score.
"""
import json
from pathlib import Path
import numpy as np

NODES=['bowl0','bowl1','bowl2','left_link7','left_link8','right_link7','right_link8','table']
PAIRS=[(a,b) for i,a in enumerate(NODES) for b in NODES[i+1:]]
FINGERS=NODES[3:7]


def save_contacts(out,dt,count,force,gap,monitors=None,monitor_names=None,capacities=None):
    np.savez_compressed(Path(out)/'contact_pairs.npz',time=np.arange(len(count))*dt,
        pair_names=np.array([a+'/'+b for a,b in PAIRS]),candidate_count=count,
        normal_force_sum_N=force,min_native_gap_m=gap,
        monitors=np.asarray(monitors) if monitors is not None else np.empty((len(count),0)),
        monitor_names=np.array(monitor_names or [],dtype=str),capacities=np.array(capacities or [],dtype=int))


class MuJoCoTelemetry:
    def __init__(self,scene,target,out,model,data,ids,slots):
        self.scene,self.target,self.out,self.model,self.data,self.ids,self.slots=scene,target,Path(out),model,data,ids,slots
        n=len(target['time'])-1;self.n=n;self.trace={k:[] for k in ['time']+[b['name']+'__'+v for b in scene['bodies'] for v in ('pose','velocity')]}
        self.count=np.zeros((n,len(PAIRS)),np.int32);self.force=np.zeros((n,len(PAIRS)));self.gap=np.full((n,len(PAIRS)),np.inf)
        self.lookup={tuple(sorted((a,b))):i for i,(a,b) in enumerate(PAIRS)}
        self.body_names={v:k for k,v in ids.items()};self.body_names[0]='table';self.joints=[];self.joint_velocity=[]

    def state(self,k,states):
        self.trace['time'].append(k*self.scene['dt'])
        for name,(p,q,v,w) in states.items():
            self.trace[name+'__pose'].append(np.r_[p,q]);self.trace[name+'__velocity'].append(np.r_[v,w])
        self.joints.append([self.data.qpos[self.slots[n][0]] for n in FINGERS])
        self.joint_velocity.append([self.data.qvel[self.slots[n][1]] for n in FINGERS])

    def contacts(self,k):
        import mujoco as mj
        f=np.zeros(6)
        for cidx,c in enumerate(self.data.contact[:self.data.ncon]):
            a=self.body_names.get(int(self.model.geom_bodyid[c.geom1]));b=self.body_names.get(int(self.model.geom_bodyid[c.geom2]))
            pair=self.lookup.get(tuple(sorted((a,b)))) if a is not None and b is not None else None
            if pair is None:continue
            self.count[k,pair]+=1;self.gap[k,pair]=min(self.gap[k,pair],float(c.dist))
            mj.mj_contactForce(self.model,self.data,cidx,f);self.force[k,pair]+=max(0.,float(f[0]))

    def finish(self):
        np.savez_compressed(self.out/'dense_trace.npz',**{k:np.asarray(v) for k,v in self.trace.items()})
        np.savez_compressed(self.out/'joint_trace.npz',time=self.target['time'],names=np.array(FINGERS),position_m=self.joints,velocity_m_s=self.joint_velocity,
                            command_left=self.target['command_left'],command_right=self.target['command_right'])
        save_contacts(self.out,self.scene['dt'],self.count,self.force,self.gap)
        (self.out/'telemetry.json').write_text(json.dumps({'dt_s':self.scene['dt'],'contact_time':'start of native integration step',
            'force_definition':'sum of positive mj_contactForce normal components per physical body pair',
            'dense_states':'post-integration body states; initial state included','read_only':True},indent=2))


def sap_telemetry(scene,target,out,model,cm,pipe,con,solver,ids,qs,vs,jl,device):
    # Imported lazily so CPU-only metric tests do not require Warp.
    import warp as wp

    @wp.kernel
    def capture_state(kbuf:wp.array(dtype=wp.int32),offset:int,body_ids:wp.array(dtype=wp.int32),
                      q:wp.array(dtype=wp.transformf),v:wp.array(dtype=wp.spatial_vectorf),values:wp.array3d(dtype=wp.float32),
                      qidx:wp.array(dtype=wp.int32),vidx:wp.array(dtype=wp.int32),jq:wp.array(dtype=wp.float32),jv:wp.array(dtype=wp.float32),jvalues:wp.array3d(dtype=wp.float32)):
        i=wp.tid();k=kbuf[0]+offset;b=body_ids[i];p=wp.transform_get_translation(q[b]);r=wp.transform_get_rotation(q[b]);vv=v[b]
        values[k,i,0]=p[0];values[k,i,1]=p[1];values[k,i,2]=p[2]
        values[k,i,3]=r[3];values[k,i,4]=r[0];values[k,i,5]=r[1];values[k,i,6]=r[2]
        for j in range(6):values[k,i,7+j]=vv[j]
        if i<4:jvalues[k,i,0]=jq[qidx[i]];jvalues[k,i,1]=jv[vidx[i]]

    @wp.kernel
    def capture_contacts(kbuf:wp.array(dtype=wp.int32),count:wp.array(dtype=wp.int32),
                         body0:wp.array2d(dtype=wp.int32),body1:wp.array2d(dtype=wp.int32),
                         phi:wp.array2d(dtype=wp.float32),gamma:wp.array2d(dtype=wp.vec3f),
                         body_node:wp.array(dtype=wp.int32),pair_id:wp.array2d(dtype=wp.int32),dt:float,
                         counts:wp.array2d(dtype=wp.int32),forces:wp.array2d(dtype=wp.float32),gaps:wp.array2d(dtype=wp.float32)):
        c=wp.tid();k=kbuf[0]
        if c<count[0]:
            a=body_node[body0[0,c]+1];b=body_node[body1[0,c]+1]
            if a>=0 and b>=0:
                pair=pair_id[a,b]
                if pair>=0:
                    wp.atomic_add(counts,k,pair,1);wp.atomic_add(forces,k,pair,wp.max(0.0,gamma[0,c][2]/dt));wp.atomic_min(gaps,k,pair,phi[0,c])

    @wp.kernel
    def capture_monitor(kbuf:wp.array(dtype=wp.int32),j:int,value:wp.array(dtype=wp.int32),result:wp.array2d(dtype=wp.int32)):
        result[kbuf[0],j]=value[0]

    n=len(target['time'])-1;names=[b['name'] for b in scene['bodies']];cap=scene['contact_capacity']
    values=wp.zeros((n+1,len(names),13),dtype=wp.float32,device=device);jvalues=wp.zeros((n+1,4,2),dtype=wp.float32,device=device)
    body_ids=wp.array([ids[x] for x in names],dtype=wp.int32,device=device)
    qidx=wp.array([int(qs[jl.index(x+'_joint')]) for x in FINGERS],dtype=wp.int32,device=device)
    vidx=wp.array([int(vs[jl.index(x+'_joint')]) for x in FINGERS],dtype=wp.int32,device=device)
    bn=np.full(len(model.body_label)+1,-1,np.int32);bn[0]=NODES.index('table')
    for i,name in enumerate(model.body_label):
        if name in NODES:bn[i+1]=NODES.index(name)
    pm=np.full((len(NODES),len(NODES)),-1,np.int32)
    for i,(a,b) in enumerate(PAIRS):pm[NODES.index(a),NODES.index(b)]=pm[NODES.index(b),NODES.index(a)]=i
    bn=wp.array(bn,dtype=wp.int32,device=device);pm=wp.array(pm,dtype=wp.int32,device=device)
    counts=wp.zeros((n,len(PAIRS)),dtype=wp.int32,device=device);forces=wp.zeros((n,len(PAIRS)),dtype=wp.float32,device=device)
    gaps=wp.full((n,len(PAIRS)),value=float('inf'),dtype=wp.float32,device=device)
    cj=solver.contact_jacobian
    if solver.contact_solve.contact_gamma.dtype!=wp.vec3f:raise TypeError('telemetry currently expects the frozen f32 SAP solver')
    specs=[('raw_contacts',con.rigid_contact_count,con.naconmax),('broad_pairs',pipe.broad_phase_pair_count,pipe.shape_pairs_max),
           ('solver_truncated',cj.contact_env_truncated_count_wp,0)]
    for label in ('gjk_candidate_pairs','shape_pairs_mesh','triangle_pairs','shape_pairs_mesh_plane','shape_pairs_mesh_mesh'):
        count=getattr(pipe.narrow_phase,label+'_count',None);buffer=getattr(pipe.narrow_phase,label,None)
        if count is not None and buffer is not None:specs.append((label,count,buffer.shape[0]))
    monitors=wp.zeros((n,len(specs)),dtype=wp.int32,device=device)

    class Telemetry:
        def state(self,counter,state,offset):
            wp.launch(capture_state,len(names),inputs=[counter,offset,body_ids,state.body_q,state.body_qd,values,qidx,vidx,state.joint_q,state.joint_qd,jvalues],device=device)
        def contacts(self,counter):
            wp.launch(capture_contacts,cap,inputs=[counter,cj.contact_env_count_wp,cj.contact_env_body0_wp,cj.contact_env_body1_wp,cj.contact_env_phi0_wp,
                solver.contact_solve.contact_gamma,bn,pm,scene['dt'],counts,forces,gaps],device=device)
            for j,(_,array,_) in enumerate(specs):wp.launch(capture_monitor,1,inputs=[counter,j,array,monitors],device=device)
        def finish(self):
            v=values.numpy();tr={'time':target['time']}
            for i,name in enumerate(names):tr[name+'__pose']=v[:,i,:7];tr[name+'__velocity']=v[:,i,7:]
            np.savez_compressed(Path(out)/'dense_trace.npz',**tr)
            j=jvalues.numpy();np.savez_compressed(Path(out)/'joint_trace.npz',time=target['time'],names=np.array(FINGERS),position_m=j[:,:,0],velocity_m_s=j[:,:,1],
                command_left=target['command_left'],command_right=target['command_right'])
            save_contacts(out,scene['dt'],counts.numpy(),forces.numpy(),gaps.numpy(),monitors.numpy(),[s[0] for s in specs],[s[2] for s in specs])
            (Path(out)/'telemetry.json').write_text(json.dumps({'dt_s':scene['dt'],'contact_time':'start of native integration step','read_only':True,
                'force_definition':'sum of positive native contact_gamma.z / dt per physical body pair','gap_definition':'native contact phi0; speculative candidates are not physical contacts',
                'dense_states':'post-integration body states; initial state included'},indent=2))
    return Telemetry()
