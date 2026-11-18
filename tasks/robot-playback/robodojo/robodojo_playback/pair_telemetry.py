"""Read-only 1-ms body-pair observers, including identities of static shapes."""
import json
from pathlib import Path
import numpy as np
import warp as wp


@wp.kernel
def sap_state(counter:wp.array(dtype=wp.int32),offset:int,ids:wp.array(dtype=wp.int32),q:wp.array(dtype=wp.transformf),
              v:wp.array(dtype=wp.spatial_vectorf),values:wp.array3d(dtype=wp.float32)):
    i=wp.tid();k=counter[0]+offset;b=ids[i];p=wp.transform_get_translation(q[b]);r=wp.transform_get_rotation(q[b]);vv=v[b]
    for j in range(3):values[k,i,j]=p[j]
    values[k,i,3]=r[3];values[k,i,4]=r[0];values[k,i,5]=r[1];values[k,i,6]=r[2]
    for j in range(6):values[k,i,7+j]=vv[j]


@wp.kernel
def sap_pairs(counter:wp.array(dtype=wp.int32),count:wp.array(dtype=wp.int32),s0:wp.array(dtype=wp.int32),s1:wp.array(dtype=wp.int32),
              env:wp.array(dtype=wp.int32),slot:wp.array(dtype=wp.int32),shape_node:wp.array(dtype=wp.int32),pair_id:wp.array2d(dtype=wp.int32),
              phi:wp.array2d(dtype=wp.float32),gamma:wp.array2d(dtype=wp.vec3f),dt:float,counts:wp.array2d(dtype=wp.int32),
              forces:wp.array2d(dtype=wp.float32),gaps:wp.array2d(dtype=wp.float32),monitors:wp.array2d(dtype=wp.int32),truncated:wp.array(dtype=wp.int32)):
    c=wp.tid();k=counter[0]
    if c==0:
        monitors[k,0]=count[0];monitors[k,1]=truncated[0]
        if count[0]>s0.shape[0]:monitors[k,2]=1
    if c<wp.min(count[0],s0.shape[0]):
        a=shape_node[s0[c]];b=shape_node[s1[c]];e=env[c];j=slot[c]
        if a>=0 and b>=0:
            p=pair_id[a,b]
            if p>=0:
                wp.atomic_add(counts,k,p,1)
                if e>=0 and j>=0 and j<phi.shape[1]:
                    wp.atomic_min(gaps,k,p,phi[e,j]);wp.atomic_add(forces,k,p,wp.max(0.0,gamma[e,j][2]/dt))


@wp.kernel
def mj_pairs(counter:wp.array(dtype=wp.int32),count:wp.array(dtype=wp.int32),geom:wp.array(dtype=wp.vec2i),geom_node:wp.array(dtype=wp.int32),
             pair_id:wp.array2d(dtype=wp.int32),dist:wp.array(dtype=wp.float32),force:wp.array(dtype=wp.spatial_vectorf),
             counts:wp.array2d(dtype=wp.int32),forces:wp.array2d(dtype=wp.float32),gaps:wp.array2d(dtype=wp.float32),monitors:wp.array2d(dtype=wp.int32),
             nefc:wp.array(dtype=wp.int32),njmax:int):
    c=wp.tid();k=counter[0]
    if c==0:
        monitors[k,0]=count[0];monitors[k,1]=nefc[0]
        if count[0]>geom.shape[0] or nefc[0]>njmax:monitors[k,2]=1
    if c<wp.min(count[0],geom.shape[0]):
        gs=geom[c];a=geom_node[gs[0]];b=geom_node[gs[1]]
        if a>=0 and b>=0:
            p=pair_id[a,b]
            if p>=0:
                wp.atomic_add(counts,k,p,1);wp.atomic_add(forces,k,p,wp.max(0.0,force[c][0]));wp.atomic_min(gaps,k,p,dist[c])


def arrays(scene,target,device):
    nodes=scene['telemetry_nodes'];pairs=[(a,b) for i,a in enumerate(nodes) for b in nodes[i+1:]];n=len(target['time'])-1
    pm=np.full((len(nodes),len(nodes)),-1,np.int32)
    for i,(a,b) in enumerate(pairs):pm[nodes.index(a),nodes.index(b)]=pm[nodes.index(b),nodes.index(a)]=i
    pm=wp.array(pm,dtype=wp.int32,device=device)
    return pairs,pm,wp.zeros((n,len(pairs)),dtype=wp.int32,device=device),wp.zeros((n,len(pairs)),dtype=wp.float32,device=device),wp.full((n,len(pairs)),value=float('inf'),dtype=wp.float32,device=device),wp.zeros((n,3),dtype=wp.int32,device=device)


def save(scene,target,out,pairs,counts,forces,gaps,monitors,monitor_names,description):
    out=Path(out);np.savez_compressed(out/'contact_pairs.npz',time=target['time'][:-1],pair_names=np.array([a+'/'+b for a,b in pairs]),candidate_count=counts.numpy(),
        normal_force_sum_N=forces.numpy(),min_native_gap_m=gaps.numpy(),monitors=monitors.numpy(),monitor_names=np.array(monitor_names))
    (out/'telemetry.json').write_text(json.dumps(dict(dt_s=scene['dt'],read_only=True,contact_time='contact solve at start-of-step state; force applied during that step',
        force_definition=description,static_identity='shape/geom labels, not static body index -1 or world body index 0',dense_states='post-integration; initial state included'),indent=2))


def sap_telemetry(scene,target,out,model,loaded,pipe,con,solver,ids,qs,vs,jl,device):
    pairs,pm,counts,forces,gaps,monitors=arrays(scene,target,device);nodes=scene['telemetry_nodes'];names=[b['name'] for b in scene['bodies']]
    labels=list(loaded.shape_labels);mapping=[]
    for label in labels:
        matches=[n for n in nodes if label==n or label.startswith(n+'_') or label.startswith(n+'/')]
        if not matches:raise ValueError('cannot assign physical shape: '+label)
        mapping.append(nodes.index(max(matches,key=len)))
    sn=wp.array(mapping,dtype=wp.int32,device=device);bodyids=wp.array([ids[x] for x in names],dtype=wp.int32,device=device)
    values=wp.zeros((len(target['time']),len(names),13),dtype=wp.float32,device=device);cj=solver.contact_jacobian
    class Observer:
        def state(self,counter,state,offset):wp.launch(sap_state,len(names),inputs=[counter,offset,bodyids,state.body_q,state.body_qd,values],device=device)
        def contacts(self,counter):
            wp.launch(sap_pairs,con.rigid_contact_shape0.shape[0],inputs=[counter,con.rigid_contact_count,con.rigid_contact_shape0,con.rigid_contact_shape1,
                cj.contact_input_env_wp,cj.contact_input_slot_wp,sn,pm,cj.contact_env_phi0_wp,solver.contact_solve.contact_gamma,scene['dt'],counts,forces,gaps,monitors,cj.contact_env_truncated_count_wp],device=device)
        def finish(self):
            v=values.numpy();tr={'time':target['time']}
            for i,n in enumerate(names):tr[n+'__pose']=v[:,i,:7];tr[n+'__velocity']=v[:,i,7:]
            np.savez_compressed(Path(out)/'dense_trace.npz',**tr)
            save(scene,target,out,pairs,counts,forces,gaps,monitors,['raw_candidates','solver_truncated','capacity_exceeded'],'sum positive SAP contact_gamma.z / dt, raw contact mapped to solver slot')
    return Observer()


def mjwarp_telemetry(scene,target,out,m,gm,d,device):
    import mujoco as mj
    import mujoco_warp as mw
    pairs,pm,counts,forces,gaps,monitors=arrays(scene,target,device);nodes=scene['telemetry_nodes'];mapping=[]
    for g in range(m.ngeom):
        bn=mj.mj_id2name(m,mj.mjtObj.mjOBJ_BODY,int(m.geom_bodyid[g]));gn=mj.mj_id2name(m,mj.mjtObj.mjOBJ_GEOM,g) or ''
        if bn in nodes:node=bn
        else:
            matches=[n for n in nodes if gn==n or gn.startswith(n+'_')];node=max(matches,key=len) if matches else None
        if node is None:raise ValueError('unmapped physical geom: '+str((bn,gn)))
        mapping.append(nodes.index(node))
    gn=wp.array(mapping,dtype=wp.int32,device=device);force=wp.zeros(d.naconmax,dtype=wp.spatial_vectorf,device=device);indices=wp.array(np.arange(d.naconmax,dtype=np.int32),dtype=wp.int32,device=device)
    class Observer:
        def contacts(self,counter):
            mw.contact_force(gm,d,indices,False,force)
            wp.launch(mj_pairs,d.naconmax,inputs=[counter,d.nacon,d.contact.geom,gn,pm,d.contact.dist,force,counts,forces,gaps,monitors,d.nefc,d.njmax],device=device)
        def finish(self):save(scene,target,out,pairs,counts,forces,gaps,monitors,['raw_candidates','constraints','capacity_exceeded'],'sum positive MuJoCo Warp contact_force normal components')
    return Observer()
