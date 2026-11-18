import numpy as np, trimesh, json, sys
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize_scalar, least_squares
from scipy.spatial import ConvexHull
from scipy import ndimage
from PIL import Image, ImageDraw
d=np.load('insert_tubes/numeric.npz')
K=d['vision__cam_head__intrinsic_matrix']; T=d['vision__cam_head__extrinsic_matrix'][0]
def project(v,T=T):
    cam=(v-T[:3,3])@T[:3,:3]; cam=cam*[1,-1,-1]; uv=cam@K.T; return uv[:,:2]/uv[:,2:3]
im=np.asarray(Image.open('insert_tubes/cam_head_0000.jpg').convert('RGB')).astype(int)
m=(im[:,:,0]>185)&(im[:,:,0]-im[:,:,2]>55)&(im[:,:,0]-im[:,:,1]>25)
lab,n=ndimage.label(m); sz=ndimage.sum(m,lab,range(1,n+1)); cc=ndimage.center_of_mass(m,lab,range(1,n+1))
caps=np.array([(c[1],c[0]) for a,c in zip(sz,cc) if a>8])
tube=trimesh.load('dl/tube.obj'); V=np.asarray(tube.convex_hull.vertices).copy(); ymin=tube.vertices[:,1].min(); V[:,1]-=ymin
L=float(V[:,1].max()); TABLE=.765
best=None
for th in np.linspace(-.1,.1,2001):
    w=R.from_euler('x',th).apply(V); h=-w[:,2].min()+ (w[:,2].mean()*0)
    c=w.mean(0); hc=c[2]-w[:,2].min()
    if best is None or hc<best[0]: best=(hc,th)
th=best[1]; print('rest tilt %.2f deg'%np.rad2deg(th))
grasp={'tube0':('left',29),'tube1':('right',128),'tube2':('left',228)}
release={'tube0':('left',84),'tube1':('right',186),'tube2':('left',288)}
def ee(side,i):
    P=d[f'state__{side}_ee_poses'][i]; Tm=np.eye(4); Tm[:3,:3]=R.from_quat(P[[4,5,6,3]]).as_matrix(); Tm[:3,3]=P[:3]; return Tm
out={'rest_tilt_deg':float(np.rad2deg(th)),'tubes':{}}
canvas=Image.open('insert_tubes/cam_head_0000.jpg').convert('RGB'); dr=ImageDraw.Draw(canvas)
for k,(side,i) in grasp.items():
    G=ee(side,i+5); p=G[:3,3]; xg=G[:3,0]; zg=G[:3,2]
    ax=np.r_[zg[:2],0]; ax/=np.linalg.norm(ax); yaw=np.arctan2(ax[1],ax[0])
    Rw=R.from_euler('z',yaw-np.pi/2)*R.from_euler('x',th)
    W=Rw.apply(V)
    def place(s):
        # axis point at distance s from tip must lie on the approach line (lateral), z from table rest
        a=Rw.apply([0,s,0]); rest_dz=TABLE-W[:,2].min()   # tip-origin z so hull touches table
        axis_z=rest_dz+a[2]                                # z of axis point s
        t=(p[2]-axis_z)/(-xg[2]); g=p+t*xg
        o=g-a; o[2]=rest_dz; return o,t
    def cap_uv(s):
        o,_=place(s); return project((o+Rw.apply([0,L-0.007,0])+[0,0,.017])[None])[0]
    f=lambda s: np.min(np.linalg.norm(caps-cap_uv(s),axis=1))
    s=minimize_scalar(f,bounds=(0.01,L-0.01),method='bounded').x; o,t=place(s)
    Rl=ee(*release[k]); To=np.eye(4); To[:3,:3]=Rw.as_matrix(); To[:3,3]=o
    Tf=Rl@np.linalg.inv(G)@To
    out['tubes'][k]=dict(origin_tip=o.tolist(),quat_wxyz=list(Rw.as_quat()[[3,0,1,2]]),s_tip_to_grasp=float(s),approach_depth_m=float(t),
                         cap_reproj_px=float(f(s)),rigid_carry_final_tip=Tf[:3,3].tolist(),rigid_carry_final_axis=Tf[:3,1].tolist(),grasp_frame=i+5)
    print(k,'s=%.4f depth=%.4f cap err %.1fpx yaw %.1f'%(s,t,f(s),np.rad2deg(yaw)),'tip',np.round(o,4),'| final tip',np.round(Tf[:3,3],4),'axis',np.round(Tf[:3,1],3))
    uv=project(W+o); hh=uv[ConvexHull(uv).vertices]; dr.line([tuple(x) for x in hh]+[tuple(hh[0])],fill=(0,255,0),width=1)
# rack fit: rigid-carry final axis xy (at rack top height) -> holes 0,2,4 (local x 0,.072,-.072; y -.018)
rk=json.load(open('dl/Geometry_test_tube_slot_00000/metadata.json'))['passive']['support']
holes={int(k.split('/')[1]):np.array(v['center'][0][:3]) for k,v in rk.items()}
fin=[]
for k,(side,j) in release.items():
    Rl=ee(side,j); fin.append(Rl[:3,3]+0.1445*Rl[:3,0])
fin=np.array(fin)
def rack_world(x):
    y,psi=x; Rz=R.from_euler('z',psi); return Rz, np.array([0.,y,TABLE+.03])
def resid(x):
    Rz,c=rack_world(x); return np.concatenate([(c+Rz.apply(holes[h]))[:2]-f[:2] for h,f in zip([0,2,4],fin)])
sol=least_squares(resid,[0.017,np.deg2rad(-7)]); Rz,c=rack_world(sol.x)
print('rack y=%.4f yaw=%.2f deg, hole residuals mm'%(sol.x[0],np.rad2deg(sol.x[1])),np.round(sol.fun.reshape(-1,2)*1000,2))
# free x as check
sol2=least_squares(lambda x: np.concatenate([(np.array([x[0],x[1]])+R.from_euler('z',x[2]).apply(holes[h])[:2])-f[:2] for h,f in zip([0,2,4],fin)]),[0,0.017,-.1])
print('free-x check: x=%.4f y=%.4f yaw=%.2f'%(sol2.x[0],sol2.x[1],np.rad2deg(sol2.x[2])), 'res mm',np.round(sol2.fun*1000,2))
out['rack']=dict(pos=c.tolist(),quat_wxyz=list(Rz.as_quat()[[3,0,1,2]]),yaw_deg=float(np.rad2deg(sol.x[1])),hole_residual_mm=(sol.fun*1000).tolist(),
                 assignment={'tube0':0,'tube1':2,'tube2':4},free_x_check=sol2.x.tolist())
rack=trimesh.load('dl/rack.obj'); uv=project(Rz.apply(rack.vertices)+c); hh=uv[ConvexHull(uv).vertices]; dr.line([tuple(x) for x in hh]+[tuple(hh[0])],fill=(0,200,255),width=1)
for h in holes.values():
    q=project((c+Rz.apply(h+[0,0,0.002]))[None])[0]; dr.ellipse([q[0]-2,q[1]-2,q[0]+2,q[1]+2],outline=(255,0,0))
canvas=canvas.resize((1280,960)); canvas.save('fit_overlay.png')
json.dump(out,open('scene_fit.json','w'),indent=1)
