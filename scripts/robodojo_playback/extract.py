from pxr import Usd, UsdGeom
import numpy as np, trimesh, sys
def load(f, path):
    st=Usd.Stage.Open(f); p=st.GetPrimAtPath(path); m=UsdGeom.Mesh(p)
    v=np.asarray(m.GetPointsAttr().Get(),float); X=np.asarray(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)).T; v=v@X[:3,:3].T+X[:3,3]
    c=np.asarray(m.GetFaceVertexCountsAttr().Get()); idx=np.asarray(m.GetFaceVertexIndicesAttr().Get()); F=[];o=0
    for n in c:
        for k in range(1,n-1): F.append([idx[o],idx[o+k],idx[o+k+1]])
        o+=n
    return trimesh.Trimesh(v,np.array(F),process=True)
b=load('dl/Geometry_piggy_bank_00000/object.usdz','/World/collision'); b.export('geom/bank.obj'); print('bank',len(b.vertices),b.is_watertight,b.bounds.round(4).tolist())
s=load('dl/Geometry_vertical_coin_stand_00000/object.usdz','/root/collision/model'); s.export('geom/stand.obj'); print('stand',len(s.vertices),s.is_watertight,s.bounds.round(4).tolist())
for c in ('cube_0','cube_1'):
    m=load('dl/Geometry_vertical_coin_stand_00000/object.usdz','/root/collision/'+c); m.export(f'geom/stand_{c}.obj'); print(c,m.bounds.round(4).tolist())
k=load('dl/Rigid_coin_00000/object.usdz','/World/collision/model'); k.export('geom/coin.obj'); print('coin',len(k.vertices),k.bounds.round(5).tolist(), 'r',np.hypot(k.vertices[:,0],k.vertices[:,1]).max())
