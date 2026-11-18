"""Extract pinned USD collision meshes and the common closed rigid assembly."""
import argparse,json
from pathlib import Path
import numpy as np,trimesh
from scipy.spatial.transform import Rotation as R
from pxr import Usd,UsdGeom,UsdPhysics


def extract_mesh(p):
    m=UsdGeom.Mesh(p);v=np.asarray(m.GetPointsAttr().Get(),float);idx=np.asarray(m.GetFaceVertexIndicesAttr().Get(),int);counts=np.asarray(m.GetFaceVertexCountsAttr().Get(),int);faces=[];k=0
    for n in counts:
        for j in range(1,n-1):faces.append([idx[k],idx[k+j],idx[k+j+1]])
        k+=n
    mesh=trimesh.Trimesh(v,faces,process=True);mesh.apply_transform(np.asarray(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)).T);return mesh


def tf(v,q):
    T=np.eye(4);T[:3,:3]=R.from_quat(np.r_[q.GetImaginary(),q.GetReal()]).as_matrix();T[:3,3]=v;return T


def main(root):
    root=Path(root);out=root/'geometry';out.mkdir(exist_ok=True);s=Usd.Stage.Open(str(root/'source_assets/laptop/main.usdc'));base=s.GetPrimAtPath('/root/E_body_1');lid=s.GetPrimAtPath('/root/E_displayer_5');j=UsdPhysics.RevoluteJoint(s.GetPrimAtPath('/root/E_displayer_5/RevoluteJoint_computer_9_up'))
    Tbase=np.asarray(UsdGeom.Xformable(base).ComputeLocalToWorldTransform(0)).T;Tlid=np.asarray(UsdGeom.Xformable(lid).ComputeLocalToWorldTransform(0)).T;A=tf(j.GetLocalPos0Attr().Get(),j.GetLocalRot0Attr().Get());B=tf(j.GetLocalPos1Attr().Get(),j.GetLocalRot1Attr().Get());parts=[]
    for p in s.Traverse():
        if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI):parts.append((str(p.GetPath()),extract_mesh(p)))
    for angle in [-110,0]:
        J=np.eye(4);J[:3,:3]=R.from_euler('x',angle,degrees=True).as_matrix();X=Tbase@A@J@np.linalg.inv(B)@np.linalg.inv(Tlid);meshes=[]
        for path,mesh in parts:
            m=mesh.copy()
            if path.startswith('/root/E_displayer_5/'):m.apply_transform(X)
            meshes.append(m)
        trimesh.util.concatenate(meshes).export(out/f'laptop_q{angle}.obj')
    s=Usd.Stage.Open(str(root/'source_assets/rack/temp.usdc'));m=extract_mesh(s.GetPrimAtPath('/root/collision/model'));m.export(out/'rack_reference.obj');v=np.asarray(m.vertices);floor=-.02094;parts=[trimesh.convex.convex_hull(v[v[:,2]<=floor])];cut=m.slice_plane([0,0,floor],[0,0,1]);vv=np.asarray(cut.vertices)
    for sign in [-1,1]:parts.append(trimesh.convex.convex_hull(vv[sign*vv[:,1]>.00495]))
    dst=out/'rack_pieces';dst.mkdir(exist_ok=True)
    for i,p in enumerate(parts):p.export(dst/f'piece_{i:03d}.obj')
    (out/'rack_representation.json').write_text(json.dumps(dict(method='three convex regions of official mesh: floor and two separately isolated walls',floor_partition_z_m=floor,wall_side_selector_abs_y_m=.00495),indent=2))

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('root');main(a.parse_args().root)
