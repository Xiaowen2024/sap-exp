"""Render recorded native body states; never advance physics in this renderer."""
import argparse
import json
import os
os.environ.setdefault('MUJOCO_GL','osmesa')
os.environ.setdefault('PYOPENGL_PLATFORM','osmesa')
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
import mujoco as mj
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial.transform import Rotation
from .run import make_xml,nums


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',required=True);ap.add_argument('--original',required=True);a=ap.parse_args()
    root=Path(a.root);original=Path(a.original);scene=json.loads((root/'scenes/r3/scene.json').read_text())
    data=np.load(original/'reference/numeric.npz',allow_pickle=False);T=data['vision__cam_head__extrinsic_matrix'][0];K=data['vision__cam_head__intrinsic_matrix']
    xml=ET.fromstring(make_xml(scene,True));wb=xml.find('worldbody')
    # Released camera-to-world poses already use OpenGL axes, as fit_initial.py confirms.
    camera_rotation=T[:3,:3]
    ET.SubElement(wb,'camera',name='head',pos=nums(T[:3,3]),quat=nums(Rotation.from_matrix(camera_rotation).as_quat()[[3,0,1,2]]),fovy=str(np.degrees(2*np.arctan(240/K[1,1]))))
    ET.SubElement(wb,'light',pos='0 -.2 2',dir='0 0 -1',diffuse='.7 .7 .7')
    m=mj.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'));d=mj.MjData(m)
    colors={'bowl0':[.45,.65,.8,1],'bowl1':[.75,.75,.78,1],'bowl2':[.85,.65,.4,1]}
    for g in range(m.ngeom):
        name=mj.mj_id2name(m,mj.mjtObj.mjOBJ_BODY,int(m.geom_bodyid[g]))
        m.geom_rgba[g]=colors.get(name,[.12,.12,.13,0 if name and name.endswith('_base') else 1] if name!='world' else [.4,.28,.18,1])
    slots={b['name']:int(m.jnt_qposadr[mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,b['name']+'_free')]) for b in scene['bodies']}
    paths=[('MuJoCo r3',original/'runs/mujoco_objects_r3'),('SAP r3 | legacy convex',root/'runs/sap_objects_r3_legacy'),('SAP r3 | Newton convex',root/'runs/sap_objects_r3_newton')]
    traces=[(name,np.load(path/'trace.npz',allow_pickle=False)) for name,path in paths]
    renderer=mj.Renderer(m,height=480,width=640)
    decoder=subprocess.Popen(['ffmpeg','-loglevel','error','-i',str(original/'reference/source_head.mp4'),'-f','rawvideo','-pix_fmt','rgb24','-'],stdout=subprocess.PIPE)
    out=root/'comparison_r3.mp4';writer=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','1280x960','-framerate','25','-i','-','-an','-c:v','libx264','-preset','fast','-crf','22','-pix_fmt','yuv420p',str(out)],stdin=subprocess.PIPE)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',17)
    def title(picture,text):
        im=Image.fromarray(picture);dr=ImageDraw.Draw(im);dr.rectangle((0,0,640,34),fill=(20,20,20));dr.text((10,8),text,font=font,fill=(240,240,240));return np.asarray(im)
    last=None
    for frame in range(int(round(traces[0][1]['time'][-1]*25))+1):
        t=frame/25;raw=decoder.stdout.read(640*480*3)
        if len(raw)==640*480*3:last=np.frombuffer(raw,dtype=np.uint8).reshape(480,640,3).copy()
        tiles=[title(last.copy(),f'Official demo | {t:.2f}s'+(' | final held' if len(raw)<640*480*3 else ''))]
        for name,tr in traces:
            i=int(np.argmin(abs(tr['time']-t)))
            for body,slot in slots.items():d.qpos[slot:slot+7]=tr[body+'__pose'][i]
            d.qvel[:]=0;mj.mj_kinematics(m,d);mj.mj_camlight(m,d);renderer.update_scene(d,camera='head')
            tiles.append(title(renderer.render().copy(),name+f' | {t:.2f}s'))
        grid=np.vstack([np.hstack(tiles[:2]),np.hstack(tiles[2:])]);writer.stdin.write(grid.tobytes())
        if frame==0:Image.fromarray(grid).save(root/'comparison_start.png')
        if frame%50==0:print('render frame',frame,flush=True)
        if frame==int(round(traces[0][1]['time'][-1]*25)):Image.fromarray(grid).save(root/'comparison_final.png')
    writer.stdin.close();rc=writer.wait();decoder.stdout.close();decoder.wait();renderer.close()
    if rc:raise RuntimeError('video encoder failed')
    (root/'render_provenance.json').write_text(json.dumps({'physics':'recorded native body states; no physics integration','renderer':'MuJoCo '+mj.__version__,
        'camera':'official first head-camera pose (released OpenGL axes) and focal length','colors':'shared bowl-ID colors; source textures omitted',
        'limitations':['detached grippers; robot arms omitted','collision hulls rendered in all simulation panels','official final frame held during appended tail'],
        'runs':[str(p) for _,p in paths]},indent=2))
    print(out,flush=True)


if __name__=='__main__':main()
