"""Render recorded insert-tubes body states (no physics is advanced here).

Every body is a mocap body placed at the recorded pose; geometry is the shared
scene geometry (the SAP panels therefore show the true tube shape, not SAP's
truncated hull).  Outputs:
  head_<name>.mp4  official head-camera view, 1280x960, 25 fps, full episode
  closeup.mp4      MuJoCo vs SAP at the hole mouth (tube1), 4x slow motion
"""
import argparse, json, os, subprocess
os.environ.setdefault('MUJOCO_GL', 'cgl')
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco as mj
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation as R

W, H = 1280, 960
COL = {'tube_0': '0.93 0.93 0.95 1', 'tube_1': '0.95 0.55 0.2 1', 'rack': '0.35 0.6 0.9 1', 'link7': '0.15 0.15 0.17 1', 'link8': '0.15 0.15 0.17 1',
       'coin_0': '0.93 0.75 0.25 1', 'stand': '0.75 0.75 0.78 1', 'bank': '0.95 0.55 0.55 1'}


def nums(v): return ' '.join(f'{float(x):.9g}' for x in np.ravel(v))


def build(scene_dir, cam):
    sc = json.loads((scene_dir / 'scene.json').read_text()); root = ET.Element('mujoco')
    ET.SubElement(root, 'compiler', angle='radian')
    vis = ET.SubElement(root, 'visual'); ET.SubElement(vis, 'global', offwidth=str(2 * W), offheight=str(H))
    ET.SubElement(vis, 'quality', shadowsize='4096', offsamples='8'); ET.SubElement(vis, 'headlight', ambient='.35 .35 .35', diffuse='.45 .45 .45')
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'texture', name='wood', type='2d', builtin='flat', rgb1='.42 .25 .16', width='64', height='64')
    ET.SubElement(asset, 'material', name='wood', texture='wood', rgba='.45 .28 .18 1', specular='.1')
    for kind, a in sc['assets'].items():
        for i, p in enumerate(a['pieces']): ET.SubElement(asset, 'mesh', name=f'{kind}_{i}', file=str((scene_dir / p).resolve()))
    wb = ET.SubElement(root, 'worldbody')
    ET.SubElement(wb, 'light', pos='0 -.3 1.9', dir='0 .15 -1', diffuse='.55 .55 .55', castshadow='true')
    ET.SubElement(wb, 'light', pos='.6 .4 1.6', dir='-.3 -.2 -1', diffuse='.25 .25 .25', castshadow='false')
    ET.SubElement(wb, 'geom', type='box', pos='0 -.05 .74', size='.7 .55 .025', material='wood')
    ET.SubElement(wb, 'camera', name='head', pos=nums(cam['pos']), quat=nums(cam['quat']), fovy=str(cam['fovy']))
    for sb in sc['static_bodies']:
        for i in range(len(sc['assets'][sb['asset']]['pieces'])):
            ET.SubElement(wb, 'geom', type='mesh', mesh=f"{sb['asset']}_{i}", pos=nums(sb['p']), quat=nums(sb['q']), rgba=COL.get(sb['asset'], COL['rack']), contype='0', conaffinity='0')
    for b in sc['bodies']:
        if b['asset'] == 'base': continue
        body = ET.SubElement(wb, 'body', name=b['name'], mocap='true', pos=nums(b['p']), quat=nums(b['q']))
        for i in range(len(sc['assets'][b['asset']]['pieces'])):
            key = f"{b['asset']}_{i}" if b['asset'] in ('tube', 'coin') else b['asset']
            ET.SubElement(body, 'geom', type='mesh', mesh=f"{b['asset']}_{i}", rgba=COL[key], contype='0', conaffinity='0')
    m = mj.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    mids = {b['name']: m.body_mocapid[mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, b['name'])] for b in sc['bodies'] if b['asset'] != 'base'}
    return sc, m, mids


def place(m, d, mids, tr, i):
    for n, k in mids.items(): d.mocap_pos[k] = tr[n + '__pose'][i, :3]; d.mocap_quat[k] = tr[n + '__pose'][i, 3:]
    mj.mj_forward(m, d)


def label(img, text, sub=None, size=30):
    im = Image.fromarray(img); dr = ImageDraw.Draw(im); f = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', size)
    dr.rectangle((0, 0, im.width, size + 22), fill=(18, 18, 20)); dr.text((16, 10), text, font=f, fill=(245, 245, 245))
    if sub: fs = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', int(size * .8)); tw = dr.textlength(sub, font=fs); dr.text((im.width - tw - 16, 13), sub, font=fs, fill=(200, 200, 205))
    return np.asarray(im)


def writer(path, w, h, fps):
    return subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{w}x{h}', '-r', str(fps), '-i', '-', '-an',
                             '-c:v', 'libx264', '-preset', 'slow', '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(path)], stdin=subprocess.PIPE)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--root', required=True); ap.add_argument('--numeric', required=True); ap.add_argument('--out', required=True)
    a = ap.parse_args(); root, out = Path(a.root), Path(a.out); out.mkdir(parents=True, exist_ok=True)
    num = np.load(a.numeric); T = num['vision__cam_head__extrinsic_matrix'][0]; K = num['vision__cam_head__intrinsic_matrix']
    cam = {'pos': T[:3, 3], 'quat': R.from_matrix(T[:3, :3]).as_quat()[[3, 0, 1, 2]], 'fovy': float(np.degrees(2 * np.arctan(240 / K[1, 1])))}
    sc, m, mids = build(root / 'scene_v1', cam); d = mj.MjData(m); rend = mj.Renderer(m, H, W)
    runs = {'mujoco': ('MuJoCo 4.0.0 · iteration 2 (best)', root / 'runs/mj_it2'), 'sap': ('SAP + Newton convex · iteration 2 (best)', root / 'runs/sapnew_it2'),
            'sap_legacy': ('SAP + legacy convex · iteration 2 (best)', root / 'runs/sapold_it2')}
    for key, (name, run) in runs.items():
        tr = np.load(run / 'trace.npz'); n = int(round(tr['time'][-1] * 25)); wr = writer(out / f'head_{key}.mp4', W, H, 25)
        for f in range(n + 1):
            t = f / 25; i = int(np.argmin(abs(tr['time'] - t))); place(m, d, mids, tr, i); rend.update_scene(d, camera='head')
            img = label(rend.render().copy(), name, f'{t:5.2f} s'); wr.stdin.write(img.tobytes())
            if f == 0: Image.fromarray(img).save(out / f'head_{key}_poster.jpg', quality=90)
            if f == n: Image.fromarray(img).save(out / f'head_{key}_final.jpg', quality=90)
        wr.stdin.close(); wr.wait(); print('wrote', key, flush=True)
    # close-up of tube1 entering hole 2: free camera, front-left of the rack, 4x slow motion
    rk = sc['static_bodies'][0]; rr = R.from_quat(np.r_[rk['q'][1:], rk['q'][0]]); hole = np.asarray(rk['p']) + rr.apply([.072, -.018, .03])
    fc = mj.MjvCamera(); fc.type = mj.mjtCamera.mjCAMERA_FREE; fc.lookat[:] = hole + [0, 0, .015]; fc.distance = .30; fc.azimuth = -70; fc.elevation = -18
    trs = [(n, np.load(p / 'trace.npz')) for n, p in (('MuJoCo 4.0.0', runs['mujoco'][1]), ('SAP + Newton convex', runs['sap'][1]))]
    wr = writer(out / 'closeup.mp4', 2 * 960, 720, 25); rc = mj.Renderer(m, 720, 960)
    t0, t1 = 6.70, 7.70; ts = trs[0][1]['time']; idx = np.nonzero((ts >= t0 - 1e-9) & (ts <= t1 + 1e-9))[0]
    for j, i in enumerate(idx):
        tiles = []
        for n, tr in trs:
            place(m, d, mids, tr, i); rc.update_scene(d, camera=fc); tiles.append(label(rc.render().copy(), n, f'{tr["time"][i]:.2f} s · 4x slow', 26))
        img = np.hstack(tiles); wr.stdin.write(img.tobytes())
        for tt in (7.00, 7.10, 7.30):
            if abs(ts[i] - tt) < 1e-6: Image.fromarray(img).save(out / f'closeup_{tt:.2f}.jpg', quality=92)
    wr.stdin.close(); wr.wait(); rend.close(); rc.close(); print('wrote closeup', flush=True)


if __name__ == '__main__': main()
