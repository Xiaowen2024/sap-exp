"""Render recorded deposit-coin states (head camera + close-ups); no physics is advanced."""
import argparse, os
os.environ.setdefault('MUJOCO_GL', 'cgl')
from pathlib import Path
import numpy as np, mujoco as mj
from PIL import Image
from scipy.spatial.transform import Rotation as R
from .render_tubes import build, place, label, writer, W, H


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--root', required=True); ap.add_argument('--numeric', required=True); ap.add_argument('--out', required=True)
    ap.add_argument('--runs', nargs='+', required=True, help='key=label=rundir'); a = ap.parse_args(); root, out = Path(a.root), Path(a.out); out.mkdir(parents=True, exist_ok=True)
    num = np.load(a.numeric); T = num['vision__cam_head__extrinsic_matrix'][0]; K = num['vision__cam_head__intrinsic_matrix']
    cam = {'pos': T[:3, 3], 'quat': R.from_matrix(T[:3, :3]).as_quat()[[3, 0, 1, 2]], 'fovy': float(np.degrees(2 * np.arctan(240 / K[1, 1])))}
    sc, m, mids = build(root / 'scene_v1', cam); d = mj.MjData(m); rend = mj.Renderer(m, H, W); rc = mj.Renderer(m, 720, 960)
    runs = [r.split('=') for r in a.runs]
    for key, name, run in runs:
        tr = np.load(root / run / 'trace.npz'); n = int(round(tr['time'][-1] * 25)); wr = writer(out / f'head_{key}.mp4', W, H, 25)
        for f in range(n + 1):
            t = f / 25; i = int(np.argmin(abs(tr['time'] - t))); place(m, d, mids, tr, i); rend.update_scene(d, camera='head')
            img = label(rend.render().copy(), name, f'{t:5.2f} s'); wr.stdin.write(img.tobytes())
            if f == 0: Image.fromarray(img).save(out / f'head_{key}_poster.jpg', quality=90)
        wr.stdin.close(); wr.wait(); print('wrote', key, flush=True)
    # close-ups following the coin: pick-up (1.4-2.4 s), handover (4.0-5.4 s), slot entry (6.1-7.1 s); 4x slow
    for seg, (t0, t1, dist, az, el) in {'pickup': (1.4, 2.4, .16, -150, -20), 'handover': (4.0, 5.4, .22, -90, -10), 'slot': (6.1, 7.1, .30, 100, -30)}.items():
        trs = [(name, np.load(root / run / 'trace.npz')) for key, name, run in runs]; ts = trs[0][1]['time']; idx = np.nonzero((ts >= t0) & (ts <= t1))[0]
        for g in range(m.ngeom):   # see the coin inside the bank: semi-transparent bank for the slot segment only
            if (mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, g) or '').startswith('bank') or m.geom_dataid[g] >= 0 and m.mesh_vertadr is not None and False: pass
        bank_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == 0 and m.geom_rgba[g][0] > .9 and abs(m.geom_rgba[g][1] - .55) < .01]
        for g in bank_geoms: m.geom_rgba[g][3] = .35 if seg == 'slot' else 1.
        wr = writer(out / f'closeup_{seg}.mp4', 960 * len(trs), 720, 25)
        for j, i in enumerate(idx):
            tiles = []
            for name, tr in trs:
                place(m, d, mids, tr, i); c = mj.MjvCamera(); c.type = mj.mjtCamera.mjCAMERA_FREE
                c.lookat[:] = tr['coin0__pose'][i, :3] if seg != 'slot' else np.asarray(next(b for b in sc['static_bodies'] if b['name'] == 'bank')['p']) + [0, 0, .09]
                c.distance, c.azimuth, c.elevation = dist, az, el; rc.update_scene(d, camera=c); tiles.append(label(rc.render().copy(), name, f'{tr["time"][i]:.2f} s · 4x slow', 24))
            img = np.hstack(tiles); wr.stdin.write(img.tobytes())
            if j == len(idx) // 2: Image.fromarray(img).save(out / f'closeup_{seg}_mid.jpg', quality=90)
        wr.stdin.close(); wr.wait(); print('wrote closeup', seg, flush=True)
    rend.close(); rc.close()


if __name__ == '__main__': main()
