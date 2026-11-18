"""Slot close-up of a perturbed deposit-coin run: MuJoCo | SAP side by side, 4x slow, translucent bank."""
import argparse, os, json
os.environ.setdefault('MUJOCO_GL', 'cgl')
from pathlib import Path
import numpy as np, mujoco as mj
from PIL import Image
from scipy.spatial.transform import Rotation as R
from .render_tubes import build, place, label, writer


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--root', required=True); ap.add_argument('--numeric', required=True); ap.add_argument('--variant', required=True); ap.add_argument('--out', required=True)
    a = ap.parse_args(); root = Path(a.root); out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    num = np.load(a.numeric); T = num['vision__cam_head__extrinsic_matrix'][0]; K = num['vision__cam_head__intrinsic_matrix']
    cam = {'pos': T[:3, 3], 'quat': R.from_matrix(T[:3, :3]).as_quat()[[3, 0, 1, 2]], 'fovy': float(np.degrees(2 * np.arctan(240 / K[1, 1])))}
    sc, m, mids = build(root / 'scene_v1', cam); d = mj.MjData(m)
    for g in range(m.ngeom):          # translucent bank (bank geoms are the pink static ones)
        if m.geom_bodyid[g] == 0 and m.geom_rgba[g][0] > .9 and abs(m.geom_rgba[g][1] - .55) < .01: m.geom_rgba[g][3] = .3
    bank = next(b for b in sc['static_bodies'] if b['name'] == 'bank'); rb = R.from_quat(np.asarray(bank['q'])[[1, 2, 3, 0]])
    slot = np.asarray(bank['p']) + rb.apply([.001, -.004, .105]); along = rb.apply([0, 1, 0]); az_along = float(np.degrees(np.arctan2(-along[1], -along[0])))
    pert = sc.get('perturbation', {}).get('bank_shift_slot_frame_mm', [0, 0])[0]
    trs = [('MuJoCo 4.0.0', np.load(root / f'runs/mj_pert_{a.variant}/trace.npz')), ('SAP + Newton convex', np.load(root / f'runs/sapnew_pert_{a.variant}/trace.npz'))]
    rc = mj.Renderer(m, 720, 960); ts = trs[0][1]['time']; idx = np.nonzero((ts >= 6.0) & (ts <= 7.3))[0]
    for view, (dist, az, el) in {'along_slot': (.16, az_along, -12), 'three_quarter': (.22, az_along + 55, -30)}.items():
        wr = writer(out / f'fail_{a.variant}_{view}.mp4', 1920, 720, 25)
        for j, i in enumerate(idx):
            tiles = []
            for name, tr in trs:
                place(m, d, mids, tr, i); c = mj.MjvCamera(); c.type = mj.mjtCamera.mjCAMERA_FREE; c.lookat[:] = slot; c.distance, c.azimuth, c.elevation = dist, az, el
                rc.update_scene(d, camera=c); tiles.append(label(rc.render().copy(), f'{name} · bank shifted {pert:+.0f} mm across slot', f'{tr["time"][i]:.2f} s · 4x slow', 22))
            img = np.hstack(tiles); wr.stdin.write(img.tobytes())
            for tt in (6.28, 6.50, 7.20):
                if abs(ts[i] - tt) < 1e-6: Image.fromarray(img).save(out / f'fail_{a.variant}_{view}_{tt:.2f}.jpg', quality=90)
        wr.stdin.close(); wr.wait(); print('wrote', a.variant, view, flush=True)
    rc.close()


if __name__ == '__main__': main()
