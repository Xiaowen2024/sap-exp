"""Reconstruct the deposit_coin scene of any episode from the gripper trajectory (+ first head frame).

Events come from the gripper commands: the first arm to close picks the coin from the stand;
if the other arm closes while it is still closed, that is a handover; the arm holding the coin
last releases it over the slot. The coin is carried rigidly through the chain; the bank is placed
so its slot (long axis = bank local y) lies in the coin plane directly below the coin at release.
The 180-degree ambiguity is resolved by silhouette IoU against the pink bank in the first frame.
Checks (all must pass, else the episode is excluded): config ranges, grasp height consistency,
coin already in the slot at release, IoU margin.
"""
import argparse, json
from pathlib import Path
import numpy as np, trimesh
from scipy.spatial.transform import Rotation as R
from scipy import ndimage
from PIL import Image, ImageDraw

TABLE, REACH, TIP = .765, .1445, .1575


def events(cmd):
    """close: frames where command falls below .5 (start) and its minimum plateau; open: rises above .5 after a close."""
    out = []; closed = False; start = None
    for i in range(1, len(cmd)):
        if not closed and cmd[i] < .5 <= cmd[i - 1]:
            closed = True; start = i
        elif closed and cmd[i] >= .5 > cmd[i - 1]:
            seg = cmd[start:i]; j = start + int(np.argmax(seg <= seg.min() + 1e-4))       # first frame at the closed minimum
            k = start + len(seg) - 1 - int(np.argmax(seg[::-1] <= seg.min() + .05))        # last frame before the command clearly rises
            out += [['close', start, j], ['open', k + 1, i]]; closed = False
    if closed:
        seg = cmd[start:]; out.append(['close', start, start + int(np.argmax(seg <= seg.min() + 1e-4))])
    return out


def fit(ep_dir, assets_dir):
    ep_dir, assets_dir = Path(ep_dir), Path(assets_dir); d = np.load(ep_dir / 'numeric.npz')
    K = d['vision__cam_head__intrinsic_matrix']; Tc = d['vision__cam_head__extrinsic_matrix'][0]
    def project(v): cam = (v - Tc[:3, 3]) @ Tc[:3, :3]; cam = cam * [1, -1, -1]; uv = cam @ K.T; return uv[:, :2] / uv[:, 2:3]
    def ee(side, i):
        P = d[f'state__{side}_ee_poses'][i]; T = np.eye(4); T[:3, :3] = R.from_quat(P[[4, 5, 6, 3]]).as_matrix(); T[:3, 3] = P[:3]; return T
    ev = {s: events(d[f'action__{s}_ee_joint_states'][:, 0]) for s in ('left', 'right')}
    closes = sorted([(e[1], s, e) for s in ev for e in ev[s] if e[0] == 'close'])
    if not closes: raise ValueError('no gripper closes')
    picker = closes[0][1]; g_pick = closes[0][2][2]
    p_open = next((e for e in ev[picker] if e[0] == 'open' and e[1] > g_pick), None)
    other = 'left' if picker == 'right' else 'right'
    o_close = next((e for e in ev[other] if e[0] == 'close' and e[1] > g_pick and (p_open is None or e[1] < p_open[1])), None)
    chain = [(picker, g_pick)]
    if o_close is not None:
        h = o_close[2]; chain = [(picker, g_pick, h), (other, h)]
        rel_arm = other; rel_open = next(e for e in ev[other] if e[0] == 'open' and e[1] > h)
    else:
        rel_arm = picker; rel_open = p_open
    if rel_open is None: raise ValueError('coin never released')
    rel = rel_open[1] - 1
    # stand + coin from the pick grasp (top-down grasp; closing axis = coin normal)
    G = ee(picker, g_pick); xg, yg = G[:3, 0], G[:3, 1]; tip = G[:3, 3] + REACH * xg
    n = yg / np.linalg.norm(yg); yaw_s = np.arctan2(-n[0], n[1])
    if abs(yaw_s) > np.pi / 2: yaw_s -= np.sign(yaw_s) * np.pi
    Rs = R.from_euler('z', yaw_s); stand_z = TABLE + .0086
    coin_c = np.r_[tip[:2], stand_z + .0068]; stand_p = coin_c - Rs.apply([0, 0, .0068]); stand_p[2] = stand_z
    Rc = Rs * R.from_quat([0, .7071068, .7071068, 0.])  # xyzw of wxyz (0,0,.7071,.7071): coin z -> stand y
    coin_origin = coin_c - Rc.apply([0, 0, .00097]); Tcoin = np.eye(4); Tcoin[:3, :3] = Rc.as_matrix(); Tcoin[:3, 3] = coin_origin
    checks = {}
    finger_low = G[2, 3] - TIP * abs(xg[2]); coin_top = coin_c[2] + .01425
    checks['grasp_height_ok'] = bool(-xg[2] > .9 and stand_z + .0085 - .002 < finger_low < coin_top - .002)
    # rigid carry through the chain
    T = Tcoin; prev = (picker, g_pick)
    if o_close is not None:
        T = ee(picker, h) @ np.linalg.inv(ee(picker, g_pick)) @ T; prev = (other, h)
    T = ee(rel_arm, rel) @ np.linalg.inv(ee(*prev)) @ T
    cc = T[:3, :3] @ [0, 0, .00097] + T[:3, 3]; nr = T[:3, 2]
    inplane = np.cross(nr, [0, 0, 1]); inplane /= np.linalg.norm(inplane)
    meta = json.loads((assets_dir / 'dl/Geometry_piggy_bank_00000/metadata.json').read_text())
    slot_c = np.array(meta['passive']['functional']['slot_opening']['frame'][0][:3])
    mesh = trimesh.load(assets_dir / 'geom/bank.obj'); F = mesh.faces
    ears = mesh.vertices[mesh.vertices[:, 2] > .111]                                   # ear tips: highest points, head end
    img = np.asarray(Image.open(ep_dir / 'cam_head_0000.jpg').convert('RGB')).astype(int)
    pink = (img[:, :, 0] > 170) & (img[:, :, 0] - img[:, :, 2] > 45) & (img[:, :, 1] < 165) & (img[:, :, 0] - img[:, :, 1] > 45)
    lab, nl = ndimage.label(pink); pink = lab == (np.argmax(ndimage.sum(pink, lab, range(1, nl + 1))) + 1) if nl else pink
    valid = img.max(2) > 70                                                             # exclude black gripper pixels (occluders)
    cands = []
    for sgn in (1, -1):
        yaw_b = np.arctan2(-sgn * inplane[0], sgn * inplane[1]); Rb = R.from_euler('z', yaw_b)
        bp = cc.copy(); bp[:2] -= Rb.apply(slot_c)[:2]; bp[2] = TABLE
        uv = project(Rb.apply(mesh.vertices) + bp); mask = Image.new('L', (img.shape[1], img.shape[0]), 0); dr = ImageDraw.Draw(mask)
        for f in F: dr.polygon([tuple(uv[i]) for i in f], fill=1)
        m = np.asarray(mask).astype(bool); iou = float((m & pink & valid).sum() / max(((m | pink) & valid).sum(), 1))
        eu = np.round(project(Rb.apply(ears) + bp)).astype(int); inb = (eu[:, 0] >= 0) & (eu[:, 0] < img.shape[1]) & (eu[:, 1] >= 0) & (eu[:, 1] < img.shape[0])
        e = eu[inb]; vis = valid[e[:, 1], e[:, 0]]; ear = float(pink[e[:, 1], e[:, 0]][vis].mean()) if vis.any() else 0.
        cands.append((iou + ear, yaw_b, bp, iou, ear))
    cands.sort(key=lambda c: -c[0]); _, yaw_b, bp, iou, ear = cands[0]
    yb = (np.degrees(yaw_b) + 180) % 360 - 180
    checks['iou'] = [round(cands[0][3], 3), round(cands[1][3], 3)]; checks['ear_score'] = [round(cands[0][4], 2), round(cands[1][4], 2)]
    checks['iou_ok'] = bool(cands[0][0] - cands[1][0] > .15)                             # combined silhouette + ear evidence must clearly prefer one orientation
    checks['coin_in_slot_at_release'] = bool(cc[2] - .01425 < TABLE + .1089)
    side_ok = (-.4 - .01 <= bp[0] <= -.3 + .01) or (.3 - .01 <= bp[0] <= .4 + .01)
    checks['config_ranges_ok'] = bool(side_ok and -.2 - .01 <= bp[1] <= -.15 + .01 and -.43 <= stand_p[0] <= .43 and -.2 - .01 <= stand_p[1] <= -.05 + .01
                                      and min(abs(yb - 89), abs(yb + 86)) <= 15 + 3 and abs(np.degrees(yaw_s)) <= 30 + 3)
    hz = float(d['additional_info__frequency'])
    evt = {'picker': picker, 'pick_closed_s': g_pick / hz, 'handover': o_close is not None, 'handover_closed_s': (o_close[2] / hz) if o_close is not None else None,
           'picker_open_s': (p_open[2] / hz) if p_open is not None else None, 'release_arm': rel_arm, 'release_s': rel / hz, 'release_open_s': rel_open[2] / hz}
    res = {'stand': {'pos': stand_p.tolist(), 'yaw_deg': float(np.degrees(yaw_s)), 'quat_wxyz': list(Rs.as_quat()[[3, 0, 1, 2]])},
           'coin': {'origin': coin_origin.tolist(), 'quat_wxyz': list(Rc.as_quat()[[3, 0, 1, 2]]), 'centre': coin_c.tolist()},
           'bank': {'pos': bp.tolist(), 'yaw_deg': float(yb), 'quat_wxyz': list(R.from_euler('z', yaw_b).as_quat()[[3, 0, 1, 2]]), 'silhouette_iou': iou, 'ear_score': ear, 'score_other_orientation': cands[1][0]},
           'events': evt, 'checks': checks, 'valid': all(v for k, v in checks.items() if k.endswith('_ok') or k == 'coin_in_slot_at_release')}
    return res


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('ep'); ap.add_argument('assets'); ap.add_argument('out'); a = ap.parse_args()
    r = fit(a.ep, a.assets); Path(a.out).write_text(json.dumps(r, indent=1))
    print(Path(a.ep).name, 'valid' if r['valid'] else 'INVALID', r['events'], {k: v for k, v in r['checks'].items()}, 'bank', np.round(r['bank']['pos'], 3).tolist(), round(r['bank']['yaw_deg'], 1), 'stand', np.round(r['stand']['pos'], 3).tolist())
