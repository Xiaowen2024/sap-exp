import coacd, trimesh, numpy as np, sys, json, time
name, thr, maxh = sys.argv[1], float(sys.argv[2]), int(sys.argv[3])
m = trimesh.load(f'geom/{name}.obj', force='mesh'); coacd.set_log_level('warn'); t = time.time()
parts = coacd.run_coacd(coacd.Mesh(np.asarray(m.vertices), np.asarray(m.faces)), threshold=thr, max_convex_hull=maxh, preprocess_mode='on',
                        resolution=2000, mcts_nodes=20, mcts_iterations=150, mcts_max_depth=3, pca=False, merge=True, seed=20261118,
                        max_ch_vertex=64, decimate=True)
import os; os.makedirs(f'geom/{name}_pieces', exist_ok=True)
nv = []
for i, (v, f) in enumerate(parts):
    p = trimesh.Trimesh(v, f, process=True).convex_hull; nv.append(len(p.vertices)); p.export(f'geom/{name}_pieces/piece_{i:03d}.obj')
print(name, 'pieces', len(parts), 'max verts', max(nv), 'wall', round(time.time() - t), flush=True)
