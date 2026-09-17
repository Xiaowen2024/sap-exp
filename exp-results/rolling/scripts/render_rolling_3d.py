"""Render full 5x5-ball rolling-test videos for MuJoCo and recorded SAP poses."""
from pathlib import Path
import sys
import imageio.v2 as imageio
import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from run_mujoco_rolling_benchmark import DT, DURATION, FORCE, model_xml

ROOT = Path(__file__).resolve().parents[1]


def make_model():
    m = mujoco.MjModel.from_xml_string(model_xml())
    m.geom_rgba[0] = [.92, .94, .97, 1]  # floor
    m.geom_rgba[1] = [.03, .10, .20, 1]  # 10 kg platform
    m.geom_rgba[2:, :] = [.16, .38, .70, 1]
    return m


def camera():
    c = mujoco.MjvCamera(); c.lookat = np.array([0., 0., .15]); c.distance = 31.; c.azimuth = 45.; c.elevation = -25.
    return c


def render_states(out: Path, states: np.ndarray, fps: int = 30):
    m, d, c = make_model(), mujoco.MjData(make_model()), camera()
    # Re-create d against the same model used by its renderer.
    m = make_model(); d = mujoco.MjData(m); renderer = mujoco.Renderer(m, width=640, height=480)
    frames = []
    for q in states:
        d.qpos[:] = q; d.qvel[:] = 0.; mujoco.mj_forward(m, d); renderer.update_scene(d, camera=c); frames.append(renderer.render())
    renderer.close(); imageio.mimsave(out, frames, fps=fps)


def mujoco_states():
    m, d = make_model(), mujoco.MjData(make_model())
    m = make_model(); d = mujoco.MjData(m); box = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "box")
    result = []
    for step in range(int(DURATION / DT)):
        if step % 10 == 0: result.append(d.qpos.copy())
        d.xfrc_applied[box, :3] = [FORCE*.5, FORCE*np.sqrt(3.)/2, 0.]
        mujoco.mj_step(m, d)
    return np.asarray(result)


render_states(ROOT / "mujoco" / "rolling-3d.mp4", mujoco_states())
sap_q = ROOT / "sap-cuda" / "all-body-q-10ms.npy"
if sap_q.exists(): render_states(ROOT / "sap-cuda" / "rolling-3d.mp4", np.load(sap_q))
