"""Render representative success/failure traces from the MuJoCo thin-disc sweep."""
from __future__ import annotations

from pathlib import Path

import imageio.v3 as iio
import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "exp-results" / "site-video-demos"
INITIAL = ((0.0, 0.0, 0.030), (0.003, -0.002, 0.070), (-0.002, 0.002, 0.110))
NAMES = ("disc_bottom", "disc_middle", "disc_top")
DURATION = 2.0


def xml(dt: float, timeconst: float, dampratio: float = 1.0) -> str:
    bodies = "".join(
        f'<body name="{name}" pos="{x} {y} {z}"><freejoint/>'
        f'<geom type="cylinder" size=".101 .002" mass=".333" rgba="{color}"/>'
        "</body>"
        for name, (x, y, z), color in zip(
            NAMES, INITIAL, (".15 .55 .95 1", ".95 .46 .12 1", ".20 .78 .42 1")
        )
    )
    return f'''<mujoco model="thin_disc_replay">
<option timestep="{dt}" gravity="0 0 -9.81" integrator="implicitfast" solver="Newton" iterations="100" tolerance="1e-10" cone="elliptic"/>
<size nconmax="2048" njmax="4096"/>
<visual><global offwidth="960" offheight="640"/></visual>
<default><geom friction="0.8 0.005 0.0001" solref="{timeconst} {dampratio}" solimp=".95 .99 .002 .5 2"/></default>
<worldbody><camera name="cam" pos=".33 -.42 .25" xyaxes=".78 .62 0 -.29 .37 .88"/><light pos="0 -1 1.5" directional="true"/><geom type="plane" size="1 1 .1" rgba=".18 .22 .28 1"/>{bodies}</worldbody>
</mujoco>'''


def simulate(dt: float, timeconst: float) -> tuple[mujoco.MjModel, np.ndarray]:
    model = mujoco.MjModel.from_xml_string(xml(dt, timeconst))
    data = mujoco.MjData(model)
    addresses = []
    for name in NAMES:
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        joint = int(model.body_jntadr[body])
        addresses.append(int(model.jnt_qposadr[joint]))
    steps = round(DURATION / dt)
    capture = np.linspace(0, steps, 240).round().astype(int)
    poses = np.empty((len(capture), 3, 7))
    frame = 0
    for step in range(steps + 1):
        while frame < len(capture) and step == capture[frame]:
            for i, address in enumerate(addresses):
                poses[frame, i] = data.qpos[address:address + 7]
            frame += 1
        if step < steps:
            mujoco.mj_step(model, data)
    return model, poses


def render(tag: str, dt: float, timeconst: float) -> None:
    model, poses = simulate(dt, timeconst)
    render_poses(f"mujoco_{tag}", model, poses)


def render_poses(tag: str, model: mujoco.MjModel, poses: np.ndarray) -> None:
    data = mujoco.MjData(model)
    addresses = []
    for name in NAMES:
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        addresses.append(int(model.jnt_qposadr[int(model.body_jntadr[body])]))
    frames = []
    with mujoco.Renderer(model, width=960, height=640) as renderer:
        for pose in poses:
            for i, address in enumerate(addresses):
                data.qpos[address:address + 7] = pose[i]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera="cam")
            frames.append(renderer.render())
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"thin_disc_{tag}_raw.mp4"
    iio.imwrite(path, frames, fps=30, codec="libx264", pixelformat="yuv420p")
    print(path)


if __name__ == "__main__":
    # Success: balanced operating point from the sweep.
    render("success", 0.00025, 0.001)
    # Failure: 16-ms solref, the consistently wrong-order soft-contact regime.
    render("failure", 0.0005, 0.016)
