"""Render within-engine analytic-disc versus shallow-CoACD-plate replays.

MuJoCo poses are generated from its native XML. SAP/Warp poses are replayed
from native trace.npz files produced by run_thin_shape_stack_comparison.py.
The renderer is MuJoCo only; it does not re-simulate SAP dynamics.
"""
from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path

import imageio.v3 as iio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "exp-results" / "site-video-demos"
MJC_RUNNER = ROOT / "mujoco-sim" / "learn_mujoco" / "concave_plate_experiments" / "run_thin_shape_stack_comparison.py"


def import_runner():
    spec = importlib.util.spec_from_file_location("thin_shape_runner", MJC_RUNNER)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def model_for(shape: str):
    runner = import_runner()
    model = mujoco.MjModel.from_xml_string(runner.xml(shape))
    model.vis.global_.offwidth = 720
    model.vis.global_.offheight = 520
    addrs = []
    for i in range(3):
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"body_{i}")
        addrs.append(int(model.jnt_qposadr[int(model.body_jntadr[body])]))
    return model, np.array(addrs)


def mujoco_poses(shape: str, nframes: int) -> np.ndarray:
    runner = import_runner()
    model, addrs = model_for(shape)
    data = mujoco.MjData(model)
    steps = round(runner.DURATION / runner.DT)
    targets = np.linspace(0, steps, nframes).round().astype(int)
    pose = np.empty((nframes, 3, 7))
    frame = 0
    for step in range(steps + 1):
        while frame < nframes and step == targets[frame]:
            pose[frame] = np.array([data.qpos[a:a + 7] for a in addrs])
            frame += 1
        if step < steps:
            mujoco.mj_step(model, data)
    return pose


def sap_poses(trace_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    disc = np.load(trace_dir / "thin_disc" / "trace.npz")["q"]
    plate = np.load(trace_dir / "shallow_concave_plate" / "trace.npz")["q"]
    # Warp free-joint rotation is xyzw; MuJoCo qpos consumes wxyz.
    def xyzw_to_wxyz(q: np.ndarray) -> np.ndarray:
        out = q.copy()
        out[..., 3:7] = q[..., [6, 3, 4, 5]]
        return out
    return xyzw_to_wxyz(disc), xyzw_to_wxyz(plate)


def font(size: int):
    for path in ("/System/Library/Fonts/Supplemental/Arial.ttf", "/System/Library/Fonts/Helvetica.ttc"):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def pane(model, addrs, pose, title: str, subtitle: str) -> np.ndarray:
    data = mujoco.MjData(model)
    for i, address in enumerate(addrs):
        data.qpos[address:address + 7] = pose[i]
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, width=720, height=520) as renderer:
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = (0.0, 0.0, 0.006)
        camera.distance = 0.34
        camera.azimuth = 128
        camera.elevation = -4
        renderer._scene_option.geomgroup[3] = 0  # hide collision hulls; show common visual body.
        renderer.update_scene(data, camera=camera)
        rgb = renderer.render()
    image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(image, "RGBA")
    draw.rounded_rectangle((16, 14, 704, 78), radius=10, fill=(7, 18, 33, 210))
    draw.text((30, 24), title, font=font(22), fill=(244, 248, 255, 255))
    draw.text((30, 52), subtitle, font=font(14), fill=(208, 221, 238, 255))
    return np.asarray(image)


def render(engine: str, disc: np.ndarray, plate: np.ndarray, out_path: Path) -> None:
    disc_model, disc_addrs = model_for("thin_disc")
    plate_model, plate_addrs = model_for("shallow_concave_plate")
    frames = []
    for i in range(len(disc)):
        left = pane(disc_model, disc_addrs, disc[i], "Analytic thin discs", "12–13 contacts; settles within 2 s")
        right = pane(plate_model, plate_addrs, plate[i], "Shallow CoACD plates", "121–147 contacts; still moving at 2 s")
        canvas = Image.new("RGB", (1440, 572), (12, 19, 31))
        canvas.paste(Image.fromarray(left), (0, 52)); canvas.paste(Image.fromarray(right), (720, 52))
        draw = ImageDraw.Draw(canvas, "RGBA")
        draw.rectangle((0, 0, 1440, 52), fill=(11, 26, 45, 255))
        draw.text((22, 14), f"{engine}: same thickness, mass, initial poses and native contact setting", font=font(22), fill=(242, 247, 255, 255))
        draw.text((1260, 18), f"t = {2.0 * i / (len(disc)-1):.2f} s", font=font(15), fill=(185, 205, 228, 255))
        frames.append(np.asarray(canvas))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    iio.imwrite(out_path, frames, fps=30, codec="libx264", pixelformat="yuv420p")
    print(out_path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sap-trace-dir", type=Path)
    parser.add_argument("--mujoco-only", action="store_true")
    args = parser.parse_args()
    nframes = 180
    render("MuJoCo", *[mujoco_poses(shape, nframes) for shape in ("thin_disc", "shallow_concave_plate")], OUT / "26_mujoco_thin_shape_disc_vs_plate.mp4")
    if args.mujoco_only:
        return
    if args.sap_trace_dir is None:
        parser.error("--sap-trace-dir is required unless --mujoco-only is selected")
    sap_disc, sap_plate = sap_poses(args.sap_trace_dir)
    render("SAP/Warp", sap_disc, sap_plate, OUT / "27_sap_thin_shape_disc_vs_plate.mp4")


if __name__ == "__main__":
    main()
