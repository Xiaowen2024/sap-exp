"""3-D trace replay of the adjacent MuJoCo N4 pinch pass/fail cases.

No physics is re-solved while rendering. The exact recorded object height and
integrated recorded pad velocity drive the physical-object panels. A separate,
explicitly magnified measurement scale reveals the 57-mm decision boundary.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import imageio.v3 as iio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "exp-results/simple-pinch-v3/mujoco/refined-boundary-20261017/N4"
OUT = ROOT / "exp-results/site-video-demos/23_mujoco_pinch_refined_boundary_3d.mp4"
CASES = (
    ("LAST FAILED", "m0.307706524kg", (249, 105, 94)),
    ("FIRST PASSED", "m0.307698926kg", (80, 200, 145)),
)
FPS, DURATION_S = 24, 8
PANEL_W, PANEL_H = 640, 470
WIDTH, HEIGHT = PANEL_W * 2, 720


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    paths = (
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf" if bold else "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    )
    for path in paths:
        if Path(path).is_file():
            return ImageFont.truetype(path, size=size)
    return ImageFont.load_default()


FONT = font(23)
SMALL = font(18)
BIG = font(30, True)


def load_case(label: str, directory: str, color: tuple[int, int, int]) -> dict:
    case_dir = BASE / directory
    with (case_dir / "trace.csv").open() as stream:
        rows = [row for row in csv.DictReader(stream) if row["phase"] == "lift"]
    metrics = json.loads((case_dir / "metrics.json").read_text())
    t = np.array([float(row["time_s"]) for row in rows])
    cube_z = np.array([float(row["cube_z_m"]) for row in rows])
    pad_vz = np.array([float(row["pad_vz_m_s"]) for row in rows])
    dt = np.diff(t)
    pad_z = np.concatenate(([0.0], np.cumsum(0.5 * (pad_vz[1:] + pad_vz[:-1]) * dt)))
    assert abs((cube_z[-1] - cube_z[0]) * 1000 - metrics["lift_follow_mm"]) < 1e-8
    assert metrics["normal_gate_pass"] is True
    return {
        "label": label, "color": color, "metrics": metrics, "time": t,
        "cube_z": cube_z, "pad_z": pad_z,
    }


def scene_model() -> mujoco.MjModel:
    xml = """<mujoco model='pinch_boundary_replay'>
      <visual><global offwidth='640' offheight='470'/>
        <rgba haze='0.04 0.07 0.10 1'/></visual>
      <worldbody>
        <camera name='cam' pos='.22 -.29 .20' xyaxes='.804 .595 0 -.27 .364 .891'/>
        <light pos='0 -0.4 1' directional='true'/>
        <geom type='plane' pos='0 0 -.035' size='1 1 .1' rgba='.16 .21 .27 1'/>
        <body name='cube'><freejoint/>
          <geom type='box' size='.02 .02 .02' rgba='.96 .47 .15 1'/></body>
        <body name='left'><freejoint/>
          <geom type='box' size='.01 .04 .04' rgba='.52 .73 .91 0.85'/></body>
        <body name='right'><freejoint/>
          <geom type='box' size='.01 .04 .04' rgba='.52 .73 .91 0.85'/></body>
      </worldbody></mujoco>"""
    return mujoco.MjModel.from_xml_string(xml)


def set_pose(data: mujoco.MjData, item: dict, sim_t: float) -> float:
    cube_z = float(np.interp(sim_t, item["time"], item["cube_z"]))
    pad_z = float(np.interp(sim_t, item["time"], item["pad_z"]))
    positions = ((0.0, cube_z), (-0.0298, pad_z), (0.0298, pad_z))
    for index, (x, z) in enumerate(positions):
        q = 7 * index
        data.qpos[q:q + 3] = [x, 0.0, z + 0.02]
        data.qpos[q + 3:q + 7] = [1.0, 0.0, 0.0, 0.0]
    return (cube_z - float(item["cube_z"][0])) * 1000


def draw_bar(draw: ImageDraw.ImageDraw, x: int, y: int, value: float,
             lo: float, hi: float, color: tuple[int, int, int], zoom: bool) -> None:
    left, right = x + 35, x + PANEL_W - 35
    draw.rounded_rectangle((left, y, right, y + 17), radius=7, fill=(40, 55, 70))
    marker = left + (57.0 - lo) / (hi - lo) * (right - left)
    draw.line((marker, y - 6, marker, y + 23), fill=(248, 217, 92), width=3)
    value_x = left + np.clip((value - lo) / (hi - lo), 0.0, 1.0) * (right - left)
    draw.ellipse((value_x - 7, y + 1, value_x + 7, y + 15), fill=color, outline="white", width=2)
    if zoom:
        draw.text((left, y + 27), "56.90 mm", font=SMALL, fill=(180, 196, 210))
        draw.text((right - 92, y + 27), "57.10 mm", font=SMALL, fill=(180, 196, 210))
    else:
        draw.text((left, y + 27), "0 mm", font=SMALL, fill=(180, 196, 210))
        draw.text((right - 70, y + 27), "60 mm", font=SMALL, fill=(180, 196, 210))


def annotate(frame_left: np.ndarray, frame_right: np.ndarray, cases: list[dict],
             follows: list[float], sim_t: float, zoom: bool) -> np.ndarray:
    image = Image.new("RGB", (WIDTH, HEIGHT), (12, 23, 34))
    image.paste(Image.fromarray(frame_left), (0, 110))
    image.paste(Image.fromarray(frame_right), (PANEL_W, 110))
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 110, WIDTH, 188), fill=(12, 23, 34))
    draw.text((WIDTH / 2, 15), "MuJoCo simple pinch | same N4 setup, mass-only boundary",
              font=BIG, fill="white", anchor="mt")
    draw.text((WIDTH / 2, 62), "Recorded 3-D motion | yellow line = 57.000 mm task gate",
              font=FONT, fill=(203, 218, 231), anchor="mt")
    draw.line((PANEL_W, 110, PANEL_W, 580), fill=(155, 176, 194), width=2)
    for index, (item, follow) in enumerate(zip(cases, follows)):
        x = PANEL_W * index
        metrics = item["metrics"]
        color = item["color"]
        draw.text((x + 26, 125), item["label"], font=BIG, fill=color)
        draw.text((x + 26, 168),
                  f"r = {metrics['capacity_ratio_2muN_over_mg']:.6f}  |  m = {metrics['mass_kg']:.9f} kg",
                  font=SMALL, fill="white")
        draw.text((x + 26, 575), f"cube lift: {follow:.5f} mm", font=FONT, fill="white")
        draw_bar(draw, x, 615, follow, 56.90 if zoom else 0.0,
                 57.10 if zoom else 60.0, color, zoom)
        draw.text((x + PANEL_W - 25, 575),
                  "FAIL" if not metrics["grasp_success"] else "PASS",
                  font=BIG, fill=color, anchor="ra")
    footer = ("Final gate magnified to 56.90-57.10 mm; body poses are NOT exaggerated"
              if zoom else f"Lift/hold simulation time: {sim_t:.2f} s  |  final 3 s shows the gate magnified")
    draw.text((WIDTH / 2, HEIGHT - 17), footer, font=SMALL,
              fill=(203, 218, 231), anchor="mb")
    return np.asarray(image)


def main() -> None:
    cases = [load_case(*case) for case in CASES]
    assert cases[0]["metrics"]["grasp_success"] is False
    assert cases[1]["metrics"]["grasp_success"] is True
    model = scene_model()
    data = mujoco.MjData(model)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    frames = []
    with mujoco.Renderer(model, width=PANEL_W, height=PANEL_H) as renderer:
        for frame in range(FPS * DURATION_S):
            video_t = frame / FPS
            zoom = video_t >= 5.0
            sim_t = min(video_t / 5.0 * 1.25, 1.25)
            panes = []
            follows = []
            for item in cases:
                follow = set_pose(data, item, sim_t)
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera="cam")
                panes.append(renderer.render().copy())
                follows.append(follow)
            frames.append(annotate(panes[0], panes[1], cases, follows, sim_t, zoom))
    iio.imwrite(OUT, frames, fps=FPS, codec="libx264", pixelformat="yuv420p")
    print(json.dumps({"output": str(OUT), "cases": [
        {"mass_kg": item["metrics"]["mass_kg"],
         "r": item["metrics"]["capacity_ratio_2muN_over_mg"],
         "follow_mm": item["metrics"]["lift_follow_mm"],
         "grasp_success": item["metrics"]["grasp_success"]}
        for item in cases
    ]}, indent=2))


if __name__ == "__main__":
    main()
