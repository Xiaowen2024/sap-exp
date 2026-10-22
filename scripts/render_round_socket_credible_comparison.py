"""Render entered-and-credible vs entered-but-not-credible round-socket comparisons.

For both MuJoCo and SAP/Warp, both panes show the shared 32-piece CoACD round
socket (c=2.0 mm, offset=1.0 mm, tilt=1.0 deg, 2.0 s). In each video, BOTH
panes physically enter the socket and settle near z=38.1-38.3 mm, demonstrating
why final pose/depth alone ("entered") is not sufficient: the left pane keeps
peak overlap below 0.1c = 0.200 mm (credible), whereas the right pane exceeds
0.1c during entry (entered, not credible).
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

import imageio.v3 as iio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "mujoco-sim" / "learn_mujoco" / "peg_in_hole_experiments"
XML = HERE / "peg_convex_decomposition.xml"
OUT = ROOT / "exp-results" / "site-video-demos"
SAP_TRACE_DIR = ROOT / "exp-results" / "peg-in-hole" / "sap-round-socket-equal-budget"

ASSETS = {"blind_socket.obj": (HERE / "assets" / "blind_socket.obj").read_bytes()}
ASSETS.update({p.name: p.read_bytes() for p in (HERE / "assets" / "collision").glob("socket_piece_*.obj")})

DURATION_S = 2.0
CLEARANCE_MM = 2.0
THRESHOLD_MM = 0.200  # 0.1 * c


def font(size: int):
    for path in ("/System/Library/Fonts/Supplemental/Arial.ttf", "/System/Library/Fonts/Helvetica.ttc"):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def source_for(cfg: dict[str, object]) -> str:
    source = XML.read_text()
    option = (
        f'<option timestep="{float(cfg["dt_ms"]) / 1000:g}" gravity="0 0 -9.81" '
        f'integrator="{cfg["integrator"]}" cone="elliptic" solver="{cfg["solver"]}"/>'
    )
    source = re.sub(r"<option\s+[^>]*/>", option, source, count=1)
    solref = f'{float(cfg["timeconst_ms"]) / 1000:g} {float(cfg["dampratio"]):g}'
    solimp = f'{float(cfg["d0"]):g} {float(cfg["dwidth"]):g} .001 .5 2'
    source = source.replace('solref="0.004 1"', f'solref="{solref}"')
    source = source.replace('solimp="0.95 0.99 0.001 0.5 2"', f'solimp="{solimp}"')
    return source


def simulate_mujoco(cfg: dict[str, object], nframes: int = 180) -> dict[str, np.ndarray]:
    model = mujoco.MjModel.from_xml_string(source_for(cfg), assets=ASSETS)
    data = mujoco.MjData(model)
    peg = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "peg_geom")
    table = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table")

    steps = round(DURATION_S / model.opt.timestep)
    all_q = np.empty((steps + 1, 7))
    all_pen_mm = np.zeros(steps + 1)
    all_pmax_mm = np.zeros(steps + 1)
    running_pmax = 0.0

    for step in range(steps + 1):
        all_q[step] = data.qpos[:7]
        if step == steps:
            break
        mujoco.mj_step(model, data)
        pen_mm = 0.0
        for i in range(data.ncon):
            contact = data.contact[i]
            if peg in (contact.geom1, contact.geom2):
                other = contact.geom2 if contact.geom1 == peg else contact.geom1
                if other != table:
                    pen_mm = max(pen_mm, float(-contact.dist * 1000.0))
        running_pmax = max(running_pmax, pen_mm)
        all_pen_mm[step + 1] = pen_mm
        all_pmax_mm[step + 1] = running_pmax

    idx = np.linspace(0, steps, nframes).round().astype(int)
    # For each frame interval, preserve peak instantaneous overlap within that window
    frame_pen = np.zeros(nframes)
    for k in range(nframes):
        lo = idx[k - 1] if k > 0 else 0
        hi = idx[k]
        frame_pen[k] = float(all_pen_mm[lo : hi + 1].max())
    return {
        "t": np.linspace(0.0, DURATION_S, nframes),
        "q": all_q[idx],
        "pen_mm": frame_pen,
        "pmax_mm": all_pmax_mm[idx],
    }


def load_sap_trace(npz_path: Path, nframes: int = 180) -> dict[str, np.ndarray]:
    raw = np.load(npz_path)
    all_q = raw["q"].copy()
    # Warp free-joint quaternion is xyzw; MuJoCo qpos is wxyz.
    q_wxyz = all_q.copy()
    q_wxyz[:, 3:7] = all_q[:, [6, 3, 4, 5]]
    all_pen_mm = raw["pen_mm"]
    all_pmax_mm = raw["pmax_mm"]
    steps = len(all_q) - 1
    idx = np.linspace(0, steps, nframes).round().astype(int)
    frame_pen = np.zeros(nframes)
    for k in range(nframes):
        lo = idx[k - 1] if k > 0 else 0
        hi = idx[k]
        frame_pen[k] = float(all_pen_mm[lo : hi + 1].max())
    return {
        "t": np.linspace(0.0, DURATION_S, nframes),
        "q": q_wxyz[idx],
        "pen_mm": frame_pen,
        "pmax_mm": all_pmax_mm[idx],
    }


def render_model():
    model = mujoco.MjModel.from_xml_string(XML.read_text(), assets=ASSETS)
    model.vis.global_.offwidth = 720
    model.vis.global_.offheight = 524
    return model


def draw_pane(
    model: mujoco.MjModel,
    pose: np.ndarray,
    pen_mm: float,
    pmax_mm: float,
    final_pmax_mm: float,
    title: str,
    subtitle: str,
    credible: bool,
) -> np.ndarray:
    data = mujoco.MjData(model)
    data.qpos[:7] = pose
    mujoco.mj_forward(model, data)
    opt = mujoco.MjvOption()
    opt.geomgroup[3] = 0
    with mujoco.Renderer(model, width=720, height=524) as renderer:
        renderer.update_scene(data, camera="peg_camera", scene_option=opt)
        rgb = renderer.render()

    image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(image, "RGBA")
    # Top banner
    draw.rounded_rectangle((16, 14, 704, 82), radius=10, fill=(7, 18, 33, 215))
    badge_color = (42, 152, 92, 255) if credible else (214, 118, 36, 255)
    draw.rounded_rectangle((536, 22, 692, 50), radius=6, fill=badge_color)
    badge_text = "CREDIBLE" if credible else "NOT CREDIBLE"
    draw.text((548, 28), badge_text, font=font(15), fill=(255, 255, 255, 255))
    draw.text((28, 22), title, font=font(20), fill=(244, 248, 255, 255))
    draw.text((28, 52), subtitle, font=font(14), fill=(204, 218, 236, 255))

    # Bottom telemetry box
    z_mm = float(pose[2] * 1000.0)
    r_mm = float(np.hypot(pose[0], pose[1]) * 1000.0)
    exceeded = pmax_mm > THRESHOLD_MM
    p_color = (255, 130, 120, 255) if exceeded else (130, 235, 168, 255)
    draw.rounded_rectangle((16, 424, 704, 510), radius=10, fill=(7, 18, 33, 218))
    draw.text(
        (28, 434),
        f"Pose: z = {z_mm:5.2f} mm, radial r = {r_mm:4.2f} mm   (entered gate: z <= 45 mm, r <= 2.0 mm)",
        font=font(14),
        fill=(228, 236, 248, 255),
    )
    draw.text(
        (28, 460),
        f"Overlap: window peak = {pen_mm:6.3f} mm | running p_max = {pmax_mm:6.3f} mm  (screen <= 0.200 mm)",
        font=font(14),
        fill=p_color,
    )
    status_line = (
        f"Final: entered = YES, p_max = {final_pmax_mm:.3f} mm ({final_pmax_mm / CLEARANCE_MM:.3f}c <= 0.1c) -> PASS"
        if credible
        else f"Final: entered = YES, but p_max = {final_pmax_mm:.3f} mm ({final_pmax_mm / CLEARANCE_MM:.2f}c > 0.1c) -> FAIL screen"
    )
    draw.text((28, 484), status_line, font=font(13), fill=(204, 218, 236, 255))
    return np.asarray(image)


def build_video(
    header: str,
    left_trace: dict[str, np.ndarray],
    left_title: str,
    left_sub: str,
    right_trace: dict[str, np.ndarray],
    right_title: str,
    right_sub: str,
    out_path: Path,
) -> None:
    model = render_model()
    nframes = len(left_trace["t"])
    left_final_pmax = float(left_trace["pmax_mm"][-1])
    right_final_pmax = float(right_trace["pmax_mm"][-1])
    frames = []
    for i in range(nframes):
        left = draw_pane(
            model,
            left_trace["q"][i],
            float(left_trace["pen_mm"][i]),
            float(left_trace["pmax_mm"][i]),
            left_final_pmax,
            left_title,
            left_sub,
            credible=True,
        )
        right = draw_pane(
            model,
            right_trace["q"][i],
            float(right_trace["pen_mm"][i]),
            float(right_trace["pmax_mm"][i]),
            right_final_pmax,
            right_title,
            right_sub,
            credible=False,
        )
        canvas = Image.new("RGB", (1440, 576), (12, 19, 31))
        canvas.paste(Image.fromarray(left), (0, 52))
        canvas.paste(Image.fromarray(right), (720, 52))
        draw = ImageDraw.Draw(canvas, "RGBA")
        draw.rectangle((0, 0, 1440, 52), fill=(11, 26, 45, 255))
        draw.text((22, 14), header, font=font(21), fill=(242, 247, 255, 255))
        draw.text((1280, 17), f"t = {float(left_trace['t'][i]):.2f} s", font=font(15), fill=(185, 205, 228, 255))
        frames.append(np.asarray(canvas))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    iio.imwrite(out_path, frames, fps=30, codec="libx264", pixelformat="yuv420p")
    print(out_path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mujoco-only", action="store_true")
    parser.add_argument("--sap-only", action="store_true")
    args = parser.parse_args()

    if not args.sap_only:
        mj_cred = simulate_mujoco(
            dict(dt_ms=0.25, timeconst_ms=0.75, dampratio=1.0, d0=0.70, dwidth=0.995, solver="Newton", integrator="implicitfast")
        )
        mj_not = simulate_mujoco(
            dict(dt_ms=0.25, timeconst_ms=4.0, dampratio=1.0, d0=0.70, dwidth=0.995, solver="Newton", integrator="implicitfast")
        )
        build_video(
            "MuJoCo shared-CoACD round socket (c=2 mm, x=1 mm, 1°): entered & credible vs entered but not credible",
            mj_cred,
            "Entered & credible (run #1)",
            "dt=0.25 ms, solref=0.75 ms, d0=0.70 — p_max=0.056 mm <= 0.20 mm",
            mj_not,
            "Entered, NOT credible (run #7)",
            "dt=0.25 ms, solref=4.0 ms, d0=0.70 — enters, but p_max=1.349 mm > 0.20 mm",
            OUT / "28_mujoco_round_socket_credible_vs_not.mp4",
        )

    if not args.mujoco_only:
        sap_cred = load_sap_trace(SAP_TRACE_DIR / "run004_credible_trace.npz")
        sap_not = load_sap_trace(SAP_TRACE_DIR / "run008_entered_not_credible_trace.npz")
        build_video(
            "SAP/Warp shared-CoACD round socket (c=2 mm, x=1 mm, 1°): entered & credible vs entered but not credible",
            sap_cred,
            "Entered & credible (run #4)",
            "dt=0.25 ms, k_pair=1e4 N/m, tau=3 ms, gap=0.1 mm — p_max=0.061 mm <= 0.20 mm",
            sap_not,
            "Entered, NOT credible (run #8)",
            "dt=0.25 ms, k_pair=1e5 N/m, tau=3 ms, gap=0.1 mm — enters, but p_max=5.384 mm",
            OUT / "29_sap_round_socket_credible_vs_not.mp4",
        )


if __name__ == "__main__":
    main()
