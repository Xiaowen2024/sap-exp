"""Render top-down trajectory replays from the recorded rolling-test traces."""
from pathlib import Path
import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
CASES = {"mujoco": ROOT / "mujoco" / "trace.csv", "sap-cuda": ROOT / "sap-cuda" / "trace.csv"}
W, H, SCALE, FPS = 900, 700, 26, 30


def make_video(name: str, trace_path: Path) -> None:
    trace = np.loadtxt(trace_path, delimiter=",", skiprows=1)
    t, box_v, ball_v = trace[:, 0], trace[:, 1:4], trace[:, 4:7]
    dt = np.diff(t, prepend=t[0]); box_p = np.cumsum(box_v * dt[:, None], axis=0)
    ball_p = np.cumsum(ball_v * dt[:, None], axis=0) + np.array([-4.0, -4.0, 0.0])
    font = ImageFont.load_default(); frames = []
    for k in range(0, len(t), max(1, int(round(1 / (FPS * np.median(np.diff(t))))))):
        im = Image.new("RGB", (W, H), "#111827"); d = ImageDraw.Draw(im)
        d.rectangle((35, 100, W - 35, H - 35), fill="#192533", outline="#64748b", width=2)
        cx, cy = W // 2, H // 2 + 35
        bx, by = cx + box_p[k, 0] * SCALE, cy - box_p[k, 1] * SCALE
        # The moving 20 m square box is intentionally scaled down for the overview.
        half = 10 * SCALE; d.rectangle((bx-half, by-half, bx+half, by+half), outline="#37d399", width=3)
        px, py = cx + ball_p[k, 0] * SCALE, cy - ball_p[k, 1] * SCALE
        d.ellipse((px-10, py-10, px+10, py+10), fill="#fb923c", outline="white")
        d.text((35, 25), f"{name.upper()} rolling-test trajectory replay", fill="white", font=font)
        d.text((35, 48), f"t={t[k]:.3f} s  |  green: 10 kg box  orange: selected corner ball", fill="#cbd5e1", font=font)
        d.text((35, 70), "Positions reconstructed by integrating recorded velocities; z velocity is not shown in this top-down replay.", fill="#94a3b8", font=font)
        frames.append(np.asarray(im))
    imageio.mimsave(ROOT / name / "rolling-trajectory-replay.mp4", frames, fps=FPS)


for case, path in CASES.items():
    make_video(case, path)
