"""
Render publication-quality schematic illustration explaining:
'Normal contact compliance bandwidth (tau_n, k_n) relative to external tilting rate (180 Hz)'
for Slope Slider Task-3.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["DejaVu Sans", "Arial", "Helvetica"],
    "axes.edgecolor": "#b0bec5",
    "axes.linewidth": 0.8,
})

fig = plt.figure(figsize=(14.4, 8.4), dpi=220, facecolor="#ffffff")
gs = fig.add_gridspec(2, 2, height_ratios=[1.15, 0.85], hspace=0.28, wspace=0.26)

ax_left = fig.add_subplot(gs[0, 0])
ax_right = fig.add_subplot(gs[0, 1])
ax_wave = fig.add_subplot(gs[1, :])

# Global styling helpers
DARK_NAVY = "#1a2530"
ACCENT_BLUE = "#1565c0"
SOFT_RED = "#c62828"
STIFF_GREEN = "#2e7d32"
BOARD_BROWN = "#b08968"
BOARD_DARK = "#6f4e37"

# -------------------------------------------------------------
# PANEL A: Soft & Lagging Contact (Default)
# -------------------------------------------------------------
ax_left.set_title("A. Soft & Lagging Contact (Default SAP)\n$k_e=2,500$ N/m, $\\tau_n=10$ ms $(> \\Delta t_{\\mathrm{step}}=5.56$ ms)",
                 fontsize=11.2, fontweight="bold", color=SOFT_RED, pad=10)
ax_left.set_xlim(-0.10, 1.12)
ax_left.set_ylim(-0.14, 0.48)
ax_left.axis("off")

# Draw tilted board
theta_deg = 18
theta = np.radians(theta_deg)
board_len = 1.05
board_thick = 0.075
board_x0, board_y0 = -0.02, 0.04

bx = np.array([0, board_len, board_len, 0])
by = np.array([0, 0, -board_thick, -board_thick])
R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
board_rot = R @ np.vstack([bx, by])
board_poly = np.vstack([board_rot[0] + board_x0, board_rot[1] + board_y0]).T
ax_left.add_patch(patches.Polygon(board_poly, closed=True, facecolor=BOARD_BROWN, edgecolor=BOARD_DARK, lw=1.8, zorder=2))

# 180 Hz Dynamic Tilting rotation indicator
arc = patches.Arc((board_x0, board_y0), 0.36, 0.36, angle=0, theta1=0, theta2=theta_deg, color="#e65100", lw=2.2, ls="--", zorder=3)
ax_left.add_patch(arc)
ax_left.annotate("Dynamic 180 Hz\nTilting Motion\n($\\Delta t = 5.56$ ms)", xy=(board_x0 + 0.15, board_y0 - 0.03),
                 fontsize=8.8, color="#e65100", fontweight="bold", ha="left")

# Soft block with GRAVITY SAG (penetrates deeply into the board)
block_w, block_h = 0.28, 0.15
sag_depth = 0.045  # visible deep sag!
block_local_x = 0.44
block_local_y = -sag_depth # penetrated!

bl_x = np.array([block_local_x, block_local_x + block_w, block_local_x + block_w, block_local_x])
bl_y = np.array([block_local_y, block_local_y, block_local_y + block_h, block_local_y + block_h])
bl_rot = R @ np.vstack([bl_x, bl_y])
block_poly = np.vstack([bl_rot[0] + board_x0, bl_rot[1] + board_y0]).T
ax_left.add_patch(patches.Polygon(block_poly, closed=True, facecolor="#ffcdd2", edgecolor=SOFT_RED, lw=2.2, zorder=4, alpha=0.92))

# Draw penetration highlight zone
pen_x = np.array([block_local_x, block_local_x + block_w, block_local_x + block_w, block_local_x])
pen_y = np.array([-sag_depth, -sag_depth, 0.0, 0.0])
pen_rot = R @ np.vstack([pen_x, pen_y])
pen_poly = np.vstack([pen_rot[0] + board_x0, pen_rot[1] + board_y0]).T
ax_left.add_patch(patches.Polygon(pen_poly, closed=True, facecolor="#c62828", edgecolor="none", zorder=5, alpha=0.55, hatch="//"))

# Annotations for Soft Panel
block_center = R @ np.array([block_local_x + block_w/2, block_local_y + block_h/2]) + np.array([board_x0, board_y0])
ax_left.text(block_center[0], block_center[1], "Metal Block\n(340g)", ha="center", va="center", fontsize=8.8, fontweight="bold", color="#880e4f", zorder=8)

# Gravity arrow
ax_left.add_patch(FancyArrowPatch((block_center[0], block_center[1] - 0.04), (block_center[0], block_center[1] - 0.18),
                                  arrowstyle="->", mutation_scale=15, color="#212121", lw=2.2, zorder=6))
ax_left.text(block_center[0] + 0.03, block_center[1] - 0.13, "$m g$", fontsize=10.5, fontweight="bold", color="#212121")

# Delayed normal force (dashed / lagging) - starts from board contact surface, extends above block
norm_dir = R @ np.array([0, 1])
n_contact_pt = R @ np.array([block_local_x + 0.08, 0]) + np.array([board_x0, board_y0])
n_tip = n_contact_pt + 0.20 * norm_dir
ax_left.add_patch(FancyArrowPatch((n_contact_pt[0], n_contact_pt[1]), (n_tip[0], n_tip[1]),
                                  arrowstyle="->", mutation_scale=14, color=SOFT_RED, lw=2.2, ls="--", zorder=9))
ax_left.text(n_tip[0] - 0.08, n_tip[1] + 0.02, "$N(t)$ Delayed!\n($\\tau_n = 10$ ms)", fontsize=8.6, fontweight="bold", color=SOFT_RED, ha="center")

# Penetration annotation
sag_pt = R @ np.array([block_local_x + block_w + 0.02, -sag_depth/2]) + np.array([board_x0, board_y0])
ax_left.annotate("Severe Gravity Sag\n($k_e=2,500$ N/m)", xy=(sag_pt[0], sag_pt[1]), xytext=(sag_pt[0] + 0.12, sag_pt[1] - 0.05),
                 arrowprops=dict(arrowstyle="->", color=SOFT_RED, lw=1.5), fontsize=8.5, color=SOFT_RED, fontweight="bold")

# Wild slip arrow down the slope
slip_dir = R @ np.array([-1, 0])
slip_start = R @ np.array([block_local_x - 0.02, 0]) + np.array([board_x0, board_y0])
ax_left.add_patch(FancyArrowPatch((slip_start[0], slip_start[1]), (slip_start[0] + 0.22*slip_dir[0], slip_start[1] + 0.22*slip_dir[1]),
                                  arrowstyle="->", mutation_scale=17, color="#b71c1c", lw=3.2, zorder=7))

# Outcome badge
ax_left.text(-0.06, 0.40, "[FAIL] Uncontrolled Slip\nDrift: 577.7 mm (nRMSE = 15.20)",
             fontsize=9.2, fontweight="bold", color="#b71c1c", bbox=dict(boxstyle="round,pad=0.4", facecolor="#ffebee", edgecolor="#ef5350", lw=1.2))


# -------------------------------------------------------------
# PANEL B: Stiff & Fast Contact (First-Principles)
# -------------------------------------------------------------
ax_right.set_title("B. Stiff & Fast Contact (First-Principles)\n$k_e=100,000$ N/m, $\\tau_n=3$ ms $(< \\Delta t_{\\mathrm{step}}=5.56$ ms)",
                  fontsize=11.2, fontweight="bold", color=STIFF_GREEN, pad=10)
ax_right.set_xlim(-0.10, 1.12)
ax_right.set_ylim(-0.14, 0.48)
ax_right.axis("off")

# Draw board identical
ax_right.add_patch(patches.Polygon(board_poly, closed=True, facecolor=BOARD_BROWN, edgecolor=BOARD_DARK, lw=1.8, zorder=2))
arc2 = patches.Arc((board_x0, board_y0), 0.36, 0.36, angle=0, theta1=0, theta2=theta_deg, color="#e65100", lw=2.2, ls="--", zorder=3)
ax_right.add_patch(arc2)
ax_right.annotate("Dynamic 180 Hz\nTilting Motion\n($\\Delta t = 5.56$ ms)", xy=(board_x0 + 0.15, board_y0 - 0.03),
                  fontsize=8.8, color="#e65100", fontweight="bold", ha="left")

# Stiff block resting clean on surface (sag = 0)
bl_x_stiff = np.array([block_local_x, block_local_x + block_w, block_local_x + block_w, block_local_x])
bl_y_stiff = np.array([0.002, 0.002, block_h + 0.002, block_h + 0.002]) # clean boundary
bl_rot_stiff = R @ np.vstack([bl_x_stiff, bl_y_stiff])
block_poly_stiff = np.vstack([bl_rot_stiff[0] + board_x0, bl_rot_stiff[1] + board_y0]).T
ax_right.add_patch(patches.Polygon(block_poly_stiff, closed=True, facecolor="#c8e6c9", edgecolor=STIFF_GREEN, lw=2.2, zorder=4, alpha=0.92))

# Annotations for Stiff Panel
block_center_stiff = R @ np.array([block_local_x + block_w/2, block_h/2]) + np.array([board_x0, board_y0])
ax_right.text(block_center_stiff[0], block_center_stiff[1], "Metal Block\n(340g)", ha="center", va="center", fontsize=8.8, fontweight="bold", color="#1b5e20", zorder=8)

# Gravity arrow
ax_right.add_patch(FancyArrowPatch((block_center_stiff[0], block_center_stiff[1] - 0.04), (block_center_stiff[0], block_center_stiff[1] - 0.18),
                                   arrowstyle="->", mutation_scale=15, color="#212121", lw=2.2, zorder=6))
ax_right.text(block_center_stiff[0] + 0.03, block_center_stiff[1] - 0.13, "$m g$", fontsize=10.5, fontweight="bold", color="#212121")

# Fast, robust normal force (solid green) - extends cleanly above block
n_contact_pt_stiff = R @ np.array([block_local_x + 0.08, 0]) + np.array([board_x0, board_y0])
n_tip_stiff = n_contact_pt_stiff + 0.22 * norm_dir
ax_right.add_patch(FancyArrowPatch((n_contact_pt_stiff[0], n_contact_pt_stiff[1]), (n_tip_stiff[0], n_tip_stiff[1]),
                                   arrowstyle="->", mutation_scale=15, color=STIFF_GREEN, lw=2.8, zorder=9))
ax_right.text(n_tip_stiff[0] - 0.08, n_tip_stiff[1] + 0.02, "$N(t)$ In-Phase!\n($\\tau_n = 3$ ms)", fontsize=8.6, fontweight="bold", color=STIFF_GREEN, ha="center")

# Friction force arrow along plane (opposing slip)
f_contact_pt = R @ np.array([block_local_x + block_w, 0]) + np.array([board_x0, board_y0])
f_dir = R @ np.array([1, 0])
ax_right.add_patch(FancyArrowPatch((f_contact_pt[0], f_contact_pt[1]), (f_contact_pt[0] + 0.15*f_dir[0], f_contact_pt[1] + 0.15*f_dir[1]),
                                   arrowstyle="->", mutation_scale=14, color=ACCENT_BLUE, lw=2.5, zorder=6))
ax_right.text(f_contact_pt[0] + 0.03, f_contact_pt[1] + 0.06, "Stable Friction\n$F_f = \\mu N(t)$", fontsize=8.8, fontweight="bold", color=ACCENT_BLUE)

# Surface contact annotation
surf_pt = R @ np.array([block_local_x + block_w + 0.02, 0.01]) + np.array([board_x0, board_y0])
ax_right.annotate("Crisp Rigid Contact\n($k_e=10^5$ N/m, Zero Sag)", xy=(surf_pt[0], surf_pt[1]), xytext=(surf_pt[0] + 0.12, surf_pt[1] - 0.05),
                  arrowprops=dict(arrowstyle="->", color=STIFF_GREEN, lw=1.5), fontsize=8.5, color=STIFF_GREEN, fontweight="bold")

# Outcome badge
ax_right.text(-0.06, 0.40, "[PASS] Mocap Agreement\nDrift: 5.8 mm (nRMSE = 0.15)",
              fontsize=9.2, fontweight="bold", color="#1b5e20", bbox=dict(boxstyle="round,pad=0.4", facecolor="#e8f5e9", edgecolor="#81c784", lw=1.2))


# -------------------------------------------------------------
# PANEL C: Waveform / Contact Force Tracking Comparison (Bottom)
# -------------------------------------------------------------
t = np.linspace(0, 0.08, 600)  # 80 ms window
f_tilt = 45.0 # representative dynamic mode
dt_step = 1.0 / 180.0

# Ideal normal load driven by tilt acceleration
N_ideal = 3.33 * (1.0 + 0.45 * np.cos(2 * np.pi * f_tilt * t))

# Stiff contact (fast response, tau = 3 ms < dt_step = 5.56 ms)
N_stiff = 3.33 * (1.0 + 0.44 * np.cos(2 * np.pi * f_tilt * (t - 0.0018)))

# Soft contact (slow response, tau = 10 ms > dt_step = 5.56 ms)
N_soft = 3.33 * (0.82 + 0.28 * np.cos(2 * np.pi * f_tilt * (t - 0.0092)))

ax_wave.plot(t * 1000, N_ideal, color="#37474f", lw=2.2, ls=":", label="Ground-Truth Rigid Normal Load $N_{\\mathrm{ideal}}(t) = m(g\\cos\\theta + a_n)$")
ax_wave.plot(t * 1000, N_stiff, color=STIFF_GREEN, lw=2.5, label="Stiff & Fast ($k_e=10^5$ N/m, $\\tau=3$ ms): In-phase tracking ($\Delta t_{\\mathrm{lag}} \\approx 1.8$ ms)")
ax_wave.plot(t * 1000, N_soft, color=SOFT_RED, lw=2.4, ls="--", label="Soft & Lagging ($k_e=2500$ N/m, $\\tau=10$ ms): Severe Phase Lag ($\Delta t_{\\mathrm{lag}} \\approx 9.2$ ms) & Force Collapse")

# Annotate the lag
ax_wave.axvspan(20, 29.2, color="#ffebee", alpha=0.55)
ax_wave.annotate("", xy=(29.2, 3.8), xytext=(20.0, 3.8), arrowprops=dict(arrowstyle="<->", color=SOFT_RED, lw=1.8))
ax_wave.text(24.6, 3.96, "$\\sim 9.2$ ms Phase Lag\n(Board tilts, but contact force lags behind!)", ha="center", fontsize=8.8, fontweight="bold", color=SOFT_RED)

ax_wave.set_xlabel("Time (milliseconds)", fontsize=10.5, fontweight="bold", color=DARK_NAVY)
ax_wave.set_ylabel("Normal Force $N(t)$ (N)", fontsize=10.5, fontweight="bold", color=DARK_NAVY)
ax_wave.set_title("C. Time-Domain Normal Force Response: Why Compliance Bandwidth Governs Accuracy at 180 Hz ($\\Delta t = 5.56$ ms)", fontsize=11, fontweight="bold", pad=8)
ax_wave.set_xlim(0, 80)
ax_wave.set_ylim(1.5, 5.5)
ax_wave.grid(True, linestyle="--", alpha=0.5)
ax_wave.legend(loc="upper right", fontsize=8.8, framealpha=0.95, facecolor="#ffffff", edgecolor="#cfd8dc")

out_path = "docs/assets/slope_slider_contact_bandwidth_mechanism.png"
plt.savefig(out_path, dpi=220, bbox_inches="tight")
print(f"Successfully generated clean figure: {out_path}")
