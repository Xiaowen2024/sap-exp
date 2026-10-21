# Layered-box body--body contact: low vs high stiffness

## Purpose

This is the missing physical-object extension of the contact-parameter
semantics discussion.  A ball--plane test has one moving body and one normal
contact.  Here five dynamic boxes transmit increasing load through **four
body--body interfaces plus one body--plane interface**.  The lowest interface
carries `5mg`, then `4mg`, ..., while the top carries `mg`.

The two videos are deliberately split by stiffness rather than hiding the
low-stiffness regime behind the high-stiffness stress case:

- [17 — low stiffness physical replay](../exp-results/site-video-demos/17_layered_box_low_stiffness_body_body.mp4)
- [18 — high stiffness physical replay](../exp-results/site-video-demos/18_layered_box_high_stiffness_body_body.mp4)

MuJoCo panes are fresh native simulations of the recorded model.  SAP/Warp
panes are 3-D offline renders of newly recorded **native CUDA SAP pose traces**;
MuJoCo only rasterizes those poses and does not solve SAP contact dynamics.

## Common task and native settings

| Item | Value |
|---|---|
| Bodies | five `40 x 40 x 40 mm`, 1-kg boxes; x/y/orientation locked, z free |
| Initial condition | 1-mm clearance above ground and between layers; gravity `9.81 m/s^2` |
| Run | 4 s, `dt=0.5 ms`; frictionless (`condim=1`-style normal-only comparison) |
| Low regime | nominal pair stiffness `k=1e4 N/m` |
| High regime | nominal pair stiffness `k=1e6 N/m` |
| MuJoCo | Newton + `implicitfast`, direct `solref`; scalar-calibration seed `s=(1-d)k/m`, `d=0.9`, explicit pair contacts, margin/gap `0` |
| SAP/Warp | CUDA, Drake preset, `rigid_gap=0`; equal shapes use `ke_shape=2*k_pair` and `tau_shape=1.5 ms`, hence actual `k_pair=k` and `tau_pair=3 ms` |

`k` is a **nominal pair-compliance target**, not a claim that the engines'
internal parameter values are numerically identical.  This is the appropriate
extension of the scalar calibration into a coupled body--body manifold.

## Results

The table reports the bottom (interface 1) and top (interface 5) interfaces.
`delta_geom` is geometric overlap from body separation.  SAP also exposes the
contact constraint residual `phi0`; `delta_constraint = max(0,-phi0)` is the
quantity its contact solve follows.  This distinction is the central result.

| Regime / engine | `delta_geom` bottom / top (mm) | SAP `delta_constraint` bottom / top (mm) | Contact support | Tail motion |
|---|---:|---:|---|---|
| Low / MuJoCo | 1.226 / 0.491 | n/a | 4 contact rows at every interface (20 total) | settled in the native run |
| Low / SAP | 1.226 / 0.145 | 1.226 / 0.245 | 4 points at every interface (20 total) | max layer gap std `5.36 um`; max `v_z,RMS=0.00058 mm/s` |
| High / MuJoCo | 0.0123 / 0.00491 | n/a | 4 contact rows at every interface (20 total) | settled in the native run |
| High / SAP | 0.0123 / 0 | 0.0123 / 0.1077 | mean points `[4.0, 1.47, 1.67, 1.24, 0.68]` (9.05 total) | max layer gap std `17.77 um`; max `v_z,RMS=18.81 mm/s` |

The complete per-interface SAP rows are retained in
[`sap-video-traces/results.csv`](../exp-results/layered-stack-stiffness/sap-video-traces/results.csv);
the original coupled-stiffness sweeps remain in
[`layered-stack-stiffness`](../exp-results/layered-stack-stiffness/).

## What the low/high contrast says

1. **Low stiffness is not missing.** At `k_pair=1e4 N/m`, SAP has four contact
   points at every interface, tiny tail motion, and its *constraint* compression
   agrees with the expected `F/(4k)` at all five interfaces.  The physical video
   therefore shows a stable body--body load path, not a single ball--plane toy.

2. **At high stiffness, the difference is representation and dynamics, not a
   simple numerical explosion.** SAP still carries the correct static loads,
   but the upper interfaces have zero geometric overlap while their `phi0`
   constraints retain about `0.1 mm` compression.  The same run has persistent
   tail velocity/jitter and fewer active contact points.  Thus it is misleading
   to score SAP solely by geometric penetration in this regime: a nonzero
   geometric gap does not mean its solver sees zero contact constraint.

3. **MuJoCo stays geometrically supported in this deliberately z-constrained
   case.** Its four normal rows per interface remain active in both regimes.
   That does **not** establish a universal engine ranking: orientations,
   friction, nonconvex geometry, other timesteps or another SAP `tau/gap/preset`
   can change the outcome.  It does establish that the two parameter semantics
   are not interchangeable once multiple redundant contacts share load.

## 中文易读版

这不是只有一个球碰地面，而是 5 个动态 box 一层一层压下去：底部要承受
`5mg`，上面依次是 `4mg ... mg`。因此它是真正的 body--body simultaneous
contact / load transmission case。

- **低刚度 `k=1e4`：** SAP 没有“缺席”。每层 4 个 contact point，全堆叠很稳，
  `v_z` 几乎为 0；它用来求解的 constraint compression 跟 `F/(4k)` 对得上。
- **高刚度 `k=1e6`：** MuJoCo 仍保持每层四个几何接触。SAP 上层则会出现
  `geometric overlap=0`，但 `phi0` 仍约为 `-0.1 mm`，所以 solver 仍在用
  constraint contact 承力；同时 tail jitter/vertical speed 明显变大。

因此 high-stiffness SAP 的现象不能简化成“没有接触”或“penetration=0 所以更硬”。
更准确说法是：**它的几何 overlap、constraint residual 和实际运动已分离；在此
multi-contact high-stiffness setting 下，这伴随明显 persistent jitter。**

两段视频都是真实 3-D box 运动；视频里小位移可能肉眼不明显，所以表格给出
`phi0`、active points 和 tail velocity 作为可审计证据。
