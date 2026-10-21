# Thin-shape stack: shallow concave plate vs thin disc

## Question

The earlier dinner-plate stack had a 44.46-mm rim-to-bowl-bottom envelope, so
it was not a thin-body test.  This controlled follow-up asks whether making
the plate thin removes the collision difficulty relative to a true thin disc.

It is a **shape/representation-complexity comparison inside each engine**. It
is not a direct wall-clock SAP-versus-MuJoCo race: SAP used CUDA and MuJoCo
used CPU, and their native contact parameters are intentionally different.

## Controlled geometry and protocol

Three free bodies settle under gravity for 2 s from the same centres:
`(0,0,30)`, `(3,-2,70)`, `(-2,2,110)` mm.  All have:

- diameter 202 mm;
- 4-mm vertical envelope;
- 333 g measured body mass per object;
- friction coefficient 0.8; and
- the same small initial x/y offsets.

`thin_disc` is an analytic cylinder (101-mm radius, 4-mm total thickness).
`shallow_concave_plate` uses the same watertight dinner-plate mesh as the old
experiment, but its bowl-depth axis is non-uniformly scaled from 44.46 mm to
4 mm.  Its collision geometry is the shared 32-piece CoACD decomposition.
The original watertight mesh is collision-disabled and supplies only mass and
inertia in SAP; the CoACD pieces supply collision only.  This avoids the
invalid zero-mass CoACD-only SAP model discovered in the initial trial.

Native settings:

| Engine | dt | Native contact configuration |
|---|---:|---|
| MuJoCo 3.12.0 | 0.25 ms | Newton, `implicitfast`, elliptic cone, `solref=1 ms`, damping ratio 1, `solimp=.95 .99 .002 .5 2` |
| SAP Warp / RTX 3090 | 0.5 ms | `drake` preset, shape `ke=1e5 N/m`, `tau=3 ms`, `rigid_gap=.1 mm` |

The reported SAP `-phi0` overlap and MuJoCo `-contact.dist` overlap are each
their engine's native geometric/contact residual.  They are useful for the
within-engine shape comparison, but are not numerically identical definitions.

## Results

| Engine | shape | order / escape / truncation | max contacts | peak native overlap (mm) | tail COM jitter (mm p-p) | settled by 2 s? | stepping wall time (s) |
|---|---|---|---:|---:|---:|---|---:|
| MuJoCo | thin disc | yes / no / n/a | 13 | 0.366 | 0.000062 | yes, 0.274 s | 0.328 |
| MuJoCo | shallow concave plate | yes / no / n/a | 121 | 0.815 | 0.00357 | no | 2.238 |
| SAP Warp | thin disc | yes / no / **0** | 12 | 0.356 | 0.000000 | yes, 0.170 s | 28.499 |
| SAP Warp | shallow concave plate | yes / no / **0** | 147 | 0.981 | 0.00104 | no | 185.495 |

The disc settles near the expected flat-stack centres (approximately 2, 6,
10 mm): MuJoCo gives 1.999/5.999/9.999 mm and SAP gives
1.935/6.002/10.086 mm.  The shallow plates instead nest in their concave
profiles, so their centre heights (MuJoCo: -0.000/1.024/2.150 mm; SAP:
0.075/1.194/2.306 mm) should **not** be compared with the flat-disc 2/6/10-mm
reference.

## Reading the result

Within both engines, the true thin disc is clearly the cleaner and more
efficient contact task:

- CoACD shallow plates create roughly 9--12 times as many contacts.
- Their peak native overlap is about 2.2--2.8 times larger.
- Neither engine reaches the chosen velocity settling threshold by 2 s,
  whereas both thin-disc cases do.
- The shape change costs about 6.5--6.8 times more stepping time within each
  engine.

This does **not** show that SAP is worse than MuJoCo.  Both show the same
qualitative shape sensitivity, both preserve layer order, neither plate case
escapes, and SAP has zero contact-capacity truncation.  The defensible result
is narrower: shrinking the bowl vertically makes the plate thin, but does not
make it an easy thin-disc problem.  The dominant remaining variable is the
many-piece concave collision manifold, not thickness alone.

## 中文说明

这次把老盘子的 44.46 mm bowl depth 压到 4 mm，并且和 4-mm cylinder thin
disc 对齐直径、质量、初始姿态。结果两边都一样：**thin disc 明显更容易**。

- MuJoCo：contact 从 13 增至 121；SAP：从 12 增至 147；
- 两边的 shallow plate 都没有逃逸、也保持上下顺序，SAP 没有 overflow；
- 但两边都没有在 2 s 内完全 settle，thin disc 则都很快 settle；
- 所以问题不只是“盘子以前太厚”，而是 shallow bowl 的 CoACD 碰撞面仍然会
  同时产生很多 contact candidate，并让三个盘相互嵌套。

不要把 MuJoCo 的 0.815 mm 和 SAP 的 0.981 mm 直接作为严格数值排名；两者的
overlap 定义不同。真正可靠的共同结论是：对于 stack / settling 类任务，若不需要
凹碗形功能，analytic thin disc 是更稳定、更省 contact budget、也更快的建模选择。

## Reproduction and raw data

- MuJoCo runner: [run_thin_shape_stack_comparison.py](../mujoco-sim/learn_mujoco/concave_plate_experiments/run_thin_shape_stack_comparison.py)
- SAP runner: [run_thin_shape_stack_comparison.py](../sap-sim/scripts/run_thin_shape_stack_comparison.py)
- MuJoCo CSV: [results.csv](../exp-results/thin-shape-comparison/mujoco/results.csv)
- SAP CSV: [results.csv](../exp-results/thin-shape-comparison/sap/results.csv)
