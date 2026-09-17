# Simple pinch: SAP Warp vs MuJoCo

## Canonical conclusion — read this report first

This document combines the earlier compression-matched v2 study and the
measured-force-matched v3 study. It is the **single source of truth** for the
simple-pinch benchmark.

**Final answer.** Both engines reproduce the expected static Coulomb grasp
threshold qualitatively when compared at the same *measured* normal preload.
For the particular native configurations tested here, SAP changes from failure
to pass closer to the ideal margin `r = 1` than MuJoCo. This is a task-level
result, **not** a universal ranking of the two engines.

The crucial methodological lesson is simple:

```text
Do not compare the same geometric pad compression.
Compare the same measured normal force N, or equivalently the same
physical grasp margin r = 2 mu N / (m g).
```

## 1. What is the task?

A 40 mm cube is pinched by two opposed pads. Gravity is disabled; during the
commanded 60 mm vertical lift the cube alone receives a known downward load
`m g`. Each pad supplies normal force `N`; the friction coefficient is
`mu = 0.4`.

For an ideal symmetric Coulomb pinch,

```math
2 mu N >= m g.
```

We therefore define the dimensionless physical grasp margin

```math
r = 2 mu N / (m g).
```

- `r < 1`: an ideal static Coulomb grasp cannot support the load.
- `r = 1`: theoretical threshold.
- `r > 1`: static support is physically possible; finite-time slip can still
  make a simulation fail its task gate.

All v3 cases use `dt = 0.5 ms`, a 1.5 s preload gate, a 1.0 s / 60 mm lift,
and a 0.25 s post-lift hold. A run passes only if it has stable preload on
both sides and cube lift-following of at least 57 mm (95% of command).

## 2. Why v2 is not a fair engine comparison

The original v2 experiment used the **same pad compression** in both engines.
That seems natural, but it does not create the same physical grasp: contact
models map compression to force differently.

| Same geometric compression | MuJoCo measured N / side | SAP measured N / side | Consequence |
|---:|---:|---:|---|
| 0.05 mm | about 0.95 N | 6.00 N | SAP has about 6x more available friction support |
| 0.20 mm | about 3.82–3.91 N | 12.00 N | SAP has about 3x more available friction support |

Thus a v2 result such as “SAP passes while MuJoCo slips at 0.05 mm” does **not**
mean SAP has better friction. It mostly says that the two simulators created
different normal forces from the same geometric overlap.

V2 remains useful for two things:

1. it revealed the confound and motivated force matching; and
2. it verified that a normal-contact validity gate and contact-capacity checks
   are necessary before interpreting grasp forces.

It must not be used for a cross-engine success-rate claim.

## 3. v3: the fairer experiment

V3 holds `mu`, mass, trajectory, lift load, and task gate fixed, while allowing
each engine to choose a **different geometric compression** to obtain the same
actual preload level.

| nominal target | MuJoCo compression / side | MuJoCo actual N / side | SAP compression / side | SAP actual N / side |
|---|---:|---:|---:|---:|
| N4 | 0.20 mm | 3.88–3.96 N | 0.001 mm | 4.040 N |
| N6 | 0.30 mm | 5.87–5.95 N | 0.050 mm | 6.000 N |

Every final v3 case passed the initial normal-contact gate:

- four force-bearing contacts per pad–cube interface;
- 100% duty during the gate;
- essentially zero normal-force coefficient of variation;
- no SAP contact-capacity truncation (`max_truncated_contacts = 0`).

So an ensuing slip cannot be blamed on a missing or chattering initial normal
contact.

## 4. Main result: transition around the Coulomb threshold

The table shows the bracketing cases: the highest failed margin and lowest
passed margin. The interval, rather than a single sample, is the honest
resolution of this discrete sweep.

| preload regime | engine | highest failed r | lowest passed r | reading |
|---|---|---:|---:|---|
| N4 | MuJoCo | 1.031 | 1.176 | needs measurable extra margin under this gate |
| N4 | SAP Warp | 0.959 | 1.010 | transition is close to ideal `r=1` |
| N6 | MuJoCo | 1.036 | 1.182 | same qualitative pattern |
| N6 | SAP Warp | 1.000 | 1.050 | `r=1` misses the 57 mm gate by only 0.08 mm |

Both engines have the expected broad behavior:

```text
r clearly below 1  -> slip / loss of support
r sufficiently above 1 -> stable lift
```

The difference is in the near-threshold band. In these configurations SAP
passes at a smaller excess load margin. Near `r=1`, both engines generally
reach `rho = |Ft| / (mu Fn)` close to one and accumulate relative tangential
motion; that accumulated slip is what causes a 95%-follow failure.

### Representative v3 boundary data

| regime | engine | r | lift-following (mm) | slip (mm) | result |
|---|---|---:|---:|---:|---|
| N4 | MuJoCo | 1.031 | 56.38 | 3.54 | fail |
| N4 | MuJoCo | 1.176 | 59.00 | 0.93 | pass |
| N4 | SAP Warp | 0.959 | 54.64 | 4.30 | fail |
| N4 | SAP Warp | 1.010 | 57.96 | 0.99 | pass |
| N6 | MuJoCo | 1.036 | 56.27 | 3.61 | fail |
| N6 | MuJoCo | 1.182 | 58.96 | 0.94 | pass |
| N6 | SAP Warp | 1.000 | 56.92 | 1.99 | fail by 0.08 mm |
| N6 | SAP Warp | 1.050 | 58.50 | 0.41 | pass |

## 5. The SAP 0.1 mm observation

SAP records both the contact constraint gap `phi0` and an independently
measured geometric signed overlap. Across every stable v3 preload case,

```math
(-phi0) - delta_geometric = 0.100000 mm (+/- 0.000002 mm).
```

This is not random noise and not evidence that the cube physically penetrates
an unexplained 0.1 mm. It says that SAP's constraint-space contact coordinate
`phi0` contains a systematic approximately 0.1 mm offset relative to the raw
geometric overlap used here. The same effect appeared independently in the
layered-box stack; it must therefore be reported whenever using `phi0` as a
penetration-like metric.

It also explains why an N2 force-matched curve is absent for the current SAP
setup (`drake`, `k_pair=1e4 N/m`, `tau_pair=3 ms`, `rigid_gap=0`):

| SAP compression / side | measured N / side | gate state |
|---:|---:|---|
| <= -0.01 mm | 0 | no contact |
| 0.000 mm | 1.434 N | unstable: 50% duty, `CV(Fn)=1.07` |
| +0.001 mm | 4.040 N | stable four-point contact |

There is no stable 2 N plateau in this fixed native configuration. That is a
property of the tested configuration, **not** proof that SAP can never model a
2 N preload. Finding one would require a separate SAP `rigid_gap` / `ke` /
contact-model study, not a silent parameter change.

## 6. Configuration scope

| engine | native setup used in final v3 curve |
|---|---|
| MuJoCo | Newton, elliptic cone, `implicitfast`; pair `solref=(4 ms, 1)`, `solimp=(.95,.99,.001,.5,2)` |
| SAP Warp | `drake` preset; `k_pair=1e4 N/m`, `tau_pair=3 ms`, `rigid_gap=0`; CUDA RTX 3090 |

These are native contact models, not a parameter-by-parameter physical
equivalence. The comparison is fairer because it matches observable preload
and load margin, but it is still a comparison of these two chosen
configurations on this task.

## 7. What can and cannot be concluded

### Supported by the data

- Normal force, not geometric compression, is the appropriate cross-engine
  calibration target for this grasp.
- Both engines recover the qualitative Coulomb threshold.
- SAP's tested configuration has a narrower transition interval nearer `r=1`
  than the tested MuJoCo configuration under the same 95%-follow criterion.
- SAP's stable contact coordinate has a repeatable 0.1 mm `phi0`–geometry
  offset in this task.

### Not supported by the data

- “SAP is universally more accurate than MuJoCo.”
- “MuJoCo has the wrong friction coefficient.”
- A comparison at identical compression values.
- A claim about long-term static creep: the hold is only 0.25 s. A 5–10 s
  sub-threshold force-hold is the proper next test for that question.

## 中文解读 / plain-language version

### 这组实验到底在比什么？

不是在比「手指压进去同样 0.05 mm，谁抓得住」。因为同样压进去 0.05 mm：
MuJoCo 只产生约 0.95 N/side，而 SAP 产生约 6 N/side；这根本不是同一个
物理抓取力。

真正比较的是：两边都先**实测**每侧法向预载 `N`，再看摩擦能否提供

```text
2 * mu * N >= m * g
```

的支撑力。`r = 2 mu N / (m g)` 就是“现在的摩擦承载能力除以需要承受的
重量”。

- `r = 0.8`：摩擦上限只有所需载荷的 80%，理应掉。
- `r = 1.0`：刚好在理想边界；真实数值仿真里只要有一点动态 slip，就可能
  跟不上 60 mm lift。
- `r = 1.2`：有 20% margin，通常应稳定。

### 最终结果怎么读？

两边都符合大方向：`r < 1` 会掉，`r` 足够大就抓住。区别只出现在非常靠近
边界的位置：SAP 在 N4 时大约 `r=1.01` 已过 95% lift gate；本次 MuJoCo
在 N4 需要到约 `r=1.18` 才过。N6 的趋势一样。

这句话的正确表述是：

> 在这个 simple-pinch、这些 native parameter、这个 1 s lift 与 95% gate 下，
> SAP 的失抓边界更接近理想 Coulomb 边界。

不是：

> SAP 在所有机器人抓取中都比 MuJoCo 准。

因为 controller、接触离散化、solver regularization、dt、接触模型都可能在
换任务后改变结果。

### SAP 的 0.1 mm 是什么意思？

不是说物体真的“多穿进去了 0.1 mm”。`phi0` 是 SAP solver 内部用的
constraint gap；它不是直接等于几何表面的 raw overlap。这里发现：

```text
SAP internal (-phi0) = measured geometric overlap + 0.1 mm.
```

所以未来读 SAP penetration 时，必须先说清楚用的是 `phi0` 还是 raw geometry。
两个数字不能直接混为一谈。

### 为什么没有 N=2 N/side 的公平曲线？

在当前 SAP 参数下，接触从“不稳定/没接触”直接跳到约 4.04 N 的稳定四点接触。
因此强行把它叫作 2 N 会是假数据。若想研究 2 N，需要单独调 SAP 的
`rigid_gap` 或 `ke`，找到一个稳定 contact regime 后再重新 matched-force。

## Reproducibility and data

- MuJoCo runner:
  `mujoco-sim/learn_mujoco/pinch_grasp_experiments/run_simple_pinch_v2.py`
- SAP runner: `sap-sim/scripts/run_simple_pinch_v2_sap.py`
- Final raw curves:
  `exp-results/simple-pinch-v3/{mujoco,sap}/matched-N4/` and
  `exp-results/simple-pinch-v3/{mujoco,sap}/matched-N6/`
- Historical v2 raw data: `exp-results/simple-pinch-v2/{mujoco,sap}/`

The full 28-row v3 machine-readable table, calibration traces, and preliminary
MuJoCo analytic-compression trials remain in the two archival reports below.

## Archive documents

- `simple-pinch-v2-sap-vs-mujoco.md`: historical compression-matched study.
- `simple-pinch-v3-force-matched-sap-vs-mujoco.md`: full v3 calibration and
  every per-case row.

Those files are retained for provenance; this document is the recommended
entry point and final interpretation.
