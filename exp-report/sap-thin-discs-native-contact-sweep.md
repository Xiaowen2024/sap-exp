# SAP Warp three-thin-disc native contact sweep

## Scope

This is the SAP-native counterpart to the MuJoCo comprehensive thin-disc
sweep. It does **not** try to rename `ke/tau/rigid_gap` as MuJoCo
`solref/solimp`. Instead it asks: for the same three-disc gravity stack, how
do SAP's own contact variables affect penetration, final geometric bias,
orientation, settling, jitter and failure?

All runs use three free discs (radius 101 mm, total thickness 4 mm, density
2598 kg/m\(^3\)), initialized at z = 30/70/110 mm with small lateral offsets,
then settled under gravity for 2 s. The metrics are:

- `max_penetration`: largest SAP active-contact \(-\phi_0\);
- `final_com_height_error`: absolute error from the zero-gap rigid 6-mm COM
  reference; with nonzero `rigid_gap`, this includes intentional geometric bias;
- maximum disc-normal tilt, tail COM/tilt jitter, settling time, final layer
  order, escape, active contact count and capacity truncation.

The default SAP setup is the `drake` preset, CUDA, \(\mu=.8\),
`rigid_gap=.1 mm`, \(\tau=3\) ms. `solver_iterations=-1` in this Warp API
path means internal solve iterations are not exposed; it is not a one-iteration
solve result.

## Coverage: completed 65-run structured native sweep

| group | variables | runs | purpose |
|---|---|---:|---|
| primary | \(dt=\{.1,.25,.5,1,2,5\}\) ms × \(ke=\{10^3,3\!\times\!10^3,10^4,3\!\times\!10^4,10^5,3\!\times\!10^5,10^6\}\) N/m | 42 | main stability and stiffness–dt map |
| damping | \(ke=10^4,10^5\), \(\tau=.5,1,3,10,30\) ms at dt=.5 ms | 10 | native normal relaxation/damping effect |
| activation gap | `rigid_gap=0,.05,.1,.5 mm` at \(ke=10^5,\tau=3\) ms | 4 | configured gap versus stability/bias |
| preset | `approx32`, `approx64`, `drake` at \(ke=10^4,10^5,10^6\) | 9 | SAP solver-path effect |

This is **not** a full \(dt\times ke\times\tau\times gap\times preset\)
Cartesian search. The first group is a complete \(dt\times ke\) grid; the
remaining groups are controlled one-at-a-time ablations around it.

## 1. Primary \(dt\times ke\) result: a usable stiffness band exists

At \(\tau=3\) ms, `rigid_gap=.1 mm`, Drake preset:

| \(ke\) (N/m) | dt=.1 ms: penetration / COM err | dt=.5 ms: penetration / COM err | dt=1 ms: penetration / COM err | main reading |
|---:|---|---|---|---|
| \(10^3\) | 200.05 / 6.17 mm, fail | 10.80 / 5.31 mm, fail | 10.51 / 5.29 mm, fail | too compliant; order/escape failures |
| \(10^4\) | 2.32 / .815 mm | 2.08 / .716 mm | 2.60 / .678 mm | order preserved, but does not settle within 2 s |
| \(10^5\) | .399 / .007 mm | **.356 / .007 mm** | 1.226 / .007 mm | best tested balance at .1–.5 ms |
| \(10^6\) | **.049 / .091 mm** | .166 / .091 mm | 1.193 / .091 mm | lower penetration, but COM bias rises because of gap-aware equilibrium |

At \(ke=10^5\), .1/.25/.5 ms all settle in roughly .17 s with small tilt
(.038–.040 degrees). At dt=1 ms the peak overlap jumps to 1.226 mm; at 2–5 ms
tail settling degrades. Thus the most balanced tested native point is

\[
\boxed{ke=10^5\ \mathrm{N/m},\quad \tau=3\ \mathrm{ms},\quad
dt=.25\text{–}.5\ \mathrm{ms},\quad rigid\_gap=.05\text{–}.5\ \mathrm{mm},\quad drake.}
\]

This is a task-local operating point, not a universal SAP recommendation.

## 2. `tau`: relaxation/damping changes transients, especially at low \(ke\)

At dt=.5 ms and `rigid_gap=.1 mm`:

| \(ke\) | \(\tau\) (ms) | peak penetration (mm) | COM error (mm) | max tilt (deg) | settling (s) |
|---:|---:|---:|---:|---:|---:|
| \(10^4\) | .5 / 1 / 3 / 10 / 30 | 3.458 / 3.164 / 2.078 / 1.518 / .991 | .692 / .694 / .716 / .728 / .831 | .807 / 1.332 / .201 / .121 / .054 | 2 / 2 / 2 / 2 / .463 |
| \(10^5\) | .5 / 1 / 3 / 10 / 30 | .800 / .697 / .356 / .216 / .170 | .0177 / .00739 / .00739 / .00738 / .00736 | .576 / .224 / .038 / .016 / .0056 | 2 / .236 / .170 / .204 / .204 |

For \(ke=10^5\), increasing \(\tau\) improves peak-overlap and tilt in the
tested range; at \(ke=10^4\), it also leaves a large geometry bias. Do not
read this as a universal “larger tau is better” law: it is the measured
relaxation behavior of this gravity-settling trajectory.

## 3. `rigid_gap`: zero gap is not automatically more accurate

At \(ke=10^5\), \(\tau=3\) ms, dt=.5 ms:

| rigid gap (mm) | peak penetration (mm) | COM error from **zero-gap** reference (mm) | settle (s) | reading |
|---:|---:|---:|---:|---|
| 0 | .887 | .0597 | 2.0 | more transient overlap and residual motion |
| .05 | .356 | .00739 | .170 | stable |
| .10 | .356 | .00739 | .170 | stable |
| .50 | .332 | .00739 | .170 | stable in this task |

This result is consistent with the separate high-stiffness box-stack
diagnostic: `rigid_gap` affects contact activation and equilibrium geometry.
A zero-gap rigid COM reference cannot be used naively as an error metric when
the model deliberately maintains a gap skin. Here the .05–.5-mm settings have
similar final COM, but that does not imply arbitrary large gaps are safe in
tight-clearance insertion tasks.

## 4. Preset ablation

At dt=.5 ms, \(\tau=3\) ms, `rigid_gap=.1 mm`:

| \(ke\) | preset | peak penetration (mm) | COM error (mm) | tilt (deg) | settle (s) |
|---:|---|---:|---:|---:|---:|
| \(10^4\) | approx32 / approx64 / drake | 2.177 / 2.177 / 2.078 | .677 / .678 / .716 | .202 / .202 / .201 | 2 / 2 / 2 |
| \(10^5\) | approx32 / approx64 / drake | .969 / .969 / **.356** | .00739 / .00739 / .00739 | .0509 / .0509 / **.0379** | .196 / .196 / **.170** |
| \(10^6\) | approx32 / approx64 / drake | .841 / .841 / **.166** | .0907 / .0907 / .0907 | .0141 / .0141 / **.00527** | .182 / .182 / **.1665** |

`drake` is the best tested preset in the useful \(10^5\)–\(10^6\) stiffness
range. `approx32` and `approx64` are nearly identical physically here;
`approx32` had one 101.7-s wall-time outlier at \(ke=10^4\), while most
successful .5-ms runs were roughly 20–26 GPU s for the full 2-s trajectory.

## Relationship to the MuJoCo thin-disc sweep

The task geometry and high-level metrics are shared, but native parameters are
not one-to-one: MuJoCo sweeps `solref`, `dampratio`, `solimp`, integrator and
solver; SAP sweeps \(ke,\tau,rigid\_gap\) and preset. The fair conclusion is
about **within-engine native operating regions**, not identical numeric
stiffness labels.

- MuJoCo's tested balanced point was dt=.25 ms, `solref=1 ms`: .368-mm peak
  overlap, .00080-mm COM error, .178-s settling.
- SAP's tested balanced point is \(ke=10^5\), \(\tau=3\) ms, dt=.25–.5 ms:
  .363/.356-mm peak \(-\phi_0\), .00739-mm zero-gap-COM error, .169/.170-s
  settling.
- These similar aggregate numbers are useful sanity checks, but they are not
  a formal accuracy ranking: SAP's \(-\phi_0\) and MuJoCo `-contact.dist`
  come from different native contact formulations, and SAP uses a nonzero
  `rigid_gap`.

## 中文总结

SAP native sweep 已补齐，不是照抄 MuJoCo 的 `solref/solimp`。主要发现是：

- \(ke=10^3\) 太软，会错层或 escape；\(ke=10^4\) 可保持顺序但 2 s 内没有完全
  settle；\(ke=10^5\) 在 dt=.25–.5 ms 是本任务最平衡的 tested regime。
- \(ke=10^6\) 的 peak penetration 更小，但若把 `rigid_gap=.1 mm` 的有意几何间隙
  当作 zero-gap rigid error，COM 会看起来更差；不能据此说高刚度“更软”。
- `tau` 与 preset 都有实质影响；在 \(ke=10^5\)–\(10^6\) 下 Drake 明显比
  approx32/64 的 peak penetration 更低。
- 当前是完整主网格 + 单变量 ablation，不是五个变量的全排列搜索。

## Reproduction

- SAP runner: `sap-sim/scripts/run_thin_disc_stack_sap_sweep.py`
- SAP data: `exp-results/thin-discs/sap-comprehensive-sweep/results_primary.csv`, `results_tau.csv`, `results_gap.csv`, `results_preset.csv`
- MuJoCo counterpart: `mujoco-thin-discs-comprehensive-results.md` and `mujoco-thin-discs-comprehensive-sweep.md`
