# MuJoCo vs SAP Warp — concave collision representation: round socket and dinner plate

## Scope and headline

This report puts the previously separate MuJoCo and SAP Warp experiments next
to each other for two concave-geometry tasks:

1. round peg insertion into the original concave blind socket; and
2. a concave dinner plate with a dropped sphere and a three-plate stack.

It is a **collision-representation benchmark**, not a blanket SAP-versus-
MuJoCo accuracy ranking. Raw concave mesh witness gaps are known to be 
unreliable inside a cavity. Therefore raw and watertight meshes are retained
as failure controls; only the shared 32-piece CoACD representation is eligible
for a mechanically credible insertion flag.

## Read this comparison correctly

The geometry, masses, initial poses, gravity, friction coefficient, time step
(`0.5 ms` for the plate tasks), and the 32 CoACD collision pieces are shared
where possible. **Native contact parameters are intentionally not numerically
matched:** MuJoCo uses `implicitfast`/Newton with `solref=8 ms, 1` and
`solimp=0.95 0.99 0.002`; SAP uses the `drake` preset with shape
`ke=1e5 N/m`, `tau=10 ms` and `rigid_gap=2 mm` for the plate task. Thus a
different equilibrium is a native-model outcome, not automatically an error.

For the round socket, MuJoCo has a five-point `solref` sweep while SAP currently
has one native `ke/tau/gap` point. The table below is consequently a comparison
of **tested configurations**, not an engine-best ranking.

| task / common representation | MuJoCo result | SAP Warp result | defensible reading |
|---|---|---|---|
| round socket, raw mesh/SDF control | 22/25 apparent entries, **0/25 credible** | raw/watertight controls are not valid cavity-contact measurements | an apparent pose inside a concave mesh is not insertion validation |
| round socket, shared 32-piece CoACD, full available matrix | 7/25 entered; **2/25 credible** | 0/10 entered; **0/10 credible** | MuJoCo has two tested credible 2-mm cases; the tested SAP native point does not insert |
| sphere-in-plate, shared CoACD | stable; ball final z/r = 27.238 / 33.524 mm | stable; ball final z/r = 27.444 / 33.667 mm | broad one-plate settling agrees closely under the common approximation |
| three-plate stack, shared CoACD | order preserved, no escape; z = -.030, 10.714, 22.201 mm | order preserved, no escape; z = .064, 11.251, 22.414 mm | both settle, but their contact manifolds/equilibria are not identical |

`credible` here always means: entered cavity, final radial position within the
nominal clearance, no capacity truncation, and peak/final penetration no more
than 10% of the clearance. It is deliberately a strict *post-processing rule*,
not either engine's internal success flag.

## 1. Round peg--socket

### Protocol

- Same nominal circular socket as the MuJoCo peg experiment: socket radius
  12 mm; peg radii 10.00, 10.50, 11.00, 11.50, and 11.75 mm, giving single-side
  clearances 2.00, 1.50, 1.00, 0.50, and 0.25 mm.
- Initial lateral offsets: 0 and 1 mm; initial tilt: 1 degree.
- SAP configuration: `drake`, CUDA, `dt=0.5 ms`, 2 s, `rigid_gap=1 mm`,
  shape `ke=1e4 N/m`, `tau=10 ms`, `mu=0.8`, capacity 2048.
- Raw original mesh and repaired watertight mesh are tested only at 2 mm
  clearance as invalidity controls. Shared 32-piece CoACD is tested over the
  complete clearance x offset matrix.

`entered_cavity` follows the MuJoCo positional criterion, strengthened with
a physically plausible final height: final `0 <= z <= 45 mm` and radial
offset <= 3 mm. A CoACD case is mechanically credible only if it enters,
ends within nominal clearance, has no truncation, and both peak and final
`phi0` penetration are <= 10% of clearance.

### Mesh controls: not interpretable as penetration measurements

| representation | offset (mm) | final radial (mm) | final z (mm) | max phi0 penetration (mm) | entered |
|---|---:|---:|---:|---:|---|
| raw mesh | 0 | 10.71 | 55.27 | .025 | no |
| raw mesh | 1 | 10.37 | 55.27 | .027 | no |
| repaired watertight mesh | 0 | 11.31 | 55.35 | .013 | no |
| repaired watertight mesh | 1 | 10.85 | 55.28 | .025 | no |

The watertight repair does not create a usable insertion contact model. These
small reported `phi0` values only mean that the solver formed little active
contact before the peg stayed above / moved away from the socket mouth; they
do not validate circular inner-wall geometry.

### Shared CoACD matrix

| clearance (mm) | offset (mm) | final radial (mm) | final z (mm) | max raw / phi0 penetration (mm) | entered | credible |
|---:|---:|---:|---:|---:|---|---|
| 2.00 | 0 | 9.25 | 59.84 | .077 / .077 | no | no |
| 2.00 | 1 | 22.56 | 50.58 | .024 / .024 | no | no |
| 1.50 | 0 | 8.50 | 61.37 | .073 / .073 | no | no |
| 1.50 | 1 | 23.12 | 51.09 | .069 / .069 | no | no |
| 1.00 | 0 | 9.58 | 61.41 | .783 / .783 | no | no |
| 1.00 | 1 | 23.63 | 51.59 | .190 / .190 | no | no |
| .50 | 0 | .35 | 65.56 | .017 / .017 | no | no |
| .50 | 1 | 38.56 | 52.07 | .036 / .036 | no | no |
| .25 | 0 | .46 | 65.61 | .018 / .018 | no | no |
| .25 | 1 | 350.42 | -9601.32 | .041 / .041 | no | no |

All 14 runs have `truncated_contacts=0`; capacity is not the cause. CoACD
gives mutually consistent raw witness and `phi0` values, unlike the raw
interior mesh path, but with the specified 1-degree tilted insertion it still
does not enter in any tested clearance. It is held above the mouth or deflects
laterally; the tight 0.25 mm, 1 mm-offset case finally escapes altogether.

### MuJoCo--SAP comparison: what the insertion matrix actually says

The relevant apples-to-apples row is 2 mm clearance, 1 mm lateral offset and
1 degree tilt, using the shared CoACD socket. MuJoCo entered in all five tested
`solref` time constants; the 1 and 2 ms settings were credible. SAP's one
native configuration did not enter: final radial offset was 22.56 mm and final
height was 50.58 mm. The complete available-matrix summary is:

| engine / representation | parameter coverage | entered / cases | credible / cases |
|---|---|---:|---:|
| MuJoCo mesh-SDF | 5 clearances x 5 `solref` time constants | 22 / 25 | 0 / 25 |
| MuJoCo shared CoACD | same 25 cases | 7 / 25 | 2 / 25 |
| SAP raw or watertight mesh controls | c=2 mm, offsets 0 and 1 mm | 0 / 4 | not eligible |
| SAP shared CoACD | 5 clearances x offsets 0/1 mm | 0 / 10 | 0 / 10 |

SAP's shared-CoACD result is therefore stricter here: 0/10 entries and 0/10
credible cases under its native `ke/tau/gap` configuration. The comparison
does not say “SAP can never insert” or “MuJoCo is always superior”; it says the
two currently tested MuJoCo CoACD configurations at 2 mm work under the strict
rule, while the currently tested SAP configuration does not.

This does **not** establish a general engine ranking. It identifies a concrete
hard case: SAP's current CoACD contact representation plus its native contact
settings does not reproduce the tilted gravity insertion that MuJoCo's best
convex setting can reproduce. A future SAP engine-best study would tune
`ke/tau/rigid_gap/dt` on CoACD *without* using raw mesh controls as an
optimization target.

## 2. Concave dinner plate

### Protocol

The original watertight dinner-plate OBJ and the exact same 32 CoACD pieces
used by MuJoCo are each tested in two tasks:

- **sphere-in-plate:** free 40 g, 20 mm-radius sphere dropped onto a free
  301 g plate;
- **three-plate stack:** three free 301 g plates with the same initial offsets
  as the existing MuJoCo plate model.

SAP configuration: `drake`, CUDA, `dt=0.5 ms`, 3 s, `rigid_gap=2 mm`, shape
`ke=1e5 N/m`, `tau=10 ms`, `mu=0.8`, capacity 4096. MuJoCo counterpart:
`implicitfast` + Newton, `dt=0.5 ms`, 3 s, `solref=8 ms 1`,
`solimp=0.95 0.99 0.002`, `mu=0.8`.

Raw mesh is a representation control: its gap/penetration must not be used as
a physical inner-cavity metric. CoACD is the physically interpretable
collision representation in this report.

### Side-by-side settled-state telemetry

The contact-count columns are **not directly comparable**: SAP reports
broad-phase candidate contacts, while MuJoCo reports its active contact points.
Their main value is to reveal representation changes within each engine.

| task | representation | MuJoCo final z (mm) | SAP final z (mm) | MuJoCo final radial (mm) | SAP final radial (mm) | MuJoCo / SAP max contacts | outcome |
|---|---|---|---|---|---|---|---|
| sphere-in-plate | native concave mesh path | plate -.025; ball 26.397 | plate -.004; ball 26.751 | plate .003; ball 19.587 | plate .000; ball 19.561 | 17 / 25 candidates | stable / no escape |
| sphere-in-plate | shared CoACD | plate -.013; ball 27.238 | plate .085; ball 27.444 | plate .005; ball 33.524 | plate .000; ball 33.667 | 9 / 13 candidates | stable / no escape |
| three-plate stack | native concave mesh path | -.055, 21.769, 50.968 | -.010, 8.334, 16.678 | 2.150, 10.841, 16.568 | .001, 3.632, 2.486 | 41 / 148 candidates | order preserved / no escape |
| three-plate stack | shared CoACD | -.030, 10.714, 22.201 | .064, 11.251, 22.414 | .198, 3.263, 1.716 | .039, 6.939, .366 | 111 / 218 candidates | order preserved / no escape |

The most useful shared-CoACD observations are:

1. In **sphere-in-plate**, both engines produce nearly the same ball height
   (27.238 versus 27.444 mm) and radial resting position (33.524 versus
   33.667 mm). This is a positive agreement result for a broad, simple cavity
   settlement task.
2. In the **three-plate stack**, both preserve order and settle. The top plate
   differs by only 0.21 mm in height, while the middle plate differs by 0.54 mm
   and lateral offsets differ more. That is evidence that coupled concave
   multi-contact is representation/model sensitive even when the same CoACD
   pieces are used.
3. Neither the raw SAP witness overlap nor MuJoCo mesh-SDF `contact.dist`
   overlap is a cross-engine penetration ground truth. For example, MuJoCo's
   native mesh-SDF stack reaches a reported 54.1 mm maximum distance overlap
   despite ending ordered and unescaped; that is precisely why this report
   uses final pose, order, escape, jitter, and the CoACD comparison instead.

### SAP-only contact telemetry (kept for reproducibility)

| task | representation | max phi0 / raw witness penetration (mm) | SAP candidates | truncation | SAP tail z jitter p-p (mm) |
|---|---|---:|---:|---:|
| sphere-in-plate | raw mesh control | .018 / .005 | 25 | 0 | < .011 |
| sphere-in-plate | shared CoACD | .025 / .019 | 13 | 0 | < .011 |
| three-plate stack | raw mesh control | .018 / .018 | 148 | 0 | < .005 |
| three-plate stack | shared CoACD | .089 / .089 | 218 | 0 | < .005 |

Tail position jitter is small in every case: below 0.011 mm for the sphere
case and below 0.005 mm per plate for the three-plate stack. Thus these are
settled states, not transient snapshots.

### Interpretation

1. Both representations complete these broad, gravity settling tasks without
   capacity truncation or escape.
2. They do **not** produce the same equilibrium geometry or contact manifold:
   CoACD raises the three-plate stack substantially and produces 218 candidate
   contacts versus 148 for raw mesh. This is expected: a convex decomposition
   replaces the original continuous concave surface by finite convex pieces.
3. Raw mesh's small witness numbers cannot by themselves establish that its
   concavity was simulated correctly. CoACD gives a collision representation
   whose gap has an interpretable primitive-contact meaning, at the price of
   approximation to the original surface.

## 中文解读 / practical reading

### Round peg

这次不是“SAP 只要调个参数就能插入”。我们把 MuJoCo 同样的 1 degree tilt、
clearance 和 offset 搬过来后，SAP 的 CoACD 在全部 10 个正式 case 都没有进入
socket。它不是 contact capacity 不够：全部 `truncated_contacts=0`。主要现象是
peg 在孔口被挡住，或者侧向被弹走。

把 MuJoCo 放在旁边看，结论才完整：同样的 CoACD socket，MuJoCo 的 25 个
`solref` sweep 中有 7 个进入、2 个（都是 2 mm clearance、1/2 ms）满足严格
credibility rule；SAP 当前这一套 native `ke/tau/gap` 的 10 个 case 是 0 个。
所以这是 **当前测试配置下的实际差异**，不是“SAP 理论上一定做不了插孔”，也不是
“MuJoCo 的所有 apparent entry 都真实”——MuJoCo mesh-SDF 就有 22 个 apparent
entry、但 0 个可信。

raw mesh / watertight mesh 看到很小的 `phi0` 并不表示它模拟得好；它只是几乎没有
形成可信的内壁接触。因此 raw/watertight 只作为“这个 representation 不可用于精密
insertion 结论”的 control。

### Concave plate

plate 的 sphere drop 和 three-plate stack 都能 settle，而且没有 SAP contact
overflow。两边共同 CoACD 的 sphere drop 最终 ball height 几乎相同（27.238 vs
27.444 mm），说明这个宽松的单接触任务确实有正向一致性。三盘堆叠也都保序、无逃逸，
但中间盘高度和横向 offset 仍不同；这不是单纯小数值误差，因为 CoACD 本身就把连续凹面
近似为 32 个凸块，而两个 engine 仍会以不同方式处理多个同时接触。

正确结论是：SAP 在较宽松的 concave plate settling task 可以稳定运行；但在紧配合、
带 tilt 的 round insertion 上，当前 CoACD + native parameters 还不能复现 MuJoCo
最佳 convex case。不能把 raw-mesh 的小 gap 当作反驳这个结论的证据。

## Data and reproduction

- Round socket SAP runner: `sap-sim/scripts/run_round_peg_socket_matrix.py`
- Plate SAP runner: `sap-sim/scripts/run_concave_plate_representation_benchmark.py`
- Plate MuJoCo counterpart: `mujoco-sim/learn_mujoco/concave_plate_experiments/run_representation_benchmark.py`
- Round raw outputs: `exp-results/peg-in-hole/sap-round-socket-matrix/`
- Plate raw outputs: `exp-results/concave-plate/sap-representation-benchmark/`
- Plate MuJoCo outputs: `exp-results/concave-plate/mujoco-representation-benchmark-v2/`
- MuJoCo comparison context:
  `mujoco-peg-in-hole-clearance-report.md` and
  `mujoco-sim/learn_mujoco/concave_plate_experiments/`.
