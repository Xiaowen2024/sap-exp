# Peg-in-hole — consolidated MuJoCo vs SAP Warp results

> One-page reading guide for every peg-in-hole conclusion collected so far.
> Detailed per-parameter tables remain in the linked source reports; this file
> is the authoritative map of what was tested, what passed, and what cannot be
> concluded.

## Bottom line

There are **two different tests**, and their outcomes should not be collapsed
into one statement such as “SAP succeeds/fails at peg insertion”.

| test | contact geometry | SAP conclusion | MuJoCo conclusion | what it establishes |
|---|---|---|---|---|
| **analytic square cavity** | four box walls + a bottom; known-valid primitive contacts | centered peg is stable for all six tested `ke/tau` points; a 1-mm offset is also stable once `rigid_gap` is reduced from 1 mm to 0.1–0.5 mm | centered and 1-mm offset both settle for all four tested `solref` time constants | SAP can handle a clean multi-contact insertion topology; `rigid_gap` is an important activation parameter |
| **round concave blind socket** | raw mesh/SDF or shared 32-piece CoACD | raw/watertight mesh has invalid internal witness behavior; shared CoACD did not insert in any of the current 10-case clearance × offset matrix | mesh-SDF gives many apparent entries but none pass the task screening; shared CoACD gives 2 credible cases of 25 | collision representation, not only contact solver parameters, is the limiting issue |

So the defensible current claim is:

> In a clean analytic cavity, both engines can complete the insertion. In the
> tilted, gravity-driven *round concave socket*, MuJoCo has two tested credible
> CoACD configurations at 2-mm clearance, whereas the currently tested SAP
> CoACD native configuration has none. Raw concave mesh/SDF poses must not be
> treated as proof of physical insertion in either engine.

This is evidence about the **tested configurations**, not an engine-best or
universal ranking.

## 1. What was actually tested?

### Coverage map

| engine | geometry / representation | centered, 0-mm offset | 1-mm offset | clearance sweep | parameter sweep | usable for precise penetration? |
|---|---|---|---|---|---|---|
| MuJoCo | analytic square primitives | yes | yes | fixed 2 mm | `solref` 1/2/4/10 ms | yes: `contact.dist` is primitive-pair gap |
| SAP Warp | analytic square primitives | yes | yes | fixed 2 mm | `ke`, `tau`, then `rigid_gap` | yes: raw witness = `phi0` in this primitive case |
| MuJoCo | round socket mesh-SDF | offset sweep includes 0 and 1 mm | yes | 2 / 1.5 / 1 / .5 / .25 mm | `solref` 1/2/4/8/16 ms plus `solimp` studies | no: apparent entry is not a validity result |
| MuJoCo | round socket shared 32-piece CoACD | offset sweep includes 0 mm | yes | same five clearances | same `solref` sweep; detailed 2-mm `solimp` sweeps | conditionally: primitive piece contacts, with task screening |
| SAP Warp | round raw and repaired-watertight mesh | yes, diagnostic control | yes, diagnostic control | 2 mm control | fixed native point | **no**: cavity witness is invalid |
| SAP Warp | round shared 32-piece CoACD | yes | yes | same five clearances | one current native point: `ke=1e4`, `tau=10 ms`, `rigid_gap=1 mm` | conditionally: primitive piece contacts, with task screening |

**Answer to the question in Chinese:** SAP both **did** do the no-offset
centered tests and **did** do the square task. The square task was deliberately
introduced as an analytic control; it is not a replacement for the circular
socket. SAP also did a centered round-socket test, but its raw/watertight mesh
result was invalid and its centered CoACD result did not enter.

## 2. Shared task rule used for the round socket

Round socket nominal inner radius is 12 mm. Peg radii 10.00, 10.50, 11.00,
11.50 and 11.75 mm give one-sided clearance
\(c=2.00,1.50,1.00,.50,.25\) mm. The main matrix has a 1-degree initial tilt
and lateral offsets 0 or 1 mm.

`entered` means a physically plausible final pose:

\[
0 \le z_{\rm final}\le45\ \mathrm{mm},\qquad r_{\rm final}\le3\ \mathrm{mm}.
\]

The stricter project screening flag is

\[
\text{entered}\ \land\ r_{\rm final}\le c\ \land\ \text{no truncation}
\ \land\ p_{\max},p_{\rm final}\le0.1c.
\]

This is a conservative **project screening rule**, not an official MuJoCo/SAP
threshold or a universal mechanical-design standard. It prevents a deep
numerical overlap from being labelled an insertion success. Always read its
yes/no flag together with the continuous penetration and final-pose values.

## 3. Analytic square cavity: the clean solver control

### Same physical task

A 20 × 20 × 50 mm box peg falls for 2 s into a square cavity with half-width
12 mm: 2-mm one-sided clearance. It starts either centered or with a 1-mm
lateral offset. The walls and bottom are analytic boxes, so the normal-gap
measurement is interpretable. Both implementations use `dt=0.5 ms`, gravity,
and \(\mu=0.8\).

### Centered case — both engines work

| engine | configuration sweep | final state | peak contact overlap range |
|---|---|---|---:|
| MuJoCo | `solref` timeconst = 1, 2, 4, 10 ms | all four enter and rest at z ≈ 35 mm with four contacts | .141, .580, 1.497, 4.242 mm |
| SAP Warp | `ke=1e3,1e4,1e5` at `tau=10 ms`; `tau=1,3,30 ms` at `ke=1e4` | all six enter and rest at z ≈ 35.09 mm with four contacts | .00162–.09803 mm |

SAP's centered primitive result has a useful internal validation: reconstructed
raw witness separation and `phi0` agree to below \(10^{-7}\) mm. In this
specific analytic box case, SAP `phi0` is therefore not merely a `rigid_gap`
or margin-shifted proxy.

The different overlap magnitudes are **not** a global accuracy score: the
native parameters are not dynamically calibrated to the same compliance.
Within each engine, however, softness matters strongly. MuJoCo peak overlap
grows from .141 to 4.242 mm as `timeconst` grows from 1 to 10 ms; SAP falls
from .0980 to .00162 mm as the tested `ke` rises from \(10^3\) to \(10^5\)
N/m at fixed `tau`.

### 1-mm offset — the SAP `rigid_gap` lesson

| engine / setting | entered? | final state / reading |
|---|---|---|
| MuJoCo, `solref` = 1/2/4/10 ms | all yes | retains the intended 1-mm offset and rests on the bottom |
| SAP, initial `rigid_gap=1 mm`, all six `ke/tau` sweep points | all no | lateral escape then fall-through; low final penetration only means loss of contact |
| SAP, `ke=1e4`, `tau=10 ms`, `rigid_gap=0 / .1 / .5 mm` | all yes | stable; peak overlap .293 / .293 / .00973 mm |
| SAP, same but `rigid_gap=1 mm` | no | escapes/falls through |

The remaining geometric side clearance at 1-mm offset is itself 1 mm. Thus a
1-mm activation gap admits early side-wall candidates at the boundary of that
clearance and changes the trajectory. This does **not** show that SAP cannot
model offset insertion: it shows that `rigid_gap` must be selected smaller than
the intended clearance (the tested 0.1–0.5-mm range works here).

## 4. Round concave socket: representation stress test

### MuJoCo matrix: apparent entry is not enough

The 1-mm-offset, 1-degree-tilt, 5-clearance × 5-`solref` matrix yields:

| collision representation | entered / 25 | passes project screening / 25 | strongest conclusion |
|---|---:|---:|---|
| mesh-SDF | 22 | 0 | it can create virtual tolerance / apparent entries, but none are trustworthy under this screen |
| shared 32-piece CoACD | 7 | 2 | the only screening passes are 2-mm clearance with `solref` timeconst 1 and 2 ms |

At 2-mm clearance with CoACD, MuJoCo's detailed parameter sweep found a
non-monotonic optimum region rather than “harder is always better”:

| parameter study | best measured peak penetration in tested 2-mm condition | interpretation |
|---|---:|---|
| `solref` timeconst | .1364 mm at 2 ms | 1–2 ms passes the project's .2-mm screen; 4 ms and above do not |
| `solimp d0` | .10845 mm at `.70` | too low *or* too high impedance worsens this coupled transient |
| `solimp dwidth` | .09817 mm at `.995` | curve shape is weaker than `timeconst`, but not irrelevant |
| `dampratio` | .08516 mm at `.25` | all tested points pass in this slow drop; this is not a high-speed damping ranking |

The combined locally-good settings did **not** generalize to smaller clearances.
For example, the tuned candidate still had .2779, .5270, .5432 and .5300 mm
peak overlap at \(c=1.5,1,.5,.25\) mm, and none of those cases passed the
project screen.

### SAP round socket: centered and offset cases both exist

There are two SAP round-socket diagnostic generations. They must be kept
separate because they use different runners/initialization details; their raw
mesh outcomes should not be averaged or counted as independent successes.

| SAP round diagnostic | centered, 0-mm offset | 1-mm offset | valid conclusion |
|---|---|---|---|
| earlier representation diagnostic, c=2 mm | raw and watertight mesh visually/positionally enter, but report ~11.98-mm `phi0` inside an empty cavity; CoACD does not enter | raw/watertight eject; CoACD ejects | raw/watertight interior witness is invalid; CoACD did not solve insertion |
| later full matrix, c=2/1.5/1/.5/.25 mm, 1° tilt | CoACD: 0/5 enter | CoACD: 0/5 enter; .25-mm case escapes | with the current native `ke=1e4`, `tau=10 ms`, `rigid_gap=1 mm`, CoACD has 0/10 entered, 0/10 screened passes |

The later matrix also tried raw and watertight mesh controls at 2-mm clearance
for offsets 0 and 1 mm; all stayed above/moved away from the mouth. This does
not contradict the old ~12-mm-centered witness: it reinforces that raw interior
mesh results depend on an invalid witness/contact path and cannot serve as a
physical insertion measurement.

All later SAP matrix cases had `truncated_contacts=0`; the CoACD failure is not
a 2048-contact-capacity failure. It is a current collision representation /
contact-activation / native-dynamics behavior that needs a dedicated
`ke × tau × rigid_gap × dt` search if we want an SAP-best round-socket result.

## 5. Cross-engine conclusions that are safe today

1. **Do not evaluate peg insertion from final depth alone.** MuJoCo mesh-SDF
   has 22 apparent entries but zero screened passes; SAP raw mesh can report a
   nearly cavity-sized overlap while the centered peg is visibly inside.
2. **A valid primitive control matters.** In the analytic square cavity SAP is
   stable when centered, and can be stable when offset after tuning
   `rigid_gap`; this isolates the round task as more than a generic SAP solver
   failure.
3. **For the tested tilted round socket, MuJoCo currently has the better
   demonstrated CoACD outcome.** Its 2-mm, 1–2-ms cases pass the project's
   strict screen; SAP's one tested native CoACD configuration does not enter at
   any clearance/offset point.
4. **This is not yet “engine-best versus engine-best.”** MuJoCo received a
   broad `solref/solimp` search. SAP's round CoACD received only a single
   `ke/tau/gap` material point in the formal matrix. SAP must receive an equal
   tuning budget before a stronger engine ranking.
5. **Raw mesh vs CoACD is a modeling choice, not a mere solver toggle.** It
   changes where contacts are permitted. Tune collision representation first;
   then tune `solref/solimp` or `ke/tau/rigid_gap`.

## 中文速读

- SAP 不止做了 offset；**centered/no-offset、1-mm offset、square、round** 都做过。
- square 是干净的 analytic control：SAP centered 六组都能进去；offset 失败不是 `ke`
  或 `tau` 怎么调都不行，而是最初 `rigid_gap=1 mm` 刚好等于剩余 side clearance。
  改成 `.1–.5 mm` 后 offset case 稳定。
- round socket 才是难点：SAP raw/watertight 的 centered “进去”不能算成功，因为空腔
  内仍报约 12 mm 的假 witness overlap；共享 CoACD 则 centered/offset 全部没有插进去。
- MuJoCo round mesh-SDF 的很多 “进去”同样不能信；共享 CoACD 只有 2-mm clearance
  的 1/2-ms `solref` 两组通过项目筛选。
- 所以最准确的当前说法是：**square control 两边均可用；round concave insertion
  在目前调参覆盖下 MuJoCo 有两组可信 CoACD 结果，SAP 尚没有。**

## Source reports and raw data

- [MuJoCo round clearance matrix](mujoco-peg-in-hole-clearance-report.md)
- [MuJoCo round contact-parameter sweeps](mujoco-peg-in-hole-contact-parameter-sweep.md)
- [SAP analytic square sweep + legacy round controls](sap-peg-in-hole-contact-parameter-sweep.md)
- [analytic square MuJoCo--SAP comparison and `rigid_gap` control](mujoco-vs-sap-square-cavity.md)
- [latest SAP round matrix](sap-round-peg-and-concave-plate-representation-benchmark.md)
- raw outputs: `exp-results/peg-in-hole/`
