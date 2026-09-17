# MuJoCo three-thin-disc comprehensive contact sweep

This extends the earlier thin-disc report with 79 controlled runs on the same
three free cylinders: radius 101 mm, total thickness 4 mm, mass 0.333 kg;
initial centers `(0,0,30)`, `(3,-2,70)`, `(-2,2,110)` mm; gravity-only settling
for 2 s. Metrics are raw `max(0,-contact.dist)`, final stack COM error from the
ideal 6 mm height, disc-normal tilt (yaw excluded), tail peak-to-peak COM/tilt
jitter, velocity-based settling time, final layer order, contact count and
MuJoCo solver iterations.

## Sweep design

| group | controlled variables | runs |
|---|---|---:|
| primary | `dt={0.1,0.25,0.5,1,2,5} ms` × `solref timeconst={0.2,0.5,1,2,4,8,16} ms`, valid `timeconst >= 2dt` only | 26 |
| damping | fixed `dt=0.5 ms`; valid solref levels × `dampratio={0.5,1,2}` | 15 |
| impedance | fixed `dt=0.5 ms`; valid solref levels × uniform `solimp d0=dwidth={0.9,0.95,0.99,0.999}` | 20 |
| ablation | `dt=0.5 ms`, `solref={1,4,16} ms` × `Euler/implicitfast/RK4` × `Newton/CG` | 18 |

MuJoCo does not expose a `discrete` option integrator in this XML interface;
the available ablation is Euler, implicitfast and RK4. Native positive-format
`solref` requires `timeconst >= 2dt`; invalid pairs are intentionally omitted
rather than silently clamped.

## Main findings

- The lowest peak penetration in the primary grid is **0.0965 mm** at
  `dt=0.1 ms, solref=0.2 ms, implicitfast/Newton`; it retains order but has
  `0.00195 mm` tail COM jitter and settles at 1.635 s.
- The best balanced low-cost point among the tested settings is arguably
  `dt=0.25 ms, solref=1 ms`: **0.3679 mm** peak, **0.00080 mm** final COM
  error and 0.178 s settling time.
- The original soft region is a real qualitative failure: `solref >= 8 ms`
  fails layer-order preservation in many dt cases. At `dt=0.5 ms, 16 ms`, peak
  penetration is 5.411 mm; at `dampratio=2` it reaches 9.148 mm.
- Changing `solimp` is not monotonic in this nonlinear impact: at
  `dt=0.5 ms, solref=1 ms`, uniform `dmax=0.9` gives 0.1449 mm peak whereas
  `0.95--0.999` gives roughly 0.65--0.67 mm. It must be assessed with
  settling/jitter, not called universally “stiffer is better.”
- Newton and CG yield near-identical physical trajectories in the tested
  stable 1 ms regime, but CG consumes around **13--15** mean solver iterations
  versus Newton around **1.2**. At 16 ms, all integrator/solver combinations
  preserve neither correct layer order nor a credible stack.

## Failure boundary observed

`failure` means non-finite state, escaped disc, or incorrect final vertical
order—not merely a high transient penetration. In the primary matrix, all
`solref=8/16 ms` at `dt<=0.5 ms` fail; 16 ms fails at every tested timestep.
At coarse dt the valid solref domain shrinks due to `timeconst >= 2dt`, so this
is a stress-test boundary for this task, not a universal MuJoCo limit.

## Full machine-readable record

Every one of the 79 runs, including all named metrics and exact parameters, is
in [results.csv](../exp-results/thin-discs/mujoco-comprehensive-sweep/results.csv)
and [results.json](../exp-results/thin-discs/mujoco-comprehensive-sweep/results.json).
The reproducible runner is
[sweep_thin_disc_stack_comprehensive.py](../mujoco-sim/learn_mujoco/thin_disc_experiments/sweep_thin_disc_stack_comprehensive.py).
