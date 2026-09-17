# MuJoCo thin-disc comprehensive sweep — readable results

All runs simulate the same three-disc gravity stack for 2 s. `pen` is the maximum raw MuJoCo `-contact.dist`; `COM err` is absolute final system-COM height error from 6 mm; orientation is maximum disc-normal tilt (yaw excluded); jitter is peak-to-peak over the final 0.5 s. `failure` means non-finite, escaped, or wrong final layer order.

The full-precision CSV remains the source of record: [results.csv](../exp-results/thin-discs/mujoco-comprehensive-sweep/results.csv).

## Metric columns

| column | meaning |
|---|---|
| pen (mm) | maximum penetration over every contact and time step |
| COM err (mm) | `abs(final COM z - 6 mm)` |
| tilt (deg) | largest pitch/roll tilt of any disc; yaw excluded |
| COM jitter / tilt jitter | tail peak-to-peak metric over final 0.5 s |
| settle (s) | last time any disc speed exceeded 1e-3 (m/s or rad/s) |
| iters mean/max | MuJoCo contact solver iterations |

## Patterns found in each sweep

### 1. Primary `dt × solref`: the ratio matters, but not by itself

- Lower `solref` (stiffer/faster contact) generally lowers the impact peak:
  at `dt=0.1 ms`, 0.2 / 0.5 / 1 / 2 / 4 ms gives 0.097 / 0.235 / 0.478 /
  0.953 / 2.020 mm, respectively.
- The extreme near-limit settings, `timeconst=2dt` (0.1/0.2 ms and
  0.25/0.5 ms), minimize penetration but have long residual settling:
  1.635 s and at least 2.0 s in this 2 s measurement window. Thus “hardest”
  is not automatically “best settled.”
- A useful balance is `dt=0.25 ms, solref=1 ms`: 0.368 mm peak, 0.00080 mm
  final COM error, 0.178 s settling, correct order. `dt=0.5 ms, solref=1 ms`
  remains stable but rises to 0.652 mm peak and 1.169 s settling.
- Soft contact becomes a qualitative stack failure, not merely a larger error:
  `solref=8 ms` fails order at 0.1/0.25/0.5/2 ms; `16 ms` fails at every
  valid tested timestep. The coarse `dt=5 ms` point only permits 16 ms and
  therefore also fails.

### 2. `dampratio`: under-damping is visible as jitter; over-damping is not universally safer

- At `dt=0.5 ms, solref=1 ms`, raising damping 0.5 → 1 → 2 reduces the
  settling time 2.000 → 1.169 → 0.179 s and reduces COM jitter
  0.00534 → 0.00018 → 0.00005 mm. The 0.5 case is clearly under-damped.
- Peak penetration at that same 1 ms stiffness changes only modestly
  (0.781 → 0.652 → 0.650 mm), so damping mostly controls the ring-down rather
  than providing an independent stiffness knob.
- In the soft 16 ms regime, `dampratio=2` makes the peak much worse
  (9.148 mm) and still fails ordering. More damping cannot repair a contact
  time scale that is too soft for this stack.

### 3. `solimp`: impedance influences transients and settling non-monotonically

- The reported ablation sets `d0=dwidth=dmax`, so it changes the impedance at
  both zero distance and at the 2 mm transition width; it is not a pure single
  coefficient comparison with the earlier standard `0.95/0.99` setting.
- At `solref=1 ms`, `dmax=0.9` produces the smallest peak (0.145 mm) but has
  not settled by 2 s; `dmax=0.999` has a larger 0.653 mm peak but settles by
  0.224 s with low tail jitter. This is a concrete penetration-versus-ringdown
  tradeoff.
- At `solref=2--4 ms`, all dmax values give similar peak ranges (about
  1.01--1.06 mm and 1.95--2.07 mm), while settling time varies substantially.
  Hence impedance tuning should use final pose/jitter as well as peak overlap.
- At 8/16 ms no tested dmax repairs the layer-order failure: impedance is not
  a substitute for an adequate contact time scale.

### 4. Integrator × solver: physical behavior is nearly solver-invariant in the stable regime; cost is not

- At `dt=0.5 ms, solref=1 ms`, Euler and implicitfast with Newton/CG all give
  0.652 mm peak; RK4 gives 0.678 mm. They preserve ordering, so integrator is
  a minor effect compared with contact softness here.
- Newton uses about 1.2 mean contact iterations at 1 ms, while CG needs about
  13--15 for nearly the same trajectory. Newton is therefore the efficient
  default in this small stack.
- At `solref=4 ms`, the same conclusion holds qualitatively; RK4 has a
  slightly higher 2.197 mm peak than Euler/implicitfast at 2.061 mm.
- At `solref=16 ms`, every Euler/implicitfast/RK4 × Newton/CG combination
  fails layer order. Changing the time integrator or nonlinear solver does not
  rescue an overly compliant contact model.

## Primary sweep — timestep × solref

| dt (ms) | solref (ms) | damp | d0/dwidth | integrator | solver | pen (mm) | COM err (mm) | tilt (deg) | COM jitter (mm) | tilt jitter (deg) | settle (s) | iters mean/max | contacts | order | failure |
|---:|---:|---:|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 0.1000 | 0.2000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 0.0965 | 0.0039 | 0.0174 | 0.0019 | 0.0026 | 1.6348 | 0.97/6 | 13 | yes | no |
| 0.1000 | 0.5000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 0.2345 | 0.0002 | 0.0026 | 0.0008 | 0.0008 | 1.8967 | 1.05/8 | 13 | yes | no |
| 0.1000 | 1.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 0.4784 | 0.0008 | 0.0012 | 0.0000 | 0.0000 | 0.1872 | 1.20/6 | 13 | yes | no |
| 0.1000 | 2.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 0.9528 | 0.0035 | 0.0039 | 0.0000 | 0.0000 | 0.2080 | 1.07/5 | 13 | yes | no |
| 0.1000 | 4.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 2.0196 | 0.0141 | 0.0022 | 0.0000 | 0.0000 | 0.2051 | 0.98/5 | 13 | yes | no |
| 0.1000 | 8.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 4.0131 | 0.0569 | 0.1030 | 0.0000 | 0.0000 | 0.2908 | 1.01/6 | 18 | no | yes |
| 0.1000 | 16.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 5.5321 | 0.2265 | 0.0471 | 0.0000 | 0.0000 | 0.2928 | 0.99/12 | 25 | no | yes |
| 0.2500 | 0.5000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 0.2109 | 0.0002 | 0.0208 | 0.0011 | 0.0007 | 2.0000 | 1.04/7 | 13 | yes | no |
| 0.2500 | 1.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 0.3679 | 0.0008 | 0.0048 | 0.0001 | 0.0000 | 0.1782 | 1.18/6 | 13 | yes | no |
| 0.2500 | 2.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 0.8373 | 0.0035 | 0.0022 | 0.0000 | 0.0000 | 0.1898 | 1.26/6 | 13 | yes | no |
| 0.2500 | 4.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 1.9077 | 0.0141 | 0.0021 | 0.0000 | 0.0000 | 0.2018 | 0.98/5 | 13 | yes | no |
| 0.2500 | 8.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 3.9777 | 0.0569 | 0.2305 | 0.0000 | 0.0000 | 0.2953 | 1.02/7 | 18 | no | yes |
| 0.2500 | 16.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 5.4560 | 0.2265 | 0.0565 | 0.0000 | 0.0001 | 0.2925 | 1.01/9 | 25 | no | yes |
| 0.5000 | 1.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 0.6525 | 0.0008 | 0.0254 | 0.0002 | 0.0000 | 1.1690 | 1.18/7 | 13 | yes | no |
| 0.5000 | 2.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 1.0625 | 0.0035 | 0.0021 | 0.0001 | 0.0000 | 0.1970 | 1.19/7 | 13 | yes | no |
| 0.5000 | 4.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 2.0608 | 0.0141 | 0.0018 | 0.0000 | 0.0001 | 0.2065 | 0.98/5 | 13 | yes | no |
| 0.5000 | 8.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 4.0142 | 0.0569 | 1.4435 | 0.0000 | 0.0001 | 0.5630 | 1.05/9 | 18 | no | yes |
| 0.5000 | 16.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 5.4115 | 0.2265 | 0.1235 | 0.0000 | 0.0002 | 1.9155 | 1.02/7 | 24 | no | yes |
| 1.0000 | 2.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 1.0014 | 0.0035 | 0.0188 | 0.0002 | 0.0001 | 1.9900 | 1.16/4 | 13 | yes | no |
| 1.0000 | 4.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 1.8368 | 0.0142 | 0.0019 | 0.0000 | 0.0002 | 1.9910 | 0.98/6 | 13 | yes | no |
| 1.0000 | 8.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 3.5340 | 0.0568 | 0.0029 | 0.0000 | 0.0002 | 1.9930 | 1.00/3 | 13 | yes | no |
| 1.0000 | 16.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 4.4442 | 0.2265 | 0.0772 | 0.0000 | 0.0002 | 1.9980 | 1.04/8 | 21 | no | yes |
| 2.0000 | 4.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 1.0768 | 0.0141 | 0.0150 | 0.0001 | 0.0005 | 1.9980 | 1.14/8 | 13 | yes | no |
| 2.0000 | 8.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 3.8919 | 0.0569 | 0.0030 | 0.0000 | 0.0005 | 2.0000 | 1.00/5 | 18 | no | yes |
| 2.0000 | 16.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 4.3670 | 0.2265 | 0.0053 | 0.0001 | 0.0008 | 2.0000 | 1.02/5 | 21 | no | yes |
| 5.0000 | 16.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 4.1661 | 0.2297 | 1.7739 | 0.0010 | 0.0012 | 2.0000 | 1.28/10 | 21 | no | yes |

## Secondary sweep — damping ratio

| dt (ms) | solref (ms) | damp | d0/dwidth | integrator | solver | pen (mm) | COM err (mm) | tilt (deg) | COM jitter (mm) | tilt jitter (deg) | settle (s) | iters mean/max | contacts | order | failure |
|---:|---:|---:|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 0.5000 | 1.0000 | 0.5000 | 0.9500/0.9900 | implicitfast | Newton | 0.7806 | 0.0002 | 2.3682 | 0.0053 | 0.0034 | 1.9995 | 1.04/8 | 13 | yes | no |
| 0.5000 | 1.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 0.6525 | 0.0008 | 0.0254 | 0.0002 | 0.0000 | 1.1690 | 1.18/7 | 13 | yes | no |
| 0.5000 | 1.0000 | 2.0000 | 0.9500/0.9900 | implicitfast | Newton | 0.6497 | 0.0035 | 0.0019 | 0.0000 | 0.0000 | 0.1790 | 1.22/8 | 13 | yes | no |
| 0.5000 | 2.0000 | 0.5000 | 0.9500/0.9900 | implicitfast | Newton | 0.7076 | 0.0007 | 0.6647 | 0.0002 | 0.0000 | 1.1535 | 1.35/8 | 13 | yes | no |
| 0.5000 | 2.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 1.0625 | 0.0035 | 0.0021 | 0.0001 | 0.0000 | 0.1970 | 1.19/7 | 13 | yes | no |
| 0.5000 | 2.0000 | 2.0000 | 0.9500/0.9900 | implicitfast | Newton | 1.0415 | 0.0141 | 0.0012 | 0.0000 | 0.0000 | 0.2155 | 0.97/4 | 13 | yes | no |
| 0.5000 | 4.0000 | 0.5000 | 0.9500/0.9900 | implicitfast | Newton | 1.6901 | 0.0035 | 5.1820 | 0.0001 | 0.0001 | 0.7165 | 1.16/9 | 13 | yes | no |
| 0.5000 | 4.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 2.0608 | 0.0141 | 0.0018 | 0.0000 | 0.0001 | 0.2065 | 0.98/5 | 13 | yes | no |
| 0.5000 | 4.0000 | 2.0000 | 0.9500/0.9900 | implicitfast | Newton | 2.2185 | 0.0568 | 0.0019 | 0.0000 | 0.0001 | 0.2935 | 0.98/3 | 13 | yes | no |
| 0.5000 | 8.0000 | 0.5000 | 0.9500/0.9900 | implicitfast | Newton | 3.4073 | 0.0142 | 2.0778 | 0.0000 | 0.0001 | 0.4965 | 1.09/10 | 13 | yes | no |
| 0.5000 | 8.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 4.0142 | 0.0569 | 1.4435 | 0.0000 | 0.0001 | 0.5630 | 1.05/9 | 18 | no | yes |
| 0.5000 | 8.0000 | 2.0000 | 0.9500/0.9900 | implicitfast | Newton | 3.9204 | 0.2264 | 0.0402 | 0.0000 | 0.0001 | 0.4385 | 1.00/7 | 21 | no | yes |
| 0.5000 | 16.0000 | 0.5000 | 0.9500/0.9900 | implicitfast | Newton | 3.9771 | 0.0569 | 0.6192 | 0.0000 | 0.0001 | 1.2565 | 1.09/9 | 21 | no | yes |
| 0.5000 | 16.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 5.4115 | 0.2265 | 0.1235 | 0.0000 | 0.0002 | 1.9155 | 1.02/7 | 24 | no | yes |
| 0.5000 | 16.0000 | 2.0000 | 0.9500/0.9900 | implicitfast | Newton | 9.1480 | 0.8320 | 0.0142 | 0.0000 | 0.0002 | 1.5365 | 1.01/7 | 21 | no | yes |

## Secondary sweep — solimp impedance

| dt (ms) | solref (ms) | damp | d0/dwidth | integrator | solver | pen (mm) | COM err (mm) | tilt (deg) | COM jitter (mm) | tilt jitter (deg) | settle (s) | iters mean/max | contacts | order | failure |
|---:|---:|---:|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 0.5000 | 1.0000 | 1.0000 | 0.9000/0.9000 | implicitfast | Newton | 0.1449 | 0.0016 | 0.0231 | 0.0002 | 0.0001 | 1.9990 | 1.15/3 | 13 | yes | no |
| 0.5000 | 1.0000 | 1.0000 | 0.9500/0.9500 | implicitfast | Newton | 0.6746 | 0.0007 | 0.1989 | 0.0011 | 0.0005 | 1.9635 | 1.20/6 | 13 | yes | no |
| 0.5000 | 1.0000 | 1.0000 | 0.9900/0.9900 | implicitfast | Newton | 0.6570 | 0.0157 | 0.0468 | 0.0018 | 0.0029 | 1.2845 | 0.97/6 | 13 | yes | no |
| 0.5000 | 1.0000 | 1.0000 | 0.9990/0.9990 | implicitfast | Newton | 0.6530 | 0.0031 | 0.0062 | 0.0001 | 0.0002 | 0.2240 | 0.97/6 | 13 | yes | no |
| 0.5000 | 2.0000 | 1.0000 | 0.9000/0.9000 | implicitfast | Newton | 1.0067 | 0.0065 | 0.0088 | 0.0001 | 0.0001 | 2.0000 | 1.13/5 | 13 | yes | no |
| 0.5000 | 2.0000 | 1.0000 | 0.9500/0.9500 | implicitfast | Newton | 1.0315 | 0.0032 | 0.0059 | 0.0001 | 0.0000 | 0.2135 | 1.20/5 | 13 | yes | no |
| 0.5000 | 2.0000 | 1.0000 | 0.9900/0.9900 | implicitfast | Newton | 1.0523 | 0.0005 | 0.0029 | 0.0013 | 0.0002 | 1.9695 | 1.21/5 | 13 | yes | no |
| 0.5000 | 2.0000 | 1.0000 | 0.9990/0.9990 | implicitfast | Newton | 1.0565 | 0.0001 | 0.0008 | 0.0014 | 0.0001 | 1.8090 | 0.98/5 | 13 | yes | no |
| 0.5000 | 4.0000 | 1.0000 | 0.9000/0.9000 | implicitfast | Newton | 1.9515 | 0.0261 | 0.0112 | 0.0000 | 0.0001 | 1.9955 | 0.98/4 | 13 | yes | no |
| 0.5000 | 4.0000 | 1.0000 | 0.9500/0.9500 | implicitfast | Newton | 2.0101 | 0.0130 | 0.0020 | 0.0000 | 0.0001 | 0.2040 | 0.98/5 | 13 | yes | no |
| 0.5000 | 4.0000 | 1.0000 | 0.9900/0.9900 | implicitfast | Newton | 2.0564 | 0.0025 | 0.0014 | 0.0000 | 0.0000 | 0.2030 | 1.23/6 | 13 | yes | no |
| 0.5000 | 4.0000 | 1.0000 | 0.9990/0.9990 | implicitfast | Newton | 2.0657 | 0.0002 | 0.0012 | 0.0001 | 0.0003 | 1.3215 | 1.03/5 | 13 | yes | no |
| 0.5000 | 8.0000 | 1.0000 | 0.9000/0.9000 | implicitfast | Newton | 3.9981 | 0.1049 | 0.0055 | 0.0000 | 0.0001 | 1.9950 | 0.99/4 | 18 | no | yes |
| 0.5000 | 8.0000 | 1.0000 | 0.9500/0.9500 | implicitfast | Newton | 3.9605 | 0.0524 | 0.1006 | 0.0000 | 0.0001 | 0.4780 | 1.04/8 | 18 | no | yes |
| 0.5000 | 8.0000 | 1.0000 | 0.9900/0.9900 | implicitfast | Newton | 3.9818 | 0.0104 | 0.0012 | 0.0000 | 0.0000 | 0.2185 | 1.02/6 | 18 | no | yes |
| 0.5000 | 8.0000 | 1.0000 | 0.9990/0.9990 | implicitfast | Newton | 3.9649 | 0.0009 | 0.0075 | 0.0001 | 0.0002 | 0.3395 | 1.16/12 | 18 | no | yes |
| 0.5000 | 16.0000 | 1.0000 | 0.9000/0.9000 | implicitfast | Newton | 4.4823 | 0.4199 | 0.0191 | 0.0001 | 0.0004 | 1.9770 | 0.97/5 | 21 | no | yes |
| 0.5000 | 16.0000 | 1.0000 | 0.9500/0.9500 | implicitfast | Newton | 4.5783 | 0.2099 | 0.0111 | 0.0000 | 0.0002 | 1.9460 | 0.99/5 | 21 | no | yes |
| 0.5000 | 16.0000 | 1.0000 | 0.9900/0.9900 | implicitfast | Newton | 5.2889 | 0.0419 | 0.0933 | 0.0000 | 0.0000 | 0.5170 | 1.03/12 | 24 | no | yes |
| 0.5000 | 16.0000 | 1.0000 | 0.9990/0.9990 | implicitfast | Newton | 5.0335 | 0.0223 | 0.4009 | 0.0122 | 0.0139 | 0.3405 | 1.13/13 | 21 | no | yes |

## Ablation — integrator × solver

| dt (ms) | solref (ms) | damp | d0/dwidth | integrator | solver | pen (mm) | COM err (mm) | tilt (deg) | COM jitter (mm) | tilt jitter (deg) | settle (s) | iters mean/max | contacts | order | failure |
|---:|---:|---:|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 0.5000 | 1.0000 | 1.0000 | 0.9500/0.9900 | Euler | Newton | 0.6525 | 0.0008 | 0.0254 | 0.0002 | 0.0000 | 1.1720 | 1.18/7 | 13 | yes | no |
| 0.5000 | 1.0000 | 1.0000 | 0.9500/0.9900 | Euler | CG | 0.6525 | 0.0008 | 0.0254 | 0.0002 | 0.0000 | 1.1710 | 13.67/74 | 13 | yes | no |
| 0.5000 | 1.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 0.6525 | 0.0008 | 0.0254 | 0.0002 | 0.0000 | 1.1690 | 1.18/7 | 13 | yes | no |
| 0.5000 | 1.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | CG | 0.6525 | 0.0008 | 0.0254 | 0.0002 | 0.0000 | 1.1690 | 13.18/57 | 13 | yes | no |
| 0.5000 | 1.0000 | 1.0000 | 0.9500/0.9900 | RK4 | Newton | 0.6781 | 0.0008 | 0.0063 | 0.0001 | 0.0000 | 0.1990 | 1.47/4 | 13 | yes | no |
| 0.5000 | 1.0000 | 1.0000 | 0.9500/0.9900 | RK4 | CG | 0.6781 | 0.0008 | 0.0063 | 0.0001 | 0.0000 | 0.1990 | 14.90/64 | 13 | yes | no |
| 0.5000 | 4.0000 | 1.0000 | 0.9500/0.9900 | Euler | Newton | 2.0608 | 0.0141 | 0.0018 | 0.0000 | 0.0001 | 0.2065 | 0.98/5 | 13 | yes | no |
| 0.5000 | 4.0000 | 1.0000 | 0.9500/0.9900 | Euler | CG | 2.0608 | 0.0141 | 0.0018 | 0.0000 | 0.0001 | 0.2065 | 10.52/75 | 13 | yes | no |
| 0.5000 | 4.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 2.0608 | 0.0141 | 0.0018 | 0.0000 | 0.0001 | 0.2065 | 0.98/5 | 13 | yes | no |
| 0.5000 | 4.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | CG | 2.0608 | 0.0142 | 0.0018 | 0.0000 | 0.0000 | 0.2065 | 10.69/62 | 13 | yes | no |
| 0.5000 | 4.0000 | 1.0000 | 0.9500/0.9900 | RK4 | Newton | 2.1970 | 0.0141 | 0.0025 | 0.0000 | 0.0000 | 0.2090 | 0.98/3 | 13 | yes | no |
| 0.5000 | 4.0000 | 1.0000 | 0.9500/0.9900 | RK4 | CG | 2.1970 | 0.0141 | 0.0025 | 0.0000 | 0.0000 | 0.2090 | 10.78/72 | 13 | yes | no |
| 0.5000 | 16.0000 | 1.0000 | 0.9500/0.9900 | Euler | Newton | 5.4115 | 0.2265 | 0.1234 | 0.0000 | 0.0002 | 1.8970 | 1.01/7 | 24 | no | yes |
| 0.5000 | 16.0000 | 1.0000 | 0.9500/0.9900 | Euler | CG | 5.4115 | 0.2265 | 0.1234 | 0.0000 | 0.0002 | 1.9340 | 9.43/81 | 24 | no | yes |
| 0.5000 | 16.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | Newton | 5.4115 | 0.2265 | 0.1235 | 0.0000 | 0.0002 | 1.9155 | 1.02/7 | 24 | no | yes |
| 0.5000 | 16.0000 | 1.0000 | 0.9500/0.9900 | implicitfast | CG | 5.4117 | 0.2265 | 0.1232 | 0.0000 | 0.0002 | 1.9610 | 9.43/92 | 24 | no | yes |
| 0.5000 | 16.0000 | 1.0000 | 0.9500/0.9900 | RK4 | Newton | 5.5847 | 0.2265 | 0.0446 | 0.0000 | 0.0000 | 0.2920 | 1.00/8 | 25 | no | yes |
| 0.5000 | 16.0000 | 1.0000 | 0.9500/0.9900 | RK4 | CG | 5.5847 | 0.2265 | 0.0446 | 0.0000 | 0.0000 | 0.2920 | 9.45/100 | 25 | no | yes |
