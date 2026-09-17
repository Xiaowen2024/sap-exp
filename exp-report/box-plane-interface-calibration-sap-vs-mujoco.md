# Four-point box–plane interface calibration — SAP Warp and MuJoCo

## Question

For a flat 40 mm cube resting on a plane, should we assume that four symmetric
contact points give an aggregate stiffness \(K_{\rm interface}=4k_{\rm pair}\)
in SAP? No: this report measures the force–deformation slope directly.

The calibration is deliberately different from the one-contact sphere test.
The sphere test identifies a scalar contact constraint. Here the cube has four
coupled normal contact constraints; the reported quantity is the empirical
aggregate interface slope

\[
K_{\rm interface}=\frac{dF_N}{d\delta_{\rm box}},
\]

not a claim that each individual MuJoCo or SAP point has an independently
measurable stiffness.

## Shared task and measurement

- 40 × 40 × 40 mm, 1 kg box; plane; gravity off.
- Box is constrained to z translation only; x/y and all rotations are locked.
- A known downward force ramps from zero over 0.5 s, then holds until 2 s.
- Four loads: 2, 5, 10, 20 N. Tail-average normal force and vertical geometric
  compression define one \((F,\delta)\) sample; a linear fit uses all four.
- SAP: Drake preset, `tau_pair=3 ms`, `rigid_gap=0`, friction off. Equal
  shape materials are set so that the actual pair value is
  \(k_{\rm pair}=k_{\rm shape}/2\).
- Both dt=.1 and .5 ms are tested. All SAP cases retain four contacts, have no
  truncation, and have negligible tail vertical speed.

## SAP result: the 4× law is experimentally confirmed here

| pair \(k_e\) (N/m) | dt (ms) | measured \(K_{\rm interface}\) (N/m) | \(4k_{\rm pair}\) (N/m) | relative error | fit \(R^2\) |
|---:|---:|---:|---:|---:|---:|
| \(10^5\) | .1 | 400,000.245 | 400,000 | \(6.12\times10^{-7}\) | 0.9999999995 |
| \(10^5\) | .5 | 400,000.334 | 400,000 | \(8.34\times10^{-7}\) | 0.9999999996 |
| \(10^6\) | .1 | 4,000,060.607 | 4,000,000 | \(1.52\times10^{-5}\) | 0.9999999789 |
| \(10^6\) | .5 | 4,000,055.570 | 4,000,000 | \(1.39\times10^{-5}\) | 0.9999999841 |

Therefore, for this very symmetric z-only flat-contact geometry, SAP reproduces

\[
F_N \simeq 4k_{\rm pair}\,\delta_{\rm box}
\]

to better than 0.002% at the tested load and timestep range. This replaces the
earlier *assumption* with a direct measured result.

## MuJoCo empirical match

The MuJoCo calibration uses the same geometry, loads and slope measurement,
but direct `solref` is a native acceleration-level parameter. It is therefore
calibrated empirically rather than numerically equated to SAP \(k_e\).

To match SAP \(k_{\rm pair}=10^6\) N/m, whose measured aggregate interface
stiffness is \(4\times10^6\) N/m, the best tested MuJoCo point is:

| MuJoCo direct `solref` stiffness (s\(^{-2}\)) | direct damping (s\(^{-1}\)) | measured \(K_{\rm interface}\) (N/m) | target (N/m) | fit \(R^2\) |
|---:|---:|---:|---:|---:|
| 100,000 | 632.456 | 3,999,999.9999995 | 4,000,000 | 1.0 |

This establishes a useful *geometry-specific matched-compliance regime* for
later box-stack experiments. It does **not** establish that a MuJoCo point has
the same material stiffness as SAP \(k_e\), nor that the mapping transfers to
rotating, sliding, uneven, or changing-contact-manifold problems.

## What this means / 中文解释

- SAP 不是“假设四点就一定是 \(4k\)”；这里实际施加 2/5/10/20 N，测出了
  \(F\)-\(\delta\) 线性斜率。此任务中它确实几乎等于 \(4k_{\rm pair}\)。
- 这是因为四个点对称、box 只可 z 方向移动、每个点的 overlap 相同。若 box 倾斜、
  接触点增减、允许转动，四个 constraint 会耦合，不能直接沿用这个 4× relation。
- MuJoCo 不应把内部 `solref` 数字硬写成 SAP 的 \(k_e\)。正确做法是像这里一样，
  直接匹配实测的 aggregate \(F\)-\(\delta\) slope。

## Reproduction and raw data

- SAP runner: `sap-sim/scripts/run_box_plane_interface_calibration.py`
- SAP data: `exp-results/box-plane-interface-calibration/sap/force_delta.csv`, `fits.csv`, `summary.json`
- MuJoCo runner: `mujoco-sim/learn_mujoco/compression_experiments/run_box_plane_interface_calibration.py`
- MuJoCo data: `exp-results/box-plane-interface-calibration/mujoco/force_delta.csv`, `fits.csv`, `summary.json`
