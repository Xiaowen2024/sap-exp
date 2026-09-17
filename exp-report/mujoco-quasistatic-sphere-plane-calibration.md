# MuJoCo quasi-static sphere--plane calibration

## Goal

Validate the target physical law \(\delta=F/k_e\) before comparing MuJoCo with SAP.
The final validation model is gravity-free, with a 1 kg sphere on a plane, a z-only slide joint, and a 0--10 N force ramp over 1 s followed by a 3 s hold.

## Strict scalar model

The initial box--plane test was not valid for the scalar formula: it can have four coupled contact points. The first sphere model was also invalid because the default `condim=3` creates a contact cone with multiple constraint rows.

The validated model uses an explicit pair:

```xml
<pair geom1="plane" geom2="ball" condim="1" margin="0" gap="0"
      solref="(-s,-b)" solimp="0.9 0.9 0.001 0.5 2"/>
```

Thus there is one normal contact, one normal constraint row, zero margin, and constant impedance \(d_0=d_w=0.9\).

## Mapping and measurements

For \(m=1\,kg\), the direct-format mapping is

\[
s_{MJ}=\frac{(1-d)k_e}{m},\qquad b_{MJ}=2\sqrt{s_{MJ}}.
\]

`s_MJ` has units \(s^{-2}\), not N/m. Accuracy is measured by

\[
R_k=\frac{\delta_{sim}}{F/k_e},\qquad E_{rel}=|R_k-1|.
\]

The hold-window load is measured using `mj_contactForce`; deformation is the z-joint translation, not a single witness-point `contact.dist`.

## Runtime audit: \(k_e=10^4\,N/m\), dt = 0.01 ms

| quantity | value |
|---|---:|
| `contact.dim` | 1 |
| `contact.dist` / `efc_pos` | -0.001000000 m |
| `efc_margin` | 0 m |
| `efc_KBIP=(K,B,I,I')` | (1234.568, 70.273, 0.9, 0) |
| measured normal force | 10.0 N |
| target / simulated deformation | 1.000000 / 1.000000 mm |
| \(R_k\) | 1.00000000000049 |

This rules out a hidden margin, a mm/m conversion error, incorrect external load, and the earlier multi-row contact cone.

## Sweep result

Sweep: \(k_e=10^4\ldots10^9\,N/m\), dt = 0.01, 0.025, 0.05, 0.1 ms.

| target \(k_e\) N/m | dt values with \(R_k\approx1\) |
|---:|---|
| \(10^4\) to \(10^8\) | 0.01, 0.025, 0.05, 0.1 ms |
| \(10^9\) | 0.01, 0.025, 0.05 ms |
| \(10^9\), 0.1 ms | unstable: \(R_k=0\), tail speed 3.01 m/s |

For all stable cases, \(R_k=1\) to floating-point accuracy (about \(10^{-10}\) relative error or better).

At \(k_e=10^9\), \(s_{MJ}=10^8\,s^{-2}\), so \(1/\sqrt{s_{MJ}}=0.1\,ms\). Therefore dt = 0.1 ms is too large relative to the contact recovery time; the observed failure is a dt-dependent numerical-stiffness boundary, not a universal MuJoCo limit.

## Conclusions

1. Direct `solref` is not a physical N/m parameter, but it exactly maps to a target physical stiffness in this strict scalar setting.
2. The mapping requires `condim=1`, one normal DOF, constant impedance, zero margin, and a quasi-static hold.
3. Multi-contact cases require a different reference; one cannot apply this scalar relation directly to box stacks.
4. The fair SAP comparison is now clear: run this same \(R_k(k_e,dt)\) matrix, then compare accuracy, tail jitter, and largest stable dt.

## Artifacts

- Runner: `mujoco-sim/learn_mujoco/compression_experiments/run_sphere_plane_scalar_calibration.py`
- Raw data: `exp-results/quasistatic-squeeze/mujoco-sphere-scalar/results.csv`
- Video: `mujoco-sim/learn_mujoco/compression_experiments/videos/sphere_plane_scalar_ke1e4_dt0p01ms.mp4`
