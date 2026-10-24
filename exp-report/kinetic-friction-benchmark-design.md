# Kinetic-Friction Benchmark Design Notes

This note records the next benchmark design. It does not revise or reinterpret the
existing SAP/Warp--MuJoCo results.

## Shared evaluation layers

Every kinetic-friction task uses the common protocol:

- **Real-world accuracy:** support-frame trajectory RMSE, DTW, fitted tangential
  acceleration/deceleration error, and terminal tangential-displacement error.
- **Simulation-level consistency:** analytic acceleration and terminal-displacement
  error, friction utilisation, contact duty, and normal-support/lift-off error.
- **Mechanism-aware usability:** bounded native-parameter tuning budget,
  plausible-range controllability, numerical local robustness, and robustness across
  meaningful environment/setup variants.

Physical environment properties, including mass, geometry, external load, and
friction coefficient, are fixed per rollout and shared across engines. They are not
engine-tuning variables.

## 1. GAUGE Slope Slider

**Role.** Real-world atomic kinetic-friction validation: a block slides along a
fixed incline.

**Additional task-level metrics.** None. The shared suite is sufficient. In
particular, acceleration and terminal displacement directly test the kinetic
friction law, while the normal-support check prevents a trajectory match obtained
through unintended lift-off or gliding.

## 2. Robot planar pushing (MIT Push Dataset)

**Role.** Task-level extension from a single sliding contact to robot manipulation.
A robot tip pushes an object across a table, coupling pusher--object contact,
object--table friction, and object rotation.

**Task-specific metrics.**

1. **Final object-pose error.** Difference between simulated and real final
   object pose \((x,y,\theta)\). This evaluates whether the same robot push
   produces the correct final placement and orientation.

2. **Force--motion response error.** Compare, over a small set of pre-registered
   stable-contact windows, both:
   \[
   E_{\mathrm{wrench}} =
   \left\|\bar{\mathbf w}_{\mathrm{sim}}-
   \bar{\mathbf w}_{\mathrm{real}}\right\|,
   \qquad
   E_{\mathrm{twist}} =
   \left\|\bar{\boldsymbol\xi}_{\mathrm{sim}}-
   \bar{\boldsymbol\xi}_{\mathrm{real}}\right\|,
   \]
   where \(\mathbf w=(F_x,F_y,\tau_z)\) is the pusher--object contact wrench
   and \(\boldsymbol\xi=(v_x,v_y,\omega)\) is the object planar twist.

Use 3--5 windows (for example: first sustained contact, early/mid/late steady
push, and optional release), each averaged over a short duration after stream
synchronisation. Do not require instantaneous \(F=ma\): the object also
experiences distributed table friction and frictional torque. Reporting wrench and
twist errors separately distinguishes a wrong force response from a wrong
motion response.

## Rationale

The two tasks form a clean progression:

\[
\text{real single-contact sliding}
\;\longrightarrow\;
\text{real robot-mediated planar pushing}.
\]

The first isolates kinetic friction. The second tests whether the same physical
mechanism remains predictive when robot contact geometry and planar rotation are
introduced.
