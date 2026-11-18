# RH20T dish on rack — task metrics (real-task playback v1)

**Status: draft for review (2026-11-16). Nothing run; the data audit (SOP phase
P0) has not started.** Regime rules are in the [protocol README](../README.md).

## Source

RH20T `task_0050` (a dish placed on a dish rack). Project file check of
2026-11-16 on a per-episode mirror: each episode carries a rating (0 robot
failure, 1 task failure, 2–9 success quality); the task has 81 successes and
7 task failures across six robot configurations; fields include TCP pose,
gripper width in millimetres, and wrist force/torque on all configurations
except the Franka one. The mirror has no depth and no calibration, resamples
timestamps to 0.1 s, and one configuration has all-zero joint angles.

Everything in this paragraph is re-verified from the official release in P0
before it is relied on. The pilot uses **one** robot configuration, chosen in
P0 by the rule in the manifest.

## Completion table

`P0` and `P1` mark the phase that fills the value. Ranges are half-widths.

| Quantity | Class | How it is obtained | Range | Note |
|---|---|---|---|---|
| TCP pose trace | E1 | official native-rate file | sensor + time base | frame definition confirmed in P0 |
| Gripper width trace | E1 | official file | resolution | drives the OPEN/CLOSED rule |
| Camera-to-base calibration | E1 | official calibration | reprojection residual of the gripper at known TCP poses | sets the E2 floor |
| Table plane | E2 | plane fit to depth | fit residual | |
| Plate diameter, height | E2 | multi-view fit of a revolved profile | spread across views | |
| Plate initial pose | E2 | same fit, first frame | spread + calibration | explicit ± offset cells |
| Rack geometry (slot pitch, post height and diameter, footprint) | E2 | multi-view fit of a parametric rack | spread across views | goal fixture: E2 only |
| Rack pose | E2 | same fit | spread + calibration | explicit ± offset cells |
| Rack fixed or free | E2 | does it move between first and last frame? | — | free only if it visibly moves |
| Rim thickness at the grasp point | E3 | stalled gripper width | width resolution | costs the `acquired` stage |
| Plate mass | E3 | wrist-force step between before and after lift | force noise | E5 interval if no force sensor |
| Pad geometry, stroke | E4 | gripper datasheet or CAD | tolerance | two-pad parallel surrogate |
| Closing force per finger | E4 | datasheet range, command if recorded | datasheet range | low / high cells |
| Plate wall and rim profile away from the grasp point | E5 | three declared variants: flat coupe, shallow rim, raised rim | the variants | variant cells |
| Friction pad–plate | E5 | cited interval for the pad material on glazed ceramic or plastic | interval | low / high cells |
| Friction plate–rack, plate–table | E5 | cited interval | interval | low / high cells |
| Plate inertia | derived | uniform shell of the fitted profile and mass | follows mass | |
| Coupling stiffness and caps | declared | protocol nominal | ÷3 / ×3 | low / high cells |

## Stages and goal predicate

All thresholds below are proposals to be fixed in the manifest before P3b.
`h_rim` is the rim thickness; `d_post` the rack post diameter.

| Stage | Test | Scored |
|---|---|---|
| `acquired` | 0.3 s after the first CLOSED segment begins and the TCP has risen 20 mm, the plate's lowest point is at least 10 mm above the table and both pads touch the plate | reported; **not scored** when rim thickness or grasp position is E3 |
| `carried` | at placement onset (first plate–rack contact, or the start of the final descent if there is none) both pads still touch the plate | yes |
| `placed` | at the instant the CLOSED segment ends, the plate touches the rack and its centre is inside the rack footprint | yes |
| `retained` (= task outcome) | at the end of a 2.0-s settle window after release: plate touches the rack and not the gripper; centre inside the rack footprint; orientation inside the slot's admissible cone (from rack geometry); linear speed ≤ 10 mm/s and angular speed ≤ 0.05 rad/s over the last 0.5 s | yes — **primary** |

The admissible cone and the footprint come from the fitted rack. Whether the
goal is "upright in a slot" or "lying on the rack" is read from the real final
frames in P0 and written into the manifest; per README rule 7 the predicate
must reproduce every admitted episode's real label before any rollout.

Expected-failure episodes (rating 1) are admitted only if P0 can name the
real failure stage from the video and the completion can represent it. Their
cell is correct when the simulated outcome is `not retained`; the report also
states whether the failing stage matches the real one.

## Anomaly thresholds

| Flag | Threshold |
|---|---|
| `deep_penetration` | pad–plate overlap > 0.5 `h_rim`; plate–rack overlap > 0.5 `d_post`; plate–table overlap > 0.5 × plate wall thickness |
| `penetration_assisted` | task success with `deep_penetration` on plate–rack at the end, or on pad–plate during `carried` |
| `ejected` | plate speed > 3 × the maximum recorded TCP speed while touching, or within 50 ms of leaving, any contact; or plate centre outside the workspace box |
| `large_slip` | plate displacement in the gripper frame between lift-off and placement onset > 10 mm or > 10° |
| `invalid` | non-finite state; contact-capacity truncation; plate centre below the table plane; plate fully through a pad or rack member |

Peak and final overlap are saved per interface in millimetres for every
rollout whether or not a flag is raised.

## Diagnostics with a real reference (nominal cell only, not gating)

| Signal | Real | Simulated |
|---|---|---|
| Stalled gripper width | recorded width in the CLOSED segment | pad separation in the CLOSED segment |
| Payload force | wrist-force step after lift | vertical coupling force step after lift |
| Rack contact time | first wrist-force transient in the final descent | first plate–rack contact |

## Margins checked at admission

| Scored stage | Margin *m* | Uncertainty *u* depends on |
|---|---|---|
| `carried` | none geometric; governed by friction and closing force | covered by the friction and force cells |
| `placed` | lateral: slot pitch − plate thickness at the rim; along the slot: footprint length − plate diameter chord at insertion depth | plate pose in hand (plate initial pose, grasp position), rack pose, calibration |
| `retained` | slot depth relative to the plate's centre-of-mass height when seated | rack geometry, plate profile variant |

An episode is admitted only if *m* ≥ 3 *u* for `placed` and `retained`.

## What is not claimed

- No contact-force accuracy: force magnitudes depend on each engine's
  regularisation and on our coupling.
- No statement about the arm, its controller, or teleoperation.
- No fidelity claim from a `reproduces` verdict alone (README §4.3).

## Pilot amendments (2026-11-17, implementation run)

These amendments were made while implementing the protocol on the real data.
Each is a declared revision; none uses a simulated outcome on a held-out
episode. Artifacts: `exp-results/real-playback-v1/rh20t-dish-rack/`.

**P0 facts that replace the draft's assumptions.**
- `task_0050` does not exist in cfg3. cfg2 (Flexiv + AG-95, wrist F/T) was
  chosen by the configuration rule: 29 episodes, 17 success / 5 task failure /
  7 robot failure (official ratings equal the mirror's).
- Official TCP quaternions are **wxyz** (checked geometrically). The 125 Hz
  base-frame TCP in `high_freq_data.npy` is the replayed stream. TCP is the
  fingertip centre; the finger axis is TCP y.
- The table plane is the calibration-board plane (z = 21.2 mm, normal 2.5°
  from base z), not depth: RealSense depth is biased by about −15 mm.
- The wrist-force payload step is unusable (inconsistent sign and size), so
  object mass is **E5**, not E3.
- The "dish" differs by scene: oval wicker basket (user 16 scenes 1–5), white
  plate (6–8), round wicker basket (9–10). Completion is per object class.

**Drive (P3a).** The coupling was raised to K_t = 1e5 N/m, force cap 300 N:
the recordings contain TCP accelerations near 200 m/s² (sample glitches), and
the drafted 1e4 N/m / 50 N failed the 1 mm gate. The coupling cells are ×3 and
÷3 of this value. Finger rule: CLOSED requires a stall lasting at least
0.3 s (stepped 19 mm width commands otherwise create false stalls). The
closing force fades to zero over the last 0.5 mm, so empty fingers do not
cross each other.

**Goal predicate (upright in slot), from the fitted rack only.** The basket
touches the rack, is free of the pads, and is at rest. Its centre is inside
the footprint. Its axis is within 45° of the rails, which means it stands
across them. Its lean is at most atan(2·pitch / peg height) + 10° = 58.5°.
On the E2 final frames this returns the real label for every evaluable
episode. The failure s5 is the basket left parallel to the rails, beside
the slot.

**Admission additions.**
1. *One grasp* (completion c2). Re-grasps are closed-loop operator
   corrections of the real object state; replayed open loop against a
   diverged simulated state they miss. Holds separated by less than 1 s are
   merged. This rule was prompted by the selection episode s1 in P3b/P4
   (disclosed), and it reads only the real gripper trace.
2. *Grasp-position completion* (c3). The E2 first-frame silhouettes (320×180)
   do not place the rim at the grasp point accurately enough: wall slope is
   unconstrained from above, and the object can be pushed during the
   approach. The object is therefore shifted along the finger axis so that its
   wall passes between the fingers at the recorded stall (grasp position, E3).
   This costs the `acquired` stage. The replay starts 0.3 s before the stall,
   or earlier until the pads are clear of the object. Shifts larger than
   3 × the E2 pose range are flagged and reviewed against the video
   (README §2.2 rule 2).

**Finding for the protocol.** `acquired` is unscored, but a grasp failure
cascades into every scored stage. The margin test must therefore cover the
grasp too, either through grasp-region uncertainty in *u* or by making the c3
conditioning standard.
