# Laptop insertion iteration journal

Budget: at most five numerical configurations per engine, one full rollout each; no perturbation repeats or extra benchmark warmup episodes.

## Before dynamics

- Pinned source episode 0, frames 450–549 (18.00–21.96 s), then 1.5 s diagnostic hold.
- Closed assembly fixed at hinge q=0 deg. Source-only initial pose fit; 3 visible face corners, hidden corner not used. Loaded-grasp history unavailable; initial twist follows measured wrist.
- First CoACD 16-piece rack representation rejected: sample slot narrowing 1.5–4.3 mm. No simulation outcome observed.
- Replacement: floor + two isolated wall convex envelopes. Twelve cross-section checks against original triangle mesh: maximum sampled boundary error 0.0174 mm. This is sampled admission, not a global error guarantee.
- Candidate 1 for both engines uses native defaults. Geometry, friction, mass, controller and source inputs are shared.
- Once initialized, stop early if both insert, release and remain supported without gross crossing; inspect common geometric depths, contact continuity, and final-window motion. Finite/capacity/drive checks must pass. Report any unresolved validity instead of declaring success.

## Runs

Completed: SAP 2 configurations and MuJoCo Warp 3 configurations; one complete rollout each. One MuJoCo API startup error occurred before integration and is excluded from rollout/configuration counts.

Initial finger openings are derived from geometric touching of the inferred plate, separately per finger, before either engine runs. The closing command is retained as the source preload request; it is not substituted for the actual loaded joint opening. Base reference meshes have no collision in this detached-gripper model.

## Candidate 1 evidence and candidate 2 decision

SAP R1 completed 5460 steps, finite, no capacity loss; independent rack component penetration reached 3.5638 mm. It stays supported after release, but this overlap is too large relative to the thin slot walls to count as valid success. SAP candidate 2 raises combined pair ke to 100000 N/m (shape ke 200000), retains pair tau 0.02 s (shape tau 0.01), and changes no physical setting. A <=0.5 mm sampled overlap is a diagnostic adequacy target, not a source fidelity tolerance.

MuJoCo R1 initial adapter attempt stopped before integration (external-force array dtype); fixed to native spatial-vector array. Preserved under invalid_adapters. The corrected default candidate completed once, finite with no capacity loss; native peak overlap 4.0062 mm, common observer pending.

## Observer correction (offline, zero additional rollouts)

Initial FCL penetration values violate provable convex separation bounds on this representation and are rejected. For SAP R1 at 0.24 s the reported finger depth 17.8286 mm exceeds the 5.79324 mm face-normal separation bound. At 2.54 s reported rack depth 3.56379 mm exceeds the 0.97271 mm bound. These preliminary values cannot be used for ranking or final reporting. Replace them with [2*intersection-inradius, separating-face translation] bounds and retain original outputs as invalid observer evidence. SAP R2 parameters were already launched on the preliminary observer; independent LP also confirms approximately 5.793 mm finger penetration, so stiffening remains justified. The historical parameter file and reason are preserved, with this correction attached.

## MuJoCo candidate 2

At 2.79 s, an independent convex intersection LP confirms at least 3.423 mm default-run penetration into the rack floor. Candidate 2 changes only geom solref from [0.02,1] to [0.004,1]; solimp and solver defaults remain native. Scene, dt, mass, friction and drive are unchanged. One full rollout, no repeats.

## Third MuJoCo configuration, SAP stops at two

Validated R2 rack penetration bracket: MuJoCo [0.6908,0.7259] mm; SAP [0.07778,0.08661] mm. SAP maximum sampled finger penetration is <=0.15764 mm and its tail remains supported, so SAP is frozen at R2. MuJoCo R3 changes only solref [0.004,1] -> [0.002,1], at the 2*dt safety floor, to reduce the remaining rack overlap. No additional SAP run.

## Stop / final selection

MuJoCo R3 rack penetration bracket [0.06421,0.22460] mm; maximum finger upper bound 0.27919 mm, tail linear RMS 1.8506 mm/s and angular RMS 0.012875 rad/s. SAP R2 rack bracket [0.07778,0.08661] mm; maximum finger upper bound 0.15764 mm, tail linear RMS 1.0075 mm/s and angular RMS 0.016802 rad/s. Both pass the declared diagnostic overlap target and retain the inserted laptop after release in the inspected videos. Stop early: SAP uses 2/5 configurations, MuJoCo 3/5. No additional physics repetitions.

Source final-pose estimation attempt (multi-frame head + wrist) returned 6.90 px residual and failed its 2 px admission gate. It is excluded; pose-reference errors remain null.
