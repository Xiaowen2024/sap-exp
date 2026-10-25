# Panda manipulation — composite task metrics

Panda manipulation evaluates whether contact-model differences remain useful at the robotic task level, where actuation, controller tracking, object contact, and longer-horizon outcome are coupled.

The exact manipulation scenario and its reference data must be registered with each experiment. This task file applies to fixed-trajectory push, grasp/transfer, or contact-rich placement rollouts.

## 1. Real-world fidelity

Use this layer only for a matched physical rollout with synchronized robot and object state.

| Metric | Definition | Use |
|---|---|---|
| **Task-object trajectory error** | Time-aligned error of the task-defined object pose/state in the relevant support frame. | Primary physical prediction metric. |
| **Final task-state error** | Error of the required final object pose, placement, or transfer state. | Captures accumulated contact error at task completion. |
| **End-effector tracking error** | Error between commanded and realized end-effector state, reported separately from object error. | Separates controller-tracking failure from contact-model failure. |

## 2. Simulation consistency

| Metric | Definition | Use |
|---|---|---|
| **Physically valid task outcome** | Success/failure under the task's registered completion criterion, including no penetration-assisted grasp, placement, or transfer. | Primary task-level diagnostic. |
| **Object retention / slip outcome** | For grasp or transfer phases, record unintended object slip, drop, or unexpected contact loss. | Identifies contact-mode failure. |
| **Post-contact residual motion** | Task-defined tail object speed or pose drift after the intended placement/hold phase. | Detects jitter or failure to settle. |

## Notes

Controller tracking and object-contact outcome must be reported separately. A task may fail because the arm did not execute its commanded motion, because contact was physically wrong, or because both occurred; a single success flag cannot identify the cause.
