# Task matrix 04: simple pinch cube

| Field | Definition |
| --- | --- |
| Physical mechanism | Bilateral normal preload converted into frictional support during lift |
| Setup | Two opposed pads pinch a cube against a known downward load, then execute a prescribed vertical lift |
| Expected outcome | The cube remains supported and follows the no-slip lift |
| Real-world reference | Optional matched robot/gripper trial |

## Task-specific metrics

| Metric | Definition |
| --- | --- |
| Minimum hold grip force \(F^*_{\rm grip,sim}\) | Minimum per-pad grip force that passes the shared static-friction criterion for the hold window. |
| Grip-force error \(E_{\rm grip}\) | Error from the ideal symmetric Coulomb threshold: \(\left|F^*_{\rm grip,sim}-(mg+F_{\rm down})/(2\mu)\right| / ((mg+F_{\rm down})/(2\mu))\). |
| Lift-height error \(E_z(T)\) | \(\left|z_{\rm sim}(T)-z_{\rm ideal}(T)\right|\), where the ideal cube follows the prescribed no-slip lift. |

The shared contact-duty, creep, and friction-utilisation metrics remain common
diagnostics and are not redefined here.
