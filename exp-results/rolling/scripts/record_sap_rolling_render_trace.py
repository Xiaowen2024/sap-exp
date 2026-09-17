"""Record full SAP rolling-test poses for visual replay (every 10 ms)."""
from pathlib import Path
import sys
import numpy as np
import warp as wp

WORKSPACE = Path(__file__).resolve().parents[3]
SAP_ROOT = WORKSPACE / "sap-sim"
if str(SAP_ROOT) not in sys.path: sys.path.insert(0, str(SAP_ROOT))
from run_sap_rolling_benchmark import DEVICE, DT, DURATION, FORCE, apply_box_force, scene_config
from sim.collision.pipeline import SapCollisionPipeline
from sim.loader.scene import load_sap_scene
from sim.resources.collision_model import sap_collision_state_from_state
from sim.solver_sap import SolverSAP

OUT = WORKSPACE / "exp-results" / "rolling" / "sap-cuda"

OUT.mkdir(parents=True, exist_ok=True)
scene = OUT / "scene.yaml"; import yaml; scene.write_text(yaml.safe_dump(scene_config(), sort_keys=False))
loaded = load_sap_scene(scene, device=DEVICE, rigid_contact_max=64, strict=True)
model, state, control = loaded.sap_model, loaded.sap_state, loaded.sap_control
nxt = model.state(); solver = SolverSAP(model, max_rigid_contact=64, contact_preset_variant="drake", line_search_variant="armijo_decay")
pipe = SapCollisionPipeline(loaded.collision_model, rigid_contact_max=64); contacts = pipe.contacts()
box_body = model.body_label.index("box"); every = 10; samples = []
for step in range(int(DURATION / DT)):
    if step % every == 0: samples.append(state.joint_q.numpy())
    state.clear_forces(); wp.launch(apply_box_force, dim=1, inputs=[state.body_f, box_body, FORCE*.5, FORCE*np.sqrt(3.)/2.], device=DEVICE)
    pipe.collide(sap_collision_state_from_state(state), contacts); solver.step(state, nxt, control, contacts, DT); state, nxt = nxt, state
np.save(OUT / "all-body-q-10ms.npy", np.asarray(samples))
print(f"saved {len(samples)} pose frames to {OUT / 'all-body-q-10ms.npy'}")
