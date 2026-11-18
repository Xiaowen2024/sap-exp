with open("docs/mechanism-eval.html", "r", encoding="utf-8") as f:
    text = f.read()

# Replace remaining math in task-setup-box with pure clean HTML
clean_task_setup = """          <div class="task-setup-box">
            <h4>3D Task Setup: Dynamically Tilting Plate &amp; Free-Sliding Block</h4>
            <ul>
              <li><strong>Three-Dimensional Spatial Geometry:</strong> A rectangular rigid board (30 cm &times; 20 cm &times; 2 cm) is mounted in 3D space. A solid 3D test cube (wood or metal, mass = 339.7 g, 5 cm edge) rests freely on the plate surface under gravity without any artificial rail or kinematic constraint.</li>
              <li><strong>Dynamic 3D Kinematics (180 Hz):</strong> Rather than resting at a static inclination, the plate undergoes high-speed 3D pitching and angular acceleration up to &sim;30&deg;, driven by dynamic robot motions and tracked by optical motion capture at 180 Hz (&Delta;<i>t</i> = 5.56 ms). This high-frequency rotation creates continuously changing 3D normal forces <b>N</b>(<i>t</i>) and multi-axis tangential shear forces.</li>
              <li><strong>3D Stick-Slip Friction Transition:</strong> The block is governed by 3D Coulomb friction at the plate-block contact interface. When the 3D inertial and gravity force vectors exceed the static friction cone limit, the block transitions from sticking to 3D sliding.</li>
              <li><strong>Sim-to-Real Prediction Target:</strong> Simulators replay the identical 6-DoF 3D rigid trajectory of the board, and must predict the unconstrained 3D motion trajectory [<i>x</i>(<i>t</i>), <i>y</i>(<i>t</i>), <i>z</i>(<i>t</i>)] of the sliding block. Accuracy is evaluated against the ground-truth 3D Mocap trajectory (reporting 3D position RMSE in mm and normalized nRMSE).</li>
              <li><strong>Why Task-3 (Metal) Is Extremely Demanding:</strong> <code>task-2 wood</code> has higher surface friction (<i>&mu;</i> &approx; 0.45) providing forgiving traction margin. In contrast, <code>task-3 metal</code> has low friction (<i>&mu;</i> &approx; 0.26) combined with heavy block mass (340 g). If normal contact compliance is too soft or reacts slower than the 180 Hz tilting motion (<i>&tau;<sub>n</sub></i> &gt; 5.56 ms), 3D normal support lags behind, causing instantaneous friction collapse and runaway 3D drift (577.7 mm).</li>
            </ul>
          </div>"""

import re
text = re.sub(r'<div class="task-setup-box">.*?</div>', clean_task_setup, text, flags=re.DOTALL)

with open("docs/mechanism-eval.html", "w", encoding="utf-8") as f:
    f.write(text)

print("Updated task setup. Checking '$' occurrences in mechanism-eval.html:")
with open("docs/mechanism-eval.html", "r", encoding="utf-8") as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if "$" in line:
        print(f"L{i+1}: {line.strip()[:80]}")
