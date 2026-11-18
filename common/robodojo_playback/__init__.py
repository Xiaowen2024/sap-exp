"""Official RoboDojo asset and playback reproduction pilot.

The engine-independent core lives here; each task's own modules (scene
preparation, evaluation, rendering) live next to the task in
``tasks/robot-playback/<task>/robodojo_playback/``. They are added to this
package's path, so ``python -m robodojo_playback.coin_eval`` and relative imports
such as ``from .laptop_prepare import ...`` work as before.
"""
from pathlib import Path as _Path

_TASKS = _Path(__file__).resolve().parents[2] / "tasks" / "robot-playback"
__path__ = list(__path__) + sorted(str(p) for p in _TASKS.glob("*/robodojo_playback") if p.is_dir())
