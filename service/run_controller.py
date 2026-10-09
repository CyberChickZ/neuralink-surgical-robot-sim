"""Launcher for an engineer's controller: applies the scenario's initial robot state, then runs the controller
script with neuro_sdk importable. Usage: python3 run_controller.py [controller.py]  (no script: initial state only)   (NEURO_SCENARIO = scenario file)
"""
import runpy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gz_demo"))
from neuro_sdk import Robot, scenario  # noqa: E402

init = scenario().get("initial", {}).get("joints", {"lift_y": 0.235})
rb = Robot(name="scenario_init")
rb.move_joints({**{j: 0.0 for j in ("arm_roll_z", "wrist_pitch_x", "head_rot", "stage_x", "stage_z", "twinz_insert")}, **init})
rb.event(f"initial state applied: {init}")
rb.n.destroy_node()
if len(sys.argv) > 1:
    runpy.run_path(sys.argv[1], run_name="__main__")
