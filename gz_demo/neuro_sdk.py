"""neuro_sdk: the API an engineer's controller uses (wraps the ROS2 topics of the simulation; same calls can be
backed by the real robot's driver later).

    from neuro_sdk import Robot
    rb = Robot()                                  # connects, waits for the simulation
    rb.move_tip(x, y)                             # needle tip to (x, y), 4.5 cm above the cortex (head hover), needle vertical
    res = rb.insert(depth=0.004, speed=0.01)      # TwinZ down at `speed` until `depth` below the cortex
    rb.retract()
    V = rb.vessels()                              # (N,5): x, y, z, radius, id  (live, world frame, from SOFA)
    rb.event("my marker")

World frame: z up, craniotomy centre at (0, 0, ~1.225). Units m, s, N.
"""
import json
import math
import os
import time

import numpy as np
import rclpy
import yaml
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Float64, String

from neuro_kin import Kin

JOINTS = ["lift_y", "arm_roll_z", "wrist_pitch_x", "head_rot", "stage_x", "stage_z", "twinz_insert"]
LIMITS = {"lift_y": (0.0, 0.25), "arm_roll_z": (-1.5708, 1.5708), "wrist_pitch_x": (-1.3963, 1.3963), "head_rot": (-0.7854, 0.7854),
          "stage_x": (-0.0125, 0.0125), "stage_z": (-0.0125, 0.0125), "twinz_insert": (-0.070, 0.0)}


def scenario():
    """The scenario dict this run was started with (NEURO_SCENARIO), {} if none."""
    p = os.environ.get("NEURO_SCENARIO")
    return yaml.safe_load(open(p)) if p else {}


class Robot:
    def __init__(self, name="controller", timeout=60.0):
        if not rclpy.ok():
            rclpy.init()
        self.n = rclpy.create_node(name)
        self.kin = Kin()
        self.q, self._state, self._vessels, self.hits, self.cmd = {}, {}, None, [], {}
        self.pub = {j: self.n.create_publisher(Float64, f"/cmd/{j}", 10) for j in JOINTS}
        self.ev = self.n.create_publisher(String, "/demo/event", 10)
        self.n.create_subscription(JointState, "/joint_states", lambda m: self.q.update(zip(m.name, m.position)), 10)
        self.n.create_subscription(String, "/brain/needle_state", lambda m: self._state.update(json.loads(m.data)), 10)
        self.n.create_subscription(String, "/brain/vessel_burst", self._on_burst, 10)
        self.n.create_subscription(PointCloud2, "/brain/vessels", self._on_vessels, 2)
        t0 = time.time()
        while not (self.q and self._state and self._vessels is not None):
            self.spin(0.1)
            if time.time() - t0 > timeout:
                raise TimeoutError("simulation not reachable (joint_states / brain state / vessels missing)")
        self.cmd = {j: self.q[j] for j in JOINTS}

    # ---------------- state
    def spin(self, dt=0.0):
        t_end = time.time() + dt
        rclpy.spin_once(self.n, timeout_sec=0)
        while time.time() < t_end:
            rclpy.spin_once(self.n, timeout_sec=min(0.01, max(0.0, t_end - time.time())))

    def _on_burst(self, m):
        e = json.loads(m.data)
        if e.get("event", "vessel_hit") == "vessel_hit":
            self.hits.append(e)

    def _on_vessels(self, m):
        a = np.frombuffer(bytes(m.data), np.float32).reshape(-1, 6)
        self._vessels = a[:, :5].astype(float)

    def joints(self):
        return dict(self.q)

    def state(self):
        """Needle/tissue state: tip_world, surface_z, depth_m, state (above|contact|inserted|off_tissue), punctured,
        needle_force_n, nearest_vessel_id, nearest_vessel_clearance_m, n_burst."""
        return dict(self._state)

    def tip(self):
        return self.kin.fk("needle_tip", self.q)[:3, 3]

    def vessels(self):
        return self._vessels.copy()

    def event(self, text):
        self.ev.publish(String(data=text)); self.n.get_logger().info(text)

    # ---------------- motion
    def _send(self, targets):
        for j, v in targets.items():
            v = min(max(v, LIMITS[j][0]), LIMITS[j][1]); self.cmd[j] = v; self.pub[j].publish(Float64(data=v))

    def move_joints(self, targets, tol=3e-4, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._send(targets); self.spin(0.02)
            if all(abs(self.q.get(j, 1e9) - self.cmd[j]) < tol for j in targets):
                return True
        raise TimeoutError(f"move_joints not reached: {targets} have { {j: round(self.q.get(j, float('nan')), 4) for j in targets} }")

    def move_tip(self, x, y, hover=0.045, tol=1e-4):
        """Tip to (x, y) and `hover` m above the cortex there, needle vertical (rotations 0), via lift + XY stage.
        Solved with a numeric Jacobian on (stage_x, stage_z, lift_y) using the robot FK."""
        self.move_joints({"twinz_insert": 0.0, "arm_roll_z": 0.0, "wrist_pitch_x": 0.0, "head_rot": 0.0})
        zs = self._surface_estimate(x, y)
        target = np.array([x, y, zs + hover])
        js = ["stage_x", "stage_z", "lift_y"]; q = dict(self.q); q.update({"twinz_insert": 0.0})
        for _ in range(8):
            p = self.kin.fk("needle_tip", q)[:3, 3]; err = target - p
            if np.linalg.norm(err) < tol:
                break
            J = np.zeros((3, 3))
            for i, j in enumerate(js):
                dq = dict(q); dq[j] += 1e-4; J[:, i] = (self.kin.fk("needle_tip", dq)[:3, 3] - p) / 1e-4
            step = np.linalg.lstsq(J, err, rcond=None)[0]
            for i, j in enumerate(js):
                q[j] = min(max(q[j] + step[i], LIMITS[j][0]), LIMITS[j][1])
        reach = np.linalg.norm(self.kin.fk("needle_tip", q)[:3, 3][:2] - target[:2])
        if reach > 5e-4:
            raise ValueError(f"({x:.4f},{y:.4f}) outside the stage workspace (miss {reach * 1e3:.1f} mm)")
        self.move_joints({j: q[j] for j in js})

    def _surface_estimate(self, x, y):
        V = self._vessels
        if V is not None and len(V):
            i = np.argmin(np.hypot(V[:, 0] - x, V[:, 1] - y)); return float(V[i, 2])
        return float(self._state.get("surface_z", 1.225))

    def needle_speed(self):
        """(insert, retract) speeds in m/s from the scenario (needle.insert_speed / retract_speed), default 0.1 / 0.3."""
        n = scenario().get("needle", {})
        return float(n.get("insert_speed", 0.1)), float(n.get("retract_speed", 0.3))

    def insert(self, depth=0.004, speed=None, max_travel=0.069):
        """Lower TwinZ at `speed` (m/s; default = scenario needle.insert_speed) until the tip is `depth` below the cortex.
        Returns {'punctured', 'depth_m', 'peak_force_n', 'hits': [vessel ids hit during this insertion]}."""
        speed = speed or self.needle_speed()[0]
        n_hits0, peak = len(self.hits), 0.0
        z0 = self.cmd["twinz_insert"]; t0 = time.time()
        while True:
            z = z0 - speed * (time.time() - t0)
            if z0 - z > max_travel or z <= LIMITS["twinz_insert"][0]:
                break
            self._send({"twinz_insert": z}); self.spin(0.002)
            s = self._state; peak = max(peak, s.get("needle_force_n", 0.0), s.get("contact_peak_force_n", 0.0))
            if s.get("state") == "inserted" and s.get("depth_m", 0.0) >= depth:
                break
        self.spin(0.05)
        return {"punctured": self._state.get("punctured", False), "depth_m": self._state.get("depth_m"),
                "peak_force_n": peak, "hits": [h["vessel_id"] for h in self.hits[n_hits0:]]}

    def retract(self, speed=None):
        """Raise TwinZ back to 0 at `speed` (m/s; default = scenario needle.retract_speed)."""
        speed = speed or self.needle_speed()[1]
        z0 = self.cmd["twinz_insert"]; t0 = time.time()
        while True:
            z = min(0.0, z0 + speed * (time.time() - t0)); self._send({"twinz_insert": z}); self.spin(0.002)
            if z >= 0.0:
                break
        self.move_joints({"twinz_insert": 0.0})

    def close(self):
        self.n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
