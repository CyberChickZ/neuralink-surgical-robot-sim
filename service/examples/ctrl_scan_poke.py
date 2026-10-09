"""Example controller for service/submit.sh: hover over the opening, then for each site of a 3x3 grid
(+-8 mm) move the head-internal XY stage, lower TwinZ to touch-down, poke 4 mm, retract.

Controller contract (what every uploaded controller gets):
  publish  std_msgs/Float64  /cmd/<joint>   joint position targets (rad or m):
           lift_y [0,0.25] arm_roll_z [-0.785,0.785] wrist_pitch_x [-0.524,0.524] head_rot [-0.524,0.524]
           stage_x [-0.0125,0.0125] stage_z [-0.0125,0.0125] twinz_insert [-0.025,0] (negative = needle down)
  publish  std_msgs/String   /demo/event    free-form markers (recorded in the bag)
  subscribe sensor_msgs/JointState /joint_states (~100 Hz), sensor_msgs/Image /cam/{east,south,north,southeast}/image (10 Hz)
Kinematics at default layout (needle vertical, other joints 0): tip_world = (stage_z, -stage_x, 1.04 + lift_y + twinz_insert);
brain surface at the opening centre (0, 0, 1.225).
"""
import time

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

JOINTS = ["lift_y", "arm_roll_z", "wrist_pitch_x", "head_rot", "stage_x", "stage_z", "twinz_insert"]
HOVER_LIFT = 0.20          # tip 15 mm above the brain surface
TOUCH, DEPTH = -0.015, -0.019


def main():
    rclpy.init()
    n = rclpy.create_node("ctrl_scan_poke")
    pub = {j: n.create_publisher(Float64, f"/cmd/{j}", 10) for j in JOINTS}
    ev = n.create_publisher(String, "/demo/event", 10)
    js = {}
    n.create_subscription(JointState, "/joint_states", lambda m: js.update(zip(m.name, m.position)), 10)

    def go(targets, settle=1.0, tol=5e-4):
        t0 = time.time()
        while time.time() - t0 < 10.0:
            for j, v in targets.items():
                pub[j].publish(Float64(data=v))
            rclpy.spin_once(n, timeout_sec=0.05)
            if js and all(abs(js.get(j, 1e9) - v) < tol for j, v in targets.items()) and time.time() - t0 > settle * 0.2:
                return True
        n.get_logger().warn(f"not reached: {targets} have { {j: round(js.get(j, float('nan')), 4) for j in targets} }")
        return False

    def mark(s):
        ev.publish(String(data=s)); n.get_logger().info(s)

    while not js:  # wait for the simulation
        rclpy.spin_once(n, timeout_sec=0.2)
    mark("start")
    go({j: 0.0 for j in JOINTS} | {"lift_y": HOVER_LIFT}, settle=3.0)
    k = 0
    for sx in (-0.008, 0.0, 0.008):
        for sz in (-0.008, 0.0, 0.008):
            go({"stage_x": sx, "stage_z": sz})
            go({"twinz_insert": TOUCH}); mark(f"touchdown site={k} stage=({sx:+.3f},{sz:+.3f})")
            go({"twinz_insert": DEPTH}, tol=3e-4); mark(f"inserted site={k}")
            go({"twinz_insert": 0.0}); mark(f"retracted site={k}")
            k += 1
    mark("done")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
