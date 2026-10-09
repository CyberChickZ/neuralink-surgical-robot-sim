"""Keyboard teleop (run in a terminal, ROS2 Jazzy).
  1-5   select joint: 1 lift_y  2 arm_roll_z  3 wrist_pitch_x  4 head_rot  5 twinz_insert
  + / - move selected joint (rad or m step)
  arrows  head-internal XY stage (stage_x / stage_z)
  space   poke: TwinZ extends at NEURO_POKE_SPEED until the tip is 4 mm into the tissue (needle state from the
          brain node), holds, retracts at NEURO_RETRACT_SPEED back to where it started
  r       reset: all joints 0, lift 0.235 (needle tip ~5 cm above the brain); publishes 'reset' on /demo/event
  q       quit
Publishes std_msgs/Float64 on /cmd/<joint> (bridged to gz JointPositionController) and std_msgs/String on /demo/event.
"""
import select
import sys
import termios
import threading
import time
import tty

import json

import rclpy
from std_msgs.msg import Float64, String

JOINTS = ["lift_y", "arm_roll_z", "wrist_pitch_x", "head_rot", "twinz_insert", "stage_x", "stage_z"]
LIMITS = {"lift_y": (0.0, 0.25), "arm_roll_z": (-1.5708, 1.5708), "wrist_pitch_x": (-1.3963, 1.3963), 
          "head_rot": (-0.7854, 0.7854), "twinz_insert": (-0.070, 0.0), "stage_x": (-0.0125, 0.0125), "stage_z": (-0.0125, 0.0125)}
STEP = {"lift_y": 0.005, "arm_roll_z": 0.03, "wrist_pitch_x": 0.03, "head_rot": 0.03, "twinz_insert": 0.001,
        "stage_x": 0.0005, "stage_z": 0.0005}
POKE_STROKE = 0.004   # m, relative stroke from the current TwinZ position (tip is 15 mm above cortex at twinz=0)
POKE_HOLD = 0.15      # s at depth before retract
POKE_SPEED = float(__import__("os").environ.get("NEURO_POKE_SPEED", 0.1))     # m/s, needle insertion speed
RETRACT_SPEED = float(__import__("os").environ.get("NEURO_RETRACT_SPEED", 0.3))


def main():
    rclpy.init()
    node = rclpy.create_node("teleop_keys")
    pubs = {j: node.create_publisher(Float64, f"/cmd/{j}", 10) for j in JOINTS}
    ev = node.create_publisher(String, "/demo/event", 10)
    bstate = {}
    node.create_subscription(String, "/brain/needle_state", lambda m: bstate.update(json.loads(m.data)), 10)
    q = {j: 0.0 for j in JOINTS}
    q["lift_y"] = 0.235  # head hover: tip ~5 cm above the brain (surface 1.225 m; tip at lift 0 = 1.04 m)
    sel = "arm_roll_z"

    def send(j):
        q[j] = min(max(q[j], LIMITS[j][0]), LIMITS[j][1])
        pubs[j].publish(Float64(data=q[j]))

    def status(msg=""):
        s = "  ".join(f"{'*' if j == sel else ' '}{j}={q[j]:+.3f}" for j in JOINTS[:5])
        sys.stdout.write(f"\r\033[K{s}  stage=({q['stage_x']*1e3:+.1f},{q['stage_z']*1e3:+.1f})mm {msg}"); sys.stdout.flush()

    def ramp(z_from, z_to, speed):
        t0, d = time.time(), z_to - z_from
        T = abs(d) / speed
        while (t := time.time() - t0) < T:
            q["twinz_insert"] = z_from + d * t / T; send("twinz_insert"); time.sleep(0.002)
        q["twinz_insert"] = z_to; send("twinz_insert")

    def poke():
        start = q["twinz_insert"]; t0 = time.time(); ev.publish(String(data="poke"))
        while q["twinz_insert"] > LIMITS["twinz_insert"][0]:          # extend until 4 mm into tissue (or full travel)
            q["twinz_insert"] = start - POKE_SPEED * (time.time() - t0); send("twinz_insert"); time.sleep(0.002)
            if bstate.get("state") == "inserted" and bstate.get("depth_m", 0) >= POKE_STROKE:
                break
        time.sleep(POKE_HOLD)
        ramp(q["twinz_insert"], start, RETRACT_SPEED); ev.publish(String(data="retract"))

    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print(__doc__)
    try:
        for j in JOINTS:
            send(j)
        status()
        while rclpy.ok():
            if not select.select([sys.stdin], [], [], 0.05)[0]:
                rclpy.spin_once(node, timeout_sec=0); continue
            c = sys.stdin.read(1)
            if c == "\x1b":  # arrow keys: ESC [ A/B/C/D
                c += sys.stdin.read(2)
            keymap = {"1": "lift_y", "2": "arm_roll_z", "3": "wrist_pitch_x", "4": "head_rot", "5": "twinz_insert"}
            if c in keymap:
                sel = keymap[c]
            elif c in "+=":
                q[sel] += STEP[sel]; send(sel)
            elif c in "-_":
                q[sel] -= STEP[sel]; send(sel)
            elif c == "\x1b[A":
                q["stage_x"] += STEP["stage_x"]; send("stage_x")
            elif c == "\x1b[B":
                q["stage_x"] -= STEP["stage_x"]; send("stage_x")
            elif c == "\x1b[C":
                q["stage_z"] += STEP["stage_z"]; send("stage_z")
            elif c == "\x1b[D":
                q["stage_z"] -= STEP["stage_z"]; send("stage_z")
            elif c == " ":
                threading.Thread(target=poke, daemon=True).start()
            elif c in "rR":
                for j in JOINTS:
                    q[j] = 0.235 if j == "lift_y" else 0.0; send(j)
                ev.publish(String(data="reset"))
            elif c in "qQ":
                break
            status()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
