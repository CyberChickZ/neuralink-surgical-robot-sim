"""Remote-control check: from another node, connect to a live session, move to the opening, insert once, report."""
import socket

from neuro_sdk import Robot

rb = Robot(name="remote_probe")
s0 = rb.state()
rb.move_joints({"lift_y": 0.235})
rb.move_tip(0.0, -0.002)
r = rb.insert()
rb.retract()
print(f"[remote_probe] from {socket.gethostname()}: brain state t={s0['t']:.2f}s, insert -> punctured={r['punctured']} "
      f"depth={r['depth_m'] * 1e3:.2f}mm peak={r['peak_force_n'] * 1e3:.1f}mN hits={r['hits']}", flush=True)
rb.close()
