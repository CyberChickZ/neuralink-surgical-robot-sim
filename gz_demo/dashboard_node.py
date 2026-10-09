"""Live dashboard (one window, 1280x720, 10 fps), published on /demo/dashboard and saved as JPEG frames to <out>/dash/
(report.py encodes <out>/dashboard.mp4):
  top-left    robot overview (world-fixed camera)            top-right  head-hub camera NEURO_DASH_CAM (default c0)
                                                                         + vessel GT overlay (green; hit vessel = red)
  bottom-left SOFA side view x-z through the needle tip      bottom-right SOFA side view y-z through the needle tip
  status bar  state, depth, force, nearest-vessel clearance, insertions, VESSEL HITS (+ red frame flash on a hit)
Usage: python3 dashboard_node.py <out_dir>
"""
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from std_msgs.msg import String

W, H = 1280, 720
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp")
CAM = os.environ.get("NEURO_DASH_CAM", "c0")


def to_np(m):
    return np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)


def fit(img, w, h):
    """Letterbox `img` into a (h, w) black tile."""
    tile = np.zeros((h, w, 3), np.uint8)
    if img is None:
        return tile
    s = min(w / img.shape[1], h / img.shape[0]); nw, nh = int(img.shape[1] * s), int(img.shape[0] * s)
    tile[(h - nh) // 2:(h - nh) // 2 + nh, (w - nw) // 2:(w - nw) // 2 + nw] = cv2.resize(img[..., :3], (nw, nh))
    return tile


class Dashboard:
    def __init__(self):
        self.n = rclpy.create_node("dashboard")
        self.img = {}
        for t in ["/cam/overview/image", f"/cam/{CAM}/image", f"/cam/{CAM}/vessel_mask", "/brain/xsection_xz", "/brain/xsection_yz"]:
            self.n.create_subscription(Image, t, lambda m, t=t: self.img.__setitem__(t, to_np(m)), 2)
        self.state, self.flash, self.last_hit = {}, 0.0, ""
        self.n_ins, self.n_hits, self.last_state = 0, 0, "above"
        self.n.create_subscription(String, "/brain/needle_state", self.on_state, 10)
        self.n.create_subscription(String, "/brain/vessel_burst", self.on_burst, 10)
        self.pub = self.n.create_publisher(Image, "/demo/dashboard", 2)
        (OUT / "dash").mkdir(parents=True, exist_ok=True); self.k = 0
        self.n.create_timer(0.1, self.render)

    def on_state(self, m):
        s = json.loads(m.data)
        if s["state"] == "inserted" and self.last_state != "inserted":
            self.n_ins += 1
        self.last_state = s["state"]; self.state = s

    def on_burst(self, m):
        e = json.loads(m.data)
        if e.get("event", "vessel_hit") == "vessel_hit":
            self.n_hits += 1; self.flash = time.time() + 1.5
            self.last_hit = f"HIT vessel #{e['vessel_id']} (r={e['vessel_radius_m'] * 1e6:.0f}um) at t={e['t']:.2f}s"

    def cam(self):
        im = self.img.get(f"/cam/{CAM}/image"); mk = self.img.get(f"/cam/{CAM}/vessel_mask")
        if im is None:
            return None
        o = im[..., :3].copy()
        if mk is not None and mk.shape[:2] == o.shape[:2]:
            m = mk[..., 0]
            o[m == 255] = (0.55 * o[m == 255] + [0, 115, 0]).astype(np.uint8)
            o[m == 128] = (0.3 * o[m == 128] + [180, 0, 0]).astype(np.uint8)
        return o

    def render(self):
        if not rclpy.ok():
            return
        f = np.full((H, W, 3), 18, np.uint8)
        f[0:340, 0:640] = fit(self.img.get("/cam/overview/image"), 640, 340)
        f[0:340, 640:1280] = fit(self.cam(), 640, 340)
        f[340:660, 0:640] = fit(self.img.get("/brain/xsection_xz"), 640, 320)
        f[340:660, 640:1280] = fit(self.img.get("/brain/xsection_yz"), 640, 320)
        for x0, label in ((0, "robot"), (640, f"head camera {CAM} + vessel GT"), (0, None), (640, None)):
            if label:
                cv2.putText(f, label, (x0 + 8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        s = self.state
        bar = (f"t={s.get('t', 0):6.2f}s  {s.get('state', '-'):>10}  depth {s.get('depth_m', 0) * 1e3:+5.2f}mm  "
               f"F {s.get('needle_force_n', 0) * 1e3:5.2f}mN  clearance {s.get('nearest_vessel_clearance_m', 0) * 1e6:6.0f}um  "
               f"insertions {self.n_ins}")
        cv2.putText(f, bar, (10, 682), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)
        cv2.putText(f, f"VESSEL HITS: {self.n_hits}   {self.last_hit}", (10, 708), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 70, 70) if self.n_hits else (120, 220, 120), 1)
        if time.time() < self.flash:
            cv2.rectangle(f, (0, 0), (W - 1, H - 1), (255, 0, 0), 10)
        cv2.imwrite(str(OUT / "dash" / f"{self.k:06d}.jpg"), cv2.cvtColor(f, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85]); self.k += 1
        m = Image(); m.height, m.width, m.encoding, m.step = H, W, "rgb8", W * 3; m.data = f.tobytes()
        m.header.stamp = self.n.get_clock().now().to_msg(); self.pub.publish(m)


def main():
    rclpy.init(); d = Dashboard()
    try:
        rclpy.spin(d.n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
