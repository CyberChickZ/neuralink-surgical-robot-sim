"""Brain simulation node (simplest version; SOFA FEM parked).
- Pulsation: analytic surface displacement dz(r,t) = A(t) * cos^2(pi r / 2R0) inside the R0=12.5 mm opening (rim fixed),
  A(t) = card + resp from config.yaml (literature amplitudes). Applied to vessels and to the cortex height.
- Vessels: texture-derived centrelines (extract_vessels.py, world frame) + radius; they move with the pulsation.
- Needle: tip from FK(/joint_states); state above/contact/inserted, depth below the (pulsating) cortex.
- Hit: inserted and horizontal distance tip->vessel centreline < radius + TIP_R  -> vessel bursts (until 'reset').
Publishes (stamped with sim time):
  /brain/needle_state  std_msgs/String JSON  50 Hz
  /brain/vessel_burst  std_msgs/String JSON  on event
  /brain/vessels       sensor_msgs/PointCloud2 (x,y,z,radius,id,burst) 10 Hz, frame "world"
  /brain/vessel_map    sensor_msgs/Image rgb8 400x400 top view (+-14 mm), 10 Hz
  /cam/<name>/vessel_mask sensor_msgs/Image mono8, same size/intrinsics/pose as /cam/<name>/image, 10 Hz
Subscribes /joint_states, /demo/event ('reset' clears bursts).
"""
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, JointState, PointCloud2, PointField
from std_msgs.msg import String

from neuro_kin import GZ, Kin

CFG = yaml.safe_load(open(Path(__file__).resolve().parents[1] / "config.yaml"))["brain"]
MO = dict(CFG["motion"])
import os as _os
if _os.environ.get("NEURO_SCENARIO"):  # scenario overrides (pulsation amplitude/frequency, ...)
    MO.update((yaml.safe_load(open(_os.environ["NEURO_SCENARIO"])) or {}).get("brain", {}))
R0 = float(np.load(GZ / "opening.npz")["radius_m"])   # craniotomy radius (m), find_opening.py
TIP_R = 12e-6         # needle tip radius (24 um etched needle)
CONTACT_TOL = 2e-4    # |tip - surface| below this = contact
OC = np.array([0.0, 0.0, 0.0])  # opening centre (world xy); z from cortex


def amp(t):
    return (0.5 * MO["amp_card_pp"] * math.sin(2 * math.pi * MO["f_card"] * t)
            + 0.5 * MO["amp_resp_pp"] * math.sin(2 * math.pi * MO["f_resp"] * t))


def weight(xy):
    r = np.linalg.norm(xy - OC[:2], axis=-1)
    return np.where(r < R0, np.cos(np.pi * r / (2 * R0)) ** 2, 0.0)


class BrainNode:
    def __init__(self):
        # NOT use_sim_time: subscribing /clock (published every physics step) saturates the Python executor.
        # Sim time is taken from the /joint_states header stamps (gz sim time, ~100 Hz) instead.
        self.n = rclpy.create_node("brain_node")
        self.sim_stamp = None
        V = np.load(GZ / "vessels.npz")
        self.vp0, self.vr, self.vid = V["xyz_world"], V["radius"], V["vessel_id"]
        self.S = V["cortex_world"]                    # cortex points around the opening (world, levelled), precomputed
        self.kin = Kin()
        self.q, self.burst = {}, {}
        self.state = "above"
        self.n.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.n.create_subscription(String, "/demo/event", self.on_event, 10)
        self.p_state = self.n.create_publisher(String, "/brain/needle_state", 10)
        self.p_burst = self.n.create_publisher(String, "/brain/vessel_burst", 10)
        self.p_cloud = self.n.create_publisher(PointCloud2, "/brain/vessels", 2)
        self.p_map = self.n.create_publisher(Image, "/brain/vessel_map", 2)
        self.p_mask = {c: self.n.create_publisher(Image, f"/cam/{c}/vessel_mask", 2) for c in self.kin.cams}
        self.n.create_timer(0.02, self.step)   # 50 Hz state / hits
        self.n.create_timer(0.1, self.publish_images)   # 10 Hz images
        self.prof = {"step": [0, 0.0], "img": [0, 0.0]}; self.prof_t = time.time()
        print("[brain_node] ready", flush=True)

    def _prof(self, k, t0):
        c = self.prof[k]; c[0] += 1; c[1] += time.time() - t0
        if time.time() - self.prof_t > 2.0:
            print("[brain_node] prof " + " ".join(f"{a}: n={n} mean={tt / max(n, 1) * 1e3:.1f}ms" for a, (n, tt) in self.prof.items()), flush=True)
            self.prof_t = time.time(); self.prof = {"step": [0, 0.0], "img": [0, 0.0]}

    def on_js(self, m):
        self.q.update(zip(m.name, m.position)); self.sim_stamp = m.header.stamp

    def now(self):  # simulation time (s)
        return self.sim_stamp.sec + self.sim_stamp.nanosec * 1e-9 if self.sim_stamp else 0.0

    def on_event(self, m):
        if m.data == "reset":
            self.burst.clear(); self.n.get_logger().info("reset: bursts cleared")

    def vessels_now(self, t):
        p = self.vp0.copy(); p[:, 2] += amp(t) * weight(p[:, :2]); return p

    def surface_z(self, xy, t):
        i = np.argmin(np.sum((self.S[:, :2] - xy) ** 2, axis=1))
        return self.S[i, 2] + amp(t) * float(weight(xy))

    def step(self):
        if not self.q or not rclpy.ok():
            return
        t0 = time.time()
        t = self.now()
        tip = self.kin.fk("needle_tip", self.q)[:3, 3]
        zs = self.surface_z(tip[:2], t)
        depth = zs - tip[2]                       # >0: tip below the cortex
        state = "inserted" if depth > CONTACT_TOL else ("contact" if depth > -CONTACT_TOL else "above")
        vp = self.vessels_now(t)
        dh = np.linalg.norm(vp[:, :2] - tip[:2], axis=1) - self.vr - TIP_R   # edge clearance (horizontal)
        k = int(np.argmin(dh))
        if state == "inserted" and dh[k] < 0:
            vid = int(self.vid[k])
            if vid not in self.burst:
                self.burst[vid] = {"t": t, "tip": tip.tolist(), "depth_m": depth}
                msg = {"t": t, "vessel_id": vid, "tip_world": tip.round(6).tolist(), "depth_m": round(depth, 6),
                       "vessel_radius_m": float(self.vr[k])}
                self.p_burst.publish(String(data=json.dumps(msg))); self.n.get_logger().warn(f"VESSEL BURST {msg}")
        self.state = state
        self.p_state.publish(String(data=json.dumps({
            "t": round(t, 4), "tip_world": tip.round(6).tolist(), "surface_z": round(zs, 6), "depth_m": round(depth, 6),
            "state": state, "nearest_vessel_id": int(self.vid[k]), "nearest_vessel_clearance_m": round(float(dh[k]), 6),
            "surface_dz_center_m": round(amp(t), 7), "n_burst": len(self.burst)})))
        self._prof("step", t0)

    @staticmethod
    def draw(canvas, u, v, r, val):
        """Disks at (u,v) radius r px. r<1.5 -> single pixels in one numpy write; larger -> cv2.circle."""
        H, W = canvas.shape[:2]
        ui, vi = np.round(u).astype(int), np.round(v).astype(int)
        small = r < 1.5
        ok = small & (ui >= 0) & (ui < W) & (vi >= 0) & (vi < H)
        canvas[vi[ok], ui[ok]] = val[ok] if np.ndim(val) and len(val) == len(u) else val
        for k in np.where(~small & (ui > -50) & (ui < W + 50) & (vi > -50) & (vi < H + 50))[0]:
            c = val[k] if np.ndim(val) and len(val) == len(u) else val
            cv2.circle(canvas, (int(ui[k]), int(vi[k])), int(round(r[k])), tuple(int(x) for x in np.atleast_1d(c)) if canvas.ndim == 3 else int(c), -1)

    def img(self, arr, enc):
        m = Image(); m.header.stamp = self.sim_stamp; m.header.frame_id = "world"
        m.height, m.width = arr.shape[:2]; m.encoding = enc; m.step = arr.strides[0]; m.data = arr.tobytes(); return m

    def publish_images(self):
        if not rclpy.ok() or self.sim_stamp is None:
            return
        t0 = time.time()
        t = self.now(); vp = self.vessels_now(t)
        burst = np.isin(self.vid, list(self.burst))
        # point cloud
        rec = np.zeros(len(vp), dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("radius", "f4"), ("id", "f4"), ("burst", "f4")])
        rec["x"], rec["y"], rec["z"], rec["radius"], rec["id"], rec["burst"] = vp[:, 0], vp[:, 1], vp[:, 2], self.vr, self.vid, burst
        pc = PointCloud2(); pc.header.stamp = self.sim_stamp; pc.header.frame_id = "world"
        pc.height, pc.width, pc.is_dense, pc.is_bigendian = 1, len(rec), True, False
        pc.fields = [PointField(name=nm, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, nm in enumerate(rec.dtype.names)]
        pc.point_step, pc.row_step, pc.data = 24, 24 * len(rec), rec.tobytes()
        self.p_cloud.publish(pc)
        # top-down vessel map
        N, half = 400, R0 + 0.002; s = N / (2 * half)
        mp = np.full((N, N, 3), 235, np.uint8)
        cv2.circle(mp, (N // 2, N // 2), int(R0 * s), (180, 180, 180), 1)
        cols = np.where(burst[:, None], np.array([220, 0, 0], np.uint8), np.array([120, 20, 30], np.uint8))
        self.draw(mp, N / 2 + vp[:, 0] * s, N / 2 - vp[:, 1] * s, self.vr * s, cols)
        if self.q:
            tip = self.kin.fk("needle_tip", self.q)[:3, 3]; c = (int(N / 2 + tip[0] * s), int(N / 2 - tip[1] * s))
            col = {"above": (0, 150, 0), "contact": (230, 150, 0), "inserted": (0, 0, 230), "off_tissue": (120, 120, 120)}[self.state]
            cv2.drawMarker(mp, c, col, cv2.MARKER_CROSS, 18, 2)
        cv2.putText(mp, f"{self.state}  bursts={len(self.burst)}  dz={amp(t) * 1e6:+.0f}um", (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        self.p_map.publish(self.img(mp, "rgb8"))
        # per-camera vessel GT masks (pinhole, gz camera: +x forward, +z up), no occlusion test
        if not self.q:
            return
        for c, hfov in self.kin.cams.items():
            W, H = self.kin.cam_wh[c]; f = (W / 2) / math.tan(hfov / 2)
            M = self.kin.fk(f"cam_{c}", self.q); pc_ = (vp - M[:3, 3]) @ M[:3, :3]
            ok = pc_[:, 0] > 1e-3
            u = W / 2 - f * pc_[ok, 1] / pc_[ok, 0]; v = H / 2 - f * pc_[ok, 2] / pc_[ok, 0]; rr = f * self.vr[ok] / pc_[ok, 0]
            mask = np.zeros((H, W), np.uint8)
            self.draw(mask, u, v, rr, np.where(burst[ok], 128, 255).astype(np.uint8))
            self.p_mask[c].publish(self.img(mask, "mono8"))
        self._prof("img", t0)


def main():
    rclpy.init(); b = BrainNode()
    try:
        rclpy.spin(b.n)   # single-threaded: rclpy's MultiThreadedExecutor pegged a core with GIL contention
    except (KeyboardInterrupt, ExternalShutdownException):  # normal stop when the job ends
        pass


if __name__ == "__main__":
    main()
