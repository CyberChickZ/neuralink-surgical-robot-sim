"""Real-time SOFA brain node: FEM tissue block under the craniotomy, live in the ROS graph.
Physics (SI, world frame): corotational TetrahedronFEM E=3 kPa nu=0.45 (Bilger 2014), side wall fixed (skull rim),
bottom pressure = calibrated pulsation (cardiac + respiratory from config.yaml), needle contact as a penalty on the
top nodes under the tip -> dimple; puncture when the normal force exceeds F_PUNCT -> tissue relaxes, needle inside.
Vessels (texture GT) ride on the FEM via BarycentricMapping -> live world positions for hit tests and camera GT.
Same topics as brain_node.py plus:
  /brain/needle_force   std_msgs/Float64  N, normal tissue force on the needle (0 after puncture)
  /brain/xsection_xz, /brain/xsection_yz  sensor_msgs/Image rgb8, two perpendicular side views through the needle tip
"""
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
import Sofa
import Sofa.Simulation
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String

from brain_node import CONTACT_TOL, MO, TIP_R, BrainNode, amp
from neuro_kin import GZ

DT = 0.02             # s, SOFA step (50 Hz, one step per state tick)
K_CONTACT = 40.0      # N/m per top node under the tip (penalty)
R_CONTACT = 0.0035    # m, contact footprint radius (>= top-surface node spacing ~3 mm, else the needle slips between nodes)
F_PUNCT = float(MO.get("puncture_force_n", 0.004))   # N, puncture threshold (scenario: brain.puncture_force_n)
DEPTH_MAX = 0.012     # m, tip deeper than this below the cortex is outside the tissue block (e.g. robot parked low)


class SofaBrainNode(BrainNode):
    def __init__(self):
        M = np.load(GZ / "block_mesh.npz")
        self.X0, self.top, self.R_block = M["nodes"], M["top"], float(M["radius"])
        self.root = Sofa.Core.Node("root"); self._build(M)
        Sofa.Simulation.initRoot(self.root)
        self.gain = self._calibrate()
        super().__init__()   # loads vessels, kin, publishers, timers
        self.vp0 = self.vmo.position.array().copy()
        self.punctured, self.force, self.peak = False, 0.0, 0.0
        self.sim_t_sofa = None
        self.p_force = self.n.create_publisher(Float64, "/brain/needle_force", 10)
        self.p_xz = self.n.create_publisher(Image, "/brain/xsection_xz", 2)
        self.p_yz = self.n.create_publisher(Image, "/brain/xsection_yz", 2)
        print(f"[sofa_brain] nodes={len(self.X0)} gain={self.gain:.3g} Pa/m F_punct={F_PUNCT} N", flush=True)

    def _build(self, M):
        r = self.root; r.dt = DT; r.gravity = [0, 0, 0]
        r.addObject("RequiredPlugin", pluginName=[
            "Sofa.Component.Constraint.Projective", "Sofa.Component.LinearSolver.Direct", "Sofa.Component.LinearSolver.Iterative", "Sofa.Component.Mapping.Linear",
            "Sofa.Component.Mass", "Sofa.Component.MechanicalLoad", "Sofa.Component.ODESolver.Backward",
            "Sofa.Component.SolidMechanics.FEM.Elastic", "Sofa.Component.StateContainer",
            "Sofa.Component.Topology.Container.Constant", "Sofa.Component.Topology.Container.Dynamic"])
        r.addObject("DefaultAnimationLoop")
        b = r.addChild("Brain")
        b.addObject("EulerImplicitSolver", rayleighStiffness=0.1, rayleighMass=0.1)
        b.addObject("CGLinearSolver", iterations=25, tolerance=1e-7, threshold=1e-9)   # SparseLDL refactorisation: ~35 ms/step
        b.addObject("TetrahedronSetTopologyContainer", name="topo", position=M["nodes"].tolist(), tetrahedra=M["tets"].tolist())
        self.dofs = b.addObject("MechanicalObject", name="dofs", template="Vec3d", position=M["nodes"].tolist())
        b.addObject("MeshMatrixMass", massDensity=1000.0, topology="@topo")
        b.addObject("TetrahedronFEMForceField", method="large", youngModulus=3000.0, poissonRatio=0.45)
        b.addObject("FixedProjectiveConstraint", indices=M["side"].tolist())
        self.needle_ff = b.addObject("ConstantForceField", name="needle", indices=[int(self.top[0])], forces=[[0, 0, 0]])
        bot = b.addChild("Bottom")
        bot.addObject("MeshTopology", position=M["nodes"].tolist(), triangles=M["bottom_tris"].tolist())
        bot.addObject("MechanicalObject", template="Vec3d")
        self.press = bot.addObject("SurfacePressureForceField", pressure=0.0)
        bot.addObject("IdentityMapping")
        v = b.addChild("Vessels")
        self.vmo = v.addObject("MechanicalObject", template="Vec3d", position=np.load(GZ / "vessels.npz")["xyz_world"].tolist())
        v.addObject("BarycentricMapping")
        self.centre = int(np.argmin(np.linalg.norm(M["nodes"][M["top"]][:, :2], axis=1)))
        self.centre = int(M["top"][self.centre])

    def _calibrate(self):
        cache = GZ / "sofa_gain.npy"
        if cache.exists():
            return float(np.load(cache))
        z0 = self.dofs.position.array()[self.centre, 2]
        self.press.pressure.value = 10.0
        for _ in range(int(3.0 / DT)):
            Sofa.Simulation.animate(self.root, DT)
        g = 10.0 / (self.dofs.position.array()[self.centre, 2] - z0)
        Sofa.Simulation.reset(self.root); np.save(cache, g)
        return g

    # ---- live geometry used by BrainNode (hit tests, GT masks, maps)
    def vessels_now(self, t):
        return self.vmo.position.array().copy()

    def surface_z(self, xy, t):
        P = self.dofs.position.array()[self.top]
        return float(P[np.argmin(np.sum((P[:, :2] - xy) ** 2, axis=1)), 2])

    def step(self):
        if not self.q or not rclpy.ok():
            return
        t0 = time.time(); t = self.now()
        tip = self.kin.fk("needle_tip", self.q)[:3, 3]
        # advance SOFA to sim time (<= 3 sub-steps per tick)
        if self.sim_t_sofa is None:
            self.sim_t_sofa = t
        if t - self.sim_t_sofa >= DT:                    # one FEM step per tick; if behind, drop the backlog
            self.press.pressure.value = self.gain * amp(t)
            self._needle_force(tip)
            Sofa.Simulation.animate(self.root, DT); self.sim_t_sofa = t
        zs = self.surface_z(tip[:2], t); depth = zs - tip[2]
        if depth > DEPTH_MAX or np.linalg.norm(tip[:2]) > self.R_block:   # not over / in the tissue block
            self.punctured, self.force = False, 0.0
        if self.punctured and depth < -CONTACT_TOL:
            self.punctured = False                       # needle withdrawn above the surface
        self.peak = max(self.peak, self.force) if (self.force > 0 or self.punctured) else 0.0   # peak of the current contact
        if not self.punctured and self.force >= F_PUNCT * (1 - 1e-9):
            self.punctured = True; self.force = 0.0
            self.p_burst.publish(String(data=json.dumps({"t": t, "event": "puncture", "tip_world": tip.round(6).tolist()})))
        state = ("off_tissue" if depth > DEPTH_MAX or np.linalg.norm(tip[:2]) > self.R_block else
                 "inserted" if self.punctured and depth > 0 else ("contact" if self.force > 0 else "above"))
        vp = self.vessels_now(t)
        dh = np.linalg.norm(vp[:, :2] - tip[:2], axis=1) - self.vr - TIP_R
        k = int(np.argmin(dh))
        if state == "inserted" and dh[k] < 0 and int(self.vid[k]) not in self.burst:
            vid = int(self.vid[k]); self.burst[vid] = {"t": t}
            msg = {"t": t, "event": "vessel_hit", "vessel_id": vid, "tip_world": tip.round(6).tolist(), "depth_m": round(depth, 6),
                   "vessel_radius_m": float(self.vr[k])}
            self.p_burst.publish(String(data=json.dumps(msg))); self.n.get_logger().warn(f"VESSEL HIT {msg}")
        self.state = state
        self.p_force.publish(Float64(data=self.force))
        self.p_state.publish(String(data=json.dumps({
            "t": round(t, 4), "tip_world": tip.round(6).tolist(), "surface_z": round(zs, 6), "depth_m": round(depth, 6),
            "state": state, "punctured": self.punctured, "needle_force_n": round(self.force, 6), "contact_peak_force_n": round(self.peak, 6),
            "nearest_vessel_id": int(self.vid[k]), "nearest_vessel_clearance_m": round(float(dh[k]), 6),
            "surface_dz_center_m": round(float(self.dofs.position.array()[self.centre, 2] - self.X0[self.centre, 2]), 7),
            "n_burst": len(self.burst)})))
        self._prof("step", t0)

    def _needle_force(self, tip):
        """Penalty on top nodes under the tip that are above it (tip pressing into the cortex); none after puncture."""
        if self.punctured or np.linalg.norm(tip[:2]) > self.R_block:
            self.force = 0.0; self.needle_ff.forces.value = [[0, 0, 0]]; return
        P = self.dofs.position.array()
        ids = self.top[np.linalg.norm(P[self.top, :2] - tip[:2], axis=1) < R_CONTACT]
        pen = P[ids, 2] - tip[2]
        sel = ids[(pen > 0) & (pen < DEPTH_MAX)]
        if len(sel) == 0:
            self.force = 0.0; self.needle_ff.indices.value = [int(self.top[0])]; self.needle_ff.forces.value = [[0, 0, 0]]; return
        f = -K_CONTACT * (P[sel, 2] - tip[2])
        f *= min(1.0, F_PUNCT / float(-f.sum()))         # tissue tears at F_PUNCT: a fast needle (mm per step) never loads it more
        self.force = float(-f.sum())
        self.needle_ff.indices.value = sel.tolist()
        self.needle_ff.forces.value = np.column_stack([np.zeros(len(sel)), np.zeros(len(sel)), f]).tolist()

    def publish_images(self):
        super().publish_images()
        if not rclpy.ok() or self.sim_stamp is None or not self.q:
            return
        tip = self.kin.fk("needle_tip", self.q)[:3, 3]
        self.p_xz.publish(self.img(self._slice(tip, 0), "rgb8"))
        self.p_yz.publish(self.img(self._slice(tip, 1), "rgb8"))

    def _slice(self, tip, ax):
        """Side view through the needle tip: ax=0 -> x-z plane (y = tip.y), ax=1 -> y-z plane (x = tip.x).
        FEM nodes within 2 mm of the plane coloured by vertical displacement, vessels within 0.5 mm, needle, state."""
        W, Hh, half = 480, 240, 0.024
        img = np.full((Hh, W, 3), 30, np.uint8)
        s = W / (2 * half); z_mid = self.X0[:, 2].mean(); h = ax          # horizontal coordinate index in the plane
        to_px = lambda u, z: (int(W / 2 + (u - tip[h]) * s), int(Hh / 2 - (z - z_mid) * s))
        P = self.dofs.position.array(); sel = np.abs(P[:, 1 - ax] - tip[1 - ax]) < 0.002
        dz = (P[:, 2] - self.X0[:, 2]) * 1e6
        for p, d in zip(P[sel], dz[sel]):
            c = int(np.clip(128 + d / 4, 0, 255))   # +-512 um -> full colour scale
            cv2.circle(img, to_px(p[h], p[2]), 3, (c, 90, 255 - c), -1)
        vp = self.vessels_now(0.0); vs = np.abs(vp[:, 1 - ax] - tip[1 - ax]) < 0.0005
        burst = np.isin(self.vid, list(self.burst))
        for p, b in zip(vp[vs], burst[vs]):
            cv2.circle(img, to_px(p[h], p[2]), 2, (255, 255, 0) if b else (230, 30, 30), -1)
        cv2.line(img, to_px(tip[h], tip[2]), to_px(tip[h], tip[2] + 0.03), (220, 220, 220), 2)
        col = {"above": (0, 200, 0), "contact": (255, 170, 0), "inserted": (60, 60, 255), "off_tissue": (120, 120, 120)}[self.state]
        cv2.circle(img, to_px(tip[h], tip[2]), 4, col, -1)
        cv2.putText(img, f"{'x-z' if ax == 0 else 'y-z'} | {self.state} F={self.force * 1e3:.2f}mN hits={len(self.burst)}",
                    (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return img


def main():
    from rclpy.executors import ExternalShutdownException
    rclpy.init(); b = SofaBrainNode()
    try:
        rclpy.spin(b.n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
