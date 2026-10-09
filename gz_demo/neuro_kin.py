"""Forward kinematics of the Gazebo robot (robot_gz.urdf) in WORLD frame (robot base at robot_xyz.npy).
fk(link, q) -> 4x4; q = {joint_name: value}. Camera links follow the gz convention: +x forward, +z up.
"""
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

GZ = Path(os.environ.get("NEURO_GZ", "/nfs/hpc/dgx2-2/zhanhaoc/neuro_demo/gz"))


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def _axis(a, t):
    a = a / np.linalg.norm(a); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


class Kin:
    def __init__(self, urdf=GZ / "robot_gz.urdf", base_xyz=None):
        root = ET.parse(urdf).getroot()
        self.J = {}
        for j in root.findall("joint"):
            o, ax = j.find("origin"), j.find("axis")
            M = np.eye(4); M[:3, :3] = _rpy(*map(float, o.get("rpy", "0 0 0").split())); M[:3, 3] = list(map(float, o.get("xyz", "0 0 0").split()))
            self.J[j.find("child").get("link")] = (j.get("name"), j.get("type"), j.find("parent").get("link"), M,
                                                    np.array(list(map(float, ax.get("xyz").split()))) if ax is not None else None)
        self.base = np.eye(4); self.base[:3, 3] = np.load(GZ / "robot_xyz.npy") if base_xyz is None else base_xyz
        self.cams = {s.get("name")[4:]: float(s.find("camera/horizontal_fov").text) for s in root.iter("sensor")}
        self.cam_wh = {s.get("name")[4:]: (int(s.find("camera/image/width").text), int(s.find("camera/image/height").text)) for s in root.iter("sensor")}

    def fk(self, link, q):
        if link not in self.J:
            return self.base.copy()
        name, typ, parent, M, ax = self.J[link]
        M = M.copy(); v = q.get(name, 0.0)
        if typ == "revolute":
            R = np.eye(4); R[:3, :3] = _axis(ax, v); M = M @ R
        elif typ == "prismatic":
            P = np.eye(4); P[:3, 3] = ax * v; M = M @ P
        return self.fk(parent, q) @ M
