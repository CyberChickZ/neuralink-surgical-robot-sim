"""robot_v1_fixed.urdf -> robot_gz.urdf for Gazebo Harmonic.
Adds: absolute mesh URIs, needle_tip frame, 3 camera links aimed at the needle tip, gz plugins
(JointPositionController per joint in velocity mode -> robust to the huge Fusion masses, JointStatePublisher).
Gravity is disabled on all links (position-controlled demo).
Usage: python3 gen_gz_urdf.py <robot_v1_fixed.urdf> <out.urdf>
"""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

src, out = Path(sys.argv[1]), Path(sys.argv[2])
tree = ET.parse(src)
r = tree.getroot()
for m in r.iter("mesh"):
    m.set("filename", "file://" + str((src.parent / m.get("filename")).resolve()))

for link in r.findall("link"):  # simple solid colours via gazebo material tags (user's choice)
    mat = "Gazebo/Black" if link.get("name") == "07_Needle" else "Gazebo/Grey"
    r.append(ET.fromstring(f'<gazebo reference="{link.get("name")}"><material>{mat}</material></gazebo>'))

# joint limit overrides requested for the demo (deg); the Fusion URDF itself is left unchanged
LIMIT_DEG = {"arm_roll_z": 90, "wrist_pitch_x": 80, "head_rot": 45}
for j in r.findall("joint"):
    if j.get("name") in LIMIT_DEG:
        a = math.radians(LIMIT_DEG[j.get("name")]); j.find("limit").set("lower", f"{-a:.6f}"); j.find("limit").set("upper", f"{a:.6f}")
    if j.get("name") == "twinz_insert":   # option A: head hovers ~5 cm above the brain, the needle extends to it
        j.find("limit").set("lower", "-0.070")

joints = [j.get("name") for j in r.findall("joint") if j.get("type") in ("revolute", "prismatic")]

# needle tip: 07_Needle mesh spans y in [-0.065, 0.025] of its link frame after the -90deg joint rotation;
# measured in world at zero pose: link origin z=1.105, mesh bottom z=1.04 -> tip is 0.065 m along the needle (-z world)
tip = ET.SubElement(r, "link", name="needle_tip")
ET.SubElement(ET.SubElement(tip, "inertial"), "mass", value="0.001")
tip.find("inertial").append(ET.fromstring('<inertia ixx="1e-8" iyy="1e-8" izz="1e-8" ixy="0" ixz="0" iyz="0"/>'))
j = ET.SubElement(r, "joint", name="needle_tip_fixed", type="fixed")
ET.SubElement(j, "parent", link="07_Needle"); ET.SubElement(j, "child", link="needle_tip")
ET.SubElement(j, "origin", xyz="0 -0.065 0.0025", rpy="0 0 0")  # needle-axis tip in 07_Needle frame (inspect_urdf)


def rpy_look(p, target):
    """rpy for a gz camera link whose +x looks from p to target (z up)."""
    d = np.asarray(target, float) - np.asarray(p, float); d /= np.linalg.norm(d)
    yaw = math.atan2(d[1], d[0]); pitch = -math.asin(d[2])
    return 0.0, pitch, yaw


# cameras fixed to the head shell (05_Wrist_B); positions given in WORLD at zero pose, converted to the link frame.
# 05_Wrist_B link frame at zero pose: origin world (-0.15,-0.55,1.2)+..., but simplest: attach to world-aligned helper
# via 06_StageX? No - cameras must follow the head, not the stage. Use 05_Wrist_B with its zero-pose world transform:
WB_R = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], float)  # 05_Wrist_B world rotation at zero pose (inspect_urdf)
WB_p = np.array([-0.15, -0.55, 1.2])                        # head_rot joint origin in world at zero pose
TIP = np.array([0.025, -0.625, 1.04])                       # needle tip world at zero pose (inspect_urdf)
# 4 RGB cameras in the head hub: on a 5 cm circle around the needle axis, in the head-base plane (shell bottom is
# ~1 cm above the tip at zero pose; cameras sit 2 mm below it), all aimed at the point 5 cm beyond the needle tip.
HUB_R, BASE_DZ = 0.05, 0.008
cams = {f"c{k}": TIP + [HUB_R * math.cos(math.radians(a)), HUB_R * math.sin(math.radians(a)), BASE_DZ]
        for k, a in enumerate((0, 90, 180, 270))}
AIM = TIP - np.array([0.0, 0.0, 0.05])
for name, pw in cams.items():
    pl = WB_R.T @ (np.asarray(pw) - WB_p)
    rw = rpy_look(pw, AIM)
    # orientation: world rpy -> link frame: R_link = WB_R^T R_world
    cr, sr, cp, sp, cy, sy = (math.cos(rw[0]), math.sin(rw[0]), math.cos(rw[1]), math.sin(rw[1]), math.cos(rw[2]), math.sin(rw[2]))
    Rw = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])
    Rl = WB_R.T @ Rw
    rl = (math.atan2(Rl[2, 1], Rl[2, 2]), math.asin(-Rl[2, 0]), math.atan2(Rl[1, 0], Rl[0, 0]))
    L = ET.SubElement(r, "link", name=f"cam_{name}")
    ET.SubElement(ET.SubElement(L, "inertial"), "mass", value="0.01")
    L.find("inertial").append(ET.fromstring('<inertia ixx="1e-6" iyy="1e-6" izz="1e-6" ixy="0" ixz="0" iyz="0"/>'))
    v = ET.SubElement(L, "visual"); ET.SubElement(v, "origin", xyz="-0.01 0 0", rpy="0 0 0")
    ET.SubElement(ET.SubElement(v, "geometry"), "box", size="0.02 0.015 0.015")
    J = ET.SubElement(r, "joint", name=f"cam_{name}_fixed", type="fixed")
    ET.SubElement(J, "parent", link="05_Wrist_B"); ET.SubElement(J, "child", link=f"cam_{name}")
    ET.SubElement(J, "origin", xyz=" ".join(f"{x:.5f}" for x in pl), rpy=" ".join(f"{x:.5f}" for x in rl))
    hfov = 1.0  # rad (~57 deg)
    r.append(ET.fromstring(f"""<gazebo reference="cam_{name}"><sensor name="cam_{name}" type="camera">
      <camera><horizontal_fov>{hfov}</horizontal_fov><image><width>320</width><height>240</height></image>
      <clip><near>0.005</near><far>5</far></clip></camera><always_on>1</always_on><update_rate>10</update_rate>
      <topic>/cam/{name}/image</topic></sensor></gazebo>"""))

for link in r.findall("link"):
    r.append(ET.fromstring(f'<gazebo reference="{link.get("name")}"><gravity>false</gravity></gazebo>'))
plug = ['<gazebo><plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">'
        '<topic>/joint_states</topic></plugin></gazebo>']
for jn in joints:
    plug.append(f'<gazebo><plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">'
                f'<joint_name>{jn}</joint_name><topic>/cmd/{jn}</topic><use_velocity_commands>true</use_velocity_commands>'
                f'<p_gain>{200 if jn in ("twinz_insert", "stage_x", "stage_z") else 8}</p_gain>'   # fine axes: high gain -> ~0.5 mm lag at 0.1 m/s
                f'<cmd_max>2.0</cmd_max><cmd_min>-2.0</cmd_min></plugin></gazebo>')
for p in plug:
    r.append(ET.fromstring(p))
ET.indent(tree)
tree.write(out)
print(f"wrote {out}: joints={joints}, cams={list(cams)}")
