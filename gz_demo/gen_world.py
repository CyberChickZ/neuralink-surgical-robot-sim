"""Build the Gazebo world + brain model from D3MIA assets.
Brain mm-coords -> metres; placed so the opening-center cortex vertex (probe) sits 15 mm below the needle tip.
Writes <out_dir>/world.sdf and <out_dir>/brain/{cortex,tissue,skull}.obj + .mtl (textured).
Brain pedestal stands at world (0,0); --brain_height = z of the cortex at the opening center.
Robot base is placed at --robot_xyz. Default: needle above the opening at zero pose (the two parallel pitch joints can only
PULL the tip back, so forward reach = zero pose + 12.5 mm stage; any set-back > 1.25 cm makes the opening unreachable)
(original layout = needle tip 15 mm above the opening with robot at (-0.025, 0.625, 0) relative to the brain).
Usage: python3 gen_world.py <out_dir> [--brain_height 1.225] [--robot_xyz -0.025 0.625 0]   (metres)
"""
import shutil
import sys
from pathlib import Path

import numpy as np

DATA = Path("/nfs/hpc/share/zhanhaoc/neuro_demo/third_party/SOFA-NeuroSim-Recorder/data")
import argparse
ap = argparse.ArgumentParser(); ap.add_argument("out")
ap.add_argument("--brain_height", type=float, default=1.025 + 0.20)
ap.add_argument("--robot_xyz", type=float, nargs=3, default=[-0.025, 0.625, 0.0]); A = ap.parse_args()
out = Path(A.out); (out / "brain").mkdir(parents=True, exist_ok=True)
S0 = np.array([l.split()[1:4] for l in open(DATA / "surface_full_decimated.obj") if l.startswith("v ")], float)
OPEN = np.load(out / "opening.npz")                                       # find_opening.py: the skull's craniotomy hole
probe_rest = S0[int(OPEN["center_idx"])] * 1e-3                           # metres, brain frame
# level the brain: PCA plane of the cortex within 10 mm of the opening center -> rotate its normal to +z
P = S0[np.linalg.norm(S0 - probe_rest * 1e3, axis=1) < 10.0] * 1e-3
n = np.linalg.svd(P - P.mean(0))[2][2]
n = n if n[2] > 0 else -n                                       # opening faces +z in the D3MIA frame
v, c = np.cross(n, [0, 0, 1.0]), float(n[2])
K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
ROT = np.eye(3) + K + K @ K / (1 + c)                           # rotation taking n to +z
offset = np.array([0.0, 0.0, A.brain_height]) - ROT @ probe_rest  # world = ROT @ p_brain + offset
roll, pitch, yaw = np.arctan2(ROT[2, 1], ROT[2, 2]), np.arcsin(-ROT[2, 0]), np.arctan2(ROT[1, 0], ROT[0, 0])
Vv = np.array([l.split()[1:4] for l in open(DATA / "volume_simplified.obj") if l.startswith("v ")], float) * 1e-3
pedestal_h = float((Vv @ ROT.T + offset)[:, 2].min())           # floor -> lowest point of the levelled brain
np.save(out / "brain_offset.npy", offset); np.save(out / "brain_rot.npy", ROT)
print(f"levelled: opening normal {n.round(3)} tilted {np.degrees(np.arccos(c)):.1f} deg -> +z")
np.save(out / "robot_xyz.npy", np.array(A.robot_xyz))

parts = {"cortex": ("surface_full_decimated.obj", "Kd 1 1 1\nmap_Kd texture_outpaint.png"),
         "tissue": ("volume_simplified.obj", "Kd 0.80 0.55 0.52"),
         "skull": ("surface_skull.obj", "Kd 0.85 0.82 0.74")}
for name, (src, mtl) in parts.items():
    if (out / "brain" / f"{name}.obj").exists():
        continue
    lines = [l for l in open(DATA / src) if not l.startswith(("mtllib", "usemtl"))]
    with open(out / "brain" / f"{name}.obj", "w") as f:
        f.write(f"mtllib {name}.mtl\nusemtl m_{name}\n"); f.writelines(lines)
    (out / "brain" / f"{name}.mtl").write_text(f"newmtl m_{name}\nKa 0.2 0.2 0.2\n{mtl}\nKs 0.3 0.3 0.3\nNs 30\nillum 2\n")
shutil.copy(DATA / "texture_outpaint.png", out / "brain" / "texture_outpaint.png")


def visual(name, dz=0.0):
    return (f'<visual name="{name}"><pose>0 0 {dz} 0 0 0</pose><geometry><mesh><uri>file://{out}/brain/{name}.obj</uri>'
            f'<scale>0.001 0.001 0.001</scale></mesh></geometry></visual>')


world = f"""<?xml version="1.0"?>
<sdf version="1.9"><world name="neuro">
  <physics name="1ms" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <gravity>0 0 -9.81</gravity>
  <scene><ambient>0.2 0.2 0.2 1</ambient><background>0.75 0.78 0.82 1</background><shadows>true</shadows></scene>
  <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 5 0 0 0</pose>
    <diffuse>0.5 0.5 0.5 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.4 0.3 -0.9</direction></light>
  <light type="point" name="surgical_lamp"><pose>0 0 {A.brain_height + 0.4} 0 0 0</pose>
    <diffuse>0.35 0.35 0.35 1</diffuse><attenuation><range>3</range><constant>1.0</constant><linear>0.05</linear></attenuation></light>
  <model name="ground"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
    <visual name="v"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
    <material><ambient>0.35 0.36 0.38 1</ambient><diffuse>0.35 0.36 0.38 1</diffuse></material></visual></link></model>
  <model name="table"><static>true</static><pose>0 0 {pedestal_h / 2} 0 0 0</pose><link name="l">
    <visual name="v"><geometry><cylinder><radius>0.09</radius><length>{pedestal_h}</length></cylinder></geometry>
    <material><ambient>0.2 0.2 0.22 1</ambient><diffuse>0.2 0.2 0.22 1</diffuse></material></visual></link></model>
  <model name="overview_cam"><static>true</static><pose>0.55 -0.45 1.55 0 0.36 2.2</pose><link name="l">
    <sensor name="overview" type="camera"><camera><horizontal_fov>1.0</horizontal_fov><image><width>640</width><height>360</height></image>
      <clip><near>0.02</near><far>10</far></clip></camera><always_on>1</always_on><update_rate>10</update_rate><topic>/cam/overview/image</topic></sensor>
  </link></model>
  <include><uri>model://neuro_robot</uri><name>robot</name><pose>{A.robot_xyz[0]} {A.robot_xyz[1]} {A.robot_xyz[2]} 0 0 0</pose></include>
  <model name="brain"><static>true</static><pose>{offset[0]} {offset[1]} {offset[2]} {roll} {pitch} {yaw}</pose><link name="l">
    {visual("tissue")}{visual("cortex", 0.0002)}{visual("skull", 0.0004)}
  </link></model>
</world></sdf>
"""
(out / "world.sdf").write_text(world)
print(f"wrote {out}/world.sdf  opening center (0,0,{A.brain_height}), pedestal h={pedestal_h:.3f}, robot at {A.robot_xyz}")
