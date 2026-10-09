# Interface reference: topics, frames, multi-user setup

Robot (Fusion URDF `robot_v1/robot_v1_fixed.urdf`) + textured brain phantom (D3MIA assets) in Gazebo, controlled over ROS2.
Runs on any cluster GPU node (A40 or H100: Gazebo renders with OpenGL/EGL, no RT cores needed).

## Versions (pinned)
| component | version | where |
|---|---|---|
| ROS2 | Jazzy (osrf/ros:jazzy-desktop-full) | `/nfs/hpc/dgx2-2/zhanhaoc/neuro_demo/ros/ros_jazzy_desktop_full.sif` |
| Gazebo | Harmonic, gz-sim 8.11.0 | same image |
| SOFA (parked, offline brain FEM) | v26.06.00 + SofaPython3, Python 3.12 | `/nfs/hpc/dgx2-2/zhanhaoc/sofa/`, env `/nfs/hpc/dgx2-2/zhanhaoc/envs/sofa` |
Units: m / kg / s / rad everywhere.

## 1. Run a controller program (batch)
```bash
/nfs/hpc/share/zhanhaoc/neuro_demo/service/submit.sh my_controller.py 120     # 120 s budget
tail -f /nfs/hpc/dgx2-2/$USER/neuro_runs/<jobid>/status.txt
```
Output in `/nfs/hpc/dgx2-2/$USER/neuro_runs/<jobid>/`: `meta.json` (node, domain, controller sha), `status.txt`,
`controller.log`, `gz_server.log`, `bag/` (rosbag2 of joint states, all `/cmd/*`, `/demo/event`, 4 cameras).
Template: `service/examples/ctrl_scan_poke.py` (hover, 3x3 grid, touchdown, 4 mm poke, retract).

## 2. Remote manual control (interactive session)
```bash
/nfs/hpc/share/zhanhaoc/neuro_demo/service/session.sh 3600                 # GPU job, up for 1 h
/nfs/hpc/share/zhanhaoc/neuro_demo/service/connect.sh <run_dir> teleop     # keyboard, from your VNC / any node
/nfs/hpc/share/zhanhaoc/neuro_demo/service/connect.sh <run_dir> cams       # camera viewer
/nfs/hpc/share/zhanhaoc/neuro_demo/service/connect.sh <run_dir> shell      # run your own ROS2 nodes against it
```
Teleop keys: `1-5` select lift / arm_roll / wrist_pitch / head_rot / TwinZ, `+ -` move it, arrows = head XY stage,
`space` = poke 4 mm from the current TwinZ position and back, `r` = reset (lift 0.20, others 0), `q` = quit.

## 3. Local GUI (VNC on a GPU node)
```bash
neuro_demo/gz_demo/start_gazebo.sh                                          # Gazebo GUI + bridge
neuro_demo/gz_demo/ros_run.sh python3 neuro_demo/gz_demo/teleop_keys.py     # keyboard
neuro_demo/gz_demo/ros_run.sh ros2 run rqt_image_view rqt_image_view /cam/east/image
```
Request >= 8 CPU cores for the VNC job (1 core is unusably slow).

## Topics (bridge: `gz_demo/bridge.yaml`)
| topic | type | dir | rate | content |
|---|---|---|---|---|
| `/joint_states` | sensor_msgs/JointState | sim -> user | ~100 Hz | 7 joints: `lift_y` [0,0.25] m, `arm_roll_z` [-90,90] deg, `wrist_pitch_x` [-80,80] deg, `head_rot` [-45,45] deg, `stage_x`/`stage_z` [-0.0125,0.0125] m, `twinz_insert` [-0.025,0] m (negative = down) |
| `/cmd/<joint>` | std_msgs/Float64 | user -> sim | any | position target per joint (gz JointPositionController, velocity mode, p=8, |v|<=2) |
| `/cam/{east,south,north,southeast}/image` | sensor_msgs/Image rgb8 | sim -> user | 10 Hz | 320x240, hfov 0.9 rad, fixed to the robot head, aimed 1.5 cm below the needle tip |
| `/demo/event` | std_msgs/String | both | events | markers: poke / retract / reset / controller-defined |
| `/clock` | rosgraph_msgs/Clock | sim -> user | sim | simulation time |
| `/brain/needle_state` | std_msgs/String (JSON) | sim -> user | 50 Hz | `tip_world`, `surface_z`, `depth_m` (>0 = below cortex), `state` above/contact/inserted, `nearest_vessel_id`, `nearest_vessel_clearance_m` (edge, horizontal), `surface_dz_center_m`, `n_burst` |
| `/brain/vessel_burst` | std_msgs/String (JSON) | sim -> user | event | `vessel_id`, `tip_world`, `depth_m`, `vessel_radius_m`; a vessel bursts when the needle is inserted and the tip is within its radius (+12 um tip); cleared by `reset` on `/demo/event` |
| `/brain/vessels` | sensor_msgs/PointCloud2 | sim -> user | 10 Hz | vessel centrelines, fields x,y,z,radius,id,burst (world frame, moving with the pulsation) |
| `/brain/vessel_map` | sensor_msgs/Image rgb8 | sim -> user | 10 Hz | 400x400 top view of the craniotomy: vessels (red = burst), needle cross (green above / orange contact / blue inserted) |
| `/cam/<name>/vessel_mask` | sensor_msgs/Image mono8 | sim -> user | 10 Hz | vessel GT for that camera (255 vessel, 128 burst vessel), same size/pose/FOV as `/cam/<name>/image`; no occlusion test (needle/robot do not hide vessels) |

Brain model (simplest version, SOFA FEM parked): vessels = 11,824 centreline points / 980 segments extracted from the
cortex texture inside the skull's craniotomy hole (`gz_demo/find_opening.py`, `gz_demo/extract_vessels.py`; radius
median 77 um); pulsation = analytic dz = A(t)*cos^2(pi r / 2R), R = 20.1 mm hole radius, A(t) = cardiac 100 um p-p @1 Hz +
respiratory 500 um p-p @1/3 Hz (config.yaml). Sim time comes from `/joint_states` stamps.

World frame: z up; craniotomy centre at (0, 0, 1.225) (skull hole found geometrically; cortex there levelled by PCA, 35.2 deg).
Robot base at (-0.025, 0.625, 0): at zero pose the needle tip is at (0, 0, 1.04); with `lift_y=0.20` it is 15 mm above
the cortex. Needle-vertical kinematics: tip = (stage_z, -stage_x, 1.04 + lift_y + twinz_insert).
Reach note: `wrist_pitch_x` and `head_rot` are parallel axes, so they only pull the tip back; forward reach is the
zero pose + 12.5 mm of stage. The robot must therefore stand with the needle above the opening.

## Multi-user isolation
Each job gets `ROS_DOMAIN_ID = 1 + jobid % 100` and `GZ_PARTITION = neuro_<jobid>` (written to `<run_dir>/connect.env`),
per-user container home `/tmp/neuro_ros_home_$USER`, per-user results `/nfs/hpc/dgx2-2/$USER/neuro_runs`.
Other users need read access to `/nfs/hpc/share/zhanhaoc/neuro_demo` and `/nfs/hpc/dgx2-2/zhanhaoc/neuro_demo/{ros,gz}`.

## Packaging (portable image)
`deploy/Dockerfile` and `deploy/neuro_sim.def` bake code + Gazebo assets into one image (assets rewritten to `/opt/neuro`).
Copy `/nfs/hpc/dgx2-2/zhanhaoc/neuro_demo/gz` to `deploy/assets/gz` first.

## Layout
`gz_demo/` Gazebo + ROS2 (URDF/world generators, bridge, teleop, container wrapper) · `service/` cluster service ·
`deploy/` image recipes · `sofa_brain/` SOFA brain (parked) · `robot_v1/` your Fusion export · `third_party/` D3MIA.
