#!/bin/bash
# Build all generated simulation assets into $NEURO_GZ (then build the image with deploy/build_image.sbatch).
#   1. fetch the brain phantom data (D3MIA/SOFA-NeuroSim-Recorder; no license file -> fetched, never redistributed)
#   2. robot: Fusion URDF -> Gazebo model (gz plugins, hub cameras, joint limits)
#   3. brain: craniotomy from skull coverage -> levelled world + textured meshes -> vessel GT -> real-time FEM block
# Needs: the ROS2 container (gz_demo/ros_run.sh) and a Python env with gmsh, scipy, scikit-image, pillow.
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
G=${NEURO_GZ:-/nfs/hpc/dgx2-2/$USER/neuro_demo/gz}; mkdir -p "$G"
PY=${NEURO_PY:-/nfs/hpc/dgx2-2/zhanhaoc/envs/sofa/bin/python}
RUN="$ROOT/gz_demo/ros_run.sh"
[ -d "$ROOT/third_party/SOFA-NeuroSim-Recorder" ] || git clone --depth 1 https://github.com/D3MIA/SOFA-NeuroSim-Recorder.git "$ROOT/third_party/SOFA-NeuroSim-Recorder"
$RUN bash -c "python3 $ROOT/gz_demo/gen_gz_urdf.py $ROOT/robot_v1/robot_v1_fixed.urdf $G/robot_gz.urdf && gz sdf -p $G/robot_gz.urdf > $G/robot_gz.sdf"
mkdir -p "$G/neuro_robot" && cp -f "$G/robot_gz.sdf" "$G/neuro_robot/model.sdf"
printf '<?xml version="1.0"?>\n<model><name>neuro_robot</name><version>1.0</version><sdf version="1.11">model.sdf</sdf></model>\n' > "$G/neuro_robot/model.config"
PYTHONNOUSERSITE=1 $PY "$ROOT/gz_demo/find_opening.py" "$G"
$RUN python3 "$ROOT/gz_demo/gen_world.py" "$G"
PYTHONNOUSERSITE=1 $PY "$ROOT/gz_demo/extract_vessels.py" "$G"
PYTHONNOUSERSITE=1 $PY "$ROOT/gz_demo/make_block_mesh.py" "$G" 0.003
rm -f "$G/sofa_gain.npy"   # recalibrated on first SOFA start
echo "assets ready in $G"
