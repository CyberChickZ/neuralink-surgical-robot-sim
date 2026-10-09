#!/bin/bash
# Attach to a running simulation (session or run) from any cluster node.
# Usage: service/connect.sh <run_dir> teleop|cams|shell|topics
#   teleop : keyboard control (1-5 select joint, +/- move, arrows XY stage, space poke, r reset)
#   cams   : camera viewer (switch among /cam/{east,south,north,southeast}/image in the dropdown)
#   topics : list live topics
#   shell  : bash inside the ROS container with this simulation's env (run your own nodes)
set -e
RUN_DIR=$1; WHAT=${2:-teleop}
ROOT=/nfs/hpc/share/zhanhaoc/neuro_demo
source "$RUN_DIR/connect.env"
export ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS GZ_PARTITION
grep -q ready "$RUN_DIR/status.txt" || { echo "simulation not ready yet: $(cat $RUN_DIR/status.txt)"; exit 1; }
case $WHAT in
  teleop) exec "$ROOT/gz_demo/ros_run.sh" python3 "$ROOT/gz_demo/teleop_keys.py";;
  cams)   exec "$ROOT/gz_demo/ros_run.sh" ros2 run rqt_image_view rqt_image_view /cam/east/image;;
  topics) exec "$ROOT/gz_demo/ros_run.sh" ros2 topic list;;
  shell)  exec "$ROOT/gz_demo/ros_run.sh" bash;;
  *) echo "unknown: $WHAT"; exit 2;;
esac
