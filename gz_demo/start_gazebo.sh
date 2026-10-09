#!/bin/bash
# Terminal 1: Gazebo (world + robot + brain) and the ROS2<->Gazebo bridge. Ctrl-C stops both.
D=/nfs/hpc/share/zhanhaoc/neuro_demo/gz_demo; G=/nfs/hpc/dgx2-2/zhanhaoc/neuro_demo/gz
cd /tmp
$D/ros_run.sh ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$D/bridge.yaml > /dev/null 2>&1 &
B=$!
$D/ros_run.sh python3 $D/gz_demo/brain_node.py > /tmp/brain_node_$USER.log 2>&1 &
trap "kill $B $! 2>/dev/null" EXIT
VGL=1 $D/ros_run.sh gz sim -r $G/world.sdf
