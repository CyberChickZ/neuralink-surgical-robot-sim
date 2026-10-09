#!/bin/bash
# One-shot launcher (run from a VNC terminal): Gazebo world + robot + ROS2 bridge + camera viewer.
# Then, in another terminal, drive it with:  neuro_demo/gz_demo/ros_run.sh python3 neuro_demo/gz_demo/teleop_keys.py
D=/nfs/hpc/share/zhanhaoc/neuro_demo/gz_demo; G=/nfs/hpc/dgx2-2/zhanhaoc/neuro_demo/gz; L=/nfs/hpc/dgx2-2/zhanhaoc/neuro_demo/logs
$D/ros_run.sh gz sim -r $G/world.sdf > $L/gz_sim.log 2>&1 &
sleep 8
# robot is <include>d in world.sdf (model://neuro_robot); spawning via ros_gz_sim create was unreliable here
$D/ros_run.sh ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$D/bridge.yaml > $L/gz_bridge.log 2>&1 &
sleep 3
$D/ros_run.sh ros2 run rqt_image_view rqt_image_view /cam/loop/image > $L/rqt_loop.log 2>&1 &
echo "launched: gz sim, robot, bridge, camera viewer. logs in $L"
wait
