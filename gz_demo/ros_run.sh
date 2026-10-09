#!/bin/bash
# Run a command inside the ROS2 Jazzy + Gazebo Harmonic container, GPU OpenGL via host VirtualGL (EGL back end).
# Works on A40 and H100 nodes from a VNC session. Usage: [VGL=1] ros_run.sh <cmd...>   (ROS env already sourced)
# VGL=1 only for 3D GL apps (gz sim). Plain Qt apps (rqt_image_view) crash under the VGL faker, so it is opt-in.
SIF=/nfs/hpc/dgx2-2/zhanhaoc/neuro_demo/ros/ros_jazzy_desktop_full.sif
H=${NEURO_HOME:-/tmp/neuro_ros_home_$USER}; mkdir -p $H   # per user, node-local
exec apptainer exec --nv --home $H:/root -B /nfs/hpc \
  -B /usr/lib64/VirtualGL:/opt/vgl:ro -B /lib64/libturbojpeg.so.0:/opt/vglx/libturbojpeg.so.0:ro \
  -B /usr/share/glvnd/egl_vendor.d/10_nvidia.json:/opt/vglx/10_nvidia.json:ro \
  --env DISPLAY=${DISPLAY},LD_LIBRARY_PATH=/opt/vgl:/opt/vglx:/nfs/hpc/dgx2-2/zhanhaoc/sofa/SOFA_v26.06.00_Linux/lib,SOFA_ROOT=/nfs/hpc/dgx2-2/zhanhaoc/sofa/SOFA_v26.06.00_Linux,PYTHONPATH=/nfs/hpc/dgx2-2/zhanhaoc/sofa/SOFA_v26.06.00_Linux/plugins/SofaPython3/lib/python3/site-packages,NEURO_SCENARIO=${NEURO_SCENARIO:-},VGL_DISPLAY=egl,VGL_EGLLIB=libEGL.so.1 \
  --env __EGL_VENDOR_LIBRARY_FILENAMES=/opt/vglx/10_nvidia.json${VGL:+,LD_PRELOAD=/opt/vgl/libdlfaker.so:/opt/vgl/libvglfaker.so} \
  --env OMP_NUM_THREADS=1,OPENBLAS_NUM_THREADS=1,MKL_NUM_THREADS=1,GZ_SIM_RESOURCE_PATH=/nfs/hpc/dgx2-2/zhanhaoc/neuro_demo/gz,ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42},GZ_PARTITION=${GZ_PARTITION:-},ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET},ROS_STATIC_PEERS=${ROS_STATIC_PEERS:-} \
  $SIF bash -c 'source /opt/ros/jazzy/setup.bash && exec "$@"' _ "$@"
