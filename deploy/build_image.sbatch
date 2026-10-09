#!/bin/bash
#SBATCH -J build_neuro_sim
#SBATCH -p share
#SBATCH -c 8
#SBATCH --mem=24G
#SBATCH -t 1:00:00
#SBATCH -o /nfs/hpc/dgx2-2/zhanhaoc/neuro_demo/logs/build_image_%j.log
set -e
export APPTAINER_CACHEDIR=/nfs/hpc/dgx2-2/zhanhaoc/.cache/apptainer APPTAINER_TMPDIR=/tmp/apptainer_build_$SLURM_JOB_ID
mkdir -p $APPTAINER_TMPDIR; cd /tmp
apptainer build --fakeroot --force --mksquashfs-args "-mem 4G -processors 8" \
    /nfs/hpc/dgx2-2/zhanhaoc/neuro_demo/neuro_sim.sif /nfs/hpc/share/zhanhaoc/neuro_demo/deploy/neuro_sim.def
rm -rf $APPTAINER_TMPDIR
ls -lh /nfs/hpc/dgx2-2/zhanhaoc/neuro_demo/neuro_sim.sif
apptainer exec /nfs/hpc/dgx2-2/zhanhaoc/neuro_demo/neuro_sim.sif bash -c 'source /opt/ros/jazzy/setup.bash; python3 -c "import Sofa, rclpy, cv2, yaml; print(\"image ok: Sofa+rclpy+cv2\")" 2>&1 | tail -1; ls /opt/neuro'
