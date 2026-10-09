#!/bin/bash
# Submit a controller program to run on a GPU node; results land in $RUNS/<jobid>/.
# Usage: service/submit.sh my_controller.py [duration_s=120] [scenario.yaml]
# Output: meta.json, status.txt, controller.log, gz_server.log, bag/ (rosbag2: joint states, cmds, events, 4 cameras)
set -e
CTRL=$(readlink -f "$1"); DUR=${2:-120}; SCEN=${3:+--scenario $(readlink -f "$3")}
ROOT=/nfs/hpc/share/zhanhaoc/neuro_demo
RUNS=${NEURO_RUNS:-/nfs/hpc/dgx2-2/$USER/neuro_runs}; mkdir -p "$RUNS"
J=$(sbatch --parsable -J neuro_run -p ${NEURO_PARTITION:-dgxh,ampere,dgx2} --gres=gpu:1 -c 8 --mem=32G -t $(( DUR / 60 + 15 )) \
     -o "$RUNS/%j.slurm.log" "$ROOT/service/sim_job.sh" --mode run --runs "$RUNS" --controller $CTRL --duration $DUR $SCEN)
echo "submitted job $J -> $RUNS/$J   (squeue -j $J; tail -f $RUNS/$J/status.txt)"
