#!/bin/bash
# Start an interactive (remote-controlled) simulation on a GPU node; it stays up for duration_s.
# Usage: service/session.sh [duration_s=3600]      then: service/connect.sh <run_dir> teleop|cams|shell
set -e
DUR=${1:-3600}
ROOT=/nfs/hpc/share/zhanhaoc/neuro_demo
RUNS=${NEURO_RUNS:-/nfs/hpc/dgx2-2/$USER/neuro_runs}; mkdir -p "$RUNS"
J=$(sbatch --parsable -J neuro_session -p ${NEURO_PARTITION:-dgxh,ampere,dgx2} --gres=gpu:1 -c 8 --mem=32G -t $(( DUR / 60 + 10 )) \
     -o "$RUNS/%j.slurm.log" "$ROOT/service/sim_job.sh" --mode session --runs "$RUNS" --duration $DUR)
echo "session job $J -> $RUNS/$J"
echo "wait for 'ready' in $RUNS/$J/status.txt, then:  $ROOT/service/connect.sh $RUNS/$J teleop"
