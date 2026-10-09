#!/bin/bash
# neuro: entry point of the packaged simulation image (apptainer run neuro_sim.sif <cmd> ...)
#   neuro run <controller.py> [--duration 120] [--out DIR]   run an engineer's controller headless, record everything
#   neuro session [--duration 3600] [--out DIR]              start a simulation and keep it up for remote control
#   neuro attach <run_dir> teleop|topics|shell               attach to a running simulation (any cluster node)
set -e
R=${NEURO_ROOT:-/opt/neuro}
source /opt/ros/jazzy/setup.bash
cmd=${1:-help}; shift || true
case $cmd in
  run)     ctrl=$(readlink -f "$1"); shift; out=${NEURO_OUT:-$PWD/neuro_run_$(date +%Y%m%d_%H%M%S)}
           exec "$R/service/sim_job.sh" --mode run --controller "$ctrl" --out "$out" "$@";;
  session) out=${NEURO_OUT:-$PWD/neuro_session_$(date +%Y%m%d_%H%M%S)}
           exec "$R/service/sim_job.sh" --mode session --out "$out" "$@";;
  attach)  set -a; source "$1/connect.env"; set +a
           case ${2:-teleop} in teleop) exec python3 "$R/gz_demo/teleop_keys.py";; topics) exec ros2 topic list;; shell) exec bash;; esac;;
  *)       sed -n '2,5p' "$0";;
esac
