#!/bin/bash
# Runs ONE simulation inside a GPU allocation (called by submit.sh / session.sh via sbatch).
# Headless Gazebo (EGL, works on A40 and H100) + ROS2 bridge + rosbag recording, isolated per job:
#   ROS_DOMAIN_ID = 1 + jobid % 100,  GZ_PARTITION = neuro_<jobid>
# Modes:
#   run     : run the uploaded controller (--controller file.py) for --duration s, then stop.
#   session : stay up for --duration s; connect with connect.sh <run_dir> (teleop / camera viewer / your own nodes).
# Usage: sim_job.sh --mode run|session (--out <run_dir> | --runs <base_dir>) [--controller ctrl.py] [--duration 120] [--record 1]
#        with --runs the run dir is <base_dir>/<SLURM_JOB_ID>
set -u
ROOT=${NEURO_ROOT:-/nfs/hpc/share/zhanhaoc/neuro_demo}
G=${NEURO_GZ:-/nfs/hpc/dgx2-2/zhanhaoc/neuro_demo/gz}
MODE=run; OUT=""; CTRL=""; DUR=120; REC=1; SCEN=""
while [ $# -gt 0 ]; do case $1 in
  --mode) MODE=$2; shift 2;; --out) OUT=$2; shift 2;; --controller) CTRL=$2; shift 2;;
  --duration) DUR=$2; shift 2;; --record) REC=$2; shift 2;; --scenario) SCEN=$2; shift 2;; --runs) OUT=$2/${SLURM_JOB_ID:-$$}; shift 2;;
  *) echo "unknown arg $1"; exit 2;; esac; done
[ -n "$OUT" ] || { echo "--out required"; exit 2; }
mkdir -p "$OUT"
if [ -n "$SCEN" ]; then cp "$SCEN" "$OUT/scenario.yaml"; export NEURO_SCENARIO="$OUT/scenario.yaml"
  d=$(sed -n 's/^duration: *\([0-9.]*\).*/\1/p' "$SCEN"); [ "$MODE" = run ] && [ -n "$d" ] && DUR=${d%.*}; fi
JOB=${SLURM_JOB_ID:-$$}
export ROS_DOMAIN_ID=$(( 1 + JOB % 100 )) GZ_PARTITION=neuro_${JOB} ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
# inside the packaged image every process runs directly; on the host each one goes through the container wrapper
if [ -n "${NEURO_IN_CONTAINER:-}" ]; then RUN=""; else RUN="$ROOT/gz_demo/ros_run.sh"; fi
cat > "$OUT/connect.env" <<EOF
# source this (or use service/connect.sh $OUT) on any cluster node to talk to this simulation
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS=$(hostname)
export GZ_PARTITION=$GZ_PARTITION
SIM_NODE=$(hostname)
SIM_JOB=$JOB
EOF
echo "{\"job\": \"$JOB\", \"node\": \"$(hostname)\", \"mode\": \"$MODE\", \"ros_domain_id\": $ROS_DOMAIN_ID, \"gz_partition\": \"$GZ_PARTITION\", \"duration_s\": $DUR, \"controller\": \"$(basename "${CTRL:-none}")\", \"controller_sha256\": \"$( [ -n "$CTRL" ] && sha256sum "$CTRL" | cut -c1-16)\", \"start\": \"$(date -Is)\"}" > "$OUT/meta.json"
[ -n "$CTRL" ] && cp "$CTRL" "$OUT/controller.py"

PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; sleep 3
  for p in "${PIDS[@]}"; do kill -9 "$p" 2>/dev/null; done; wait 2>/dev/null;   # some gz/ros processes ignore SIGTERM echo "end $(date -Is)" >> "$OUT/status.txt"
  [ "$REC" = 1 ] && [ -d "$OUT/bag" ] && $RUN python3 "$ROOT/service/report.py" "$OUT" > "$OUT/report.txt" 2>&1; }
trap cleanup EXIT
echo "starting $(date -Is) on $(hostname) domain=$ROS_DOMAIN_ID partition=$GZ_PARTITION" > "$OUT/status.txt"

cd /tmp
$RUN gz sim -s -r --headless-rendering "$G/world.sdf" > "$OUT/gz_server.log" 2>&1 & PIDS+=($!)
sleep 10
$RUN ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$ROOT/gz_demo/bridge.yaml" > "$OUT/bridge.log" 2>&1 & PIDS+=($!)
sleep 4
# brain model: sofa (real-time FEM, default) | analytic (closed-form pulsation, no tissue mechanics)
BRAIN_PY=$([ "${NEURO_BRAIN:-sofa}" = analytic ] && echo brain_node.py || echo sofa_brain_node.py)
$RUN python3 "$ROOT/gz_demo/$BRAIN_PY" > "$OUT/brain_node.log" 2>&1 & PIDS+=($!)
for i in $(seq 240); do grep -q "\[brain_node\] ready" "$OUT/brain_node.log" && break; sleep 0.5; done
grep -q "\[brain_node\] ready" "$OUT/brain_node.log" || { echo "brain_node failed to start" >> "$OUT/status.txt"; exit 1; }
if [ "$REC" = 1 ]; then
  $RUN ros2 bag record -o "$OUT/bag" -e '/joint_states|/demo/event|/cmd/.*|/cam/.*|/brain/.*' > "$OUT/record.log" 2>&1 & PIDS+=($!)
fi
$RUN python3 "$ROOT/gz_demo/dashboard_node.py" "$OUT" > "$OUT/dashboard.log" 2>&1 & PIDS+=($!)
echo "ready $(date -Is)" >> "$OUT/status.txt"

if [ "$MODE" = run ]; then
  [ -n "$CTRL" ] || { echo "--controller required in run mode"; exit 2; }
  timeout "$DUR" $RUN python3 "$ROOT/service/run_controller.py" "$OUT/controller.py" > "$OUT/controller.log" 2>&1
  echo "controller exit $? $(date -Is)" >> "$OUT/status.txt"
  sleep 1
else
  sleep "$DUR"
fi
