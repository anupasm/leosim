#!/bin/bash -l
#SBATCH --job-name=leosim-rp1
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --time=00:30:00
#SBATCH --array=0-8%9
#SBATCH --output=logs/%x-%A_%a.out
#SBATCH --error=logs/%x-%A_%a.err

set -euo pipefail

# The implemented pilot supports these nine distinct configurations. The array
# throttle (%9 above) is deliberately modest; increase it only after measuring
# memory and runtime with sacct.
METRICS=(
  distance
  prop-delay
  hop-count
  visibility
  path-lifetime
  congestion
  failure-aware
  handover-aware
  multi-objective
)

REPO_ROOT="${REPO_ROOT:-${SLURM_SUBMIT_DIR}}"
NS3_ROOT="${REPO_ROOT}/ns3"
RESULTS_ROOT="${RESULTS_ROOT:-${REPO_ROOT}/results/rp1-routing/${SLURM_ARRAY_JOB_ID}}"
task_id="${SLURM_ARRAY_TASK_ID}"

if (( task_id < 0 || task_id >= ${#METRICS[@]} )); then
  echo "Array task ${task_id} is outside 0..$((${#METRICS[@]} - 1))" >&2
  exit 2
fi

metric="${METRICS[$task_id]}"
run_dir="${RESULTS_ROOT}/${metric}"
mkdir -p "${REPO_ROOT}/logs" "$run_dir"

if [[ ! -x "${NS3_ROOT}/ns3" ]]; then
  echo "Missing ${NS3_ROOT}/ns3; set REPO_ROOT to the LeoSim checkout." >&2
  exit 2
fi

cd "$NS3_ROOT"
command=(
  ./ns3 run
  "leosim-routing-research-example --routingMetric=${metric} --congestionWeight=${CONGESTION_WEIGHT:-0.25} --output=${run_dir}/result.csv"
  --no-build
)

echo "job=${SLURM_ARRAY_JOB_ID}_${task_id} host=$(hostname) metric=${metric}"
printf 'command='; printf ' %q' "${command[@]}"; printf '\n'

if [[ -x /usr/bin/time ]]; then
  /usr/bin/time -v -o "${run_dir}/resources.txt" "${command[@]}"
else
  "${command[@]}"
fi

{
  echo "job_id=${SLURM_ARRAY_JOB_ID}_${task_id}"
  echo "metric=${metric}"
  echo "host=$(hostname)"
  echo "started=${SLURM_JOB_START_TIME:-unknown}"
  echo "git_commit=$(git -C "$REPO_ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
} > "${run_dir}/metadata.txt"
