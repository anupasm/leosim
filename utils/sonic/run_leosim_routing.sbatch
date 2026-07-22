#!/bin/bash -l
#SBATCH --job-name=leosim-routing
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --time=24:00:00
#SBATCH --array=0-4%10
#SBATCH --output=logs/%x-%A_%a.out
#SBATCH --error=logs/%x-%A_%a.err

# To enable email notifications, either submit with `--mail-user`/`--mail-type`
# options to `sbatch` or uncomment and set the following two lines below.
# Example submit that sends mail on job begin, end, and failure:
#   EMAIL=you@example.com \
#   sbatch --mail-user="$EMAIL" --mail-type=BEGIN,END,FAIL \
#     ns3/contrib/leosim/utils/sonic/run_leosim_routing.sbatch
# Uncomment and edit to hardcode notifications in the script itself:
#SBATCH --mail-user=anupa.desilva@ucd.ie
#SBATCH --mail-type=BEGIN,END,FAIL

set -euo pipefail

if command -v module >/dev/null 2>&1; then
  module purge
  module load "${APPTAINER_MODULE:-apptainer/1.3.4-gcc-11.5.0-ojp6nts}"
fi

# Matrix: 5 metrics x 1 constellation size x 1 duration x 3 update intervals.
METRICS=(hop distance path-loss snr signal-strength)
SATELLITE_COUNTS=(1000)
SIM_TIMES=(1200)
UPDATE_INTERVALS=(5)

REPO_ROOT="${REPO_ROOT:-${SLURM_SUBMIT_DIR}}"
SIF="${SIF:-${REPO_ROOT}/leosim.sif}"
RESULTS_ROOT="${RESULTS_ROOT:-${REPO_ROOT}/results/leosim-routing/${SLURM_ARRAY_JOB_ID}}"
if [[ "$RESULTS_ROOT" != /* ]]; then
  RESULTS_ROOT="${REPO_ROOT}/${RESULTS_ROOT}"
fi
CONTAINER_ROOT="/opt/leosim"
CONTAINER_NS3_ROOT="${CONTAINER_ROOT}/ns3"
CONTAINER_RESULTS_ROOT="/results"
task_id="${SLURM_ARRAY_TASK_ID}"
matrix_size=$((${#METRICS[@]} * ${#SATELLITE_COUNTS[@]} * ${#SIM_TIMES[@]} * ${#UPDATE_INTERVALS[@]}))

if (( task_id < 0 || task_id >= matrix_size )); then
  echo "Array task ${task_id} is outside 0..$((matrix_size - 1))" >&2
  exit 2
fi

# Decode the array index, with update interval varying fastest.
index=$task_id
interval="${UPDATE_INTERVALS[$((index % ${#UPDATE_INTERVALS[@]}))]}"
index=$((index / ${#UPDATE_INTERVALS[@]}))
sim_time="${SIM_TIMES[$((index % ${#SIM_TIMES[@]}))]}"
index=$((index / ${#SIM_TIMES[@]}))
satellites="${SATELLITE_COUNTS[$((index % ${#SATELLITE_COUNTS[@]}))]}"
index=$((index / ${#SATELLITE_COUNTS[@]}))
metric="${METRICS[$index]}"

app_start="${APP_START:-1}"
app_stop="${APP_STOP:-$((sim_time - 1))}"
num_ues="${NUM_UES:-0}"
num_servers="${NUM_SERVERS:-0}"
all_to_all="${ALL_TO_ALL_TRAFFIC:-1}"
tcp_rate="${TCP_RATE:-1Mbps}"
verbose="${VERBOSE:-1}"
statistics_interval="${STATISTICS_INTERVAL:-1}"
max_isl_neighbors="${MAX_ISL_NEIGHBORS:-4}"
tle_timestep="${TLE_TIMESTEP:-5}"
tle_workers="${TLE_WORKERS:-${SLURM_CPUS_PER_TASK:-1}}"

prepro_dir="${RESULTS_ROOT}/_preprocessed"
satellite_positions="${prepro_dir}/alpha-beta-gamma_duration-${sim_time}s_step-${tle_timestep}s.csv"
container_satellite_positions="${CONTAINER_RESULTS_ROOT}/_preprocessed/$(basename "$satellite_positions")"
prepro_lock="${satellite_positions}.lock"

run_id="metric-${metric}_sats-${satellites}_dynamic_duration-${sim_time}s-interval-${interval}s"
run_dir="${RESULTS_ROOT}/${run_id}"
container_prefix="${CONTAINER_RESULTS_ROOT}/${run_id}/result"
mkdir -p "${REPO_ROOT}/logs" "$run_dir"

if ! command -v apptainer >/dev/null 2>&1; then
  echo "Apptainer is unavailable; load the site Apptainer module or set APPTAINER_MODULE." >&2
  exit 2
fi
if [[ ! -f "$SIF" ]]; then
  echo "Missing LeoSim container image: ${SIF}" >&2
  echo "Build it with ns3/contrib/leosim/utils/sonic/build_leosim_container.sh." >&2
  exit 2
fi
if ! apptainer exec --cleanenv "$SIF" /opt/venv/bin/python -c 'import numpy, skyfield' 2>/dev/null; then
  echo "The container does not provide the required LeoSim Python environment: ${SIF}" >&2
  exit 2
fi

# All array elements share one generated position file. The lock prevents 15
# simultaneous conversions; the first task generates it and the rest reuse it.
mkdir -p "$prepro_dir"
exec 9>"$prepro_lock"
flock -w 21600 9
if [[ ! -s "$satellite_positions" ]]; then
  temporary_positions="${satellite_positions}.tmp-${SLURM_ARRAY_JOB_ID}-${task_id}-$$"
  container_temporary_positions="${CONTAINER_RESULTS_ROOT}/_preprocessed/$(basename "$temporary_positions")"
  apptainer exec --cleanenv \
    --bind "${RESULTS_ROOT}:${CONTAINER_RESULTS_ROOT}" \
    "$SIF" \
    /opt/venv/bin/python "${CONTAINER_NS3_ROOT}/contrib/leosim/utils/tle_to_positions.py" \
    --input "${CONTAINER_NS3_ROOT}/contrib/leosim/data/tles" \
    --output "$container_temporary_positions" \
    --duration "$sim_time" \
    --timestep "$tle_timestep" \
    --workers "$tle_workers"
  mv "$temporary_positions" "$satellite_positions"
fi
flock -u 9

scenario="leosim-param-scenario"
scenario+=" --satellites=${container_satellite_positions}"
scenario+=" --useTrace=0"
scenario+=" --numSatellites=${satellites}"
scenario+=" --numServers=${num_servers}"
scenario+=" --numUes=${num_ues}"
scenario+=" --allToAllTraffic=${all_to_all}"
scenario+=" --routingMetric=${metric}"
scenario+=" --enableDynamicRouting=1"
scenario+=" --routingUpdateInterval=${interval}"
scenario+=" --maxIslNeighbors=${max_isl_neighbors}"
scenario+=" --simTime=${sim_time}"
scenario+=" --appStart=${app_start}"
scenario+=" --appStop=${app_stop}"
scenario+=" --tcpRate=${tcp_rate}"
scenario+=" --enableStatistics=1"
scenario+=" --statisticsInterval=${statistics_interval}"
scenario+=" --enableRouteLogging=1"
scenario+=" --verbose=${verbose}"
scenario+=" --enableVisualization=0"
scenario+=" --writeFlowMonitor=1"
scenario+=" --outputPrefix=${container_prefix}"
command=(apptainer exec --cleanenv
  --bind "${RESULTS_ROOT}:${CONTAINER_RESULTS_ROOT}"
  --pwd "$CONTAINER_NS3_ROOT"
  "$SIF"
  ./ns3 run "$scenario" --no-build)

{
  echo "job_id=${SLURM_ARRAY_JOB_ID}_${task_id}"
  echo "run_id=${run_id}"
  echo "metric=${metric}"
  echo "satellites=${satellites}"
  echo "sim_time_s=${sim_time}"
  echo "routing_mode=dynamic"
  echo "routing_update_interval_s=${interval}"
  echo "num_ues=${num_ues}"
  echo "num_servers=${num_servers}"
  echo "all_to_all_traffic=${all_to_all}"
  echo "tcp_rate_per_flow=${tcp_rate}"
  echo "satellite_positions_host=${satellite_positions}"
  echo "satellite_positions_container=${container_satellite_positions}"
  echo "tle_sources=alpha.csv,beta.csv,gamma.csv"
  echo "tle_catalog_satellites=11514"
  echo "tle_timestep_s=${tle_timestep}"
  echo "container_image=${SIF}"
  echo "host=$(hostname)"
  echo "started=${SLURM_JOB_START_TIME:-unknown}"
  echo "git_commit=$(git -C "$REPO_ROOT" rev-parse HEAD 2>/dev/null || echo embedded-in-image)"
  printf 'command='; printf ' %q' "${command[@]}"; printf '\n'
} | tee "${run_dir}/metadata.txt"

if [[ -x /usr/bin/time ]]; then
  /usr/bin/time -v -o "${run_dir}/resources.txt" "${command[@]}"
else
  "${command[@]}"
fi

echo "PASS" > "${run_dir}/status.txt"
