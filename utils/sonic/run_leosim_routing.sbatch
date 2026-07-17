#!/bin/bash -l
#SBATCH --job-name=leosim-routing
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --time=06:00:00
#SBATCH --array=0-14%10
#SBATCH --output=logs/%x-%A_%a.out
#SBATCH --error=logs/%x-%A_%a.err

set -euo pipefail

# Matrix: 5 metrics x 1 constellation size x 1 duration x 3 update intervals.
METRICS=(hop distance path-loss snr signal-strength)
SATELLITE_COUNTS=(11514)
SIM_TIMES=(1800)
UPDATE_INTERVALS=(5 10 30)

REPO_ROOT="${REPO_ROOT:-${SLURM_SUBMIT_DIR}}"
NS3_ROOT="${REPO_ROOT}/ns3"
RESULTS_ROOT="${RESULTS_ROOT:-${REPO_ROOT}/results/leosim-routing/${SLURM_ARRAY_JOB_ID}}"
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
statistics_interval="${STATISTICS_INTERVAL:-1}"
max_isl_neighbors="${MAX_ISL_NEIGHBORS:-4}"
tle_timestep="${TLE_TIMESTEP:-5}"
tle_workers="${TLE_WORKERS:-${SLURM_CPUS_PER_TASK:-1}}"
tle_python="${TLE_PYTHON:-${REPO_ROOT}/.venv-leosim-sonic/bin/python}"

tle_dir="${NS3_ROOT}/contrib/leosim/data/tles"
tle_converter="${NS3_ROOT}/contrib/leosim/utils/tle_to_positions.py"
prepro_dir="${RESULTS_ROOT}/_preprocessed"
satellite_positions="${prepro_dir}/alpha-beta-gamma_duration-${sim_time}s_step-${tle_timestep}s.csv"
prepro_lock="${satellite_positions}.lock"

run_id="metric-${metric}_sats-${satellites}_dynamic_duration-${sim_time}s-interval-${interval}s"
run_dir="${RESULTS_ROOT}/${run_id}"
prefix="${run_dir}/result"
mkdir -p "${REPO_ROOT}/logs" "$run_dir"

if [[ ! -x "${NS3_ROOT}/ns3" ]]; then
  echo "Missing ${NS3_ROOT}/ns3; set REPO_ROOT to the LeoSim checkout." >&2
  exit 2
fi

if [[ ! -f "${tle_dir}/alpha.csv" || ! -f "${tle_dir}/beta.csv" || ! -f "${tle_dir}/gamma.csv" ]]; then
  echo "Expected alpha.csv, beta.csv, and gamma.csv under ${tle_dir}." >&2
  exit 2
fi
if [[ ! -f "$tle_converter" ]]; then
  echo "Missing TLE converter: ${tle_converter}" >&2
  exit 2
fi
if [[ ! -x "$tle_python" ]] || ! "$tle_python" -c 'import numpy, skyfield' 2>/dev/null; then
  echo "TLE Python environment is unavailable or lacks numpy/skyfield: ${tle_python}" >&2
  echo "Run ns3/contrib/leosim/utils/sonic/build_leosim.sh before submitting." >&2
  exit 2
fi

# All array elements share one generated position file. The lock prevents 75
# simultaneous conversions; the first task generates it and the rest reuse it.
mkdir -p "$prepro_dir"
exec 9>"$prepro_lock"
flock -w 21600 9
if [[ ! -s "$satellite_positions" ]]; then
  temporary_positions="${satellite_positions}.tmp-${SLURM_ARRAY_JOB_ID}-${task_id}-$$"
  "$tle_python" "$tle_converter" \
    --input "$tle_dir" \
    --output "$temporary_positions" \
    --duration "$sim_time" \
    --timestep "$tle_timestep" \
    --workers "$tle_workers"
  mv "$temporary_positions" "$satellite_positions"
fi
flock -u 9

cd "$NS3_ROOT"
scenario="leosim-param-scenario"
scenario+=" --satellites=${satellite_positions}"
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
scenario+=" --enableVisualization=0"
scenario+=" --writeFlowMonitor=1"
scenario+=" --outputPrefix=${prefix}"
command=(./ns3 run "$scenario" --no-build)

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
  echo "satellite_positions=${satellite_positions}"
  echo "tle_sources=alpha.csv,beta.csv,gamma.csv"
  echo "tle_catalog_satellites=11514"
  echo "tle_timestep_s=${tle_timestep}"
  echo "tle_python=${tle_python}"
  echo "host=$(hostname)"
  echo "started=${SLURM_JOB_START_TIME:-unknown}"
  echo "git_commit=$(git -C "$REPO_ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
  printf 'command='; printf ' %q' "${command[@]}"; printf '\n'
} | tee "${run_dir}/metadata.txt"

if [[ -x /usr/bin/time ]]; then
  /usr/bin/time -v -o "${run_dir}/resources.txt" "${command[@]}"
else
  "${command[@]}"
fi

echo "PASS" > "${run_dir}/status.txt"
