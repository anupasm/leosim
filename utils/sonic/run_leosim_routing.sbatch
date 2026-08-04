#!/bin/bash -l
#SBATCH --job-name=leosim-routing
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
#SBATCH --array=0-6%3
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

# Matrix: 7 metrics x 1 constellation size x 1 duration x 1 update interval.
# Every metric experiences the same synthetic ISL load; only its routing edge
# criterion changes. The load experiment routes on residual capacity.
METRICS=(hop distance path-loss snr signal-strength lifetime load)
SATELLITE_COUNTS=(11514)
SIM_TIMES=(1200)
UPDATE_INTERVALS=(30)

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
enable_handover_logging="${ENABLE_HANDOVER_LOGGING:-1}"
max_isl_neighbors="${MAX_ISL_NEIGHBORS:-4}"
tle_timestep="${TLE_TIMESTEP:-5}"
allocated_cpus="${SLURM_CPUS_PER_TASK:-1}"
tle_workers="${TLE_WORKERS:-${allocated_cpus}}"
use_route_tree_cache="${USE_ROUTE_TREE_CACHE:-1}"
# ns-3 invokes the external routing engine synchronously and waits for it to
# finish, so the engine may use the full task allocation. The simulator itself
# remains single-threaded outside these short routing-engine bursts.
default_route_tree_workers="${allocated_cpus}"
route_tree_workers="${ROUTE_TREE_WORKERS:-${default_route_tree_workers}}"
route_tree_max_entries="${ROUTE_TREE_MAX_ENTRIES:-10000000}"
container_route_tree_engine="${CONTAINER_NS3_ROOT}/contrib/leosim/utils/rengine/leosim-rengine"
enable_synthetic_isl_load="${ENABLE_SYNTHETIC_ISL_LOAD:-1}"
enable_isl_load_logging="${ENABLE_ISL_LOAD_LOGGING:-0}"
synthetic_load_seed="${SYNTHETIC_LOAD_SEED:-12345}"
synthetic_load_min="${SYNTHETIC_LOAD_MIN:-0.1}"
synthetic_load_max="${SYNTHETIC_LOAD_MAX:-0.9}"
satellite_isl_capacity="${SATELLITE_ISL_CAPACITY:-30Gbps}"

for worker_setting in allocated_cpus tle_workers route_tree_workers; do
  worker_value="${!worker_setting}"
  if [[ ! "$worker_value" =~ ^[1-9][0-9]*$ ]]; then
    echo "${worker_setting} must be a positive integer, got '${worker_value}'." >&2
    exit 2
  fi
done
if (( route_tree_workers > allocated_cpus )); then
  echo "ROUTE_TREE_WORKERS=${route_tree_workers} exceeds the ${allocated_cpus} CPUs allocated by Slurm." >&2
  echo "Request more CPUs with sbatch --cpus-per-task or lower ROUTE_TREE_WORKERS." >&2
  exit 2
fi
if (( tle_workers > allocated_cpus )); then
  echo "TLE_WORKERS=${tle_workers} exceeds the ${allocated_cpus} CPUs allocated by Slurm." >&2
  echo "Request more CPUs with sbatch --cpus-per-task or lower TLE_WORKERS." >&2
  exit 2
fi

prepro_dir="${RESULTS_ROOT}/_preprocessed"
satellite_positions="${prepro_dir}/alpha-beta-gamma_duration-${sim_time}s_step-${tle_timestep}s.csv"
container_satellite_positions="${CONTAINER_RESULTS_ROOT}/_preprocessed/$(basename "$satellite_positions")"
prepro_lock="${satellite_positions}.lock"

run_id="metric-${metric}_sats-${satellites}_dynamic_duration-${sim_time}s-interval-${interval}s"
run_dir="${RESULTS_ROOT}/${run_id}"
container_prefix="${CONTAINER_RESULTS_ROOT}/${run_id}/result"
container_route_tree_dir="${CONTAINER_RESULTS_ROOT}/${run_id}/route-trees"
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
# Older container images use fixed /tmp filenames during the engine self-test.
# Serialize tests on each node so concurrent array tasks cannot truncate one
# another's binary fixtures. FD 8 is kept separate from the preprocessing lock.
exec 8>/tmp/leosim-rengine-selftest.lock
if ! flock -w 300 8; then
  echo "Timed out waiting for the routing-engine self-test lock on $(hostname)." >&2
  exit 2
fi
if ! apptainer exec --cleanenv "$SIF" "$container_route_tree_engine" --self-test; then
  flock -u 8
  echo "The container routing engine is missing or failed its threaded self-test: ${SIF}" >&2
  exit 2
fi
flock -u 8

# All array elements share one generated position file. The lock prevents
# simultaneous conversions; the first task generates it and the rest reuse it.
mkdir -p "$prepro_dir"
exec 9>"$prepro_lock"
flock -w 21600 9
if [[ ! -s "$satellite_positions" ]]; then
  temporary_positions="${satellite_positions}.tmp-${SLURM_ARRAY_JOB_ID}-${task_id}-$$"
  container_temporary_positions="${CONTAINER_RESULTS_ROOT}/_preprocessed/$(basename "$temporary_positions")"
  srun --ntasks=1 --cpus-per-task="${allocated_cpus}" --cpu-bind=cores \
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

scenario="leosim-experiments"
scenario+=" --satellites=${container_satellite_positions}"
scenario+=" --useTrace=0"
scenario+=" --numSatellites=${satellites}"
scenario+=" --numServers=${num_servers}"
scenario+=" --numUes=${num_ues}"
scenario+=" --allToAllTraffic=${all_to_all}"
scenario+=" --routingMetric=${metric}"
scenario+=" --useRouteTreeCache=${use_route_tree_cache}"
scenario+=" --routeTreeEngine=${container_route_tree_engine}"
scenario+=" --routeTreeWorkers=${route_tree_workers}"
scenario+=" --routeTreeWorkDir=${container_route_tree_dir}"
scenario+=" --routeTreeMaxEntries=${route_tree_max_entries}"
scenario+=" --enableDynamicRouting=1"
scenario+=" --routingUpdateInterval=${interval}"
scenario+=" --maxIslNeighbors=${max_isl_neighbors}"
scenario+=" --enableSyntheticIslLoad=${enable_synthetic_isl_load}"
scenario+=" --enableIslLoadLogging=${enable_isl_load_logging}"
scenario+=" --syntheticLoadSeed=${synthetic_load_seed}"
scenario+=" --syntheticLoadMin=${synthetic_load_min}"
scenario+=" --syntheticLoadMax=${synthetic_load_max}"
scenario+=" --satelliteIslCapacity=${satellite_isl_capacity}"
scenario+=" --simTime=${sim_time}"
scenario+=" --appStart=${app_start}"
scenario+=" --appStop=${app_stop}"
scenario+=" --tcpRate=${tcp_rate}"
scenario+=" --enableStatistics=1"
scenario+=" --statisticsInterval=${statistics_interval}"
scenario+=" --enableRouteLogging=0"
scenario+=" --enableHandoverLogging=${enable_handover_logging}"
scenario+=" --verbose=${verbose}"
scenario+=" --enableVisualization=0"
scenario+=" --writeFlowMonitor=1"
scenario+=" --outputPrefix=${container_prefix}"
command=(srun
  --ntasks=1
  --cpus-per-task="${allocated_cpus}"
  --cpu-bind=cores
  apptainer exec --cleanenv
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
  echo "slurm_cpus_per_task=${allocated_cpus}"
  echo "slurm_cpu_bind=cores"
  echo "external_route_tree_cache=${use_route_tree_cache}"
  echo "route_tree_engine=${container_route_tree_engine}"
  echo "route_tree_workers=${route_tree_workers}"
  echo "ns3_reserved_cpus=$((allocated_cpus - route_tree_workers))"
  echo "route_tree_max_entries=${route_tree_max_entries}"
  echo "route_tree_work_dir=${run_dir}/route-trees"
  echo "routing_update_interval_s=${interval}"
  echo "synthetic_isl_load=${enable_synthetic_isl_load}"
  echo "isl_load_logging=${enable_isl_load_logging}"
  echo "synthetic_load_seed=${synthetic_load_seed}"
  echo "synthetic_load_min=${synthetic_load_min}"
  echo "synthetic_load_max=${synthetic_load_max}"
  echo "satellite_isl_capacity=${satellite_isl_capacity}"
  echo "num_ues=${num_ues}"
  echo "num_servers=${num_servers}"
  echo "all_to_all_traffic=${all_to_all}"
  echo "tcp_rate_per_flow=${tcp_rate}"
  echo "handover_logging=${enable_handover_logging}"
  echo "handover_log=${run_dir}/result-handovers.csv"
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
