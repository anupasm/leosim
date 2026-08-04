#!/bin/bash -l
#SBATCH --job-name=leosim-aldsr
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=6
#SBATCH --time=24:00:00
#SBATCH --array=0-44%10
#SBATCH --output=logs/%x-%A_%a.out
#SBATCH --error=logs/%x-%A_%a.err

set -euo pipefail

if command -v module >/dev/null 2>&1; then
  module purge
  module load "${APPTAINER_MODULE:-apptainer/1.3.4-gcc-11.5.0-ojp6nts}"
fi

# Three controls followed by the six pre-registered ALDSR candidates.
POLICIES=(load distance hop combined combined combined combined combined combined)
LOAD_WEIGHTS=(1 0 0 0.70 0.60 0.50 0.40 0.40 0.30)
DISTANCE_WEIGHTS=(0 1 0 0.20 0.20 0.30 0.40 0.30 0.40)
LIFETIME_WEIGHTS=(0 0 0 0.05 0.15 0.15 0.15 0.25 0.25)
SNR_WEIGHTS=(0 0 0 0.00 0.00 0.00 0.00 0.00 0.00)
HOP_WEIGHTS=(0 0 1 0.05 0.05 0.05 0.05 0.05 0.05)
SEEDS=(1 2 3 4 5)

SATELLITE_COUNTS=(11514)
SIM_TIMES=(1200)
UPDATE_INTERVALS=(30)

REPO_ROOT="${REPO_ROOT:-${SLURM_SUBMIT_DIR}}"
SIF="${SIF:-${REPO_ROOT}/leosim.sif}"
RESULTS_ROOT="${ALDSR_RESULTS_ROOT:-${RESULTS_ROOT:-${REPO_ROOT}/results/leosim-aldsr/${SLURM_ARRAY_JOB_ID}}}"
if [[ "$RESULTS_ROOT" != /* ]]; then
  RESULTS_ROOT="${REPO_ROOT}/${RESULTS_ROOT}"
fi
CONTAINER_NS3_ROOT="/opt/leosim/ns3"
CONTAINER_RESULTS_ROOT="/results"
container_route_tree_engine="${CONTAINER_NS3_ROOT}/contrib/leosim/utils/rengine/leosim-rengine"

task_id=${SLURM_ARRAY_TASK_ID}
config_count=${#POLICIES[@]}
seed_count=${#SEEDS[@]}
matrix_size=$((config_count * seed_count))
if ((task_id < 0 || task_id >= matrix_size)); then
  echo "Array task ${task_id} is outside 0..$((matrix_size - 1))" >&2
  exit 2
fi

# Configuration varies fastest, so every consecutive group of nine is a paired seed block.
config_index=$((task_id % config_count))
seed_index=$((task_id / config_count))
policy=${POLICIES[$config_index]}
load_weight=${LOAD_WEIGHTS[$config_index]}
distance_weight=${DISTANCE_WEIGHTS[$config_index]}
lifetime_weight=${LIFETIME_WEIGHTS[$config_index]}
snr_weight=${SNR_WEIGHTS[$config_index]}
hop_weight=${HOP_WEIGHTS[$config_index]}
seed=${SEEDS[$seed_index]}
satellites=${ALDSR_SATELLITES:-${SATELLITE_COUNTS[0]}}
sim_time=${ALDSR_DURATION:-${SIM_TIMES[0]}}
interval=${ALDSR_UPDATE_INTERVAL:-${UPDATE_INTERVALS[0]}}

allocated_cpus=${SLURM_CPUS_PER_TASK:-1}
tle_workers=${TLE_WORKERS:-$allocated_cpus}
route_tree_workers=${ROUTE_TREE_WORKERS:-$allocated_cpus}
tle_timestep=${TLE_TIMESTEP:-5}
for worker_setting in allocated_cpus tle_workers route_tree_workers; do
  worker_value=${!worker_setting}
  if [[ ! "$worker_value" =~ ^[1-9][0-9]*$ ]]; then
    echo "${worker_setting} must be a positive integer, got '${worker_value}'." >&2
    exit 2
  fi
done
if ((route_tree_workers > allocated_cpus || tle_workers > allocated_cpus)); then
  echo "TLE and route-tree workers may not exceed allocated CPUs (${allocated_cpus})." >&2
  exit 2
fi

if ! command -v apptainer >/dev/null 2>&1; then
  echo "Apptainer is unavailable; load APPTAINER_MODULE first." >&2
  exit 2
fi
if [[ ! -f "$SIF" ]]; then
  echo "Missing LeoSim image: $SIF" >&2
  exit 2
fi

# Verify that this is the rebuilt image containing ALDSR, not an older routing image.
scenario_help=$(apptainer exec --cleanenv --pwd "$CONTAINER_NS3_ROOT" "$SIF" \
  ./ns3 run "leosim-experiments --PrintHelp" --no-build)
for required_option in routingMetric combinedLoadWeight combinedMinLifetime rngRun; do
  if ! grep -q -- "--${required_option}" <<<"$scenario_help"; then
    echo "Container lacks --${required_option}; rebuild it with run_all_experiments.sh build --rebuild-container." >&2
    exit 2
  fi
done

exec 8>/tmp/leosim-rengine-selftest.lock
flock -w 300 8
apptainer exec --cleanenv "$SIF" "$container_route_tree_engine" --self-test
flock -u 8

prepro_dir="${RESULTS_ROOT}/_preprocessed"
satellite_positions="${prepro_dir}/alpha-beta-gamma_duration-${sim_time}s_step-${tle_timestep}s.csv"
container_satellite_positions="${CONTAINER_RESULTS_ROOT}/_preprocessed/$(basename "$satellite_positions")"
mkdir -p "$prepro_dir" "${REPO_ROOT}/logs"
exec 9>"${satellite_positions}.lock"
flock -w 21600 9
if [[ ! -s "$satellite_positions" ]]; then
  temporary_positions="${satellite_positions}.tmp-${SLURM_ARRAY_JOB_ID}-${task_id}-$$"
  container_temporary_positions="${CONTAINER_RESULTS_ROOT}/_preprocessed/$(basename "$temporary_positions")"
  srun --ntasks=1 --cpus-per-task="$allocated_cpus" --cpu-bind=cores \
    apptainer exec --cleanenv --bind "${RESULTS_ROOT}:${CONTAINER_RESULTS_ROOT}" "$SIF" \
    /opt/venv/bin/python "${CONTAINER_NS3_ROOT}/contrib/leosim/utils/tle_to_positions.py" \
    --input "${CONTAINER_NS3_ROOT}/contrib/leosim/data/tles" \
    --output "$container_temporary_positions" --duration "$sim_time" \
    --timestep "$tle_timestep" --workers "$tle_workers"
  mv "$temporary_positions" "$satellite_positions"
fi
flock -u 9

weight_id="l-${load_weight}_d-${distance_weight}_t-${lifetime_weight}_s-${snr_weight}_h-${hop_weight}"
run_id="aldsr-${policy}_${weight_id}_seed-${seed}_sats-${satellites}_duration-${sim_time}s-interval-${interval}s"
run_dir="${RESULTS_ROOT}/${run_id}"
container_prefix="${CONTAINER_RESULTS_ROOT}/${run_id}/result"
container_route_tree_dir="${CONTAINER_RESULTS_ROOT}/${run_id}/route-trees"
mkdir -p "$run_dir"
if [[ -s "${run_dir}/result-statistics.json" && -s "${run_dir}/status.txt" ]]; then
  echo "Completed run already exists: $run_id"
  exit 0
fi

scenario="leosim-experiments"
scenario+=" --satellites=${container_satellite_positions} --useTrace=0"
scenario+=" --numSatellites=${satellites} --numServers=${NUM_SERVERS:-0} --numUes=${NUM_UES:-0}"
scenario+=" --rngSeed=1 --rngRun=${seed} --syntheticLoadSeed=${seed}"
scenario+=" --allToAllTraffic=${ALL_TO_ALL_TRAFFIC:-1} --routingMetric=${policy}"
scenario+=" --combinedLoadWeight=${load_weight} --combinedDistanceWeight=${distance_weight}"
scenario+=" --combinedLifetimeWeight=${lifetime_weight} --combinedSnrWeight=${snr_weight}"
scenario+=" --combinedHopWeight=${hop_weight} --combinedMaxDistance=${COMBINED_MAX_DISTANCE:-5000000}"
scenario+=" --combinedTargetLifetime=${COMBINED_TARGET_LIFETIME:-120}"
scenario+=" --combinedMinLifetime=${COMBINED_MIN_LIFETIME:-60}"
scenario+=" --combinedMinSnr=${COMBINED_MIN_SNR:-10} --combinedGoodSnr=${COMBINED_GOOD_SNR:-20}"
scenario+=" --combinedCriticalUtilization=${COMBINED_CRITICAL_UTILIZATION:-0.90}"
scenario+=" --useRouteTreeCache=1 --routeTreeEngine=${container_route_tree_engine}"
scenario+=" --routeTreeWorkers=${route_tree_workers} --routeTreeWorkDir=${container_route_tree_dir}"
scenario+=" --routeTreeMaxEntries=${ROUTE_TREE_MAX_ENTRIES:-10000000}"
scenario+=" --enableDynamicRouting=1 --routingUpdateInterval=${interval} --maxIslNeighbors=${MAX_ISL_NEIGHBORS:-4}"
scenario+=" --enableSyntheticIslLoad=1 --enableIslLoadLogging=${ENABLE_ISL_LOAD_LOGGING:-0}"
scenario+=" --syntheticLoadMin=${SYNTHETIC_LOAD_MIN:-0.1} --syntheticLoadMax=${SYNTHETIC_LOAD_MAX:-0.9}"
scenario+=" --satelliteIslCapacity=${SATELLITE_ISL_CAPACITY:-30Gbps}"
scenario+=" --simTime=${sim_time} --appStart=${APP_START:-1} --appStop=${APP_STOP:-$((sim_time - 1))}"
scenario+=" --tcpRate=${TCP_RATE:-1Mbps} --enableStatistics=1 --statisticsInterval=${STATISTICS_INTERVAL:-1}"
scenario+=" --enableRouteLogging=${ENABLE_ROUTE_LOGGING:-1}"
scenario+=" --enableHandoverLogging=${ENABLE_HANDOVER_LOGGING:-1}"
scenario+=" --verbose=${VERBOSE:-1} --enableVisualization=0 --writeFlowMonitor=1"
scenario+=" --outputPrefix=${container_prefix}"

command=(srun --ntasks=1 --cpus-per-task="$allocated_cpus" --cpu-bind=cores
  apptainer exec --cleanenv --bind "${RESULTS_ROOT}:${CONTAINER_RESULTS_ROOT}"
  --pwd "$CONTAINER_NS3_ROOT" "$SIF" ./ns3 run "$scenario" --no-build)

{
  echo "job_id=${SLURM_ARRAY_JOB_ID}_${task_id}"
  echo "run_id=${run_id}"
  echo "metric=${policy}"
  echo "rng_seed=1"
  echo "rng_run=${seed}"
  echo "synthetic_load_seed=${seed}"
  echo "combined_load_weight=${load_weight}"
  echo "combined_distance_weight=${distance_weight}"
  echo "combined_lifetime_weight=${lifetime_weight}"
  echo "combined_snr_weight=${snr_weight}"
  echo "combined_hop_weight=${hop_weight}"
  echo "satellites=${satellites}"
  echo "sim_time_s=${sim_time}"
  echo "routing_update_interval_s=${interval}"
  echo "route_tree_workers=${route_tree_workers}"
  echo "satellite_positions_host=${satellite_positions}"
  echo "container_image=${SIF}"
  echo "host=$(hostname)"
  printf 'command='; printf ' %q' "${command[@]}"; printf '\n'
} | tee "${run_dir}/metadata.txt"

if [[ -x /usr/bin/time ]]; then
  /usr/bin/time -v -o "${run_dir}/resources.txt" "${command[@]}"
else
  "${command[@]}"
fi
echo PASS >"${run_dir}/status.txt"
