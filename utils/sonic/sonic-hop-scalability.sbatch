#!/usr/bin/env bash
#SBATCH --job-name=leosim-hop-scale
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --array=0-2%3
#SBATCH --time=48:00:00
#SBATCH --output=/scratch/adesilva/logs/%x-%A_%a.out
#SBATCH --error=/scratch/adesilva/logs/%x-%A_%a.err

# Hop-routing scalability campaign: three concurrent array tasks use 12 CPUs.
# Together with ALDSR (27) and handover (6), the full campaign uses 45 of the
# server's 47 CPUs. Each task has six UEs and six GSSs.
# Build the container/experiment before submission; runs use --no-build.

set -euo pipefail

SATELLITE_COUNTS=(1000 5000 10000)
task_id=${SLURM_ARRAY_TASK_ID:?Submit this script with sbatch}
if (( task_id < 0 || task_id >= ${#SATELLITE_COUNTS[@]} )); then
    echo "Array task ${task_id} is outside 0..$((${#SATELLITE_COUNTS[@]} - 1))" >&2
    exit 2
fi

satellites=${SATELLITE_COUNTS[task_id]}
allocated_cpus=${SLURM_CPUS_PER_TASK:-4}
sim_time=${LEOSIM_SIM_TIME:-1200}
update_interval=${LEOSIM_ROUTING_UPDATE_INTERVAL:-30}
tle_timestep=${LEOSIM_TLE_TIMESTEP:-5}
num_ues=6
num_servers=6

export TMPDIR="/scratch/adesilva/tmp/${SLURM_JOB_ID}-${task_id}"
export APPTAINER_CACHEDIR="/scratch/adesilva/cache/apptainer"
export APPTAINER_TMPDIR="${TMPDIR}/apptainer"
export XDG_CACHE_HOME="/scratch/adesilva/cache"
export OMP_NUM_THREADS=${allocated_cpus}
mkdir -p "${TMPDIR}" "${APPTAINER_CACHEDIR}" "${APPTAINER_TMPDIR}" /scratch/adesilva/logs

if command -v module >/dev/null 2>&1; then
    module purge
    module load "${APPTAINER_MODULE:-apptainer/1.3.4-gcc-11.5.0-ojp6nts}"
fi

repo_root=${LEOSIM_REPO_ROOT:-${SLURM_SUBMIT_DIR:-$PWD}}
sif=${SIF:-/scratch/adesilva/leosim.sif}
base_results_root=${LEOSIM_RESULTS_ROOT:-/scratch/adesilva/results/leosim-hop-scalability}
results_root="${base_results_root}/${SLURM_ARRAY_JOB_ID}"
container_ns3_root=/opt/leosim/ns3
container_results_root=/results
route_tree_engine="${container_ns3_root}/contrib/leosim/utils/rengine/leosim-rengine"

case "${sif}" in
    /scratch/adesilva/*) ;;
    *) echo "SIF must be under /scratch/adesilva: ${sif}" >&2; exit 2 ;;
esac
case "${base_results_root}" in
    /scratch/adesilva|/scratch/adesilva/*) ;;
    *) echo "LEOSIM_RESULTS_ROOT must be under /scratch/adesilva: ${base_results_root}" >&2; exit 2 ;;
esac
if [[ ! "${allocated_cpus}" =~ ^[1-9][0-9]*$ ]]; then
    echo "SLURM_CPUS_PER_TASK must be a positive integer: ${allocated_cpus}" >&2
    exit 2
fi
if ! command -v apptainer >/dev/null 2>&1 || [[ ! -f "${sif}" ]]; then
    echo "Apptainer or LeoSim image is unavailable: ${sif}" >&2
    exit 2
fi

run_id="metric-hop_sats-${satellites}_ues-${num_ues}_gss-${num_servers}_duration-${sim_time}s-interval-${update_interval}s"
run_dir="${results_root}/${run_id}"
output_stem="result-job${SLURM_ARRAY_JOB_ID}-task${task_id}-restart${SLURM_RESTART_COUNT:-0}-pid$$"
container_prefix="${container_results_root}/${run_id}/${output_stem}"
container_route_tree_dir="${container_results_root}/${run_id}/route-trees"
mkdir -p "${run_dir}"

# The three tasks share the same 10,000-satellite position trace.  flock makes
# one task generate it while the others wait and then reuse it.
prepro_dir="${results_root}/_preprocessed"
positions="${prepro_dir}/alpha-beta-gamma_duration-${sim_time}s_step-${tle_timestep}s.csv"
container_positions="${container_results_root}/_preprocessed/$(basename "${positions}")"
mkdir -p "${prepro_dir}"
exec 9>"${positions}.lock"
flock -w 21600 9
if [[ ! -s "${positions}" ]]; then
    temporary_positions="${positions}.tmp-${SLURM_ARRAY_JOB_ID}-${task_id}-$$"
    container_temporary_positions="${container_results_root}/_preprocessed/$(basename "${temporary_positions}")"
    srun --exclusive --ntasks=1 --cpus-per-task="${allocated_cpus}" --cpu-bind=cores \
        apptainer exec --cleanenv \
        --bind "${results_root}:${container_results_root}" \
        "${sif}" \
        /opt/venv/bin/python "${container_ns3_root}/contrib/leosim/utils/tle_to_positions.py" \
        --input "${container_ns3_root}/contrib/leosim/data/tles" \
        --output "${container_temporary_positions}" \
        --duration "${sim_time}" --timestep "${tle_timestep}" \
        --workers "${allocated_cpus}"
    mv -- "${temporary_positions}" "${positions}"
fi
flock -u 9

scenario="leosim-experiments"
scenario+=" --satellites=${container_positions} --useTrace=0"
scenario+=" --numSatellites=${satellites} --numServers=${num_servers} --numUes=${num_ues}"
scenario+=" --allToAllTraffic=1 --routingMetric=hop"
scenario+=" --useRouteTreeCache=1 --routeTreeEngine=${route_tree_engine}"
scenario+=" --routeTreeWorkers=${allocated_cpus} --routeTreeWorkDir=${container_route_tree_dir}"
scenario+=" --routeTreeMaxEntries=${LEOSIM_ROUTE_TREE_MAX_ENTRIES:-10000000}"
scenario+=" --enableIsl=1 --maxIslNeighbors=4"
scenario+=" --enableDynamicRouting=1 --routingUpdateInterval=${update_interval}"
scenario+=" --simTime=${sim_time} --appStart=1 --appStop=$((sim_time - 1))"
scenario+=" --tcpRate=${LEOSIM_TCP_RATE:-1Mbps} --tcpPacketSize=512"
scenario+=" --enableStatistics=1 --statisticsInterval=1"
scenario+=" --enableRouteLogging=0 --enableHandoverLogging=1"
scenario+=" --enableTaskProfiler=1 --progressLogInterval=30"
scenario+=" --enableVisualization=0 --writeFlowMonitor=1 --verbose=1"
scenario+=" --outputPrefix=${container_prefix} --uniqueOutputPrefix=0"

command=(srun --exclusive --ntasks=1 --cpus-per-task="${allocated_cpus}" --cpu-bind=cores
    apptainer exec --cleanenv
    --bind "${results_root}:${container_results_root}"
    --pwd "${container_ns3_root}" "${sif}"
    ./ns3 run --no-build "${scenario}")

{
    echo "run_id=${run_id}"
    echo "job_id=${SLURM_ARRAY_JOB_ID}_${task_id}"
    echo "satellites=${satellites}"
    echo "routing_metric=hop"
    echo "num_ues=${num_ues}"
    echo "num_ground_station_servers=${num_servers}"
    echo "total_ground_nodes=$((num_ues + num_servers))"
    echo "sim_time_s=${sim_time}"
    echo "routing_update_interval_s=${update_interval}"
    echo "slurm_cpus_per_task=${allocated_cpus}"
    echo "concurrent_array_tasks=3"
    echo "campaign_cpus=12"
    echo "hostname=$(hostname)"
    echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "git_commit=$(git -C "${repo_root}" rev-parse HEAD 2>/dev/null || echo embedded-in-image)"
    printf 'command='; printf ' %q' "${command[@]}"; printf '\n'
} > "${run_dir}/metadata.txt"

set +e
if [[ -x /usr/bin/time ]]; then
    /usr/bin/time -v -o "${run_dir}/resources-time.txt" "${command[@]}" \
        > "${run_dir}/stdout.log" 2> "${run_dir}/stderr.log"
else
    "${command[@]}" > "${run_dir}/stdout.log" 2> "${run_dir}/stderr.log"
fi
status=$?
set -e

echo "${status}" > "${run_dir}/exit-status.txt"
date -u +%Y-%m-%dT%H:%M:%SZ > "${run_dir}/finished-utc.txt"

# Slurm accounting adds CPU efficiency, RSS/VM size, disk I/O and energy when
# the cluster's jobacct_gather and energy plugins expose those fields. Energy
# may be "N/A"; node-level energy is not an isolated per-process measurement.
if command -v sacct >/dev/null 2>&1; then
    accounting_fields="JobIDRaw,JobName,State,ExitCode,ElapsedRaw,TotalCPU,UserCPU,SystemCPU,CPUTimeRAW,AllocCPUS,AveCPU,MaxRSS,MaxVMSize,AveRSS,MaxDiskRead,MaxDiskWrite,ConsumedEnergyRaw"
    # Keep step records: the completed srun step contains the useful resource
    # counters while the enclosing array task is still finalising.
    sacct -j "${SLURM_ARRAY_JOB_ID}_${task_id}" -P \
        --format="${accounting_fields}" > "${run_dir}/slurm-accounting.psv" 2> "${run_dir}/slurm-accounting.err" || true
fi

if (( status != 0 )); then
    echo "LeoSim run failed with status ${status}; see ${run_dir}/stderr.log" >&2
    exit "${status}"
fi

required=(
    "${run_dir}/${output_stem}-statistics.csv"
    "${run_dir}/${output_stem}-statistics.json"
    "${run_dir}/${output_stem}-flowmon.xml"
)
for output in "${required[@]}"; do
    if [[ ! -s "${output}" ]]; then
        echo "Required output missing or empty: ${output}" >&2
        echo 3 > "${run_dir}/exit-status.txt"
        exit 3
    fi
done

echo "PASS" > "${run_dir}/status.txt"
echo "Completed ${run_id}: ${run_dir}"
