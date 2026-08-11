#!/usr/bin/env bash
#SBATCH --job-name=leosim-ho
#SBATCH --array=0-5%6
#SBATCH --time=48:00:00
#SBATCH --cpus-per-task=4
#SBATCH --output=/scratch/adesilva/logs/leosim-ho-%A_%a.out
#SBATCH --error=/scratch/adesilva/logs/leosim-ho-%A_%a.err

# SONIC Slurm array for the 3 policy x 2 buffering design. Each
# policy/buffer treatment uses the same 5 Mbps per-flow traffic load and 12
# ground nodes: two GSS and two UEs from each of alpha, beta, and gamma. Each
# task uses the same fixed RNG run for paired comparisons.
# Build leosim-experiments once on a SONIC login/build node before
# submitting this script.  The array deliberately uses --no-build so concurrent
# tasks never modify the shared ns-3 build tree.

set -euo pipefail

export TMPDIR="/scratch/adesilva/tmp/${SLURM_JOB_ID}-${SLURM_ARRAY_TASK_ID}"
export APPTAINER_CACHEDIR="/scratch/adesilva/cache/apptainer"
export APPTAINER_TMPDIR="${TMPDIR}/apptainer"
export XDG_CACHE_HOME="/scratch/adesilva/cache"
mkdir -p "$TMPDIR" "$APPTAINER_CACHEDIR" "$APPTAINER_TMPDIR" /scratch/adesilva/logs

if command -v module >/dev/null 2>&1; then
    module purge
    module load "${APPTAINER_MODULE:-apptainer/1.3.4-gcc-11.5.0-ojp6nts}" 2>/dev/null || true
fi

if [[ -z "${SLURM_ARRAY_TASK_ID:-}" ]]; then
    echo "This file must be submitted with sbatch." >&2
    exit 2
fi

SATELLITE_COUNT=5000
SATELLITE_COUNTS=${SATELLITE_COUNT}
SIM_TIME=1800
UPDATE_INTERVAL=30
GROUND_DEVICES_PER_OPERATOR=${LEOSIM_GROUND_DEVICES_PER_OPERATOR:-2}
NUM_UES=${LEOSIM_NUM_UES:-$((3 * GROUND_DEVICES_PER_OPERATOR))}
NUM_SERVERS=${LEOSIM_NUM_SERVERS:-$((3 * GROUND_DEVICES_PER_OPERATOR))}


script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ns3_dir="${LEOSIM_NS3_ROOT:-}"
if [[ -n "${ns3_dir}" ]]; then
    if [[ ! -x "${ns3_dir}/ns3" && ! ( -d "${ns3_dir}/ns3" && -x "${ns3_dir}/ns3/ns3" ) ]]; then
        echo "Ignoring invalid LEOSIM_NS3_ROOT=${ns3_dir}; resolving the actual repo checkout instead." >&2
        ns3_dir=""
    fi
fi
if [[ -z "${ns3_dir}" ]]; then
    current="${script_dir}"
    while [[ "${current}" != "/" ]]; do
        if [[ -x "${current}/ns3" && -d "${current}/contrib/leosim" ]]; then
            ns3_dir="${current}"
            break
        fi
        if [[ -x "${current}/ns3/ns3" && -d "${current}/contrib/leosim" ]]; then
            ns3_dir="${current}/ns3"
            break
        fi
        if [[ -f "${current}/CMakeLists.txt" && -d "${current}/model" && -d "${current}/helper" ]]; then
            ns3_dir="${current}"
            break
        fi
        current="$(dirname "${current}")"
    done
fi
if [[ -z "${ns3_dir}" ]]; then
    ns3_dir="$(cd "${script_dir}/../../.." 2>/dev/null || pwd)"
fi
repo_root="${LEOSIM_REPO_ROOT:-${ns3_dir}}"
if [[ -z "${repo_root}" || "${repo_root}" == "/" || "${repo_root}" == "/ns3" ]]; then
    repo_root="$(pwd)"
fi
CONTAINER_ROOT="/opt/leosim"
CONTAINER_NS3_ROOT="${CONTAINER_ROOT}/ns3"
if [[ -z "${SIF:-}" ]]; then
    SIF="/scratch/adesilva/leosim.sif"
fi
if [[ "${SIF}" == "/" || "${SIF}" == "/leosim.sif" ]]; then
    SIF="/scratch/adesilva/leosim.sif"
fi
case "${SIF}" in
    /scratch/adesilva/*) ;;
    *) echo "SIF must be under /scratch/adesilva: ${SIF}" >&2; exit 2 ;;
esac
base_results_root="${LEOSIM_RESULTS_ROOT:-/scratch/adesilva/handover/results}"
case "${base_results_root}" in
    /scratch/adesilva|/scratch/adesilva/*) ;;
    *) echo "LEOSIM_RESULTS_ROOT must be under /scratch/adesilva: ${base_results_root}" >&2; exit 2 ;;
esac
results_root="${base_results_root}/${SLURM_ARRAY_JOB_ID}"

# Decode the six paired tasks. Adjacent IDs compare buffering off/on for the
# same handover policy, with one common random run across the campaign.
index=${SLURM_ARRAY_TASK_ID}
buffering=$((index % 2))
index=$((index / 2))
policy_index=$((index % 3))
rng_run=1

policies=(bho cho1 cho3)
load=5Mbps
policy=${policies[policy_index]}

case "${policy}" in
    bho)
        ho_mode=BHO
        max_candidates=1
        ;;
    cho1)
        ho_mode=CHO
        max_candidates=1
        ;;
    cho3)
        ho_mode=CHO
        max_candidates=3
        ;;
    *)
        echo "Internal policy decoding error: ${policy}" >&2
        exit 2
        ;;
esac

run_id="policy-${policy}_load-${load}_buffer-${buffering}_run-${rng_run}"
run_dir="${results_root}/${run_id}"
output_stem="result-job${SLURM_ARRAY_JOB_ID}-task${SLURM_ARRAY_TASK_ID}-restart${SLURM_RESTART_COUNT:-0}-pid$$"
output_prefix="${run_dir}/${output_stem}"
route_tree_dir="/scratch/adesilva/tmp/leosim-route-trees-${SLURM_JOB_ID}-${SLURM_ARRAY_TASK_ID}"
if [[ -d "${repo_root}/data" ]]; then
    data_root="${repo_root}/data"
elif [[ -d "${PWD}/data" ]]; then
    data_root="${PWD}/data"
else
    data_root="${PWD}/data"
fi
prepro_dir="${results_root}/_preprocessed"
mkdir -p "${run_dir}" "${route_tree_dir}" "${prepro_dir}"

exec 9>"${prepro_dir}/.prepare.lock"
flock 9
trace_file="${prepro_dir}/satellite_mobility.tcl"
trace_names_file="${trace_file}.names.csv"
trace_ready_file="${prepro_dir}/.satellite_mobility.ready"
trace_is_ready=false
if [[ -s "${trace_file}" && -s "${trace_names_file}" && -s "${trace_ready_file}" ]]; then
    prepared_satellites=$(awk 'END { print (NR > 0 ? NR - 1 : 0) }' "${trace_names_file}")
    if [[ "${prepared_satellites}" -ge "${SATELLITE_COUNT}" ]]; then
        trace_is_ready=true
    fi
fi
if [[ "${trace_is_ready}" != true ]]; then
    if ! command -v apptainer >/dev/null 2>&1; then
        echo "Apptainer is unavailable; load the site Apptainer module or set APPTAINER_MODULE." >&2
        exit 2
    fi
    if [[ ! -f "${SIF}" ]]; then
        echo "Missing LeoSim container image: ${SIF}" >&2
        echo "Build it with utils/sonic/build_leosim_container.sh or set SIF explicitly." >&2
        exit 2
    fi
    if ! apptainer exec --cleanenv \
        --bind "${repo_root}:${repo_root}" \
        --bind "${data_root}:${data_root}" \
        --bind "${results_root}:${results_root}" \
        "${SIF}" \
        /opt/venv/bin/python -c 'import numpy, skyfield' >/dev/null 2>&1; then
        echo "Container Python is missing NumPy/Skyfield: ${SIF}" >&2
        exit 2
    fi

    staging_dir=$(mktemp -d "${prepro_dir}/.prepare-${SLURM_ARRAY_JOB_ID}-XXXXXX")
    cleanup_staging() {
        rm -rf -- "${staging_dir}"
    }
    trap cleanup_staging EXIT
    srun --ntasks=1 --cpus-per-task="${SLURM_CPUS_PER_TASK:-8}" --cpu-bind=cores \
        apptainer exec --cleanenv \
        --bind "${repo_root}:${repo_root}" \
        --bind "${data_root}:${data_root}" \
        --bind "${results_root}:${results_root}" \
        "${SIF}" \
        /opt/venv/bin/python "${CONTAINER_NS3_ROOT}/contrib/leosim/utils/tle_to_positions.py" \
        --input "${data_root}/tles" \
        --output "${staging_dir}" \
        --duration "${SIM_TIME}" \
        --timestep "${UPDATE_INTERVAL}" \
        --workers "${SLURM_CPUS_PER_TASK:-8}" \
        --ns3-format >/dev/null
    staged_trace="${staging_dir}/satellite_mobility.tcl"
    staged_names="${staged_trace}.names.csv"
    if [[ ! -s "${staged_trace}" || ! -s "${staged_names}" ]]; then
        echo "Preprocessing did not produce a complete mobility trace." >&2
        exit 3
    fi
    prepared_satellites=$(awk 'END { print (NR > 0 ? NR - 1 : 0) }' "${staged_names}")
    if [[ "${prepared_satellites}" -lt "${SATELLITE_COUNT}" ]]; then
        echo "Mobility trace contains ${prepared_satellites} satellites; ${SATELLITE_COUNT} required." >&2
        exit 3
    fi
    # Publish data first and the ready marker last. A task can therefore never
    # mistake a partially written shared trace for completed preprocessing.
    mv -f -- "${staged_trace}" "${trace_file}"
    mv -f -- "${staged_names}" "${trace_names_file}"
    if [[ -s "${staging_dir}/satellite_positions.csv" ]]; then
        mv -f -- "${staging_dir}/satellite_positions.csv" "${prepro_dir}/satellite_positions.csv"
    fi
    printf 'satellites=%s\n' "${prepared_satellites}" > "${trace_ready_file}.tmp.$$"
    mv -f -- "${trace_ready_file}.tmp.$$" "${trace_ready_file}"
    trap - EXIT
    cleanup_staging
fi
flock -u 9

if [[ -x "${ns3_dir}/ns3" ]]; then
    cd "${ns3_dir}"
    ns3_cmd=("${ns3_dir}/ns3" run --no-build)
    route_tree_engine="${ns3_dir}/contrib/leosim/utils/rengine/leosim-rengine"
    echo "Using host ns-3 at ${ns3_dir}/ns3"
else
    if ! command -v apptainer >/dev/null 2>&1; then
        echo "ns-3 launcher not found at ${ns3_dir}/ns3" >&2
        echo "Apptainer is unavailable, so no containerized runtime can be used." >&2
        exit 2
    fi
    if [[ ! -f "${SIF}" ]]; then
        echo "ns-3 launcher not found at ${ns3_dir}/ns3" >&2
        echo "Container image missing: ${SIF}" >&2
        echo "Build it with utils/sonic/build_leosim_container.sh or set SIF explicitly." >&2
        exit 2
    fi
    mkdir -p "${results_root}"
    ns3_cmd=(apptainer exec --cleanenv \
        --bind "${results_root}:${results_root}" \
        --bind "${route_tree_dir}:${route_tree_dir}" \
        --pwd /opt/leosim/ns3 "${SIF}" ./ns3 run --no-build)
    route_tree_engine="${CONTAINER_NS3_ROOT}/contrib/leosim/utils/rengine/leosim-rengine"
    echo "Using containerized ns-3 from ${SIF}"
fi

if [[ -x "${ns3_dir}/ns3" ]]; then
    if [[ ! -x "${route_tree_engine}" ]] || ! "${route_tree_engine}" --self-test >/dev/null; then
        echo "Missing or invalid routing engine: ${route_tree_engine}" >&2
        echo "Run contrib/leosim/utils/sonic/build_leosim.sh before submitting the array." >&2
        exit 2
    fi
else
    if ! apptainer exec --cleanenv \
        --bind "${route_tree_dir}:${route_tree_dir}" \
        --pwd /opt/leosim/ns3 "${SIF}" \
        "${route_tree_engine}" --self-test >/dev/null; then
        echo "Container routing engine failed its self-test: ${route_tree_engine}" >&2
        exit 2
    fi
    if ! apptainer exec --cleanenv \
        --bind "${route_tree_dir}:${route_tree_dir}" \
        "${SIF}" test -d "${route_tree_dir}" -a -w "${route_tree_dir}"; then
        echo "Container cannot write routing work directory: ${route_tree_dir}" >&2
        exit 2
    fi
fi

export OMP_NUM_THREADS="${SLURM_CPUS_PER_TASK:-8}"
export NS_GLOBAL_VALUE="RngSeed=${LEOSIM_RNG_SEED:-20260803};RngRun=${rng_run}"

satellite_trace="${trace_file}"
if [[ -d "${repo_root}/contrib/leosim/data" ]]; then
    data_dir="${repo_root}/contrib/leosim/data"
elif [[ -d "${data_root}" ]]; then
    data_dir="${data_root}"
else
    data_dir="${PWD}/data"
fi

command_line="leosim-experiments \
--dataDir=${data_dir} \
--satellites=${satellite_trace} \
--useTrace=1 \
--simTime=${SIM_TIME} \
--numSatellites=${SATELLITE_COUNTS} \
--numUes=${NUM_UES} \
--numServers=${NUM_SERVERS} \
--groundDevicesPerOperator=${GROUND_DEVICES_PER_OPERATOR} \
--appStart=60 --appStop=3599 \
--tcpRate=${load} --tcpPacketSize=512 --allToAllTraffic=1 \
--enableIsl=1 --maxIslNeighbors=4 \
--enableDynamicRouting=1 --routingUpdateInterval=10 \
--useRouteTreeCache=1 --routeTreeWorkers=${SLURM_CPUS_PER_TASK:-8} \
--routeTreeEngine=${route_tree_engine} \
--routeTreeWorkDir=${route_tree_dir} \
--hoMode=${ho_mode} --maxCandidates=${max_candidates} \
--enableHoBuffering=${buffering} --hoBufferSize=${LEOSIM_HO_BUFFER_PACKETS:-1024} \
--enableLoadBalancing=0 \
--ttt=1 --t310=1 --n310=3 --n311=3 \
--a3Offset=3 --a4Threshold=-110 --tteTrigger=30 \
--choPrep=100 --choExec=150 --beamUpdateIntervalMs=100 \
--enableStatistics=1 --statisticsInterval=1 \
--enableHandoverLogging=1 --writeFlowMonitor=1 \
--enableTaskProfiler=1 --progressLogInterval=30 \
--outputPrefix=${output_prefix} --uniqueOutputPrefix=0"

{
    echo "run_id=${run_id}"
    echo "output_prefix=${output_prefix}"
    echo "job_id=${SLURM_JOB_ID}"
    echo "array_task_id=${SLURM_ARRAY_TASK_ID}"
    echo "hostname=$(hostname)"
    echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "ns_global_value=${NS_GLOBAL_VALUE}"
    echo "command=${command_line}"
    git -C "${repo_root}" rev-parse HEAD 2>/dev/null || echo "commit=unavailable"
    if [[ -f "${repo_root}/contrib/leosim/data/prepro/satellite_mobility.tcl" ]]; then
        sha256sum "${repo_root}/contrib/leosim/data/prepro/satellite_mobility.tcl"
    elif [[ -f "${repo_root}/data/tles/alpha.csv" ]]; then
        sha256sum "${repo_root}/data/tles/alpha.csv"
    elif [[ -f "${repo_root}/CMakeLists.txt" ]]; then
        sha256sum "${repo_root}/CMakeLists.txt"
    else
        echo "artifact_sha256=unavailable"
    fi
} > "${run_dir}/metadata.txt"

set +e
if command -v /usr/bin/time >/dev/null 2>&1; then
    /usr/bin/time -v "${ns3_cmd[@]}" "${command_line}" \
        > "${run_dir}/stdout.log" 2> "${run_dir}/stderr.log"
else
    time "${ns3_cmd[@]}" "${command_line}" \
        > "${run_dir}/stdout.log" 2> "${run_dir}/stderr.log"
fi
status=$?
set -e

echo "${status}" > "${run_dir}/exit-status.txt"
date -u +%Y-%m-%dT%H:%M:%SZ > "${run_dir}/finished-utc.txt"

if [[ ${status} -ne 0 ]]; then
    echo "LeoSim run failed with status ${status}; see ${run_dir}/stderr.log" >&2
    exit "${status}"
fi

required=(
    "${output_prefix}-statistics.csv"
    "${output_prefix}-statistics.json"
    "${output_prefix}-handovers.csv"
    "${output_prefix}-flowmon.xml"
)
for output in "${required[@]}"; do
    if [[ ! -s "${output}" ]]; then
        echo "Required output missing or empty: ${output}" >&2
        echo 3 > "${run_dir}/exit-status.txt"
        exit 3
    fi
done

summary_json="${output_prefix}-statistics.json"
tx_packets=$(sed -n 's/.*"network": {"tx_packets": \([0-9][0-9]*\).*/\1/p' "${summary_json}")
sink_count=$(sed -n 's/.*"sink_count": \([0-9][0-9]*\).*/\1/p' "${summary_json}")
sinks_with_rx=$(sed -n 's/.*"sinks_with_rx": \([0-9][0-9]*\).*/\1/p' "${summary_json}")
valid_routes=$(sed -n 's/.*"routing": {"samples": [0-9][0-9]*, "valid_samples": \([0-9][0-9]*\).*/\1/p' "${summary_json}")
expected_sinks=$(( NUM_UES * NUM_SERVERS ))
if [[ -z "${tx_packets}" || -z "${sink_count}" || -z "${sinks_with_rx}" || -z "${valid_routes}" ]]; then
    echo "Could not validate statistics summary: ${summary_json}" >&2
    echo 4 > "${run_dir}/exit-status.txt"
    exit 4
fi
if [[ "${sink_count}" -ne "${expected_sinks}" ]]; then
    echo "Expected ${expected_sinks} traffic sinks, observed ${sink_count}." >&2
    echo 4 > "${run_dir}/exit-status.txt"
    exit 4
fi
if [[ "${tx_packets}" -eq 0 || "${sinks_with_rx}" -eq 0 || "${valid_routes}" -eq 0 ]]; then
    echo "Invalid completed run: tx_packets=${tx_packets}, sinks_with_rx=${sinks_with_rx}, valid_routes=${valid_routes}." >&2
    echo 4 > "${run_dir}/exit-status.txt"
    exit 4
fi

echo "Completed ${run_id}: ${run_dir}"
