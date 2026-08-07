#!/usr/bin/env bash
#SBATCH --job-name=leosim-ho
#SBATCH --array=0-89%12
#SBATCH --time=48:00:00
#SBATCH --cpus-per-task=4
#SBATCH --output=logs/leosim-ho-%A_%a.out
#SBATCH --error=logs/leosim-ho-%A_%a.err

# SONIC Slurm array for the 3 policy x 3 load x 2 buffering x 5 run design.
# Build leosim-experiments once on a SONIC login/build node before
# submitting this script.  The array deliberately uses --no-build so concurrent
# tasks never modify the shared ns-3 build tree.

set -euo pipefail

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
    if [[ -d "/scratch/adesilva" && -w "/scratch/adesilva" ]]; then
        SIF="/scratch/adesilva/leosim.sif"
    elif [[ -d "${HOME}" && -w "${HOME}" ]]; then
        SIF="${HOME}/leosim.sif"
    else
        SIF="${repo_root}/leosim.sif"
    fi
fi
if [[ "${SIF}" == "/" || "${SIF}" == "/leosim.sif" ]]; then
    SIF="/scratch/adesilva/leosim.sif"
fi
base_results_root="${LEOSIM_RESULTS_ROOT:-/scratch/adesilva/handover/results}"
results_root="${base_results_root}/${SLURM_ARRAY_JOB_ID}"

# Decode the complete factorial array. Adjacent IDs vary the random run first,
# keeping every five-task block a directly comparable treatment.
index=${SLURM_ARRAY_TASK_ID}
rng_run=$((index % 5 + 1))
index=$((index / 5))
buffering=$((index % 2))
index=$((index / 2))
load_index=$((index % 3))
policy_index=$((index / 3))

loads=(250Kbps 1Mbps 4Mbps)
policies=(bho cho1 cho3)
load=${loads[load_index]}
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
route_tree_dir="${TMPDIR:-/tmp}/leosim-route-trees-${SLURM_JOB_ID}-${SLURM_ARRAY_TASK_ID}"
if [[ -d "${repo_root}/data" ]]; then
    data_root="${repo_root}/data"
elif [[ -d "${PWD}/data" ]]; then
    data_root="${PWD}/data"
else
    data_root="${PWD}/data"
fi
prepro_dir="${data_root}/prepro"
mkdir -p "${run_dir}" "${route_tree_dir}" "${prepro_dir}"

if [[ ! -s "${prepro_dir}/satellite_mobility.tcl" ]]; then
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

    srun --ntasks=1 --cpus-per-task="${SLURM_CPUS_PER_TASK:-8}" --cpu-bind=cores \
        apptainer exec --cleanenv \
        --bind "${repo_root}:${repo_root}" \
        --bind "${data_root}:${data_root}" \
        --bind "${results_root}:${results_root}" \
        "${SIF}" \
        /opt/venv/bin/python "${CONTAINER_NS3_ROOT}/contrib/leosim/utils/tle_to_positions.py" \
        --input "${data_root}/tles" \
        --output "${prepro_dir}" \
        --duration "${SIM_TIME}" \
        --timestep "${UPDATE_INTERVAL}" \
        --workers "${SLURM_CPUS_PER_TASK:-8}" \
        --ns3-format >/dev/null
fi

if [[ -x "${ns3_dir}/ns3" ]]; then
    cd "${ns3_dir}"
    ns3_cmd=("${ns3_dir}/ns3" run --no-build)
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
    ns3_cmd=(apptainer exec --cleanenv --bind "${results_root}:${results_root}" --pwd /opt/leosim/ns3 "${SIF}" ./ns3 run --no-build)
    echo "Using containerized ns-3 from ${SIF}"
fi

export OMP_NUM_THREADS="${SLURM_CPUS_PER_TASK:-8}"
export NS_GLOBAL_VALUE="RngSeed=${LEOSIM_RNG_SEED:-20260803};RngRun=${rng_run}"

satellite_trace="${prepro_dir}/satellite_mobility.tcl"
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
--numUes=${LEOSIM_NUM_UES:-20} \
--numServers=${LEOSIM_NUM_SERVERS:-4} \
--appStart=60 --appStop=3599 \
--tcpRate=${load} --tcpPacketSize=512 --allToAllTraffic=1 \
--enableIsl=1 --maxIslNeighbors=4 \
--enableDynamicRouting=1 --routingUpdateInterval=10 \
--useRouteTreeCache=1 --routeTreeWorkers=${SLURM_CPUS_PER_TASK:-8} \
--routeTreeWorkDir=${route_tree_dir} \
--hoMode=${ho_mode} --maxCandidates=${max_candidates} \
--enableHoBuffering=${buffering} --enableLoadBalancing=0 \
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
        exit 3
    fi
done

echo "Completed ${run_id}: ${run_dir}"
