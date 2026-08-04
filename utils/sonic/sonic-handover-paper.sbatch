#!/usr/bin/env bash
#SBATCH --job-name=leosim-ho
#SBATCH --array=0-179%12
#SBATCH --time=48:00:00
#SBATCH --cpus-per-task=8
#SBATCH --mem=64G
#SBATCH --output=leosim-ho-%A_%a.out
#SBATCH --error=leosim-ho-%A_%a.err

# SONIC Slurm array for the 3 policy x 3 load x 2 buffering x 10 run design.
# Build leosim-experiments once on a SONIC login/build node before
# submitting this script.  The array deliberately uses --no-build so concurrent
# tasks never modify the shared ns-3 build tree.

set -euo pipefail

if [[ -z "${SLURM_ARRAY_TASK_ID:-}" ]]; then
    echo "This file must be submitted with sbatch." >&2
    exit 2
fi

SATELLITE_COUNT=5000
SIM_TIME=3600
UPDATE_INTERVAL=30


script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ns3_dir="${LEOSIM_NS3_ROOT:-$(cd "${script_dir}/../../../.." && pwd)}"
repo_root="${LEOSIM_REPO_ROOT:-${ns3_dir}}"
results_root="${LEOSIM_RESULTS_ROOT:-${repo_root}/experiments/handover/results}"

# Decode the complete factorial array. Adjacent IDs vary the random run first,
# keeping every ten-task block a directly comparable treatment.
index=${SLURM_ARRAY_TASK_ID}
rng_run=$((index % 10 + 1))
index=$((index / 10))
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
route_tree_dir="${TMPDIR:-/tmp}/leosim-route-trees-${SLURM_JOB_ID}-${SLURM_ARRAY_TASK_ID}"
mkdir -p "${run_dir}" "${route_tree_dir}"

if [[ ! -x "${ns3_dir}/ns3" ]]; then
    echo "ns-3 launcher not found at ${ns3_dir}/ns3" >&2
    exit 2
fi

cd "${ns3_dir}"

export OMP_NUM_THREADS="${SLURM_CPUS_PER_TASK:-8}"
export NS_GLOBAL_VALUE="RngSeed=${LEOSIM_RNG_SEED:-20260803};RngRun=${rng_run}"

command_line="leosim-experiments \
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
--outputPrefix=${run_dir}/result"

{
    echo "run_id=${run_id}"
    echo "job_id=${SLURM_JOB_ID}"
    echo "array_task_id=${SLURM_ARRAY_TASK_ID}"
    echo "hostname=$(hostname)"
    echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "ns_global_value=${NS_GLOBAL_VALUE}"
    echo "command=${command_line}"
    git -C "${repo_root}" rev-parse HEAD 2>/dev/null || echo "commit=unavailable"
    sha256sum contrib/leosim/data/prepro/satellite_mobility.tcl
} > "${run_dir}/metadata.txt"

set +e
/usr/bin/time -v ./ns3 run --no-build "${command_line}" \
    > "${run_dir}/stdout.log" 2> "${run_dir}/stderr.log"
status=$?
set -e

echo "${status}" > "${run_dir}/exit-status.txt"
date -u +%Y-%m-%dT%H:%M:%SZ > "${run_dir}/finished-utc.txt"

if [[ ${status} -ne 0 ]]; then
    echo "LeoSim run failed with status ${status}; see ${run_dir}/stderr.log" >&2
    exit "${status}"
fi

required=(
    "${run_dir}/result-statistics.csv"
    "${run_dir}/result-statistics.json"
    "${run_dir}/result-handovers.csv"
    "${run_dir}/result-flowmon.xml"
)
for output in "${required[@]}"; do
    if [[ ! -s "${output}" ]]; then
        echo "Required output missing or empty: ${output}" >&2
        exit 3
    fi
done

echo "Completed ${run_id}: ${run_dir}"
