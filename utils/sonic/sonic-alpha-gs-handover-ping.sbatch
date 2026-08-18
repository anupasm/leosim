#!/usr/bin/env bash
#SBATCH --job-name=leosim-alpha-gs
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --time=48:00:00
#SBATCH --output=/scratch/adesilva/logs/leosim-alpha-gs-%A.out
#SBATCH --error=/scratch/adesilva/logs/leosim-alpha-gs-%A.err

# Single-task LeoSim run for the alpha-GS handover-ping scenario: 500
# satellites, 1 ground station server, 5000 UEs, a single GS->UE TCP flow,
# and an ICMP pre-handover ping burst (20 x 1400 B @ 10 ms) fired on every
# alpha-GS handover.  SIM_TIME defaults to 1800 s.  The scenario is faithful
# to the 200 s local pilot (leosim-alpha-gs-handover-ping-*) except for the
# longer duration.  The batch deliberately uses --no-build so it never
# modifies the shared ns-3 build tree.

set -euo pipefail

export TMPDIR="/scratch/adesilva/tmp/${SLURM_JOB_ID}"
export APPTAINER_CACHEDIR="/scratch/adesilva/cache/apptainer"
export APPTAINER_TMPDIR="${TMPDIR}/apptainer"
export XDG_CACHE_HOME="/scratch/adesilva/cache"
mkdir -p "$TMPDIR" "$APPTAINER_CACHEDIR" "$APPTAINER_TMPDIR" /scratch/adesilva/logs

if command -v module >/dev/null 2>&1; then
    module purge
    module load "${APPTAINER_MODULE:-apptainer/1.3.4-gcc-11.5.0-ojp6nts}" 2>/dev/null || true
fi

# --- Configuration (all overridable via environment) -------------------------
SIM_TIME="${LEOSIM_SIM_TIME:-1800}"
SATELLITE_COUNT="${LEOSIM_SATELLITES:-500}"
NUM_SERVERS="${LEOSIM_NUM_SERVERS:-1}"
NUM_UES="${LEOSIM_NUM_UES:-5000}"
SERVER_ID="${LEOSIM_SERVER_ID:-0}"
UE_ID="${LEOSIM_UE_ID:-0}"
UPDATE_INTERVAL="${LEOSIM_ROUTING_UPDATE_INTERVAL:-30}"
BEAM_UPDATE_INTERVAL_MS="${LEOSIM_BEAM_UPDATE_INTERVAL_MS:-1000}"
RNG_SEED="${LEOSIM_RNG_SEED:-20260803}"
RNG_RUN="${LEOSIM_RNG_RUN:-1}"
TCP_RATE="${LEOSIM_TCP_RATE:-1Mbps}"
TCP_PACKET_SIZE="${LEOSIM_TCP_PACKET_SIZE:-1024}"
PING_TARGET_UE="${LEOSIM_PING_TARGET_UE:-0}"
PING_PACKET_SIZE="${LEOSIM_PING_PACKET_SIZE:-1400}"
PING_COUNT="${LEOSIM_PING_COUNT:-20}"
PING_INTERVAL_MS="${LEOSIM_PING_INTERVAL_MS:-10}"

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
base_results_root="${LEOSIM_RESULTS_ROOT:-/scratch/adesilva/results/leosim-alpha-gs-handover-ping}"
case "${base_results_root}" in
    /scratch/adesilva|/scratch/adesilva/*) ;;
    *) echo "LEOSIM_RESULTS_ROOT must be under /scratch/adesilva: ${base_results_root}" >&2; exit 2 ;;
esac
results_root="${base_results_root}/${SLURM_JOB_ID}"
run_id="alpha-gs-handover-ping_sats-${SATELLITE_COUNT}_ues-${NUM_UES}_gss-${NUM_SERVERS}_duration-${SIM_TIME}s"
run_dir="${results_root}/${run_id}"
output_stem="result-job${SLURM_JOB_ID}-restart${SLURM_RESTART_COUNT:-0}-pid$$"
output_prefix="${run_dir}/${output_stem}"
route_tree_dir="/scratch/adesilva/tmp/leosim-route-trees-${SLURM_JOB_ID}"
mkdir -p "${run_dir}" "${route_tree_dir}"

# Data directory: prefer the data-b dataset used by the local pilot.
if [[ -d "${repo_root}/contrib/leosim/data-b" ]]; then
    data_dir="${repo_root}/contrib/leosim/data-b"
elif [[ -d "${repo_root}/contrib/leosim/data" ]]; then
    data_dir="${repo_root}/contrib/leosim/data"
else
    data_dir="${PWD}/data"
fi
satellite_trace="${data_dir}/prepro/satellite_mobility.tcl"
satellite_names="${satellite_trace}.names.csv"
if [[ ! -s "${satellite_trace}" || ! -s "${satellite_names}" ]]; then
    echo "Missing preprocessed mobility trace: ${satellite_trace}" >&2
    echo "Generate it with utils/tle_to_positions.py --ns3-format for ${SIM_TIME}s." >&2
    exit 2
fi
prepared_satellites=$(awk 'END { print (NR > 0 ? NR - 1 : 0) }' "${satellite_names}")
if [[ "${prepared_satellites}" -lt "${SATELLITE_COUNT}" ]]; then
    echo "Mobility trace contains ${prepared_satellites} satellites; ${SATELLITE_COUNT} required." >&2
    exit 2
fi
trace_duration=$(awk '
    { for (i = 1; i <= NF; ++i) { if ($i == "at" && $(i+1) ~ /^[0-9.]+$/) { v = $(i+1) + 0; if (v > max) max = v } } }
    END { print (max + 0) }' "${satellite_trace}")
if [[ -z "${trace_duration}" || $(awk 'BEGIN { print ('"${trace_duration}"' < '"${SIM_TIME}"') ? 1 : 0 }') -eq 1 ]]; then
    echo "Mobility trace covers ${trace_duration}s but ${SIM_TIME}s are required." >&2
    exit 2
fi

# Host ns-3 if available, otherwise the Apptainer image.
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
    ns3_cmd=(apptainer exec --cleanenv \
        --bind "${repo_root}:${repo_root}" \
        --bind "${results_root}:${results_root}" \
        --bind "${route_tree_dir}:${route_tree_dir}" \
        --pwd /opt/leosim/ns3 "${SIF}" ./ns3 run --no-build)
    route_tree_engine="${CONTAINER_NS3_ROOT}/contrib/leosim/utils/rengine/leosim-rengine"
    echo "Using containerized ns-3 from ${SIF}"
fi

if [[ -x "${ns3_dir}/ns3" ]]; then
    if [[ ! -x "${route_tree_engine}" ]] || ! "${route_tree_engine}" --self-test >/dev/null; then
        echo "Missing or invalid routing engine: ${route_tree_engine}" >&2
        echo "Run contrib/leosim/utils/sonic/build_leosim.sh before submitting." >&2
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
fi

export OMP_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
export NS_GLOBAL_VALUE="RngSeed=${RNG_SEED};RngRun=${RNG_RUN}"

command_line="leosim-experiments \
--dataDir=${data_dir} \
--satellites=${satellite_trace} \
--useTrace=1 \
--simTime=${SIM_TIME} \
--numSatellites=${SATELLITE_COUNT} \
--numServers=${NUM_SERVERS} \
--numUes=${NUM_UES} \
--allToAllTraffic=0 \
--serverId=${SERVER_ID} --ueId=${UE_ID} \
--appStart=1 --appStop=$((SIM_TIME - 1)) \
--tcpRate=${TCP_RATE} --tcpPacketSize=${TCP_PACKET_SIZE} \
--enablePreHandoverPing=1 \
--preHandoverPingTargetUe=${PING_TARGET_UE} \
--preHandoverPingPacketSize=${PING_PACKET_SIZE} \
--preHandoverPingCount=${PING_COUNT} \
--preHandoverPingIntervalMs=${PING_INTERVAL_MS} \
--enableIsl=1 --maxIslNeighbors=4 \
--enableDynamicRouting=1 --routingUpdateInterval=${UPDATE_INTERVAL} \
--useRouteTreeCache=1 --routeTreeWorkers=${SLURM_CPUS_PER_TASK:-1} \
--routeTreeEngine=${route_tree_engine} \
--routeTreeWorkDir=${route_tree_dir} \
--hoMode=CHO --maxCandidates=3 --enableHoBuffering=1 --hoBufferSize=1024 \
--ttt=1 --t310=1 --n310=3 --n311=3 \
--a3Offset=3 --a4Threshold=-110 --tteTrigger=30 \
--choPrep=100 --choExec=150 --beamUpdateIntervalMs=${BEAM_UPDATE_INTERVAL_MS} \
--enableStatistics=1 --statisticsInterval=1 \
--enableHandoverLogging=1 --writeFlowMonitor=1 \
--enableTaskProfiler=1 --progressLogInterval=30 \
--enableVisualization=0 --verbose=1 \
--outputPrefix=${output_prefix} --uniqueOutputPrefix=0"

{
    echo "run_id=${run_id}"
    echo "output_prefix=${output_prefix}"
    echo "job_id=${SLURM_JOB_ID}"
    echo "hostname=$(hostname)"
    echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "sim_time_s=${SIM_TIME}"
    echo "satellites=${SATELLITE_COUNT}"
    echo "num_servers=${NUM_SERVERS}"
    echo "num_ues=${NUM_UES}"
    echo "server_id=${SERVER_ID}"
    echo "ue_id=${UE_ID}"
    echo "all_to_all_traffic=0"
    echo "tcp_rate=${TCP_RATE}"
    echo "routing_update_interval_s=${UPDATE_INTERVAL}"
    echo "beam_update_interval_ms=${BEAM_UPDATE_INTERVAL_MS}"
    echo "pre_handover_ping=1"
    echo "pre_handover_ping_target_ue=${PING_TARGET_UE}"
    echo "pre_handover_ping_packet_size=${PING_PACKET_SIZE}"
    echo "pre_handover_ping_count=${PING_COUNT}"
    echo "pre_handover_ping_interval_ms=${PING_INTERVAL_MS}"
    echo "ns_global_value=${NS_GLOBAL_VALUE}"
    echo "data_dir=${data_dir}"
    echo "satellite_trace=${satellite_trace}"
    echo "trace_duration_s=${trace_duration}"
    echo "command=${command_line}"
    git -C "${repo_root}" rev-parse HEAD 2>/dev/null || echo "commit=unavailable"
    if [[ -f "${satellite_trace}" ]]; then
        sha256sum "${satellite_trace}"
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
    "${output_prefix}-pre-handover-ping.csv"
    "${output_prefix}-cho-candidates.csv"
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
ping_bursts=$(awk -F, 'END { print (NR > 1 ? NR - 1 : 0) }' "${output_prefix}-pre-handover-ping.csv")
if [[ -z "${tx_packets}" || -z "${sink_count}" || -z "${sinks_with_rx}" ]]; then
    echo "Could not validate statistics summary: ${summary_json}" >&2
    echo 4 > "${run_dir}/exit-status.txt"
    exit 4
fi
if [[ "${tx_packets}" -eq 0 || "${sinks_with_rx}" -eq 0 || "${ping_bursts}" -lt 1 ]]; then
    echo "Invalid completed run: tx_packets=${tx_packets}, sinks_with_rx=${sinks_with_rx}, ping_bursts=${ping_bursts}." >&2
    echo 4 > "${run_dir}/exit-status.txt"
    exit 4
fi

echo "Completed ${run_id}: ${run_dir}"
