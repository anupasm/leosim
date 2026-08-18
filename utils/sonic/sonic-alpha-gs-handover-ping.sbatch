#!/usr/bin/env bash
#SBATCH --job-name=leosim-alpha-gs
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --time=48:00:00
#SBATCH --output=/scratch/adesilva/logs/leosim-alpha-gs-%A.out
#SBATCH --error=/scratch/adesilva/logs/leosim-alpha-gs-%A.err

# Single-task LeoSim run for the alpha-GS handover-ping scenario: 500
# satellites, 1 ground station server, 5000 UEs, a single GS->UE TCP flow,
# and an ICMP pre-handover ping burst (20 x 1400 B @ 10 ms) fired on every
# alpha-GS handover.  SIM_TIME defaults to 1800 s.  The scenario is faithful
# to the 200 s local pilot (leosim-alpha-gs-handover-ping-*) except for the
# longer duration.  The ns-3 core loop is single-threaded; the CPUs allocated
# here are used by the parallel pre-simulation stages (TLE preprocessing and
# route-tree cache build via --routeTreeWorkers).  The batch deliberately uses
# --no-build so it never modifies the shared ns-3 build tree.

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

# Preprocessing / coverage selection
allocated_cpus="${SLURM_CPUS_PER_TASK:-8}"
TLE_TIMESTEP="${LEOSIM_TLE_TIMESTEP:-5}"
# alpha_ue_South_India = Chennai (13.0827 N, 80.2707 E); used to require that the
# selected constellation also covers the South-India UE location.
SOUTH_INDIA_LAT="${LEOSIM_SOUTH_INDIA_LAT:-13.0827}"
SOUTH_INDIA_LON="${LEOSIM_SOUTH_INDIA_LON:-80.2707}"
# 0 = use every satellite that covers the UE area or the South-India location;
# otherwise cap the selection file to the first N IDs (sorted numerically).
SELECTION_COUNT="${LEOSIM_SELECTION_COUNT:-0}"

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

# Python runner used for TLE -> ns-3 trace preprocessing and for the coverage-
# based satellite selection. Host mode uses the repo virtualenv when present
# (NumPy/Skyfield required for tle_to_positions.py); otherwise the Apptainer
# image provides /opt/venv/bin/python.
if [[ -n "${LEOSIM_PYTHON:-}" ]]; then
    python_cmd=("${LEOSIM_PYTHON}")
    python_scripts_dir="${repo_root}/contrib/leosim/utils"
elif [[ -x "${ns3_dir}/ns3" ]]; then
    python_cmd=()
    for candidate in "${repo_root}/.venv/bin/python" /opt/venv/bin/python python3; do
        if command -v "${candidate}" >/dev/null 2>&1 || [[ -x "${candidate}" ]]; then
            if "${candidate}" -c 'import numpy, skyfield' >/dev/null 2>&1; then
                python_cmd=("${candidate}")
                break
            fi
        fi
    done
    if [[ ${#python_cmd[@]} -eq 0 ]]; then
        echo "Host mode requires Python with NumPy and Skyfield for trace preprocessing." >&2
        echo "Set LEOSIM_PYTHON to a suitable interpreter or use the Apptainer image." >&2
        exit 2
    fi
    python_scripts_dir="${repo_root}/contrib/leosim/utils"
else
    if ! command -v apptainer >/dev/null 2>&1; then
        echo "Apptainer is unavailable; no containerized runtime for preprocessing." >&2
        exit 2
    fi
    if [[ ! -f "${SIF}" ]]; then
        echo "Container image missing: ${SIF}" >&2
        echo "Build it with utils/sonic/build_leosim_container.sh or set SIF explicitly." >&2
        exit 2
    fi
    python_cmd=(apptainer exec --cleanenv \
        --bind "${repo_root}:${repo_root}" \
        --bind "${results_root}:${results_root}" \
        "${SIF}" /opt/venv/bin/python)
    # Python utilities come from the bound host repo, NOT the image, so the SIF
    # only needs the compiled leosim-experiments binary and /opt/venv. This
    # avoids stale in-image copies (e.g. missing find_satellites_covering_ue_area.py).
    python_scripts_dir="${repo_root}/contrib/leosim/utils"
fi

if [[ ! -f "${python_scripts_dir}/find_satellites_covering_ue_area.py" || \
      ! -f "${python_scripts_dir}/tle_to_positions.py" ]]; then
    echo "Missing LeoSim Python utilities in ${python_scripts_dir}" >&2
    echo "Sync the repo (rsync) so contrib/leosim/utils is present on this host." >&2
    exit 2
fi

# Preprocessed ns-3 mobility trace, generated per job from the data-b TLEs when a
# valid copy is not already present. The repo's data-b/prepro trace is reused
# (copied) when it is present and long enough.
prepro_dir="${results_root}/_preprocessed"
mkdir -p "${prepro_dir}"
repo_trace="${data_dir}/prepro/satellite_mobility.tcl"
repo_names="${repo_trace}.names.csv"
satellite_trace="${prepro_dir}/satellite_mobility.tcl"
satellite_names="${satellite_trace}.names.csv"

trace_duration_awk='
    { for (i = 1; i <= NF; ++i) { if ($i == "at" && $(i+1) ~ /^[0-9.]+$/) { v = $(i+1) + 0; if (v > max) max = v } } }
    END { print (max + 0) }'

prepare_trace() {
    if [[ -s "${satellite_trace}" && -s "${satellite_names}" ]]; then
        prepared_satellites=$(awk 'END { print (NR > 0 ? NR - 1 : 0) }' "${satellite_names}")
        trace_duration=$(awk "${trace_duration_awk}" "${satellite_trace}")
        if [[ "${prepared_satellites}" -ge "${SATELLITE_COUNT}" && -n "${trace_duration}" &&
              $(awk 'BEGIN { print ('"${trace_duration}"' >= '"${SIM_TIME}"') ? 1 : 0 }') -eq 1 ]]; then
            echo "Reusing prepared trace: ${satellite_trace} (${prepared_satellites} sats, ${trace_duration}s)"
            return 0
        fi
    fi
    if [[ -s "${repo_trace}" && -s "${repo_names}" ]]; then
        repo_prepared=$(awk 'END { print (NR > 0 ? NR - 1 : 0) }' "${repo_names}")
        repo_duration=$(awk "${trace_duration_awk}" "${repo_trace}")
        if [[ "${repo_prepared}" -ge "${SATELLITE_COUNT}" && -n "${repo_duration}" &&
              $(awk 'BEGIN { print ('"${repo_duration}"' >= '"${SIM_TIME}"') ? 1 : 0 }') -eq 1 ]]; then
            echo "Copying repo mobility trace into ${prepro_dir} (${repo_prepared} sats, ${repo_duration}s)"
            cp -f "${repo_trace}" "${satellite_trace}"
            cp -f "${repo_names}" "${satellite_names}"
            prepared_satellites="${repo_prepared}"
            trace_duration="${repo_duration}"
            return 0
        fi
    fi
    if [[ ! -d "${data_dir}/tles" ]]; then
        echo "No TLE directory for preprocessing: ${data_dir}/tles" >&2
        exit 2
    fi
    echo "Generating ${SIM_TIME}s mobility trace from ${data_dir}/tles ..."
    "${python_cmd[@]}" "${python_scripts_dir}/tle_to_positions.py" \
        --input "${data_dir}/tles" \
        --output "${prepro_dir}" \
        --duration "${SIM_TIME}" \
        --timestep "${TLE_TIMESTEP}" \
        --workers "${allocated_cpus}" \
        --ns3-format
    if [[ ! -s "${satellite_trace}" || ! -s "${satellite_names}" ]]; then
        echo "Trace generation did not produce the expected ns-3 outputs in ${prepro_dir}." >&2
        exit 3
    fi
    prepared_satellites=$(awk 'END { print (NR > 0 ? NR - 1 : 0) }' "${satellite_names}")
    trace_duration=$(awk "${trace_duration_awk}" "${satellite_trace}")
}
prepare_trace
if [[ "${prepared_satellites}" -lt "${SATELLITE_COUNT}" ]]; then
    echo "Prepared trace has ${prepared_satellites} satellites; ${SATELLITE_COUNT} required." >&2
    exit 3
fi
if [[ -z "${trace_duration}" || $(awk 'BEGIN { print ('"${trace_duration}"' < '"${SIM_TIME}"') ? 1 : 0 }') -eq 1 ]]; then
    echo "Prepared trace covers ${trace_duration}s but ${SIM_TIME}s are required." >&2
    exit 3
fi

# Select the satellites that cover the UE area (data-b/ues) and the South-India
# UE location (alpha_ue_South_India), then use their union as the constellation.
coverage_ue_csv="${prepro_dir}/coverage-ue-area.csv"
coverage_si_csv="${prepro_dir}/coverage-south-india.csv"
satellite_selection="${prepro_dir}/satellite-selection.txt"
echo "Selecting satellites covering the UE area and the South-India UE location ..."
"${python_cmd[@]}" "${python_scripts_dir}/find_satellites_covering_ue_area.py" \
    --data-dir "${data_dir}" \
    --trace "${satellite_trace}" \
    --names "${satellite_names}" \
    --start 0 --duration "${SIM_TIME}" \
    --sample-interval "${TLE_TIMESTEP}" \
    --num-satellites 0 \
    --mode any \
    --output "${coverage_ue_csv}"
"${python_cmd[@]}" "${python_scripts_dir}/find_satellites_covering_ue_area.py" \
    --data-dir "${data_dir}" \
    --trace "${satellite_trace}" \
    --names "${satellite_names}" \
    --location "${SOUTH_INDIA_LAT}" "${SOUTH_INDIA_LON}" \
    --start 0 --duration "${SIM_TIME}" \
    --sample-interval "${TLE_TIMESTEP}" \
    --num-satellites 0 \
    --mode any \
    --output "${coverage_si_csv}"
cat "${coverage_ue_csv}" "${coverage_si_csv}" \
    | awk -F, 'NR>1 && $1 ~ /^[0-9]+$/ { print $1 }' \
    | sort -n -u > "${satellite_selection}"
selection_count=$(wc -l < "${satellite_selection}")
if [[ "${selection_count}" -eq 0 ]]; then
    echo "Coverage selection produced no satellites (check UE/TLE data)." >&2
    exit 3
fi
if [[ "${SELECTION_COUNT}" -gt 0 && "${selection_count}" -gt "${SELECTION_COUNT}" ]]; then
    head -n "${SELECTION_COUNT}" "${satellite_selection}" > "${satellite_selection}.tmp"
    mv -f "${satellite_selection}.tmp" "${satellite_selection}"
    selection_count="${SELECTION_COUNT}"
fi
echo "Coverage selection: ${selection_count} satellites cover the UE area and the South-India location."

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
    if ! apptainer exec --cleanenv --pwd /opt/leosim/ns3 "${SIF}" \
        ./ns3 run "leosim-experiments --PrintHelp" --no-build 2>/dev/null \
        | grep -q -- "--enablePreHandoverPing"; then
        echo "Container leosim-experiments is missing --enablePreHandoverPing; rebuild the SIF." >&2
        echo "Run utils/sonic/build_leosim_container.sh from the current repo." >&2
        exit 2
    fi
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

export OMP_NUM_THREADS="${allocated_cpus}"
export NS_GLOBAL_VALUE="RngSeed=${RNG_SEED};RngRun=${RNG_RUN}"

command_line="leosim-experiments \
--dataDir=${data_dir} \
--satellites=${satellite_trace} \
--useTrace=1 \
--simTime=${SIM_TIME} \
--numSatellites=${selection_count} \
--satelliteSelectionFile=${satellite_selection} \
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
--useRouteTreeCache=1 --routeTreeWorkers=${allocated_cpus} \
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
    echo "selected_satellites=${selection_count}"
    echo "satellite_selection_file=${satellite_selection}"
    echo "coverage_ue_area_csv=${coverage_ue_csv}"
    echo "coverage_south_india_csv=${coverage_si_csv}"
    echo "south_india_lat=${SOUTH_INDIA_LAT}"
    echo "south_india_lon=${SOUTH_INDIA_LON}"
    echo "tle_timestep=${TLE_TIMESTEP}"
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
    echo "prepared_satellites=${prepared_satellites}"
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
