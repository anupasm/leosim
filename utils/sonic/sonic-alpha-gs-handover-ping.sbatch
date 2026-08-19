#!/usr/bin/env bash
#SBATCH --job-name=leosim-alpha-gs
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --time=48:00:00
#SBATCH --output=/scratch/adesilva/logs/leosim-alpha-gs-%A.out
#SBATCH --error=/scratch/adesilva/logs/leosim-alpha-gs-%A.err

# Single-task LeoSim run for the alpha-GS UDP scenario: the satellites that
# cover the Sri-Lankan UE area are selected from the full constellation, giving
# 1 ground station server, 1,001 UEs (alpha_ue_0 plus 1,000 UEs within 50 km of
# alpha_gs, all active from t=0), continuous bidirectional CBR UDP between
# UE-0 <-> Server-0, and every other UE continuously pinging its currently-
# connected serving satellite (optionally with a ramped ping load). SIM_TIME
# defaults to 1800 s; the scenario is faithful to the 200 s local pilot
# (leosim-alpha-gs-sat-ping-*) except for the longer duration. The ns-3 core
# loop is single-threaded; the CPUs allocated here are used by the parallel
# pre-simulation stages (TLE preprocessing and route-tree cache build via
# --routeTreeWorkers). The batch deliberately uses --no-build so it never
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
NUM_UES="${LEOSIM_NUM_UES:-1001}"
# Maximum candidate satellite access links per ground node per trajectory epoch.
# 4 keeps 2x handover redundancy at half the access-device/IP cost.
MAX_ACCESS_SATS="${LEOSIM_MAX_ACCESS_SATS:-4}"
SERVER_ID="${LEOSIM_SERVER_ID:-0}"
UE_ID="${LEOSIM_UE_ID:-0}"
UPDATE_INTERVAL="${LEOSIM_ROUTING_UPDATE_INTERVAL:-30}"
BEAM_UPDATE_INTERVAL_MS="${LEOSIM_BEAM_UPDATE_INTERVAL_MS:-1000}"
RNG_SEED="${LEOSIM_RNG_SEED:-20260803}"
RNG_RUN="${LEOSIM_RNG_RUN:-1}"
UDP_RATE="${LEOSIM_UDP_RATE:-1Mbps}"
UDP_PACKET_SIZE="${LEOSIM_UDP_PACKET_SIZE:-1400}"
# Shared access/beam capacity and per-link access rate. Defaults are set to
# the upper-realistic Starlink-V2 range (per beam ~0.5-3 Gbps DL / 100-500 Mbps
# UL; per satellite ~20-80 Gbps DL / 5-10 Gbps UL; per-user ~100 Mbps access).
# For a neighbor-load stress study, LOWER the beam capacity so the satellite
# pings + GS<->UE-0 contend for the same beam (e.g. LEOSIM_BEAM_DOWNLINK_CAPACITY=10Mbps).
ACCESS_DATA_RATE="${LEOSIM_ACCESS_DATA_RATE:-100Mbps}"
BEAM_UL_CAP="${LEOSIM_BEAM_UPLINK_CAPACITY:-500Mbps}"
BEAM_DL_CAP="${LEOSIM_BEAM_DOWNLINK_CAPACITY:-2Gbps}"
SAT_UL_CAP="${LEOSIM_SATELLITE_UPLINK_CAPACITY:-10Gbps}"
SAT_DL_CAP="${LEOSIM_SATELLITE_DOWNLINK_CAPACITY:-40Gbps}"
# UE -> serving-satellite ICMP pings: every UE with index >= SAT_PING_START_INDEX
# continuously pings whichever satellite it is currently connected to, following
# handovers (the beam manager retargets each UE's ping on handover). UE-0 (index
# 0) is skipped because it carries the GS<->UE UDP flows. The per-UE ping rate
# ramps upward over the simulation (rate(t) = 1/interval + SAT_PING_LOAD_RAMP*t,
# capped at SAT_PING_MAX_RATE) so the aggregate access-link load grows with time
# -- used to study how neighbor load affects the GS<->UE traffic. Set
# LEOSIM_SATELLITE_PING_LOAD_RAMP=0 for a constant ping load.
SAT_PING_ENABLE="${LEOSIM_SATELLITE_PING_ENABLE:-1}"
SAT_PING_START_INDEX="${LEOSIM_SATELLITE_PING_START_INDEX:-1}"
SAT_PING_INTERVAL="${LEOSIM_SATELLITE_PING_INTERVAL:-1}"
SAT_PING_PACKET_SIZE="${LEOSIM_SATELLITE_PING_PACKET_SIZE:-56}"
SAT_PING_STAGGER="${LEOSIM_SATELLITE_PING_STAGGER_WINDOW:-1}"
SAT_PING_LOAD_RAMP="${LEOSIM_SATELLITE_PING_LOAD_RAMP:-1}"
SAT_PING_MAX_RATE="${LEOSIM_SATELLITE_PING_MAX_RATE:-200}"
SAT_PING_RAMP_INTERVAL="${LEOSIM_SATELLITE_PING_RAMP_INTERVAL:-1}"
# UE introduction: by default ALL UEs (UE-0 plus the 1,000 near-alpha_gs UEs)
# are active from t=0 -- no batched introduction. To enable periodic batches
# instead, override LEOSIM_INITIAL_UES to a smaller value (must be >= 1).
UE_DYNAMIC_ENABLE="${LEOSIM_ENABLE_DYNAMIC_GROUND_NODES:-1}"
UE_INITIAL="${LEOSIM_INITIAL_UES:-${NUM_UES}}"
UE_INTRO_START="${LEOSIM_UE_INTRODUCTION_START:-30}"
UE_INTRO_INTERVAL="${LEOSIM_UE_INTRODUCTION_INTERVAL:-10}"
UE_INTRO_BATCH="${LEOSIM_UES_PER_INTRODUCTION:-100}"

# Preprocessing / coverage selection
allocated_cpus="${SLURM_CPUS_PER_TASK:-8}"
TLE_TIMESTEP="${LEOSIM_TLE_TIMESTEP:-5}"
# 0 = use every satellite that covers the UE area; otherwise cap the selection
# file to the first N IDs (sorted numerically).
SELECTION_COUNT="${LEOSIM_SELECTION_COUNT:-0}"

# --- Diagnostics -------------------------------------------------------------
# Timestamped stage marker so the slurm log shows exactly which step the job is
# on. A watchdog runs alongside the simulation and periodically logs elapsed
# wall time, the number of live leosim processes, the stdout.log line count, and
# the most recent experiment [timing] line - so a stall is attributable to the
# exact step (preprocess / coverage / route-tree / Simulator::Run).
job_start=$(date +%s)
WATCHDOG_INTERVAL="${LEOSIM_WATCHDOG_INTERVAL:-60}"
stage() {
    echo "[alpha-gs][$(date -u +%Y-%m-%dT%H:%M:%SZ)] $*"
}
trap 'echo "[alpha-gs][$(date -u +%Y-%m-%dT%H:%M:%SZ)] script exit status=$?"' EXIT

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

# Locate the LeoSim module root (the directory holding utils/, model/, helper/).
# Two layouts are supported:
#   A) ns-3 checkout:   <repo_root>/contrib/leosim   (e.g. ~/LeoSim/ns3 locally)
#   B) standalone:      <repo_root>                  (e.g. ~/leosim on the server)
leo_module_root="${repo_root}/contrib/leosim"
if [[ ! -d "${leo_module_root}/utils" ]]; then
    if [[ -d "${repo_root}/utils" && -d "${repo_root}/model" && -d "${repo_root}/helper" ]]; then
        leo_module_root="${repo_root}"
    else
        leo_module_root="$(cd "${script_dir}/../.." 2>/dev/null && pwd)" || leo_module_root="${repo_root}"
    fi
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
run_id="alpha-gs-udp_sats-${SATELLITE_COUNT}_ues-${NUM_UES}_gss-${NUM_SERVERS}_duration-${SIM_TIME}s"
run_dir="${results_root}/${run_id}"
output_stem="result-job${SLURM_JOB_ID}-restart${SLURM_RESTART_COUNT:-0}-pid$$"
output_prefix="${run_dir}/${output_stem}"
route_tree_dir="/scratch/adesilva/tmp/leosim-route-trees-${SLURM_JOB_ID}"
mkdir -p "${run_dir}" "${route_tree_dir}"
stage "job preparing: run_dir=${run_dir}, cpus=${allocated_cpus}"

# Data directory: prefer the data-b dataset used by the local pilot.
if [[ -d "${leo_module_root}/data-b" ]]; then
    data_dir="${leo_module_root}/data-b"
elif [[ -d "${leo_module_root}/data" ]]; then
    data_dir="${leo_module_root}/data"
else
    data_dir="${PWD}/data"
fi

# Python runner used for TLE -> ns-3 trace preprocessing and for the coverage-
# based satellite selection. Host mode uses the repo virtualenv when present
# (NumPy/Skyfield required for tle_to_positions.py); otherwise the Apptainer
# image provides /opt/venv/bin/python.
if [[ -n "${LEOSIM_PYTHON:-}" ]]; then
    python_cmd=("${LEOSIM_PYTHON}")
    python_scripts_dir="${leo_module_root}/utils"
elif [[ -x "${ns3_dir}/ns3" ]]; then
    python_cmd=()
    for candidate in "${leo_module_root}/.venv/bin/python" "${repo_root}/../.venv/bin/python" /opt/venv/bin/python python3; do
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
    python_scripts_dir="${leo_module_root}/utils"
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
    python_scripts_dir="${leo_module_root}/utils"
fi

missing=()
for needed in find_satellites_covering_ue_area.py tle_to_positions.py; do
    if [[ ! -f "${python_scripts_dir}/${needed}" ]]; then
        missing+=("${needed}")
    fi
done
if [[ ${#missing[@]} -gt 0 ]]; then
    echo "Missing LeoSim Python utilities in ${python_scripts_dir}: ${missing[*]}" >&2
    echo "Sync the repo (rsync) so contrib/leosim/utils is present on this host." >&2
    exit 2
fi

# Preprocessed ns-3 mobility trace. Always generated from the FULL data-b TLE
# constellation (11,514 satellites: alpha/beta/gamma), so the coverage-based
# satellite selection and the simulation use the complete constellation rather
# than the small 1,600-satellite sample in data-b/prepro. A trace already
# generated in this job's _preprocessed dir is reused (restart-friendly).
prepro_dir="${results_root}/_preprocessed"
mkdir -p "${prepro_dir}"
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
stage "trace ready: ${prepared_satellites} sats over ${trace_duration}s -> ${satellite_trace}"

# Select the satellites that cover the UE area (data-b/ues) and use them as the
# constellation (matches the local pilot's satellite-selection.txt).
coverage_ue_csv="${prepro_dir}/coverage-ue-area.csv"
satellite_selection="${prepro_dir}/satellite-selection.txt"
echo "Selecting satellites covering the UE area ..."
"${python_cmd[@]}" "${python_scripts_dir}/find_satellites_covering_ue_area.py" \
    --data-dir "${data_dir}" \
    --trace "${satellite_trace}" \
    --names "${satellite_names}" \
    --start 0 --duration "${SIM_TIME}" \
    --sample-interval "${TLE_TIMESTEP}" \
    --num-satellites 0 \
    --mode any \
    --output "${coverage_ue_csv}"
awk -F, 'NR>1 && $1 ~ /^[0-9]+$/ { print $1 }' "${coverage_ue_csv}" \
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
echo "Coverage selection: ${selection_count} satellites cover the UE area."
stage "selection ready: ${selection_count} sats -> ${satellite_selection}"

# Host ns-3 if available, otherwise the Apptainer image.
if [[ -x "${ns3_dir}/ns3" ]]; then
    cd "${ns3_dir}"
    ns3_cmd=("${ns3_dir}/ns3" run --no-build)
    route_tree_engine="${leo_module_root}/utils/rengine/leosim-rengine"
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
        | grep -q -- "--udpRate"; then
        echo "Container leosim-experiments is missing --udpRate; rebuild the SIF." >&2
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

stage "ns-3 and routing engine ready; command_line built"
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
--udpRate=${UDP_RATE} --udpPacketSize=${UDP_PACKET_SIZE} \
--accessDataRate=${ACCESS_DATA_RATE} \
--beamUplinkCapacity=${BEAM_UL_CAP} --beamDownlinkCapacity=${BEAM_DL_CAP} \
--satelliteUplinkCapacity=${SAT_UL_CAP} --satelliteDownlinkCapacity=${SAT_DL_CAP} \
--enableBeamCapacityLogging=0 \
--enableSatellitePing=${SAT_PING_ENABLE} \
--satellitePingStartIndex=${SAT_PING_START_INDEX} \
--satellitePingInterval=${SAT_PING_INTERVAL} \
--satellitePingPacketSize=${SAT_PING_PACKET_SIZE} \
--satellitePingStaggerWindow=${SAT_PING_STAGGER} \
--satellitePingLoadRamp=${SAT_PING_LOAD_RAMP} \
--satellitePingMaxRate=${SAT_PING_MAX_RATE} \
--satellitePingRampInterval=${SAT_PING_RAMP_INTERVAL} \
--enableDynamicGroundNodes=${UE_DYNAMIC_ENABLE} --initialUes=${UE_INITIAL} \
--ueIntroductionStart=${UE_INTRO_START} --ueIntroductionInterval=${UE_INTRO_INTERVAL} \
--uesPerIntroduction=${UE_INTRO_BATCH} --enableGroundNodeLifecycleLogging=1 \
--enableIsl=1 --maxIslNeighbors=4 --maxAccessSatellites=${MAX_ACCESS_SATS} \
--enableSyntheticIslLoad=0 --enableIslLoadLogging=0 \
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
    echo "tle_timestep=${TLE_TIMESTEP}"
    echo "num_servers=${NUM_SERVERS}"
    echo "num_ues=${NUM_UES}"
    echo "max_access_satellites=${MAX_ACCESS_SATS}"
    echo "server_id=${SERVER_ID}"
    echo "ue_id=${UE_ID}"
    echo "all_to_all_traffic=0"
    echo "udp_rate=${UDP_RATE}"
    echo "udp_packet_size=${UDP_PACKET_SIZE}"
    echo "access_data_rate=${ACCESS_DATA_RATE}"
    echo "beam_uplink_capacity=${BEAM_UL_CAP}"
    echo "beam_downlink_capacity=${BEAM_DL_CAP}"
    echo "satellite_uplink_capacity=${SAT_UL_CAP}"
    echo "satellite_downlink_capacity=${SAT_DL_CAP}"
    echo "beam_capacity_logging=0"
    echo "satellite_ping_enabled=${SAT_PING_ENABLE}"
    echo "satellite_ping_start_index=${SAT_PING_START_INDEX}"
    echo "satellite_ping_interval=${SAT_PING_INTERVAL}"
    echo "satellite_ping_packet_size=${SAT_PING_PACKET_SIZE}"
    echo "satellite_ping_stagger_window=${SAT_PING_STAGGER}"
    echo "satellite_ping_load_ramp_pps_per_s=${SAT_PING_LOAD_RAMP}"
    echo "satellite_ping_max_rate_pps=${SAT_PING_MAX_RATE}"
    echo "satellite_ping_ramp_interval_s=${SAT_PING_RAMP_INTERVAL}"
    echo "dynamic_ue_introduction=${UE_DYNAMIC_ENABLE}"
    echo "initial_ues=${UE_INITIAL}"
    echo "ue_introduction_start=${UE_INTRO_START}"
    echo "ue_introduction_interval=${UE_INTRO_INTERVAL}"
    echo "ues_per_introduction=${UE_INTRO_BATCH}"
    echo "routing_update_interval_s=${UPDATE_INTERVAL}"
    echo "beam_update_interval_ms=${BEAM_UPDATE_INTERVAL_MS}"
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

stage "starting leosim-experiments (simTime=${SIM_TIME}s, ues=${NUM_UES}, sats=${selection_count}, workers=${allocated_cpus})"
run_start_epoch=$(date +%s)
set +e
if command -v /usr/bin/time >/dev/null 2>&1; then
    /usr/bin/time -v "${ns3_cmd[@]}" "${command_line}" \
        > "${run_dir}/stdout.log" 2> "${run_dir}/stderr.log" &
else
    time "${ns3_cmd[@]}" "${command_line}" \
        > "${run_dir}/stdout.log" 2> "${run_dir}/stderr.log" &
fi
run_pid=$!
while kill -0 "${run_pid}" 2>/dev/null; do
    sleep "${WATCHDOG_INTERVAL}"
    elapsed=$(( $(date +%s) - run_start_epoch ))
    last_line=$(tail -n 1 "${run_dir}/stdout.log" 2>/dev/null || true)
    loglines=$(wc -l < "${run_dir}/stdout.log" 2>/dev/null || echo 0)
    nproc=$(pgrep -fc "leosim-experiments|leosim-rengine" 2>/dev/null || echo 0)
    printf '[alpha-gs][%s] sim t+%ss pid=%s live_procs=%s loglines=%s last=[%s]\n' \
        "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "${elapsed}" "${run_pid}" "${nproc}" "${loglines}" "${last_line}" \
        | tee -a "${run_dir}/watchdog.log"
done
wait "${run_pid}"
status=$?
set -e
stage "sim finished status=${status} after $(( $(date +%s) - run_start_epoch ))s"

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
if [[ -z "${tx_packets}" || -z "${sink_count}" || -z "${sinks_with_rx}" ]]; then
    echo "Could not validate statistics summary: ${summary_json}" >&2
    echo 4 > "${run_dir}/exit-status.txt"
    exit 4
fi
if [[ "${tx_packets}" -eq 0 || "${sinks_with_rx}" -eq 0 ]]; then
    echo "Invalid completed run: tx_packets=${tx_packets}, sinks_with_rx=${sinks_with_rx}." >&2
    echo 4 > "${run_dir}/exit-status.txt"
    exit 4
fi

stage "COMPLETE: ${run_id}"
echo "Completed ${run_id}: ${run_dir}"
