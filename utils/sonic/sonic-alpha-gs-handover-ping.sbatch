#!/usr/bin/env bash
#SBATCH --job-name=leosim-alpha-gs
#SBATCH --nodes=1
#SBATCH --ntasks=3
#SBATCH --cpus-per-task=8
#SBATCH --time=48:00:00
#SBATCH --output=/scratch/adesilva/logs/leosim-alpha-gs-%A.out
#SBATCH --error=/scratch/adesilva/logs/leosim-alpha-gs-%A.err

# Matched LeoSim BHO, CHO-1, and CHO-3 runs for the alpha-GS handover-ping
# scenario. The satellites covering the Sri-Lankan UE area are selected from the
# full constellation. The scenario is UE-only: no ground station (server) and no
# UE0 probe endpoint are loaded, and no TCP/UDP data flows are installed. All
# 1,000 loaded UEs (alpha_ue_1..alpha_ue_1000) are active from t=0 and each
# continuously pings its serving satellite at a constant one ping per second for
# the whole application window (flat 1 ping/s/UE load, no rate ramp); the
# successfully received requests generate echo replies on the downlink. All
# three policies use the same trace, traffic, capacity, RNG seed/run, and start
# together in separate Slurm steps; only handover mode/candidate count differs.
# The batch deliberately uses --no-build so it never modifies the shared ns-3
# build tree.

set -euo pipefail

comparison_status_file=""
on_exit() {
    local exit_status=$?
    if [[ -n "${comparison_status_file:-}" ]]; then
        printf '%s\n' "${exit_status}" > "${comparison_status_file}" 2>/dev/null || true
    fi
    echo "[alpha-gs][$(date -u +%Y-%m-%dT%H:%M:%SZ)] script exit status=${exit_status}"
}
trap on_exit EXIT

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
SIM_TIME="${LEOSIM_SIM_TIME:-1200}"
SATELLITE_COUNT="${LEOSIM_SATELLITES:-500}"
# Ground stations (servers) are completely removed: the scenario is ping-only
# UE-to-serving-satellite with no UDP/TCP data traffic and no server/UE0 probe
# endpoint. enableGroundStations=0 makes leosim-experiments skip loading the
# ground station entirely.
ENABLE_GROUND_STATIONS="${LEOSIM_ENABLE_GROUND_STATIONS:-0}"
NUM_SERVERS="${LEOSIM_NUM_SERVERS:-0}"
# All loaded UEs (alpha_ue_1..alpha_ue_1000; alpha_ue_0 is absent from the
# input) participate in the ping load.
NUM_UES="${LEOSIM_NUM_UES:-1000}"
# Maximum candidate satellite access links per ground node per trajectory epoch.
# 4 keeps 2x handover redundancy at half the access-device/IP cost.
MAX_ACCESS_SATS="${LEOSIM_MAX_ACCESS_SATS:-4}"
UPDATE_INTERVAL="${LEOSIM_ROUTING_UPDATE_INTERVAL:-30}"
BEAM_UPDATE_INTERVAL_MS="${LEOSIM_BEAM_UPDATE_INTERVAL_MS:-1000}"
# Beam allocation cadence is deliberately separate from the handover beam-manager
# cadence above. Capacity allocations stay at the native 100 ms cadence. CSV
# snapshots are independently throttled to 1 second to avoid tens of gigabytes
# per policy without changing scheduler behaviour.
BEAM_CAPACITY_UPDATE_INTERVAL_MS="${LEOSIM_BEAM_CAPACITY_UPDATE_INTERVAL_MS:-100}"
BEAM_CAPACITY_LOG_INTERVAL_MS="${LEOSIM_BEAM_CAPACITY_LOG_INTERVAL_MS:-1000}"
RNG_SEED="${LEOSIM_RNG_SEED:-20260803}"
RNG_RUN="${LEOSIM_RNG_RUN:-1}"
# Shared access/beam capacity and per-link access rate. Defaults are set to
# the upper-realistic Starlink-V2 range (per beam ~0.5-3 Gbps DL / 100-500 Mbps
# UL; per satellite ~20-80 Gbps DL / 5-10 Gbps UL; per-user ~100 Mbps access).
# For an access-load stress study, lower the beam capacity so the satellite
# pings contend for the same beam (e.g. LEOSIM_BEAM_DOWNLINK_CAPACITY=10Mbps).
ACCESS_DATA_RATE="${LEOSIM_ACCESS_DATA_RATE:-100Mbps}"
BEAM_UL_CAP="${LEOSIM_BEAM_UPLINK_CAPACITY:-500Mbps}"
BEAM_DL_CAP="${LEOSIM_BEAM_DOWNLINK_CAPACITY:-2Gbps}"
SAT_UL_CAP="${LEOSIM_SATELLITE_UPLINK_CAPACITY:-10Gbps}"
SAT_DL_CAP="${LEOSIM_SATELLITE_DOWNLINK_CAPACITY:-40Gbps}"
BEAM_SCHEDULER="${LEOSIM_BEAM_SCHEDULER:-alpha-fair}"
# UE -> serving-satellite ICMP pings: every loaded UE with container index >=
# SAT_PING_START_INDEX continuously pings whichever satellite it is currently
# connected to, following handovers. alpha_ue_0 is absent from the input (index
# 0 denotes alpha_ue_1), so the default SAT_PING_START_INDEX=0 includes every
# loaded UE in the ping load.
# A literal 2 MB ICMP payload is impossible (IPv4 permits at most 65,507 payload
# bytes) and fragmented ping replies are not measured reliably by ns-3 Ping.
# The default is a constant one ping per second per UE (no load ramp), so the
# aggregate ICMP load stays flat for the whole simulation. To model a growing
# load instead, set LEOSIM_SATELLITE_PING_LOAD_RAMP to a positive
# pings-per-second-per-second value; the per-UE rate then ramps from
# 1/interval toward LEOSIM_SATELLITE_PING_MAX_RATE (or the equivalent rate
# derived from LEOSIM_SATELLITE_PING_TARGET_MBPS) and is capped by it.
SAT_PING_START_INDEX="${LEOSIM_SATELLITE_PING_START_INDEX:-0}"
PING_APP_START="${LEOSIM_PING_APP_START:-1}"
SAT_PING_INTERVAL="${LEOSIM_SATELLITE_PING_INTERVAL:-1}"
SAT_PING_PACKET_SIZE="${LEOSIM_SATELLITE_PING_PACKET_SIZE:-1000}"
SAT_PING_STAGGER="${LEOSIM_SATELLITE_PING_STAGGER_WINDOW:-1}"
SAT_PING_TARGET_MBPS="${LEOSIM_SATELLITE_PING_TARGET_MBPS:-2}"
SAT_PING_RAMP_INTERVAL="${LEOSIM_SATELLITE_PING_RAMP_INTERVAL:-1}"
decimal_pattern='^([0-9]+([.][0-9]*)?|[.][0-9]+)$'
if [[ ! "${SIM_TIME}" =~ ^[0-9]+$ ]] || (( SIM_TIME < 3 )); then
    echo "LEOSIM_SIM_TIME must be an integer number of seconds greater than or equal to 3." >&2
    exit 2
fi
default_ping_max_rate=$(awk -v target_mbps="${SAT_PING_TARGET_MBPS}" \
    -v payload_bytes="${SAT_PING_PACKET_SIZE}" \
    'BEGIN { printf "%.12g\n", target_mbps * 1000000.0 / (payload_bytes * 8.0) }')
SAT_PING_MAX_RATE="${LEOSIM_SATELLITE_PING_MAX_RATE:-${default_ping_max_rate}}"
ping_app_stop=$((SIM_TIME - 1))
# Constant one-ping-per-second-per-UE load by default (ramp 0). Override with
# LEOSIM_SATELLITE_PING_LOAD_RAMP to enable a linear per-UE rate ramp toward
# LEOSIM_SATELLITE_PING_MAX_RATE over the application window.
SAT_PING_LOAD_RAMP="${LEOSIM_SATELLITE_PING_LOAD_RAMP:-0}"
# Effective per-UE peak payload rate actually reached at the end of the
# application window (rate0 + ramp*duration, capped at the max rate). With the
# default constant load this equals 1/interval pings per second.
SAT_PING_EFFECTIVE_PEAK_MBPS=$(awk -v payload_bytes="${SAT_PING_PACKET_SIZE}" \
    -v max_rate="${SAT_PING_MAX_RATE}" -v interval="${SAT_PING_INTERVAL}" \
    -v ramp="${SAT_PING_LOAD_RAMP}" -v start="${PING_APP_START}" \
    -v stop="${ping_app_stop}" -v ramp_interval="${SAT_PING_RAMP_INTERVAL}" \
    'BEGIN { rate0 = 1.0 / interval; duration = stop - start - ramp_interval; peak = rate0 + ramp * (duration > 0 ? duration : 0); if (peak > max_rate) peak = max_rate; if (peak < rate0) peak = rate0; printf "%.12g\n", payload_bytes * 8.0 * peak / 1000000.0 }')
if [[ ! "${NUM_UES}" =~ ^[0-9]+$ || ! "${SAT_PING_START_INDEX}" =~ ^[0-9]+$ ]]; then
    echo "NUM_UES and SATELLITE_PING_START_INDEX must be non-negative integers." >&2
    exit 2
fi
if (( SAT_PING_START_INDEX >= NUM_UES )); then
    echo "SATELLITE_PING_START_INDEX must be smaller than NUM_UES." >&2
    exit 2
fi
EXPECTED_PING_UES=$((NUM_UES - SAT_PING_START_INDEX))
if [[ ! "${SAT_PING_PACKET_SIZE}" =~ ^[0-9]+$ ||
      ! "${SAT_PING_TARGET_MBPS}" =~ ${decimal_pattern} ||
      ! "${SAT_PING_MAX_RATE}" =~ ${decimal_pattern} ||
      ! "${PING_APP_START}" =~ ${decimal_pattern} ||
      ! "${SAT_PING_INTERVAL}" =~ ${decimal_pattern} ||
      ! "${SAT_PING_RAMP_INTERVAL}" =~ ${decimal_pattern} ]] || \
   ! awk -v payload="${SAT_PING_PACKET_SIZE}" -v target="${SAT_PING_TARGET_MBPS}" \
       -v max_rate="${SAT_PING_MAX_RATE}" -v app_start="${PING_APP_START}" \
       -v app_stop="${ping_app_stop}" -v ramp_interval="${SAT_PING_RAMP_INTERVAL}" \
       -v capacity_update_ms="${BEAM_CAPACITY_UPDATE_INTERVAL_MS}" \
       -v capacity_log_ms="${BEAM_CAPACITY_LOG_INTERVAL_MS}" \
       'BEGIN { exit !(payload >= 16 && payload <= 1472 && target > 0 && max_rate > 0 && app_start >= 0 && app_stop > app_start + ramp_interval && capacity_update_ms > 0 && capacity_log_ms > 0) }'; then
    echo "Ping configuration requires a 16..1472-byte payload and positive target/max rate." >&2
    echo "The application window must exceed one ramp interval, and beam capacity update/log intervals must be positive." >&2
    echo "Larger payloads fragment on the 1500-byte access MTU." >&2
    exit 2
fi
# UE introduction: by default all UEs are active from t=0 -- no batched
# introduction. To enable periodic batches
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
WATCHDOG_INTERVAL="${LEOSIM_WATCHDOG_INTERVAL:-60}"
if [[ ! "${WATCHDOG_INTERVAL}" =~ ${decimal_pattern} ]] || \
   ! awk -v interval="${WATCHDOG_INTERVAL}" 'BEGIN { exit !(interval > 0) }'; then
    echo "LEOSIM_WATCHDOG_INTERVAL must be a positive number of seconds." >&2
    exit 2
fi
stage() {
    echo "[alpha-gs][$(date -u +%Y-%m-%dT%H:%M:%SZ)] $*"
}

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
run_id="alpha-gs-handover-comparison_trace-minsats-${SATELLITE_COUNT}_ues-${NUM_UES}_gss-${NUM_SERVERS}_duration-${SIM_TIME}s"
comparison_dir="${results_root}/${run_id}"
output_stem="result-job${SLURM_JOB_ID}-restart${SLURM_RESTART_COUNT:-0}-pid$$"
bho_run_dir="${comparison_dir}/bho"
cho1_run_dir="${comparison_dir}/cho1"
cho3_run_dir="${comparison_dir}/cho3"
bho_output_prefix="${bho_run_dir}/${output_stem}"
cho1_output_prefix="${cho1_run_dir}/${output_stem}"
cho3_output_prefix="${cho3_run_dir}/${output_stem}"
route_tree_root="/scratch/adesilva/tmp/leosim-route-trees-${SLURM_JOB_ID}"
bho_route_tree_dir="${route_tree_root}/bho"
cho1_route_tree_dir="${route_tree_root}/cho1"
cho3_route_tree_dir="${route_tree_root}/cho3"
mkdir -p "${bho_run_dir}" "${cho1_run_dir}" "${cho3_run_dir}" \
    "${bho_route_tree_dir}" "${cho1_route_tree_dir}" "${cho3_route_tree_dir}"
comparison_status_file="${comparison_dir}/exit-status.txt"
echo "RUNNING" > "${comparison_status_file}"
stage "job preparing: comparison_dir=${comparison_dir}, tasks=3, cpus_per_task=${allocated_cpus}"

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
    execution_mode="host"
    ns3_cmd=("${ns3_dir}/ns3" run --no-build)
    route_tree_engine="${leo_module_root}/utils/rengine/leosim-rengine"
    echo "Using host ns-3 at ${ns3_dir}/ns3"
    executable_help=$("${ns3_cmd[@]}" "leosim-experiments --PrintHelp" 2>/dev/null || true)
    for required_option in --enableDataTraffic --enableGroundStations --beamCapacityLogIntervalMs --satellitePingMetricsVersion; do
        if ! grep -q -- "${required_option}" <<< "${executable_help}"; then
            echo "Host leosim-experiments is missing ${required_option}; rebuild before submitting." >&2
            echo "Run contrib/leosim/utils/sonic/build_leosim.sh from the current repo." >&2
            exit 2
        fi
    done
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
        --bind "${route_tree_root}:${route_tree_root}" \
        --pwd /opt/leosim/ns3 "${SIF}" ./ns3 run --no-build)
    execution_mode="apptainer"
    route_tree_engine="${CONTAINER_NS3_ROOT}/contrib/leosim/utils/rengine/leosim-rengine"
    echo "Using containerized ns-3 from ${SIF}"
    container_help=$(apptainer exec --cleanenv --pwd /opt/leosim/ns3 "${SIF}" \
        ./ns3 run "leosim-experiments --PrintHelp" --no-build 2>/dev/null || true)
    for required_option in --enableDataTraffic --enableGroundStations --beamCapacityLogIntervalMs --satellitePingMetricsVersion; do
        if ! grep -q -- "${required_option}" <<< "${container_help}"; then
            echo "Container leosim-experiments is missing ${required_option}; rebuild the SIF." >&2
            echo "Run utils/sonic/build_leosim_container.sh from the current repo." >&2
            exit 2
        fi
    done
fi

# Hash the exact ground inputs and generated selection artifacts once for the
# whole comparison. This makes the removal of alpha_ue_0 and the container used
# by --no-build independently auditable without hashing large files three times.
provenance_file="${comparison_dir}/input-provenance.txt"
ue_input_file_count=0
ue_input_rows=0
gss_input_file_count=0
gss_input_rows=0
mapfile -d '' ue_input_files < <(find "${data_dir}/ues" -maxdepth 1 -type f -name '*.txt' -print0 | sort -z)
mapfile -d '' gss_input_files < <(find "${data_dir}/gss" -maxdepth 1 -type f -name '*.txt' -print0 | sort -z)
for input_file in "${ue_input_files[@]}"; do
    input_rows=$(awk 'NF { ++rows } END { print rows + 0 }' "${input_file}")
    ue_input_file_count=$((ue_input_file_count + 1))
    ue_input_rows=$((ue_input_rows + input_rows))
done
for input_file in "${gss_input_files[@]}"; do
    input_rows=$(awk 'NF { ++rows } END { print rows + 0 }' "${input_file}")
    gss_input_file_count=$((gss_input_file_count + 1))
    gss_input_rows=$((gss_input_rows + input_rows))
done
if [[ ${ue_input_file_count} -eq 0 || ${gss_input_file_count} -eq 0 ]]; then
    echo "No UE or ground-station .txt inputs found under ${data_dir}." >&2
    exit 3
fi
{
    echo "execution_mode=${execution_mode}"
    echo "ue_input_file_count=${ue_input_file_count}"
    echo "ue_input_nonempty_rows=${ue_input_rows}"
    echo "gss_input_file_count=${gss_input_file_count}"
    echo "gss_input_nonempty_rows=${gss_input_rows}"
    for input_file in "${ue_input_files[@]}" "${gss_input_files[@]}" \
        "${satellite_trace}" "${satellite_names}" "${satellite_selection}" "${coverage_ue_csv}"; do
        if [[ -f "${input_file}" ]]; then
            printf 'sha256[%s]=' "${input_file}"
            sha256sum "${input_file}" | awk '{ print $1 }'
        fi
    done
    if [[ "${execution_mode}" == "apptainer" ]]; then
        echo "container_image=${SIF}"
        printf 'sha256[%s]=' "${SIF}"
        sha256sum "${SIF}" | awk '{ print $1 }'
    fi
} > "${provenance_file}"
stage "provenance ready: ue_rows=${ue_input_rows}, gss_rows=${gss_input_rows} -> ${provenance_file}"

if [[ -x "${ns3_dir}/ns3" ]]; then
    if [[ ! -x "${route_tree_engine}" ]] || ! "${route_tree_engine}" --self-test >/dev/null; then
        echo "Missing or invalid routing engine: ${route_tree_engine}" >&2
        echo "Run contrib/leosim/utils/sonic/build_leosim.sh before submitting." >&2
        exit 2
    fi
else
    if ! apptainer exec --cleanenv \
        --bind "${route_tree_root}:${route_tree_root}" \
        --pwd /opt/leosim/ns3 "${SIF}" \
        "${route_tree_engine}" --self-test >/dev/null; then
        echo "Container routing engine failed its self-test: ${route_tree_engine}" >&2
        exit 2
    fi
fi

stage "ns-3 and routing engine ready; building matched policy commands"
export OMP_NUM_THREADS="${allocated_cpus}"
export NS_GLOBAL_VALUE="RngSeed=${RNG_SEED};RngRun=${RNG_RUN}"

common_command_line="leosim-experiments \
--dataDir=${data_dir} \
--satellites=${satellite_trace} \
--useTrace=1 \
--simTime=${SIM_TIME} \
--numSatellites=${selection_count} \
--satelliteSelectionFile=${satellite_selection} \
--numServers=${NUM_SERVERS} \
--numUes=${NUM_UES} \
--rngSeed=${RNG_SEED} --rngRun=${RNG_RUN} \
--enableDataTraffic=0 \
--allToAllTraffic=0 \
--enableGroundStations=${ENABLE_GROUND_STATIONS} \
--appStart=${PING_APP_START} --appStop=${ping_app_stop} \
--accessDataRate=${ACCESS_DATA_RATE} \
--enableSharedBeamCapacity=1 \
--beamScheduler=${BEAM_SCHEDULER} \
--beamUplinkCapacity=${BEAM_UL_CAP} --beamDownlinkCapacity=${BEAM_DL_CAP} \
--satelliteUplinkCapacity=${SAT_UL_CAP} --satelliteDownlinkCapacity=${SAT_DL_CAP} \
--beamCapacityUpdateIntervalMs=${BEAM_CAPACITY_UPDATE_INTERVAL_MS} \
--beamCapacityLogIntervalMs=${BEAM_CAPACITY_LOG_INTERVAL_MS} \
--enableBeamCapacityLogging=1 \
--enableDynamicGroundNodes=${UE_DYNAMIC_ENABLE} --initialUes=${UE_INITIAL} \
--ueIntroductionStart=${UE_INTRO_START} --ueIntroductionInterval=${UE_INTRO_INTERVAL} \
--uesPerIntroduction=${UE_INTRO_BATCH} --enableGroundNodeLifecycleLogging=0 \
--enableIsl=1 --maxIslNeighbors=4 --maxAccessSatellites=${MAX_ACCESS_SATS} \
--enableSyntheticIslLoad=0 --enableIslLoadLogging=0 \
--enableDynamicRouting=1 --routingUpdateInterval=${UPDATE_INTERVAL} \
--useRouteTreeCache=1 --routeTreeWorkers=${allocated_cpus} \
--routeTreeEngine=${route_tree_engine} \
--enableHoBuffering=1 --hoBufferSize=1024 \
--ttt=1 --t310=1 --n310=3 --n311=3 \
--a3Offset=3 --a4Threshold=-110 --tteTrigger=30 \
--choPrep=100 --choExec=150 --beamUpdateIntervalMs=${BEAM_UPDATE_INTERVAL_MS} \
--enableStatistics=1 --statisticsInterval=1 \
--enableHandoverLogging=1 --writeFlowMonitor=1 \
--enableTaskProfiler=1 --progressLogInterval=120 \
--enableVisualization=0 --verbose=0 \
--enableUePing=0 \
--enableSatellitePing=1 \
--satellitePingStartIndex=${SAT_PING_START_INDEX} \
--satellitePingInterval=${SAT_PING_INTERVAL} \
--satellitePingPacketSize=${SAT_PING_PACKET_SIZE} \
--satellitePingStaggerWindow=${SAT_PING_STAGGER} \
--satellitePingLoadRamp=${SAT_PING_LOAD_RAMP} \
--satellitePingMaxRate=${SAT_PING_MAX_RATE} \
--satellitePingRampInterval=${SAT_PING_RAMP_INTERVAL} \
--satellitePingMetricsVersion=1 \
--uniqueOutputPrefix=0"

bho_command_line="${common_command_line} --hoMode=BHO --maxCandidates=1 --routeTreeWorkDir=${bho_route_tree_dir} --outputPrefix=${bho_output_prefix}"
cho1_command_line="${common_command_line} --hoMode=CHO --maxCandidates=1 --routeTreeWorkDir=${cho1_route_tree_dir} --outputPrefix=${cho1_output_prefix}"
cho3_command_line="${common_command_line} --hoMode=CHO --maxCandidates=3 --routeTreeWorkDir=${cho3_route_tree_dir} --outputPrefix=${cho3_output_prefix}"

write_metadata() {
    local policy="$1"
    local ho_mode="$2"
    local max_candidates="$3"
    local policy_run_dir="$4"
    local policy_output_prefix="$5"
    local policy_route_tree_dir="$6"
    local policy_command_line="$7"
    local commit
    commit=$(git -C "${repo_root}" rev-parse HEAD 2>/dev/null || true)
    {
        echo "comparison_id=${run_id}"
        echo "policy=${policy}"
        echo "ho_mode=${ho_mode}"
        echo "max_candidates=${max_candidates}"
        echo "traffic=satellite_ping_only"
        echo "data_traffic_enabled=0"
        echo "enable_ground_stations=${ENABLE_GROUND_STATIONS}"
        echo "output_prefix=${policy_output_prefix}"
        echo "route_tree_work_dir=${policy_route_tree_dir}"
        echo "job_id=${SLURM_JOB_ID}"
        echo "hostname=$(hostname)"
        echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
        echo "sim_time_s=${SIM_TIME}"
        echo "minimum_preprocessed_satellites=${SATELLITE_COUNT}"
        echo "selected_satellites=${selection_count}"
        echo "satellite_selection_file=${satellite_selection}"
        echo "coverage_ue_area_csv=${coverage_ue_csv}"
        echo "tle_timestep_s=${TLE_TIMESTEP}"
        echo "num_servers=${NUM_SERVERS}"
        echo "num_ues=${NUM_UES}"
        echo "max_access_satellites=${MAX_ACCESS_SATS}"
        echo "access_data_rate=${ACCESS_DATA_RATE}"
        echo "beam_uplink_capacity=${BEAM_UL_CAP}"
        echo "beam_downlink_capacity=${BEAM_DL_CAP}"
        echo "satellite_uplink_capacity=${SAT_UL_CAP}"
        echo "satellite_downlink_capacity=${SAT_DL_CAP}"
        echo "beam_scheduler=${BEAM_SCHEDULER}"
        echo "shared_beam_capacity_enabled=1"
        echo "beam_capacity_logging=1"
        echo "beam_capacity_update_interval_ms=${BEAM_CAPACITY_UPDATE_INTERVAL_MS}"
        echo "beam_capacity_log_interval_ms=${BEAM_CAPACITY_LOG_INTERVAL_MS}"
        echo "ue_pair_ping_enabled=0"
        echo "satellite_ping_enabled=1"
        echo "satellite_ping_start_index=${SAT_PING_START_INDEX}"
        echo "satellite_ping_app_start_s=${PING_APP_START}"
        echo "satellite_ping_app_stop_s=${ping_app_stop}"
        echo "satellite_ping_interval_s=${SAT_PING_INTERVAL}"
        echo "satellite_ping_packet_size_bytes=${SAT_PING_PACKET_SIZE}"
        echo "satellite_ping_target_payload_mbps=${SAT_PING_TARGET_MBPS}"
        echo "satellite_ping_effective_peak_payload_mbps=${SAT_PING_EFFECTIVE_PEAK_MBPS}"
        echo "satellite_ping_stagger_window_s=${SAT_PING_STAGGER}"
        echo "satellite_ping_load_ramp_pps_per_s=${SAT_PING_LOAD_RAMP}"
        echo "satellite_ping_max_rate_pps=${SAT_PING_MAX_RATE}"
        echo "satellite_ping_ramp_interval_s=${SAT_PING_RAMP_INTERVAL}"
        echo "satellite_ping_metrics_version=1"
        echo "dynamic_ue_introduction=${UE_DYNAMIC_ENABLE}"
        echo "initial_ues=${UE_INITIAL}"
        echo "ue_introduction_start_s=${UE_INTRO_START}"
        echo "ue_introduction_interval_s=${UE_INTRO_INTERVAL}"
        echo "ues_per_introduction=${UE_INTRO_BATCH}"
        echo "routing_update_interval_s=${UPDATE_INTERVAL}"
        echo "beam_update_interval_ms=${BEAM_UPDATE_INTERVAL_MS}"
        echo "rng_seed=${RNG_SEED}"
        echo "rng_run=${RNG_RUN}"
        echo "ns_global_value=${NS_GLOBAL_VALUE}"
        echo "data_dir=${data_dir}"
        echo "satellite_trace=${satellite_trace}"
        echo "prepared_satellites=${prepared_satellites}"
        echo "trace_duration_s=${trace_duration}"
        echo "execution_mode=${execution_mode}"
        echo "input_provenance=${provenance_file}"
        echo "ue_input_nonempty_rows=${ue_input_rows}"
        echo "gss_input_nonempty_rows=${gss_input_rows}"
        echo "commit=${commit:-unavailable}"
        echo "command=${policy_command_line}"
    } > "${policy_run_dir}/metadata.txt"
}

write_metadata "BHO" "BHO" 1 "${bho_run_dir}" "${bho_output_prefix}" \
    "${bho_route_tree_dir}" "${bho_command_line}"
write_metadata "CHO-1" "CHO" 1 "${cho1_run_dir}" "${cho1_output_prefix}" \
    "${cho1_route_tree_dir}" "${cho1_command_line}"
write_metadata "CHO-3" "CHO" 3 "${cho3_run_dir}" "${cho3_output_prefix}" \
    "${cho3_route_tree_dir}" "${cho3_command_line}"

if command -v srun >/dev/null 2>&1; then
    step_prefix=(srun --exclusive --nodes=1 --ntasks=1 \
        --cpus-per-task="${allocated_cpus}" --cpu-bind=cores)
else
    step_prefix=()
    stage "warning: srun unavailable; launching without Slurm CPU binding"
fi
if [[ -x /usr/bin/time ]]; then
    time_prefix=(/usr/bin/time -v)
else
    time_prefix=()
fi

run_experiment() {
    local policy_run_dir="$1"
    local policy_command_line="$2"
    OMP_NUM_THREADS="${allocated_cpus}" \
        "${step_prefix[@]}" "${time_prefix[@]}" "${ns3_cmd[@]}" "${policy_command_line}" \
        > "${policy_run_dir}/stdout.log" 2> "${policy_run_dir}/stderr.log"
}

log_progress() {
    local policy="$1"
    local experiment_pid="$2"
    local policy_run_dir="$3"
    local state="finished"
    if kill -0 "${experiment_pid}" 2>/dev/null; then
        state="running"
    fi
    local last_line
    local loglines
    last_line=$(tail -n 1 "${policy_run_dir}/stdout.log" 2>/dev/null || true)
    loglines=$(wc -l < "${policy_run_dir}/stdout.log" 2>/dev/null || echo 0)
    printf '[alpha-gs][%s] policy=%s state=%s t+%ss pid=%s loglines=%s last=[%s]\n' \
        "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "${policy}" "${state}" "${elapsed}" \
        "${experiment_pid}" "${loglines}" "${last_line}" \
        | tee -a "${policy_run_dir}/watchdog.log"
}

stage "starting BHO, CHO-1, and CHO-3 (simTime=${SIM_TIME}s, ues=${NUM_UES}, sats=${selection_count}, 3x${allocated_cpus} CPUs)"
run_start_epoch=$(date +%s)
set +e
run_experiment "${bho_run_dir}" "${bho_command_line}" &
bho_pid=$!
run_experiment "${cho1_run_dir}" "${cho1_command_line}" &
cho1_pid=$!
run_experiment "${cho3_run_dir}" "${cho3_command_line}" &
cho3_pid=$!

policy_names=("BHO" "CHO-1" "CHO-3")
policy_pids=("${bho_pid}" "${cho1_pid}" "${cho3_pid}")
policy_run_dirs=("${bho_run_dir}" "${cho1_run_dir}" "${cho3_run_dir}")

while :; do
    active=0
    for experiment_pid in "${policy_pids[@]}"; do
        if kill -0 "${experiment_pid}" 2>/dev/null; then
            active=$((active + 1))
        fi
    done
    if [[ ${active} -eq 0 ]]; then
        break
    fi
    sleep "${WATCHDOG_INTERVAL}"
    elapsed=$(( $(date +%s) - run_start_epoch ))
    for index in "${!policy_pids[@]}"; do
        log_progress "${policy_names[index]}" "${policy_pids[index]}" "${policy_run_dirs[index]}"
    done
done

policy_statuses=()
overall_status=0
finished_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
for index in "${!policy_pids[@]}"; do
    wait "${policy_pids[index]}"
    status=$?
    policy_statuses+=("${status}")
    echo "${status}" > "${policy_run_dirs[index]}/exit-status.txt"
    echo "${finished_utc}" > "${policy_run_dirs[index]}/finished-utc.txt"
    if [[ ${status} -ne 0 && ${overall_status} -eq 0 ]]; then
        overall_status=${status}
    fi
    stage "${policy_names[index]} finished: status=${status}"
done
set -e

if [[ ${overall_status} -ne 0 ]]; then
    echo "At least one policy failed: BHO=${policy_statuses[0]}, CHO-1=${policy_statuses[1]}, CHO-3=${policy_statuses[2]}." >&2
    echo "See the per-policy stderr.log files under ${comparison_dir}." >&2
    exit "${overall_status}"
fi

declare -A policy_ping_tx_packets
declare -A policy_ping_rx_packets
declare -A policy_ping_tx_bytes
declare -A policy_ping_rx_bytes

validate_outputs() {
    local policy="$1"
    local policy_run_dir="$2"
    local policy_output_prefix="$3"
    local required_outputs=(
        "${policy_output_prefix}-statistics.csv"
        "${policy_output_prefix}-statistics.json"
        "${policy_output_prefix}-handovers.csv"
        "${policy_output_prefix}-cho-candidates.csv"
        "${policy_output_prefix}-beam-capacity.csv"
        "${policy_output_prefix}-satellite-ping.csv"
        "${policy_output_prefix}-satellite-ping-timeseries.csv"
        "${policy_output_prefix}-flowmon.xml"
    )
    local output
    for output in "${required_outputs[@]}"; do
        if [[ ! -s "${output}" ]]; then
            echo "Required ${policy} output missing or empty: ${output}" >&2
            echo 3 > "${policy_run_dir}/exit-status.txt"
            return 3
        fi
    done

    local capacity_csv="${policy_output_prefix}-beam-capacity.csv"
    local expected_capacity_header="time_s,satellite_id,beam_id,ground_id,associated_nodes,active_uplink_nodes,active_downlink_nodes,uplink_beam_capacity_bps,downlink_beam_capacity_bps,uplink_package_cap_bps,downlink_package_cap_bps,uplink_minimum_bps,downlink_minimum_bps,uplink_queue_bytes,downlink_queue_bytes,uplink_bps,downlink_bps,sinr_db,scheduler,uplink_limit_reason,downlink_limit_reason"
    local actual_capacity_header
    actual_capacity_header=$(sed -n '1p' "${capacity_csv}")
    if [[ "${actual_capacity_header}" != "${expected_capacity_header}" ]] || \
       [[ -z "$(sed -n '2p' "${capacity_csv}")" ]]; then
        echo "Invalid ${policy} beam-capacity CSV schema or missing data rows: ${capacity_csv}" >&2
        echo 4 > "${policy_run_dir}/exit-status.txt"
        return 4
    fi
    # Stream every snapshot so truncation, malformed later rows, and any
    # directional over-allocation are caught without retaining the full CSV.
    if ! awk -F, -v expected_scheduler="${BEAM_SCHEDULER}" \
        -v expected_interval_ms="${BEAM_CAPACITY_LOG_INTERVAL_MS}" \
        -v update_interval_ms="${BEAM_CAPACITY_UPDATE_INTERVAL_MS}" \
        -v sim_time="${SIM_TIME}" '
        function is_uint(value) { return value ~ /^[0-9]+$/ }
        BEGIN {
            expected_interval = expected_interval_ms / 1000.0
            update_interval = update_interval_ms / 1000.0
            # CSV times use the default C++ stream precision, so allow a small
            # rounding margin when comparing long-simulation timestamps.
            tolerance = 0.02
            minimum_snapshots = (sim_time > expected_interval + update_interval + tolerance) ? 2 : 1
        }
        function finish_snapshot(beam_key) {
            if (!have_snapshot) { return }
            for (beam_key in seen) {
                if (uplink_sum[beam_key] > uplink_budget[beam_key] ||
                    downlink_sum[beam_key] > downlink_budget[beam_key]) {
                    invalid = 1
                }
                delete seen[beam_key]
                delete uplink_budget[beam_key]
                delete downlink_budget[beam_key]
                delete uplink_sum[beam_key]
                delete downlink_sum[beam_key]
            }
            ++snapshot_count
            last_snapshot_time = current_time + 0
            have_snapshot = 0
        }
        NR == 1 { next }
        {
            if (NF != 21 || $1 !~ /^[0-9]+([.][0-9]+)?$/ ||
                !is_uint($8) || !is_uint($9) || !is_uint($16) || !is_uint($17) ||
                $19 != expected_scheduler) {
                invalid = 1
                exit
            }
            if (!have_snapshot) {
                current_time = $1
                have_snapshot = 1
            } else if ($1 != current_time) {
                delta = ($1 + 0) - (current_time + 0)
                if (delta <= 0 || delta + tolerance < expected_interval) {
                    invalid = 1
                    exit
                }
                finish_snapshot()
                current_time = $1
                have_snapshot = 1
            }
            beam_key = $2 SUBSEP $3
            if (beam_key in seen &&
                (uplink_budget[beam_key] != $8 || downlink_budget[beam_key] != $9)) {
                invalid = 1
                exit
            }
            seen[beam_key] = 1
            uplink_budget[beam_key] = $8
            downlink_budget[beam_key] = $9
            uplink_sum[beam_key] += $16
            downlink_sum[beam_key] += $17
        }
        END {
            finish_snapshot()
            maximum_tail_gap = expected_interval + update_interval + tolerance
            if (snapshot_count < minimum_snapshots ||
                last_snapshot_time + maximum_tail_gap < sim_time) {
                invalid = 1
            }
            exit invalid
        }
    ' "${capacity_csv}"; then
        echo "Invalid ${policy} beam-capacity timeline or allocation totals: ${capacity_csv}" >&2
        echo 4 > "${policy_run_dir}/exit-status.txt"
        return 4
    fi

    local ping_csv="${policy_output_prefix}-satellite-ping.csv"
    local ping_timeseries_csv="${policy_output_prefix}-satellite-ping-timeseries.csv"
    local expected_ping_header="ue_index,ue_node_id,expected_start_s,measurement_duration_s,tx_packets,rx_packets,lost_packets,drop_events,tx_payload_bytes,rx_payload_bytes,offered_payload_mbps,received_payload_mbps,delivery_ratio,rtt_samples,rtt_mean_ms,rtt_min_ms,rtt_max_ms,target_changes,last_satellite_id,first_tx_s,last_tx_s"
    local expected_ping_timeseries_header="time_start_s,time_end_s,active_ues,tx_packets,rx_packets,drop_events,tx_payload_bytes,rx_payload_bytes,offered_payload_mbps,received_payload_mbps,delivery_ratio,rtt_samples,rtt_mean_ms,rtt_min_ms,rtt_max_ms"
    if [[ "$(sed -n '1p' "${ping_csv}")" != "${expected_ping_header}" ]] || \
       [[ "$(sed -n '1p' "${ping_timeseries_csv}")" != "${expected_ping_timeseries_header}" ]]; then
        echo "Invalid ${policy} satellite-ping CSV schema." >&2
        echo 4 > "${policy_run_dir}/exit-status.txt"
        return 4
    fi

    local ping_summary
    if ! ping_summary=$(awk -F, -v first_index="${SAT_PING_START_INDEX}" \
        -v num_ues="${NUM_UES}" -v expected_rows="${EXPECTED_PING_UES}" \
        -v payload_bytes="${SAT_PING_PACKET_SIZE}" '
        function is_uint(value) { return value ~ /^[0-9]+$/ }
        NR == 1 { next }
        {
            if (NF != 21 || !is_uint($1) || !is_uint($2) ||
                !is_uint($5) || !is_uint($6) || !is_uint($7) || !is_uint($8) ||
                !is_uint($9) || !is_uint($10) || !is_uint($14) ||
                !is_uint($18) || !is_uint($19)) {
                invalid = 1
                next
            }
            ue_index = $1 + 0
            if (ue_index < first_index || ue_index >= num_ues || seen[ue_index]++) {
                invalid = 1
            }
            if ($5 == 0 || $6 > $5 || $7 != $5 - $6 || $14 != $6 ||
                $9 != $5 * payload_bytes || $10 != $6 * payload_bytes) {
                invalid = 1
            }
            ++rows
            tx_packets += $5
            rx_packets += $6
            tx_bytes += $9
            rx_bytes += $10
        }
        END {
            if (invalid || rows != expected_rows || tx_packets <= 0 || rx_packets <= 0) {
                exit 1
            }
            printf "%.0f,%.0f,%.0f,%.0f\n", tx_packets, rx_packets, tx_bytes, rx_bytes
        }
    ' "${ping_csv}"); then
        echo "Invalid ${policy} per-UE satellite-ping coverage or counters: ${ping_csv}" >&2
        echo 4 > "${policy_run_dir}/exit-status.txt"
        return 4
    fi

    local ping_tx_packets_value
    local ping_rx_packets_value
    local ping_tx_bytes_value
    local ping_rx_bytes_value
    IFS=',' read -r ping_tx_packets_value ping_rx_packets_value \
        ping_tx_bytes_value ping_rx_bytes_value <<< "${ping_summary}"

    if ! awk -F, -v expected_tx="${ping_tx_packets_value}" \
        -v expected_rx="${ping_rx_packets_value}" -v expected_tx_bytes="${ping_tx_bytes_value}" \
        -v expected_rx_bytes="${ping_rx_bytes_value}" -v payload_bytes="${SAT_PING_PACKET_SIZE}" '
        function is_uint(value) { return value ~ /^[0-9]+$/ }
        NR == 1 { next }
        {
            if (NF != 15 || $1 !~ /^[0-9]+$/ || $2 !~ /^[0-9]+$/ ||
                !is_uint($3) || !is_uint($4) || !is_uint($5) || !is_uint($6) ||
                !is_uint($7) || !is_uint($8) || !is_uint($12) ||
                (rows > 0 && $1 <= previous_time) || $7 != $4 * payload_bytes ||
                $8 != $5 * payload_bytes) {
                invalid = 1
            }
            previous_time = $1
            ++rows
            tx_packets += $4
            rx_packets += $5
            tx_bytes += $7
            rx_bytes += $8
        }
        END {
            if (invalid || rows < 2 || tx_packets != expected_tx || rx_packets != expected_rx ||
                tx_bytes != expected_tx_bytes || rx_bytes != expected_rx_bytes) {
                exit 1
            }
        }
    ' "${ping_timeseries_csv}"; then
        echo "Invalid ${policy} satellite-ping time series: ${ping_timeseries_csv}" >&2
        echo 4 > "${policy_run_dir}/exit-status.txt"
        return 4
    fi

    policy_ping_tx_packets["${policy}"]="${ping_tx_packets_value}"
    policy_ping_rx_packets["${policy}"]="${ping_rx_packets_value}"
    policy_ping_tx_bytes["${policy}"]="${ping_tx_bytes_value}"
    policy_ping_rx_bytes["${policy}"]="${ping_rx_bytes_value}"

    local summary_json="${policy_output_prefix}-statistics.json"
    local tx_packets
    local rx_packets
    local sink_count
    tx_packets=$(sed -n 's/.*"network": {"tx_packets": \([0-9][0-9]*\).*/\1/p' "${summary_json}")
    rx_packets=$(sed -n 's/.*"rx_packets": \([0-9][0-9]*\).*/\1/p' "${summary_json}")
    sink_count=$(sed -n 's/.*"sink_count": \([0-9][0-9]*\).*/\1/p' "${summary_json}")
    if [[ -z "${tx_packets}" || -z "${rx_packets}" || -z "${sink_count}" ]]; then
        echo "Could not validate ${policy} ping statistics summary: ${summary_json}" >&2
        echo 4 > "${policy_run_dir}/exit-status.txt"
        return 4
    fi
    # FlowMonitor classifies only TCP/UDP, while ns-3 Ping uses ICMP. A ping-only
    # run must therefore have no FlowMonitor packets or registered PacketSinks.
    if [[ "${tx_packets}" -ne 0 || "${rx_packets}" -ne 0 || "${sink_count}" -ne 0 ]]; then
        echo "Invalid ${policy} ping-only run: TCP/UDP tx_packets=${tx_packets}, rx_packets=${rx_packets}, sink_count=${sink_count}." >&2
        echo 4 > "${policy_run_dir}/exit-status.txt"
        return 4
    fi
    if grep -Eq '"protocol": (6|17)(,|})' "${summary_json}"; then
        echo "Invalid ${policy} ping-only run: TCP/UDP flow found in ${summary_json}." >&2
        echo 4 > "${policy_run_dir}/exit-status.txt"
        return 4
    fi
    stage "${policy} outputs validated: ping_tx=${ping_tx_packets_value}, ping_rx=${ping_rx_packets_value}, capacity_csv=${capacity_csv}"
}

validation_status=0
set +e
validate_outputs "BHO" "${bho_run_dir}" "${bho_output_prefix}"
validation_status=$((validation_status | $?))
validate_outputs "CHO-1" "${cho1_run_dir}" "${cho1_output_prefix}"
validation_status=$((validation_status | $?))
validate_outputs "CHO-3" "${cho3_run_dir}" "${cho3_output_prefix}"
validation_status=$((validation_status | $?))
set -e
if [[ ${validation_status} -ne 0 ]]; then
    exit "${validation_status}"
fi

ping_comparison_csv="${comparison_dir}/ping-traffic-comparison.csv"
{
    echo "policy,tx_packets,rx_packets,tx_payload_bytes,rx_payload_bytes"
    for policy in "${policy_names[@]}"; do
        echo "${policy},${policy_ping_tx_packets[${policy}]},${policy_ping_rx_packets[${policy}]},${policy_ping_tx_bytes[${policy}]},${policy_ping_rx_bytes[${policy}]}"
    done
} > "${ping_comparison_csv}"

reference_tx_bytes="${policy_ping_tx_bytes[BHO]}"
for policy in "CHO-1" "CHO-3"; do
    if ! awk -v reference="${reference_tx_bytes}" -v candidate="${policy_ping_tx_bytes[${policy}]}" '
        BEGIN {
            difference = reference > candidate ? reference - candidate : candidate - reference
            exit !(reference > 0 && difference / reference <= 0.001)
        }
    '; then
        echo "Actual ${policy} offered ping payload differs from BHO by more than 0.1%." >&2
        mismatch_run_dir="${cho3_run_dir}"
        if [[ "${policy}" == "CHO-1" ]]; then
            mismatch_run_dir="${cho1_run_dir}"
        fi
        echo 5 > "${mismatch_run_dir}/exit-status.txt"
        exit 5
    fi
done

date -u +%Y-%m-%dT%H:%M:%SZ > "${comparison_dir}/completed-utc.txt"
stage "COMPLETE: ${run_id}"
echo "Completed matched BHO, CHO-1, and CHO-3 comparison: ${comparison_dir}"
