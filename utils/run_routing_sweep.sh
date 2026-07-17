#!/usr/bin/env bash
# Run leosim-param-scenario across routing metrics and topology parameters.
# Override any sweep dimension with an environment variable, for example:
#   METRICS="hop snr" SATELLITE_COUNTS="50 100" ROUTING_MODES="dynamic" \
#   UPDATE_INTERVALS="5 10" SIM_TIMES="180 360" \
#   ./contrib/leosim/utils/run_routing_sweep.sh

set -uo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ns3_dir="$(cd -- "${script_dir}/../../.." && pwd)"
cd "${ns3_dir}"

read -r -a metrics <<< "${METRICS:-hop distance path-loss snr signal-strength}"
read -r -a satellite_counts <<< "${SATELLITE_COUNTS:-10 50 100}"
read -r -a routing_modes <<< "${ROUTING_MODES:-static dynamic}"
read -r -a update_intervals <<< "${UPDATE_INTERVALS:-5 10 30}"
read -r -a sim_times <<< "${SIM_TIMES:-${SIM_TIME:-60}}"

app_start="${APP_START:-1}"
tcp_rate="${TCP_RATE:-1Mbps}"
max_isl_neighbors="${MAX_ISL_NEIGHBORS:-4}"
statistics_interval="${STATISTICS_INTERVAL:-1}"
enable_visualization="${ENABLE_VISUALIZATION:-0}"
results_dir="${RESULTS_DIR:-routing-sweep-results}"

mkdir -p "${results_dir}"
manifest="${results_dir}/manifest.csv"
echo "run_id,metric,satellites,mode,update_interval_s,sim_time_s,tcp_rate,status,wall_time_s,output_prefix,route_log_file" > "${manifest}"

failures=0
runs=0

run_case() {
    local metric="$1"
    local satellites="$2"
    local mode="$3"
    local interval="$4"
    local sim_time="$5"
    local app_stop="${APP_STOP:-$((sim_time - 1))}"
    local dynamic=0
    local interval_label="none"

    if [[ "${mode}" == "dynamic" ]]; then
        dynamic=1
        interval_label="${interval}"
    fi

    local run_id="metric-${metric}_sats-${satellites}_${mode}_duration-${sim_time}s"
    if [[ "${mode}" == "dynamic" ]]; then
        run_id+="-interval-${interval}s"
    fi

    local run_dir="${results_dir}/${run_id}"
    local prefix="${run_dir}/result"
    local log_file="${run_dir}/run.log"
    mkdir -p "${run_dir}"

    local args="leosim-param-scenario"
    args+=" --numSatellites=${satellites}"
    args+=" --routingMetric=${metric}"
    args+=" --enableDynamicRouting=${dynamic}"
    args+=" --routingUpdateInterval=${interval}"
    args+=" --maxIslNeighbors=${max_isl_neighbors}"
    args+=" --simTime=${sim_time}"
    args+=" --appStart=${app_start}"
    args+=" --appStop=${app_stop}"
    args+=" --tcpRate=${tcp_rate}"
    args+=" --enableStatistics=1"
    args+=" --enableRouteLogging=1"
    args+=" --statisticsInterval=${statistics_interval}"
    args+=" --enableVisualization=${enable_visualization}"
    args+=" --outputPrefix=${prefix}"

    echo "[RUN] ${run_id}"
    local started="${SECONDS}"
    if env XDG_CONFIG_HOME="${XDG_CONFIG_HOME:-/tmp/leosim-xdg-config}" \
           CCACHE_DIR="${CCACHE_DIR:-/tmp/leosim-ccache}" \
           ./ns3 run "${args}" > "${log_file}" 2>&1; then
        status="PASS"
    else
        status="FAIL"
        failures=$((failures + 1))
    fi
    local elapsed=$((SECONDS - started))
    runs=$((runs + 1))

    echo "${run_id},${metric},${satellites},${mode},${interval_label},${sim_time},${tcp_rate},${status},${elapsed},${prefix},${prefix}-routes.csv" >> "${manifest}"
    echo "[${status}] ${run_id} (${elapsed}s)"
}

for metric in "${metrics[@]}"; do
    for satellites in "${satellite_counts[@]}"; do
        for sim_time in "${sim_times[@]}"; do
            for mode in "${routing_modes[@]}"; do
                if [[ "${mode}" == "static" ]]; then
                    run_case "${metric}" "${satellites}" "${mode}" "10" "${sim_time}"
                elif [[ "${mode}" == "dynamic" ]]; then
                    for interval in "${update_intervals[@]}"; do
                        run_case "${metric}" "${satellites}" "${mode}" "${interval}" "${sim_time}"
                    done
                else
                    echo "Unknown routing mode: ${mode}" >&2
                    exit 2
                fi
            done
        done
    done
done

echo "Completed ${runs} runs with ${failures} failure(s). Manifest: ${manifest}"
exit "${failures}"
