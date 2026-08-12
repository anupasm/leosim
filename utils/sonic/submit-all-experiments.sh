#!/usr/bin/env bash

# Submit all 36 tasks with one common eligibility time. Routing consists only
# of the 27-task ALDSR campaign. The combined request is 45 CPUs:
# scalability 3x4, ALDSR 27x1, and handover 6x1.

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
start_time=${LEOSIM_START_TIME:-now+2minutes}
node_args=()
if [[ -n "${LEOSIM_NODE:-}" ]]; then
    node_args=(--nodelist="${LEOSIM_NODE}")
fi

submit() {
    sbatch --parsable --begin="${start_time}" "${node_args[@]}" "$@"
}

scalability_job=$(submit "${script_dir}/sonic-hop-scalability.sbatch")
aldsr_job=$(submit "${script_dir}/run_aldsr_weight_search.sbatch")
handover_job=$(submit \
    --export="ALL,LEOSIM_RESULTS_ROOT=/scratch/adesilva/results/leosim-handover" \
    "${script_dir}/sonic-handover-paper.sbatch")

echo "Submitted all campaigns for ${start_time}:"
echo "  scalability=${scalability_job} (3 tasks x 4 CPUs = 12)"
echo "  routing/ALDSR=${aldsr_job} (27 tasks x 1 CPU = 27)"
echo "  handover=${handover_job} (6 tasks x 1 CPU = 6)"
echo "  total=36 tasks, 45 CPUs; 2 of 47 CPUs remain free"
if [[ -z "${LEOSIM_NODE:-}" ]]; then
    echo "Set LEOSIM_NODE=<hostname> to pin all arrays to one specific server."
fi
