#!/usr/bin/env bash

# Submit all 16 tasks with one common eligibility time. Their maximum combined
# request is 44 CPUs: scalability 3x6, routing 7x2, and handover 6x2.

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
start_time=${LEOSIM_START_TIME:-now+2minutes}
node_args=()
if [[ -n "${LEOSIM_NODE:-}" ]]; then
    node_args=(--nodelist="${LEOSIM_NODE}")
fi

submit() {
    sbatch --parsable --begin="${start_time}" "${node_args[@]}" "$1"
}

scalability_job=$(submit "${script_dir}/sonic-hop-scalability.sbatch")
routing_job=$(submit "${script_dir}/run_leosim_routing.sbatch")
handover_job=$(submit "${script_dir}/sonic-handover-paper.sbatch")

echo "Submitted all campaigns for ${start_time}:"
echo "  scalability=${scalability_job} (3 tasks x 6 CPUs = 18)"
echo "  routing=${routing_job} (7 tasks x 2 CPUs = 14)"
echo "  handover=${handover_job} (6 tasks x 2 CPUs = 12)"
echo "  total=16 tasks, 44 CPUs; 3 of 47 CPUs remain free"
if [[ -z "${LEOSIM_NODE:-}" ]]; then
    echo "Set LEOSIM_NODE=<hostname> to pin all arrays to one specific server."
fi
