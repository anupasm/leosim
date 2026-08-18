#!/usr/bin/env bash

# Submit one or more LeoSim campaigns with a common eligibility time.
#
# Usage:
#   submit-all-experiments.sh [campaign[,campaign...]]
#
# campaign is one of: all | scalability | aldsr | handover | alpha-gs
# (a comma-separated list selects several; the default is "all").
#
# Examples:
#   submit-all-experiments.sh               # submit every campaign (37 tasks, 46 CPUs)
#   submit-all-experiments.sh alpha-gs      # run ONLY the alpha-GS handover-ping experiment
#   submit-all-experiments.sh handover,alpha-gs
#   LEOSIM_CAMPAIGNS=alpha-gs submit-all-experiments.sh   # same via env
#
# CPU budget (47 total): scalability 3x4=12, ALDSR 27x1=27,
# handover 6x1=6, alpha-gs 1x1=1.

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
start_time=${LEOSIM_START_TIME:-now+2minutes}
# CPUs for the alpha-GS job; used by its parallel TLE/route-tree stages. The
# default keeps the full-campaign budget at 46 CPUs; raise it when running the
# alpha-GS campaign alone (up to 47).
alpha_gs_cpus=${LEOSIM_ALPHA_GS_CPUS:-1}
node_args=()
if [[ -n "${LEOSIM_NODE:-}" ]]; then
    node_args=(--nodelist="${LEOSIM_NODE}")
fi

submit() {
    sbatch --parsable --begin="${start_time}" "${node_args[@]}" "$@"
}

# campaign -> "sbatch_file|tasks|cpus_per_task|export_args"
declare -A CAMPAIGN_SPEC=(
    [scalability]="sonic-hop-scalability.sbatch|3|4|"
    [aldsr]="run_aldsr_weight_search.sbatch|27|1|"
    [handover]="sonic-handover-paper.sbatch|6|1|LEOSIM_RESULTS_ROOT=/scratch/adesilva/results/leosim-handover"
    [alpha-gs]="sonic-alpha-gs-handover-ping.sbatch|1|${alpha_gs_cpus}|LEOSIM_RESULTS_ROOT=/scratch/adesilva/results/leosim-alpha-gs-handover-ping"
)

requested="${1:-all}"
if [[ -n "${LEOSIM_CAMPAIGNS:-}" ]]; then
    requested="${LEOSIM_CAMPAIGNS}"
fi

if [[ "${requested}" == "all" ]]; then
    selected=(scalability aldsr handover alpha-gs)
else
    IFS=',' read -r -a selected <<< "${requested}"
    if [[ " ${selected[*]} " == *" all "* ]]; then
        selected=(scalability aldsr handover alpha-gs)
    fi
fi

campaign_jobs=()
labels=()
tasks=0
cpus=0
for campaign in "${selected[@]}"; do
    [[ -z "${campaign}" ]] && continue
    if [[ -z "${CAMPAIGN_SPEC[${campaign}]+x}" ]]; then
        echo "Unknown campaign: ${campaign} (valid: all, scalability, aldsr, handover, alpha-gs)" >&2
        exit 2
    fi
    IFS='|' read -r sbatch_file campaign_tasks campaign_cpus export_args <<< "${CAMPAIGN_SPEC[${campaign}]}"
    if [[ -n "${export_args}" ]]; then
        job=$(submit --cpus-per-task="${campaign_cpus}" \
            --export="ALL,${export_args}" "${script_dir}/${sbatch_file}")
    else
        job=$(submit --cpus-per-task="${campaign_cpus}" "${script_dir}/${sbatch_file}")
    fi
    labels+=("${campaign}=${job} (${campaign_tasks} tasks x ${campaign_cpus} CPUs = $((campaign_tasks * campaign_cpus)))")
    tasks=$((tasks + campaign_tasks))
    cpus=$((cpus + campaign_tasks * campaign_cpus))
done

echo "Submitted ${#labels[@]} campaign(s) for ${start_time}:"
for label in "${labels[@]}"; do
    echo "  ${label}"
done
echo "  total=${tasks} tasks, ${cpus} CPUs; $((47 - cpus)) of 47 CPUs remain free"
if [[ -z "${LEOSIM_NODE:-}" ]]; then
    echo "Set LEOSIM_NODE=<hostname> to pin all arrays to one specific server."
fi
