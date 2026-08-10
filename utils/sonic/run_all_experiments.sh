#!/bin/bash -l
set -euo pipefail

# Build and submit the LeoSim routing and 5 Mbps handover Slurm campaigns.
# Previous outputs are archived only when --rerun is requested.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

is_allowed_storage_path() {
  local path="${1}"
  [[ -z "${path}" ]] && return 1
  if [[ "${path}" == "${HOME}" || "${path}" == "${HOME}/"* ]]; then
    return 0
  fi
  if [[ "${path}" == "/scratch/adesilva" || "${path}" == "/scratch/adesilva/"* ]]; then
    return 0
  fi
  return 1
}

find_repo_root() {
  local current="${1}"
  while [[ "${current}" != "/" ]]; do
    if [[ -d "${current}/contrib/leosim" ]]; then
      printf '%s\n' "${current}"
      return 0
    fi
    if [[ -d "${current}/ns3/contrib/leosim" ]]; then
      printf '%s\n' "${current}/ns3"
      return 0
    fi
    current="$(dirname "${current}")"
  done
  printf '%s\n' "$(cd "${1}/../.." 2>/dev/null || pwd)"
}
DETECTED_REPO_ROOT="$(find_repo_root "${SCRIPT_DIR}")"

# LEOSIM_REPO_ROOT historically referred to either the checkout containing
# ns3/ or to ns3 itself. Accept both layouts and avoid producing ns3/ns3.
configured_root="${LEOSIM_REPO_ROOT:-${DETECTED_REPO_ROOT}}"
if [[ -d "${configured_root}/contrib/leosim" ]]; then
  REPO_ROOT="${configured_root}"
elif [[ -d "${configured_root}/ns3/contrib/leosim" ]]; then
  REPO_ROOT="${configured_root}/ns3"
else
  REPO_ROOT="${configured_root}"
fi
candidate_ns3_root="${LEOSIM_NS3_ROOT:-}"
if [[ -n "${candidate_ns3_root}" ]]; then
  if [[ -x "${candidate_ns3_root}/ns3" && -d "${candidate_ns3_root}/contrib/leosim" ]]; then
    NS3_ROOT="${candidate_ns3_root}"
  elif [[ -d "${candidate_ns3_root}/ns3" && -x "${candidate_ns3_root}/ns3/ns3" && -d "${candidate_ns3_root}/ns3/contrib/leosim" ]]; then
    NS3_ROOT="${candidate_ns3_root}/ns3"
  else
    echo "Ignoring invalid LEOSIM_NS3_ROOT=${candidate_ns3_root}; resolving from the repo instead." >&2
    NS3_ROOT="${REPO_ROOT}"
  fi
else
  NS3_ROOT="${REPO_ROOT}"
fi
ROUTING_SBATCH="${SCRIPT_DIR}/run_leosim_routing.sbatch"
ALDSR_SBATCH="${SCRIPT_DIR}/run_aldsr_weight_search.sbatch"
HANDOVER_SBATCH="${SCRIPT_DIR}/sonic-handover-paper.sbatch"
BUILD_HOST="${SCRIPT_DIR}/build_leosim.sh"
BUILD_CONTAINER="${SCRIPT_DIR}/build_leosim_container.sh"
if [[ -z "${SIF:-}" ]]; then
  if [[ -d "/scratch/adesilva" && -w "/scratch/adesilva" ]]; then
    SIF="/scratch/adesilva/leosim.sif"
  elif [[ -d "${HOME}" && -w "${HOME}" ]]; then
    SIF="${HOME}/leosim.sif"
  else
    SIF="${REPO_ROOT}/leosim.sif"
  fi
fi
if [[ "${SIF}" == "/" || "${SIF}" == "/leosim.sif" ]]; then
  SIF="/scratch/adesilva/leosim.sif"
fi

for opt_path in "${SIF}" "${RESULTS_ROOT:-}" "${ALDSR_RESULTS_ROOT:-}" \
  "${LEOSIM_RESULTS_ROOT:-}" "${LEOSIM_ARCHIVE_ROOT:-}"; do
  if [[ -n "${opt_path}" ]] && ! is_allowed_storage_path "${opt_path}"; then
    echo "Path is outside the allowed storage roots: ${opt_path}" >&2
    echo "Use a path under ${HOME} or /scratch/adesilva." >&2
    exit 2
  fi
done

action="all"
rebuild_container=0
skip_container=0

usage() {
  cat <<'EOF'
Usage: run_all_experiments.sh [ACTION] [OPTIONS]

Actions:
  build       Build the routing container and any available host helper.
  routing     Submit only the routing array campaign.
  aldsr       Build and submit only the ALDSR weight-search campaign.
  handover    Submit only the handover array campaign.
  submit      Submit all three campaigns using existing builds.
  all         Build and submit routing, ALDSR, and handover campaigns (default).
  rerun       Archive previous outputs, build, and submit all three campaigns.
  clean       Archive previous outputs without building or submitting.

Options:
  --rebuild-container  Rebuild leosim.sif even when it already exists.
  --skip-container     Build only the host executable; useful for handover-only work.
  -h, --help           Show this help.

Environment:
  LEOSIM_REPO_ROOT     LeoSim/ns-3 checkout root (for example, $HOME/leosim).
  LEOSIM_NS3_ROOT      Optional separate ns-3 directory for a nested checkout.
  SIF                  Routing container image path.
  RESULTS_ROOT         Optional routing-results root passed through to sbatch.
  ALDSR_RESULTS_ROOT   Optional ALDSR-results root passed through to sbatch.
  LEOSIM_RESULTS_ROOT  Optional handover-results root passed through to sbatch.
  LEOSIM_ARCHIVE_ROOT  Archive destination root used by clean/rerun.
  LEOSIM_HO_CPUS       CPUs per handover task (default: 4).
  LEOSIM_HO_BUFFER_PACKETS
                       Per-ground-node handover buffer size (default: 1024).
EOF
}

if (($# > 0)) && [[ "$1" != -* ]]; then
  action="$1"
  shift
fi
while (($# > 0)); do
  case "$1" in
    --rebuild-container) rebuild_container=1 ;;
    --skip-container) skip_container=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

case "$action" in
  build|routing|aldsr|handover|submit|all|rerun|clean) ;;
  *) echo "Unknown action: $action" >&2; usage >&2; exit 2 ;;
esac

for required in "$ROUTING_SBATCH" "$ALDSR_SBATCH" "$HANDOVER_SBATCH"; do
  if [[ ! -e "$required" ]]; then
    echo "Required file is missing: $required" >&2
    exit 2
  fi
done
# Host ns-3 is intentionally not required. LeoSim runs inside the Apptainer
# image, which embeds the ns-3 checkout and the LeoSim module.

archive_outputs() {
  local stamp archive_root routing_results aldsr_results handover_results
  stamp="$(date -u +%Y%m%dT%H%M%SZ)"
  local scratch_root="/scratch/adesilva"
  if [[ ! -d "$scratch_root" ]]; then
    mkdir -p "$scratch_root"
  fi
  archive_root="${LEOSIM_ARCHIVE_ROOT:-${scratch_root}/rerun-archive}/${stamp}"
  routing_results="${RESULTS_ROOT:-${scratch_root}/results/leosim-routing}"
  aldsr_results="${ALDSR_RESULTS_ROOT:-${scratch_root}/results/leosim-aldsr}"
  handover_results="${LEOSIM_RESULTS_ROOT:-${scratch_root}/handover/results}"
  mkdir -p "$archive_root"

  if [[ -d "$routing_results" ]]; then
    mkdir -p "$archive_root/routing"
    mv "$routing_results" "$archive_root/routing/results"
    echo "Archived routing results to $archive_root/routing/results"
  fi
  if [[ -d "$handover_results" ]]; then
    mkdir -p "$archive_root/handover"
    mv "$handover_results" "$archive_root/handover/results"
    echo "Archived handover results to $archive_root/handover/results"
  fi
  if [[ -d "$aldsr_results" ]]; then
    mkdir -p "$archive_root/aldsr"
    mv "$aldsr_results" "$archive_root/aldsr/results"
    echo "Archived ALDSR results to $archive_root/aldsr/results"
  fi

  mkdir -p "$archive_root/logs"
  find "$REPO_ROOT/logs" -maxdepth 1 -type f \
    \( -name 'leosim-routing-*.out' -o -name 'leosim-routing-*.err' \) \
    -exec mv -t "$archive_root/logs" -- {} + 2>/dev/null || true
  find "$REPO_ROOT/handover_logs" -maxdepth 1 -type f \
    \( -name 'leosim-ho-*.out' -o -name 'leosim-ho-*.err' \) \
    -exec mv -t "$archive_root/logs" -- {} + 2>/dev/null || true
  echo "Rerun archive: $archive_root"
}

build_experiments() {
  mkdir -p "$(dirname "$SIF")"
  if ((skip_container)); then
    echo "Skipping routing container build as requested."
  elif ((rebuild_container)) || [[ ! -s "$SIF" ]]; then
    "$BUILD_CONTAINER" "$REPO_ROOT" "$SIF"
  else
    echo "Using existing routing container: $SIF"
    echo "Pass --rebuild-container after source changes that must enter the image."
  fi

  # The handover batch prefers the host executable when this checkout has an
  # ns-3 launcher. Build it once here; array jobs use --no-build concurrently.
  "$BUILD_HOST" "$REPO_ROOT"
}

submit_routing_experiments() {
  local results_root
  if ! command -v sbatch >/dev/null 2>&1; then
    echo "sbatch is unavailable; run this script on a Slurm login node." >&2
    exit 2
  fi
  if [[ ! -s "$SIF" ]]; then
    echo "Routing container is missing: $SIF" >&2
    echo "Run the build action without --skip-container first." >&2
    exit 2
  fi

  mkdir -p "/scratch/adesilva/logs" "$REPO_ROOT/logs"
  cd "$REPO_ROOT"
  results_root="${RESULTS_ROOT:-/scratch/adesilva/results/leosim-routing}"
  mkdir -p "$results_root"
  routing_job="$(sbatch --parsable \
    --export="ALL,SIF=${SIF},RESULTS_ROOT=${results_root}" "$ROUTING_SBATCH")"
  echo "Submitted routing array:  $routing_job"
  echo "Monitor with: squeue -j ${routing_job%%;*}"
}

submit_handover_experiments() {
  local results_root ho_cpus ho_buffer_packets
  if ! command -v sbatch >/dev/null 2>&1; then
    echo "sbatch is unavailable; run this script on a Slurm login node." >&2
    exit 2
  fi
  if [[ ! -x "${NS3_ROOT}/ns3" && ! -s "$SIF" ]]; then
    echo "Neither a host ns-3 launcher nor a container image is available." >&2
    echo "Expected ${NS3_ROOT}/ns3 or ${SIF}." >&2
    exit 2
  fi

  mkdir -p "/scratch/adesilva/logs" "$REPO_ROOT/handover_logs"
  cd "$REPO_ROOT"
  results_root="${LEOSIM_RESULTS_ROOT:-/scratch/adesilva/handover/results}"
  ho_cpus="${LEOSIM_HO_CPUS:-4}"
  ho_buffer_packets="${LEOSIM_HO_BUFFER_PACKETS:-1024}"
  mkdir -p "$results_root"
  handover_job="$(sbatch --parsable --cpus-per-task="${ho_cpus}" \
    --export="ALL,SIF=${SIF},LEOSIM_REPO_ROOT=${REPO_ROOT},LEOSIM_NS3_ROOT=${NS3_ROOT},LEOSIM_RESULTS_ROOT=${results_root},LEOSIM_HO_BUFFER_PACKETS=${ho_buffer_packets}" \
    "$HANDOVER_SBATCH")"
  echo "Submitted handover array: $handover_job"
  echo "Handover results: ${results_root}/${handover_job%%;*}"
  echo "Monitor with: squeue -j ${handover_job%%;*}"
}

submit_aldsr_experiments() {
  if ! command -v sbatch >/dev/null 2>&1; then
    echo "sbatch is unavailable; run this script on a Slurm login node." >&2
    exit 2
  fi
  if [[ ! -s "$SIF" ]]; then
    echo "Routing container is missing: $SIF" >&2
    exit 2
  fi
  local results_root="${ALDSR_RESULTS_ROOT:-/scratch/adesilva/results/leosim-aldsr}"
  mkdir -p "/scratch/adesilva/logs" "$REPO_ROOT/logs" "$results_root"
  cd "$REPO_ROOT"
  aldsr_job="$(sbatch --parsable \
    --export="ALL,SIF=${SIF},ALDSR_RESULTS_ROOT=${results_root}" "$ALDSR_SBATCH")"
  echo "Submitted ALDSR array: $aldsr_job"
  echo "Monitor with: squeue -j ${aldsr_job%%;*}"
}

submit_experiments() {
  submit_routing_experiments
  submit_aldsr_experiments
  submit_handover_experiments
}

case "$action" in
  clean) archive_outputs ;;
  rerun) archive_outputs; build_experiments; submit_experiments ;;
  build) build_experiments ;;
  routing) build_experiments; submit_routing_experiments ;;
  aldsr) build_experiments; submit_aldsr_experiments ;;
  handover) build_experiments; submit_handover_experiments ;;
  submit) submit_experiments ;;
  all) build_experiments; submit_experiments ;;
esac
