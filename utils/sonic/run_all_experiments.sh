#!/bin/bash -l
set -euo pipefail

# Build and submit the LeoSim routing and handover Slurm campaigns.
# Previous outputs are archived only when --rerun is requested.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DETECTED_NS3_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

# LEOSIM_REPO_ROOT historically referred to either the checkout containing
# ns3/ or to ns3 itself. Accept both layouts and avoid producing ns3/ns3.
configured_root="${LEOSIM_REPO_ROOT:-${DETECTED_NS3_ROOT}}"
if [[ -x "${configured_root}/ns3" && -d "${configured_root}/contrib/leosim" ]]; then
  NS3_ROOT="${LEOSIM_NS3_ROOT:-${configured_root}}"
  REPO_ROOT="${configured_root}"
elif [[ -x "${configured_root}/ns3/ns3" ]]; then
  REPO_ROOT="${configured_root}"
  NS3_ROOT="${LEOSIM_NS3_ROOT:-${REPO_ROOT}/ns3}"
else
  REPO_ROOT="${configured_root}"
  NS3_ROOT="${LEOSIM_NS3_ROOT:-${DETECTED_NS3_ROOT}}"
fi
ROUTING_SBATCH="${SCRIPT_DIR}/run_leosim_routing.sbatch"
HANDOVER_SBATCH="${SCRIPT_DIR}/sonic-handover-paper.sbatch"
BUILD_HOST="${SCRIPT_DIR}/build_leosim.sh"
BUILD_CONTAINER="${SCRIPT_DIR}/build_leosim_container.sh"
SIF="${SIF:-${REPO_ROOT}/leosim.sif}"

action="all"
rebuild_container=0
skip_container=0

usage() {
  cat <<'EOF'
Usage: run_all_experiments.sh [ACTION] [OPTIONS]

Actions:
  build       Build the host executable and routing container.
  submit      Submit both existing builds with sbatch.
  all         Build and submit both campaigns (default).
  rerun       Archive previous outputs, build, and submit both campaigns.
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
  LEOSIM_RESULTS_ROOT  Optional handover-results root passed through to sbatch.
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
  build|submit|all|rerun|clean) ;;
  *) echo "Unknown action: $action" >&2; usage >&2; exit 2 ;;
esac

for required in "$NS3_ROOT/ns3" "$ROUTING_SBATCH" "$HANDOVER_SBATCH"; do
  if [[ ! -e "$required" ]]; then
    echo "Required file is missing: $required" >&2
    exit 2
  fi
done

archive_outputs() {
  local stamp archive_root routing_results handover_results
  stamp="$(date -u +%Y%m%dT%H%M%SZ)"
  archive_root="${REPO_ROOT}/rerun-archive/${stamp}"
  routing_results="${RESULTS_ROOT:-${REPO_ROOT}/results/leosim-routing}"
  handover_results="${LEOSIM_RESULTS_ROOT:-${REPO_ROOT}/experiments/handover/results}"
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

  mkdir -p "$archive_root/logs"
  find "$REPO_ROOT/logs" -maxdepth 1 -type f \
    \( -name 'leosim-routing-*.out' -o -name 'leosim-routing-*.err' \) \
    -exec mv -t "$archive_root/logs" -- {} + 2>/dev/null || true
  find "$REPO_ROOT" -maxdepth 1 -type f \
    \( -name 'leosim-ho-*.out' -o -name 'leosim-ho-*.err' \) \
    -exec mv -t "$archive_root/logs" -- {} + 2>/dev/null || true
  echo "Rerun archive: $archive_root"
}

build_experiments() {
  "$BUILD_HOST" "$REPO_ROOT"
  if ((skip_container)); then
    echo "Skipping routing container build as requested."
  elif ((rebuild_container)) || [[ ! -s "$SIF" ]]; then
    "$BUILD_CONTAINER" "$REPO_ROOT" "$SIF"
  else
    echo "Using existing routing container: $SIF"
    echo "Pass --rebuild-container after source changes that must enter the image."
  fi
}

submit_experiments() {
  if ! command -v sbatch >/dev/null 2>&1; then
    echo "sbatch is unavailable; run this script on a Slurm login node." >&2
    exit 2
  fi
  if [[ ! -s "$SIF" ]]; then
    echo "Routing container is missing: $SIF" >&2
    echo "Run the build action without --skip-container first." >&2
    exit 2
  fi

  mkdir -p "$REPO_ROOT/logs"
  cd "$REPO_ROOT"
  routing_job="$(sbatch --parsable --export="ALL,SIF=${SIF}" "$ROUTING_SBATCH")"
  handover_job="$(sbatch --parsable \
    --export="ALL,LEOSIM_REPO_ROOT=${REPO_ROOT},LEOSIM_NS3_ROOT=${NS3_ROOT}" \
    "$HANDOVER_SBATCH")"
  echo "Submitted routing array:  $routing_job"
  echo "Submitted handover array: $handover_job"
  echo "Monitor with: squeue -j ${routing_job%%;*},${handover_job%%;*}"
}

case "$action" in
  clean) archive_outputs ;;
  rerun) archive_outputs; build_experiments; submit_experiments ;;
  build) build_experiments ;;
  submit) submit_experiments ;;
  all) build_experiments; submit_experiments ;;
esac
