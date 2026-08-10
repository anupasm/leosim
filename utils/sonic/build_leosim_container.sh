#!/bin/bash -l
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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
    if [[ -f "${current}/CMakeLists.txt" && -d "${current}/model" && -d "${current}/helper" ]]; then
      printf '%s\n' "${current}"
      return 0
    fi
    current="$(dirname "${current}")"
  done
  printf '%s\n' "$(cd "${1}/../.." 2>/dev/null || pwd)"
}
REPO_ROOT="${1:-$(find_repo_root "${SCRIPT_DIR}")}"
if [[ -n "${2:-}" ]]; then
  SIF="${2}"
elif [[ -d "/scratch/adesilva" && -w "/scratch/adesilva" ]]; then
  SIF="/scratch/adesilva/leosim.sif"
else
  echo "/scratch/adesilva is unavailable or not writable." >&2
  exit 2
fi
if [[ "${SIF}" == "/" || "${SIF}" == "/leosim.sif" ]]; then
  SIF="/scratch/adesilva/leosim.sif"
fi
case "${SIF}" in
  /scratch/adesilva/*) ;;
  *) echo "Container output must be under /scratch/adesilva: ${SIF}" >&2; exit 2 ;;
esac
export TMPDIR="/scratch/adesilva/tmp/container-build-${SLURM_JOB_ID:-local}-$$"
export APPTAINER_CACHEDIR="/scratch/adesilva/cache/apptainer"
export APPTAINER_TMPDIR="${TMPDIR}/apptainer"
export XDG_CACHE_HOME="/scratch/adesilva/cache"
mkdir -p "$TMPDIR" "$APPTAINER_CACHEDIR" "$APPTAINER_TMPDIR" "$XDG_CACHE_HOME"
DEFINITION="${SCRIPT_DIR}/leosim.def"
if [[ -d "${REPO_ROOT}/contrib/leosim" ]]; then
  LEO_MODULE_ROOT="${REPO_ROOT}/contrib/leosim"
elif [[ -d "${REPO_ROOT}/ns3/contrib/leosim" ]]; then
  LEO_MODULE_ROOT="${REPO_ROOT}/ns3/contrib/leosim"
elif [[ -d "${REPO_ROOT}/model" && -d "${REPO_ROOT}/helper" ]]; then
  LEO_MODULE_ROOT="${REPO_ROOT}"
else
  LEO_MODULE_ROOT="${REPO_ROOT}"
fi
RENGINE_SOURCE="${LEO_MODULE_ROOT}/utils/rengine/leosim-rengine.cc"
RENGINE_MAKEFILE="${LEO_MODULE_ROOT}/utils/rengine/Makefile"

if command -v module >/dev/null 2>&1; then
  module purge
  module load "${APPTAINER_MODULE:-apptainer/1.3.4-gcc-11.5.0-ojp6nts}"
fi

if ! command -v apptainer >/dev/null 2>&1; then
  echo "Apptainer is unavailable; load the site module or set APPTAINER_MODULE." >&2
  exit 2
fi
if [[ ! -f "$DEFINITION" || ! -f "$RENGINE_SOURCE" || ! -f "$RENGINE_MAKEFILE" ]]; then
  echo "Invalid LeoSim build context: ${REPO_ROOT}" >&2
  echo "Expected ${DEFINITION}, ${RENGINE_SOURCE}, and ${RENGINE_MAKEFILE}." >&2
  exit 2
fi
mkdir -p "$(dirname "$SIF")"
# leosim.def copies '.', so the build context must be the repository root.
cd "$REPO_ROOT"
apptainer build --fakeroot "$SIF" "$DEFINITION"
apptainer test "$SIF"

# Verify the exact runtime paths and switches used by run_leosim_routing.sbatch.
apptainer exec --cleanenv --pwd /opt/leosim/ns3 "$SIF" \
  test -x contrib/leosim/utils/rengine/leosim-rengine
apptainer exec --cleanenv --pwd /opt/leosim/ns3 "$SIF" \
  contrib/leosim/utils/rengine/leosim-rengine --self-test
scenario_help="$(apptainer exec --cleanenv --pwd /opt/leosim/ns3 "$SIF" \
  ./ns3 run "leosim-experiments --PrintHelp" --no-build)"
grep -q -- "--useRouteTreeCache" <<<"$scenario_help"
grep -q -- "--routeTreeEngine" <<<"$scenario_help"
grep -q -- "--routeTreeWorkers" <<<"$scenario_help"
grep -q -- "--routeTreeWorkDir" <<<"$scenario_help"
grep -q -- "--routeTreeMaxEntries" <<<"$scenario_help"
grep -q -- "--combinedLoadWeight" <<<"$scenario_help"
grep -q -- "--combinedMinLifetime" <<<"$scenario_help"
grep -q -- "--rngSeed" <<<"$scenario_help"
grep -q -- "--rngRun" <<<"$scenario_help"

echo "Built and tested with external destination-tree and ALDSR routing: $SIF"
