#!/bin/bash -l
set -euo pipefail

REPO_ROOT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
SIF="${2:-${REPO_ROOT}/leosim.sif}"
DEFINITION="${REPO_ROOT}/utils/sonic/leosim.def"
RENGINE_SOURCE="${REPO_ROOT}/utils/rengine/leosim-rengine.cc"
RENGINE_MAKEFILE="${REPO_ROOT}/utils/rengine/Makefile"

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

echo "Built and tested with external destination-tree routing: $SIF"
