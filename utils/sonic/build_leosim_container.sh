#!/bin/bash -l
set -euo pipefail

REPO_ROOT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
SIF="${2:-${REPO_ROOT}/leosim.sif}"
DEFINITION="${REPO_ROOT}/utils/sonic/leosim.def"

if command -v module >/dev/null 2>&1; then
  module purge
  module load "${APPTAINER_MODULE:-apptainer/1.3.4-gcc-11.5.0-ojp6nts}"
fi

if ! command -v apptainer >/dev/null 2>&1; then
  echo "Apptainer is unavailable; load the site module or set APPTAINER_MODULE." >&2
  exit 2
fi

# leosim.def copies '.', so the build context must be the repository root.
cd "$REPO_ROOT"
apptainer build --fakeroot "$SIF" "$DEFINITION"
apptainer test "$SIF"

echo "Built and tested: $SIF"
