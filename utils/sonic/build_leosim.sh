#!/bin/bash -l
set -euo pipefail

# Run this once on Sonic (preferably in an interactive Slurm allocation), not
# concurrently in every array task.
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
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${1:-$(find_repo_root "${SCRIPT_DIR}")}"
if [[ -x "${REPO_ROOT}/ns3" && -d "${REPO_ROOT}/contrib/leosim" ]]; then
  NS3_ROOT="${REPO_ROOT}"
elif [[ -d "${REPO_ROOT}/ns3" && -x "${REPO_ROOT}/ns3/ns3" ]]; then
  NS3_ROOT="${REPO_ROOT}/ns3"
else
  NS3_ROOT="${REPO_ROOT}"
fi
PYTHON_ENV="${LEOSIM_PYTHON_ENV:-${REPO_ROOT}/.venv-leosim-sonic}"

if [[ ! -d "${NS3_ROOT}" || ! -x "${NS3_ROOT}/ns3" ]]; then
  echo "No host ns-3 launcher found at ${NS3_ROOT}/ns3; skipping host build because ns-3 is embedded in the container."
  exit 0
fi

if command -v module >/dev/null 2>&1; then
  module purge
  # Sonic's available versions change; use site defaults when present.
  module load gcc 2>/dev/null || true
  module load cmake 2>/dev/null || true
fi

python3 -m venv "$PYTHON_ENV"
"${PYTHON_ENV}/bin/python" -m pip install --upgrade pip
"${PYTHON_ENV}/bin/python" -m pip install numpy skyfield
"${PYTHON_ENV}/bin/python" -c 'import numpy, skyfield'

cd "$NS3_ROOT"
./ns3 configure --enable-examples --disable-tests --build-profile=optimized
./ns3 build leosim-experiments

# Verify both the build and the command-line interface.
./ns3 run "leosim-experiments --PrintHelp" --no-build
