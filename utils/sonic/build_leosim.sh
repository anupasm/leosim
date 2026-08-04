#!/bin/bash -l
set -euo pipefail

# Run this once on Sonic (preferably in an interactive Slurm allocation), not
# concurrently in every array task.
REPO_ROOT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)}"
NS3_ROOT="${REPO_ROOT}/ns3"
PYTHON_ENV="${LEOSIM_PYTHON_ENV:-${REPO_ROOT}/.venv-leosim-sonic}"

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
