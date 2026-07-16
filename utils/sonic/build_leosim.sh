#!/bin/bash -l
set -euo pipefail

# Run this once on Sonic (preferably in an interactive Slurm allocation), not
# concurrently in every array task.
REPO_ROOT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
NS3_ROOT="${REPO_ROOT}/ns3"

if command -v module >/dev/null 2>&1; then
  module purge
  # Sonic's available versions change; use site defaults when present.
  module load gcc 2>/dev/null || true
  module load cmake 2>/dev/null || true
fi

cd "$NS3_ROOT"
./ns3 configure --enable-examples --disable-tests --build-profile=optimized
./ns3 build leosim-routing-research-example

# Verify both the build and the command-line interface.
./ns3 run "leosim-routing-research-example --listMetrics=1" --no-build
