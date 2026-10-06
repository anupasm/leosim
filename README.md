# LeoSim

LeoSim is an ns-3 module for packet-level simulation of low-Earth-orbit
satellite networks. The module contains reusable satellite mobility, channel,
beam, routing, handover, traffic, weather, and statistics components.

## Repository scope

This repository is the reusable library. Research campaigns, cluster runners,
analysis scripts, generated datasets, and simulation results are deliberately
kept outside the module. In the development workspace they live under:

- `experiments/leosim/`
- `datasets/leosim/`
- `results/leosim/`

The calling simulation supplies its input data directory; generated datasets
are not a library dependency.

## Install and build

Place this repository at `contrib/leosim` in an ns-3 checkout, then run from the
ns-3 root:

```bash
./ns3 configure --enable-tests
./ns3 build leosim
./ns3 run "test-runner --suite=leosim" --no-build
./ns3 run "test-runner --suite=leosim-statistics" --no-build
```

The current development workspace uses ns-3.45. Compatibility with additional
ns-3 releases should be recorded as they are exercised in CI.

## Using the module

Include the aggregate header when the full API is required:

```cpp
#include "ns3/leosim-module.h"
```

The complete example driver is available in
[`examples/leosim-experiments.cc`](examples/leosim-experiments.cc). Research
campaign scripts and generated results remain outside the library repository.

The optional external routing helper uses the companion routing engine in
`utils/rengine`. Build it with:

```bash
make -C contrib/leosim/utils/rengine
```

More detailed architecture and API documentation is available in
[`doc/README.md`](doc/README.md).

## Licence

LeoSim is distributed under the GNU General Public License version 2. See
[`LICENSE`](LICENSE).
