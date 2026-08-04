# LeoSim routing and handover experiments

`leosim-experiments.cc` is the single LeoSim example executable. It combines the
shared topology, satellite mobility, access links, dynamic ISL routing, TCP
traffic, FlowMonitor, statistics, beam management, and BHO/CHO handover logic so
routing and handover treatments use the same scenario implementation.

Run all commands below from the `ns3` directory.

## Build

```bash
CCACHE_DISABLE=1 ./ns3 build leosim-experiments
```

List every available option:

```bash
./ns3 run "leosim-experiments --PrintHelp" --no-build
```

## Routing experiment

Select a routing metric with `--routingMetric`. Supported values are `hop`,
`distance`, `path-loss`, `snr`, `signal-strength`, `lifetime`, and `load`.

```bash
./ns3 run "leosim-experiments \
  --simTime=1200 --numSatellites=1000 \
  --enableDynamicRouting=1 --routingUpdateInterval=30 \
  --routingMetric=distance --maxIslNeighbors=4 \
  --tcpRate=1Mbps --outputPrefix=results/routing-distance"
```

The SONIC routing sweep is submitted from the repository root after building the
container image:

```bash
sbatch ns3/contrib/leosim/utils/sonic/run_leosim_routing.sbatch
```

## Handover experiment

The executable supports reactive BHO and predictive CHO. `--choPrep` and
`--choExec` are milliseconds; `--ttt`, `--t310`, and `--tteTrigger` are seconds.
Use a unique `--outputPrefix` for each run.

BHO baseline:

```bash
./ns3 run "leosim-experiments \
  --simTime=300 --numUes=20 --numServers=4 \
  --hoMode=BHO --maxCandidates=1 --enableHoBuffering=0 \
  --tcpRate=1Mbps --enableHandoverLogging=1 \
  --outputPrefix=results/handover-bho"
```

CHO with three candidates and buffering:

```bash
./ns3 run "leosim-experiments \
  --simTime=300 --numUes=20 --numServers=4 \
  --hoMode=CHO --maxCandidates=3 --enableHoBuffering=1 \
  --ttt=1 --t310=1 --a3Offset=3 --a4Threshold=-110 \
  --tteTrigger=30 --choPrep=100 --choExec=150 \
  --beamUpdateIntervalMs=100 --tcpRate=1Mbps \
  --enableHandoverLogging=1 --outputPrefix=results/handover-cho3"
```

Submit the full BHO/CHO SONIC factorial campaign from the repository root:

```bash
sbatch experiments/handover/sonic-handover-paper.sbatch
```

## Main outputs

With `--outputPrefix=DIR/result`, the experiment writes files including:

- `DIR/result-statistics.csv` and `DIR/result-statistics.json`
- `DIR/result-handovers.csv`
- `DIR/result-flowmon.xml`
- optional route, beam, visualization, and ISL-load traces when enabled

For reproducible replications, set the ns-3 seed and run number:

```bash
NS_GLOBAL_VALUE="RngSeed=20260803;RngRun=1" \
  ./ns3 run "leosim-experiments --simTime=300 --outputPrefix=results/run-1"
```
