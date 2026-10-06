# LeoSim example

`leosim-experiments.cc` demonstrates how to compose LeoSim mobility, channel,
beam, routing, handover, traffic, and statistics components in an ns-3
simulation.

Build and inspect its options from the ns-3 root:

```bash
./ns3 configure --enable-examples
./ns3 build leosim-experiments
./ns3 run "leosim-experiments --PrintHelp" --no-build
```

Research designs, Slurm runners, analysis scripts, and generated results are
kept outside the reusable module under `experiments/leosim/` and
`results/leosim/` in the development workspace.
