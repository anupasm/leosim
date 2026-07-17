# Running the LeoSim routing sweep on Sonic

This directory runs `leosim-param-scenario` as a 15-task Slurm array:

- metrics: hop, distance, path loss, SNR, signal strength;
- satellites: all 11,514 catalog satellites;
- duration: 1800 seconds (30 minutes);
- dynamic-routing intervals: 5, 10, and 30 seconds.

The compute job runs from `leosim.sif`; ns-3, LeoSim, the compiled scenario,
Python, NumPy, Skyfield, the converter, and TLE inputs are embedded in that
image. The compute node therefore does not need a separate ns-3/LeoSim checkout
or Python environment. Before simulation, the array combines `alpha.csv`,
`beta.csv`, and `gamma.csv` using the converter in the container. A shared file lock ensures the
30-minute, 5-second-resolution position CSV is generated only once per array
job and reused by every routing run.

Every task writes FlowMonitor XML, periodic/summary statistics, selected-route
history, resource usage, and metadata. By default it loads all UEs and ground
stations and creates every UE-to-GSS TCP flow at 1 Mbps per flow.

## Build the container once

Build the image on a login/build node where Apptainer fakeroot and network
access are available. The definition copies the current repository into the
image and builds `leosim-param-scenario` there:

```bash
cd ~/LeoSim
ns3/contrib/leosim/utils/sonic/build_leosim_container.sh
```

This produces `~/LeoSim/leosim.sif`. Rebuild it after changing LeoSim or its
ns-3 scenario. If Sonic uses another module name, set it for both build and run:

```bash
APPTAINER_MODULE=apptainer/OTHER_VERSION \
  ns3/contrib/leosim/utils/sonic/build_leosim_container.sh
```

## Submit

Slurm opens output files before the job script starts, so create `logs` first:

```bash
cd ~/LeoSim
mkdir -p logs
sbatch ns3/contrib/leosim/utils/sonic/run_leosim_routing.sbatch
```

The job binds only `RESULTS_ROOT` at `/results`; all simulation software and
input data come from the read-only image. Select an image in another location
with `SIF=/path/to/leosim.sif`.

Use assigned scratch storage for the 15-run output, including the preprocessed
position CSV:

```bash
mkdir -p /YOUR/SCRATCH/leosim-routing
sbatch --export=ALL,RESULTS_ROOT=/YOUR/SCRATCH/leosim-routing \
  ns3/contrib/leosim/utils/sonic/run_leosim_routing.sbatch
```

Useful optional overrides include:

```bash
sbatch --export=ALL,NUM_UES=4,NUM_SERVERS=4,TCP_RATE=500Kbps \
  ns3/contrib/leosim/utils/sonic/run_leosim_routing.sbatch
```

`NUM_UES=0` and `NUM_SERVERS=0` mean all loaded endpoints. The array throttle
is `%10`; reduce it if `sacct` shows excessive memory pressure. Override the
position resolution with `TLE_TIMESTEP` or converter parallelism with
`TLE_WORKERS` when needed.

Monitor and inspect jobs with:

```bash
squeue -u "$USER"
sacct -j JOB_ID --format=JobID,State,Elapsed,MaxRSS,TotalCPU,ExitCode
```
