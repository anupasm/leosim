# Running the LeoSim routing sweep on Sonic

This directory runs `leosim-param-scenario` as a 15-task Slurm array:

- metrics: hop, distance, path loss, SNR, signal strength;
- satellites: all 11,514 catalog satellites;
- duration: 1800 seconds (30 minutes);
- dynamic-routing intervals: 5, 10, and 30 seconds.

Before simulation, the array combines `data/tles/alpha.csv`, `beta.csv`, and
`gamma.csv` using `tle_to_positions.py`. A shared file lock ensures the
30-minute, 5-second-resolution position CSV is generated only once per array
job and reused by every routing run.

Every task writes FlowMonitor XML, periodic/summary statistics, selected-route
history, resource usage, and metadata. By default it loads all UEs and ground
stations and creates every UE-to-GSS TCP flow at 1 Mbps per flow.

## Build once

Run the setup helper before submitting. It creates `.venv-leosim-sonic`,
installs NumPy and Skyfield for TLE preprocessing, and builds the scenario.
Each array task then uses `--no-build`:

```bash
cd ~/LeoSim
srun --pty --nodes=1 --ntasks=1 --cpus-per-task=4 --time=01:00:00 bash -l
ns3/contrib/leosim/utils/sonic/build_leosim.sh
exit
```

Add the account/partition arguments assigned to you by Sonic where required.

## Submit

Slurm opens output files before the job script starts, so create `logs` first:

```bash
cd ~/LeoSim
mkdir -p logs
sbatch ns3/contrib/leosim/utils/sonic/run_leosim_routing.sbatch
```

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
`TLE_WORKERS` when needed. `TLE_PYTHON` can select another Python environment
that already contains NumPy and Skyfield.

Monitor and inspect jobs with:

```bash
squeue -u "$USER"
sacct -j JOB_ID --format=JobID,State,Elapsed,MaxRSS,TotalCPU,ExitCode
```
