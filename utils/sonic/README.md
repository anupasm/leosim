# Running the RP1 routing pilot on Sonic

This directory runs the currently implemented
`leosim-routing-research-example` as a Slurm job array: one independent routing
metric per array task. It does **not** yet run the full experiment described in
`ns3/contrib/leosim/doc/rp1_routing.txt`.

## 1. Copy the repository to Sonic

Connect through the UCD network/VPN and log in using the hostname and account
details supplied with your Sonic account. Transfer the checkout with `rsync`
from your own computer (replace both placeholders):

```bash
rsync -az --exclude ns3/build --exclude ns3/cmake-cache \
  /path/to/LeoSim/ UCD_USER@SONIC_LOGIN:~/LeoSim/
```

For production output, use your assigned Sonic scratch/project path rather
than home: home has a 50 GB quota and Sonic scratch is computational,
non-archival storage. The exact scratch path is account-specific, so do not
hard-code one copied from another user.

## 2. Build once

Never compile separately in every array element. First request an interactive
CPU allocation using the account/partition values assigned to you, then build:

```bash
cd ~/LeoSim
srun --pty --nodes=1 --ntasks=1 --cpus-per-task=4 --time=01:00:00 bash -l
./scripts/sonic/build_leosim.sh
exit
```

If Sonic requires an account or partition, add `--account=YOUR_ACCOUNT` and/or
`--partition=YOUR_PARTITION` to `srun` and `sbatch`. Do not guess these values;
check them with `sacctmgr show assoc user=$USER` and `sinfo` or use the values
from Research IT.

## 3. Submit the nine metrics in parallel

Slurm creates the log directory before the script starts, so create it before
submission:

```bash
cd ~/LeoSim
mkdir -p logs
sbatch scripts/sonic/run_leosim_routing.sbatch
```

To put results on assigned scratch storage:

```bash
mkdir -p /YOUR/SCRATCH/LeoSim-results
sbatch --export=ALL,RESULTS_ROOT=/YOUR/SCRATCH/LeoSim-results/rp1-pilot \
  scripts/sonic/run_leosim_routing.sbatch
```

Useful controls:

```bash
squeue -u "$USER"
sacct -j JOB_ID --format=JobID,State,Elapsed,MaxRSS,TotalCPU,ExitCode
scancel JOB_ID
```

Results are grouped by metric under `results/rp1-routing/JOB_ID/` by default.
The `%9` in `#SBATCH --array=0-8%9` caps simultaneous tasks at nine. For a
larger future matrix, keep a concurrency cap (for example `%20`) rather than
launching every simulation at once.

## Research-plan gap before the full campaign

The present example is a deterministic, static eight-node routing calculation.
It has no random seed, 24-hour duration, traffic class/load, failure rate,
topology selection, or full-scale constellation arguments. Repeating it for
multiple seeds would therefore produce duplicate rows. Before launching the
full study, extend one scenario executable to accept those factors and write
network, routing, reliability, and resource KPIs. Then map a manifest row (not
a fragile hard-coded arithmetic product) to each Slurm array task and use
common random seeds across routing metrics for paired comparisons.
