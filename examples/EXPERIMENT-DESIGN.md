# Paper experiment: service continuity during LEO handover

## One-sentence contribution

This experiment uses LeoSim to quantify when predictive, multi-candidate
Conditional Handover (CHO) improves application service continuity over reactive
Best-effort Handover (BHO), and whether packet buffering preserves that advantage
as traffic load increases.

The study showcases LeoSim's orbital mobility, phased-array spot beams, dynamic
ground-to-satellite association, 3GPP-style BHO/CHO state machines, TOPSIS target
ranking, handover-triggered route repair, TCP traffic, FlowMonitor, and event-level
handover/channel traces in one controlled packet-level experiment.

## Research questions and hypotheses

- **RQ1:** Does CHO reduce application interruption and packet loss relative to BHO?
- **RQ2:** Does preparing multiple CHO candidates improve robustness relative to a
  single prepared candidate?
- **RQ3:** How do offered load and handover buffering modify these effects?
- **RQ4:** What control/performance cost is paid in preparations, candidate state,
  and handover frequency?

Pre-register these directional hypotheses:

- **H1:** CHO-3 has shorter post-handover interruption and lower event-window loss
  than BHO.
- **H2:** CHO-3 has a higher success ratio than CHO-1, particularly at high load.
- **H3:** buffering reduces loss during CHO execution, but its benefit decreases
  when queues are persistently occupied at high load.
- **H4:** CHO-3 improves the lower tail of post-handover SINR and reduces radio-link
  failures, at the cost of more candidate preparation state.

The null for every hypothesis is no paired difference between policies after
controlling for load, buffering, and random run.

## Experimental scenario

Use `leosim-experiments` with the bundled LeoSim satellite trace and
ground-node data. It is an independent experiment entry point and does not include
or invoke the routing experiment. Both executables use common, experiment-neutral
scenario infrastructure so mobility, channel, routing, traffic, and KPI behavior
remain consistent. The handover executable sets the paper defaults documented
below and exposes every handover treatment on the command line.
Select a constellation subset that provides continuous overlapping visibility for
all studied UEs. Place 20 stationary UEs in four geographically distinct clusters
(urban mid-latitude, rural mid-latitude, equatorial, and high-latitude), with five
UEs per cluster, and one server/gateway per cluster. Each UE runs one long-lived TCP
flow to its cluster's server.

Run for **1,800 simulated seconds**. Start applications at 60 s and exclude the
first 120 s from confirmatory analysis. This gives the transport connections time
to leave startup while retaining many naturally occurring satellite/beam
transitions. If the pilot produces fewer than 30 inter-satellite handovers per run,
increase the duration for every condition before beginning the main campaign; do
not selectively lengthen treatments.

Use dynamic degree-4 ISLs and handover-triggered route refresh. Keep trajectories,
ground positions, traffic endpoints, channel settings, routing settings, and beam
geometry byte-identical within each random-run block.

## Treatments

### Primary handover policy factor

| Label | LeoSim configuration | Interpretation |
|---|---|---|
| BHO | `--hoMode=BHO` | Reactive baseline |
| CHO-1 | `--hoMode=CHO --maxCandidates=1` | Predictive CHO without candidate diversity |
| CHO-3 | `--hoMode=CHO --maxCandidates=3` | Predictive, TOPSIS-ranked multi-candidate CHO |

### Factorial modifiers

| Factor | Levels | Purpose |
|---|---|---|
| Offered rate per UE | 0.25, 1, 4 Mbit/s | Low, medium, and congested regimes |
| HO buffering | off, on | Isolates continuity gained from buffering |
| Random run | 1--10 | Paired replication block |

The primary design contains **180 runs**: 3 policies x 3 loads x 2 buffering states
x 10 random runs. If compute is constrained, five complete paired random-run blocks
(90 runs) are the pre-registered minimum. Add replication only in complete blocks
and never stop because a result becomes significant.

### Fixed controls

| Parameter | Value |
|---|---|
| Simulation duration | 1,800 s |
| Analysis warm-up | first 120 s excluded |
| UEs / servers | 20 / 4 |
| ISLs | enabled, maximum degree 4 |
| Routing | dynamic plus reactive refresh, 10 s periodic interval |
| Transport | TCP, 512-byte application packets |
| Beam manager update | 100 ms |
| Beam geometry update | 1 s |
| TTT / T310 | 1 s / 1 s |
| A3 / A4 | 3 dB / -110 dBm |
| TTE trigger | 30 s |
| CHO preparation / execution | 100 ms / 150 ms |
| TOPSIS weights | repository defaults; frozen for all runs |
| Weather | disabled in the primary experiment |

Do not tune thresholds separately for BHO and CHO in the primary study. A policy
comparison is only interpretable if both observe the same geometry and radio
conditions.

## Outcomes and operational definitions

The **simulation run** is the experimental unit. Packets and individual handovers
are repeated observations within a run, not independent replicates.

### Confirmatory primary endpoint

**Handover interruption time** is the longest gap between consecutive packets
successfully received by a flow in the window from 2 s before to 5 s after an
inter-satellite handover. Subtract that flow's median inter-arrival time in the
preceding 30 s, clamp at zero, and aggregate first to the median per run. This
directly measures user-visible service continuity.

### Other primary outcomes

- event-window packet loss ratio, from -2 s to +5 s around each inter-satellite HO;
- inter-satellite handover success ratio;
- application PDR over the post-warm-up run;
- goodput over the post-warm-up run.

### Secondary and diagnostic outcomes

- handover latency (median and P95), failures, and failure reasons;
- packets buffered and packets dropped per handover;
- SINR change (`sinr_after - sinr_before`) and P10 post-HO SINR;
- RLF-triggered handovers and ping-pongs (A-B-A within 30 s);
- time without a serving association and route-repair delay;
- CHO preparations, candidates prepared, expired/unused candidates, and TOPSIS
  score of the selected target;
- end-to-end delay, jitter, throughput, route changes, simulation wall time, and
  peak resident memory.

Analyze inter-satellite events separately from intra-satellite beam changes. The
two mechanisms have different delays and combining them would obscure the policy
effect.

## Execution and provenance protocol

1. Run a 300 s pilot for every policy at medium load with buffering on. Confirm
   overlapping visibility, non-zero traffic, valid SINR values, and at least one
   inter-satellite event. The existing `pilot/baseline-statistics.csv` is useful for
   plumbing checks but is not inferential evidence.
2. Freeze and checksum the satellite trace, ground-node files, and flow manifest.
3. For each `(load, buffering, random-run)` block, randomize the order of BHO,
   CHO-1, and CHO-3 and execute them on the same hardware class.
4. Set `RngSeed` once for the paper and vary `RngRun` from 1 to 10. Record both.
5. Save stdout/stderr, exit status, wall time, peak RSS, commit ID, build flags,
   input hashes, and the complete command line.
6. Retain statistics JSON/CSV, FlowMonitor XML, packet trace, unified link-state
   trace, beam associations, handover events, and CHO candidates for every run.
7. Reject a run only under a pre-declared rule: crash/non-zero exit; missing output;
   different input hash; no application packets; malformed trace; or offered load
   differing by more than 0.5% from its paired treatments. Report all exclusions.

Illustrative command (replace paths with a run-specific directory):

```bash
NS_GLOBAL_VALUE="RngSeed=20260803;RngRun=<RUN>" ./ns3 run \
  "leosim-experiments \
  --satellites=contrib/leosim/data/prepro/satellite_mobility.tcl \
  --dataDir=contrib/leosim/data --simTime=1800 \
  --numUes=20 --numServers=4 --enableIsl=1 --maxIslNeighbors=4 \
  --enablePeriodicRouting=1 --routingUpdateInterval=10 \
  --tcpRate=<250Kbps|1Mbps|4Mbps> --tcpPacketSize=512 --appStart=60 \
  --hoMode=<BHO|CHO> --maxCandidates=<1|3> \
  --enableHoBuffering=<0|1> --enableLoadBalancing=0 \
  --ttt=1 --t310=1 --a3Offset=3 --a4Threshold=-110 --tteTrigger=30 \
  --choPrep=100 --choExec=150 --beamUpdateIntervalMs=100 \
  --logPackets=1 --logBeams=1 --flowMonitorScope=endpoints \
  --packets=<RUN_DIR>/packets.csv \
  --linkState=<RUN_DIR>/link-state.csv \
  --beamAssociations=<RUN_DIR>/beam-associations.csv \
  --handovers=<RUN_DIR>/handovers.csv --cho=<RUN_DIR>/cho.csv \
  --statisticsFile=<RUN_DIR>/statistics.json \
  --statisticsTimeSeriesFile=<RUN_DIR>/statistics.csv"
```

Before launching the campaign, verify the executable name shown by `./ns3 show
targets` and the expected satellite input format in the local build. The example's
current command-line interface exposes all primary treatment controls; the runner
should fail fast if any argument is rejected.

The complete SONIC campaign is encoded by `sonic-handover-paper.sbatch`. Build once
and then submit from the repository root:

```bash
cd ns3
CCACHE_DISABLE=1 ./ns3 build leosim-experiments
cd ..
sbatch experiments/handover/sonic-handover-paper.sbatch
```

For a small allocation test, override the array and scenario size without editing
the script:

```bash
LEOSIM_SIM_TIME=120 LEOSIM_NUM_SATELLITES=1000 \
  sbatch --array=0-2%1 experiments/handover/sonic-handover-paper.sbatch
```

## Statistical analysis

Estimate policy effects with a mixed-effects model containing policy, load,
buffering, and their interactions as fixed effects, with random-run block and UE
cluster as random intercepts. Use a log transform for interruption time and latency
if residual diagnostics support it. Model success/failure and packet loss using an
appropriate binomial mixed model rather than ordinary least squares.

For the confirmatory comparison, test CHO-3 versus BHO on median per-run interruption
time. Report the paired effect, percent change, 95% confidence interval, and p-value.
For the planned secondary comparisons (CHO-1 versus BHO and CHO-3 versus CHO-1),
apply Holm correction within each outcome. If parametric diagnostics fail, use a
paired block bootstrap with 10,000 resamples. Report effect sizes and confidence
intervals regardless of significance.

Sensitivity analyses should (i) vary the event window to [-1,+3] and [-5,+10] s,
(ii) exclude intra-beam events, failed flows, and the final 30 s separately, and
(iii) repeat the primary comparison using P95 rather than median interruption.
Label all sensitivity results as such.

## Paper figures and tables

1. **Headline event study:** received goodput from -10 s to +20 s around
   inter-satellite handover, normalized to the pre-event baseline, with confidence
   bands across runs for BHO, CHO-1, and CHO-3.
2. **Primary result:** paired point-range plot of interruption-time difference
   versus BHO, faceted by offered load and buffering.
3. **Reliability trade-off:** success ratio versus candidate state/preparations;
   point size represents event-window packet loss.
4. **Radio continuity:** violin/ECDF plot of post-HO SINR change, separated by
   policy and load.
5. **Mechanism figure:** packets buffered and dropped around CHO execution with
   buffering on/off.
6. **Main table:** policy effects with 95% CIs, adjusted p-values, run/event counts,
   exclusions, and failures.

## Ablations and extensions

Keep these out of the primary factorial experiment:

- **Prediction ablation:** CHO-3 with `tteTrigger=0` versus 30 s isolates the TTE
  lead-time mechanism.
- **Candidate sensitivity:** 1, 2, 3, and 5 candidates at medium load quantifies
  diminishing returns.
- **Control-delay stress:** preparation/execution pairs of 50/50, 100/150, and
  250/500 ms test robustness to signalling latency.
- **Weather external validity:** repeat BHO and CHO-3 with the bundled Markov weather
  trace and weather-fade handover trigger.
- **Earth-fixed beams:** repeat the winning conditions with
  `--earthFixedBeam=1` to test dependence on beam-layout assumptions.

## Validity threats and claims to avoid

- A simulator comparison demonstrates effects under LeoSim's models, not universal
  superiority in deployed networks. State all propagation, queue, TCP, beam, and
  failure assumptions.
- Do not call packets or handovers independent samples; inference is across paired
  simulation runs.
- Different numbers of handover opportunities can bias raw counts; report rates per
  UE-hour and conditional success as well as totals.
- TCP congestion recovery can extend an interruption beyond radio reconnection;
  this is intentional for the user-visible endpoint, but also report radio-only
  association outage.
- BHO and CHO may exercise different code paths. Add unit/pilot checks confirming
  identical initial association and pre-handover traffic under matched seeds.
- Candidate traces can contain non-finite SINR when the scenario lacks an
  interference estimate. Treat these values as missing, investigate their cause,
  and never silently replace them with zero.
- The TOPSIS criterion documentation and implementation should be reconciled before
  interpreting individual criterion weights; the primary study freezes weights and
  tests the complete policy rather than attributing effects to one weight.

## Decision rule for a publishable conclusion

Claim improved service continuity only if CHO-3 both (i) reduces the pre-registered
interruption endpoint relative to BHO with a confidence interval excluding zero and
(ii) does not reduce handover success or whole-run PDR. If only radio metrics improve
but application interruption does not, report that cross-layer null result: it is a
useful finding and a stronger demonstration of LeoSim's packet-level value than a
selective PHY-only claim.
