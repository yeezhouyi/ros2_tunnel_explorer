# Stage 2A: Nearest-Frontier Baseline Benchmark Plan

## Objective

Quantify the nearest-frontier baseline exploration performance over 5 repeated
runs to establish a statistically meaningful baseline for comparing Stage 2B+
tunnel-aware scoring variants.

## Frozen Configuration

| Component | Setting |
|-----------|---------|
| Controller | RotationShimController + DWB |
| Goal timeout | 60.0 s |
| Cooldown | 5.0 s |
| Blacklist radius | 0.5 m |
| Blacklist timeout | 60.0 s |
| Min cluster size | 10 cells |
| Min goal distance | 0.50 m |
| Frontier connectivity | 4-neighbor |
| Cluster connectivity | 8-neighbor |

## Protocol

1. Run `scripts/cleanup_simulation.sh` before each run
2. Launch simulation with `nav2_params_rotation_shim.yaml`
3. Wait 90 s for WSL2 DDS discovery (or until Nav2 lifecycle nodes are active)
4. Launch `tunnel_frontier_explorer` with `frontier_explorer_params.yaml`
5. Record rosbag (10 topics: map, scan, tf, odom, cmd_vel, costmaps, markers)
6. Run up to 600 s (10 min) per run, with early-stop-on-completed:

   - Maximum wall-clock duration is 600 s (safety cap)
   - When the explorer enters the COMPLETED state (all frontiers explored),
     the benchmark waits a 20 s grace period
   - If the robot remains in COMPLETED for the full grace window (no new
     goals dispatched, no re-evaluation triggered by SLAM map updates),
     the run ends early
   - If new goals appear during the grace window, the timer resets
7. Kill all processes
8. Extract metrics from explorer log
9. Repeat 5×

### Why early-stop?

In bounded environments like the default TurtleBot3 world, the robot may
complete exploration in 2-3 minutes. Waiting the full 600 s adds no signal
and wastes time. Early-stop shifts the primary metric from "goals per 10 min"
to **time-to-completion**, which is more meaningful for autonomous exploration.

### Run status classification

After each run, the benchmark assigns one of:

| Status | Meaning |
|--------|---------|
| `COMPLETED` | Explorer reached COMPLETED state and stayed there for the grace window |
| `TIMEOUT` | Max duration reached before exploration completed |
| `CRASHED` | Explorer node died prematurely |
| `STARTUP_FAILED` | Nav2 not ready after max retry attempts |
| `INVALID_ORCHESTRATION_TERMINATION` | Run was externally terminated (e.g., task killed) or outputs are incomplete |

Only `COMPLETED` runs are included in cross-run aggregation by default.
`TIMEOUT` runs can be included with `--allow-timeout`.

## Metrics per Run

### Summary

| Metric | Source |
|--------|--------|
| Run status | Enum from termination condition |
| Completion detected | Whether COMPLETED state was ever observed |
| Completion time | Wall-clock seconds from explorer start to COMPLETED detection |
| Completed grace seconds | Grace window configured |
| Max duration | Safety cap |
| Explorer active duration | Total time explorer was running |

### Exploration Efficiency

| Metric | Source |
|--------|--------|
| Total goals dispatched | `grep -c "Goal:" frontier_explorer.log` |
| Goals succeeded | `grep -c "succeeded" frontier_explorer.log` |
| Goals timed out | `grep -ci "timeout" frontier_explorer.log` |
| Goals aborted | `grep -ci "abort" frontier_explorer.log` |
| Unique goals visited | 0.25 m spatial binning |
| Repeated goals | total - unique |
| Revisit rate | repeated / total × 100 % |

### Goal Quality

| Metric | Source |
|--------|--------|
| Goal distance min/max | `grep -oP 'goal=\K[0-9.]+' \| sort -n` |
| Centroid distance min/max | `grep -oP 'centroid=\K[0-9.]+' \| sort -n` |
| Filter activations | centroid < 0.5 m AND goal ≥ 0.5 m |

### Cluster Characteristics

| Metric | Source |
|--------|--------|
| Min/max cluster cells | `grep -oP '\[\K[0-9]+(?= cells)' \| sort -n` |

### System Stability

| Metric | Source |
|--------|--------|
| Node crashes | Explorer alive check during run |
| Nav2 lifecycle failures | `ros2 lifecycle get` at start/end |

## Cross-Run Aggregation (after 5 runs)

The aggregation script (`scripts/aggregate_stage2a_results.sh`) reads each
run's `benchmark_results.md` and `rosbag_metrics.json`, filters to valid runs,
and computes median/min/max for each metric.

Only `COMPLETED` runs are included by default. `TIMEOUT` runs can be included
with `--allow-timeout`. `CRASHED`, `STARTUP_FAILED`, and
`INVALID_ORCHESTRATION_TERMINATION` runs are always excluded.

Excluded runs are listed with their status in the output report.

## Offline Analysis (from rosbag)

The rosbag enables deeper analysis not captured in the explorer log:

- **Travel distance**: Integrate `/odom` linear velocity over time
- **Map coverage**: Sample `/map` at intervals, compute known/free ratio
- **Control analysis**: Count DWB "No valid trajectories" warnings,
  RotationShim triggers
- **Timeline**: Plot goal sequence, distances, and results over time

These analyses require post-processing scripts (not built in v1 of the
benchmark runner).

## Expected Outcomes

Based on Stage 1C smoke test (single run, 5 min):

- Goals dispatched: ~10-20 per run (depends on world size)
- Success rate: ≥ 90 %
- Unique goals: ~7-15
- Revisit rate: 0-40 %
- Completion time: ~2-3 min in default TurtleBot3 world
- Filter activations: ~60-80 % of all goals (centroid likely near robot in
  early exploration)

## Deliverable

After 5 runs, a summary table:

```text
Metric                      Median   Min   Max
Goals dispatched               18     15     22
Goals succeeded                17     14     21
Success rate (%)              100     90    100
Unique goals                   12     10     15
Revisit rate (%)               33     20     50
Filter activations             12     10     15
Min goal distance (m)         0.5    0.5    0.5
Max goal distance (m)         2.7    1.6    3.5
Completion time (s)          134    120    150
```
