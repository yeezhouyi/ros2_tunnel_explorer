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
     the benchmark waits a 20 s grace period (configurable).
   - **Only new goal dispatch** resets the grace timer. SLAM map updates,
     marker refreshes, and state-machine re-evaluations do NOT reset it.
   - If the robot remains in COMPLETED for the full grace window (no new
     goals dispatched), the run ends early with `run_status=COMPLETED` and
     `stable_completion_detected=true`.
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
| `COMPLETED` | Exploration finished within the max duration |
| `TIMEOUT` | Max duration reached before exploration completed |
| `STALLED` | Explorer process alive but no progress for N seconds (DDS stall, SLAM freeze, TF timeout, action server hang) |
| `CRASHED` | Explorer node died prematurely |
| `STARTUP_FAILED` | Nav2 not ready after max retry attempts |
| `INVALID_ORCHESTRATION_TERMINATION` | Run was externally terminated or outputs incomplete |

### STALLED Detection

STALLED indicates the explorer process is alive but making no progress.
It is distinct from TIMEOUT (ran for max duration) and CRASHED (process died).

**Progress events**: The monitor loop tracks the last progress event timestamp.
Only the following log patterns refresh the progress timer (map updates are NOT
progress events — SLAM may publish identical maps even when the robot is stuck):

- `Goal accepted by Nav2 server` — goal dispatched
- `Navigation to goal succeeded` — goal completed
- `Navigation aborted` — genuine failure (still progress)
- `Goal timed out after` — goal timed out (still progress)

**Threshold**: `--stall-timeout-seconds` (default: 90 s). If no progress event
occurs within the threshold and the run is not in stable COMPLETED, the run
is classified as STALLED.

**STALLED is not an algorithm failure**: The nearest-frontier algorithm did
not cause the stall. Root causes in the current environment include:

- **DDS action server stall**: Under WSL2, Fast DDS action server communication
  can block the ROS2 executor, preventing timer callbacks from firing. The explorer
  gets stuck in NAVIGATING state.
- **SLAM Toolbox freeze**: Occasionally SLAM stops publishing map updates.
- **TF timeout**: Transform lookup blocks if a frame chain is interrupted.
- **State machine deadlock**: A race between goal result and map callback.

**Mitigation**: In the WSL2 test environment, limited automatic retries
(`--runtime-retries`) are allowed. Each retry runs `cleanup_simulation.sh`
and starts fresh. CRASHED runs are NOT automatically retried.

Only `COMPLETED` runs are included in cross-run aggregation by default.
`TIMEOUT` runs can be included with `--allow-timeout`.

### STALLED Injection (Test Only)

A deterministic STALLED injection mechanism exists for verifying the detection
path end-to-end:

- Parameter: `--inject-stall-after-seconds` (default: 0 = disabled)
- When > 0: after N seconds, the runner stops refreshing the progress timestamp
- The explorer continues running normally, but the runner waits for
  `STALL_TIMEOUT_SECONDS` without progress, then classifies the run as STALLED
- The injection activation epoch is recorded for precise timing verification
- **Test only**: formal benchmarks must NOT use this flag
- Runs with `injected_stall_enabled=true` are always excluded from algorithm aggregation

## Metrics per Run

### Summary

| Metric | Source |
|--------|--------|
| Run status | Enum from termination condition |
| Completion reset count | JSON `completion_reset_count` |
| Completion reset reason | JSON `completion_reset_reason` |
| Stable completion | JSON `stable_completion_detected` |
| Max duration | Safety cap |
| Explorer active duration | Total time explorer was running |
| Stall detected | Monitor loop progress check |
| Stall timeout seconds | `--stall-timeout-seconds` parameter |
| Stalled after seconds | Elapsed time at STALLED detection |
| Last progress event | Most recent progress event string |

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

The aggregation script uses per-attempt `benchmark_results.json` as the source of
truth (not markdown tables).

### Three-Layer Classification

**Algorithm Outcomes** (included in performance stats):
- `COMPLETED` with `stable_completion_detected=true`
- `TIMEOUT` (transparently reported with count)

**Algorithm Exclusions** (listed, not aggregated):
- `COMPLETED` but `injected_stall_enabled=true` (test runs)
- `COMPLETED` but `stable_completion_detected=false` (candidate never stabilized)

**Infrastructure Exclusions** (never in algorithm stats):
- `STALLED`
- `CRASHED`
- `STARTUP_FAILED`
- `INVALID_ORCHESTRATION_TERMINATION`

TIMEOUT runs can be included in aggregation with `--allow-timeout`.

### Injected Test Hard Filter

Runs with `injected_stall_enabled=true` are always excluded from algorithm
aggregation, even if they somehow produce COMPLETED status.

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

> **Note**: STALLED runs may occur due to WSL2 DDS issues. These are excluded
> from formal aggregation. Allow up to 1-2 STALLED runs per 5-run batch.

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
