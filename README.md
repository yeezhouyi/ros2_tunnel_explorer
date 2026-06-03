# ros2_tunnel_explorer

面向隧道巡检场景的 ROS2 自主探索与风险感知路径规划系统。

## Status

| Stage | Description | Status |
|-------|-------------|--------|
| 0A | WSL2 Environment Stability | **PASS** |
| 0B-1 | Known-Free Navigation (RotationShim + DWB, 60s, 10/11) | **PASS** |
| 0B-D | DWB Turn Failure Diagnosis | **RESOLVED** |
| 1A | Frontier Algorithms (detector + blacklist + goal selector) | **PASS** |
| 1B | ROS2 Node Build & Unit Tests (24+ tests) | **PASS** |
| **1C** | **Nearest-Frontier Closed-Loop Integration** | **PASS** |
| **2A** | **Nearest-Frontier Baseline Benchmark** | **PASS** — 5 runs, 80% completion, TTC median 281.5 s |
| **2B** | **Information Gain + Revisit Penalty v1** | **PASS** — 5 runs, 100% completion, TTC median 174 s (4 formal runs, excl. run_debug), revisit median 0% |
| **2C** | **Revisit Radius Robustness** | **PASS** — revisit_radius=0.75 selected as Stage 2 final; 5 runs, 100% completion, worst revisit 9%, median TTC 200 s |
| **3A** | **Y-World Smoke/Connectivity** | **PASS** — Y-shaped branching tunnel SDF, 2.5 m corridor, thick slabs, goal projection + cooldown, spawn at (0,1) |
| **3B** | **Branching-World Dry Run** | **PASS** — COMPLETED at 732 s, 9/9 nav success, 5 unique bins, 44.4% revisit |
| **3C** | **Topology Generalization (Formal)** | **FAIL** — completion 40% (2/5), mean revisit 49.3%, entrance-frontier oscillation despite 100% Nav2 success |
| **3D** | **Entrance-Loop Recovery** | **PASS** — 5/5 explorer-level completion, mean revisit 34.6%, recovery probe 4/4 success, Nav2 100% |
| **4A** | **Tunnel Centerline Extraction** | **PASS** — distance field, Zhang-Suen skeleton, branch/endpoint detection, 8/8 tests |
| **4B** | **Tunnel-Aware Frontier Scoring** | **EVALUATED** — geometry reduces revisit (−6.9pp) and TTC (−18%) but completion regresses (80%→60%); experimental, not default |
| **4B.1** | **Safe Geometry Bias** | **EVALUATED** — wall_risk_weak best: 60% completion (vs 20% baseline), 0 fallbacks, geometry pipeline solid |

### Stage 1C Baseline Metrics

| Metric | Result |
|--------|--------|
| Navigation goal success rate | **18/18 = 100 %** |
| Unique frontier goals visited | **12** |
| Autonomous exploration runtime | **5+ min** |
| Robot movement range | **(0, 0) → (3.88, 0.40)** |
| Node crashes | **0** |
| Nav2 lifecycle failures | **0** |
| Near-goal filter activations (`centroid < 0.50 m`) | **12/18** |

**Known baseline limitation**: nearest-frontier selection creates local revisit
cycles in bounded areas. This is the baseline for comparison — Stage 2
introduces information gain and revisit-aware scoring to quantify improvement.

See [Stage 1C Smoke Results](docs/stage1c_smoke_results.md) for full details.

## Prerequisites

- Ubuntu 24.04 (or WSL2 with Ubuntu 24.04)
- ROS2 Jazzy Jalisco
- Gazebo Harmonic

### Install system dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-minimal-tb3-sim
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select tunnel_explorer_bringup benchmark_tools tunnel_frontier_explorer
source install/setup.bash
```

## Quick Start (Stage 0: Environment Verification)

### 1. Clean up stale processes

```bash
./scripts/cleanup_simulation.sh
```

This kills leftover `gz sim` processes, stops the ROS2 daemon, and removes
stale Fast DDS shared-memory files in `/dev/shm`.

### 2. Launch simulation

```bash
ros2 launch tunnel_explorer_bringup stage0_simulation.launch.py headless:=True
```

> **WSL2 note**: Use `headless:=True` to avoid Gazebo GUI conflicts.
> The `gz sim` GUI client can accidentally start a second server with an
> empty world. See [WSL2 Restart Checklist](#wsl2-restart-checklist).

This starts:
- TurtleBot3 in Gazebo Harmonic (headless simulation)
- SLAM Toolbox (online async mapping)
- Nav2 navigation stack
- RViz2 with pre-configured view (launches separately even in headless mode)

### 3. Record metrics

In a second terminal:

```bash
ros2 run benchmark_tools record_stage0_metrics.py \
    --duration 600 \
    --output-dir /tmp/stage0_results
```

### 4. Run navigation smoke test

In a third terminal (after SLAM has built a map):

```bash
ros2 run benchmark_tools run_navigation_smoke_test.py \
    --output-dir /tmp/stage0b_results
```

**Important**: Adjust goal coordinates in `benchmark_tools/config/stage0b_goals.yaml`
to match your current SLAM map.

### 5. Review results

```bash
cat /tmp/stage0_results/stage0_metrics.json
cat /tmp/stage0_results/stage0_metrics.md
cat /tmp/stage0b_results/stage0b_results.json
cat /tmp/stage0b_results/stage0b_results.md
```

## Quick Start (Stage 1: Frontier-Based Exploration)

Stage 1 uses `tunnel_frontier_explorer` (C++) to detect frontier clusters from
the `/map` occupancy grid and send nearest-frontier goals to Nav2.

> **Stage 1C verified**: 18/18 navigation goals succeeded, 12 unique frontiers
> visited, 0 crashes over 5+ min. See [Smoke Results](docs/stage1c_smoke_results.md).

### Prerequisites

- Stage 0 simulation stack running (Gazebo + SLAM Toolbox + Nav2)
- Map partially built (SLAM has published at least one map)

### Launch frontier explorer

In a separate terminal while the simulation is running:

```bash
ros2 launch tunnel_frontier_explorer frontier_explorer.launch.py
```

The node will:
1. Wait for a map from SLAM Toolbox
2. Wait for the Nav2 `/navigate_to_pose` action server
3. Detect frontier clusters (free cells adjacent to unknown space)
4. Select the nearest non-blacklisted frontier
5. Send a `NavigateToPose` goal using the cluster's representative cell
6. On failure, blacklist the goal with a configurable radius and timeout
7. Publish RViz markers on `~/frontier_markers`

### RViz visualization

Add a MarkerArray display in RViz subscribed to `/frontier_markers`:

| Namespace | Color | Shape | Description |
|-----------|-------|-------|-------------|
| `frontier_clusters` | Green | Points | Centroids of all detected frontier clusters |
| `selected_goal` | Red | Sphere | Currently selected navigation goal |
| `blacklisted` | Grey | Sphere | Temporarily forbidden failed goals |
| `too_close_frontiers` | Yellow | Points | Candidate goals within `min_goal_distance_meters` (filtered by FrontierGoalSelector) |
| `scored_frontiers` | Red→Green | Sphere List | Scored candidates (`information_gain_revisit` strategy), size ∝ score |

### Parameters

All parameters are in `config/frontier_explorer_params.yaml`. Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `exploration_period_seconds` | 1.0 | Main loop interval (Hz) |
| `cooldown_seconds` | 5.0 | Pause between navigation goals |
| `goal_timeout_seconds` | 60.0 | Single-goal timeout before cancel + blacklist |
| `min_cluster_size` | 10 | Minimum cells for a frontier cluster |
| `frontier_neighbor_connectivity` | 4 | Neighbourhood for frontier detection |
| `cluster_connectivity` | 8 | Neighbourhood for BFS clustering |
| `blacklist_radius_meters` | 0.5 | Radius to blacklist failed goals |
| `blacklist_timeout_seconds` | 60.0 | Blacklist expiry time |
| `orient_goal_toward_frontier` | true | Face the robot toward the goal |
| `min_goal_distance_meters` | 0.50 | Minimum distance from robot to goal; prevents false-success within Nav2 `xy_goal_tolerance` |
| `selection_strategy` | `nearest` | Goal selection: `nearest` (Stage 2A baseline) or `information_gain_revisit` (Stage 2B) |
| `information_gain_radius_meters` | 0.75 | Radius (m) to count unknown cells around each candidate goal |
| `revisit_radius_meters` | 0.50 | Radius (m) for detecting previously visited areas |
| `max_revisit_count` | 3 | Clamp for revisit count (prevents extreme penalty) |
| `weight_information_gain` | 1.0 | Score weight for normalised information gain |
| `weight_distance` | 1.0 | Score weight for normalised distance (subtracted) |
| `weight_revisit` | 1.5 | Score weight for normalised revisit penalty (subtracted) |

### Architecture

- `FrontierDetector`: Pure C++ class (no ROS deps) for BFS frontier detection,
  clustering, centroid computation, and representative-cell selection.
- `FrontierBlacklist`: Pure C++ class (no ROS deps) for radius + timeout-based
  goal blacklisting with injectable clocks for deterministic testing.
- `FrontierGoalSelector`: Pure C++ class (no ROS deps) that enforces
  `min_goal_distance_meters` from robot to goal; searches cluster for
  alternative free cells when the representative is too close, preventing
  false-success cycles caused by Nav2 `xy_goal_tolerance`.  Also provides
  `selectAll()` to return all distance-filtered candidates for external scoring.
- `FrontierScorer` (Stage 2B): Pure C++ class (no ROS deps) that scores candidate
  goals by information gain (unknown cells within circular radius), distance,
  and revisit penalty.  Supports deterministic tie-breaking.
- `FrontierVisitHistory` (Stage 2B): Pure C++ class (no ROS deps) that records
  goals accepted by the Nav2 action server for use in revisit penalty calculation.
- `TunnelFrontierExplorerNode`: ROS2 node with 6-state machine
  (WAITING_FOR_MAP → WAITING_FOR_NAV2 → IDLE → NAVIGATING → COOLDOWN → COMPLETED).
  Supports `nearest` and `information_gain_revisit` selection strategies.

## Packages

| Package | Language | Description |
|---------|----------|-------------|
| `tunnel_explorer_bringup` | Python/YAML | Launch files, configs, RViz views |
| `benchmark_tools` | Python | Metrics recording, analysis, plotting |
| `tunnel_frontier_explorer` | C++ | Frontier-based autonomous exploration (Stage 1, nearest-frontier baseline) |
| `tunnel_centerline_extractor` | C++ | Tunnel centerline distance field, skeleton, branch-point extraction (Stage 4A) |
| `tunnel_aware_planner` | C++ | Tunnel-aware global planner plugin for Nav2 (Planned — Stage 5) |
| `tunnel_worlds` | Python/SDF | Parametric tunnel world generator (Stage 3) |

## Quick Start (Stage 2A: Baseline Benchmark)

Stage 2A runs the nearest-frontier baseline 5× to collect repeatability metrics.
This establishes the baseline for comparing tunnel-aware scoring (Stage 2B+).

The benchmark supports **early-stop-on-completed**: in bounded environments the
robot may finish exploration in 2-3 minutes. Instead of waiting the full 600 s,
the benchmark detects the COMPLETED state, applies a grace window, and finishes
early. The primary metric becomes **time-to-completion** rather than goals-per-time.

### Single run

```bash
./scripts/run_stage2a_benchmark.sh \
  --output-dir ~/stage2a_benchmark \
  --duration 600 \
  --run-id 01 \
  --stop-on-completed true \
  --completed-grace-seconds 20 \
  --stall-timeout-seconds 90
```

### Full benchmark (5 runs)

```bash
for i in $(seq -w 1 5); do
  ./scripts/run_stage2a_benchmark.sh \
    --output-dir ~/stage2a_benchmark \
    --duration 600 \
    --run-id "${i}" \
    --stop-on-completed true \
    --completed-grace-seconds 20 \
    --stall-timeout-seconds 90
  sleep 10
done
```

### Aggregation (after all runs)

```bash
./scripts/aggregate_stage2a_results.sh ~/stage2a_benchmark
```

Only `COMPLETED` runs are included in the aggregate. Excluded runs are listed
separately. Use `--allow-timeout` to include `TIMEOUT` runs.

### Each run produces

- `benchmark_results.md` — goals, success rate, unique goals, run status, completion time
- `benchmark_results.json` — structured JSON metrics (used by aggregation)
- `frontier_explorer.log` — full explorer log
- `bag/` — ROS2 bag with `/map`, `/odom`, `/cmd_vel`, `/tf`, markers
- `attempt_NN/` — per-attempt subdirectory (when `--runtime-retries > 0`)

Run with `--wait-time 90` on WSL2 to account for DDS discovery delay (default).

### Run status reference

| Status | Meaning |
|--------|---------|
| `COMPLETED` | Exploration finished within the max duration |
| `TIMEOUT` | Max duration reached before completion |
| `STALLED` | Explorer alive but no progress for N seconds (DDS/SLAM/TF stall) |
| `CRASHED` | Explorer node crashed during run |
| `STARTUP_FAILED` | Nav2 not ready after retries |
| `INVALID_ORCHESTRATION_TERMINATION` | Run externally terminated or outputs incomplete |

### STALLED Injection Test

For verifying the STALLED detection path:

```bash
./scripts/run_stage2a_benchmark.sh \
  --output-dir ~/stage2a_benchmark_test \
  --duration 180 \
  --run-id stall_test \
  --stop-on-completed true \
  --stall-timeout-seconds 45 \
  --inject-stall-after-seconds 20
```

Expected: run enters STALLED at ~65 s (20 + 45, with ±5 s monitor jitter).
This is a **test-only** feature. Formal benchmarks must not use `--inject-stall-after-seconds`.

## Quick Start (Stage 2B: Information Gain + Revisit Penalty)

Stage 2B adds information gain weighting and revisit penalty to frontier goal
selection.  **PASS** — 5-run A/B benchmark vs Stage 2A baseline shows:

| Metric | 2A (nearest) | 2B (info+revisit) | Change |
|--------|:------------:|:-----------------:|:------:|
| Completion rate | 80 % | **100 %** | +25 % |
| TTC median | 281.5 s | **156.0 s** | −44.6 % |
| Revisit rate median | 20 % | **0 %** | −100 % |
| Nav goal success | 97.7 % | **100 %** | +2.3 pp |

See [docs/stage2b_information_gain_revisit_results.md](docs/stage2b_information_gain_revisit_results.md).

Use the `information_gain_revisit` strategy via a separate params file.

### Launch with Stage 2B strategy

```bash
ros2 launch tunnel_frontier_explorer frontier_explorer.launch.py \
  params_file:=<path-to-install>/config/frontier_explorer_params_info_revisit.yaml
```

The explorer node will log scoring details for each dispatched goal:

```
Goal: (1.23, 4.56) dist=2.34 gain=42(raw)/3.76(tr)/0.85(norm) revisit=0(raw)/0(cl)/0.00(norm) score=0.85 [5 cand]
```

### Stage 2A baseline (unchanged)

```bash
ros2 launch tunnel_frontier_explorer frontier_explorer.launch.py
```

This still uses the default `nearest` strategy — exactly the same behaviour as
Stage 2A.

### Stage 2B benchmark variant

```bash
./scripts/run_stage2a_benchmark.sh \
  --output-dir ~/stage2b_benchmark \
  --duration 600 \
  --run-id 01 \
  --stop-on-completed true \
  --completed-grace-seconds 20 \
  --stall-timeout-seconds 90 \
  --explorer-params-file <path-to-install>/config/frontier_explorer_params_info_revisit.yaml
```

The benchmark records `selection_strategy`, all scoring parameters, and a
SHA-256 hash of the params file in each run's `benchmark_results.json`.

## Quick Start (Stage 3: Topology Generalization)

Stage 3 tests whether the Stage 2C exploration policy (information-gain +
revisit-suppression) generalizes beyond L-shaped corridors to a Y-shaped
branching tunnel with bifurcated geometry.

### Y-World

A custom Gazebo SDF world (`tunnel_worlds/worlds/branching_tunnel_y.sdf`)
with a 2.5 m trunk corridor splitting into left (120°) and right (60°) branches.
Negative-space geometry (thick solid slabs instead of thin walls) ensures only
the intended Y-shaped tunnel interior can be mapped as free space.

### Stage 3 Explorer Additions

| Parameter | Default | Description |
|-----------|---------|-------------|
| `goal_projection_enabled` | true | Pull frontier goal inward toward robot |
| `goal_projection_distance` | 0.4 | Projection distance (m) |
| `goal_projection_min_remaining_distance` | 0.6 | Min robot-to-projected-goal distance (m) |
| `goal_success_cooldown_seconds` | 120.0 | Radius-based cooldown after success (s) |
| `goal_success_cooldown_radius` | 1.0 | Cooldown region radius (m) |
| `loop_detection_enabled` | true | Sliding-window entrance-loop detector |
| `loop_window_size` | 6 | Recent goal bin window |
| `loop_unique_bins_threshold` | 2 | Trigger when unique bins ≤ 2 |
| `loop_min_successes` | 3 | Trigger when successes ≥ 3 in window |
| `recovery_probe_distances` | [1.2, 1.0, 0.8] | Forward probe distances (m) |
| `recovery_probe_angle_offsets_deg` | [0, 20, -20, 35, -35] | Probe angle offsets (°) |
| `recovery_probe_cooldown_seconds` | 30.0 | Cooldown between probes (s) |
| `recovery_max_attempts` | 3 | Max probes before declaring STALLED |

### Launch Stage 3 benchmark

```bash
WORLD_PATH="$(ros2 pkg prefix tunnel_worlds)/share/tunnel_worlds/worlds/branching_tunnel_y.sdf"
PARAMS_PATH="$(ros2 pkg prefix tunnel_frontier_explorer)/share/tunnel_frontier_explorer/config/frontier_explorer_params_info_revisit_r075.yaml"

./scripts/run_stage2a_benchmark.sh \
  --explorer-params-file "$PARAMS_PATH" \
  --world "$WORLD_PATH" \
  --stop-on-completed true \
  --stall-timeout-seconds 240 \
  --duration 900
```

Nav2 uses `nav2_params_rotation_shim.yaml` with relaxed goal checker:
`xy_goal_tolerance: 0.35, yaw_goal_tolerance: 6.28`.

### Stage 3 Results

| Stage | Result | Completion | Mean unique bins | Mean revisit | Notes |
|-------|--------|------------|-----------------|-------------|-------|
| 3A | PASS | smoke | — | — | World geometry + launch verified |
| 3B | PASS | dry run | 5.0 | 44.4% | Single-run validation |
| 3C | FAIL | 40% (2/5) | 4.0 | 49.3% | Entrance-frontier oscillation |
| **3D** | **PASS** | **100% (5/5)** | **6.0** | **34.6%** | Loop detector + recovery probe |

Stage 3D recovery probe: dispatched in 4/5 runs, succeeded 4/4 times, consistently
broke entrance oscillation enabling trunk-depth frontier discovery. Nav2 execution
remained 100% across all stages.

> **Note**: 3/5 Stage 3D runs were classified as TIMEOUT by the benchmark harness
> due to late-arriving map messages resetting the stable-completion grace period.
> All 5 runs logged `"No frontiers for 10 cycles — exploration complete"` at the
> explorer level. Actual completion was 100%.

### Artifacts

- `artifacts/stage3c_branching_y_failed_eval/` — Stage 3C formal results
- `artifacts/stage3d_entrance_loop_recovery/` — Stage 3D formal results + source snapshot

## Documentation

- [Environment Feasibility](docs/environment_feasibility.md) — Stage 0 verification
- [DWB Turn Diagnosis](docs/stage0b_dwb_turn_diagnosis.md) — Stage 0B-D diagnostic plan
- [Stage 1C Smoke Results](docs/stage1c_smoke_results.md) — nearest-frontier baseline verification
- [Stage 2A Baseline Results](docs/stage2a_nearest_frontier_baseline_results.md) — 5-run nearest-frontier baseline
- [Stage 2B Design](docs/stage2b_information_gain_revisit_design.md) — information gain + revisit penalty scoring
- [ROS2 Jazzy Compatibility](docs/jazzy_compatibility.md)
- [Stage 3D Entrance-Loop Recovery Results](docs/stage3d_entrance_loop_recovery_results.md)
- [Stage 4B Tunnel-Aware Scoring Results](docs/stage4b_tunnel_aware_frontier_scoring_results.md)
- [Stage 4B.1 Safe Geometry Bias Results](docs/stage4b1_safe_geometry_bias_results.md)

## Roadmap

| Milestone | Description | Status |
|-----------|-------------|--------|
| **v0.1** | Stage 3D finalization, benchmark harness cleanup | ✅ `v0.1-stage3d-clean` |
| **Stage 4A** | Tunnel centerline / distance-field / branch-point extraction | ✅ `stage4a-centerline-extraction` |
| **Stage 4B** | Centerline + wall-risk features in frontier scorer | ✅ `stage4b-tunnel-aware-frontier-scoring` |
| **Stage 4B.1** | Safe geometry bias — wall-risk penalty, completion 20%→60% | ✅ `stage4b1-safe-geometry-bias-eval` |
| **Stage 4C** | L / Y / T / cross / dead-end multi-topology benchmark suite | 📋 |
| **Stage 5** | Nav2 Tunnel-Aware Planner plugin (wall risk + centerline deviation cost) | 📋 |
| **Stage 6** | Sensor degradation, narrow passages, dynamic obstacles, rosbag validation | 📋 |
| **Stage 7** | Hardware / HITL validation | 📋 |

## WSL2 Restart Checklist

When restarting the simulation on WSL2, follow these steps to avoid
common issues:

1. **Clean up processes**: `./scripts/cleanup_simulation.sh`
   - Kills stale `gz sim` and `ros_gz_bridge` processes
   - Stops the ROS2 daemon
   - Removes Fast DDS shared-memory files from `/dev/shm`
2. **Launch headless**: Use `headless:=True` — the Gazebo GUI client
   (`gz sim -g`) can inadvertently start a second server with an empty
   `empty.sdf` world if a server is not already running, which conflicts
   with the main simulation.
3. **Wait for Nav2 (WSL2 DDS discovery delay)**: Under WSL2, the ROS2 daemon
   takes 60–90 s to discover all lifecycle nodes even after they are running.
   Run `scripts/wait_for_nav2_active.sh` — if it times out at 60 s, retry;
   the nodes are likely up but not yet discovered. The benchmark script uses
   `--wait-time 90` by default.
4. **SLAM Toolbox lifecycle race**: In rare cases, `slam_toolbox` gets stuck
   in `unconfigured [1]` because the lifecycle manager sends CONFIGURE/ACTIVATE
   before the node is ready. Workaround:
   ```bash
   ros2 lifecycle set /slam_toolbox configure
   ros2 lifecycle set /slam_toolbox activate
   ```
   After activation, Nav2 nodes that depend on the `map` frame will come up.
5. **Verify topics**: After launch, confirm `/clock`, `/scan`, and `/map`
   are publishing before starting metrics recording.
6. **Monitor DDS**: If you see `Failed init_port fastrtps_port7000:
   open_and_lock_file failed` errors, run `cleanup_simulation.sh` again.

## ROS2 Jazzy Compatibility Notes

This project targets ROS2 Jazzy. Notable differences from Humble-era examples:

- **Plugin names**: Use `::` separator (e.g., `nav2_navfn_planner::NavfnPlanner`)
  instead of `/` (e.g., `nav2_navfn_planner/NavfnPlanner`).
- **Additional server configs**: Jazzy's Nav2 requires `collision_monitor`,
  `docking_server`, `route_server`, and `map_saver` sections in `nav2_params.yaml`.
- **bt_navigator**: Requires explicit `navigators` plugin declarations.

See [docs/jazzy_compatibility.md](docs/jazzy_compatibility.md) for full details.

## License

Apache-2.0
