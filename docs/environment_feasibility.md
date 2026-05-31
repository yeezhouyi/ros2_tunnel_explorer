# Stage 0: WSL2 Environment Feasibility Verification

## Overview

Stage 0 is split into two sub-stages:

- **Stage 0A — Environment Stability**: Verify that the simulation stack (TurtleBot3 +
  Gazebo Harmonic + Nav2 + SLAM Toolbox) runs stably under WSL2 for 10 continuous
  minutes without crashes, clock reversals, or sustained sensor gaps.
- **Stage 0B — Navigation Functionality**: Verify that the Nav2 navigation stack can
  execute simple NavigateToPose goals with >= 90 % success rate in the default
  sandbox world.

## Current Status Summary

| Stage | Status | Notes |
|-------|--------|-------|
| Stage 0A (environment stability) | **PASS** | WSL2 environment verified — clock monotonic, no TF timeouts, 10 min stable |
| Stage 0B-1 (known-free navigation) | **FAIL** — 36.4% (4/11) | ≥90% required. DWB "No valid trajectories" at >18° heading change. |
| Stage 0B-D (DWB turn diagnosis) | **IN PROGRESS** | Minimal reproduction, RViz costmap inspection, Spin action test pending |
| Stage 1A (frontier algorithms) | **PASS** | Pure C++ unit tests pass |
| Stage 1B (ROS2 node build) | **PASS** | Package compiles, launches, static analysis clean |
| Stage 1C (simulation integration) | **BLOCKED** | Requires Stage 0B-1 pass for clean baseline attribution |

## Test Infrastructure

- **OS**: Ubuntu 24.04 (Noble) under WSL2 on Windows 11
- **ROS2**: Jazzy Jalisco
- **Simulator**: Gazebo Harmonic (gz-sim8)
- **Robot**: TurtleBot3 (via nav2_minimal_tb3_sim)
- **SLAM**: SLAM Toolbox (online async mode)
- **Navigation**: Nav2 (NavFn planner + DWB controller)

---

## Stage 0A: Environment Stability Verification

### Hard Requirements (all must pass)

| # | Criterion | Measurement Method |
|---|-----------|-------------------|
| H1 | `/clock` is strictly monotonic (no time reversals) | `record_stage0_metrics.py` — `clock_monotonicity.strictly_monotonic` |
| H2 | No continuous TF timeouts (> 5 s gaps) | `ros2 run tf2_tools view_frames` after run |
| H3 | Simulation runs for >= 10 min without Gazebo or RViz crash | Manual observation + process exit codes |

### Soft Thresholds

| # | Criterion | Threshold | Measurement Method |
|---|-----------|-----------|-------------------|
| S1 | Steady-state Real-Time Factor (RTF) median | >= 0.8 | `record_stage0_metrics.py` — `real_time_factor.steady.rtf` |
| S2 | `/scan` steady-state average frequency | >= 90 % of expected (5 Hz nominal → >= 4.5 Hz) | `record_stage0_metrics.py` — `topics.scan.steady.mean_hz` |
| S3 | `/scan` steady-state gaps > 1.0 s | 0 gaps in steady phase | `record_stage0_metrics.py` — `topics.scan.steady.gaps.count` |
| S4 | `/map` updates arrive within update_interval + 5 s | No gaps > 10 s in any phase | `record_stage0_metrics.py` — `topics.map.*.gaps` |

### Duration Boundary

Recording duration is measured as:

```
pass = observed_wall_duration >= requested_duration - tolerance
```

where `tolerance` defaults to 2.0 s. This accounts for the shutdown-timer
granularity that may cause the recorded wall time to fall slightly short of
the requested value. Both `requested_duration_seconds` and
`duration_check` are included in the JSON report.

### Procedure

```bash
# Terminal 1: Launch simulation (headless recommended for WSL2)
./scripts/cleanup_simulation.sh
ros2 launch tunnel_explorer_bringup stage0_simulation.launch.py headless:=True

# Terminal 2: Record metrics
ros2 run benchmark_tools record_stage0_metrics.py \
    --duration 600 \
    --warmup-seconds 30 \
    --output-dir /tmp/stage0_results

# Monitor during the run
ros2 topic hz /clock
ros2 topic hz /scan

# Review results
cat /tmp/stage0_results/stage0_metrics.json
cat /tmp/stage0_results/stage0_metrics.md
```

---

## Stage 0B-1: Known Free Space Navigation Verification

Verify that the Nav2 navigation stack (NavFn + DWB) can execute simple
NavigateToPose goals within areas already observed by SLAM.

### Background

Stage 0B-1 is split from the original Stage 0B after Attempt 1 revealed
that the DWB controller cannot find valid trajectories when goals require
turning into unknown (unobserved) space. This is expected behaviour: the
online-SLAM navigation handover is the responsibility of the Frontier
Explorer (Stage 1C).

### Acceptance Criteria

| # | Criterion | Threshold | Measurement Method |
|---|-----------|-----------|-------------------|
| N1 | NavigateToPose success rate | >= 90 % | `run_navigation_smoke_test.py` — `summary.success_rate` |
| N2 | No sustained TF timeout during navigation | No continuous TF warnings in log | Manual log inspection |
| N3 | No sustained costmap timeout during navigation | No continuous costmap warnings in log | Manual log inspection |

### Procedure

1. Complete Stage 0A first (simulation running with SLAM and Nav2 active).
2. Verify goal coordinates are within known-free costmap cells (see RViz
   diagnosis below).
3. Run the automated smoke test:

```bash
# Terminal 3 (while simulation is running):
ros2 run benchmark_tools run_navigation_smoke_test.py \
    --goals config/stage0b_known_free_goals.yaml \
    --goal-timeout-seconds 30 \
    --continue-on-failure true \
    --cooldown-seconds 1.0 \
    --output-dir /tmp/stage0b_results
```

The smoke test sends goals from a YAML config file sequentially. Each goal
has its own timeout; if it expires the goal is cancelled and the next one
starts after a cooldown. Results are written as JSON and Markdown reports.

### Review Results

```bash
cat /tmp/stage0b_results/stage0b_results.json
cat /tmp/stage0b_results/stage0b_results.md
```

### Diagnosis

If Stage 0B-1 fails at heading changes, see the controlled diagnosis
experiment in [stage0b_dwb_turn_diagnosis.md](stage0b_dwb_turn_diagnosis.md).

### RViz Diagnosis (Before Running)

Before running the smoke test, visually check the local costmap at the
turn point to ensure goal coordinates are in free space:

1. Add these displays in RViz:
   - **Map**: `/local_costmap/costmap` (or `/local_costmap/costmap_raw`)
   - **Map**: `/global_costmap/costmap` (or `/global_costmap/costmap_raw`)
   - **Polygon**: `/local_costmap/published_footprint`

2. Find actual topic names if the defaults differ:
   ```bash
   ros2 topic list | grep costmap
   ros2 topic list | grep footprint
   ```

3. Colour legend for costmap cells:
   - **Gray**: Unknown (unobserved) — goals should not be placed here
   - **Green/White**: Free space — goals should be placed here
   - **Red**: Lethal obstacle / inflation layer — keep distance

4. If a goal is rejected by the DWB controller, check whether the
   turn point costmap shows unknown, lethal, or inflated cells blocking
   the trajectory.

## Stage 0B-2: Online SLAM and Frontier Explorer Handover Verification

Verify that the Frontier Explorer (Stage 1C) correctly generates
navigation goals at the boundary of known free space, using the
`representative_cell` mechanism to avoid sending goals into unknown area.

### Acceptance Criteria

| # | Criterion | Threshold |
|---|-----------|-----------|
| F1 | Frontier Explorer starts and builds a map | Map expands beyond initial area |
| F2 | Frontier navigates to at least 3 frontiers | 3 consecutive successful navigations |
| F3 | No navigation goal sent to unknown/lethal costmap cell | All goals fall on known free cells |

### Procedure

```bash
# Terminal 1: Launch simulation
ros2 launch tunnel_explorer_bringup stage0_simulation.launch.py headless:=True

# Terminal 2: Launch Frontier Explorer
ros2 launch tunnel_frontier_explorer frontier_explorer.launch.py

# Terminal 3: Monitor frontiers
ros2 topic echo /tunnel_frontier_explorer/frontier_markers
```

---

## Outcome Decision

| Result | Action | Current Status |
|--------|--------|----------------|
| Stage 0A PASS, Stage 0B-1 PASS | Proceed to Stage 1C (Frontier Explorer) with WSL2 | — |
| Stage 0A PASS, Stage 0B-1 FAIL (known-free goals also fail) | Diagnose Nav2 — Stage 0B-D | **ACTIVE** — 4/11 = 36.4%, DWB fail at >18° turn |
| Stage 0A H-criterion fails | Switch to native Ubuntu 24.04 | — |

### Diagnosis Path (Stage 0B-D)

Stage 0B-D is a controlled diagnostic experiment triggered by Stage 0B-1
failure. It does **not** modify Nav2 parameters — it collects evidence:

1. Run 4-goal minimal reproduction (all previously passing)
2. RViz costmap inspection at the turn trigger point (3.3, 0.3)
3. Spin action test to distinguish costmap blocking from DWB path tracking
4. Rosbag recording for offline analysis
5. Parameter snapshot for baseline documentation

### Outcome After Diagnosis

| Diagnosis Result | Next Step |
|-----------------|-----------|
| Spin succeeds, costmap clean | Evaluate RotationShimController wrapping DWB |
| Spin succeeds, costmap shows inflation blocking | Reduce inflation_radius or cost_scaling_factor |
| Spin returns COLLISION_AHEAD | Fix footprint clearing or track_unknown_space |
| Spin succeeds, costmap shows unknown blocking | Set local costmap track_unknown_space: false explicitly |

Stage 0B-2 (Online SLAM and Frontier Explorer handover) is evaluated
during Stage 1C simulation integration, not as a standalone test.

## Fallback: Native Ubuntu 24.04

If WSL2 does not pass, set up native Ubuntu 24.04 (dual-boot or dedicated machine).
Keep the WSL2 failure report as evidence of environment diagnosis.

## Report Template

After the run, fill in `benchmark_tools/reports/stage0_wsl2_report.md` with:

- Date, machine specs (CPU, RAM, GPU if any)
- WSL2 kernel version (`wsl.exe --version`)
- Gazebo version (`gz sim --version`)
- ROS2 distro and key package versions
- Actual metrics from `stage0_metrics.json`
- Navigation goal success/failure log from `stage0b_results.json`
- Screenshots of RViz during the run
- Decision: PASS / PASS_WITH_CAVEATS / FAIL → switch to native
