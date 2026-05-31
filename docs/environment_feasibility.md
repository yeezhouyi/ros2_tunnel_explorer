# Stage 0: WSL2 Environment Feasibility Verification

## Overview

Stage 0 is split into two sub-stages:

- **Stage 0A — Environment Stability**: Verify that the simulation stack (TurtleBot3 +
  Gazebo Harmonic + Nav2 + SLAM Toolbox) runs stably under WSL2 for 10 continuous
  minutes without crashes, clock reversals, or sustained sensor gaps.
- **Stage 0B — Navigation Functionality**: Verify that the Nav2 navigation stack can
  execute simple NavigateToPose goals with >= 90 % success rate in the default
  sandbox world.

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

## Stage 0B: Navigation Functional Verification

### Acceptance Criteria

| # | Criterion | Threshold | Measurement Method |
|---|-----------|-----------|-------------------|
| N1 | NavigateToPose success rate | >= 90 % | `run_navigation_smoke_test.py` — `summary.success_rate` |
| N2 | No sustained TF timeout during navigation | No continuous TF warnings in log | Manual log inspection |
| N3 | No sustained costmap timeout during navigation | No continuous costmap warnings in log | Manual log inspection |
| N4 | Motion `/scan` steady-state frequency | >= 4.5 Hz | Simultaneous `record_stage0_metrics.py` run |
| N5 | Motion steady-state RTF | >= 0.8 | Simultaneous `record_stage0_metrics.py` run |

### Procedure

1. Complete Stage 0A first (simulation running with SLAM and Nav2 active).
2. Run the automated smoke test:

```bash
# Terminal 3 (while simulation is running):
ros2 run benchmark_tools run_navigation_smoke_test.py \
    --goals src/benchmark_tools/config/stage0b_goals.yaml \
    --output-dir /tmp/stage0b_results
```

3. Alternatively, send goals interactively via RViz `2D Goal Pose`.

The smoke test sends up to 10 goals from a YAML config file sequentially,
waiting for each to complete before sending the next. Results are written
as JSON and Markdown reports.

**Important**: The example goal coordinates in `stage0b_goals.yaml` are for the
`nav2_minimal_tb3_sim` sandbox world. Adjust them to match your actual SLAM map
before running.

### Review Results

```bash
cat /tmp/stage0b_results/stage0b_results.json
cat /tmp/stage0b_results/stage0b_results.md
```

---

## Outcome Decision

| Result | Action |
|--------|--------|
| Stage 0A PASS, Stage 0B PASS | Proceed to Stage 1 with WSL2 |
| Stage 0A PASS, Stage 0B marginal (>= 7/10) | Proceed with documented caveats |
| Stage 0A PASS, Stage 0B < 7/10 | Debug Nav2 configuration, re-run 0B |
| Stage 0A H-criterion fails | Switch to native Ubuntu 24.04 |

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
