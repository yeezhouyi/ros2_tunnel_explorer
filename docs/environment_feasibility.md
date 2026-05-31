# Stage 0: WSL2 Environment Feasibility Verification

## Purpose

Before developing any tunnel-specific navigation code, we must verify that the base simulation
stack (TurtleBot3 + Gazebo Harmonic + Nav2 + SLAM Toolbox) runs stably under WSL2 for at least
10 minutes of continuous operation.

## Test Infrastructure

- **OS**: Ubuntu 24.04 (Noble) under WSL2 on Windows 11
- **ROS2**: Jazzy Jalisco
- **Simulator**: Gazebo Harmonic (gz-sim8)
- **Robot**: TurtleBot3 (via nav2_minimal_tb3_sim)
- **SLAM**: SLAM Toolbox (online async mode)
- **Navigation**: Nav2 (NavFn planner + DWB controller)

## Acceptance Criteria

> **Note**: These thresholds are engineering acceptance criteria defined for this
> project. They are NOT official Gazebo, Nav2, or ROS2 standards. The values
> were chosen to ensure the WSL2 simulation environment is stable enough for
> frontier exploration algorithm development in later stages.

### Hard Requirements (all must pass)

| # | Criterion | Measurement Method |
|---|-----------|-------------------|
| H1 | `/clock` is strictly monotonic (no time reversals) | `record_stage0_metrics.py` — `clock_monotonicity.strictly_monotonic` |
| H2 | No continuous TF timeouts (> 5s gaps) | `ros2 run tf2_tools view_frames` after run |
| H3 | Simulation runs for >= 10 min without Gazebo or RViz crash | Manual observation + process exit codes |

### Soft Thresholds

Soft thresholds are evaluated against the **steady-state** phase only
(excluding the first `--warmup-seconds` of recording, default 30 s).
The `full` phase numbers are reported for reference but the pass/fail
decision uses the `steady` phase.

| # | Criterion | Threshold | Measurement Method |
|---|-----------|-----------|-------------------|
| S1 | Steady-state Real-Time Factor (RTF) median | >= 0.8 | `record_stage0_metrics.py` — `real_time_factor.steady.rtf` |
| S2 | `/scan` steady-state average frequency | >= 90% of expected (5 Hz nominal → >= 4.5 Hz) | `record_stage0_metrics.py` — `topics.scan.steady.mean_hz` |
| S3 | `/scan` steady-state gaps > 1.0 s | 0 gaps in steady phase | `record_stage0_metrics.py` — `topics.scan.steady.gaps.count` |
| S4 | `/map` updates arrive within update_interval + 5 s | No gaps > 10 s in any phase | `record_stage0_metrics.py` — `topics.map.*.gaps` |
| S5 | Multi-goal navigation success rate | >= 9/10 simple goals | Manual testing via RViz `2D Goal Pose` |

### Multi-Goal Navigation Test

Send 10 navigation goals to different reachable positions in the default world:

```bash
# Example: use ros2 action to send goals programmatically
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {...}}"
```

Or use RViz's `2D Goal Pose` tool interactively.

Record:
- Number of goals reached (status = SUCCEEDED)
- Number of failures (ABORTED / CANCELED)
- Any recovery behaviors triggered

## Procedure

### 1. Launch the simulation

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch tunnel_explorer_bringup stage0_simulation.launch.py
```

### 2. Start metrics recording

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run benchmark_tools record_stage0_metrics.py \
    --duration 600 \
    --warmup-seconds 30 \
    --output-dir /tmp/stage0_results
```

### 3. Monitor during the run

```bash
# Topic frequencies
ros2 topic hz /clock
ros2 topic hz /scan
ros2 topic hz /map

# TF tree health
ros2 run tf2_tools view_frames
```

### 4. Send navigation goals

Use RViz `2D Goal Pose` tool to send at least 10 goals to different
reachable positions. Observe whether Nav2 completes each goal.

### 5. Review results

```bash
cat /tmp/stage0_results/stage0_metrics.json
cat /tmp/stage0_results/stage0_metrics.md
```

## Outcome Decision

| Result | Action |
|--------|--------|
| All H1-H3 pass, S1-S5 pass | Proceed to Stage 1 with WSL2 |
| H1-H3 pass, some S fail but marginal | Proceed with documented caveats |
| H1-H3 pass, S1 < 0.5 or S5 < 7/10 | Switch to native Ubuntu 24.04 |
| Any H criterion fails | Switch to native Ubuntu 24.04 |

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
- Navigation goal success/failure log
- Screenshots of RViz during the run
- Decision: PASS / PASS_WITH_CAVEATS / FAIL → switch to native
