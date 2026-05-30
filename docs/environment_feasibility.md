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

These are engineering thresholds defined for this project — they are NOT official
Gazebo or Nav2 standards.

### Hard Requirements (all must pass)

| # | Criterion | Measurement Method |
|---|-----------|-------------------|
| H1 | `/clock` is strictly monotonic (no time reversals) | `record_stage0_metrics.py` — monotonicity check |
| H2 | No continuous TF timeouts (> 5s gaps) | `ros2 run tf2_tools view_frames` after run |
| H3 | Simulation runs for >= 10 min without Gazebo or RViz crash | Manual observation + process exit codes |

### Soft Thresholds (median across run)

| # | Criterion | Threshold | Measurement Method |
|---|-----------|-----------|-------------------|
| S1 | Simulation real-time factor (median) | >= 0.8 | `gz sim --verbose` or `/clock` interval analysis |
| S2 | `/scan` message reception rate | >= 90% of expected (10 Hz nominal → >= 9 Hz) | `record_stage0_metrics.py` |
| S3 | `/scan` no gaps > 1.0s | 0 gaps over full run | `record_stage0_metrics.py` |
| S4 | `/map` updates arrive within update_interval + 5s | No gaps > 10s | `record_stage0_metrics.py` |
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
