# ros2_tunnel_explorer

面向隧道巡检场景的 ROS2 自主探索与风险感知路径规划系统。

## Status

**Stage 0** — WSL2 environment feasibility verification.  
The base simulation stack (TurtleBot3 + Gazebo Harmonic + Nav2 + SLAM Toolbox) is being validated.

- Stage 0A (Environment Stability): PASS
- Stage 0B (Navigation Functionality): PENDING

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
colcon build --packages-select tunnel_explorer_bringup benchmark_tools
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

## Packages

| Package | Language | Description |
|---------|----------|-------------|
| `tunnel_explorer_bringup` | Python/YAML | Launch files, configs, RViz views |
| `benchmark_tools` | Python | Metrics recording, analysis, plotting |
| `tunnel_frontier_explorer` | C++ | Frontier-based autonomous exploration (Stage 2) |
| `tunnel_centerline_extractor` | C++ | Tunnel centerline distance field extraction (Stage 4) |
| `tunnel_aware_planner` | C++ | Tunnel-aware global planner plugin for Nav2 (Stage 5) |
| `tunnel_worlds` | Python/SDF | Parametric tunnel world generator (Stage 3) |

## Documentation

- [Architecture](docs/architecture.md) (Stage 1+)
- [Environment Feasibility](docs/environment_feasibility.md)
- [Algorithm Design](docs/algorithm.md) (Stage 2+)
- [Benchmark Methodology](docs/benchmark.md) (Stage 3+)
- [ROS2 Jazzy Compatibility](docs/jazzy_compatibility.md)

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
3. **Verify topics**: After launch, confirm `/clock`, `/scan`, and `/map`
   are publishing before starting metrics recording.
4. **Monitor DDS**: If you see `Failed init_port fastrtps_port7000:
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
