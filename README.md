# ros2_tunnel_explorer

面向隧道巡检场景的 ROS2 自主探索与风险感知路径规划系统。

## Status

**Stage 0** — WSL2 environment feasibility verification.  
The base simulation stack (TurtleBot3 + Gazebo Harmonic + Nav2 + SLAM Toolbox) is being validated.

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

### 1. Launch simulation

```bash
ros2 launch tunnel_explorer_bringup stage0_simulation.launch.py
```

This starts:
- TurtleBot3 in Gazebo Harmonic
- SLAM Toolbox (online async mapping)
- Nav2 navigation stack
- RViz2 with pre-configured view

### 2. Record metrics

In a second terminal:

```bash
ros2 run benchmark_tools record_stage0_metrics.py \
    --duration 600 \
    --output-dir /tmp/stage0_results
```

### 3. Send navigation goals

Use RViz `2D Goal Pose` tool to send goals. Observe navigation performance.

### 4. Review results

```bash
cat /tmp/stage0_results/stage0_metrics.json
cat /tmp/stage0_results/stage0_metrics.md
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

## License

Apache-2.0
