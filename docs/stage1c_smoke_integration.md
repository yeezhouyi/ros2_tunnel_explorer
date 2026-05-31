# Stage 1C: Frontier Explorer Simulation Integration

## Status

| Item | Status |
|------|--------|
| Stage 1C | **READY** |
| Nav2 baseline | **PASS** (Stage 0B-1: RotationShimController + DWB, 60s timeout, 10/11 = 90.9%) |
| Latency Optimization | **OPEN** (cooldown reduction after follow_path handover is hardened) |

## Objective

Verify that the Frontier Explorer (`tunnel_frontier_explorer`) completes the
online closed loop:

```
Detect frontier clusters
→ Select nearest non-blacklisted cluster
→ Compute representative_cell (guaranteed free space)
→ Send NavigateToPose via Nav2
→ RotationShim handles heading change if needed
→ DWB tracks path
→ SLAM updates map
→ Repeat until no frontiers remain
```

## NOT in Scope

- Formal benchmark (repeat 3×, min/max/median) — deferred to Stage 2A
- Tunnel centerline extraction — Stage 2+
- Information gain scoring — Stage 2+
- DWB parameter tuning — Stage 2+
- RotationShim threshold tuning — Stage 2+
- Costmap parameter tuning — Stage 2+

## Configuration

### Control Pipeline

| Parameter | Value | Source |
|-----------|-------|--------|
| Controller plugin | `RotationShimController` → `dwb_core::DWBLocalPlanner` | `nav2_params_rotation_shim.yaml` |
| Goal timeout | 60.0 s | `frontier_explorer_params.yaml` |
| Goal cooldown | 5.0 s | `frontier_explorer_params.yaml` |
| Blacklist radius | 0.5 m | `frontier_explorer_params.yaml` |
| Blacklist timeout | 60.0 s | `frontier_explorer_params.yaml` |
| Min cluster size | 10 cells | `frontier_explorer_params.yaml` |

### Frontier Explorer Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `exploration_period_seconds` | 1.0 | Main loop interval (s) |
| `cooldown_seconds` | 5.0 | Pause between navigation goals (s) |
| `goal_timeout_seconds` | 60.0 | Single-goal timeout; triggers cancel + blacklist (s) |
| `min_cluster_size` | 10 | Minimum cells for a cluster |
| `frontier_neighbor_connectivity` | 4 | Neighbourhood for frontier detection (4 or 8) |
| `cluster_connectivity` | 8 | Neighbourhood for BFS clustering (4 or 8) |
| `blacklist_radius_meters` | 0.5 | Radius to blacklist around a failed goal (m) |
| `blacklist_timeout_seconds` | 60.0 | Blacklist expiry (s) |
| `orient_goal_toward_frontier` | true | Face robot towards goal yaw |

## Launch Procedure

### Prerequisites

1. ROS2 Jazzy workspace built:
   ```bash
   cd ~/ros2_tunnel_explorer
   colcon build --packages-select tunnel_explorer_bringup benchmark_tools tunnel_frontier_explorer
   ```

2. All previous stages verified:
   - Stage 0A: Environment stability PASS
   - Stage 0B-1: Navigation PASS (RotationShim + 60s, >= 90 %)
   - Stage 1A: Frontier algorithms unit tests PASS
   - Stage 1B: ROS2 node build PASS

### Terminal 1: Launch Simulation with RotationShim

```bash
cd ~/ros2_tunnel_explorer

./scripts/cleanup_simulation.sh

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch tunnel_explorer_bringup \
  stage0_simulation.launch.py \
  headless:=True \
  use_composition:=False \
  params_file:="$(ros2 pkg prefix --share tunnel_explorer_bringup)/config/nav2_params_rotation_shim.yaml"
```

> **Note**: `use_composition:=False` is recommended for WSL2 to avoid
> composed-node lifecycle race conditions.

### Terminal 2: Wait for Nav2 (or use the smoke script)

```bash
cd ~/ros2_tunnel_explorer

source /opt/ros/jazzy/setup.bash
source install/setup.bash

./scripts/wait_for_nav2_active.sh
```

Expected output:

```
All Nav2 lifecycle nodes are active.
```

### Terminal 3: Launch Frontier Explorer

```bash
cd ~/ros2_tunnel_explorer

source /opt/ros/jazzy/setup.bash
source install/setup.bash

mkdir -p ~/stage1c_smoke

ros2 launch tunnel_frontier_explorer \
  frontier_explorer.launch.py \
  2>&1 | tee ~/stage1c_smoke/frontier_explorer.log
```

### Terminal 4 (optional): Record Bag

```bash
source /opt/ros/jazzy/setup.bash

ros2 bag record \
  /map \
  /scan \
  /tf \
  /tf_static \
  /odom \
  /cmd_vel \
  /local_costmap/costmap_raw \
  /global_costmap/costmap_raw \
  /local_costmap/published_footprint \
  /tunnel_frontier_explorer/frontier_markers \
  -o ~/stage1c_smoke/frontier_smoke_bag
```

### Automated Smoke Script

```bash
cd ~/ros2_tunnel_explorer
./scripts/run_stage1c_smoke.sh ~/stage1c_smoke
```

This waits for Nav2, starts the Frontier Explorer, and prints the bag
record command.

## RViz Visualization

Add these displays in RViz:

| Topic | Type | Purpose |
|-------|------|---------|
| `/map` | Map | SLAM occupancy grid |
| `/local_costmap/costmap_raw` | Map | Local costmap with inflation |
| `/global_costmap/costmap_raw` | Map | Global costmap with inflation |
| `/local_costmap/published_footprint` | Polygon | Robot footprint |
| `/tunnel_frontier_explorer/frontier_markers` | MarkerArray | Frontier centroids, selected goal, blacklisted |

### Marker Namespaces

| Namespace | Color | Shape | Description |
|-----------|-------|-------|-------------|
| `frontier_clusters` | Green | Points | Centroids of all frontier clusters |
| `selected_goal` | Red | Sphere | Current NavigateToPose target |
| `blacklisted` | Grey | Spheres | Temporarily forbidden failed goals |

## Verification Criteria (5–10 min run)

| Check | Acceptance |
|-------|------------|
| `/map` reception | Continuous updates from SLAM |
| Frontier clusters | Detected and published each cycle |
| `representative_cell` | Always falls on a known free cell |
| State machine | No duplicate goals while `NAVIGATING` |
| Consecutive frontiers | At least 5 distinct frontiers visited |
| Blacklist | Timed-out / aborted goals blacklisted by position |
| Recovery | Failed goal does not block exploration permanently |
| Markers | Clusters, goal, blacklist visible in RViz |
| Node stability | No node crashes or permanent hangs |
| Nav2 lifecycle | Core nodes remain `active` throughout |

## Not Required for Stage 1C

- Map coverage >= 90 % — Stage 2A benchmark
- Shortest path — not a goal of nearest-frontier baseline
- 11/11 fixed waypoint success — Stage 0B-1 verifies this separately
- Zero DWB warnings — warnings are expected in unexplored corridors

## State Machine

```
WAITING_FOR_MAP → WAITING_FOR_NAV2 → IDLE → NAVIGATING → COOLDOWN → IDLE
                     ↑                  ↓                      |
                     |              COMPLETED ←────────────────┘
                     └──── map received (if COMPLETED)
```

- **WAITING_FOR_MAP**: No map received yet. Silently waiting.
- **WAITING_FOR_NAV2**: Map received, checking for Nav2 action server.
- **IDLE**: Detect frontiers, select nearest non-blacklisted, send goal.
- **NAVIGATING**: Goal sent, awaiting result. Timeout after
  `goal_timeout_seconds` → cancel + blacklist + cooldown.
- **COOLDOWN**: Brief pause (`cooldown_seconds`) before next IDLE cycle.
- **COMPLETED**: No frontiers for `k_max_empty_cycles` (10) consecutive
  cycles. New map data re-activates IDLE.

## Known Issues

### WSL2 Launch Race Condition

Under WSL2, the SLAM Toolbox lifecycle may fail to configure on the first
launch attempt:

```
Failed to make transition 'TRANSITION_CONFIGURE' for LifecycleNode '/slam_toolbox'
```

This is a timing race: the lifecycle event manager sends CONFIGURE before
slam_toolbox's rclcpp node is fully initialized.

**Workaround**: Thorough process cleanup and retry:

```bash
./scripts/cleanup_simulation.sh
sleep 5
# Re-launch
```

The cleanup script kills stale `gz sim` processes, stops the ROS2 daemon,
and removes Fast DDS shared-memory files from `/dev/shm`.

**Do not** blindly adjust `bond_timeout` or lifecycle transition timeouts
until the race condition is confirmed to be a server crash rather than a
startup ordering issue.

### RotationShimController Observability

The RotationShimController does not emit INFO-level log messages on per-goal
activation. Shim triggering must be inferred from:
- Reduced "No valid trajectories" count vs DWB-only baseline
- Robot rotation observed in `/cmd_vel` or RViz before forward motion

## Next Steps After Stage 1C

1. **Stage 2A: Nearest-Frontier Baseline Benchmark**
   - Repeat the smoke test 3× to collect min/max/median metrics
   - Measure: coverage time, distance, CPU, memory, shim triggers, DWB errors
2. **Stage 2B: Tunnel-Adapted Exploration** (tunnel centreline, dead-end
   detection, information gain)
3. **Latency Optimization**: Reduce `cooldown_seconds` from 5.0 s after
   `follow_path` handover stability is confirmed
