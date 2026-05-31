# Stage 1C: Frontier Explorer Smoke Test Results

- **Date**: 2026-05-31
- **Run ID**: 001
- **Duration**: ~5 minutes (18 navigation goals)
- **Simulation**: Stage 0 (TB3 world, RotationShimController + DWB)
- **Explorer**: `tunnel_frontier_explorer` with `min_goal_distance_meters: 0.50`

## Executive Summary

PASS — all verification criteria met. The FrontierGoalSelector `min_goal_distance`
filter actively prevented the infinite-loop condition (goal within
`xy_goal_tolerance`) in 12/18 goal selections, where the cluster centroid was
below the 0.50 m threshold but an alternative free cell was available.

## Verification Criteria

| Check | Result | Evidence |
|-------|--------|----------|
| `/map` reception | **PASS** | Continuous SLAM map updates; map grew from 112×102 to 112×103 |
| Frontier clusters | **PASS** | Detected every cycle (12–402 cells per cluster) |
| `representative_cell` in free space | **PASS** | All 18 navigation goals accepted and reached by Nav2 |
| `min_goal_distance_meters` filter | **PASS** | All 18 goals ≥ 0.50 m from robot (min=0.50, max=2.70) |
| Alternative cell search | **PASS** | 12/18 goals had centroid < 0.50 m; FrontierGoalSelector found alternative cell ≥ 0.50 m |
| State machine | **PASS** | No duplicate goals while in NAVIGATING state |
| Distinct frontiers visited | **PASS** | 12 unique goal locations |
| Blacklist | N/A | No timed-out or aborted goals (100 % navigation success) |
| Recovery | N/A | No failures to recover from |
| RViz markers | **PASS** | Frontier clusters + selected goal published every cycle |
| Node stability | **PASS** | Zero crashes; node ran continuously for 5+ minutes |
| Nav2 lifecycle | **PASS** | All core nodes (`planner_server`, `controller_server`, `bt_navigator`, `behavior_server`, `slam_toolbox`) remained `active [3]` throughout |
| Robot movement | **PASS** | Non-zero `/cmd_vel` observed; robot moved from near (0, 0) to (3.88, 0.40) |

## Detailed Metrics

### Navigation Goals

| # | Goal (x, y) | Cells | Centroid Dist. | Goal Dist. | Result |
|---|-------------|-------|----------------|------------|--------|
| 1 | (0.44, 0.32) | 336 | 0.4 m | 0.5 m | Succeeded |
| 2 | (0.74, 0.42) | 336 | 0.0 m | 0.5 m | Succeeded |
| 3 | (0.34, 0.72) | 336 | 0.3 m | 0.5 m | Succeeded |
| 4 | (0.39, -0.28) | 336 | 0.3 m | 0.8 m | Succeeded |
| 5 | (0.49, 0.42) | 336 | 0.3 m | 0.5 m | Succeeded |
| 6 | (0.39, -0.28) | 336 | 0.2 m | 0.5 m | Succeeded |
| 7 | (0.44, 0.42) | 336 | 0.3 m | 0.5 m | Succeeded |
| 8 | (0.39, -0.28) | 336 | 0.1 m | 0.5 m | Succeeded |
| 9 | (0.44, 0.42) | 336 | 0.3 m | 0.5 m | Succeeded |
| 10 | (0.39, -0.28) | 336 | 0.1 m | 0.5 m | Succeeded |
| 11 | (0.44, 0.42) | 336 | 0.3 m | 0.5 m | Succeeded |
| 12 | (0.34, 0.72) | 336 | 0.1 m | 0.5 m | Succeeded |
| 13 | (1.34, 1.42) | — | 1.4 m | 1.4 m | Succeeded |
| 14 | (0.98, 2.47) | 36 | 1.2 m | 1.2 m | Succeeded |
| 15 | (2.28, 1.55) | 20 | 1.5 m | 1.6 m | Succeeded |
| 16 | (3.88, 0.40) | 402 | 2.0 m | 2.0 m | Succeeded |
| 17 | (3.63, -0.95) | 82 | 1.6 m | 1.6 m | Succeeded |
| 18 | (0.93, -0.80) | 12 | 2.7 m | 2.7 m | Succeeded |

### Filter Activation

- **12/18** goals had `centroid_distance_to_robot < 0.50 m` but
  `goal_distance_to_robot ≥ 0.50 m`
- The FrontierGoalSelector searched each cluster for cells at least 0.50 m from
  the robot and selected the closest qualifying cell to the cluster centroid
- **0/18** violations (no goal below 0.50 m)

### Goal Distance Distribution

| Distance | Count |
|----------|-------|
| 0.5 m | 11 |
| 0.8 m | 1 |
| 1.2 m | 1 |
| 1.4 m | 1 |
| 1.6 m | 2 |
| 2.0 m | 1 |
| 2.7 m | 1 |

### Centroid Distance Distribution

| Distance | Count |
|----------|-------|
| 0.0 m | 1 |
| 0.1 m | 3 |
| 0.2 m | 1 |
| 0.3 m | 6 |
| 0.4 m | 1 |
| 1.2 m | 1 |
| 1.4 m | 1 |
| 1.5 m | 1 |
| 1.6 m | 1 |
| 2.0 m | 1 |
| 2.7 m | 1 |

## Observations

### 1. FrontierGoalSelector Prevents Instant-Success Loop

Goals 1–12 cycled within an initial bounded room where centroid distances were
all ≤ 0.4 m. Without the `min_goal_distance_meters: 0.50` filter, the
representative cell would have been ~0.1–0.4 m from the robot — well within
`xy_goal_tolerance: 0.25 m` — causing instant navigation success without
movement. The filter forced an alternative cell search within each cluster,
producing `goal_distance_to_robot ≥ 0.50 m` for every dispatched goal.

The most extreme case was Goal 2: `centroid=0.0 m` (centroid at robot
position), but `goal=0.5 m` (alternative cell found 0.5 m away).

### 2. Exploration Breakout

Goals 1–12 explored the initial bounded room (cluster size consistently 336
cells). After thorough exploration, the robot found the exit to a larger area
(Goals 13–18). Cluster sizes dropped from 336 to 12–402 cells as the map
expanded, and centroid distances grew from 0.1–0.4 m to 1.2–2.7 m.

### 3. Nearest-Frontier Cycling

Within the bounded room (Goals 1–12), the nearest-frontier baseline exhibited
cycling between 3–4 goal locations. This is expected behavior for a bounded
environment with persistent boundary frontiers and a 60 s blacklist timeout.
The robot eventually broke out of the cycle when SLAM expanded the map
boundary.

This cycling is addressed in Stage 2+ (information gain scoring, tunnel
centerline extraction, back-turning penalty).

### 4. Stability

- 100 % navigation success rate (18/18)
- Zero node crashes
- Zero Nav2 lifecycle failures
- All lifecycle nodes active throughout the 5+ minute run

### 5. WSL2 Startup Timing

The `wait_for_nav2_active.sh` script's 60 s timeout was insufficient for WSL2
DDS discovery. With 90 s startup delay, all lifecycle nodes were confirmed
active. The script timeout should be increased for WSL2 environments.

## Issues Found

### Minor: wait_for_nav2_active.sh timeout too short for WSL2

The 60 s timeout in `scripts/wait_for_nav2_active.sh` expired before the ROS2
daemon discovered all lifecycle nodes, even though the nodes were eventually
reported as `active [3]`. Recommend increasing to 120 s for WSL2, or adding a
retry loop.

**Fix**: Not applied (out of scope for Stage 1C). Tracked for documentation
update.

## Conclusion

**Stage 1C baseline verification PASS.** The nearest-frontier exploration
pipeline is operational:

1. Map received from SLAM ✓
2. Frontier clusters detected ✓
3. `FrontierGoalSelector` applies `min_goal_distance` filter ✓
4. Alternative cells selected when centroid too close ✓
5. NavigateToPose goal sent to Nav2 ✓
6. Robot moves to frontier ✓
7. SLAM updates map ✓
8. Cycle continues without crashes ✓

The system is ready for Stage 2A benchmarking.

## Raw Data

Log file: `/tmp/frontier_explorer.log`
