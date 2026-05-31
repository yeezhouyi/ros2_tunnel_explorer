# Stage 0B Attempt 1: Online SLAM Rectangle Navigation — FAIL

## Test Info

- **Date**: 2026-05-31
- **Goals file**: `benchmark_tools/config/stage0b_goals.yaml` (counterclockwise rectangle)
- **SLAM**: SLAM Toolbox (online async)
- **Navigation**: Nav2 (NavFn + DWB) with `use_composition:=False`
- **Collision monitor**: `PolygonStop` disabled

## Per-Goal Results

| # | Description | Position | Yaw | Result | Duration |
|---|-------------|----------|-----|--------|----------|
| 1 | East 1m | (1.0, 0.0) | 0° | SUCCEEDED | 3.7s |
| 2 | East 2m | (2.0, 0.0) | 0° | SUCCEEDED | 8.6s |
| 3 | East 3m | (3.0, 0.0) | 0° | SUCCEEDED | 14.6s |
| 4 | North 1m | (3.0, 1.0) | 90° | ABORTED | 6.0s |
| 5 | North 2m | (3.0, 2.0) | 90° | ABORTED | 1.2s |
| 6 | West 1m | (2.0, 2.0) | 180° | timed out | — |
| 7–10 | (remaining) | — | — | not run | — |

**Success rate**: 3/10 = 30% (threshold: ≥90%)

## Root Cause Hypothesis (unconfirmed)

The first 90° left turn at (3.0, 0.0) triggers consistent DWB controller failures:

```
[DWBLocalPlanner]: No valid trajectories out of 200!
[DWBLocalPlanner]: 1.00: BaseObstacle/Trajectory Hits Obstacle.
```

Three possible explanations (not yet distinguished):

1. **Unknown space blocking**: The area north of (3.0, 0.0) has not been fully observed by the laser during the eastward traversal. The local costmap may treat unknown cells as lethal or inflated, causing all 200 sampled trajectories to collide with "obstacles" that are actually unmapped space.

2. **Inflation layer interference**: The robot's footprint during rotation may sweep into the inflation zone of walls or other obstacles already marked in the costmap. With `inflation_radius: 0.55` and `cost_scaling_factor: 8.0`, the inflated region may block the turning maneuver.

3. **Goal in unmapped area**: Target (3.0, 1.0) may itself fall in unknown or lethal costmap cells, causing the global planner to produce a path that the DWB cannot track.

### Need for Diagnosis

Visual confirmation via RViz is required:

- `/local_costmap/costmap` — check whether the area north of (3, 0) is gray (unknown), red (lethal), or green (free) when the turn is attempted.
- `/local_costmap/published_footprint` — verify the robot footprint does not intersect inflated walls.
- Check actual topic names with `ros2 topic list | grep costmap` and `ros2 topic list | grep footprint`.

A rosbag recording is recommended:

```bash
ros2 bag record \
  /map /scan /tf /tf_static /odom /cmd_vel \
  /local_costmap/costmap_raw /global_costmap/costmap_raw \
  /local_costmap/published_footprint \
  -o ~/stage0b_results/turn_failure_bag
```

## Context

This failure does **not** invalidate the WSL2 environment (Stage 0A remains PASS). It exposes a boundary condition between the DWB local controller and the online-SLAM mapping scenario:

- Straight-line navigation (goals 1–3) works reliably.
- The `/clock`, `/scan`, and RTF metrics were stable throughout the run.
- The failure is in navigation strategy, not environment stability.

## Next Steps

1. **Diagnose** the local costmap state at the turn failure point.
2. **Split** Stage 0B into 0B-1 (known free area navigation) and 0B-2 (online-SLAM frontier handover).
3. **Design** known-free-space goals with progressive turns for Stage 0B-1 validation.
4. **Proceed** to Stage 1C (Frontier Explorer integration) once Stage 0B-1 passes.
