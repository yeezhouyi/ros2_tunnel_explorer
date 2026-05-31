# Stage 0B-D: DWB Turn Failure Diagnosis

## Purpose

Isolate the root cause of DWB "No valid trajectories" failure at heading
changes beyond approximately 18° during Stage 0B-1 known-free-space
navigation, and determine whether the fix is in costmap configuration,
controller selection, or DWB parameter tuning.

## Current Status

| Stage | Result |
|-------|--------|
| Stage 0A (environment stability) | PASS |
| Stage 0B-1 (known-free navigation) | FAIL — 4/11 = 36.4% (< 90% threshold) |
| Stage 0B-D (diagnosis) | IN PROGRESS |
| Stage 1A (frontier algorithms) | PASS |
| Stage 1B (ROS2 node build) | PASS |
| Stage 1C (simulation integration) | NOT STARTED — blocked on Stage 0B-1 |

### ⚠ Configuration Defect Found During Review

The Nav2 costmap configuration used an incorrect parameter name:

| Config File | Had | Should Be |
|-------------|-----|-----------|
| `nav2_params.yaml` local_costmap | `plugin_names:` | `plugins:` |
| `nav2_params.yaml` global_costmap | `plugin_names:` | `plugins:` |

This means the original experiment's costmap layer assumptions are **not
reliable**:

- The local costmap may have been running default layers (`static_layer +
  obstacle_layer + inflation_layer`) instead of the intended `voxel_layer +
  inflation_layer`.
- Any conclusions about voxel-layer-specific behaviour (footprint clearing,
  unknown threshold, 3D raycasting) cannot be drawn from the experimental
  data collected before this fix.
- The diagnosis procedure below now requires `dump_nav2_diagnostic_params.sh`
  to **confirm** which layers are actually loaded at runtime before
  interpreting any costmap observations.

**Reference**: Nav2 costmap configuration uses the `plugins:` key for layer
lists. See [Nav2 Costmap 2D documentation](https://docs.nav2.org/configuration/packages/configuring-costmaps.html).

### Local Costmap

| Parameter | Value |
|-----------|-------|
| `track_unknown_space` | **not set** (defaults to `false` in local) |
| `rolling_window` | `true` |
| `width` x `height` | 3 m x 3 m |
| `resolution` | 0.05 m |
| `robot_radius` | 0.13 m |
| `footprint` | **not set** (uses robot_radius) |
| `footprint_padding` | **not set** (default 0.01 m) |
| `plugins` | `[voxel_layer, inflation_layer]` |

Note: Prior to the fix, the config used `plugin_names:` instead of `plugins:`,
so the actual runtime layers for the original experiment are unknown.
`dump_nav2_diagnostic_params.sh` must confirm runtime values before further
diagnosis.

## Observed Behaviour

| Goal | Position | Yaw | Result | Note |
|------|----------|-----|--------|------|
| 1: East 1m | (1.0, 0.0) | 0° | SUCCEEDED 3.7s | Straight line — no issue |
| 2: East 2m | (2.0, 0.0) | 0° | SUCCEEDED 8.6s | Straight line — no issue |
| 3: East 3m | (3.0, 0.0) | 0° | SUCCEEDED 14.6s | Straight line — no issue |
| 4: NE shallow | (3.3, 0.3) | ~18° | SUCCEEDED 1.6s | First progressive turn — passes |
| 5: NE medium | (3.4, 0.7) | ~34° | ABORTED 28.5s | **First failure** — DWB no valid trajectories |
| 6–11 | various | 65°–180° | ABORTED 25s ea | Cascading — all subsequent fail |

### Key Log Pattern

```
[DWBLocalPlanner]: No valid trajectories out of 199!
   1.00: BaseObstacle/Trajectory Hits Obstacle
```

- 194 total "No valid trajectories" events in simulation log
- 0 "Failed to make progress" events (progress checker never triggered)
- After the first ABORT on goal 5, all 7 subsequent goals also ABORT at
  approximately 25s each — recovery behaviours do not restore navigation
  viability

## Hypotheses (Not Yet Distinguished)

### H1: Local costmap unknown region blocking

The area north of the eastward traversal path at (3.0–3.4, 0.3–0.7) may not
have been fully observed by the laser scan. If `track_unknown_space` is
enabled in the local costmap, unknown cells may be treated differently from
free cells. The DWB controller samples 200 trajectories; if all of them
intersect unknown cells scored as lethal or inflated by BaseObstacleCritic,
all are rejected.

**Check**: `track_unknown_space` value in local costmap configuration.
Visual inspection of `/local_costmap/costmap_raw` at the failure point.

### H2: Inflation layer covering turning sweep

With `inflation_radius: 0.55` and `cost_scaling_factor: 8.0`, the inflated
region around observed walls (along y=0, the south wall of the corridor) may
extend into the area the robot's footprint sweeps through during a turn.
The turtle's footprint radius is 0.13 m (robot_radius), but the inflation
radius is 4.2× larger.

**Check**: Whether the area north of the robot at (3.0–3.4, 0.3–0.7) is
coloured red (inflation) or green (free) in the costmap.

### H3: Footprint / footprint_padding too large

If `footprint_padding` adds extra clearance beyond the actual robot radius,
the effective footprint may extend into inflated zones during rotation.
Nav2 default footprint_padding is 0.01 m — small but worth verifying.

**Check**: `footprint` vs `robot_radius` vs `footprint_padding` values.

### H4: Stale obstacle under footprint

If `footprint_clearing_enabled` is false or the clearing mechanism is not
working, the voxel grid may retain occupied cells under the robot after it
has moved past an obstacle. This would make all sampled trajectories that
pass through the robot's own position score as hitting an obstacle.

**Check**: `footprint_clearing_enabled` value. Visual inspection of costmap
under the robot's current position in RViz.

### H5: DWB trajectory sampling insufficient for rotation

With `vtheta_samples: 20`, `sim_time: 1.7`, and `max_vel_theta: 1.0`, the
DWB may not sample aggressive enough rotational velocities to turn the
robot onto a new heading within the simulation horizon. The robot's
controller_frequency is 20 Hz, so each sim_time window covers ~34 control
cycles.

**Check**: Whether Spin action succeeds (isolates H5 from H1–H4).

## Current Parameter Values (nav2_params.yaml)

### Local Costmap

| Parameter | Value |
|-----------|-------|
| `track_unknown_space` | **not set** (defaults to `false` in local) |
| `rolling_window` | `true` |
| `width` x `height` | 3 m x 3 m |
| `resolution` | 0.05 m |
| `robot_radius` | 0.13 m |
| `footprint` | **not set** (uses robot_radius) |
| `footprint_padding` | **not set** (default 0.01 m) |
| `plugins` | `[voxel_layer, inflation_layer]` |

Note: The local costmap has `voxel_layer` (not `obstacle_layer`). VoxelLayer
also supports `footprint_clearing_enabled` and observation sources.

### Voxel Layer (local)

| Parameter | Value |
|-----------|-------|
| `footprint_clearing_enabled` | `true` |
| `observation_sources` | `scan` |
| `scan.clearing` | `true` |
| `scan.marking` | `true` |
| `obstacle_max_range` | 2.5 m |
| `raytrace_max_range` | 3.0 m |
| `unknown_threshold` | 15 |
| `mark_threshold` | 0 |

### Inflation Layer (local and global)

| Parameter | Value |
|-----------|-------|
| `inflation_radius` | 0.55 m |
| `cost_scaling_factor` | 8.0 |
| `inflate_unknown` | **not set** (defaults `false`) |
| `inflate_around_unknown` | **not set** (defaults `false`) |

### Global Costmap

| Parameter | Value |
|-----------|-------|
| `track_unknown_space` | `true` |
| `rolling_window` | `false` |
| `robot_radius` | 0.13 m |
| `plugins` | `[static_layer, obstacle_layer, inflation_layer]` |

### Obstacle Layer (global)

| Parameter | Value |
|-----------|-------|
| `footprint_clearing_enabled` | **not set** (defaults `true`) |
| `scan.clearing` | `true` |
| `scan.marking` | `true` |

### DWB Controller (FollowPath)

| Parameter | Value |
|-----------|-------|
| `debug_trajectory_details` | `true` |
| `vtheta_samples` | 20 |
| `sim_time` | 1.7 s |
| `max_vel_theta` | 1.0 rad/s |
| `min_speed_theta` | 0.0 |
| `acc_lim_theta` | 3.2 rad/s² |
| `dec_lim_theta` | -3.2 rad/s² |
| `linear_granularity` | 0.05 |
| `angular_granularity` | 0.025 |
| `vx_samples` | 10 |
| `sim_time` | 1.7 |
| Critics | BaseObstacle, GoalAlign, GoalDist, PathAlign, PathDist, PreferForward, Oscillation |
| `BaseObstacle.scale` | 0.02 |
| `PathAlign.scale` | 32.0 |
| `GoalAlign.scale` | 24.0 |

### Behaviour Server (Spin)

| Parameter | Value |
|-----------|-------|
| `simulate_ahead_time` | 2.0 s |
| `max_rotational_vel` | 1.0 rad/s |
| `min_rotational_vel` | 0.4 rad/s |
| `rotational_acc_lim` | 3.2 rad/s² |

### Collision Monitor

| Parameter | Value |
|-----------|-------|
| `PolygonStop.enabled` | `false` (disabled to allow movement) |
| `PolygonStop.points` | `[[0.3,0.3],[0.3,-0.3],[0.0,-0.3],[0.0,0.3]]` |

## Key Observation: Unknown Space Handling Asymmetry

The global and local costmaps have different unknown-space configurations:

| Costmap | `track_unknown_space` | Effect |
|---------|----------------------|--------|
| Global | `true` | Unknown areas treated as unknown (not free) |
| Local | default `false` | Unknown areas treated as free |

However, NavFn global planner has `allow_unknown: true`, meaning it can plan
paths through unknown space. But the DWB local controller's trajectory
evaluation depends on the local costmap, which may or may not penalise
trajectories passing through unknown cells depending on how
BaseObstacleCritic scores them.

**This asymmetry is a key diagnostic target**: if DWB treats unknown cells as
obstacle-free (because local costmap has `track_unknown_space: false`), then
the failures are NOT caused by unknown space blocking, and the root cause
lies elsewhere (inflation, stale obstacles, or DWB sampling).

## Diagnosis Procedure

### Step 1: Run minimal reproduction (4 goals)

```bash
# Terminal 1: Launch simulation (already running)
# Terminal 2: Record diagnostic rosbag BEFORE sending goals
mkdir -p ~/stage0b_diagnostic
ros2 bag record \
  /map /scan /tf /tf_static /odom /cmd_vel \
  /local_costmap/costmap_raw /global_costmap/costmap_raw \
  /local_costmap/published_footprint \
  -o ~/stage0b_diagnostic/dwb_turn_failure

# Terminal 3: Run smoke test (stops after goal 4)
ros2 run benchmark_tools run_navigation_smoke_test.py \
  --goals "$(ros2 pkg prefix --share benchmark_tools)/config/stage0b_turn_diagnostic_goals.yaml" \
  --goal-timeout-seconds 30 \
  --continue-on-failure true \
  --cooldown-seconds 1
```

### Step 2: Snapshot current parameters

```bash
./scripts/dump_nav2_diagnostic_params.sh ~/stage0b_diagnostic/params
```

### Step 3: RViz costmap inspection

Wait for the robot to arrive at (3.3, 0.3) after goal 4. Before sending
any further commands, inspect in RViz:

```text
/local_costmap/costmap_raw   — what colour dominates north of robot?
/global_costmap/costmap_raw  — what is the global costmap state?
/local_costmap/published_footprint — footprint position and orientation
```

Check actual topic names first:

```bash
ros2 topic list | grep -E 'costmap|footprint'
```

Inspect these specific questions:

1. Is the area immediately north of the robot (3.3–4.0, 0.3–1.5) gray
   (unknown), green/white (free), or red (lethal/inflated)?
2. Does the robot footprint intersect any red cells?
3. Is there any residual occupied cell under the robot's current position?
4. Is the inflation layer visibly extending beyond the known-free corridor?

### Step 4: Spin action test

After visual inspection, send a 90° left turn:

```bash
ros2 action send_goal \
  /spin nav2_msgs/action/Spin \
  "{target_yaw: 1.5708, time_allowance: {sec: 10, nanosec: 0}}" \
  --feedback
```

Expected outcomes:

| Result | Interpretation |
|--------|---------------|
| SUCCEEDED | Robot can rotate in place — problem is DWB path-tracking, not costmap blocking |
| COLLISION_AHEAD | Costmap or footprint issue — check inflation, clearing, track_unknown_space |
| TIMEOUT | Spin behaviour started but couldn't complete in 10 s |

### Step 5: Stop rosbag

```bash
# Press Ctrl+C in the ros2 bag record terminal
```

## Decision Tree After Diagnosis

```
Spin SUCCEEDED?
├── Yes → Robot can rotate in place safely
│   ├── RViz costmap looks clean (green/white around robot)
│   │   └── Problem is DWB path-tracking at heading changes
│   │       → Evaluate RotationShimController as wrapper
│   │       → Parameter series: debug_trajectory_details, vtheta_samples, sim_time
│   └── RViz costmap shows inflation/unknown around robot
│       └── Problem is costmap parameterisation
│           → Check: inflation_radius, cost_scaling_factor
│           → Check: footprint_padding
│           → Check: track_unknown_space in local costmap
│
└── No (COLLISION_AHEAD)
    └── Costmap or footprint configuration issue
        → Check footprint_clearing_enabled (voxel layer)
        → Check inflate_unknown / inflate_around_unknown
        → Check footprint_padding
        → Check footprint vs robot_radius consistency
        → Check obstacle_range vs raytrace_range balance
```

## Parameter Evaluation Order (If Needed)

| Experiment | Change | Hypothesis Tested |
|------------|--------|------------------|
| 0 | Baseline (current config) | — |
| 1 | Set `track_unknown_space: false` explicitly in local costmap | H1: unknown blocking |
| 2 | Reduce `inflation_radius` from 0.55 to 0.35 | H2: inflation overreach |
| 3 | Add `RotationShimController` wrapping DWB | H5: DWB path tracking |
| 4 | Increase `vtheta_samples` (20→40), `sim_time` (1.7→2.5) | H5: sampling coverage |
| 5 | Set `inflate_unknown: false`, `inflate_around_unknown: false` | H1/H2 combined |

**Rule**: Change exactly one variable per experiment. Record all results
before moving to the next experiment. Do not batch changes.

## Files Referenced

- `benchmark_tools/config/stage0b_turn_diagnostic_goals.yaml` — 4-goal reproduction set
- `benchmark_tools/config/stage0b_known_free_goals.yaml` — original 11-goal set
- `tunnel_explorer_bringup/config/nav2_params.yaml` — Nav2 configuration
- `scripts/dump_nav2_diagnostic_params.sh` — parameter snapshot tool
- `docs/stage0b_attempt1_online_slam_rectangle.md` — Attempt 1 failure report
- `docs/environment_feasibility.md` — Stage 0 master document
