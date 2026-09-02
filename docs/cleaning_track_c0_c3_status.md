# Cleaning Track — C0–C3 (4-week MVP slice) Status

Branch: `coverage-cleaning-track` (based on `stage3d-entrance-loop-recovery`
@ `15fc269` — the frozen exploration baseline from the cleaning-track plan).

This document records what the first implementation slice delivers, how the
geometry semantics are defined, how to build/run it in WSL2, and which plan
items remain for the next slices (U6+).  It is the companion of
`title_ROS2 Tun.docx` (ROS2 Tunnel Explorer 全覆盖清扫进阶 - Plan).

## Deliverables in this slice

| Package | Scope (plan unit) | Status |
|---|---|---|
| `tunnel_map_core` | U1 grid geometry + map digest | Implemented + GTest |
| `tunnel_coverage_planner` | U2 cleanable masks, U3 coverage tracker/metrics, U4 scanline planner | Implemented + GTest |
| `tunnel_coverage_msgs` | U5 action/messages | Defined |
| `tunnel_coverage_executor` | U5 executor first pass + checkpoint + task core | Implemented + GTest (core/checkpoint) |
| `tunnel_worlds` | rect benchmark world + map + initial poses | Assets generated |
| `tunnel_explorer_bringup` | `coverage_simulation.launch.py` + `nav2_params_coverage_dwb.yaml` | Added |
| `benchmark_tools` | pure coverage-metrics module + `send_coverage_goal.py` + tests | Added |
| `scripts/run_coverage_benchmark.sh` | 5-run runner | Added |

Core principle kept from the plan (KTD3, R18): every algorithm unit is a
pure C++ static library with zero ROS dependencies; the ROS2 node only
adapts topics/actions/TF to it.

## Geometry and metric semantics

- Grid layout follows nav_msgs/OccupancyGrid row-major order used across the
  repo: cell `(row, col)` centre =
  `(origin_x + (col+0.5)*res, origin_y + (row+0.5)*res)` for yaw=0.
- `task_input_id` = digest of map content + geometry (`tunnel_map_core`);
  `plan_id` = digest of the canonical serialised plan.  Checkpoint resume is
  refused when either id mismatches (R12, R22, R23).
- Masks (R1, R19):
  - `intended_target` = all free cells (denominator of gross coverage);
  - `navigable_center` = free cells eroded by the chassis clearance disc
    (`robot_radius + safety_margin`);
  - `reachable_cleanable` = target cells inside the tool-reach dilation of
    the navigable component connected to the seed pose;
  - `exempt` = intended \ reachable with auditable per-cell cause
    (`UNREACHABLE`/`ISLAND`).
- Tracker (R3): only poses from the fresh `map -> base_footprint` transform
  are fed in; segments are interpolated at ≤ half a cell before stamping the
  circular cleaning footprint.
- Metrics (R4): gross = |T∩V|/|T|; effective = |(T\E)∩V|/|T\E|;
  exempt = |E|/|T|; repeat = |{c∈T\E: visits>1}|/|T\E|.
- Scanline rows (R5, R7): spacing =
  `min(w*(1-eta), w - 2*(e_loc+e_track))`; first/last rows sit `w/2` from the
  region bounds so the centred footprint reaches the walls; endpoint inset is
  kept at ~1 cell (0.05 m) because a larger inset cuts the wall strips.

## Local verification performed (Windows, Python mirror)

No ROS2/C++ toolchain is reachable on this Windows host (WSL2 service is not
accessible from the sandbox), so the algorithms were mirrored 1:1 in Python
(`benchmark_tools/benchmark_tools/coverage_metrics.py` + a local mirror
script) and numerically verified against the generated rect asset:

- rect room (5.5 × 3.5 m, 0.05 m): exempt == 0; plan valid (19 work rows);
  row spacing ≤ 0.28 m; simulated perfect execution effective coverage
  **0.9984** (gate ≥ 0.97); deterministic plan id.
- narrow brush (w=0.20 m): wall strips correctly become exempt (>0).
- pillar room: 14 work rows, no work segment crosses the pillar, endpoints
  inside navigable space, deterministic.
- digest: stable for identical maps, sensitive to content/origin change.
- `pytest benchmark_tools/tests/test_coverage_metrics.py`: 4 passed.

These mirrors validate semantics and assets; the C++ GTests must still be run
under `colcon test` in WSL2.

## How to build and run (WSL2 / Ubuntu 24.04, ROS2 Jazzy)

```bash
cd ~/ros2_ws/src/ros2_tunnel_explorer   # or your checkout of this branch
git checkout coverage-cleaning-track

# 1) Build (order matters: tunnel_map_core -> planner -> msgs -> executor)
colcon build --symlink-install --packages-select \
  tunnel_map_core tunnel_coverage_planner tunnel_coverage_msgs \
  tunnel_coverage_executor benchmark_tools tunnel_worlds \
  tunnel_explorer_bringup tunnel_frontier_explorer
source install/setup.bash

# 2) Unit gates
colcon test --packages-select tunnel_map_core tunnel_coverage_planner \
  tunnel_coverage_executor benchmark_tools
colcon test-result --verbose

# 3) Simulation smoke (rect world, static map + AMCL, DWB)
./scripts/cleanup_simulation.sh
ros2 launch tunnel_explorer_bringup coverage_simulation.launch.py \
  headless:=True rviz:=False

# The executor should go READY_IDLE (phase 4 on /coverage/status).
# In a second terminal send one coverage goal:
ros2 run benchmark_tools send_coverage_goal.py --output-dir /tmp/cov_run_01

# 4) Formal 5-run benchmark (run each gate from a fresh terminal)
./scripts/run_coverage_benchmark.sh \
  --world cleaning_room_rect.sdf \
  --map <abs>/tunnel_worlds/maps/cleaning_room_rect.yaml \
  --controller dwb --runs 5 \
  --output-dir ~/coverage_benchmarks/rect_dwb
```

Expected MVP gate (plan DoD, C0–C3): rect world 5/5 completed, each run
effective/gross ≥ 97 %, exempt ratio 0, zero collisions.

## What is intentionally deferred (next slices)

- U6 Boustrophedon cell decomposition + segment ordering (pillar/L/doorway
  maps are the C5 milestone; the rect plan is generated with the scanline
  MVP only).
- U7 boundary/wall pass + residual replanning and per-segment coverage gates
  (the executor currently applies a geometric endpoint gate per segment plus
  a global coverage gate at RESIDUAL_CHECK).
- U8 full dynamic-obstacle lifecycle (BLOCKED_TEMP exists in the core model;
  the executor first pass is sequential).
- U9 controller A/B config (`nav2_params_coverage_rpp.yaml` still to be
  added once RPP availability is confirmed), MC controller plugin.
- U10 remaining worlds (`pillar`, `l_shape`, `doorway`, `dynamic`) and the
  rosbag/plot aggregator.
- U11/U12 optional robot migration / cloud runner.

The executor node is a first-pass integration; compile & simulation fixes
are expected there first (see Known Gaps).  Exploration behaviour is
untouched (R15): `stage0_simulation.launch.py` and the frontier defaults are
not modified by this branch.

## Known gaps / risks

1. Executor C++ (`tunnel_coverage_executor`) has not been compiled yet on
   this machine; treat it as requiring a compile/fix cycle in WSL2 before
   running the simulation gates.
2. Stop confirmation uses TF displacement, not wheel odometry — threshold
   tuning may be needed.
3. `coverage_simulation.launch.py` assumes Jazzy `tb3_simulation_launch.py`
   accepts `slam`, `map`, `world`, `x_pose`, `y_pose`, `yaw` (verified
   against the jazzy branch source).
4. Python mirrors are reference implementations, not the C++ (keep formulas
   in sync when touching `CoverageTracker::metrics()`).
