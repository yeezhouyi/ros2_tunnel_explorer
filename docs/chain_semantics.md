# Chain semantics & boundary clarifications (Day 9)

Reference: merge tree `bline-merge-20260908`; audit record
`docs/day68_chain_audit.md`; delivery-denominator doc §5 Day 6-9.

## 1. Which planner produced the executed route?

The coverage executor (`tunnel_coverage_executor`) plans internally with
`tunnel_coverage_planner::ScanlinePlanner` over the frozen static map it
is served; it does NOT ingest the python `cleaning_mode` route at
runtime.  All Day 6-8 and historical sim runs served the STATIC map
`cleaning_room_rect.yaml` (map_server 'Loading yaml file' lines verify
this).  Python `cleaning_mode` is canonical ONLY for the plan
reproducibility chain, and it must be run over the SERVED map: over
cleaning_room_rect it yields a 60.75 m / 22-waypoint route (reproducible
with `plan_from_map.py --map-yaml cleaning_room_rect.yaml`).  It is NOT
route-identical to the executor's internal C++ segments (do not assume
equivalence on any map).  An earlier note that the executed route was
geometry-equivalent to a 69.95 m python plan is RETRACTED: 69.95 m /
250 waypoints / executable 6266 was `plan_from_map` over
`b6_chain/map_saved.yaml` (a SLAM map, different origin), which no sim
run served.

## 2. Two denominators, never swapped (B3 / R10)

- Executor ledger (`coverage_goal_result.json`): self-tracking — segment
  endpoints reached + per-segment /odom self-consistency.  Stable across
  runs (effective 0.886–0.913 over 4 runs) but is NOT absolute coverage.
- Grid audit (`audit_b6_coverage.py`, D-line repo): tool-disc visited vs
  executable / known_free masks.  `coverage_task` (planned region
  denominator) is the absolute-coverage figure; sensitive to odom→map
  alignment and per-run trajectory tightness (driven 117–173 m across
  runs; coverage_task 0.28–0.37).
- Report the RANGE and both denominators; a single number is a category
  error.  `repeat_ratio` 0.69–0.72 (row-working inefficiency) remains the
  open work item.

## 3. Controller identity / scope boundary

- The coverage chain runs Nav2 `RotationShimController` + `dwb_core`
  (`nav2_params_coverage_dwb.yaml`).  All chain coverage numbers above
  refer to that controller.
- The linear-MPC Nav2 plugin (B6B, D-line `cloud-exploration-20260908`)
  is a separate deliverable: pluginlib load + lifecycle + straight/arc
  sandbox gates passed (stage 1 4/4, 2a 7/7, 2b 8/8) and the terminal
  clamp negative control Δ=11.18 m.  It is NOT wired into the coverage
  chain runs and has NO hard-real-time claim.  Do not present it as the
  chain controller.
- No hard real-time statement is authorized anywhere (seal constraint).

## 4. Startup readiness race (fixed)

Executor rejects goals unless phase==READY_IDLE (map + AMCL + Nav2
bring-up order nondeterministic).  Fix (Day 9): client waits in-process
on `/coverage/status` until phase==4 before sending; raises fast if a
task is already active.  Shell retry loop demoted to fallback.
