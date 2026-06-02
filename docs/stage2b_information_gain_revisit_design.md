# Stage 2B: Information Gain + Revisit Penalty Frontier Scoring

## Baseline Reference

Stage 2A nearest-frontier baseline (5 formal runs):

| Metric | Value |
|--------|-------|
| Valid algorithm samples | 5 |
| Stable COMPLETED | 4 |
| TIMEOUT | 1 |
| Completion rate | 80 % |
| TTC median | 281.5 s |
| TTC min | 182 s |
| TTC max | 358 s |
| Goals dispatched (median) | 15 |
| Goals dispatched (max) | 35 |
| Revisit rate (median) | 20 % |
| Revisit rate (max) | 60 % |
| Navigation goal success rate | 97.7 % |
| Infrastructure exclusions | 0 |

## Motivation

Run 3 exhibited 60 % revisit rate with 35 goals in only 14 unique spatial bins.
Nearest-frontier's purely distance-based selection causes local cycling when
frontier clusters are distributed evenly around the robot.  Stage 2B adds two
terms to the goal selection formula to break these cycles.

## Scope

### Added
- `FrontierScorer` — scores candidates by info gain, distance, revisit penalty
- `FrontierVisitHistory` — tracks accepted goals for revisit penalty
- `FrontierGoalSelector::selectAll()` — returns all distance-filtered candidates
- Strategy branch in exploration node (`selection_strategy` parameter)
- `scored_frontiers` RViz marker namespace

### Frozen (no changes)
- `FrontierDetector`, `FrontierBlacklist`, `FrontierGoalSelector::select()`
- 6-state state machine; `goal_timeout=60.0s`, `cooldown=5.0s`,
  `min_goal_distance=0.50m`
- Nav2: RotationShimController, DWB, costmap, SLAM
- Stage 2A benchmark runner (new `--explorer-params-file` arg only)

## Scoring Formula

```
raw_gain       = countUnknownCellsInRadius(goal, radius=0.75 m)    [circular]
transformed    = log(1 + raw_gain)

raw_revisit    = countVisitsNear(goal, radius=0.50 m)
clamped        = min(raw_revisit, max_revisit_count=3)

per-set normalisation to [0, 1]:
  norm_gain     = (t_gain - min_gain) / (max_gain - min_gain)     0 if all equal
  norm_dist     = (dist - min_dist) / (max_dist - min_dist)        0 if all equal
  norm_revisit  = clamped_revisit / max_revisit_count

score = w_gain * norm_gain  -  w_dist * norm_dist  -  w_revisit * norm_revisit
```

Higher score = better.  Distance reduces score (closer is better).

### Tie-break
When `|score_a - score_b| <= 1e-9`:
1. Score descending
2. Goal distance ascending
3. Raw information gain descending
4. Representative cell row ascending
5. Representative cell col ascending

## Default Parameters

```yaml
selection_strategy: information_gain_revisit

information_gain_radius_meters: 0.75
revisit_radius_meters: 0.50
max_revisit_count: 3

weight_information_gain: 1.0
weight_distance: 1.0
weight_revisit: 1.5
```

All Stage 2A parameters unchanged (`cooldown_seconds`, `goal_timeout_seconds`,
`min_goal_distance_meters`, etc.).

## Architecture

```
FrontierDetector::detect()
  → initial blacklist filter
  → FrontierGoalSelector::selectAll()       [min-distance filtering]
  → second blacklist check on final rep     [catches altered representatives]
  → FrontierScorer::scoreAndRank()          [gain + distance + revisit]
  → send_goal_async()
  → goalResponseCallback (accepted)
  → FrontierVisitHistory::recordAcceptedGoal()
```

For `nearest` strategy: existing `FrontierGoalSelector::select()` path
unchanged.

### VisitHistory recording rules

| Outcome | Recorded? | Rationale |
|---------|-----------|-----------|
| Goal accepted, later succeeds | Yes | Area was actually visited |
| Goal accepted, later times out | Yes | Area was attempted |
| Goal accepted, later aborted | Yes | Area was attempted |
| Goal rejected by server | No | Never dispatched |
| Action server unavailable | No | Never dispatched |

## New Files

| File | Purpose |
|------|---------|
| `include/.../frontier_scorer.hpp` | `FrontierScorerConfig`, `ScoredGoal`, `FrontierScorer` class |
| `src/frontier_scorer.cpp` | Implementation: circular radius counting, scoring, tie-break |
| `include/.../frontier_visit_history.hpp` | `FrontierVisitHistory` class |
| `src/frontier_visit_history.cpp` | Implementation: record, query, clear |
| `test/test_frontier_scorer.cpp` | 21 unit tests |
| `test/test_frontier_visit_history.cpp` | 9 unit tests |
| `config/frontier_explorer_params_info_revisit.yaml` | Stage 2B variant params |

## Modified Files

| File | Change |
|------|--------|
| `include/.../frontier_goal_selector.hpp` | Added `AllCandidatesResult`, `selectAll()`, made `gridToWorld` public |
| `src/frontier_goal_selector.cpp` | `collectAccepted()` helper, `selectAll()` implementation |
| `include/.../frontier_explorer_node.hpp` | Added `scorer_`, `visit_history_`, strategy params, scoring params |
| `src/frontier_explorer_node.cpp` | Strategy branch in IDLE state, visit history recording, scored markers |
| `CMakeLists.txt` | New sources and test targets |
| `launch/frontier_explorer.launch.py` | `params_file` launch argument |
| `scripts/run_stage2a_benchmark.sh` | `--explorer-params-file`, params sha256 in JSON |
| `README.md` | Stage 2B section |

## Experiment Protocol

### Smoke test (before formal runs)
1. Launch with `frontier_explorer_params_info_revisit.yaml`
2. Verify in RViz:
   - `scored_frontiers` markers with correct colour gradient
   - Goal dispatch log shows scoring details
   - Robot moves and explores without crashing
   - State machine transitions correctly
3. Verify baseline still works with default params (nearest strategy)

### Formal 5-run A/B

Same protocol as Stage 2A:

| Parameter | Value |
|-----------|-------|
| Runs | 5 valid algorithm samples |
| Max duration | 600 s |
| Completed grace | 20 s |
| Stall timeout | 90 s |
| Infrastructure exclusion retries | Yes (re-run) |
| Gazebo world | Identical to Stage 2A |
| Initial pose | Identical |
| Nav2 params | Identical |

Only changed variable: `selection_strategy: information_gain_revisit`.

### Success Criteria

Any of:
- Completion rate improvement, OR
- TTC median decrease ≥ 15 %, OR
- Revisit rate median decrease ≥ 30 %

With:
- Navigation goal success rate ≥ 95 % (no regression below Stage 2A)

## RViz Markers

| Namespace | Type | Purpose |
|-----------|------|---------|
| `frontier_clusters` | POINTS | All detected frontier centroids (green) |
| `too_close_frontiers` | POINTS | Rejected (too close) frontiers (yellow) |
| `blacklisted` | SPHERES | Blacklisted positions (grey) |
| `scored_frontiers` | SPHERE_LIST | Scored candidates, size ∝ score (red→green) |
| `selected_goal` | SPHERE | Final dispatched goal (red) |

## Not in Stage 2B

- Turn cost
- Dead-end risk
- Centerline bias
- Wall proximity risk
- TunnelAwarePlanner
- MPC Controller
- Time decay for revisit history
- Machine learning / learned scoring

---

- **Date:** 2026-06-01
- **Branch:** `stage2b-info-revisit`
- **Baseline tag:** `stage2a-nearest-frontier-baseline`
