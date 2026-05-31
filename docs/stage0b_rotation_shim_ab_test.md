# Stage 0B-1: RotationShimController A/B Test Report

## Test Date

2026-05-31

## Configuration Comparison

| Parameter | Baseline A | Variant B |
|-----------|-----------|-----------|
| Controller plugin | `dwb_core::DWBLocalPlanner` | `nav2_rotation_shim_controller::RotationShimController` |
| Primary controller | — | `dwb_core::DWBLocalPlanner` (identical params) |
| angular_dist_threshold | — | 0.45 rad (~26°) |
| angular_disengage_threshold | — | 0.20 rad (~11°) |
| rotate_to_heading_angular_vel | — | 0.8 rad/s |
| max_angular_accel | — | 1.5 rad/s² |
| simulate_ahead_time | — | 1.0 s |
| closed_loop | — | true |
| DWB params | All default | All identical to Baseline A |
| Local costmap | obstacle_layer + inflation_layer | Same |
| Global costmap | static_layer + obstacle_layer + inflation_layer | Same |
| Cooldown | 5.0 s | 5.0 s |
| Goal timeout | 30 s | 30 s |

The only variable changed between A and B is the controller plugin and the addition of RotationShim parameters. All DWB, costmap, and inflation parameters are identical.

---

## Results Summary

| Metric | Baseline A (DWB only) | Variant B (RotationShim + DWB) | Change |
|--------|----------------------|--------------------------------|--------|
| **Success rate** | 6/11 = **54.5%** | 9/11 = **81.8%** | **+27.3 pp** |
| Total duration | 195.8 s | 161.6 s | **−34.2 s** |
| Mean goal duration | 17.8 s | 14.7 s | −3.1 s |
| DWB "No valid trajectories" | 66 | 46 | −20 |
| BaseObstacle "Trajectory Hits Obstacle" | 33 | 23 | −10 |
| "Failed to make progress" | 10 | 7 | −3 |
| follow_path action server timeout | 0 | 0 | 0 |
| ABORTED (immediate reject) | 0 | 0 | 0 |

---

## Goal-by-Goal Comparison

| # | Goal | Baseline A | Variant B | Δ |
|---|------|-----------|-----------|----|
| 1 | East 1m (1.0, 0.0) | **3.2s ✅** | **3.3s ✅** | +0.1s |
| 2 | East 2m (2.0, 0.0) | **4.0s ✅** | **4.0s ✅** | 0.0s |
| 3 | East 3m (3.0, 0.0) | **3.9s ✅** | **4.0s ✅** | +0.1s |
| 4 | NE shallow (3.3, 0.3) ~18° | **8.7s ✅** | **16.0s ✅** | +7.3s |
| 5 | NE medium (3.4, 0.7) ~34° | **20.1s ✅** | **30.1s ❌** | +10.0s → timeout |
| 6 | North 1.5m (3.3, 1.2) | **30.1s ❌** | **17.4s ✅** | **−12.7s → rescued** |
| 7 | West shallow (2.7, 1.4) | **5.4s ✅** | **22.6s ✅** | +17.2s |
| 8 | West medium (2.0, 1.5) | **30.1s ❌** | **30.1s ❌** | 0.0s |
| 9 | West 3m (0.5, 1.5) | **30.1s ❌** | **12.3s ✅** | **−17.8s → rescued** |
| 10 | South return (0.5, 0.8) | **30.1s ❌** | **15.8s ✅** | **−14.3s → rescued** |
| 11 | Finish at origin (0.2, 0.2) | **30.1s ❌** | **6.0s ✅** | **−24.1s → rescued** |

---

## Key Observations

### 1. RotationShim rescued the return path (goals 9–11)

In Baseline A, once the robot entered the north corridor and started going west (goal 8+), DWB got stuck in all subsequent goals. With the RotationShim, goals 9, 10, and 11 all succeeded — the shim's rotate-to-heading behavior helped the robot navigate the corridor turns that DWB alone couldn't handle.

### 2. Goal 5 regressed (20.1s → timeout)

The one goal that succeeded in Baseline A but failed in Variant B is Goal 5 (NE medium, ~34° turn). The RotationShim adds rotation overhead at this heading change — the robot spent ~10s rotating before engaging DWB, pushing the total over 30s. This is the cost of the rotation shim: for goals where DWB alone barely succeeds, the added rotation time can cause timeout.

### 3. Goal 4 also shows overhead (8.7s → 16.0s)

Even though ~18° is below the angular_dist_threshold of 0.45 rad (~26°), Goal 4 took nearly twice as long. This suggests the RotationShim's sampling/checking overhead adds time even when it doesn't actively rotate. Alternatively, the robot's path may have drifted, requiring rotation-while-moving.

### 4. DWB trajectory failures still occur

The shim reduces but doesn't eliminate DWB "No valid trajectories" events — 46 events vs 66 in baseline. The primary improvement comes from the rotate-to-heading phase, which eliminates the need for DWB to find paths through large heading changes.

### 5. follow_path handover fully resolved

0 handover timeouts in both variants with 5s cooldown.

---

## First-Failure Analysis

| Metric | Value |
|--------|-------|
| First failed goal | Goal 5 (NE medium, 3.4, 0.7) |
| Failure mode | TIMED_OUT after 30.1s |
| DWB errors before timeout | ~12 "No valid trajectories" events |
| Root cause | Rotation overhead + DWB struggle at ~34° heading change pushed total over 30s |

---

## Decision

Per the branching criteria:

| Threshold | Result | Action |
|-----------|--------|--------|
| **10/11 or 11/11** | ❌ 9/11 | Not reached |
| **7/11 to 9/11** | ✅ **9/11** | Check shim triggering; consider `angular_dist_threshold` adjustment |
| ≤ 6/11 | ❌ | Not applicable |

### Analysis

The RotationShim with `angular_dist_threshold: 0.45` (~26°) was triggered for the ~34° heading change at Goal 5 and likely at Goal 7 (westward turn). The improvement is clear — 54.5% → 81.8% — but the threshold may be slightly too permissive:

- **At 0.45 rad**: The shim doesn't trigger for shallow turns (~18° at Goal 4), letting DWB handle them with smaller overhead. But at 34° (Goal 5), the shim triggers but the combined rotation + DWB time exceeds 30s.
- **At 0.35 rad (~20°)**: The shim would also trigger at Goal 4's ~18° turn, adding more overhead there. But it would handle Goal 5's ~34° more aggressively, potentially completing within 30s.

### Recommended Next Step

**Option 1 (per your branching logic)**: Reduce `angular_dist_threshold` from 0.45 to 0.35 and re-run.

This makes the shim trigger on smaller heading changes (≥20° instead of ≥26°). The trade-off:
- **Pro**: More aggressive rotation handling may help Goal 5 complete within 30s
- **Con**: Goal 4 (now ~18°) may also trigger the shim, adding overhead to the 4 goals that currently succeed cleanly (1-4)

**Option 2**: Increase `--goal-timeout-seconds` to 60 for Goal 5 only.

Goal 5 succeeds at 20.1s in baseline — with rotation overhead it needs ~40s. If Goal 5 is treated as a "corner case" that just needs more time, increasing the timeout would make Variant B achieve 10/11 or 11/11.

**Option 3**: Accept 9/11 as sufficient and proceed to Stage 1C.

The 9/11 vs 10/11 gap is caused by a single borderline case (Goal 5). The shim clearly rescues the return path. If Stage 1C's Frontier Explorer generates goals that avoid this specific corner, the 81.8% rate may not be limiting.

---

## Raw Data

### Baseline A (DWB only)

Results: `~/stage0b_retry_corrected/stage0b_results.json`
Log: `~/stage0b_retry_simulation.log`

### Variant B (RotationShim + DWB)

Results: `~/stage0b_retry_shim/stage0b_results.json`
Log: `~/stage0b_retry_rotation_shim.log`
Config: `tunnel_explorer_bringup/config/nav2_params_rotation_shim.yaml`
