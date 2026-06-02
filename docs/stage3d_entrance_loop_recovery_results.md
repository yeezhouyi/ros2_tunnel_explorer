# Stage 3D: Entrance-Loop Recovery — Formal Results

**Branch**: `stage3d-entrance-loop-recovery`  
**Tag**: `stage3d-entrance-loop-recovery-eval`  
**Date**: 2026-06-02  
**World**: `branching_tunnel_y.sdf` (2.5 m corridor, Y-shaped bifurcation)  
**Nav2 config**: `nav2_params_rotation_shim.yaml` (RotationShim + DWB, xy=0.35, yaw=6.28)  
**Explorer config**: `frontier_explorer_params_info_revisit_r075.yaml`

## Summary

Stage 3C exposed a topology-generalization failure: the Stage 2C information-gain +
revisit-suppression policy oscillated at the entrance of a Y-shaped branching tunnel in
3/5 runs, yielding 40% completion despite 100% Nav2 execution success.

Stage 3D adds:
- **Sliding-window loop detector**: tracks recent goal spatial bins; triggers when
  ≤2 unique bins and ≥3 successes within a 6-goal window.
- **Forward recovery probe**: when a loop is detected, dispatches a goal 0.8–1.2 m
  forward (0°/±20°/±35° offsets) in known free space instead of re-selecting the
  entrance frontier.

## 5-Run Results

| Run | Status | Nav % | Goals | Unique bins | Revisit | Recovery probe | Time |
|-----|--------|-------|-------|-------------|---------|-----------------|------|
| 01 | COMPLETED* | 100% | 10 | 6 | 40.0% | 1/1 succeeded | 886s |
| 02 | COMPLETED* | 100% | 10 | 6 | 40.0% | 1/1 succeeded | 894s |
| 03 | COMPLETED* | 100% | 9 | 6 | 33.3% | 1/1 succeeded | 891s |
| 04 | COMPLETED | 100% | 8 | 5 | 37.5% | 1/1 succeeded | 867s |
| 05 | COMPLETED | 100% | 9 | 7 | 22.2% | not needed | 509s |

\* Runs 01–03 logged `No frontiers for 10 cycles — exploration complete` (explorer-level
completion) but the v0.0 benchmark harness classified them as TIMEOUT due to a
completion-detection artifact (late `/map` messages reset the stable-completion grace
period). This artifact is fixed in the v0.1 harness (`v0.1-stage3d-clean`).

## Aggregate Metrics

| Metric | Stage 3C (baseline) | Stage 3D (recovery) | Delta |
|--------|:-------------------:|:-------------------:|:-----:|
| Completion rate (explorer-level) | 40% (2/5) | **100% (5/5)** | +60 pp |
| Mean unique bins | 4.0 | **6.0** | +50% |
| Median revisit rate | 57.1% | **37.5%** | −19.6 pp |
| Mean revisit rate | 49.3% | **34.6%** | −14.7 pp |
| Nav2 success rate (all runs) | 100% | **100%** | 0 pp |

## Recovery Probe Performance

| Metric | Value |
|--------|-------|
| Runs where loop detected | 4/5 |
| Recovery probes dispatched | 4 |
| Recovery probes succeeded | 4 |
| Recovery probe success rate | **100%** |
| Runs with natural breakout | 1 (run 05) |

The recovery probe consistently broke the entrance oscillation. In run 05 the SLAM
naturally expanded the map after 4 entrance-area goals, so no probe was needed.

## Key Log Evidence (run 01)

```
Goal: (-0.45, 0.43) dist=0.62 → succeeded   # entrance oscillation begins
Goal: (0.30, 0.53)  dist=0.69 → succeeded
Goal: (-0.45, 0.43) dist=0.51 → succeeded
Goal: (0.35, 0.48)  dist=0.51 → succeeded
Goal: (-0.45, 0.43) dist=0.53 → succeeded
Goal: (0.35, 0.48)  dist=0.51 → succeeded   # loop detected: ≤2 bins, ≥3 successes

Recovery probe goal: (1.12, -0.16) d=1.20 a=-28.8deg → succeeded
Goal: (0.05, 1.92) [3 cand]  dist=2.03 → succeeded    # TRUNK BREAKOUT
Goal: (-0.36, 4.27) [9 cand] dist=3.06 → succeeded    # mid-trunk
Goal: (-0.01, 6.22) dist=2.45 → succeeded             # junction reached

No frontiers for 10 cycles — exploration complete
```

## Configuration Reference

```yaml
# Goal projection
goal_projection_enabled: true
goal_projection_distance: 0.4
goal_projection_min_remaining_distance: 0.6

# Success cooldown (radius-based)
goal_success_cooldown_seconds: 120.0
goal_success_cooldown_radius: 1.0

# Loop detection
loop_detection_enabled: true
loop_window_size: 6
loop_unique_bins_threshold: 2
loop_min_successes: 3
loop_bin_size: 0.75

# Recovery probe
recovery_probe_enabled: true
recovery_probe_distances: [1.2, 1.0, 0.8]
recovery_probe_angle_offsets_deg: [0.0, 20.0, -20.0, 35.0, -35.0]
recovery_probe_cooldown_seconds: 30.0
recovery_max_attempts: 3

# Nav2 goal checker
xy_goal_tolerance: 0.35
yaw_goal_tolerance: 6.28
```

## Artifacts

- `artifacts/stage3d_entrance_loop_recovery/` — 5-run raw results + config snapshot
- `artifacts/stage3c_branching_y_failed_eval/` — Stage 3C baseline for comparison
- `scripts/stage3_comparison.py` — automated Stage 3C vs 3D comparison
