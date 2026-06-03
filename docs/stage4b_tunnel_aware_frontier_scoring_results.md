# Stage 4B: Tunnel-Aware Frontier Scoring — Results

**Branch**: `stage4b-tunnel-aware-frontier-scoring`  
**Tag**: `stage4b-tunnel-aware-frontier-scoring`  
**Date**: 2026-06-02  

## Objective

Evaluate whether tunnel geometry features (centerline alignment, wall risk)
from Stage 4A's `tunnel_centerline_extractor` can be integrated into the
Stage 3D frontier scorer without regressing completion behavior.

## Implementation Summary

- **`TunnelGeometryGrid`**: wraps `/tunnel_centerline/distance_map` and `/risk_map`
- **`scoreAndRankTunnelAware`**: extends Stage 3D scoring with two new terms
- **`tunnel_aware` selection strategy**: uses geometry when available, falls back
  to Stage 3D when centerline node is not running
- **No changes** to FrontierDetector, FrontierBlacklist, FrontierGoalSelector,
  FrontierVisitHistory, or Nav2 configuration

## Score Formula

```
score =
    w_gain       * normalized_information_gain
  - w_distance   * normalized_distance
  - w_revisit    * normalized_revisit
  + w_centerline * normalized_centerline_alignment
  - w_wall_risk  * normalized_wall_risk
```

The scoring point is projected 0.4 m toward the robot from the frontier
representative before sampling tunnel geometry, so the scorer evaluates the
reachable safe goal rather than the frontier boundary cell.

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `selection_strategy` | `tunnel_aware` | New strategy name |
| `weight_centerline_alignment` | 0.3 | Weight for distance-from-wall term |
| `weight_wall_risk` | 0.5 | Weight for wall-proximity risk penalty |
| `geometry_sampling_radius_meters` | 0.25 | Radius for fallback sampling |
| `geometry_missing_fallback_to_stage3d` | true | Fall back when no centerline node |
| `tunnel_geometry_max_age_seconds` | 3.0 | Max staleness before fallback |

## Ablation Variants

| Variant | Centerline | Wall Risk | Config |
|---------|:----------:|:---------:|--------|
| Stage 3D baseline | — | — | `stage4b/stage3d_baseline.yaml` |
| 4B risk-only | — | 0.5 | `stage4b/risk_only.yaml` |
| 4B centerline-only | 0.3 | — | `stage4b/centerline_only.yaml` |
| 4B full | 0.3 | 0.5 | `stage4b/full.yaml` |

## Benchmark Setup

- World: `branching_tunnel_y.sdf` (2.5 m corridor, Y-shaped)
- Nav2: `nav2_params_rotation_shim.yaml`
- Explorer base: `frontier_explorer_params_info_revisit_r075.yaml`
- Duration: 900 s, stop-on-completed: true, stall-timeout: 240 s
- Centerline extractor: `centerline_extractor.launch.py` running for 4B variants

## Results

All 15 geometry-enabled runs validated: 0 fallbacks, non-zero geometry features
confirmed in every run. Geometry topics delivered within 200s gate on all runs.

| Variant | Runs | Completion | Mean TTC | Mean Revisit | Mean Bins | Recovery Probes | Nav2 Success |
|---------|------|:---------:|----------|:------------:|:---------:|:---------------:|:------------:|
| Stage 3D baseline | 5 | **80%** | 736s | 37.7% | 6.0 | 4/4 | 105.9% |
| 4B risk-only | 5 | 60% | 728s | 35.9% | 6.2 | 4/4 | 106.8% |
| 4B centerline-only | 5 | 20% | 251s | **29.8%** | **7.0** | 4/4 | 103.8% |
| 4B full | 5 | 60% | **602s** | **30.8%** | **6.6** | 4/4 | 107.3% |

### Key Deltas (3D baseline → 4B full)

| Metric | Stage 3D | 4B full | Delta |
|--------|:--------:|:-------:|:-----:|
| Completion | 80% | 60% | −20pp ❌ |
| Mean revisit | 37.7% | 30.8% | −6.9pp ✅ |
| Mean TTC | 736s | 602s | −18% ✅ |
| Mean unique bins | 6.0 | 6.6 | +10% ✅ |

## Running the Ablation

```bash
# Prerequisites: simulation + centerline extractor running
ros2 launch tunnel_centerline_extractor centerline_extractor.launch.py

# Run each variant (5 runs each)
for variant in stage3d_baseline risk_only centerline_only full; do
  for i in 1 2 3 4 5; do
    ./scripts/run_stage2a_benchmark.sh \
      --explorer-params-file "$(ros2 pkg prefix tunnel_frontier_explorer)/share/tunnel_frontier_explorer/config/stage4b/${variant}.yaml" \
      --world "$(ros2 pkg prefix tunnel_worlds)/share/tunnel_worlds/worlds/branching_tunnel_y.sdf" \
      --output-dir ~/stage4b_benchmark/${variant} \
      --run-id "$(printf '%02d' $i)" \
      --stop-on-completed true --stall-timeout-seconds 240 --duration 900
  done
done

# Compare
python3 scripts/stage4b_comparison.py
```

## Unit Tests

7/7 gtest pass (5 existing Stage 2/3 + 2 new Stage 4B):
- `test_tunnel_geometry_grid` — 5 tests (valid, bounds, avg, unknown)
- `test_frontier_scorer_tunnel_aware` — 3 tests (fallback matches 3D, tiebreak, empty)

## Conclusion

Stage 4B validates the tunnel-aware scoring integration path. Geometry maps
were successfully delivered to the frontier scorer with zero fallback across
all 15 geometry-enabled runs. Tunnel geometry reduced mean revisit by 6.9 pp
and improved mean TTC by 18% in the full configuration.

However, completion regressed from 80% to 60%, and centerline-only scoring
was especially harmful (20% completion). Therefore, `tunnel_aware` is retained
as an **experimental strategy**, while Stage 3D `information_gain_revisit`
remains the default stable policy.

**Finding**: tunnel geometry can reduce revisit and TTC, but current weights
sacrifice completion. Geometry should be used as a conservative penalty on
high-risk candidates, not as a primary scoring term.

## Next

- **Stage 4B.1**: Safe geometry bias — lower weights, geometry-as-tiebreaker, or wall-risk-only penalty to recover completion while retaining revisit gains
- **Stage 4C**: Multi-topology benchmark suite (straight, L, Y, T, cross, dead-end)
- **Stage 5**: Nav2 Tunnel-Aware Planner plugin
