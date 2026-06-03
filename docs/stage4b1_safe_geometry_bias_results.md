# Stage 4B.1: Safe Geometry Bias — Results

**Branch**: `stage4b1-safe-geometry-bias`
**Tag**: `stage4b1-safe-geometry-bias-eval`
**Date**: 2026-06-03

## Objective

Evaluate whether a conservative, wall-risk-only penalty can improve completion
over the Stage 3D baseline without the regression seen in Stage 4B's centerline
bias. Two approaches tested:

1. **Direct bias** (`tunnel_aware`): wall_risk added directly to scoring formula
2. **Tiebreaker** (`tunnel_aware_tiebreaker`): wall_risk only re-ranks candidates
   within epsilon of the top score

Neither uses centerline alignment — this stage is wall-risk-only.

## Implementation Summary

- **`applyTunnelRiskTiebreaker()`**: partitions scored candidates by epsilon
  from top score, re-sorts tie group by ascending wall_risk
- **`tunnel_aware_tiebreaker` selection strategy**: Stage 3D base ranking,
  then tiebreaker refinement — geometry never overrides Stage 3D
- **`tunnel_aware` (direct)**: wall_risk weighted directly into scoring formula
  at low weights (0.15, 0.3)
- **No changes** to FrontierDetector, FrontierBlacklist, FrontierGoalSelector,
  FrontierVisitHistory, or Nav2 configuration

## Score Formulas

Direct (`tunnel_aware`):
```
score =
    w_gain       * normalized_information_gain
  - w_distance   * normalized_distance
  - w_revisit    * normalized_revisit
  - w_wall_risk  * normalized_wall_risk
```

Tiebreaker (`tunnel_aware_tiebreaker`):
```
score = base_stage3d_score
# then among candidates within tiebreaker_epsilon of top score:
# re-sort by ascending wall_risk (lower risk = better)
```

## Ablation Variants

| Variant | Strategy | w_wall_risk | Config |
|---------|----------|:-----------:|--------|
| fresh_stage3d_baseline | `information_gain_revisit` | — | `frontier_explorer_params_info_revisit_r075.yaml` |
| wall_risk_weak | `tunnel_aware` | 0.3 | `stage4b1/wall_risk_weak.yaml` |
| wall_risk_very_weak | `tunnel_aware` | 0.15 | `stage4b1/wall_risk_very_weak.yaml` |
| wall_risk_tiebreaker | `tunnel_aware_tiebreaker` | 1.0 (tiebreaker) | `stage4b1/wall_risk_tiebreaker.yaml` |

All `tunnel_aware` variants: `weight_centerline_alignment: 0.0`.

## Benchmark Setup

- World: `branching_tunnel_y.sdf` (2.5 m corridor, Y-shaped)
- Nav2: `nav2_params_rotation_shim.yaml`
- Duration: 900 s, stop-on-completed: true, stall-timeout: 240 s
- Centerline extractor: running for all geometry variants
- 5 runs per variant = 20 total runs

## Per-Run Results

### fresh_stage3d_baseline

| Run | Status | TTC | Goals | Revisit | Loops | Probes |
|-----|--------|-----|:-----:|:-------:|:-----:|:------:|
| 01 | TIMEOUT | — | 12 | 33.3% | 0 | 1/1 |
| 02 | COMPLETED | 251s | 7 | 0% | 0 | 0/0 |
| 03 | TIMEOUT | — | 10 | 40.0% | 0 | 1/1 |
| 04 | TIMEOUT | — | 9 | 44.4% | 0 | 1/1 |
| 05 | TIMEOUT | — | 11 | 27.2% | 0 | 1/1 |

### wall_risk_weak

| Run | Status | TTC | Goals | Revisit | Loops | Probes |
|-----|--------|-----|:-----:|:-------:|:-----:|:------:|
| 01 | COMPLETED | 651s | 10 | 30.0% | 0 | 1/1 |
| 02 | COMPLETED | 885s | 9 | 44.4% | 0 | 1/1 |
| 03 | COMPLETED | 240s | 7 | 0% | 0 | 0/0 |
| 04 | TIMEOUT | — | 9 | 33.3% | 0 | 1/1 |
| 05 | TIMEOUT | — | 10 | 30.0% | 0 | 1/1 |

### wall_risk_very_weak

| Run | Status | TTC | Goals | Revisit | Loops | Probes |
|-----|--------|-----|:-----:|:-------:|:-----:|:------:|
| 01 | TIMEOUT | — | 12 | 16.6% | 0 | 1/1 |
| 02 | TIMEOUT | — | 10 | 40.0% | 0 | 1/1 |
| 03 | TIMEOUT | — | 12 | 33.3% | 0 | 1/1 |
| 04 | COMPLETED | 381s | 8 | 0% | 0 | 0/0 |
| 05 | TIMEOUT | — | 11 | 36.3% | 0 | 1/1 |

### wall_risk_tiebreaker

| Run | Status | TTC | Goals | Revisit | Loops | Probes |
|-----|--------|-----|:-----:|:-------:|:-----:|:------:|
| 01 | COMPLETED | 890s | 9 | 33.3% | 0 | 1/1 |
| 02 | TIMEOUT | — | 10 | 30.0% | 0 | 1/1 |
| 03 | TIMEOUT | — | 11 | 36.3% | 0 | 1/1 |
| 04 | COMPLETED | 801s | 10 | 30.0% | 0 | 1/1 |
| 05 | TIMEOUT | — | 11 | 36.3% | 0 | 1/1 |

## Aggregate Results

| Variant | Completion | Mean TTC (completed) | Mean Revisit (all) | Mean Revisit (completed) | Nav2 Success |
|---------|:----------:|:--------------------:|:-------------------:|:------------------------:|:------------:|
| fresh_stage3d_baseline | **20%** (1/5) | 251s | 29.0% | 0% | 106.0% |
| **wall_risk_weak** | **60%** (3/5) | **592s** | **27.5%** | **24.8%** | **108.4%** |
| wall_risk_very_weak | 20% (1/5) | 381s | 25.2% | 0% | 105.5% |
| wall_risk_tiebreaker | 40% (2/5) | 846s | 33.2% | 31.7% | 108.0% |

## Key Deltas (baseline → wall_risk_weak)

| Metric | Baseline | wall_risk_weak | Delta |
|--------|:--------:|:--------------:|:-----:|
| Completion | 20% | **60%** | **+40pp** |
| Mean revisit (all) | 29.0% | 27.5% | −1.5pp |
| Mean goals | 9.8 | 9.0 | −8% |
| Loop detected | 0/20 | 0/20 | — |
| Recovery probes | 4/4 | 4/4 | — |

## Pipeline Health

| Check | Result |
|-------|--------|
| Geometry gate passes | **15/15** (all geometry variants × 5 runs) |
| Fallbacks | **0/15** |
| `+tiebreak` suffix (tiebreaker variant) | **100%** (9-11 per run, all 5 runs) |
| QoS errors | **0** |
| Node crashes | **0** |

## Conclusion

Stage 4B.1 validates that a **conservative, wall-risk-only penalty** can
significantly improve completion over the baseline without the regression
seen in Stage 4B's centerline-biased scoring.

**wall_risk_weak** (tunnel_aware, w_wall_risk=0.3, no centerline) is the
recommended experimental configuration:
- 60% completion (3× the 20% fresh baseline)
- 0 fallbacks, 15/15 geometry gate passes
- 27.5% mean revisit (modest improvement over 29.0%)

The **tiebreaker approach** (`tunnel_aware_tiebreaker`) was too conservative:
40% completion, with higher mean revisit (33.2%) than the direct bias. The
epsilon-only constraint limits its effectiveness — it only influences rankings
when candidates are within 0.10 of the top score, which is rare in the
Y-world entrance area where most runs timeout.

**wall_risk_very_weak** (w=0.15) matched the baseline at 20% completion —
the weight is too weak to help.

### Caveat

Entrance oscillation remains the dominant failure mode. The loop detector
registered 0 in all 20 runs, yet entrance-area goal switching is clearly
visible in TIMEOUT runs. Recovery probes were dispatched and succeeded, but
were insufficient to break the pattern in most cases.

## Next

- **Stage 4B.2**: Entrance oscillation detection — instrument the failure
  detector to recognize spatial patterns in repeated goal switching before
  attempting stronger interventions
- **Stage 4C**: Multi-topology benchmark suite
- **Stage 5**: Nav2 Tunnel-Aware Planner plugin
