# Stage 4B.4: Alternating-Pair Oscillation Detection — Results

**Branch**: `stage4b4-alternating-pair-oscillation`
**Tag**: `stage4b4-alternating-pair-oscillation`
**Date**: 2026-06-04
**Status**: Activation verified, behavior improved

## Objective

Detect Type B entrance oscillillation — alternating A/B goal switching patterns
that occur with 0% revisit ratio and balanced bin usage. The Stage 4B.2
detector (Type A: revisit-heavy) intentionally does not catch these patterns.
Stage 4B.4 adds a separate detection path so the existing escape mode can
activate on Type B oscillillation.

## Design

### Type A vs Type B

| Property | Type A (4B.2) | Type B (4B.4) |
|----------|:-------------:|:-------------:|
| Pattern | Localized, high revisit | Alternating A/B pair |
| Revisit ratio | ≥ 35% | 0% possible |
| Bin dominance | > 50% | Exactly 50/50 |
| Detection | `evaluate()` conditions 1-4 | `detectAlternatingPair()` |
| Reason | `type=revisit_heavy` | `type=alternating_pair` |

### Detection Algorithm

1. **Greedy clustering**: group goals by spatial proximity (fixed anchor,
   cluster_radius = 0.75m)
2. **Cluster count**: must be exactly 2
3. **Balanced representation**: each cluster ≥ 2 goals
4. **Spatial compactness**: overall radius ≤ 1.5m
5. **No progress**: unique bins not growing between halves
6. **Alternation score**: ≥ 0.5 (transitions / (window - 1))

### Alternation Score Examples

```
A B A B A B → 5/5 = 1.0  (perfect alternation)
A A B B A B → 3/5 = 0.6  (mixed)
A A A B B B → 1/5 = 0.2  (clustered, not alternating)
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `detect_alternating_pair` | true | Enable Type B detection |
| `pair_cluster_radius_m` | 0.75 | Clustering radius for goals |
| `pair_max_spatial_radius_m` | 1.5 | Max overall spatial extent |
| `pair_min_cluster_count` | 2 | Min goals per cluster |
| `pair_min_alternation_score` | 0.5 | Min alternation threshold |

## Integration

Type B detection is a fallback in `evaluate()`: if Type A doesn't fire and
`detect_alternating_pair` is enabled, `detectAlternatingPair()` is called.
If it detects oscillillation, `OscillationStatus.oscillating` is set to true
with `reason` prefixed by `type=alternating_pair`.

The existing escape mode (Stage 4B.3) activates on any `oscillating=true`
regardless of type.

## Smoke Test Results

3-run Y-world smoke test with `wall_risk_weak_escape` config:

| Run | Status | Goals | Revisit | Oscillation | Escape | Type A | Type B |
|-----|--------|:-----:|:-------:|:-----------:|:------:|:------:|:------:|
| 01 | COMPLETED | 10 | 30.0% | 2 | 5 | 0 | **2** |
| 02 | COMPLETED | 10 | 40.0% | 2 | 5 | 0 | **2** |
| 03 | COMPLETED | 9 | 44.4% | 2 | 5 | 0 | **2** |

### Key Metrics

- **Completion**: 3/3 (100%)
- **Type B detection**: 2 events per run (6 total)
- **Escape mode activation**: 5 per run (15 total)
- **Type A detection**: 0 (all were Type B patterns)
- **No no-goal states**: 0

### Example Log

```
[EntranceOscillation] detected=true count=1 window=5 radius=0.85m
unique_bins=2 revisit=0.00 reason=type=alternating_pair
two_cluster_pair+stagnant_bins+localized_goals
```

## Interpretation

The dominant Y-world entrance oscillillation mode is now:

```
detectable → classified as type=alternating_pair → triggers escape → completes
```

Before Stage 4B.4, this pattern was invisible to the detector (0% revisit,
50/50 bin split). Now it fires consistently on every run that exhibits
alternating-pair behavior, and the escape mode activates to break the cycle.

## Unit Tests

17/17 gtest pass (11 original + 6 new for alternating-pair detection).

## Next

- **Stage 4B.4-Eval**: Formal 5-run benchmark comparing baseline,
  wall_risk_weak, and wall_risk_weak_escape to quantify completion improvement
