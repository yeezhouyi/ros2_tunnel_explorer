# Stage 4B.3: Oscillation Escape Response — Results

**Branch**: `stage4b3-oscillation-escape-response`
**Date**: 2026-06-03
**Status**: Safe integration — escape path implemented, not exercised in smoke test

## Objective

Add an adaptive response to the Stage 4B.2 entrance oscillillation detector.
When oscillation is detected, temporarily penalize goal candidates inside the
oscillation cluster to push the scorer toward goals outside the entrance area.

## Design

### Escape Mode State

```
escape_mode_active: bool
escape_mode_remaining_goals: int (countdown from escape_goals)
oscillation_center: Point2D (centroid of recent goals in detector window)
```

### Flow

1. Before scoring: call `detector_.evaluate()` — if oscillating and not
   already in escape mode, activate
2. After scoring, before best selection: if escape mode active, penalize
   candidates within `suppression_radius` of `oscillation_center`
3. Safety: if ALL candidates are within radius, skip penalty (avoid no-goal)
4. Decrement `escape_mode_remaining_goals` on each goal dispatch
5. When countdown reaches 0, deactivate escape mode

### Key Methods

- `getOscillationCenter()`: computes centroid of goals in detector window
- `applyEscapeModePenalty()`: soft-penalizes and re-scores candidates

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `entrance_oscillation_response_enabled` | true | Enable escape mode |
| `entrance_oscillation_escape_goals` | 3 | Goals before escape expires |
| `entrance_oscillation_suppression_radius_m` | 1.5 | Penalty radius around oscillation center |
| `entrance_oscillation_escape_penalty` | 0.75 | Score penalty for in-radius candidates |

## Config

`config/stage4b3/wall_risk_weak_escape.yaml` — based on Stage 4B.1 winner
(wall_risk_weak) + escape mode params.

## Smoke Test Results

3-run smoke test on Y-world with `tunnel_aware` + `w_wall_risk=0.3`:

| Run | Status | Goals | Revisit | Oscillation | Escape |
|-----|--------|:-----:|:-------:|:-----------:|:------:|
| 01 | TIMEOUT | 9 | 44.4% | 0 | 0 |
| 02 | COMPLETED | 9 | 44.4% | 0 | 0 |
| 03 | COMPLETED | 9 | 33.3% | 0 | 0 |

**Completion**: 2/3 (67%)
**Escape mode activation**: 0/3 runs

### Why the detector didn't trigger

The timeout run (01) exhibited a different oscillation pattern than what the
Stage 4B.2 detector was designed to catch:

- Goals alternated between two nearby positions (~1.0m apart)
- `revisit_ratio` was 0.0 throughout (visit history didn't count them as revisits)
- Bin dominance was exactly 50/50 (3 goals in each bin, not > 50%)

The detector requires condition 4 (`avg_revisit_ratio >= 0.35`) which wasn't met.

### Interpretation

The escape mode is **safely integrated** — it doesn't break normal behavior
when not activated. However, activation was not observed because the detector
is intentionally conservative and the observed timeout pattern didn't match
the high-revisit oscillation signature.

## Two Types of Entrance Oscillation

The smoke test revealed a second oscillation pattern:

### Type A: Revisit-Heavy Localized Oscillation
- Goals cluster in small area with high revisit ratio
- Bin dominance clear (> 50%)
- **Stage 4B.2 detector fires** (as validated in 4B.2 smoke test)

### Type B: Alternating-Pair Oscillation
- Two nearby goals alternate
- Revisit ratio may be 0%
- Bin dominance may be exactly 50/50
- **Stage 4B.2 detector does not fire**

Type B is the dominant failure mode in the Y-world entrance. A future
detector extension (alternating-pair rule) could capture this pattern
without relaxing the conservative high-revisit conditions.

## Unit Tests

11/11 gtest pass (8 original + 3 new for `getOscillationCenter()`).

## Next

- **Stage 4B.4**: Alternating-pair oscillation detection — new detector
  condition that catches Type B patterns without increasing false positives
- Long-term: 20-run benchmark to validate escape mode activation and
  completion improvement
