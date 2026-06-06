# Stage 4D.0b: Failure Attribution — Results

**Branch**: `stage4d0-reproducibility-lock`
**Date**: 2026-06-06

## Purpose

Post-hoc failure attribution for the Stage 4D.0 baseline reproducibility test (30 runs). No policy/planner changes — only analysis and documentation.

## Failure Taxonomy

| Priority | Type | Indicators |
|----------|------|-----------|
| 1 | NAV2_FAILURE_208 | goals_aborted > 3, error_code=208 |
| 2 | FRONTIER_STARVATION_COOLDOWN | All frontiers suppressed, 0 aborts |
| 3 | EXPLORATION_STUCK | Low unique_goal_bins, high revisit |
| 4 | ENTRANCE_OSCILLATION | High oscillillation count, high revisit |
| 5 | UNKNOWN_FAILURE | None of the above |
| 6 | SUCCESS_LIKELY | no_frontiers_for_10_cycles + low aborts |

## Results Summary

| Failure Type | Count | Percentage |
|--------------|:-----:|:----------:|
| SUCCESS_LIKELY | 17 | 57% |
| FRONTIER_STARVATION_COOLDOWN | 8 | 27% |
| NAV2_FAILURE_208 | 4 | 13% |
| UNKNOWN_FAILURE | 1 | 3% |

**Key finding**: FRONTIER_STARVATION_COOLDOWN is the dominant failure mode (8/13 failed runs). The 120-second success cooldown suppresses all remaining frontiers, preventing the `no_frontiers_for_10_cycles` completion condition from triggering.

## Per-Topology Breakdown

### Stable Tier (≥ 4/5)

| Topology | Completion | Failure Type |
|----------|:----------:|--------------|
| t_junction_tunnel | **5/5** | — |
| straight_tunnel | **4/5** | 1× FRONTIER_STARVATION_COOLDOWN |
| dead_end_branch | **4/5** | 1× FRONTIER_STARVATION_COOLDOWN |

### Stress Tier (moderate variance)

| Topology | Completion | Failure Type |
|----------|:----------:|--------------|
| branching_tunnel_y | **3/5** | 2× FRONTIER_STARVATION_COOLDOWN |
| l_turn_tunnel | **1/5** | 4× FRONTIER_STARVATION_COOLDOWN |

### Excluded Tier

| Topology | Completion | Failure Type |
|----------|:----------:|--------------|
| loop_tunnel | **0/5** | 4× NAV2_FAILURE_208, 1× UNKNOWN |

## Key Findings

1. **FRONTIER_STARVATION_COOLDOWN is the dominant failure mode** — 8/13 failed runs (62%).
   The 120-second success cooldown suppresses all frontiers after the robot
   visits the entrance area, preventing completion detection.

2. **l_turn instability is explained by cooldown suppression** — 4/4 failures are
   FRONTIER_STARVATION_COOLDOWN. The L-turn topology has a single entrance that
   triggers cooldown on all frontiers.

3. **loop_tunnel failures are Nav2 execution errors** — 4/5 runs have error_code=208
   (controller failure). Goals to unreachable positions behind the loop structure.

4. **No entrance oscillillation as primary failure** — oscillillation counts are
   moderate (1-5) in most runs, not the dominant cause.

5. **Stable topologies are reproducible** — t_junction 5/5, straight 4/5, dead_end 4/5.

## Benchmark Tiers

| Tier | Topologies | Criterion | Status |
|------|-----------|-----------|--------|
| Stable | t_junction, straight, dead_end | ≥ 4/5 | Ready for comparison |
| Stress | Y-world, l_turn | Report variance | Observe failure patterns |
| Excluded | loop | Nav2 unreachable | Not comparable |

> Stable does not mean solved; it means reproducible enough for strategy comparison.

## Implications for Stage 4D.1b

The dominant failure mode (FRONTIER_STARVATION_COOLDOWN) suggests future fixes
should target cooldown behavior, not escape probe mechanics:

- Cooldown decay when no valid frontier remains
- Suppression radius adjustment
- Frontier resurrection after cooldown
- Cooldown exception when all frontiers are suppressed

Strategy comparison should use the Stable tier (t_junction, straight, dead_end)
as the primary benchmark, with Stress tier (Y-world, l_turn) as secondary.
