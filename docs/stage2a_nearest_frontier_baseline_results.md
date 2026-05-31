# Stage 2A: Nearest-Frontier Baseline Benchmark Results

## Overview

Five formal runs of the nearest-frontier exploration strategy in a staged tunnel
environment. Each run uses the same algorithm, parameters, and environment —
only the random seed differs.

- **Strategy:** Nearest-frontier (centroid distance tie-break)
- **Environment:** Tunnel (Stage 0A, staged tunnel with bends)
- **Max duration:** 600 s per run
- **Stop-on-completed:** true, with 20 s grace window
- **Stall timeout:** 90 s
- **Runtime retries:** 2 (none triggered)
- **Infrastructure anomalies:** 0

## Algorithm Outcomes

| Metric | Value |
|--------|-------|
| Valid algorithm samples | 5 |
| Stable COMPLETED | 4 |
| TIMEOUT | 1 |
| Completion rate | 80 % |
| Infrastructure exclusions | 0 |
| Startup retries | 0 |

## Time-to-Completion (stable COMPLETED only)

| Metric | Value |
|--------|-------|
| Min | 182 s |
| Median | 281.5 s |
| Max | 358 s |

## Exploration Efficiency (all algorithm samples)

| Metric | Min | Median | Max |
|--------|-----|--------|-----|
| Goals dispatched | 9 | 15 | 35 |
| Unique spatial bins (0.25 m) | 9 | 12 | 14 |
| Revisit rate | 0 % | 20 % | 60 % |
| Navigation goal success rate | 77.7 % | 100 % | 100 % |

**Total navigation:** 85 / 87 goals succeeded = **97.7 %**

## Navigation Robustness

| Metric | Total |
|--------|-------|
| Goals succeeded | 85 |
| Goals timed out | 0 |
| Goals aborted | 2 |
| FrontierGoalSelector filter activations | Run-dependent |

## System Stability

| Metric | Value |
|--------|-------|
| Node crashes | 0 across all runs |
| Nav2 lifecycle failures (end-of-run) | 0 across all runs |
| Startup failures | 0 |
| STALLED | 0 |
| CRASHED | 0 |

## Observations

### Run 3 — Local Cycling Case Study

Run 3 is the most instructive sample. It reached 60 % revisit rate with 35 goals
dispatched across only 14 unique spatial bins, and only entered the COMPLETED
candidate state at ~590 s — just before the 600 s timeout. This pattern is
characteristic of nearest-frontier's fundamental weakness: **purely distance-based
goal selection can produce prolonged local cycling when frontier clusters are
distributed evenly around the robot.**

By contrast, Run 5 achieved 0 % revisit rate with 10 goals and 10 unique bins,
completing in 182 s. The same algorithm, same parameters — differing only in
the random seed's effect on frontier emergence order.

This variance is the primary motivation for Stage 2B.

### Nearest-Frontier Characterisation

- **Strengths:** Reliable goal generation; no navigation failures caused by the
  exploration strategy itself; simple and predictable per-step behaviour.
- **Weakness:** No mechanism to prefer unexplored regions over recently-visited
  ones. When frontier clusters persist in multiple directions, the robot can
  oscillate between them rather than committing to one direction.
- **Impact:** Completion time varies by ~2× (182–358 s for successful runs), and
  the 60 % revisit rate in Run 3 indicates substantial wasted travel.

## Stage 2B Recommendations

The data supports a focused, minimal intervention:

1. **Information gain weighting** — prefer frontier clusters with more unknown
   cells, biasing the robot toward larger unexplored regions.
2. **Revisit penalty** — discount clusters in recently-visited spatial bins to
   break local cycles.

These two additions directly target the observed failure mode without adding
complexity. See `stage2b_plan.md` (forthcoming).

---

- **Runner tag:** `stage2a-runner-validated`
- **Algorithm tag:** `stage1c-nearest-frontier-pass`
- **Date:** 2026-06-01
