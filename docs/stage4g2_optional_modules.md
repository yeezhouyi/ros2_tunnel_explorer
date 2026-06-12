# Stage 4G.2: Optional Modules — Final Classification

**Date**: 2026-06-12

## Purpose

Classify all validated-but-not-selected modules as optional/diagnostic, documenting their status and recommended use.

## Final Default Strategy

```
baseline + startup_bootstrap + cooldown_loop_guard
```

## Optional Modules

### 1. Cooldown Starvation Recovery

**Status**: Implemented, validated, NOT selected as default.

**What it does**: When all frontiers are suppressed by 120s cooldown, recovers one suppressed frontier.

**When to use**: Diagnostic / fallback for cooldown starvation scenarios.

**When not to use**: Default — the 120s cooldown is by design to prevent re-selecting recently visited areas.

**Config**: `cooldown_starvation_recovery_enabled: false` (disabled in default).

### 2. Forced Escape Probe

**Status**: Implemented, validated (Y-world 2/3, T-junction 3/3), NOT selected as default.

**What it does**: When escape mode is active and all candidates are inside oscillillation radius, generates a forced probe point outside the zone.

**When to use**: Diagnostic / fallback for all-inside oscillillation scenarios.

**When not to use**: Default — escape mode itself doesn't improve completion/runtime on non-straight topologies.

**Config**: `entrance_oscillation_force_escape_probe: false` (disabled in default).

### 3. Wall Risk Weak

**Status**: Evaluated (60% vs 20% baseline on Y-world), NOT selected as default.

**What it does**: Applies wall-risk penalty to frontier scoring.

**When to use**: If wall-safety metrics show clear benefit.

**When not to use**: Default — no completion/runtime improvement on non-straight topologies.

**Config**: `weight_wall_risk: 0.0` (disabled in default).

### 4. Oscillation Escape Response

**Status**: Evaluated (forced_escape 3/3, wall_risk 3/3 on non-straight), NOT selected as default.

**What it does**: Penalizes candidates near oscillillation center, forces probe outside zone.

**When not to use**: Default — escape mode doesn't improve completion/runtime.

**Config**: `entrance_oscillation_response_enabled: false` (disabled in default).

## Summary

All optional modules are **implemented and validated** but **not selected as final default**. They remain available as diagnostic/fallback tools for specific scenarios.

The final default strategy is: **baseline + startup_bootstrap + cooldown_loop_guard**.
