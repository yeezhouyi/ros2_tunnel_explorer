# Final Default Strategy

## Configuration

```yaml
selection_strategy: information_gain_revisit
startup_bootstrap_enabled: true
cooldown_starvation_recovery_enabled: false
entrance_oscillation_response_enabled: false
entrance_oscillation_force_escape_probe: false
```

## Components

### 1. Information-Gain Frontier Selection (Stage 2B)

Score formula:
```
score = w_gain * normalized_gain - w_dist * normalized_dist - w_revisit * normalized_revisit
```

Parameters: w_gain=1.0, w_dist=1.0, w_revisit=1.5

### 2. Startup Bootstrap (Stage 4E.2)

When no frontiers exist at startup and goals=0, dispatch a short bootstrap probe to move the robot and create frontiers.

### 3. Cooldown Loop Guard (Stage 4D.2)

When cooldown recovery targets the same frontier repeatedly and no progress is made, give up after max_attempts to prevent infinite recovery loops.

## Why Not Forced Escape

- Validated on Y-world (2/3) and T-junction (3/3)
- But slower runtime (154s vs 134s baseline)
- More recovery activations
- Doesn't improve completion on non-straight topologies

## Why Not Wall Risk Weak

- Validated in Stage 4B.1 (60% vs 20% baseline on Y-world)
- But no completion/runtime improvement on non-straight topologies in corrected benchmark
- Safety metrics not yet evaluated

## Validation

Stage 4G.1: 20/20 completed, all goals=1, no fake completions
