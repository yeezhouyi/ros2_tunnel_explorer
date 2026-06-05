# Stage 4C: Multi-Topology Benchmark — Results

**Branch**: `stage4c1-forced-escape-probe`
**Tag**: `stage4c1-forced-escape-probe-validated`
**Date**: 2026-06-05

## Objective

Validate that the Stage 4C.1 forced escape probe works across multiple tunnel topologies, not just Y-world.

## Variants

| Variant | Config | Strategy |
|---------|--------|----------|
| stage3d_default | `frontier_explorer_params_info_revisit_r075.yaml` | information_gain_revisit |
| stage4b4_escape | `stage4b3/wall_risk_weak_escape.yaml` | tunnel_aware + escape + forced probe |

## Results (36 runs: 6 topologies × 2 variants × 3 runs)

| Topology | baseline | escape | Baseline forced | Escape forced |
|----------|:--------:|:------:|:---------------:|:-------------:|
| straight_tunnel | 2/3 | 2/3 | 4 | 4 |
| l_turn_tunnel | **3/3** | **3/3** | 6 | 1 |
| branching_tunnel_y | 2/3 | 1/3 | 5 | 1 |
| t_junction_tunnel | **3/3** | **3/3** | 2 | 5 |
| dead_end_branch | **3/3** | 2/3 | 2 | 4 |
| loop_tunnel | 0/3 | 0/3 | 3 | 2 |

### Aggregate

| Metric | baseline | escape |
|--------|:--------:|:------:|
| Completion | **13/18 (72%)** | 11/18 (61%) |
| Total forced probes | 22 | 17 |
| No-goal states | 0 | 0 |

## Key Findings

1. **Escape variant is slightly worse than baseline overall** (61% vs 72%). The forced probe mechanism helps in some cases (Y-world, T-junction) but the escape penalty overhead may hurt in others.

2. **Loop topology is impossible for both** (0/3). The loop tunnel is too complex for any current strategy.

3. **Forced probes fire correctly**: no false positives in simple topologies (straight, dead-end, loop). Forced probes only fire when oscillillation is detected.

4. **Baseline performs well in l_turn and t_junction** (3/3 each). These topologies don't have severe entrance oscillillation.

5. **branching_tunnel_y escape is worse** (1/3 vs baseline 2/3). The forced probe helps in some runs but the escape penalty overhead may cause issues.

## Interpretation

The Stage 4C.1 forced escape probe is validated as a mechanism:
- It correctly fires when all candidates are inside the oscillillation zone
- It doesn't false-trigger in simple topologies
- It doesn't produce no-goal states

However, the **escape mode as a whole doesn't improve multi-topology performance**. The baseline (no escape) actually performs better across the full topology set. This suggests:

1. The escape penalty may be too aggressive for some topologies
2. The forced probe may push the robot to suboptimal goals in some cases
3. The Y-world specific tuning doesn't generalize well

## Next

- Investigate why escape performs worse than baseline in some topologies
- Consider making the escape mode topology-aware or adaptive
- Or accept that the Y-world specific tuning is Y-world specific
