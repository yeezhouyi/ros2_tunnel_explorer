# Stage 2C: Revisit Radius 0.50 → 0.75 Single-Variable Experiment

## Motivation

Stage 2B v1 (information_gain_revisit, revisit_radius=0.50 m) completed with:

| Metric | Stage 2B v1 |
|--------|:-----------:|
| Completion rate | 100 % |
| TTC median | 156.0 s |
| TTC max | 387 s |
| Revisit median | 0 % |
| Revisit max | 65 % |
| Goals max | 20 |
| Navigation success | 100 % |

### Run 4 Outlier Diagnosis

Run 4 (387 s, 65 % revisit rate, 20 goals) exhibited a clear ping-pong cycle:

```
Bin sequence:  01212121212121213465
                ↑ 14 goals cycling between 2 bins ↑
```

| Bin | Visits | Center |
|-----|-------:|--------|
| (0.0, 0.5) | 8 | ~(0.15, 0.82) |
| (0.5, 0.0) | 7 | ~(0.55, 0.22) |

Adjacent bin distance: **~0.72 m** (just outside revisit_radius=0.50 m).

**Root cause**: revisit_radius=0.50 m treats these two adjacent, semantically
connected spatial bins as independent regions.  The robot alternates between
bin A and bin B without accumulating shared revisit history, so both bins
independently reach `max_revisit_count=3` and the scoring function loses
discriminative power with only 1 candidate available.

## Hypothesis

Increasing `revisit_radius_meters` from 0.50 to 0.75 will cause the two
cycle bins (0.72 m apart) to share revisit history.  Visiting one will
penalise the other, breaking the ping-pong cycle and reducing worst-case TTC
and revisit rate.

## Frozen Parameters

All parameters identical to Stage 2B v1 except `revisit_radius_meters`:

| Parameter | Stage 2B v1 | Stage 2C | Change |
|-----------|:-----------:|:--------:|:------:|
| `selection_strategy` | `information_gain_revisit` | same | — |
| `information_gain_radius_meters` | 0.75 | 0.75 | — |
| `revisit_radius_meters` | **0.50** | **0.75** | **+0.25** |
| `max_revisit_count` | 3 | 3 | — |
| `weight_information_gain` | 1.0 | 1.0 | — |
| `weight_distance` | 1.0 | 1.0 | — |
| `weight_revisit` | 1.5 | 1.5 | — |
| `goal_timeout_seconds` | 60.0 | 60.0 | — |
| `cooldown_seconds` | 5.0 | 5.0 | — |
| `min_goal_distance_meters` | 0.50 | 0.50 | — |

All Nav2, SLAM, costmap, RotationShimController, and DWB parameters frozen.

## Risk: Over-Penalty

Increasing revisit radius may penalise legitimate nearby frontiers, forcing
the robot to choose farther goals and increasing path length.  0.75 m was
chosen because:
- The cycle bins are ~0.72 m apart — 0.75 m just barely covers them
- This is the smallest increment likely to affect the observed pattern
- A larger jump (e.g. 1.0 m) risks over-penalty without evidence

## Smoke Test Protocol

1. Launch simulation:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ./scripts/cleanup_simulation.sh
   ros2 launch tunnel_explorer_bringup \
     stage0_simulation.launch.py \
     headless:=True use_composition:=False \
     params_file:="$(ros2 pkg prefix --share tunnel_explorer_bringup)/config/nav2_params_rotation_shim.yaml"
   ```

2. Wait for Nav2:
   ```bash
   ./scripts/wait_for_nav2_active.sh --timeout-seconds 120
   ```

3. Launch explorer with Stage 2C params:
   ```bash
   ros2 launch tunnel_frontier_explorer \
     frontier_explorer.launch.py \
     params_file:="$(ros2 pkg prefix --share tunnel_frontier_explorer)/config/frontier_explorer_params_info_revisit_r075.yaml"
   ```

4. Smoke pass criteria:
   - 0.72 m adjacent goals share revisit history (raw_revisit_count > 0 on alternation)
   - No permanent ping-pong cycle
   - nav success ≥ 95 %
   - No crashes

## Formal 5-Run Benchmark

```bash
mkdir -p ~/stage2c_revisit_radius_075_formal

for i in $(seq -w 1 5); do
  ./scripts/run_stage2a_benchmark.sh \
    --output-dir ~/stage2c_revisit_radius_075_formal \
    --duration 600 \
    --run-id "${i}" \
    --stop-on-completed true \
    --completed-grace-seconds 20 \
    --stall-timeout-seconds 90 \
    --runtime-retries 2 \
    --explorer-params-file \
    "$(ros2 pkg prefix --share tunnel_frontier_explorer)/config/frontier_explorer_params_info_revisit_r075.yaml"
  sleep 10
done

./scripts/aggregate_stage2a_results.sh ~/stage2c_revisit_radius_075_formal
```

## Success Criteria

**Hard requirements** (must all hold):
- Completion rate = 100 %
- Navigation success rate ≥ 95 %
- TTC median no more than 10 % above Stage 2B v1 (≤ 171.6 s)

**At least one of**:
- TTC max ≥ 20 % reduction (≤ 309.6 s)
- Revisit max ≥ 30 % reduction (≤ 45.5 %)
- Goals max ≥ 20 % reduction (≤ 16)

## Expected Outcomes

| Scenario | Interpretation | Action |
|----------|---------------|--------|
| Worst-case improved, median stable | Radius=0.75 accepted | Tag, multi-scene |
| Worst-case unchanged, median stable | Saturation not radius-driven | Try max_revisit_count=5 |
| Median degraded | Revisit over-penalty | Revert to 0.50 |
| Cycling persists despite shared history | Not a radius problem | Consider turn_cost |

---

- **Date:** 2026-06-01
- **Branch:** `stage2c-revisit-radius-075`
- **Previous tag:** `stage2b-info-revisit-v1`
