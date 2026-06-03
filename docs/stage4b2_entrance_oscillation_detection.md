# Stage 4B.2: Entrance Oscillation Detection — Results

**Branch**: `stage4b2-entrance-oscillation-detection`
**Tag**: `stage4b2-entrance-oscillation-detection`
**Date**: 2026-06-03

## Objective

Make entrance-area oscillation observable and quantifiable. Stage 4B.1 revealed
that the loop detector registered 0 in all 20 runs despite clear entrance-goal
switching patterns in TIMEOUT runs. Stage 4B.2 introduces a dedicated detector
that captures this failure mode as a logged event.

## Detector Design

`EntranceOscillationDetector` is a pure C++ class (no ROS dependencies) that
tracks a sliding window of recent goal dispatch events and checks for
localized, stagnant, revisit-heavy behavior.

### Input

Each goal dispatch creates a `GoalDispatchEvent`:
```cpp
struct GoalDispatchEvent {
  Point2D goal;          // selected goal position
  Point2D robot;         // robot position at dispatch
  double stamp_seconds;  // timestamp
  int selected_bin;      // spatial bin index
  double revisit_ratio;  // normalized revisit penalty
  int unique_bins;       // unique bins in current window
};
```

### Output

```cpp
struct OscillationStatus {
  bool oscillating;           // true if 3/4 conditions met
  int event_count;            // cumulative oscillation events
  double spatial_radius_m;    // max pairwise distance in window
  int repeated_goal_count;    // goals in most frequent bin
  int stagnant_bin_count;     // same as repeated_goal_count
  std::string reason;         // human-readable condition list
};
```

### Trigger Conditions (3 out of 4 required)

| # | Condition | Threshold | Interpretation |
|---|-----------|-----------|----------------|
| 1 | Spatial radius < `radius_m` | 2.0 m | Goals clustered in small area |
| 2 | Unique bins ≤ `max_unique_bins` | 2 | Not exploring new areas |
| 3 | Bin dominance > `window_goals / 2` | >3 of 6 | One bin dominates the window |
| 4 | Avg revisit ratio ≥ `min_revisit_ratio` | 0.35 | High revisit penalty |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `entrance_oscillation_enabled` | true | Enable/disable detector |
| `entrance_oscillation_window_goals` | 6 | Sliding window size |
| `entrance_oscillation_radius_m` | 2.0 | Max spatial radius for clustering |
| `entrance_oscillation_min_repeated_goals` | 4 | Min goals before checking |
| `entrance_oscillation_max_unique_bins` | 2 | Max unique bins for stagnation |
| `entrance_oscillation_min_revisit_ratio` | 0.35 | Min avg revisit for high_revisit |
| `entrance_oscillation_min_goals_to_check` | 4 | Min events before evaluation |

## Integration Points

- **Goal dispatch** (both normal goals and recovery probes): creates
  `GoalDispatchEvent` and calls `recordGoal()` + `evaluate()`
- **Logging**: `[EntranceOscillation]` WARN when oscillation detected
- **No behavioral changes**: detection and logging only — no intervention

## Smoke Test Results

3-run smoke test on Y-world with `information_gain_revisit` strategy:

| Run | Status | Goals | Revisit | Oscillation Events |
|-----|--------|:-----:|:-------:|:------------------:|
| 01 | COMPLETED | 9 | 44.4% | **1** |
| 02 | COMPLETED | 8 | 37.5% | 0 |
| 03 | COMPLETED | 9 | 33.3% | 0 |

### Run 01: Oscillation Detected

The detector fired once at goal 6, correctly identifying entrance oscillation:

```
[EntranceOscillation] detected=true count=1 window=6 radius=0.86m
unique_bins=2 revisit=1.00 reason=localized_goals+stagnant_bins+bin_dominance+high_revisit
```

All 4 conditions were met:
- Goals within 0.86m radius (well below 2.0m threshold)
- Only 2 unique bins (at threshold)
- One bin dominated (4 of 6 goals)
- 100% revisit ratio (well above 0.35)

The robot was alternating between two entrance goals with increasing revisit
counts (0→0→1→1→2→3). After the detection, the robot broke through to (0.05, 2.02)
at goal 7 and continued exploring.

### Runs 02-03: Clean Completion

No oscillation events detected. The robot explored cleanly without entrance-area
goal switching.

## Interpretation

The detector successfully captures the entrance oscillation failure mode without
obvious false positives:

- **True positive**: run 01 had clear entrance-goal switching (6 goals, 2 bins,
  100% revisit) → detector fired
- **True negatives**: runs 02-03 completed cleanly → detector stayed quiet
- **No spam**: single WARN per oscillation episode, not per-goal
- **Recovery probes**: still dispatched normally (2 in run 01, 2 in run 03)

The detector provides a reliable trigger condition for future adaptive responses
(Stage 4B.3).

## Unit Tests

8/8 gtest pass:
- `NoOscillationWithProgress` — wide exploration → no trigger
- `DetectsLocalizedRepeatedGoals` — clustered goals → trigger
- `DoesNotTriggerOnWideExploration` — spread goals → no trigger
- `RequiresMinimumWindow` — too few events → no trigger
- `ResetsAfterProgress` — reset clears state
- `WindowTrimsToMaxSize` — deque maintains window size
- `GetCurrentUniqueBins` — correct bin counting
- `EventCountIncrements` — cumulative event counting

## Next

- **Stage 4B.3**: Adaptive response — temporary wall-risk increase,
  entrance-local goal suppression, earlier recovery probe triggering
  when oscillation is detected
