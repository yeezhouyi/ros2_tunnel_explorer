// Copyright 2026 zhouyi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include "tunnel_frontier_explorer/entrance_oscillation_detector.hpp"

using tunnel_frontier_explorer::EntranceOscillationConfig;
using tunnel_frontier_explorer::EntranceOscillationDetector;
using tunnel_frontier_explorer::GoalDispatchEvent;

static GoalDispatchEvent makeEvent(double x, double y, int bin, double revisit = 0.0)
{
  GoalDispatchEvent e;
  e.goal = {x, y};
  e.robot = {0.0, 0.0};
  e.stamp_seconds = 0.0;
  e.selected_bin = bin;
  e.revisit_ratio = revisit;
  e.unique_bins = 0;
  return e;
}

// ── Test 1: No oscillation with progress (wide exploration) ─────────────

TEST(EntranceOscillation, NoOscillationWithProgress)
{
  EntranceOscillationConfig cfg;
  cfg.window_goals = 6;
  cfg.min_goals_to_check = 4;
  EntranceOscillationDetector det(cfg);

  // Goals spread across 3 different bins, large radius
  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(3.0, 3.0, 1, 0.0));
  det.recordGoal(makeEvent(6.0, 0.0, 2, 0.0));
  det.recordGoal(makeEvent(0.0, 6.0, 3, 0.0));

  auto status = det.evaluate();
  EXPECT_FALSE(status.oscillating);
}

// ── Test 2: Detects localized repeated goals ────────────────────────────

TEST(EntranceOscillation, DetectsLocalizedRepeatedGoals)
{
  EntranceOscillationConfig cfg;
  cfg.window_goals = 6;
  cfg.radius_m = 2.0;
  cfg.min_goals_to_check = 4;
  cfg.max_unique_bins = 2;
  cfg.min_revisit_ratio = 0.35;
  EntranceOscillationDetector det(cfg);

  // All goals within 1.0m radius, same bin, high revisit
  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.5));
  det.recordGoal(makeEvent(0.3, 0.2, 0, 0.6));
  det.recordGoal(makeEvent(0.1, 0.4, 0, 0.7));
  det.recordGoal(makeEvent(0.2, 0.1, 0, 0.4));

  auto status = det.evaluate();
  EXPECT_TRUE(status.oscillating);
  EXPECT_LE(status.spatial_radius_m, cfg.radius_m);
  EXPECT_TRUE(status.reason.find("localized_goals") != std::string::npos);
  EXPECT_TRUE(status.reason.find("stagnant_bins") != std::string::npos);
  EXPECT_TRUE(status.reason.find("high_revisit") != std::string::npos);
}

// ── Test 3: Does not trigger on wide exploration ────────────────────────

TEST(EntranceOscillation, DoesNotTriggerOnWideExploration)
{
  EntranceOscillationConfig cfg;
  cfg.window_goals = 6;
  cfg.radius_m = 2.0;
  cfg.min_goals_to_check = 4;
  EntranceOscillationDetector det(cfg);

  // Goals far apart, different bins
  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(5.0, 5.0, 5, 0.0));
  det.recordGoal(makeEvent(10.0, 0.0, 10, 0.0));
  det.recordGoal(makeEvent(0.0, 10.0, 15, 0.0));

  auto status = det.evaluate();
  EXPECT_FALSE(status.oscillating);
  EXPECT_GT(status.spatial_radius_m, cfg.radius_m);
}

// ── Test 4: Requires minimum window ─────────────────────────────────────

TEST(EntranceOscillation, RequiresMinimumWindow)
{
  EntranceOscillationConfig cfg;
  cfg.window_goals = 6;
  cfg.min_goals_to_check = 4;
  EntranceOscillationDetector det(cfg);

  // Only 3 goals — below min_goals_to_check
  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.8));
  det.recordGoal(makeEvent(0.1, 0.1, 0, 0.9));
  det.recordGoal(makeEvent(0.2, 0.2, 0, 0.7));

  auto status = det.evaluate();
  EXPECT_FALSE(status.oscillating);
  EXPECT_EQ(det.windowSize(), 3u);
}

// ── Test 5: Resets after progress ───────────────────────────────────────

TEST(EntranceOscillation, ResetsAfterProgress)
{
  EntranceOscillationConfig cfg;
  cfg.window_goals = 6;
  cfg.min_goals_to_check = 4;
  EntranceOscillationDetector det(cfg);

  // Detect oscillation first
  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.5));
  det.recordGoal(makeEvent(0.1, 0.1, 0, 0.6));
  det.recordGoal(makeEvent(0.2, 0.2, 0, 0.7));
  det.recordGoal(makeEvent(0.3, 0.3, 0, 0.4));
  auto status = det.evaluate();
  EXPECT_TRUE(status.oscillating);
  EXPECT_EQ(det.eventCount(), 1u);

  // Reset and verify clean state
  det.reset();
  EXPECT_EQ(det.windowSize(), 0u);
  EXPECT_EQ(det.eventCount(), 0u);

  // New wide exploration should not oscillate
  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(5.0, 5.0, 5, 0.0));
  det.recordGoal(makeEvent(10.0, 0.0, 10, 0.0));
  det.recordGoal(makeEvent(0.0, 10.0, 15, 0.0));
  status = det.evaluate();
  EXPECT_FALSE(status.oscillating);
}

// ── Test 6: Window trims to max size ────────────────────────────────────

TEST(EntranceOscillation, WindowTrimsToMaxSize)
{
  EntranceOscillationConfig cfg;
  cfg.window_goals = 4;
  cfg.min_goals_to_check = 4;
  EntranceOscillationDetector det(cfg);

  for (int i = 0; i < 8; ++i) {
    det.recordGoal(makeEvent(i * 1.0, 0.0, i, 0.0));
  }

  EXPECT_EQ(det.windowSize(), 4u);
}

// ── Test 7: getCurrentUniqueBins ────────────────────────────────────────

TEST(EntranceOscillation, GetCurrentUniqueBins)
{
  EntranceOscillationConfig cfg;
  EntranceOscillationDetector det(cfg);

  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(1.0, 0.0, 1, 0.0));
  det.recordGoal(makeEvent(2.0, 0.0, 0, 0.0));

  EXPECT_EQ(det.getCurrentUniqueBins(), 2);  // bins {0, 1}
}

// ── Test 8: Oscillation count increments across evaluations ─────────────

TEST(EntranceOscillation, EventCountIncrements)
{
  EntranceOscillationConfig cfg;
  cfg.window_goals = 6;
  cfg.min_goals_to_check = 4;
  EntranceOscillationDetector det(cfg);

  // First oscillation event
  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.5));
  det.recordGoal(makeEvent(0.1, 0.1, 0, 0.6));
  det.recordGoal(makeEvent(0.2, 0.2, 0, 0.7));
  det.recordGoal(makeEvent(0.3, 0.3, 0, 0.4));
  auto s1 = det.evaluate();
  EXPECT_TRUE(s1.oscillating);
  EXPECT_EQ(det.eventCount(), 1u);

  // Second evaluation — same window, still oscillating
  auto s2 = det.evaluate();
  EXPECT_TRUE(s2.oscillating);
  EXPECT_EQ(det.eventCount(), 2u);
}

// ── Test 9: getOscillationCenter computes centroid ─────────────────────

TEST(EntranceOscillation, GetOscillationCenterComputesCentroid)
{
  EntranceOscillationConfig cfg;
  EntranceOscillationDetector det(cfg);

  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(4.0, 0.0, 1, 0.0));
  det.recordGoal(makeEvent(0.0, 4.0, 2, 0.0));
  det.recordGoal(makeEvent(4.0, 4.0, 3, 0.0));

  auto center = det.getOscillationCenter();
  EXPECT_NEAR(center.x, 2.0, 1e-9);
  EXPECT_NEAR(center.y, 2.0, 1e-9);
}

// ── Test 10: getOscillationCenter empty window ─────────────────────────

TEST(EntranceOscillation, GetOscillationCenterEmptyWindow)
{
  EntranceOscillationConfig cfg;
  EntranceOscillationDetector det(cfg);

  auto center = det.getOscillationCenter();
  EXPECT_NEAR(center.x, 0.0, 1e-9);
  EXPECT_NEAR(center.y, 0.0, 1e-9);
}

// ── Test 11: getOscillationCenter with window trim ─────────────────────

TEST(EntranceOscillation, GetOscillationCenterWithWindowTrim)
{
  EntranceOscillationConfig cfg;
  cfg.window_goals = 3;
  EntranceOscillationDetector det(cfg);

  // Add 5 goals, only last 3 remain
  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.0));  // trimmed
  det.recordGoal(makeEvent(10.0, 10.0, 1, 0.0));  // trimmed
  det.recordGoal(makeEvent(2.0, 0.0, 2, 0.0));
  det.recordGoal(makeEvent(4.0, 0.0, 3, 0.0));
  det.recordGoal(makeEvent(6.0, 0.0, 4, 0.0));

  // Center should be average of last 3: (2+4+6)/3=4.0, y=0
  auto center = det.getOscillationCenter();
  EXPECT_NEAR(center.x, 4.0, 1e-9);
  EXPECT_NEAR(center.y, 0.0, 1e-9);
}

// ── Test 12: Detects alternating pair A B A B A B ─────────────────────

TEST(EntranceOscillation, DetectsAlternatingPairABAB)
{
  EntranceOscillationConfig cfg;
  cfg.window_goals = 6;
  cfg.min_goals_to_check = 4;
  cfg.detect_alternating_pair = true;
  cfg.pair_cluster_radius_m = 0.75;
  cfg.pair_max_spatial_radius_m = 1.5;
  cfg.pair_min_cluster_count = 2;
  cfg.pair_min_alternation_score = 0.5;
  EntranceOscillationDetector det(cfg);

  // A B A B A B — two clusters ~1.0m apart, 0% revisit
  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(0.8, 0.0, 1, 0.0));
  det.recordGoal(makeEvent(0.1, 0.1, 0, 0.0));
  det.recordGoal(makeEvent(0.9, 0.1, 1, 0.0));
  det.recordGoal(makeEvent(0.0, 0.2, 0, 0.0));
  det.recordGoal(makeEvent(0.8, 0.2, 1, 0.0));

  auto status = det.evaluate();
  EXPECT_TRUE(status.oscillating);
  EXPECT_TRUE(status.reason.find("alternating_pair") != std::string::npos);
}

// ── Test 13: Detects balanced pair without revisit ────────────────────

TEST(EntranceOscillation, DetectsBalancedPairWithoutRevisit)
{
  EntranceOscillationConfig cfg;
  cfg.window_goals = 6;
  cfg.min_goals_to_check = 4;
  cfg.detect_alternating_pair = true;
  cfg.pair_cluster_radius_m = 0.75;
  cfg.pair_max_spatial_radius_m = 1.5;
  cfg.pair_min_cluster_count = 2;
  cfg.pair_min_alternation_score = 0.5;
  EntranceOscillationDetector det(cfg);

  // A A B B A B — 0% revisit, balanced
  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(0.1, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(0.8, 0.0, 1, 0.0));
  det.recordGoal(makeEvent(0.9, 0.0, 1, 0.0));
  det.recordGoal(makeEvent(0.0, 0.1, 0, 0.0));
  det.recordGoal(makeEvent(0.8, 0.1, 1, 0.0));

  auto status = det.evaluate();
  EXPECT_TRUE(status.oscillating);
  EXPECT_TRUE(status.reason.find("alternating_pair") != std::string::npos);
}

// ── Test 14: Does not detect wide pair ────────────────────────────────

TEST(EntranceOscillation, DoesNotDetectWidePair)
{
  EntranceOscillationConfig cfg;
  cfg.window_goals = 6;
  cfg.min_goals_to_check = 4;
  cfg.detect_alternating_pair = true;
  cfg.pair_cluster_radius_m = 0.75;
  cfg.pair_max_spatial_radius_m = 1.5;
  cfg.pair_min_cluster_count = 2;
  cfg.pair_min_alternation_score = 0.5;
  EntranceOscillationDetector det(cfg);

  // Two clusters 3.0m apart — exceeds pair_max_spatial_radius_m
  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(3.0, 0.0, 1, 0.0));
  det.recordGoal(makeEvent(0.1, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(3.1, 0.0, 1, 0.0));
  det.recordGoal(makeEvent(0.0, 0.1, 0, 0.0));
  det.recordGoal(makeEvent(3.0, 0.1, 1, 0.0));

  auto status = det.evaluate();
  EXPECT_FALSE(status.oscillating);
}

// ── Test 15: Does not detect single cluster ───────────────────────────

TEST(EntranceOscillation, DoesNotDetectSingleCluster)
{
  EntranceOscillationConfig cfg;
  cfg.window_goals = 6;
  cfg.min_goals_to_check = 4;
  cfg.detect_alternating_pair = true;
  cfg.pair_cluster_radius_m = 0.75;
  EntranceOscillationDetector det(cfg);

  // All goals in one cluster
  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(0.1, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(0.2, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(0.3, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(0.4, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(0.5, 0.0, 0, 0.0));

  auto status = det.evaluate();
  // Single cluster → alternating_pair not detected
  // May still be detected by Type A if revisit is high enough
  auto pair = det.detectAlternatingPair();
  EXPECT_FALSE(pair.detected);
  EXPECT_EQ(pair.cluster_count, 1);
}

// ── Test 16: Does not detect two clusters with progress ───────────────

TEST(EntranceOscillation, DoesNotDetectTwoClustersWithProgress)
{
  EntranceOscillationConfig cfg;
  cfg.window_goals = 6;
  cfg.min_goals_to_check = 4;
  cfg.detect_alternating_pair = true;
  cfg.pair_cluster_radius_m = 0.75;
  cfg.pair_max_spatial_radius_m = 2.0;
  cfg.pair_min_cluster_count = 2;
  cfg.pair_min_alternation_score = 0.5;
  EntranceOscillationDetector det(cfg);

  // Two clusters but second half has more unique bins (progress)
  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(0.1, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(0.8, 0.0, 1, 0.0));
  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.0));  // bin 0 again
  det.recordGoal(makeEvent(2.0, 0.0, 5, 0.0));  // new bin 5 in second half
  det.recordGoal(makeEvent(2.1, 0.0, 6, 0.0));  // new bin 6 in second half

  auto pair = det.detectAlternatingPair();
  EXPECT_FALSE(pair.detected);
}

// ── Test 17: Does not detect unbalanced dominance as pair ─────────────

TEST(EntranceOscillation, DoesNotDetectUnbalancedDominanceAsPair)
{
  EntranceOscillationConfig cfg;
  cfg.window_goals = 6;
  cfg.min_goals_to_check = 4;
  cfg.detect_alternating_pair = true;
  cfg.pair_cluster_radius_m = 0.75;
  cfg.pair_max_spatial_radius_m = 1.5;
  cfg.pair_min_cluster_count = 2;
  cfg.pair_min_alternation_score = 0.5;
  EntranceOscillationDetector det(cfg);

  // A A A A B B — cluster B only has 2 goals, but alternation score is low
  // A A A A B B → transitions = 1 (A→B at index 4), score = 1/5 = 0.2
  det.recordGoal(makeEvent(0.0, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(0.1, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(0.2, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(0.3, 0.0, 0, 0.0));
  det.recordGoal(makeEvent(0.8, 0.0, 1, 0.0));
  det.recordGoal(makeEvent(0.9, 0.0, 1, 0.0));

  auto pair = det.detectAlternatingPair();
  // Cluster B has 2 goals (meets min_cluster_count), but alternation=0.2 < 0.5
  EXPECT_FALSE(pair.detected);
}
