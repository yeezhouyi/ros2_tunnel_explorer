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

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "tunnel_frontier_explorer/frontier_scorer.hpp"
#include "tunnel_frontier_explorer/frontier_visit_history.hpp"

namespace tunnel_frontier_explorer
{

// ── Helpers ─────────────────────────────────────────────────────────────────

/// Build a map with known dimensions.  All cells default to -1 (unknown).
GridMap makeMap(std::size_t w, std::size_t h, double res = 0.1,
                double ox = 0.0, double oy = 0.0)
{
  GridMap m;
  m.width = w;
  m.height = h;
  m.resolution = res;
  m.origin_x = ox;
  m.origin_y = oy;
  m.data.assign(w * h, -1);  // all unknown
  return m;
}

/// Build a simple cluster with its representative at (row, col).
/// The cluster contains a single cell — the representative itself.
FrontierCluster makeCluster(int row, int col, const GridMap & map)
{
  FrontierCluster c;
  GridCell cell;
  cell.row = row;
  cell.col = col;
  c.cells = {cell};
  c.representative_cell = cell;
  c.centroid_world = {
    map.origin_x + (static_cast<double>(col) + 0.5) * map.resolution,
    map.origin_y + (static_cast<double>(row) + 0.5) * map.resolution};
  c.representative_world = c.centroid_world;
  c.goal_distance_to_robot = 5.0;  // default
  return c;
}

// ════════════════════════════════════════════════════════════════════════════
// FrontierScorerConfig validation
// ════════════════════════════════════════════════════════════════════════════

TEST(FrontierScorerTest, configRejectsZeroRadius)
{
  FrontierScorerConfig cfg;
  cfg.information_gain_radius_meters = 0.0;
  EXPECT_THROW(FrontierScorer{cfg}, std::invalid_argument);
}

TEST(FrontierScorerTest, configRejectsNegativeRadius)
{
  FrontierScorerConfig cfg;
  cfg.information_gain_radius_meters = -0.1;
  EXPECT_THROW(FrontierScorer{cfg}, std::invalid_argument);
}

TEST(FrontierScorerTest, configRejectsZeroRevisitRadius)
{
  FrontierScorerConfig cfg;
  cfg.revisit_radius_meters = 0.0;
  EXPECT_THROW(FrontierScorer{cfg}, std::invalid_argument);
}

TEST(FrontierScorerTest, configRejectsZeroMaxRevisitCount)
{
  FrontierScorerConfig cfg;
  cfg.max_revisit_count = 0;
  EXPECT_THROW(FrontierScorer{cfg}, std::invalid_argument);
}

TEST(FrontierScorerTest, configRejectsNegativeWeight)
{
  FrontierScorerConfig cfg;
  cfg.weight_information_gain = -0.1;
  EXPECT_THROW(FrontierScorer{cfg}, std::invalid_argument);
}

TEST(FrontierScorerTest, configAcceptsZeroWeight)
{
  FrontierScorerConfig cfg;  // all defaults
  cfg.weight_information_gain = 0.0;
  cfg.weight_distance = 0.0;
  cfg.weight_revisit = 0.0;
  EXPECT_NO_THROW(FrontierScorer{cfg});
}

// ════════════════════════════════════════════════════════════════════════════
// countUnknownCellsInRadius (tested indirectly via scoreAndRank)
// ════════════════════════════════════════════════════════════════════════════

TEST(FrontierScorerTest, singleCandidate)
{
  // Map 10x10, all unknown.  One candidate at (5,5) with distance 5.
  auto map = makeMap(10, 10, 0.1);
  // Mark a few free cells around the robot area so it's not all unknown.
  for (int r = 0; r < 10; ++r) {
    for (int c = 0; c < 10; ++c) {
      map.data[static_cast<std::size_t>(r) * 10 + static_cast<std::size_t>(c)] = 0;
    }
  }
  // Set a small patch as frontier (free cells adjacent to unknown).
  // Actually for scoring we just need free cells with unknown near them.
  // Let's set a patch of unknown cells at (3,3)-(3,5).
  map.data[3 * 10 + 3] = -1;
  map.data[3 * 10 + 4] = -1;
  map.data[3 * 10 + 5] = -1;

  auto cluster = makeCluster(5, 5, map);
  cluster.goal_distance_to_robot = 1.0;

  FrontierScorerConfig cfg;
  cfg.information_gain_radius_meters = 0.5;  // ~5 cells at 0.1 res
  FrontierScorer scorer(cfg);
  FrontierVisitHistory empty_history;

  auto scored = scorer.scoreAndRank({cluster}, map, {0.0, 0.0}, empty_history);

  EXPECT_EQ(scored.size(), 1u);
  EXPECT_GE(scored[0].score, 0.0);
}

TEST(FrontierScorerTest, higherGainWins)
{
  // Two candidates at same distance, same revisit — the one with more unknown
  // cells in radius should win.
  auto map = makeMap(20, 20, 0.1);
  // Fill everything as free.
  map.data.assign(20 * 20, 0);

  // Candidate A (row=5, col=5): 5 unknown cells to its right.
  for (int c = 6; c <= 10; ++c) {
    map.data[5 * 20 + c] = -1;
  }
  // Candidate B (row=15, col=5): 2 unknown cells to its right.
  for (int c = 6; c <= 7; ++c) {
    map.data[15 * 20 + c] = -1;
  }

  auto ca = makeCluster(5, 5, map);
  auto cb = makeCluster(15, 5, map);
  ca.goal_distance_to_robot = 5.0;
  cb.goal_distance_to_robot = 5.0;

  FrontierScorerConfig cfg;
  cfg.information_gain_radius_meters = 0.5;
  cfg.weight_distance = 0.0;   // ignore distance
  cfg.weight_revisit = 0.0;    // ignore revisit
  FrontierScorer scorer(cfg);
  FrontierVisitHistory empty_history;

  auto scored = scorer.scoreAndRank(
    {ca, cb}, map, {0.0, 0.0}, empty_history);

  ASSERT_EQ(scored.size(), 2u);
  EXPECT_GT(scored[0].raw_information_gain, scored[1].raw_information_gain);
}

TEST(FrontierScorerTest, closerDistanceWins)
{
  // Same gain, same revisit — closer candidate should win.
  auto map = makeMap(20, 20, 0.1);
  map.data.assign(20 * 20, 0);  // all free (0 gain)
  // Both clusters are identical in gain (0 unknown), different distance.

  auto ca = makeCluster(2, 2, map);   // near robot
  auto cb = makeCluster(18, 18, map);  // far
  ca.goal_distance_to_robot = 0.3;
  cb.goal_distance_to_robot = 2.5;

  FrontierScorerConfig cfg;
  cfg.weight_information_gain = 0.0;  // zero gain weight
  cfg.weight_revisit = 0.0;
  FrontierScorer scorer(cfg);
  FrontierVisitHistory empty_history;

  auto scored = scorer.scoreAndRank(
    {ca, cb}, map, {0.0, 0.0}, empty_history);

  ASSERT_EQ(scored.size(), 2u);
  EXPECT_LT(scored[0].goal_distance_meters, scored[1].goal_distance_meters);
}

TEST(FrontierScorerTest, lowerRevisitWins)
{
  // Same gain, same distance — candidate with fewer revisits wins.
  auto map = makeMap(20, 20, 0.1);
  map.data.assign(20 * 20, 0);  // all free (0 gain)

  auto ca = makeCluster(5, 5, map);
  auto cb = makeCluster(15, 5, map);
  ca.goal_distance_to_robot = 5.0;
  cb.goal_distance_to_robot = 5.0;

  // Record a visit near A but not near B.
  FrontierVisitHistory history;
  Point2D near_a;
  near_a.x = ca.representative_world.x + 0.01;
  near_a.y = ca.representative_world.y + 0.01;
  history.recordAcceptedGoal(near_a);

  FrontierScorerConfig cfg;
  cfg.revisit_radius_meters = 0.3;
  cfg.weight_information_gain = 0.0;
  cfg.weight_distance = 0.0;
  cfg.weight_revisit = 1.0;
  FrontierScorer scorer(cfg);

  auto scored = scorer.scoreAndRank(
    {ca, cb}, map, {0.0, 0.0}, history);

  ASSERT_EQ(scored.size(), 2u);
  EXPECT_EQ(scored[0].raw_revisit_count, 0u);  // no revisit → higher score
  EXPECT_EQ(scored[1].raw_revisit_count, 1u);
}

TEST(FrontierScorerTest, unknownCellDedup)
{
  // Overlapping radii should not double-count cells.
  auto map = makeMap(10, 10, 0.1);
  map.data.assign(10 * 10, 0);   // all free

  // One unknown cell at (5,6) within radius of both.
  map.data[5 * 10 + 6] = -1;

  auto ca = makeCluster(5, 5, map);
  auto cb = makeCluster(5, 5, map);  // same position
  ca.goal_distance_to_robot = 3.0;
  cb.goal_distance_to_robot = 3.0;

  FrontierScorerConfig cfg;
  cfg.information_gain_radius_meters = 0.5;
  FrontierScorer scorer(cfg);
  FrontierVisitHistory empty_history;

  auto scored = scorer.scoreAndRank(
    {ca, cb}, map, {0.0, 0.0}, empty_history);

  ASSERT_EQ(scored.size(), 2u);
  // Each has the same single unknown cell — no dedup issue across cands.
  // This test validates that per-candidate counting is correct.
  EXPECT_EQ(scored[0].raw_information_gain, 1u);
  EXPECT_EQ(scored[1].raw_information_gain, 1u);
}

TEST(FrontierScorerTest, occupiedNotCounted)
{
  auto map = makeMap(10, 10, 0.1);
  map.data.assign(10 * 10, 0);   // all free

  auto cluster = makeCluster(5, 5, map);
  cluster.goal_distance_to_robot = 3.0;

  // Set occupied cells (value 100) within the radius.
  map.data[5 * 10 + 6] = 100;
  map.data[5 * 10 + 7] = 100;

  FrontierScorerConfig cfg;
  cfg.information_gain_radius_meters = 0.5;
  FrontierScorer scorer(cfg);
  FrontierVisitHistory empty_history;

  auto scored = scorer.scoreAndRank(
    {cluster}, map, {0.0, 0.0}, empty_history);

  ASSERT_EQ(scored.size(), 1u);
  EXPECT_EQ(scored[0].raw_information_gain, 0u);
}

TEST(FrontierScorerTest, freeNotCounted)
{
  auto map = makeMap(10, 10, 0.1);
  map.data.assign(10 * 10, 0);   // all free (no unknown)

  auto cluster = makeCluster(5, 5, map);
  cluster.goal_distance_to_robot = 3.0;

  FrontierScorerConfig cfg;
  cfg.information_gain_radius_meters = 0.5;
  FrontierScorer scorer(cfg);
  FrontierVisitHistory empty_history;

  auto scored = scorer.scoreAndRank(
    {cluster}, map, {0.0, 0.0}, empty_history);

  ASSERT_EQ(scored.size(), 1u);
  EXPECT_EQ(scored[0].raw_information_gain, 0u);
}

TEST(FrontierScorerTest, originResolutionConversion)
{
  // Non-zero origin and resolution.
  auto map = makeMap(20, 20, 0.05, -2.0, -1.0);
  map.data.assign(20 * 20, 0);
  // Unknown cell at grid (10,10) = world (-2 + 10.5*0.05, -1 + 10.5*0.05)
  map.data[10 * 20 + 10] = -1;

  auto cluster = makeCluster(5, 5, map);
  cluster.goal_distance_to_robot = 5.0;

  FrontierScorerConfig cfg;
  cfg.information_gain_radius_meters = 0.5;
  FrontierScorer scorer(cfg);
  FrontierVisitHistory empty_history;

  // Should not crash and should find the unknown cell.
  auto scored = scorer.scoreAndRank(
    {cluster}, map, {0.0, 0.0}, empty_history);

  ASSERT_EQ(scored.size(), 1u);
  EXPECT_EQ(scored[0].raw_information_gain, 1u);
}

TEST(FrontierScorerTest, normaliseNoDivisionByZero)
{
  // All candidates same distance and same gain — should not crash.
  auto map = makeMap(10, 10, 0.1);
  map.data.assign(10 * 10, 0);  // all free → all gain = 0

  auto ca = makeCluster(2, 2, map);
  auto cb = makeCluster(3, 3, map);
  ca.goal_distance_to_robot = 5.0;
  cb.goal_distance_to_robot = 5.0;

  FrontierScorerConfig cfg;
  cfg.weight_information_gain = 1.0;
  cfg.weight_distance = 1.0;
  cfg.weight_revisit = 0.0;
  FrontierScorer scorer(cfg);
  FrontierVisitHistory empty_history;

  EXPECT_NO_THROW(
    scorer.scoreAndRank({ca, cb}, map, {0.0, 0.0}, empty_history));
}

TEST(FrontierScorerTest, revisitClampToMax)
{
  auto map = makeMap(10, 10, 0.1);
  map.data.assign(10 * 10, 0);

  auto cluster = makeCluster(5, 5, map);
  cluster.goal_distance_to_robot = 5.0;

  // Record many visits at the same location.
  FrontierVisitHistory history;
  for (int i = 0; i < 10; ++i) {
    history.recordAcceptedGoal(cluster.representative_world);
  }

  FrontierScorerConfig cfg;
  cfg.revisit_radius_meters = 0.5;
  cfg.max_revisit_count = 3;
  cfg.weight_information_gain = 0.0;
  cfg.weight_distance = 0.0;
  cfg.weight_revisit = 1.0;
  FrontierScorer scorer(cfg);

  auto scored = scorer.scoreAndRank(
    {cluster}, map, {0.0, 0.0}, history);

  ASSERT_EQ(scored.size(), 1u);
  EXPECT_EQ(scored[0].clamped_revisit_count, 3u);
  EXPECT_EQ(scored[0].normalized_revisit_penalty, 1.0);
}

TEST(FrontierScorerTest, emptyCandidates)
{
  auto map = makeMap(10, 10, 0.1);
  FrontierScorerConfig cfg;
  FrontierScorer scorer(cfg);
  FrontierVisitHistory empty_history;

  auto scored = scorer.scoreAndRank({}, map, {0.0, 0.0}, empty_history);
  EXPECT_TRUE(scored.empty());
}

TEST(FrontierScorerTest, circularRadiusBoundary)
{
  // Cells inside bounding box but outside circular radius should not be
  // counted.
  auto map = makeMap(20, 20, 1.0);  // 1 m resolution for easy math
  map.data.assign(20 * 20, 0);

  // Candidate at (10, 10).
  auto cluster = makeCluster(10, 10, map);
  cluster.goal_distance_to_robot = 5.0;

  // One unknown at (10, 11) — 1 m away, within radius=1.5.
  // One unknown at (10, 13) — 3 m away, outside radius=1.5.
  // One unknown at (7, 10)  — 3 m away, outside.
  map.data[10 * 20 + 11] = -1;
  map.data[10 * 20 + 13] = -1;
  map.data[7 * 20 + 10] = -1;

  FrontierScorerConfig cfg;
  cfg.information_gain_radius_meters = 1.5;
  FrontierScorer scorer(cfg);
  FrontierVisitHistory empty_history;

  auto scored = scorer.scoreAndRank(
    {cluster}, map, {0.0, 0.0}, empty_history);

  ASSERT_EQ(scored.size(), 1u);
  EXPECT_EQ(scored[0].raw_information_gain, 1u);  // only (10,11)
}

TEST(FrontierScorerTest, mapBoundaryDoesNotCrash)
{
  // Candidate at corner — radius extends beyond map boundary.
  auto map = makeMap(5, 5, 1.0);
  map.data.assign(5 * 5, 0);

  auto cluster = makeCluster(0, 0, map);  // top-left corner
  cluster.goal_distance_to_robot = 3.0;

  FrontierScorerConfig cfg;
  cfg.information_gain_radius_meters = 10.0;  // huge radius
  FrontierScorer scorer(cfg);
  FrontierVisitHistory empty_history;

  EXPECT_NO_THROW(
    scorer.scoreAndRank({cluster}, map, {0.0, 0.0}, empty_history));
}

TEST(FrontierScorerTest, tieBreakSameScoreDifferentDistance)
{
  // Same score but different distance — closer should rank first.
  auto map = makeMap(20, 20, 0.1);
  map.data.assign(20 * 20, 0);

  auto ca = makeCluster(2, 2, map);
  auto cb = makeCluster(18, 18, map);
  ca.goal_distance_to_robot = 0.3;
  cb.goal_distance_to_robot = 2.5;

  FrontierScorerConfig cfg;
  cfg.weight_information_gain = 0.0;
  cfg.weight_distance = 0.0;   // distance doesn't affect SCORE
  cfg.weight_revisit = 0.0;
  FrontierScorer scorer(cfg);
  FrontierVisitHistory empty_history;

  // All scores = 0 — tie-break by distance.
  auto scored = scorer.scoreAndRank(
    {cb, ca}, map, {0.0, 0.0}, empty_history);
  // ^^ cb (far) inserted first in input; scoring should reorder by distance.

  ASSERT_EQ(scored.size(), 2u);
  EXPECT_LT(scored[0].goal_distance_meters, scored[1].goal_distance_meters);
}

TEST(FrontierScorerTest, tieBreakStableRowCol)
{
  // Same score, same distance, same gain — tie-break by row then col.
  auto map = makeMap(10, 10, 0.1);
  map.data.assign(10 * 10, 0);

  auto c1 = makeCluster(5, 2, map);  // row 5, col 2
  auto c2 = makeCluster(3, 4, map);  // row 3, col 4 (smaller row → should win)
  c1.goal_distance_to_robot = 3.0;
  c2.goal_distance_to_robot = 3.0;

  FrontierScorerConfig cfg;
  cfg.weight_information_gain = 0.0;
  cfg.weight_distance = 0.0;
  cfg.weight_revisit = 0.0;
  FrontierScorer scorer(cfg);
  FrontierVisitHistory empty_history;

  auto scored = scorer.scoreAndRank(
    {c1, c2}, map, {0.0, 0.0}, empty_history);

  ASSERT_EQ(scored.size(), 2u);
  // c2 has row=3, c1 has row=5 — c2 should be first.
  EXPECT_LT(scored[0].cluster.representative_cell.row,
            scored[1].cluster.representative_cell.row);
}

}  // namespace tunnel_frontier_explorer
