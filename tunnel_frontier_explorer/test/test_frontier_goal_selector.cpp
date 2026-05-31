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

#include <cmath>
#include <vector>

#include "tunnel_frontier_explorer/frontier_goal_selector.hpp"

namespace tfe = tunnel_frontier_explorer;

// ── Helpers ──────────────────────────────────────────────────────────────

/// Build a GridMap from a 2-D initializer list.
tfe::GridMap makeGridMap(
  std::vector<std::vector<std::int8_t>> rows,
  double resolution = 1.0,
  double origin_x = 0.0,
  double origin_y = 0.0)
{
  tfe::GridMap map;
  map.height = rows.size();
  map.width = (map.height > 0) ? rows[0].size() : 0;
  map.resolution = resolution;
  map.origin_x = origin_x;
  map.origin_y = origin_y;

  map.data.clear();
  map.data.reserve(map.height * map.width);
  for (const auto & row : rows) {
    map.data.insert(map.data.end(), row.begin(), row.end());
  }
  return map;
}

tfe::FrontierGoalSelectorConfig defaultConfig()
{
  tfe::FrontierGoalSelectorConfig cfg;
  cfg.min_goal_distance_meters = 5.0;
  return cfg;
}

/// Build a 1-cell cluster at a given world position.
/// Helper: cell (row, col) maps to world using the map's origin+resolution.
tfe::FrontierCluster makeCluster(
  int row, int col,
  const tfe::GridMap & map,
  double centroid_x = 0.0, double centroid_y = 0.0)
{
  tfe::FrontierCluster c;
  c.cells.push_back({row, col});
  c.centroid_grid = {centroid_x, centroid_y};
  c.centroid_world = {
    map.origin_x + (centroid_x + 0.5) * map.resolution,
    map.origin_y + (centroid_y + 0.5) * map.resolution};
  c.representative_cell = {row, col};
  c.representative_world = {
    map.origin_x + (static_cast<double>(col) + 0.5) * map.resolution,
    map.origin_y + (static_cast<double>(row) + 0.5) * map.resolution};
  return c;
}

// ── 1. Representative goal already far enough ────────────────────────────

TEST(FrontierGoalSelectorTest, representativeFarEnough)
{
  // min_goal_distance_meters=5.0, cluster at (0, 0) world, robot at (10, 10)
  // distance = 14.14 >> 5.0, so it passes through unchanged.
  auto cfg = defaultConfig();
  tfe::FrontierGoalSelector selector(cfg);
  auto map = makeGridMap({{0, -1}, {-1, -1}});
  tfe::Point2D robot{10.0, 10.0};

  std::vector<tfe::FrontierCluster> clusters;
  clusters.push_back(makeCluster(0, 0, map));

  auto result = selector.select(clusters, map, robot);
  ASSERT_TRUE(result.selected.has_value());
  EXPECT_DOUBLE_EQ(result.selected->representative_world.x, 0.5);
  EXPECT_DOUBLE_EQ(result.selected->representative_world.y, 0.5);
  EXPECT_TRUE(result.too_close_goals.empty());
}

// ── 2. Representative too close, but alternative exists in cluster ───────

TEST(FrontierGoalSelectorTest, representativeTooCloseButAlternativeExists)
{
  // Cluster with 2 cells: (0,0) at world (0.5, 0.5) and (5,0) at world (5.5, 0.5).
  // Robot at (0, 0).  min_goal_distance=5.0.
  // Cell (0,0) dist=0.71 < 5.0, cell (5,0) dist=5.70 >= 5.0.
  auto cfg = defaultConfig();
  tfe::FrontierGoalSelector selector(cfg);
  auto map = makeGridMap(
    {{0, -1, -1, -1, -1, 0}},   // row 0: two free cells at col 0 and col 5
    1.0, 0.0, 0.0);
  tfe::Point2D robot{0.0, 0.0};

  // Manually build a cluster with 2 cells.
  tfe::FrontierCluster c;
  c.cells.push_back({0, 0});
  c.cells.push_back({0, 5});
  c.centroid_grid = {2.5, 0.0};
  c.centroid_world = {
    0.0 + (2.5 + 0.5) * 1.0,   // 3.0
    0.0 + (0.0 + 0.5) * 1.0};  // 0.5
  c.representative_cell = {0, 0};
  c.representative_world = {0.5, 0.5};  // too close!

  std::vector<tfe::FrontierCluster> clusters;
  clusters.push_back(c);

  auto result = selector.select(clusters, map, robot);
  ASSERT_TRUE(result.selected.has_value());
  // Should have switched to cell (0,5) at world (5.5, 0.5)
  EXPECT_DOUBLE_EQ(result.selected->representative_world.x, 5.5);
  EXPECT_DOUBLE_EQ(result.selected->representative_world.y, 0.5);
  EXPECT_TRUE(result.too_close_goals.empty());
}

// ── 3. All cells in cluster too close — skip ─────────────────────────────

TEST(FrontierGoalSelectorTest, allCellsTooClose)
{
  // Single cell cluster at (0,0) world (0.5, 0.5), robot at (0,0)
  // distance 0.71 < 5.0, no alternative cells → reject.
  auto cfg = defaultConfig();
  tfe::FrontierGoalSelector selector(cfg);
  auto map = makeGridMap({{0, -1}, {-1, -1}});
  tfe::Point2D robot{0.0, 0.0};

  std::vector<tfe::FrontierCluster> clusters;
  clusters.push_back(makeCluster(0, 0, map));

  auto result = selector.select(clusters, map, robot);
  EXPECT_FALSE(result.selected.has_value());
  ASSERT_EQ(result.too_close_goals.size(), 1u);
  EXPECT_DOUBLE_EQ(result.too_close_goals[0].x, 0.5);
  EXPECT_DOUBLE_EQ(result.too_close_goals[0].y, 0.5);
}

// ── 4. Multiple clusters — select nearest that passes ────────────────────

TEST(FrontierGoalSelectorTest, multipleClustersSelectNearestValid)
{
  // Cluster A at world (2, 2) → dist 2.83 < 5.0, no alternative → skip
  // Cluster B at world (8, 0) → dist 8.0 >= 5.0 → selected
  auto cfg = defaultConfig();
  tfe::FrontierGoalSelector selector(cfg);
  auto map = makeGridMap(
    {{0, -1, -1, -1, -1, -1, -1, -1, 0}},
    1.0, 0.0, 0.0);
  tfe::Point2D robot{0.0, 0.0};

  std::vector<tfe::FrontierCluster> clusters;
  // Cluster A at col 1 → world (1.5, 0.5)
  clusters.push_back(makeCluster(0, 1, map));
  // Cluster B at col 8 → world (8.5, 0.5)
  clusters.push_back(makeCluster(0, 8, map));

  auto result = selector.select(clusters, map, robot);
  ASSERT_TRUE(result.selected.has_value());
  // B should be selected despite being farther from centroid
  EXPECT_DOUBLE_EQ(result.selected->representative_world.x, 8.5);
  EXPECT_EQ(result.too_close_goals.size(), 1u);
  EXPECT_DOUBLE_EQ(result.too_close_goals[0].x, 1.5);
}

// ── 5. Distance exactly at boundary ──────────────────────────────────────

TEST(FrontierGoalSelectorTest, exactBoundary)
{
  // min_goal_distance=5.0, cluster at world (5.5, 0.0).
  // robot at (0, 0).  dist = 5.5 >= 5.0 → accepted.
  auto cfg = defaultConfig();
  tfe::FrontierGoalSelector selector(cfg);
  auto map = makeGridMap(
    {{-1, -1, -1, -1, -1, 0}},
    1.0, 0.0, 0.0);
  tfe::Point2D robot{0.0, 0.0};

  std::vector<tfe::FrontierCluster> clusters;
  clusters.push_back(makeCluster(0, 5, map));

  auto result = selector.select(clusters, map, robot);
  ASSERT_TRUE(result.selected.has_value());
  EXPECT_NEAR(result.selected->goal_distance_to_robot, 5.523, 0.001);
}

// ── 6. World coordinate conversion with non-zero origin ──────────────────

TEST(FrontierGoalSelectorTest, nonZeroOriginConversion)
{
  // Map origin at (10, 20), resolution=2.0.
  // Cell (0, 6) → world (10 + 6.5*2, 20 + 0.5*2) = (23, 21)
  // Robot far away → representative passes min distance check.
  auto cfg = defaultConfig();
  tfe::FrontierGoalSelector selector(cfg);
  auto map = makeGridMap(
    {{-1, -1, -1, -1, -1, -1, 0}},
    2.0, 10.0, 20.0);
  tfe::Point2D robot{100.0, 100.0};

  std::vector<tfe::FrontierCluster> clusters;
  clusters.push_back(makeCluster(0, 6, map));

  auto result = selector.select(clusters, map, robot);
  ASSERT_TRUE(result.selected.has_value());
  // Cell (0,6) with res=2, origin=(10,20): world = (23, 21)
  EXPECT_DOUBLE_EQ(result.selected->representative_world.x, 23.0);
  EXPECT_DOUBLE_EQ(result.selected->representative_world.y, 21.0);
}

// ── 7. Selected goal is always a frontier cell ───────────────────────────

TEST(FrontierGoalSelectorTest, selectedGoalIsFrontierCell)
{
  // Cluster with two cells: (0,0) at world (0.5, 0.5) and (0,7) at world (7.5, 0.5).
  // Robot at (0,0), min_goal_distance=5.0.
  // Original rep (0,0) is too close (0.71 < 5.0) but cell (0,7) is far enough (7.52 >= 5.0).
  auto cfg = defaultConfig();
  tfe::FrontierGoalSelector selector(cfg);
  auto map = makeGridMap(
    {{0, -1, -1, -1, -1, -1, -1, 0}},
    1.0, 0.0, 0.0);
  tfe::Point2D robot{0.0, 0.0};

  tfe::FrontierCluster c;
  c.cells = {{0, 0}, {0, 7}};
  c.centroid_grid = {3.5, 0.0};
  c.centroid_world = {4.0, 0.5};
  c.representative_cell = {0, 0};
  c.representative_world = {0.5, 0.5};  // dist 0.71 < 5.0 — too close

  std::vector<tfe::FrontierCluster> clusters;
  clusters.push_back(c);

  auto result = selector.select(clusters, map, robot);
  ASSERT_TRUE(result.selected.has_value());

  // The selected representative must be one of the original cells.
  const auto & rep = result.selected->representative_cell;
  bool found = false;
  for (const auto & cell : c.cells) {
    if (cell.row == rep.row && cell.col == rep.col) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Selected cell (" << rep.row << ", " << rep.col
                     << ") is not in the original cluster";
  // Must have switched to the far cell.
  EXPECT_EQ(rep.col, 7);
}

// ── 8. Empty cluster list → no selection ─────────────────────────────────

TEST(FrontierGoalSelectorTest, emptyClusterList)
{
  auto cfg = defaultConfig();
  tfe::FrontierGoalSelector selector(cfg);
  auto map = makeGridMap({{0, -1}, {-1, -1}});
  tfe::Point2D robot{0.0, 0.0};

  std::vector<tfe::FrontierCluster> clusters;
  auto result = selector.select(clusters, map, robot);
  EXPECT_FALSE(result.selected.has_value());
  EXPECT_TRUE(result.too_close_goals.empty());
}

// ── 9. Too-close count matches rejected clusters ─────────────────────────

TEST(FrontierGoalSelectorTest, tooCloseCountMatches)
{
  // Two clusters, both too close with no alternatives.
  auto cfg = defaultConfig();
  tfe::FrontierGoalSelector selector(cfg);
  auto map = makeGridMap(
    {{0, -1, 0}},
    1.0, 0.0, 0.0);
  tfe::Point2D robot{0.0, 0.0};

  std::vector<tfe::FrontierCluster> clusters;
  clusters.push_back(makeCluster(0, 0, map));  // world (0.5, 0.5)
  clusters.push_back(makeCluster(0, 2, map));  // world (2.5, 0.5)
  // both dist < 5.0

  auto result = selector.select(clusters, map, robot);
  EXPECT_FALSE(result.selected.has_value());
  EXPECT_EQ(result.too_close_goals.size(), 2u);
}

// ── 10. Sorting uses goal_distance_to_robot, not centroid ─────────────────

TEST(FrontierGoalSelectorTest, sortsByGoalDistance)
{
  // Two clusters both far enough.  Cluster A has centroid at (0,0) but
  // representative at (10, 0).  Cluster B has centroid at (0,10) and
  // representative at (0, 10).  Robot at (0, 0).
  //
  // If sorting by centroid: A is closer (centroid 0 vs 10).
  // If sorting by goal: B is closer (rep 10 vs 10, tie — so make it more clear).
  //
  // Let me be more concrete:
  // Cluster A: centroid at (100, 0), rep at (10, 0)  → rep goal dist = 10
  // Cluster B: centroid at (0, 50), rep at (0, 5)     → rep goal dist = 5
  //
  // Selection should pick B (lower goal distance).

  auto cfg = defaultConfig();
  cfg.min_goal_distance_meters = 1.0;   // low threshold so both pass
  tfe::FrontierGoalSelector selector(cfg);
  auto map = makeGridMap(
    {{0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0}},
    1.0, 0.0, 0.0);
  tfe::Point2D robot{0.0, 0.0};

  std::vector<tfe::FrontierCluster> clusters;

  // Cluster A: cell at col 10 → world (10.5, 0.5), centroid at (100, 0)
  {
    tfe::FrontierCluster ca;
    ca.cells = {{0, 10}};
    ca.centroid_grid = {100.0, 0.0};
    ca.centroid_world = {100.5, 0.5};
    ca.representative_cell = {0, 10};
    ca.representative_world = {10.5, 0.5};
    clusters.push_back(ca);
  }

  // Cluster B: cell at col 5 → world (5.5, 0.5), centroid at (0, 50)
  {
    tfe::FrontierCluster cb;
    cb.cells = {{0, 5}};
    cb.centroid_grid = {0.0, 50.0};
    cb.centroid_world = {0.5, 50.5};
    cb.representative_cell = {0, 5};
    cb.representative_world = {5.5, 0.5};
    clusters.push_back(cb);
  }

  auto result = selector.select(clusters, map, robot);
  ASSERT_TRUE(result.selected.has_value());
  // Should pick B with goal dist = 5.5 (lower than A's 10.5)
  EXPECT_LT(result.selected->goal_distance_to_robot, 6.0);
  EXPECT_DOUBLE_EQ(result.selected->representative_world.x, 5.5);
}

// ── 11. Invalid config throws ────────────────────────────────────────────

TEST(FrontierGoalSelectorTest, invalidMinGoalDistanceThrows)
{
  tfe::FrontierGoalSelectorConfig cfg;
  cfg.min_goal_distance_meters = 0.0;
  EXPECT_THROW(tfe::FrontierGoalSelector s(cfg), std::invalid_argument);

  cfg.min_goal_distance_meters = -1.0;
  EXPECT_THROW(tfe::FrontierGoalSelector s(cfg), std::invalid_argument);
}
