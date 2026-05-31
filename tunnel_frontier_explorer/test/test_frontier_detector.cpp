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

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "tunnel_frontier_explorer/frontier_detector.hpp"

namespace tfe = tunnel_frontier_explorer;

// ── Helpers ──────────────────────────────────────────────────────────────

/// Build a GridMap from a 2-D initializer list of ints (-1=unknown, 0=free, 100=occupied).
/// Row-major order matching OccupancyGrid convention.
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

tfe::FrontierDetectorConfig defaultConfig()
{
  tfe::FrontierDetectorConfig cfg;
  cfg.min_cluster_size = 1;
  cfg.free_threshold = 0;
  cfg.frontier_neighbor_connectivity = 4;
  cfg.cluster_connectivity = 8;
  return cfg;
}

// ── 1. Empty map ─────────────────────────────────────────────────────────

TEST(FrontierDetectorTest, emptyMapReturnsEmpty)
{
  auto cfg = defaultConfig();
  tfe::FrontierDetector detector(cfg);
  tfe::GridMap map;
  map.width = 0;
  map.height = 0;
  map.resolution = 0.05;
  // valid() requires width>0 and height>0, so this map is not valid.
  // We expect an exception.
  tfe::Point2D robot{0, 0};
  EXPECT_THROW(detector.detect(map, robot), std::invalid_argument);
}

// ── 2. All unknown ───────────────────────────────────────────────────────

TEST(FrontierDetectorTest, allUnknownReturnsEmpty)
{
  auto cfg = defaultConfig();
  tfe::FrontierDetector detector(cfg);
  auto map = makeGridMap({{-1, -1, -1}, {-1, -1, -1}, {-1, -1, -1}});
  auto clusters = detector.detect(map, {0, 0});
  EXPECT_TRUE(clusters.empty());
}

// ── 3. All free ──────────────────────────────────────────────────────────

TEST(FrontierDetectorTest, allFreeReturnsEmpty)
{
  auto cfg = defaultConfig();
  tfe::FrontierDetector detector(cfg);
  auto map = makeGridMap({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}});
  auto clusters = detector.detect(map, {0, 0});
  EXPECT_TRUE(clusters.empty());
}

// ── 4. All occupied ──────────────────────────────────────────────────────

TEST(FrontierDetectorTest, allOccupiedReturnsEmpty)
{
  auto cfg = defaultConfig();
  tfe::FrontierDetector detector(cfg);
  auto map = makeGridMap({{100, 100}, {100, 100}});
  auto clusters = detector.detect(map, {0, 0});
  EXPECT_TRUE(clusters.empty());
}

// ── 5. Single frontier ───────────────────────────────────────────────────

TEST(FrontierDetectorTest, singleFrontier)
{
  // 3x3: centre cell is free (0), all others unknown (-1)
  // The centre cell has 4 unknown neighbours → frontier
  auto cfg = defaultConfig();
  tfe::FrontierDetector detector(cfg);
  auto map = makeGridMap({
    {-1, -1, -1},
    {-1, 0, -1},
    {-1, -1, -1},
  });
  auto clusters = detector.detect(map, {0, 0});
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].size(), 1u);
  // Centroid should be at the centre cell
  EXPECT_EQ(clusters[0].centroid_grid.x, 1.0);
  EXPECT_EQ(clusters[0].centroid_grid.y, 1.0);
  // With resolution=1, origin=0, cell (1,1) → world (1.5, 1.5)
  EXPECT_DOUBLE_EQ(clusters[0].centroid_world.x, 1.5);
  EXPECT_DOUBLE_EQ(clusters[0].centroid_world.y, 1.5);
}

// ── 6. Multiple independent frontiers ────────────────────────────────────

TEST(FrontierDetectorTest, multipleIndependentFrontiers)
{
  // 5x5 with two free cells at (1,1) and (3,3), unknown elsewhere.
  // Occupied barrier at (2,2) prevents 8-neighbor connection between them.
  // With 4-neighbor frontier detection + 8-neighbor clustering, they should
  // remain separate because the path between them goes through (2,2) which is
  // occupied (100), not a frontier cell.
  auto cfg = defaultConfig();
  tfe::FrontierDetector detector(cfg);
  std::vector<std::vector<std::int8_t>> rows(5, std::vector<std::int8_t>(5, -1));
  rows[1][1] = 0;  // frontier 1
  rows[3][3] = 0;  // frontier 2
  rows[2][2] = 100;  // barrier
  auto map = makeGridMap(rows);
  auto clusters = detector.detect(map, {0, 0});
  ASSERT_EQ(clusters.size(), 2u);
}

// ── 7. Map boundary frontier ─────────────────────────────────────────────

TEST(FrontierDetectorTest, mapBoundaryFrontier)
{
  // 2x2: top-left free, others unknown.
  // Cell (0,0) is free with unknown neighbours at (0,1) and (1,0).
  // Out-of-bounds neighbours are ignored.
  auto cfg = defaultConfig();
  tfe::FrontierDetector detector(cfg);
  auto map = makeGridMap({
    {0, -1},
    {-1, -1},
  });
  auto clusters = detector.detect(map, {0, 0});
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].size(), 1u);
}

// ── 8. Small cluster filter ──────────────────────────────────────────────

TEST(FrontierDetectorTest, smallClusterFilter)
{
  // 3x3 with centre free and rest unknown → single cell cluster.
  // Set min_cluster_size=2, expect it to be filtered out.
  auto cfg = defaultConfig();
  cfg.min_cluster_size = 2;
  tfe::FrontierDetector detector(cfg);
  auto map = makeGridMap({
    {-1, -1, -1},
    {-1, 0, -1},
    {-1, -1, -1},
  });
  auto clusters = detector.detect(map, {0, 0});
  EXPECT_TRUE(clusters.empty());
}

// ── 9. Centroid calculation (world coords with origin + resolution) ─────

TEST(FrontierDetectorTest, centroidCalculation)
{
  // A 1×3 row: cells (0,0) and (0,1) free, col 2 unknown.
  // Cell (0,0) is NOT a frontier (neighbor (0,1)=0, not unknown).
  // Cell (0,1) IS a frontier (neighbor (0,2)=-1).
  // Resolution=2.0, origin=(10, 20).
  // Cell (0,1) world: (10 + 1.5*2, 20 + 0.5*2) = (13, 21)
  auto cfg = defaultConfig();
  tfe::FrontierDetector detector(cfg);
  auto map = makeGridMap({
    {0, 0, -1},
  }, 2.0, 10.0, 20.0);
  auto clusters = detector.detect(map, {0, 0});
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].size(), 1u);
  EXPECT_DOUBLE_EQ(clusters[0].centroid_world.x, 13.0);
  EXPECT_DOUBLE_EQ(clusters[0].centroid_world.y, 21.0);
}

// ── 10. Fully known map (no unknown) ─────────────────────────────────────

TEST(FrontierDetectorTest, fullyKnownMapNoFrontiers)
{
  // Mix of free and occupied, no unknown cells.
  auto cfg = defaultConfig();
  tfe::FrontierDetector detector(cfg);
  auto map = makeGridMap({
    {0, 0, 100},
    {0, 100, 0},
    {100, 0, 0},
  });
  auto clusters = detector.detect(map, {0, 0});
  EXPECT_TRUE(clusters.empty());
}

// ── 11. data.size mismatch → exception ───────────────────────────────────

TEST(FrontierDetectorTest, dataSizeMismatchThrows)
{
  auto cfg = defaultConfig();
  tfe::FrontierDetector detector(cfg);
  tfe::GridMap map;
  map.width = 3;
  map.height = 3;
  map.resolution = 1.0;
  map.data = {0, 0, 0};  // only 3 elements, not 9
  EXPECT_THROW(detector.detect(map, {0, 0}), std::invalid_argument);
}

// ── 12. resolution ≤ 0 → exception ───────────────────────────────────────

TEST(FrontierDetectorTest, invalidResolutionThrows)
{
  auto cfg = defaultConfig();
  tfe::FrontierDetector detector(cfg);
  auto map = makeGridMap({{0, -1}, {-1, -1}}, 0.0);
  EXPECT_THROW(detector.detect(map, {0, 0}), std::invalid_argument);
}

// ── 13. 8-neighbor diagonal merge vs 4-neighbor separate ─────────────────

TEST(FrontierDetectorTest, diagonalMergesWith8NeighborClustering)
{
  // Two free cells diagonally adjacent at (0,0) and (1,1) with unknown
  // neighbours.  With 4-neighbor detection each is a frontier cell.
  // With 8-neighbor clustering they merge into one cluster.
  auto cfg = defaultConfig();
  cfg.frontier_neighbor_connectivity = 4;
  cfg.cluster_connectivity = 8;
  tfe::FrontierDetector detector(cfg);
  auto map = makeGridMap({
    {0, -1, -1},
    {-1, 0, -1},
    {-1, -1, -1},
  });
  auto clusters = detector.detect(map, {0, 0});
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].size(), 2u);
}

TEST(FrontierDetectorTest, diagonalStaysSeparateWith4NeighborClustering)
{
  // With both detection and clustering using 4-neighbor, the two diagonally
  // adjacent frontier cells should form separate clusters.
  auto cfg = defaultConfig();
  cfg.frontier_neighbor_connectivity = 4;
  cfg.cluster_connectivity = 4;
  tfe::FrontierDetector detector(cfg);
  auto map = makeGridMap({
    {0, -1, -1},
    {-1, 0, -1},
    {-1, -1, -1},
  });
  auto clusters = detector.detect(map, {0, 0});
  EXPECT_EQ(clusters.size(), 2u);
}

// ── 14. representative_cell is always a valid free cell ──────────────────

TEST(FrontierDetectorTest, representativeCellIsFree)
{
  // Create a frontier where the centroid might fall near an occupied cell.
  // The representative should always be a free (frontier) cell.
  auto cfg = defaultConfig();
  cfg.min_cluster_size = 1;
  tfe::FrontierDetector detector(cfg);
  // 5x5: a strip of free cells at row=2, cols 0-4.  Row 3 is unknown.
  // All cells at (2,0)-(2,4) are frontiers (free + unknown below).
  std::vector<std::vector<std::int8_t>> rows(5, std::vector<std::int8_t>(5, -1));
  for (int c = 0; c < 5; ++c) {
    rows[2][c] = 0;
  }
  auto map = makeGridMap(rows);
  auto clusters = detector.detect(map, {0, 0});
  ASSERT_GE(clusters.size(), 1u);
  for (const auto & cluster : clusters) {
    // representative_world must be a point within the map bounds
    ASSERT_LT(cluster.representative_cell.row,
              static_cast<int>(map.height));
    ASSERT_LT(cluster.representative_cell.col,
              static_cast<int>(map.width));
    // The cell data at representative should be free (== 0)
    const auto idx = cluster.representative_cell.row * map.width +
      cluster.representative_cell.col;
    EXPECT_EQ(map.data[idx], 0)
      << "representative cell at (" << cluster.representative_cell.row
      << "," << cluster.representative_cell.col << ") is not free";
  }
}

// ── 15. distance_to_robot is populated ───────────────────────────────────

TEST(FrontierDetectorTest, distanceToRobotPopulated)
{
  auto cfg = defaultConfig();
  tfe::FrontierDetector detector(cfg);
  auto map = makeGridMap({
    {-1, -1, -1},
    {-1, 0, -1},
    {-1, -1, -1},
  });
  // Robot at (0, 0) world.  Frontier centroid at (1.5, 1.5) world.
  // Distance should be sqrt(1.5^2 + 1.5^2) ≈ 2.121
  auto clusters = detector.detect(map, {0, 0});
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_NEAR(clusters[0].distance_to_robot, 2.121, 0.01);
}

// ── 16. Invalid connectivity throws in constructor ───────────────────────

TEST(FrontierDetectorTest, invalidConnectivityThrows)
{
  auto cfg = defaultConfig();
  cfg.frontier_neighbor_connectivity = 3;
  EXPECT_THROW(tfe::FrontierDetector d(cfg), std::invalid_argument);

  cfg.frontier_neighbor_connectivity = 4;
  cfg.cluster_connectivity = 6;
  EXPECT_THROW(tfe::FrontierDetector d(cfg), std::invalid_argument);
}
