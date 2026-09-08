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
#include <cstddef>
#include <vector>

#include "tunnel_map_core/grid_geometry.hpp"
#include "tunnel_map_core/grid_map.hpp"
#include "tunnel_coverage_executor/goal_clamp.hpp"

namespace tunnel_coverage_executor
{
namespace
{

using tunnel_map_core::GridGeometry;
using tunnel_map_core::GridMap;
using tunnel_map_core::OCC_FREE;
using tunnel_map_core::OCC_OCCUPIED;

constexpr double kRes = 0.025;      // 80 x 80 cells -> 2.0 m x 2.0 m map
constexpr std::size_t kSide = 80;
constexpr double kInset = 0.10;     // footprint radius + one safety cell

GridMap makeMap()
{
  GridMap map;
  map.width = kSide;
  map.height = kSide;
  map.resolution = kRes;
  map.origin_x = 0.0;
  map.origin_y = 0.0;
  map.origin_yaw = 0.0;
  map.data.assign(kSide * kSide, OCC_OCCUPIED);
  // Free rectangle: rows 0..78 (y < 1.975), cols 4..75 (0.1 < x < 1.9).
  for (std::size_t row = 0; row < 79; ++row) {
    for (std::size_t col = 4; col < 76; ++col) {
      map.data[row * kSide + col] = OCC_FREE;
    }
  }
  return map;
}

std::vector<std::uint8_t> makeMask(const GridMap & map)
{
  const GridGeometry geo(map);
  std::vector<std::uint8_t> mask(map.width * map.height, 0);
  for (std::size_t i = 0; i < mask.size(); ++i) {
    mask[i] = map.data[i] == OCC_FREE ? 1 : 0;
  }
  return mask;
}

bool cellValid(
  const std::vector<std::uint8_t> & mask, const GridGeometry & geo,
  double x, double y)
{
  tunnel_map_core::GridCell c;
  if (!geo.worldToGridCell({x, y}, c)) {
    return false;
  }
  return mask[static_cast<std::size_t>(c.row) *
           static_cast<std::size_t>(geo.width()) +
           static_cast<std::size_t>(c.col)] != 0;
}

TEST(GoalClamp, ValidEndpointUntouched)
{
  const GridMap map = makeMap();
  const GridGeometry geo(map);
  const auto mask = makeMask(map);
  const auto r = clampGoalEndpoint(mask, geo, 1.0, 1.0, kInset);
  EXPECT_FALSE(r.clamped);
  EXPECT_DOUBLE_EQ(r.x, 1.0);
  EXPECT_DOUBLE_EQ(r.y, 1.0);
}

TEST(GoalClamp, TopEdgeEndpointClampedWithInset)
{
  const GridMap map = makeMap();
  const GridGeometry geo(map);
  const auto mask = makeMask(map);
  // y = 1.98 falls on row 79 -- outside the mask (the room0-w18-0 case:
  // endpoint sits on the map top edge, my = 80 out of size_y = 80 class).
  const auto r = clampGoalEndpoint(mask, geo, 1.5, 1.98, kInset);
  ASSERT_TRUE(r.clamped);
  EXPECT_TRUE(cellValid(mask, geo, r.x, r.y));
  // Mask top edge is 1.975; the clamped goal keeps >= inset distance.
  EXPECT_LE(r.y, 1.975 - kInset + 1e-9);
  // Never pushed below the inset window on the valid side either.
  EXPECT_GE(r.y, 0.0 + kInset - 1e-9);
  // The clamp moves the goal the shortest way -- x stays on the row.
  EXPECT_NEAR(r.x, 1.5, kRes);
}

TEST(GoalClamp, FullyOutsideGridClamped)
{
  const GridMap map = makeMap();
  const GridGeometry geo(map);
  const auto mask = makeMask(map);
  const auto r = clampGoalEndpoint(mask, geo, 1.5, 2.5, kInset);
  ASSERT_TRUE(r.clamped);
  EXPECT_TRUE(cellValid(mask, geo, r.x, r.y));
  EXPECT_LE(r.y, 1.975 - kInset + 1e-9);
}

TEST(GoalClamp, InteriorHoleSnapsToNearestValid)
{
  GridMap map = makeMap();
  // Carve a hole: rows 38..42, cols 38..42.
  for (std::size_t row = 38; row <= 42; ++row) {
    for (std::size_t col = 38; col <= 42; ++col) {
      map.data[row * kSide + col] = OCC_OCCUPIED;
    }
  }
  const GridGeometry geo(map);
  const auto mask = makeMask(map);
  const auto r = clampGoalEndpoint(mask, geo, 1.0, 1.0, kInset);
  ASSERT_TRUE(r.clamped);
  EXPECT_TRUE(cellValid(mask, geo, r.x, r.y));
  // Nearest valid centre sits on the hole rim (one cell away at most).
  EXPECT_LE(std::hypot(r.x - 1.0, r.y - 1.0), 3.0 * kRes + 1e-9);
}

TEST(GoalClamp, ThinRegionFallsBackToValidCentre)
{
  GridMap map = makeMap();
  // Thin strip: only rows 39..40 free (0.05 m tall < 2 * inset).
  map.data.assign(kSide * kSide, OCC_OCCUPIED);
  for (std::size_t row = 39; row <= 40; ++row) {
    for (std::size_t col = 4; col < 76; ++col) {
      map.data[row * kSide + col] = OCC_FREE;
    }
  }
  const GridGeometry geo(map);
  const auto mask = makeMask(map);
  const auto r = clampGoalEndpoint(mask, geo, 1.5, 1.9, kInset);
  ASSERT_TRUE(r.clamped);
  EXPECT_TRUE(cellValid(mask, geo, r.x, r.y));
}

TEST(GoalClamp, ZeroInsetSnapsToNearestValidCentre)
{
  const GridMap map = makeMap();
  const GridGeometry geo(map);
  const auto mask = makeMask(map);
  const auto r = clampGoalEndpoint(mask, geo, 1.5, 1.98, 0.0);
  ASSERT_TRUE(r.clamped);
  EXPECT_TRUE(cellValid(mask, geo, r.x, r.y));
  // With no inset requirement the nearest valid centre (row 78) wins.
  EXPECT_DOUBLE_EQ(r.y, 78.0 * kRes + 0.5 * kRes);
}

TEST(GoalClamp, EmptyMaskLeavesGoalUntouched)
{
  const GridMap map = makeMap();
  const GridGeometry geo(map);
  const std::vector<std::uint8_t> mask(map.width * map.height, 0);
  const auto r = clampGoalEndpoint(mask, geo, 1.5, 1.98, kInset);
  EXPECT_FALSE(r.clamped);
  EXPECT_DOUBLE_EQ(r.x, 1.5);
  EXPECT_DOUBLE_EQ(r.y, 1.98);
}

}  // namespace
}  // namespace tunnel_coverage_executor
