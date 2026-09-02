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
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "tunnel_map_core/grid_geometry.hpp"
#include "tunnel_map_core/grid_map.hpp"

namespace tunnel_map_core
{
namespace
{

GridMap makeMap(
  std::size_t width, std::size_t height, double resolution,
  double origin_x, double origin_y, double origin_yaw)
{
  GridMap map;
  map.width = width;
  map.height = height;
  map.resolution = resolution;
  map.origin_x = origin_x;
  map.origin_y = origin_y;
  map.origin_yaw = origin_yaw;
  map.data.assign(width * height, OCC_FREE);
  return map;
}

TEST(GridGeometryTest, ThrowsOnInvalidMap)
{
  GridMap map;
  EXPECT_THROW(GridGeometry(map), std::invalid_argument);

  map.width = 10;
  map.height = 10;
  EXPECT_THROW(GridGeometry(map), std::invalid_argument);  // data missing

  map.data.assign(100, OCC_FREE);
  map.resolution = 0.0;
  EXPECT_THROW(GridGeometry(map), std::invalid_argument);  // bad resolution
}

TEST(GridGeometryTest, NonUnitResolutionRoundTrip)
{
  // resolution 0.1, origin (1.0, 2.0), yaw 0
  auto map = makeMap(20, 30, 0.1, 1.0, 2.0, 0.0);
  GridGeometry geo(map);

  // Cell (3, 7) centre: x = 1.0 + 7.5*0.1 = 1.75, y = 2.0 + 3.5*0.1 = 2.35
  const auto w = geo.gridToWorld(3, 7);
  EXPECT_NEAR(w.x, 1.75, 1e-12);
  EXPECT_NEAR(w.y, 2.35, 1e-12);

  // Round trip: a cell centre must land back on the same cell.
  const auto back = geo.worldToGrid(w);
  GridCell cell;
  ASSERT_TRUE(geo.gridToCell(back, cell));
  EXPECT_EQ(cell.row, 3);
  EXPECT_EQ(cell.col, 7);

  // Arbitrary world point must map to the expected cell.
  GridCell p;
  ASSERT_TRUE(geo.worldToGridCell(Point2D{1.72, 2.33}, p));
  EXPECT_EQ(p.row, 3);
  EXPECT_EQ(p.col, 7);
}

TEST(GridGeometryTest, NonZeroOriginRoundTrip)
{
  // Negative origin, resolution 0.05.
  auto map = makeMap(50, 40, 0.05, -1.5, -2.0, 0.0);
  GridGeometry geo(map);

  const auto w = geo.gridToWorld(10, 20);
  // x = -1.5 + 20.5*0.05 = -0.475, y = -2.0 + 10.5*0.05 = -1.475
  EXPECT_NEAR(w.x, -0.475, 1e-12);
  EXPECT_NEAR(w.y, -1.475, 1e-12);

  GridCell cell;
  ASSERT_TRUE(geo.worldToGridCell(w, cell));
  EXPECT_EQ(cell.row, 10);
  EXPECT_EQ(cell.col, 20);

  // Error of world -> grid -> world must stay below half a cell.
  const Point2D probe{-0.3, -1.9};
  const auto g = geo.worldToGrid(probe);
  const auto w2 = geo.gridToWorld(g);
  EXPECT_LT(std::abs(w2.x - probe.x), 0.5 * 0.05);
  EXPECT_LT(std::abs(w2.y - probe.y), 0.5 * 0.05);
}

TEST(GridGeometryTest, OriginYawRoundTrip)
{
  const double yaw = 0.6;
  auto map = makeMap(40, 40, 0.05, 0.0, 0.0, yaw);
  GridGeometry geo(map);

  const Point2D probe{1.2, -0.7};
  const auto g = geo.worldToGrid(probe);
  const auto w2 = geo.gridToWorld(g);
  EXPECT_NEAR(w2.x, probe.x, 1e-9);
  EXPECT_NEAR(w2.y, probe.y, 1e-9);

  // Grid axis x should point along (cos yaw, sin yaw): a pure grid-axis-x
  // step must map to a world step in that direction.
  const auto c0 = geo.gridToWorld(GridPoint{0.0, 0.0});
  const auto c1 = geo.gridToWorld(GridPoint{1.0, 0.0});
  const double dx = c1.x - c0.x;
  const double dy = c1.y - c0.y;
  EXPECT_NEAR(std::atan2(dy, dx), yaw, 1e-9);
}

TEST(GridGeometryTest, NegativeWorldCoordinatesAndBounds)
{
  auto map = makeMap(10, 10, 0.5, -2.5, -2.5, 0.0);  // world span [-2.5, 2.5)
  GridGeometry geo(map);

  // A point just inside the bottom-left corner cell (0,0).
  GridCell cell;
  ASSERT_TRUE(geo.worldToGridCell(Point2D{-2.45, -2.45}, cell));
  EXPECT_EQ(cell.row, 0);
  EXPECT_EQ(cell.col, 0);

  // Out of bounds on all four sides.
  EXPECT_FALSE(geo.worldToGridCell(Point2D{-2.6, -2.45}, cell));
  EXPECT_FALSE(geo.worldToGridCell(Point2D{2.6, 0.0}, cell));
  EXPECT_FALSE(geo.worldToGridCell(Point2D{0.0, 2.6}, cell));
  EXPECT_FALSE(geo.worldToGridCell(Point2D{0.0, -2.6}, cell));

  EXPECT_TRUE(geo.inBounds(0, 0));
  EXPECT_TRUE(geo.inBounds(9, 9));
  EXPECT_FALSE(geo.inBounds(10, 0));
  EXPECT_FALSE(geo.inBounds(-1, 0));
}

TEST(GridGeometryTest, BoundaryCellAndIndex)
{
  auto map = makeMap(4, 3, 0.1, 0.0, 0.0, 0.0);
  GridGeometry geo(map);
  EXPECT_EQ(geo.width(), 4u);
  EXPECT_EQ(geo.height(), 3u);
  EXPECT_DOUBLE_EQ(geo.cellSize(), 0.1);
  EXPECT_EQ(geo.index(2, 3), 11u);  // row-major: 2*4 + 3
}

TEST(GridGeometryTest, ContentDigestIsStableAndSensitive)
{
  auto base = makeMap(4, 4, 0.1, 0.0, 0.0, 0.0);
  GridGeometry geo(base);

  const auto d1 = geo.contentDigest(base);
  // Same map twice -> identical digest.
  EXPECT_EQ(geo.contentDigest(base), d1);

  // Occupancy change -> different digest.
  auto changed = base;
  changed.data[0] = OCC_OCCUPIED;
  EXPECT_NE(geo.contentDigest(changed), d1);

  // Resolution change -> different digest.
  auto res = base;
  res.resolution = 0.2;
  EXPECT_NE(geo.contentDigest(res), d1);

  // Origin change -> different digest.
  auto origin = base;
  origin.origin_x = 0.5;
  EXPECT_NE(geo.contentDigest(origin), d1);

  // Same digest from an equivalent freshly built map (determinism).
  auto clone = makeMap(4, 4, 0.1, 0.0, 0.0, 0.0);
  clone.data = base.data;
  EXPECT_EQ(geo.contentDigest(clone), d1);
}

}  // namespace
}  // namespace tunnel_map_core
