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
#include <set>
#include <vector>

#include "tunnel_coverage_planner/boustrophedon_decomposer.hpp"
#include "tunnel_coverage_planner/cleanable_map_builder.hpp"
#include "tunnel_coverage_planner/coverage_masks.hpp"
#include "tunnel_map_core/grid_geometry.hpp"
#include "tunnel_map_core/grid_map.hpp"

namespace tunnel_coverage_planner
{
namespace
{

using tunnel_map_core::GridGeometry;
using tunnel_map_core::GridMap;
using tunnel_map_core::OCC_FREE;
using tunnel_map_core::OCC_OCCUPIED;

GridMap makeMap(std::size_t width, std::size_t height)
{
  GridMap map;
  map.width = width;
  map.height = height;
  map.resolution = 0.05;
  map.origin_x = 0.0;
  map.origin_y = 0.0;
  map.origin_yaw = 0.0;
  map.data.assign(width * height, OCC_FREE);
  return map;
}

void fillRect(
  GridMap & map, int c0, int r0, int c1, int r1, std::int8_t value)
{
  for (int row = r0; row <= r1; ++row) {
    for (int col = c0; col <= c1; ++col) {
      map.data[static_cast<std::size_t>(row) * map.width +
        static_cast<std::size_t>(col)] = value;
    }
  }
}

CoverageMasks allFreeMasks(const GridGeometry & geo)
{
  CoverageMasks m;
  m.geometry = geo;
  const auto n = geo.width() * geo.height();
  m.intended_target.assign(n, 1);
  m.navigable_center.assign(n, 1);
  m.reachable_cleanable.assign(n, 1);
  m.exempt.assign(n, 0);
  m.exempt_cause.assign(n, 0);
  return m;
}

// 40x30 cell room (2.0 x 1.5 m) at 0.05 m.
GridMap roomMap()
{
  return makeMap(40, 30);
}

TEST(BoustrophedonDecomposerTest, PlainRoomIsSingleCell)
{
  auto map = roomMap();
  GridGeometry geo(map);
  const auto masks = allFreeMasks(geo);
  BoustrophedonDecomposer dec(geo, masks);
  const auto cells = dec.decompose();
  ASSERT_EQ(cells.size(), 1u);
  EXPECT_EQ(cells[0].id, "c0");
  // Mask covers the entire reachable region.
  const auto n = geo.width() * geo.height();
  std::size_t ones = 0;
  for (std::size_t i = 0; i < n; ++i) {
    ones += cells[0].mask[i] != 0 ? 1u : 0u;
  }
  EXPECT_EQ(ones, n);
}

CoverageMasks buildMasks(
  const GridMap & map, const tunnel_map_core::Point2D & seed)
{
  GridGeometry geo(map);
  CleanableMapBuilderConfig cfg;
  CleanableMapBuilder builder(geo, cfg);
  RobotCleaningGeometry robot;
  return builder.build(map, robot, seed);
}

TEST(BoustrophedonDecomposerTest, PillarSplitsAndMerges)
{
  // Room with a centred pillar occupying rows 8..21, cols 12..17.
  auto map = roomMap();
  fillRect(map, 12, 8, 17, 21, OCC_OCCUPIED);
  GridGeometry geo(map);
  const auto masks = buildMasks(map, tunnel_map_core::Point2D{0.3, 0.3});
  BoustrophedonDecomposer dec(geo, masks);
  const auto cells = dec.decompose();

  // Top band, left band, right band, bottom band.
  ASSERT_EQ(cells.size(), 4u);

  // Neighbour topology: top neighbours left+right; bottom neighbours
  // left+right; left/right not directly adjacent (pillar between).
  auto hasNeighbour = [&](int ci, int ni) {
      const auto & ns = cells[static_cast<std::size_t>(ci)].neighbors;
      return std::find(ns.begin(), ns.end(), ni) != ns.end();
    };
  EXPECT_TRUE(hasNeighbour(0, 1));
  EXPECT_TRUE(hasNeighbour(0, 2));
  EXPECT_TRUE(hasNeighbour(3, 1));
  EXPECT_TRUE(hasNeighbour(3, 2));
  EXPECT_FALSE(hasNeighbour(1, 2));

  // Every free cell belongs to exactly one cell mask.
  const auto n = geo.width() * geo.height();
  std::size_t owned = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (map.data[i] != OCC_FREE) {
      continue;
    }
    int owners = 0;
    for (const auto & c : cells) {
      owners += c.mask[i] != 0 ? 1 : 0;
    }
    EXPECT_LE(owners, 1);
    owned += owners == 1 ? 1u : 0u;
  }
  std::size_t free_count = 0;
  for (const auto v : map.data) {
    free_count += v == OCC_FREE ? 1u : 0u;
  }
  EXPECT_EQ(owned, free_count);

  // Determinism.
  const auto cells2 = dec.decompose();
  ASSERT_EQ(cells2.size(), cells.size());
  for (std::size_t i = 0; i < cells.size(); ++i) {
    EXPECT_EQ(cells2[i].id, cells[i].id);
    EXPECT_EQ(cells2[i].neighbors, cells[i].neighbors);
    EXPECT_EQ(cells2[i].mask, cells[i].mask);
  }
}

TEST(BoustrophedonDecomposerTest, DoorwayRoomsDecomposeIntoAtLeastTwoCells)
{
  // Two rooms separated by a vertical wall (col 20) with a 0.4 m doorway
  // (rows 11..18, wider than 2 * chassis clearance).
  auto map = roomMap();
  fillRect(map, 20, 0, 20, 29, OCC_OCCUPIED);
  fillRect(map, 20, 11, 20, 18, OCC_FREE);  // doorway
  GridGeometry geo(map);
  const auto masks = buildMasks(map, tunnel_map_core::Point2D{0.3, 0.3});
  BoustrophedonDecomposer dec(geo, masks);
  const auto cells = dec.decompose();
  ASSERT_GE(cells.size(), 2u);

  // Union of cell masks == every free cell.
  const auto n = geo.width() * geo.height();
  std::size_t union_count = 0;
  for (std::size_t i = 0; i < n; ++i) {
    bool owned = false;
    for (const auto & c : cells) {
      if (c.mask[i] != 0) {
        owned = true;
        break;
      }
    }
    if (owned) {
      ++union_count;
      EXPECT_EQ(map.data[i], OCC_FREE) << "occupied cell owned by a cell";
    }
  }
  std::size_t free_count = 0;
  for (const auto v : map.data) {
    free_count += v == OCC_FREE ? 1u : 0u;
  }
  EXPECT_EQ(union_count, free_count);
}

}  // namespace
}  // namespace tunnel_coverage_planner
