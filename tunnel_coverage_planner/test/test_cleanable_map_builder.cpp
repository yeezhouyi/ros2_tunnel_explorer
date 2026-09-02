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

#include "tunnel_coverage_planner/cleanable_map_builder.hpp"
#include "tunnel_coverage_planner/coverage_masks.hpp"
#include "tunnel_coverage_planner/robot_cleaning_geometry.hpp"
#include "tunnel_map_core/grid_geometry.hpp"
#include "tunnel_map_core/grid_map.hpp"

namespace tunnel_coverage_planner
{
namespace
{

using tunnel_map_core::GridCell;
using tunnel_map_core::GridGeometry;
using tunnel_map_core::GridMap;
using tunnel_map_core::OCC_FREE;
using tunnel_map_core::OCC_OCCUPIED;
using tunnel_map_core::OCC_UKNOWN;
using tunnel_map_core::Point2D;

/// Small helper map builder: width x height cells at `res`, origin (0, 0).
struct MapSpec
{
  std::size_t width = 0;
  std::size_t height = 0;
  double res = 0.05;
};

GridMap makeMap(const MapSpec & spec)
{
  GridMap map;
  map.width = spec.width;
  map.height = spec.height;
  map.resolution = spec.res;
  map.origin_x = 0.0;
  map.origin_y = 0.0;
  map.origin_yaw = 0.0;
  map.data.assign(spec.width * spec.height, OCC_FREE);
  return map;
}

/// Fill an axis-aligned world rectangle [x0, x1) x [y0, y1) with @p value.
void fillRect(
  GridMap & map, double x0, double y0, double x1, double y1,
  std::int8_t value)
{
  for (std::size_t row = 0; row < map.height; ++row) {
    for (std::size_t col = 0; col < map.width; ++col) {
      const double wx = map.origin_x + (static_cast<double>(col) + 0.5) * map.resolution;
      const double wy = map.origin_y + (static_cast<double>(row) + 0.5) * map.resolution;
      if (wx >= x0 && wx < x1 && wy >= y0 && wy < y1) {
        map.data[row * map.width + col] = value;
      }
    }
  }
}

/// Seed pose at the centre of a given world point.
Point2D seedAt(double x, double y)
{
  return Point2D{x, y};
}

// 2.0 x 1.5 m all-free room: 40 x 30 cells at 0.05 m.
MapSpec rectSpec()
{
  MapSpec s;
  s.width = 40;
  s.height = 30;
  s.res = 0.05;
  return s;
}

TEST(CleanableMapBuilderTest, ThrowsOnSeedOutsideNavigable)
{
  auto map = makeMap(rectSpec());
  fillRect(map, 0.5, 0.5, 1.0, 1.0, OCC_OCCUPIED);  // pillar
  GridGeometry geo(map);
  CleanableMapBuilderConfig cfg;
  CleanableMapBuilder builder(geo, cfg);
  RobotCleaningGeometry robot;

  // Seed on an occupied cell.
  EXPECT_THROW(
    builder.build(map, robot, seedAt(0.75, 0.75)), std::invalid_argument);

  // Seed outside the map.
  EXPECT_THROW(
    builder.build(map, robot, seedAt(100.0, 100.0)), std::invalid_argument);
}

TEST(CleanableMapBuilderTest, AllFreeRoomInvariantsAndZeroExempt)
{
  auto map = makeMap(rectSpec());
  GridGeometry geo(map);
  CleanableMapBuilderConfig cfg;
  CleanableMapBuilder builder(geo, cfg);

  // Default robot: clearance 0.18 m, tool radius 0.25 m >= clearance, so the
  // centred footprint can reach every wall-adjacent strip -> exempt == 0.
  RobotCleaningGeometry robot;
  const auto masks = builder.build(map, robot, seedAt(0.5, 0.5));

  ASSERT_TRUE(masks.consistent());
  const auto n = map.data.size();
  EXPECT_EQ(CoverageMasks::count(masks.intended_target), n);
  EXPECT_EQ(CoverageMasks::count(masks.exempt), 0u);

  // Navigator centre must be strictly smaller than free (erosion by 0.18 m).
  EXPECT_LT(CoverageMasks::count(masks.navigable_center), n);
  EXPECT_GT(CoverageMasks::count(masks.navigable_center), 0u);
  EXPECT_EQ(masks.reachable_cleanable, masks.intended_target);

  // Every navigable cell is free.
  for (std::size_t i = 0; i < n; ++i) {
    ASSERT_LE(masks.navigable_center[i], masks.intended_target[i]);
  }
}

TEST(CleanableMapBuilderTest, WallStripBecomesExemptWhenToolNarrowerThanClearance)
{
  auto map = makeMap(rectSpec());
  GridGeometry geo(map);
  CleanableMapBuilderConfig cfg;
  CleanableMapBuilder builder(geo, cfg);

  // Narrow brush: tool radius 0.10 m < clearance 0.18 m -> the wall-adjacent
  // strip (0.18 m wide) cannot be reached by the centred footprint.
  RobotCleaningGeometry robot;
  robot.cleaning_width_m = 0.20;

  const auto masks = builder.build(map, robot, seedAt(0.5, 0.5));
  ASSERT_TRUE(masks.consistent());

  EXPECT_GT(CoverageMasks::count(masks.exempt), 0u);
  // Invariant: target = reachable + exempt.
  EXPECT_EQ(
    CoverageMasks::count(masks.intended_target),
    CoverageMasks::count(masks.reachable_cleanable) +
      CoverageMasks::count(masks.exempt));

  // All exempt cells carry an auditable cause.
  for (std::size_t i = 0; i < masks.exempt.size(); ++i) {
    if (masks.exempt[i] != 0) {
      EXPECT_NE(masks.exempt_cause[i],
        static_cast<std::uint8_t>(ExemptCause::NONE));
    }
  }
}

TEST(CleanableMapBuilderTest, PillarIsExcludedFromNavigableButCleanable)
{
  auto map = makeMap(rectSpec());
  // Occupied pillar of 0.2 x 0.2 m at the room centre.
  fillRect(map, 0.9, 0.65, 1.1, 0.85, OCC_OCCUPIED);
  GridGeometry geo(map);
  CleanableMapBuilderConfig cfg;
  CleanableMapBuilder builder(geo, cfg);
  RobotCleaningGeometry robot;

  const auto masks = builder.build(map, robot, seedAt(0.3, 0.3));
  ASSERT_TRUE(masks.consistent());

  // Pillar cells are neither target nor navigable.
  {
    GridCell c;
    ASSERT_TRUE(geo.worldToGridCell(Point2D{1.0, 0.75}, c));
    const auto idx = geo.index(c.row, c.col);
    EXPECT_EQ(masks.intended_target[idx], 0u);
    EXPECT_EQ(masks.navigable_center[idx], 0u);
  }

  // The ring around the pillar is cleanable (tool radius 0.25 m bridges the
  // 0.18 m clearance gap) and exempt stays zero in this single-room scene.
  EXPECT_EQ(CoverageMasks::count(masks.exempt), 0u);
  EXPECT_EQ(masks.reachable_cleanable, masks.intended_target);
}

TEST(CleanableMapBuilderTest, NarrowCorridorExcluded)
{
  // World 1.0 x 1.0 m; two slabs leaving a 0.30 m gap (smaller than twice the
  // 0.18 m clearance).  A robot spawned inside such a gap cannot move: the
  // builder must reject the input instead of producing an empty plan.
  MapSpec spec;
  spec.width = 20;
  spec.height = 20;
  spec.res = 0.05;
  auto map = makeMap(spec);
  fillRect(map, 0.0, 0.0, 0.35, 1.0, OCC_OCCUPIED);   // left slab
  fillRect(map, 0.65, 0.0, 1.0, 1.0, OCC_OCCUPIED);   // right slab
  GridGeometry geo(map);
  CleanableMapBuilderConfig cfg;
  CleanableMapBuilder builder(geo, cfg);
  RobotCleaningGeometry robot;

  // Seed inside the 0.3 m gap -> not navigable -> rejected.
  EXPECT_THROW(
    builder.build(map, robot, seedAt(0.5, 0.5)), std::invalid_argument);
}

TEST(CleanableMapBuilderTest, WideEnoughGapIsNavigable)
{
  // Gap of 0.44 m (> 2 * 0.18 m clearance): navigable cells exist and lie
  // only inside the gap.
  MapSpec spec;
  spec.width = 20;
  spec.height = 20;
  spec.res = 0.05;
  auto map = makeMap(spec);
  fillRect(map, 0.0, 0.0, 0.28, 1.0, OCC_OCCUPIED);   // left slab
  fillRect(map, 0.72, 0.0, 1.0, 1.0, OCC_OCCUPIED);   // right slab
  GridGeometry geo(map);
  CleanableMapBuilderConfig cfg;
  CleanableMapBuilder builder(geo, cfg);
  RobotCleaningGeometry robot;

  const auto masks = builder.build(map, robot, seedAt(0.5, 0.5));
  ASSERT_TRUE(masks.consistent());
  EXPECT_GT(CoverageMasks::count(masks.navigable_center), 0u);
  // No navigable cell lies inside the slabs.
  for (std::size_t row = 0; row < map.height; ++row) {
    for (std::size_t col = 0; col < map.width; ++col) {
      const auto idx = row * map.width + col;
      if (masks.navigable_center[idx] == 0) {
        continue;
      }
      const double wx = 0.0 + (static_cast<double>(col) + 0.5) * 0.05;
      EXPECT_GE(wx, 0.28);
      EXPECT_LE(wx, 0.72);
    }
  }
}

TEST(CleanableMapBuilderTest, UnknownCellsNeverTargetOrNavigable)
{
  auto map = makeMap(rectSpec());
  // Left half unknown (grey in the PGM), right half free.
  fillRect(map, 0.0, 0.0, 1.0, 1.5, OCC_UKNOWN);
  GridGeometry geo(map);
  CleanableMapBuilderConfig cfg;
  CleanableMapBuilder builder(geo, cfg);
  RobotCleaningGeometry robot;

  const auto masks = builder.build(map, robot, seedAt(1.5, 0.75));
  ASSERT_TRUE(masks.consistent());

  const auto n = map.data.size();
  for (std::size_t i = 0; i < n; ++i) {
    if (map.data[i] == OCC_UKNOWN) {
      EXPECT_EQ(masks.intended_target[i], 0u);
      EXPECT_EQ(masks.navigable_center[i], 0u);
    }
  }
  EXPECT_EQ(CoverageMasks::count(masks.intended_target),
    CoverageMasks::count(masks.reachable_cleanable) +
      CoverageMasks::count(masks.exempt));
}

TEST(CleanableMapBuilderTest, DoorwayConnectsTwoRooms)
{
  // Two rooms connected by a 0.55 m doorway (wider than 2 * 0.18 m clearance).
  // Wall runs at x = 2.0 with a door gap y in [0.7, 1.25].
  MapSpec spec;
  spec.width = 120;   // 6.0 m
  spec.height = 40;   // 2.0 m
  spec.res = 0.05;
  auto map = makeMap(spec);

  // Room floors implicit (free).  Wall: x in [1.97, 2.03], y in [0.0, 2.0],
  // minus the door gap.
  fillRect(map, 0.0, 0.0, 6.0, 2.0, OCC_FREE);
  {
    for (std::size_t row = 0; row < map.height; ++row) {
      const double wy = map.origin_y +
        (static_cast<double>(row) + 0.5) * map.resolution;
      for (std::size_t col = 0; col < map.width; ++col) {
        const double wx = map.origin_x +
          (static_cast<double>(col) + 0.5) * map.resolution;
        if (wx >= 1.97 && wx <= 2.03) {
          if (wy < 0.70 || wy > 1.25) {
            map.data[row * map.width + col] = OCC_OCCUPIED;
          }
        }
      }
    }
  }
  GridGeometry geo(map);
  CleanableMapBuilderConfig cfg;
  CleanableMapBuilder builder(geo, cfg);
  RobotCleaningGeometry robot;

  // Seed in the left room; the right room stays reachable through the door.
  const auto masks = builder.build(map, robot, seedAt(0.5, 1.0));
  ASSERT_TRUE(masks.consistent());
  EXPECT_EQ(CoverageMasks::count(masks.exempt), 0u);

  // A fully walled second room (no door) must be exempt as UNREACHABLE.
  auto map2 = makeMap(spec);
  fillRect(map2, 0.0, 0.0, 6.0, 2.0, OCC_FREE);
  fillRect(map2, 1.97, 0.0, 2.03, 2.0, OCC_OCCUPIED);  // no door
  GridGeometry geo2(map2);
  CleanableMapBuilder builder2(geo2, cfg);
  const auto masks2 = builder2.build(map2, robot, seedAt(0.5, 1.0));
  ASSERT_TRUE(masks2.consistent());
  EXPECT_GT(CoverageMasks::count(masks2.exempt), 0u);
  for (std::size_t i = 0; i < masks2.exempt.size(); ++i) {
    if (masks2.exempt[i] != 0) {
      EXPECT_EQ(masks2.exempt_cause[i],
        static_cast<std::uint8_t>(ExemptCause::UNREACHABLE));
    }
  }
}

TEST(CleanableMapBuilderTest, MinAreaGateMarksWholeRegionIsland)
{
  // Small 1.0 x 1.0 m room: 20 x 20 cells.  With a huge min-area gate the
  // whole cleanable region is dropped and every free cell becomes an exempt
  // island.
  MapSpec spec;
  spec.width = 20;
  spec.height = 20;
  spec.res = 0.05;
  auto map = makeMap(spec);
  GridGeometry geo(map);
  CleanableMapBuilderConfig cfg;
  CleanableMapBuilder builder(geo, cfg);

  RobotCleaningGeometry robot;
  robot.min_cleanable_region_area_m2 = 4.0;  // > whole room area

  const auto masks = builder.build(map, robot, seedAt(0.5, 0.5));
  ASSERT_TRUE(masks.consistent());
  EXPECT_EQ(CoverageMasks::count(masks.reachable_cleanable), 0u);
  EXPECT_EQ(
    CoverageMasks::count(masks.exempt),
    CoverageMasks::count(masks.intended_target));
  for (std::size_t i = 0; i < masks.exempt.size(); ++i) {
    if (masks.exempt[i] != 0) {
      EXPECT_EQ(masks.exempt_cause[i],
        static_cast<std::uint8_t>(ExemptCause::ISLAND));
    }
  }
}

TEST(CleanableMapBuilderTest, InvalidConfigRejected)
{
  auto map = makeMap(rectSpec());
  GridGeometry geo(map);
  CleanableMapBuilderConfig cfg;
  cfg.free_max = 80;   // >= occupied_min(65) -> invalid
  EXPECT_THROW(CleanableMapBuilder(geo, cfg), std::invalid_argument);
  cfg.free_max = 0;
  cfg.connectivity = 5;
  EXPECT_THROW(CleanableMapBuilder(geo, cfg), std::invalid_argument);
}

}  // namespace
}  // namespace tunnel_coverage_planner
