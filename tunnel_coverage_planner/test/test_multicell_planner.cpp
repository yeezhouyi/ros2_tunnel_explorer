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
#include <set>
#include <string>
#include <vector>

#include "tunnel_coverage_planner/cleanable_map_builder.hpp"
#include "tunnel_coverage_planner/coverage_masks.hpp"
#include "tunnel_coverage_planner/coverage_tracker.hpp"
#include "tunnel_coverage_planner/robot_cleaning_geometry.hpp"
#include "tunnel_coverage_planner/scanline_planner.hpp"
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
using tunnel_map_core::Point2D;

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

CoverageMasks buildMasks(
  const GridMap & map, const Point2D & seed)
{
  GridGeometry geo(map);
  CleanableMapBuilderConfig cfg;
  CleanableMapBuilder builder(geo, cfg);
  RobotCleaningGeometry robot;
  return builder.build(map, robot, seed);
}

double simulatedEffectiveCoverage(
  const CoverageMasks & masks, const CoveragePlan & plan)
{
  CoverageTracker tracker(masks.geometry, masks, 0.25);
  for (const auto & s : plan.segments) {
    if (s.type == SegmentType::WORK) {
      tracker.addSweepSegment(
        Point2D{s.start_x, s.start_y}, Point2D{s.end_x, s.end_y});
    }
  }
  return tracker.metrics().effective_coverage;
}

// 70x40 room (3.5 x 2.0 m).
GridMap room()
{
  return makeMap(70, 40);
}

TEST(MulticellPlannerTest, SingleRegionEqualsPlainPlan)
{
  auto map = room();
  const auto masks = buildMasks(map, Point2D{0.5, 1.0});
  GridGeometry geo(map);
  ScanlinePlannerConfig cfg;
  RobotCleaningGeometry robot;
  ScanlinePlanner planner(geo, masks, robot, cfg);

  const auto plain = planner.plan();
  const auto multi = planner.planMultiCell(Point2D{0.5, 1.0});
  ASSERT_TRUE(plain.valid());
  ASSERT_TRUE(multi.valid());
  EXPECT_EQ(multi.plan_id, plain.plan_id);
}

TEST(MulticellPlannerTest, DoorwayRoomsBothCovered)
{
  // Two rooms divided by a vertical wall at col 35 with a door gap rows
  // 16..23.
  auto map = room();
  fillRect(map, 35, 0, 35, 39, OCC_OCCUPIED);
  fillRect(map, 35, 16, 35, 23, OCC_FREE);
  const Point2D seed{1.0, 1.0};
  const auto masks = buildMasks(map, seed);
  GridGeometry geo(map);
  ScanlinePlannerConfig cfg;
  RobotCleaningGeometry robot;
  ScanlinePlanner planner(geo, masks, robot, cfg);
  const auto plan = planner.planMultiCell(seed);
  ASSERT_TRUE(plan.valid());

  // Work segments must visit both rooms (>= 2 distinct cell ids).
  std::set<std::string> cells;
  for (const auto & s : plan.segments) {
    if (s.type == SegmentType::WORK) {
      cells.insert(s.cell_id);
    }
  }
  EXPECT_GE(cells.size(), 2u);

  // No work segment crosses the wall column.
  for (const auto & s : plan.segments) {
    if (s.type != SegmentType::WORK) {
      continue;
    }
    const int n = std::max(1, static_cast<int>(std::ceil(s.lengthM() / 0.05)));
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(n);
      const double x = s.start_x + t * (s.end_x - s.start_x);
      const double y = s.start_y + t * (s.end_y - s.start_y);
      tunnel_map_core::GridCell cell;
      ASSERT_TRUE(geo.worldToGridCell(Point2D{x, y}, cell));
      const auto idx = geo.index(cell.row, cell.col);
      EXPECT_NE(map.data[idx], OCC_OCCUPIED) << "crosses dividing wall";
    }
  }

  // Deterministic.
  ScanlinePlanner planner2(geo, masks, robot, cfg);
  EXPECT_EQ(planner2.planMultiCell(seed).plan_id, plan.plan_id);

  // Perfect execution sweeps both rooms.
  const double cov = simulatedEffectiveCoverage(masks, plan);
  EXPECT_GE(cov, 0.85) << "effective=" << cov;
}

TEST(MulticellPlannerTest, PillarRoomDecomposesWithoutCrossing)
{
  auto map = room();
  // Pillar 10x10 cells at the centre.
  fillRect(map, 30, 15, 39, 24, OCC_OCCUPIED);
  const Point2D seed{0.5, 1.0};
  const auto masks = buildMasks(map, seed);
  GridGeometry geo(map);
  ScanlinePlannerConfig cfg;
  RobotCleaningGeometry robot;
  ScanlinePlanner planner(geo, masks, robot, cfg);
  const auto plan = planner.planMultiCell(seed);
  ASSERT_TRUE(plan.valid());
  EXPECT_GE(plan.work_count, 6u);

  // No work segment crosses the pillar.
  for (const auto & s : plan.segments) {
    if (s.type != SegmentType::WORK) {
      continue;
    }
    const int n = std::max(1, static_cast<int>(std::ceil(s.lengthM() / 0.05)));
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(n);
      const double x = s.start_x + t * (s.end_x - s.start_x);
      const double y = s.start_y + t * (s.end_y - s.start_y);
      tunnel_map_core::GridCell cell;
      ASSERT_TRUE(geo.worldToGridCell(Point2D{x, y}, cell));
      const auto idx = geo.index(cell.row, cell.col);
      EXPECT_NE(map.data[idx], OCC_OCCUPIED) << "crosses pillar";
    }
  }

  ScanlinePlanner planner2(geo, masks, robot, cfg);
  EXPECT_EQ(planner2.planMultiCell(seed).plan_id, plan.plan_id);

  const double cov = simulatedEffectiveCoverage(masks, plan);
  EXPECT_GE(cov, 0.80) << "effective=" << cov;
}

TEST(MulticellPlannerTest, LShapedRoomCovered)
{
  // L-shape: full-width bottom half + right half of top half.
  auto map = room();
  fillRect(map, 0, 0, 34, 19, OCC_OCCUPIED);  // top-left block
  const Point2D seed{1.0, 1.5};  // bottom half is free
  const auto masks = buildMasks(map, seed);
  GridGeometry geo(map);
  ScanlinePlannerConfig cfg;
  RobotCleaningGeometry robot;
  ScanlinePlanner planner(geo, masks, robot, cfg);
  const auto plan = planner.planMultiCell(seed);
  ASSERT_TRUE(plan.valid());
  EXPECT_GE(plan.work_count, 8u);
  const double cov = simulatedEffectiveCoverage(masks, plan);
  EXPECT_GE(cov, 0.90) << "effective=" << cov;
}

}  // namespace
}  // namespace tunnel_coverage_planner
