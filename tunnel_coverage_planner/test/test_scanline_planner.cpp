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
#include <stdexcept>
#include <string>
#include <vector>

#include "tunnel_coverage_planner/cleanable_map_builder.hpp"
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
  GridMap & map, double x0, double y0, double x1, double y1,
  std::int8_t value)
{
  for (std::size_t row = 0; row < map.height; ++row) {
    for (std::size_t col = 0; col < map.width; ++col) {
      const double wx = map.origin_x +
        (static_cast<double>(col) + 0.5) * map.resolution;
      const double wy = map.origin_y +
        (static_cast<double>(row) + 0.5) * map.resolution;
      if (wx >= x0 && wx < x1 && wy >= y0 && wy < y1) {
        map.data[row * map.width + col] = value;
      }
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

/// Simulate perfect execution of the plan's work segments through the real
/// trajectory sweep and return effective coverage.
double simulatedEffectiveCoverage(const CoverageMasks & masks, const CoveragePlan & plan)
{
  CoverageTracker tracker(
    masks.geometry, masks, 0.5 * 0.5 /* sweep radius = width/2 */);
  for (const auto & s : plan.segments) {
    if (s.type == SegmentType::WORK) {
      tracker.addSweepSegment(Point2D{s.start_x, s.start_y},
        Point2D{s.end_x, s.end_y});
    }
  }
  return tracker.metrics().effective_coverage;
}

// 3.5 x 2.0 m plain room.
GridMap rectRoom()
{
  return makeMap(70, 40);
}

TEST(ScanlinePlannerTest, PlainRoomPlanCoversAndIsDeterministic)
{
  auto map = rectRoom();
  const auto masks = buildMasks(map, Point2D{0.5, 1.0});
  GridGeometry geo(map);

  ScanlinePlannerConfig cfg;
  RobotCleaningGeometry robot;
  ScanlinePlanner planner(geo, masks, robot, cfg);

  const auto p1 = planner.plan();
  ASSERT_TRUE(p1.valid());
  EXPECT_GT(p1.work_count, 5u);
  EXPECT_FALSE(p1.plan_id.empty());

  // Repeatable planning -> identical ids and plan digest.
  ScanlinePlanner planner2(geo, masks, robot, cfg);
  const auto p2 = planner2.plan();
  ASSERT_TRUE(p2.valid());
  EXPECT_EQ(p1.plan_id, p2.plan_id);
  EXPECT_EQ(p1.segments.size(), p2.segments.size());
  for (std::size_t i = 0; i < p1.segments.size(); ++i) {
    EXPECT_EQ(p1.segments[i].id, p2.segments[i].id);
  }

  // Work rows must be spaced no wider than the configured safe spacing
  // (0.28 m for the default geometry) and alternate in direction.
  const double max_gap = 0.28 + 1e-6;
  std::vector<double> rows;
  for (const auto & s : p1.segments) {
    if (s.type == SegmentType::WORK) {
      if (std::abs(p1.direction_rad) < 1e-6) {
        rows.push_back(s.start_y);
      } else {
        rows.push_back(s.start_x);
      }
    }
  }
  std::sort(rows.begin(), rows.end());
  for (std::size_t i = 1; i < rows.size(); ++i) {
    EXPECT_LE(rows[i] - rows[i - 1], max_gap);
  }

  // Simulated perfect execution must reach the MVP gate (>= 97%).
  const double cov = simulatedEffectiveCoverage(masks, p1);
  EXPECT_GE(cov, 0.97);

  // Every segment id is unique.
  std::set<std::string> ids;
  for (const auto & s : p1.segments) {
    EXPECT_TRUE(ids.insert(s.id).second) << "duplicate id " << s.id;
  }
}

TEST(ScanlinePlannerTest, SegmentEndpointsLieInNavigableSpace)
{
  auto map = rectRoom();
  const auto masks = buildMasks(map, Point2D{0.5, 1.0});
  GridGeometry geo(map);
  ScanlinePlannerConfig cfg;
  RobotCleaningGeometry robot;
  ScanlinePlanner planner(geo, masks, robot, cfg);
  const auto plan = planner.plan();
  ASSERT_TRUE(plan.valid());

  for (const auto & s : plan.segments) {
    if (s.type != SegmentType::WORK) {
      continue;
    }
    for (const auto & pt : {Point2D{s.start_x, s.start_y},
        Point2D{s.end_x, s.end_y}})
    {
      tunnel_map_core::GridCell cell;
      ASSERT_TRUE(geo.worldToGridCell(pt, cell));
      const auto idx = geo.index(cell.row, cell.col);
      EXPECT_EQ(masks.navigable_center[idx], 1u)
        << "work endpoint not navigable: " << pt.x << ", " << pt.y;
    }
  }
}

TEST(ScanlinePlannerTest, PillarRoomSplitsRowsWithoutCrossingObstacle)
{
  // Same room with a 0.2 x 0.2 m pillar at the centre.
  auto map = rectRoom();
  fillRect(map, 1.65, 0.9, 1.85, 1.1, OCC_OCCUPIED);
  const auto masks = buildMasks(map, Point2D{0.3, 1.0});
  GridGeometry geo(map);

  ScanlinePlannerConfig cfg;
  RobotCleaningGeometry robot;
  ScanlinePlanner planner(geo, masks, robot, cfg);
  const auto plan = planner.plan();
  ASSERT_TRUE(plan.valid());

  // No work segment may cross the pillar: every sampled cell along the
  // segment must be free (target or navigable), i.e. not occupied.
  for (const auto & s : plan.segments) {
    if (s.type != SegmentType::WORK) {
      continue;
    }
    const double len = s.lengthM();
    const int n = std::max(1, static_cast<int>(std::ceil(len / 0.05)));
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(n);
      const Point2D p{s.start_x + t * (s.end_x - s.start_x),
        s.start_y + t * (s.end_y - s.start_y)};
      tunnel_map_core::GridCell cell;
      ASSERT_TRUE(geo.worldToGridCell(p, cell));
      const auto idx = geo.index(cell.row, cell.col);
      EXPECT_NE(map.data[idx], OCC_OCCUPIED) << "work segment crosses pillar";
      EXPECT_EQ(masks.reachable_cleanable[idx], 1u);
    }
  }

  // A full-height pillar forces row splits: several work segments are
  // expected, plus transitions between them.
  EXPECT_GT(plan.work_count, 3u);
  std::size_t transitions = 0;
  for (const auto & s : plan.segments) {
    if (s.type == SegmentType::TRANSITION) {
      ++transitions;
    }
  }
  EXPECT_GT(transitions, 0u);

  // Deterministic too.
  ScanlinePlanner planner2(geo, masks, robot, cfg);
  EXPECT_EQ(planner2.plan().plan_id, plan.plan_id);
}

TEST(ScanlinePlannerTest, EmptyReachableRegionYieldsEmptyPlan)
{
  // A map whose only free space is a pocket narrower than the chassis
  // clearance: the seed is inside it, so builder throws; use a seed outside
  // the map instead is invalid too — here we verify the planner rejects
  // nothing-cleanable masks via an empty plan by constructing masks manually.
  auto map = rectRoom();
  GridGeometry geo(map);
  CoverageMasks empty;
  empty.geometry = geo;
  const auto n = geo.width() * geo.height();
  empty.intended_target.assign(n, 0);
  empty.navigable_center.assign(n, 0);
  empty.reachable_cleanable.assign(n, 0);
  empty.exempt.assign(n, 0);
  empty.exempt_cause.assign(n, 0);

  ScanlinePlannerConfig cfg;
  RobotCleaningGeometry robot;
  ScanlinePlanner planner(geo, empty, robot, cfg);
  const auto plan = planner.plan();
  EXPECT_FALSE(plan.valid());
  EXPECT_EQ(plan.work_count, 0u);
}

TEST(ScanlinePlannerTest, InvalidConfigRejected)
{
  auto map = rectRoom();
  GridGeometry geo(map);
  const auto masks = buildMasks(map, Point2D{0.5, 1.0});

  auto expect_invalid = [](auto && fn) -> bool {
      try {
        fn();
        return false;
      } catch (const std::invalid_argument &) {
        return true;
      } catch (...) {
        return false;
      }
    };

  ScanlinePlannerConfig cfg;
  cfg.overlap_eta = 1.5;
  RobotCleaningGeometry robot;
  EXPECT_TRUE(expect_invalid([&]() {
      ScanlinePlanner p(geo, masks, robot, cfg); (void)p;
  }));
  cfg.overlap_eta = 0.1;
  cfg.endpoint_inset_m = -1.0;
  EXPECT_TRUE(expect_invalid([&]() {
      ScanlinePlanner p(geo, masks, robot, cfg); (void)p;
  }));
}

}  // namespace
}  // namespace tunnel_coverage_planner
