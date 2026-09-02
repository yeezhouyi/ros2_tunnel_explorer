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
#include <vector>

#include "tunnel_coverage_planner/cleanable_map_builder.hpp"
#include "tunnel_coverage_planner/coverage_masks.hpp"
#include "tunnel_coverage_planner/coverage_tracker.hpp"
#include "tunnel_coverage_planner/residual_planner.hpp"
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
using tunnel_map_core::Point2D;

GridMap room()
{
  GridMap map;
  map.width = 70;
  map.height = 40;
  map.resolution = 0.05;
  map.origin_x = 0.0;
  map.origin_y = 0.0;
  map.origin_yaw = 0.0;
  map.data.assign(70 * 40, OCC_FREE);
  return map;
}

CoverageMasks buildMasks(const GridMap & map, const Point2D & seed)
{
  GridGeometry geo(map);
  CleanableMapBuilderConfig cfg;
  CleanableMapBuilder builder(geo, cfg);
  RobotCleaningGeometry robot;
  return builder.build(map, robot, seed);
}

/// Execute the given segments into visit counts (optionally skipping every
/// third *work* row to simulate an interrupted pass).
std::vector<std::int32_t> execute(
  const CoverageMasks & masks,
  const std::vector<CoverageSegment> & segments,
  bool skip_third)
{
  CoverageTracker tracker(masks.geometry, masks, 0.25);
  std::size_t work_index = 0;
  for (const auto & s : segments) {
    if (s.type != SegmentType::WORK) {
      continue;
    }
    if (!(skip_third && work_index % 3 == 1)) {
      tracker.addSweepSegment(
        Point2D{s.start_x, s.start_y}, Point2D{s.end_x, s.end_y});
    }
    ++work_index;
  }
  return tracker.visitCounts();
}

TEST(ResidualPlannerTest, FullCoverageHasNoResidual)
{
  auto map = room();
  const auto masks = buildMasks(map, Point2D{0.5, 1.0});
  GridGeometry geo(map);
  RobotCleaningGeometry robot;
  ScanlinePlannerConfig cfg;
  ScanlinePlanner planner(geo, masks, robot, cfg);
  const auto main = planner.plan();
  ASSERT_TRUE(main.valid());

  const auto counts = execute(masks, main.segments, false);
  ResidualPlanner residual(geo, masks, robot, cfg);
  const auto r = residual.planResidual(counts);
  EXPECT_EQ(r.total_uncovered_cells, 0u);
  EXPECT_TRUE(r.patches.empty());
}

TEST(ResidualPlannerTest, MissingRowsProducePatchesThatCloseTheGap)
{
  auto map = room();
  const auto masks = buildMasks(map, Point2D{0.5, 1.0});
  GridGeometry geo(map);
  RobotCleaningGeometry robot;
  ScanlinePlannerConfig cfg;
  ScanlinePlanner planner(geo, masks, robot, cfg);
  const auto main = planner.plan();
  ASSERT_TRUE(main.valid());

  // Simulate a partial execution that skips every third work row.
  const auto partial_counts = execute(masks, main.segments, true);

  ResidualPlanner residual(geo, masks, robot, cfg);
  const auto r = residual.planResidual(partial_counts);
  EXPECT_GT(r.total_uncovered_cells, 0u);
  EXPECT_FALSE(r.patches.empty());

  // Executing main (kept rows) + every patch must restore ~full coverage.
  CoverageTracker tracker(geo, masks, 0.25);
  std::size_t work_index = 0;
  for (const auto & s : main.segments) {
    if (s.type != SegmentType::WORK) {
      continue;
    }
    if (work_index % 3 != 1) {
      tracker.addSweepSegment(
        Point2D{s.start_x, s.start_y}, Point2D{s.end_x, s.end_y});
    }
    ++work_index;
  }
  for (const auto & patch : r.patches) {
    for (const auto & s : patch.segments) {
      if (s.type == SegmentType::WORK) {
        tracker.addSweepSegment(
          Point2D{s.start_x, s.start_y}, Point2D{s.end_x, s.end_y});
      }
    }
  }
  const double final_cov = tracker.metrics().effective_coverage;
  EXPECT_GE(final_cov, 0.99) << "final effective coverage = " << final_cov;

  // Deterministic residual analysis.
  const auto r2 = residual.planResidual(partial_counts);
  ASSERT_EQ(r2.patches.size(), r.patches.size());
  for (std::size_t p = 0; p < r.patches.size(); ++p) {
    EXPECT_EQ(r2.patches[p].plan_id, r.patches[p].plan_id);
  }
}

TEST(ResidualPlannerTest, TinyLeftoverIsExemptedNotDropped)
{
  auto map = room();
  const auto masks = buildMasks(map, Point2D{0.5, 1.0});
  GridGeometry geo(map);
  RobotCleaningGeometry robot;
  ScanlinePlannerConfig cfg;
  ScanlinePlanner planner(geo, masks, robot, cfg);
  const auto main = planner.plan();
  ASSERT_TRUE(main.valid());

  // Full coverage, then punch a tiny 2x2 uncovered hole (4 cells < min gate).
  auto counts = execute(masks, main.segments, false);
  const std::size_t w = geo.width();
  const std::size_t hole_idx = 30 * w + 30;
  counts[hole_idx] = 0;
  counts[hole_idx + 1] = 0;
  counts[hole_idx + w] = 0;
  counts[hole_idx + w + 1] = 0;

  ResidualPlanner residual(geo, masks, robot, cfg);
  const auto r = residual.planResidual(counts);
  EXPECT_EQ(r.total_uncovered_cells, 4u);
  EXPECT_TRUE(r.patches.empty());
  EXPECT_EQ(r.exempted_uncovered_cells, 4u);
}

}  // namespace
}  // namespace tunnel_coverage_planner
