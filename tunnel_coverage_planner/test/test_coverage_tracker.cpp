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
#include <vector>

#include "tunnel_coverage_planner/coverage_masks.hpp"
#include "tunnel_coverage_planner/coverage_tracker.hpp"
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

// 2.0 x 1.5 m all-free map: 40 x 30 cells at 0.05 m, origin (0, 0).
GridMap makeRectMap()
{
  GridMap map;
  map.width = 40;
  map.height = 30;
  map.resolution = 0.05;
  map.origin_x = 0.0;
  map.origin_y = 0.0;
  map.origin_yaw = 0.0;
  map.data.assign(40 * 30, OCC_FREE);
  return map;
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

/// Exempt the world-x band [x_lo, x_hi) as UNREACHABLE.
CoverageMasks masksWithExemptBand(
  const GridGeometry & geo, double x_lo, double x_hi)
{
  auto m = allFreeMasks(geo);
  const auto n = geo.width() * geo.height();
  for (std::size_t row = 0; row < geo.height(); ++row) {
    for (std::size_t col = 0; col < geo.width(); ++col) {
      const auto w = geo.gridToWorld(static_cast<int>(row), static_cast<int>(col));
      if (w.x >= x_lo && w.x < x_hi) {
        const auto idx = row * geo.width() + col;
        m.exempt[idx] = 1;
        m.exempt_cause[idx] = static_cast<std::uint8_t>(ExemptCause::UNREACHABLE);
      }
    }
  }
  return m;
}

TEST(CoverageTrackerTest, SinglePoseMarksDisc)
{
  auto map = makeRectMap();
  GridGeometry geo(map);
  const auto masks = allFreeMasks(geo);
  CoverageTracker tracker(geo, masks, 0.25);  // cleaning width 0.5 m

  tracker.addToolPose(ToolPose{Point2D{1.0, 0.75}, 0.0});

  const auto m = tracker.metrics();
  // Expected disc area pi * 0.25^2 within +/- 2% (discretisation).
  const double expected = M_PI * 0.25 * 0.25;
  const double cell_area = 0.05 * 0.05;
  const double measured = static_cast<double>(tracker.uniqueCoveredCells()) * cell_area;
  EXPECT_NEAR(measured, expected, 0.02 * expected);
  EXPECT_DOUBLE_EQ(m.path_length_m, 0.0);
  EXPECT_GT(m.effective_coverage, 0.0);
  EXPECT_LT(m.effective_coverage, 0.2);
}

TEST(CoverageTrackerTest, SparseAndDenseSamplingAgree)
{
  auto map = makeRectMap();
  GridGeometry geo(map);
  const auto masks = allFreeMasks(geo);
  CoverageTracker dense(geo, masks, 0.25);
  CoverageTracker sparse(geo, masks, 0.25);

  // Same straight corridor, interior of the map.
  const Point2D a{0.2, 0.75};
  const Point2D b{1.8, 0.75};

  // Dense input: many small poses.
  constexpr int kDense = 200;
  for (int i = 0; i < kDense; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(kDense - 1);
    dense.addSweepSegment(a, Point2D{a.x + t * (b.x - a.x), a.y});
  }
  // Sparse input: one long segment — the tracker interpolates internally at
  // <= half a cell, so the swept set must match.
  sparse.addSweepSegment(a, b);

  EXPECT_EQ(dense.visitCounts(), sparse.visitCounts());
  EXPECT_NEAR(dense.metrics().path_length_m, 1.6, 1e-9);
  EXPECT_NEAR(sparse.metrics().path_length_m, 1.6, 1e-9);
}

TEST(CoverageTrackerTest, StraightLineSweepWidth)
{
  auto map = makeRectMap();
  GridGeometry geo(map);
  const auto masks = allFreeMasks(geo);
  CoverageTracker tracker(geo, masks, 0.25);

  tracker.addSweepSegment(Point2D{0.3, 0.75}, Point2D{1.7, 0.75});

  const auto m = tracker.metrics();
  // Swept set = 0.5 m x 1.4 m rectangle plus two end half-discs of radius
  // 0.25 m (the stamp chain sweeps a full disc at every pose).
  const double expected_area = 0.5 * 1.4 + M_PI * 0.25 * 0.25;
  const double cell_area = 0.05 * 0.05;
  const double measured = static_cast<double>(tracker.uniqueCoveredCells()) * cell_area;
  EXPECT_NEAR(measured, expected_area, 0.03 * expected_area);
  EXPECT_NEAR(m.path_length_m, 1.4, 1e-9);
}

TEST(CoverageTrackerTest, BackAndForthCreatesRepeats)
{
  auto map = makeRectMap();
  GridGeometry geo(map);
  const auto masks = allFreeMasks(geo);
  CoverageTracker tracker(geo, masks, 0.25);

  tracker.addSweepSegment(Point2D{0.4, 0.75}, Point2D{1.6, 0.75});
  const auto after_one = tracker.metrics();
  EXPECT_DOUBLE_EQ(after_one.repeat_ratio, 0.0);

  // Second pass over the same band.
  tracker.addSweepSegment(Point2D{1.6, 0.75}, Point2D{0.4, 0.75});
  const auto after_two = tracker.metrics();

  EXPECT_GT(after_two.repeat_ratio, 0.0);
  // Covered set (unique) may only grow by tiny end effects; repeat cells are
  // inside the effective set, so effective coverage cannot decrease.
  EXPECT_GE(after_two.effective_coverage, after_one.effective_coverage - 1e-9);
  EXPECT_GT(after_two.total_swept_area_m2, after_two.unique_swept_area_m2);
}

TEST(CoverageTrackerTest, OffMapSweepIsIgnored)
{
  auto map = makeRectMap();
  GridGeometry geo(map);
  const auto masks = allFreeMasks(geo);
  CoverageTracker tracker(geo, masks, 0.25);

  tracker.addSweepSegment(Point2D{10.0, 10.0}, Point2D{11.0, 10.0});
  EXPECT_EQ(tracker.uniqueCoveredCells(), 0u);
  EXPECT_DOUBLE_EQ(tracker.metrics().effective_coverage, 0.0);
}

TEST(CoverageTrackerTest, ExemptBandAffectsEffectiveOnly)
{
  auto map = makeRectMap();
  GridGeometry geo(map);
  // Exempt the world-x band [0.0, 0.6); use a narrow brush (radius 0.1 m)
  // whose sweep never bleeds across the 0.6 boundary.
  const auto masks = masksWithExemptBand(geo, 0.0, 0.6);
  CoverageTracker tracker(geo, masks, 0.10);

  // Sweep only the middle of the exempt band (x in [0.2, 0.4]).
  tracker.addSweepSegment(Point2D{0.3, 0.1}, Point2D{0.3, 1.4});
  const auto m = tracker.metrics();

  EXPECT_EQ(m.exempt_ratio, 0.6 / 2.0);  // 12 exempt cols of 40
  EXPECT_NEAR(m.effective_coverage, 0.0, 1e-9);
  EXPECT_GT(m.gross_coverage, 0.0);
  EXPECT_LT(m.gross_coverage, 0.5);

  // Now cover the whole effective region with parallel vertical sweeps at
  // spacing <= 2 * radius; the exempt band stays untouched.
  CoverageTracker tracker2(geo, masks, 0.10);
  for (double x = 0.7; x <= 1.9; x += 0.19) {
    tracker2.addSweepSegment(Point2D{x, 0.05}, Point2D{x, 1.45});
  }
  tracker2.addSweepSegment(Point2D{1.9, 0.05}, Point2D{1.9, 1.45});
  const auto m2 = tracker2.metrics();

  EXPECT_GE(m2.effective_coverage, 0.99);
  EXPECT_GT(m2.gross_coverage, 0.60);
  EXPECT_LT(m2.gross_coverage, 0.80);
  EXPECT_NEAR(m2.exempt_ratio, 0.3, 1e-9);
}

TEST(CoverageTrackerTest, ResetClearsState)
{
  auto map = makeRectMap();
  GridGeometry geo(map);
  const auto masks = allFreeMasks(geo);
  CoverageTracker tracker(geo, masks, 0.25);
  tracker.addSweepSegment(Point2D{0.2, 0.75}, Point2D{1.8, 0.75});
  EXPECT_GT(tracker.uniqueCoveredCells(), 0u);

  tracker.reset();
  EXPECT_EQ(tracker.uniqueCoveredCells(), 0u);
  EXPECT_DOUBLE_EQ(tracker.metrics().path_length_m, 0.0);
  EXPECT_DOUBLE_EQ(tracker.metrics().effective_coverage, 0.0);
}

TEST(CoverageTrackerTest, GridSwitchResetsState)
{
  auto map = makeRectMap();
  GridGeometry geo1(map);
  const auto masks1 = allFreeMasks(geo1);
  CoverageTracker tracker(geo1, masks1, 0.25);
  tracker.addSweepSegment(Point2D{0.2, 0.75}, Point2D{1.8, 0.75});
  EXPECT_GT(tracker.uniqueCoveredCells(), 0u);

  // A new tracker over a different geometry starts clean (map-switch path).
  GridMap map2 = map;
  map2.origin_x = 5.0;
  GridGeometry geo2(map2);
  const auto masks2 = allFreeMasks(geo2);
  CoverageTracker tracker2(geo2, masks2, 0.25);
  EXPECT_EQ(tracker2.uniqueCoveredCells(), 0u);
}

}  // namespace
}  // namespace tunnel_coverage_planner
