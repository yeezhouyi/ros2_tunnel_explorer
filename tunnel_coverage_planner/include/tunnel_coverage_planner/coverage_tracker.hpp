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

#ifndef TUNNEL_COVERAGE_PLANNER__COVERAGE_TRACKER_HPP_
#define TUNNEL_COVERAGE_PLANNER__COVERAGE_TRACKER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "tunnel_coverage_planner/coverage_masks.hpp"
#include "tunnel_map_core/grid_geometry.hpp"
#include "tunnel_map_core/grid_map.hpp"

namespace tunnel_coverage_planner
{

/// One sample of the `map -> cleaning_tool` pose.
struct ToolPose
{
  tunnel_map_core::Point2D position;
  double yaw = 0.0;
};

/// Online coverage metrics over the *effective* denominator T \ E and the
/// gross denominator T (R4, R23).
struct CoverageMetrics
{
  /// |(T \ E) ∩ V| / |T \ E|
  double effective_coverage = 0.0;
  /// |T ∩ V| / |T|
  double gross_coverage = 0.0;
  /// |E| / |T|
  double exempt_ratio = 0.0;
  /// |{c in T \ E : visits(c) > 1}| / |T \ E|
  double repeat_ratio = 0.0;

  /// Area actually swept at least once, m^2 (unique coverage).
  double unique_swept_area_m2 = 0.0;
  /// Sum of every tool-disc marking (repeat-inclusive), m^2.
  double total_swept_area_m2 = 0.0;
  /// Accumulated path length between accepted poses, m.
  double path_length_m = 0.0;

  double intended_area_m2 = 0.0;
  double effective_area_m2 = 0.0;  // |T \ E|
  double exempt_area_m2 = 0.0;
};

/// Tracks which cells the real `map -> cleaning_tool` trajectory has swept.
///
/// Pure algorithm.  The ROS executor decides *when* a pose is fresh enough to
/// feed the tracker (TF staleness, time reversal, frozen localisation); this
/// class only performs the geometric sweep.  A pose pair is interpolated at
/// steps no larger than half a cell before the circular cleaning footprint is
/// stamped, so sparse odometry does not create holes (R3, R19).
class CoverageTracker
{
public:
  /// @param geometry    Frozen map geometry (same one used for the masks).
  /// @param masks       Coverage masks; denominators are read from here.
  /// @param sweep_radius_m  Radius of the virtual circular cleaning
  ///                        footprint (= cleaning_width_m / 2).
  CoverageTracker(
    const tunnel_map_core::GridGeometry & geometry,
    const CoverageMasks & masks,
    double sweep_radius_m);

  /// Sweep the footprint along the straight segment [a, b] (world frame).
  void addSweepSegment(
    const tunnel_map_core::Point2D & a, const tunnel_map_core::Point2D & b);

  /// Stamp the footprint once at @p pose (zero-length segment).
  void addToolPose(const ToolPose & pose);

  /// Recompute all metrics from current visit counts.
  CoverageMetrics metrics() const;

  /// Per-cell visit counts (row-major; read-only).
  const std::vector<std::int32_t> & visitCounts() const {return counts_;}

  /// Total number of cells with visits > 0.
  std::size_t uniqueCoveredCells() const;

  /// True when every effective target cell has at least one visit.
  bool effectivelyComplete() const;

  /// Forget all state (keeps geometry/masks).
  void reset();

  /// Restore from a checkpoint: visit counts + accumulated path length.
  /// @throws std::invalid_argument when the counts vector size differs.
  void restore(const std::vector<std::int32_t> & counts, double path_length_m);

private:
  tunnel_map_core::GridGeometry geometry_;
  CoverageMasks masks_;
  double sweep_radius_m_;
  double max_step_m_;
  std::vector<std::int32_t> counts_;
  double path_length_m_ = 0.0;
  /// Sum of per-stamp marked areas (repeat-inclusive), m^2.
  double total_swept_area_m2_ = 0.0;

  /// Stamp the cleaning disc at @p centre; returns the number of cells marked.
  std::size_t stampDisc(const tunnel_map_core::Point2D & centre);
};

}  // namespace tunnel_coverage_planner

#endif  // TUNNEL_COVERAGE_PLANNER__COVERAGE_TRACKER_HPP_
