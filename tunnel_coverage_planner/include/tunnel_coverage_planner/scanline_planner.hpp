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

#ifndef TUNNEL_COVERAGE_PLANNER__SCANLINE_PLANNER_HPP_
#define TUNNEL_COVERAGE_PLANNER__SCANLINE_PLANNER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "tunnel_coverage_planner/cleanable_map_builder.hpp"
#include "tunnel_coverage_planner/coverage_masks.hpp"
#include "tunnel_coverage_planner/robot_cleaning_geometry.hpp"
#include "tunnel_map_core/grid_geometry.hpp"

namespace tunnel_coverage_planner
{

/// Semantic type of a plan segment (R5).
enum class SegmentType : std::uint8_t
{
  WORK = 0,        ///< execute with FollowPath along the swept line
  TRANSITION = 1   ///< reposition with Nav2 NavigateToPose
};

/// One ordered path segment (work line or connector).
struct CoverageSegment
{
  /// Stable id derived from discretised grid geometry, e.g. "room0-w003-00".
  std::string id;
  /// Decomposition cell id ("room0" for the MVP single-region plan).
  std::string cell_id;
  SegmentType type = SegmentType::WORK;

  double start_x = 0.0;
  double start_y = 0.0;
  double start_yaw = 0.0;
  double end_x = 0.0;
  double end_y = 0.0;
  double end_yaw = 0.0;

  double lengthM() const;
};

/// A complete boustrophedon coverage plan for the current map.
struct CoveragePlan
{
  /// Bound later by the executor (map + geometry digest).
  std::string task_input_id;
  /// Digest of the canonical serialisation of this plan.
  std::string plan_id;
  /// Chosen sweep direction (radians in the map frame).
  double direction_rad = 0.0;
  /// Chosen strip spacing (m).
  double spacing_m = 0.0;
  /// Number of work segments.
  std::size_t work_count = 0;
  /// Ordered segments (work and transition).
  std::vector<CoverageSegment> segments;

  bool valid() const {return !segments.empty();}
};

/// Scanline planner configuration.
struct ScanlinePlannerConfig
{
  /// Desired overlap ratio eta in [0, 1): strip spacing <= w_clean * (1-eta).
  double overlap_eta = 0.10;
  /// Localisation error p95 (m) used by the conservative spacing formula.
  double loc_err_p95_m = 0.06;
  /// Tracking error p95 (m) used by the conservative spacing formula.
  double track_err_p95_m = 0.05;
  /// Work runs shorter than this are dropped (m).
  double min_segment_length_m = 0.40;
  /// Work segment endpoints are pulled in by this amount (m).
  ///
  /// Must stay small relative to the tool sweep radius: the wall-adjacent
  /// strip is covered only if the row end remains within one sweep radius of
  /// the strip cells (default = 1 cell keeps ~full coverage in a plain room).
  double endpoint_inset_m = 0.05;
  /// Extra cost weight per direction reversal.
  double weight_turn = 0.0;
  /// Extra cost weight per segment (drops many short segments).
  double weight_segment = 0.02;
};

/// Boustrophedon (scanline) coverage planner over the reachable-cleanable
/// mask.  Pure algorithm, no ROS (R18).
///
/// MVP scope (C3): one connected region; candidate sweep directions are the
/// map axes (0 and pi/2); rows are spaced by
///   spacing = min(w_clean * (1 - eta), w_clean - 2*(e_loc + e_track))
/// with the first/last row placed w_clean/2 from the region bounds so the
/// centred footprint reaches both walls.
class ScanlinePlanner
{
public:
  /// @throws std::invalid_argument on invalid config / geometry / masks.
  ScanlinePlanner(
    const tunnel_map_core::GridGeometry & geometry,
    const CoverageMasks & masks,
    const RobotCleaningGeometry & robot,
    ScanlinePlannerConfig config);

  /// Build the plan.
  ///
  /// @return Plan over the effective region; empty plan when the
  ///         reachable-cleanable area is empty.
  CoveragePlan plan();

private:
  tunnel_map_core::GridGeometry geometry_;
  CoverageMasks masks_;
  RobotCleaningGeometry robot_;
  ScanlinePlannerConfig config_;

  double spacingM() const;

  /// Strip offsets (row positions) that tile [lo, hi] with tool reach r_tool.
  static std::vector<double> rowOffsets(double lo, double hi, double r_tool,
    double spacing);

  /// Build ordered segments for a sweep along @p axis_angle (0 or pi/2).
  /// @param run_direction_sign +1/-1 for the first row.
  CoveragePlan buildSweep(double axis_angle, bool & ok) const;

  /// Deterministic serialisation used for plan_id.
  std::string serialize(const CoveragePlan & plan) const;
};

}  // namespace tunnel_coverage_planner

#endif  // TUNNEL_COVERAGE_PLANNER__SCANLINE_PLANNER_HPP_
