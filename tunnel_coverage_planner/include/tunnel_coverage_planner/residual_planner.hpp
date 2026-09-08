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

#ifndef TUNNEL_COVERAGE_PLANNER__RESIDUAL_PLANNER_HPP_
#define TUNNEL_COVERAGE_PLANNER__RESIDUAL_PLANNER_HPP_

#include <cstdint>
#include <vector>

#include "tunnel_coverage_planner/coverage_masks.hpp"
#include "tunnel_coverage_planner/robot_cleaning_geometry.hpp"
#include "tunnel_coverage_planner/scanline_planner.hpp"
#include "tunnel_map_core/grid_geometry.hpp"

namespace tunnel_coverage_planner
{

/// Result of residual analysis after the main coverage pass (R8, U7).
struct ResidualResult
{
  /// One local scanline patch plan per residual region large enough to
  /// clean (executor appends these after the main plan).
  std::vector<CoveragePlan> patches;
  /// Total uncovered effective-target cells before patching.
  std::size_t total_uncovered_cells = 0;
  /// Uncovered cells below the minimum-area gate (auditable exemptions,
  /// cause ISLAND-equivalent "residual-too-small").
  std::size_t exempted_uncovered_cells = 0;
};

/// Finds cells the main pass missed and plans local patches for them.
///
/// Pure algorithm, no ROS.  Operates on the *effective* target
/// (intended minus exempt); a cell is "covered" when its visit count > 0.
/// Uncovered cells are grouped into 8-connected components; components at
/// or above the minimum cleanable area get a local scanline patch, smaller
/// leftovers are returned as exempted (never silently dropped, R23).
class ResidualPlanner
{
public:
  /// @throws std::invalid_argument on inconsistent masks / invalid geometry.
  ResidualPlanner(
    const tunnel_map_core::GridGeometry & geometry,
    const CoverageMasks & masks,
    const RobotCleaningGeometry & robot,
    const ScanlinePlannerConfig & config);

  /// Analyse @p visit_counts (row-major, one entry per cell) and produce
  /// patch plans for every residual region above the minimum area.
  ResidualResult planResidual(
    const std::vector<std::int32_t> & visit_counts) const;

private:
  tunnel_map_core::GridGeometry geometry_;
  CoverageMasks masks_;
  RobotCleaningGeometry robot_;
  ScanlinePlannerConfig config_;
};

}  // namespace tunnel_coverage_planner

#endif  // TUNNEL_COVERAGE_PLANNER__RESIDUAL_PLANNER_HPP_
