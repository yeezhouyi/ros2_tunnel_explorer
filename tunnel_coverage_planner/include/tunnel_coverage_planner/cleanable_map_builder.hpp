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

#ifndef TUNNEL_COVERAGE_PLANNER__CLEANABLE_MAP_BUILDER_HPP_
#define TUNNEL_COVERAGE_PLANNER__CLEANABLE_MAP_BUILDER_HPP_

#include <cstdint>
#include <vector>

#include "tunnel_coverage_planner/coverage_masks.hpp"
#include "tunnel_coverage_planner/robot_cleaning_geometry.hpp"
#include "tunnel_map_core/grid_geometry.hpp"
#include "tunnel_map_core/grid_map.hpp"

namespace tunnel_coverage_planner
{

/// Occupancy classification thresholds for the builder.
struct CleanableMapBuilderConfig
{
  /// Cells with value in [0, free_max] count as free.
  int free_max = 0;
  /// Cells with value >= occupied_min count as occupied.
  int occupied_min = 65;
  /// 4- or 8-connectivity for connected-component analysis.
  int connectivity = 8;
};

/// Builds the four coverage masks from a frozen map and robot geometry.
///
/// Pure algorithm, no ROS.  Semantics (see docs/cleaning_track_c1_geometry.md):
///   1. classify free / occupied / unknown;
///   2. intended_target = free cells (cleaning target);
///   3. navigable_center = free cells eroded by chassis clearance radius;
///   4. reachable_cleanable = target cells inside the tool-reach dilation of
///      the navigable component connected to the seed pose;
///   5. exempt = intended_target \ reachable_cleanable, with per-cell cause.
class CleanableMapBuilder
{
public:
  /// @throws std::invalid_argument on invalid map / geometry / config.
  CleanableMapBuilder(
    const tunnel_map_core::GridGeometry & geometry,
    CleanableMapBuilderConfig config);

  /// Build masks from @p map content.
  ///
  /// @param map        Frozen occupancy grid; geometry must match `geometry_`.
  /// @param robot      Chassis + cleaning tool geometry (must be valid()).
  /// @param seed_world Seed pose (robot start) in world coordinates.
  /// @return Four mutually consistent masks.
  /// @throws std::invalid_argument if map dimensions/geometry mismatch, robot
  ///         geometry is invalid, or the seed is outside the map.
  CoverageMasks build(
    const tunnel_map_core::GridMap & map,
    const RobotCleaningGeometry & robot,
    const tunnel_map_core::Point2D & seed_world) const;

private:
  tunnel_map_core::GridGeometry geometry_;
  CleanableMapBuilderConfig config_;

  /// Cells free / occupied / unknown (see config thresholds).
  void classify(
    const tunnel_map_core::GridMap & map,
    std::vector<std::uint8_t> & free_mask,
    std::vector<std::uint8_t> & occupied_mask) const;

  /// Free cells whose whole chassis-clearance disc is free.
  std::vector<std::uint8_t> erodeByDisc(
    const std::vector<std::uint8_t> & free_mask, double radius_cells) const;

  /// Cells whose centre is inside `radius_cells` of a true cell of `src`.
  std::vector<std::uint8_t> dilateByDisc(
    const std::vector<std::uint8_t> & src, double radius_cells) const;

  /// BFS from seed over `src` (4/8-connected); result is the component mask.
  std::vector<std::uint8_t> connectedComponentFrom(
    const std::vector<std::uint8_t> & src,
    std::size_t seed_index) const;
};

}  // namespace tunnel_coverage_planner

#endif  // TUNNEL_COVERAGE_PLANNER__CLEANABLE_MAP_BUILDER_HPP_
