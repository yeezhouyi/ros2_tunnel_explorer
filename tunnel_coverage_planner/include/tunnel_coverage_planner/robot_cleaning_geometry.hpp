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

#ifndef TUNNEL_COVERAGE_PLANNER__ROBOT_CLEANING_GEOMETRY_HPP_
#define TUNNEL_COVERAGE_PLANNER__ROBOT_CLEANING_GEOMETRY_HPP_

#include <cmath>
#include <string>

namespace tunnel_coverage_planner
{

/// Unified chassis + cleaning-tool geometry (KTD4 of the cleaning track).
///
/// MVP contract:
///   - the chassis is modelled as a circle of radius `robot_radius_m` plus a
///     safety margin; the navigable-centre mask is the free space eroded by
///     `robot_radius_m + safety_margin_m`;
///   - the cleaning tool is *centred* on the chassis (`tool_offset_x/y == 0`)
///     and *rotationally symmetric* with effective width `cleaning_width_m`.
///     A non-centred or asymmetric footprint is rejected by validation (R19).
struct RobotCleaningGeometry
{
  /// Chassis bounding radius in metres (TurtleBot3: 0.13).
  double robot_radius_m = 0.13;

  /// Extra clearance added around the chassis centre, in metres.
  double safety_margin_m = 0.05;

  /// Tool centre offset in the robot base frame, in metres.
  /// MVP: must be (0, 0).
  double tool_offset_x_m = 0.0;
  double tool_offset_y_m = 0.0;

  /// Effective cleaning (sweep) width in metres; the virtual footprint used
  /// for coverage statistics is a disc of radius cleaning_width_m / 2.
  double cleaning_width_m = 0.50;

  /// Min area (m^2) of a cleanable connected region to keep; smaller islands
  /// are discarded (treated as exempt with reason "island").
  double min_cleanable_region_area_m2 = 0.04;

  /// Radius of the virtual cleaning footprint, metres.
  double toolSweepRadiusM() const {return 0.5 * cleaning_width_m;}

  /// Chassis-centre clearance radius, metres.
  double clearanceRadiusM() const {return robot_radius_m + safety_margin_m;}

  /// True when the geometry satisfies the MVP contract.
  bool valid() const
  {
    if (!(robot_radius_m > 0.0)) {return false;}
    if (!(safety_margin_m >= 0.0)) {return false;}
    if (!(cleaning_width_m > 0.0)) {return false;}
    if (!(min_cleanable_region_area_m2 >= 0.0)) {return false;}
    // MVP: centred, rotationally-symmetric footprint only.
    if (!(std::abs(tool_offset_x_m) < 1e-9)) {return false;}
    if (!(std::abs(tool_offset_y_m) < 1e-9)) {return false;}
    return true;
  }

  /// Human-readable failure reason (empty string when valid()).
  std::string validationError() const
  {
    if (!(robot_radius_m > 0.0)) {return "robot_radius_m must be > 0";}
    if (!(safety_margin_m >= 0.0)) {return "safety_margin_m must be >= 0";}
    if (!(cleaning_width_m > 0.0)) {return "cleaning_width_m must be > 0";}
    if (!(min_cleanable_region_area_m2 >= 0.0)) {
      return "min_cleanable_region_area_m2 must be >= 0";
    }
    if (!(std::abs(tool_offset_x_m) < 1e-9) ||
      !(std::abs(tool_offset_y_m) < 1e-9))
    {
      return "MVP requires a centred tool (tool_offset_x/y == 0); "
             "offset brushes are deferred";
    }
    return "";
  }
};

}  // namespace tunnel_coverage_planner

#endif  // TUNNEL_COVERAGE_PLANNER__ROBOT_CLEANING_GEOMETRY_HPP_
