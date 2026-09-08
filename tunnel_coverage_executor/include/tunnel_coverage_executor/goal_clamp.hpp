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

#ifndef TUNNEL_COVERAGE_EXECUTOR__GOAL_CLAMP_HPP_
#define TUNNEL_COVERAGE_EXECUTOR__GOAL_CLAMP_HPP_

#include <cstdint>
#include <vector>

#include "tunnel_map_core/grid_geometry.hpp"

namespace tunnel_coverage_executor
{

/// Outcome of a goal-endpoint clamp.
struct GoalClampResult
{
  double x = 0.0;
  double y = 0.0;
  bool clamped = false;
};

/// Clamp a goal endpoint into the nearest valid cell of a row-major 0-or-1
/// mask sharing the geometry.  For goal poses pass navigable_center (cells
/// whose centre the chassis can occupy); reachable_cleanable would be the
/// wrong notion -- it is dilated by the tool radius and keeps top-edge
/// goals Nav2 cannot generate.
///
/// An endpoint whose cell is outside the grid or not set in the mask is moved
/// to the nearest valid cell centre, keeping at least inset_m distance
/// from the mask bounding-box edge on the clamped axis.  Endpoints that
/// already sit on a valid cell are returned unchanged.  This is the
/// post-seal2 guard decided in review: make the goal generatable at the
/// source instead of relying on tracking noise to pass the endpoint
/// self-check -- tolerances are never relaxed.
GoalClampResult clampGoalEndpoint(
  const std::vector<std::uint8_t> & mask,
  const tunnel_map_core::GridGeometry & geometry,
  double x,
  double y,
  double inset_m);

}  // namespace tunnel_coverage_executor

#endif  // TUNNEL_COVERAGE_EXECUTOR__GOAL_CLAMP_HPP_
