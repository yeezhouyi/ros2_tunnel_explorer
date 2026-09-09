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

// U9 entrance-goal hysteresis (single mechanism, AE5) — pure decision
// function, no ROS dependencies, so the conservation tests run anywhere.
//
// Contract: while inside the cooldown window since the last ACCEPTED goal,
// a candidate within entrance_radius_m of that goal is rejected and the
// next-ranked candidate is taken.  Candidates outside the radius, and all
// candidates once the cooldown expires, pass through unchanged.  Disabled
// (enabled=false) -> zero behaviour change.
#ifndef TUNNEL_FRONTIER_EXPLORER__ENTRANCE_HYSTERESIS_HPP_
#define TUNNEL_FRONTIER_EXPLORER__ENTRANCE_HYSTERESIS_HPP_

#include <chrono>
#include <cmath>
#include <optional>

namespace tunnel_frontier_explorer
{

/// Returns true when @p candidate must be rejected by the hysteresis gate.
template<typename PointT>
bool entranceHysteresisRejects(
  bool enabled,
  const std::chrono::steady_clock::time_point & now,
  const std::optional<PointT> & last_accepted_goal,
  const std::chrono::steady_clock::time_point & last_accepted_time,
  const PointT & candidate,
  double cooldown_s,
  double radius_m)
{
  if (!enabled) {
    return false;
  }
  if (!last_accepted_goal.has_value()) {
    return false;
  }
  const double since = std::chrono::duration<double>(
    now - last_accepted_time).count();
  if (since >= cooldown_s) {
    return false;
  }
  const double d = std::hypot(
    candidate.x - last_accepted_goal->x,
    candidate.y - last_accepted_goal->y);
  return d < radius_m;
}

}  // namespace tunnel_frontier_explorer

#endif  // TUNNEL_FRONTIER_EXPLORER__ENTRANCE_HYSTERESIS_HPP_
