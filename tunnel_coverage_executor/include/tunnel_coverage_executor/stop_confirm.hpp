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

#ifndef TUNNEL_COVERAGE_EXECUTOR__STOP_CONFIRM_HPP_
#define TUNNEL_COVERAGE_EXECUTOR__STOP_CONFIRM_HPP_

#include <cmath>
#include <optional>

namespace tunnel_coverage_executor
{

/// One robot state sample used by the cancel / stop-confirmation logic.
///
/// ``yaw`` is the heading about the Z axis (REP-103).  Translation and
/// rotation are checked *together*: "not observed moving" is not accepted as
/// "observed stationary" — a stop is only confirmed from consecutive valid
/// samples whose per-step translation AND rotation both stay within bounds.
struct StopSample
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

/// Classify one step between two consecutive valid samples.
///
/// @param prev previous valid sample (std::nullopt => cannot confirm yet)
/// @param cur  the newer sample
/// @param max_step_m    per-step translation bound for "stationary"
/// @param max_step_rad  per-step rotation bound for "stationary"
/// @return 0 stationary, 1 moving, -1 unknown (no previous sample).
inline int classifyStopSample(
  const std::optional<StopSample> & prev,
  const StopSample & cur,
  double max_step_m,
  double max_step_rad)
{
  if (!prev) {
    return -1;  // first valid sample only: nothing to compare against
  }
  const double dx = cur.x - prev->x;
  const double dy = cur.y - prev->y;
  double dyaw = cur.yaw - prev->yaw;
  while (dyaw > M_PI) {dyaw -= 2.0 * M_PI;}
  while (dyaw < -M_PI) {dyaw += 2.0 * M_PI;}
  if (std::hypot(dx, dy) > max_step_m || std::fabs(dyaw) > max_step_rad) {
    return 1;  // observed motion
  }
  return 0;  // observed stationary (translation and rotation within bounds)
}

}  // namespace tunnel_coverage_executor

#endif  // TUNNEL_COVERAGE_EXECUTOR__STOP_CONFIRM_HPP_
