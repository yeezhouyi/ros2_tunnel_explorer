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
///
/// ``stamp_s`` is the source timestamp of the sample in seconds (the /tf
/// transform stamp).  It exists so a stop cannot be confirmed from repeated
/// reads of the SAME localization (frozen / not-advancing stamp) and so the
/// per-step motion bound scales with the ACTUAL inter-sample interval
/// instead of a fixed 0.2 s conversion.
struct StopSample
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double stamp_s = 0.0;
};

/// Time-aware classification of one step between two samples.
///
/// The per-step translation / rotation budgets are derived from per-second
/// thresholds and the ACTUAL elapsed time between the two stamps:
///   step_m  = max_speed_mps * dt
///   step_rad = max_yaw_rate_radps * dt
/// where dt = cur.stamp_s - prev.stamp_s.
///
/// Returns:
///   -1  unknown: no previous sample, or dt is not a usable positive finite
///       interval (frozen stamp, re-sent identical sample, gap > max_gap_s)
///   0   stationary: translation and rotation within the per-step bounds
///   1   moving: either exceeds its per-step bound
inline int classifyStopSampleTimed(
  const std::optional<StopSample> & prev,
  const StopSample & cur,
  double max_speed_mps,
  double max_yaw_rate_radps,
  double max_gap_s = 2.0)
{
  if (!prev) {
    return -1;  // first valid sample only: nothing to compare against
  }
  const double dt = cur.stamp_s - prev->stamp_s;
  if (!std::isfinite(dt) || dt <= 0.0 || dt > max_gap_s) {
    // Not a new observation (frozen / repeated same stamp) or the gap is too
    // large to be one integration step -- never counts as stationary.
    return -1;
  }
  const double dx = cur.x - prev->x;
  const double dy = cur.y - prev->y;
  double dyaw = cur.yaw - prev->yaw;
  while (dyaw > M_PI) {dyaw -= 2.0 * M_PI;}
  while (dyaw < -M_PI) {dyaw += 2.0 * M_PI;}
  const double step_m = max_speed_mps * dt;
  const double step_rad = max_yaw_rate_radps * dt;
  if (std::hypot(dx, dy) > step_m || std::fabs(dyaw) > step_rad) {
    return 1;  // observed motion over a valid interval
  }
  return 0;  // observed stationary over a valid interval
}

}  // namespace tunnel_coverage_executor

#endif  // TUNNEL_COVERAGE_EXECUTOR__STOP_CONFIRM_HPP_
