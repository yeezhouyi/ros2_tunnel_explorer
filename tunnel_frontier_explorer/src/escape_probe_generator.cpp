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

#include "tunnel_frontier_explorer/escape_probe_generator.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace tunnel_frontier_explorer
{

// ── Constructor ─────────────────────────────────────────────────────────────

EscapeProbeGenerator::EscapeProbeGenerator(EscapeProbeConfig config)
: config_(config)
{
}

// ── isFree ──────────────────────────────────────────────────────────────────

bool EscapeProbeGenerator::isFree(
  const Point2D & point, const GridMap & map) const
{
  const int col = static_cast<int>(
    std::round((point.x - map.origin_x) / map.resolution - 0.5));
  const int row = static_cast<int>(
    std::round((point.y - map.origin_y) / map.resolution - 0.5));

  if (row < 0 || row >= static_cast<int>(map.height) ||
      col < 0 || col >= static_cast<int>(map.width))
  {
    return false;
  }

  const std::size_t idx = static_cast<std::size_t>(row) * map.width +
                          static_cast<std::size_t>(col);
  return map.data[idx] == 0;  // 0 = free
}

// ── hasWallClearance ────────────────────────────────────────────────────────

bool EscapeProbeGenerator::hasWallClearance(
  const Point2D & point, const GridMap & map) const
{
  const int clearance_cells = static_cast<int>(
    std::ceil(config_.min_wall_distance_m / map.resolution));
  const int centre_col = static_cast<int>(
    std::round((point.x - map.origin_x) / map.resolution - 0.5));
  const int centre_row = static_cast<int>(
    std::round((point.y - map.origin_y) / map.resolution - 0.5));

  const int min_row = std::max(0, centre_row - clearance_cells);
  const int max_row = std::min(
    static_cast<int>(map.height) - 1, centre_row + clearance_cells);
  const int min_col = std::max(0, centre_col - clearance_cells);
  const int max_col = std::min(
    static_cast<int>(map.width) - 1, centre_col + clearance_cells);

  const double radius_sq =
    config_.min_wall_distance_m * config_.min_wall_distance_m;

  for (int r = min_row; r <= max_row; ++r) {
    for (int c = min_col; c <= max_col; ++c) {
      const double wx = map.origin_x +
        (static_cast<double>(c) + 0.5) * map.resolution;
      const double wy = map.origin_y +
        (static_cast<double>(r) + 0.5) * map.resolution;
      const double dx = wx - point.x;
      const double dy = wy - point.y;

      if (dx * dx + dy * dy > radius_sq) {continue;}

      const std::size_t idx = static_cast<std::size_t>(r) * map.width +
                              static_cast<std::size_t>(c);
      if (map.data[idx] == 100) {return false;}  // occupied → too close
    }
  }
  return true;
}

// ── generateEscapeProbe ─────────────────────────────────────────────────────

std::optional<Point2D> EscapeProbeGenerator::generateEscapeProbe(
  const Point2D & robot,
  const Point2D & oscillation_center,
  const Point2D & preferred_direction,
  const GridMap & map,
  double suppression_radius) const
{
  // Compute base direction away from oscillillation center
  const double dx_pref = preferred_direction.x - oscillation_center.x;
  const double dy_pref = preferred_direction.y - oscillation_center.y;
  const double norm_pref = std::hypot(dx_pref, dy_pref);

  double base_ux, base_uy;
  if (norm_pref > 0.01) {
    base_ux = dx_pref / norm_pref;
    base_uy = dy_pref / norm_pref;
  } else {
    // Fallback: use robot direction
    const double dx_robot = robot.x - oscillation_center.x;
    const double dy_robot = robot.y - oscillation_center.y;
    const double norm_robot = std::hypot(dx_robot, dy_robot);
    if (norm_robot > 0.01) {
      base_ux = dx_robot / norm_robot;
      base_uy = dy_robot / norm_robot;
    } else {
      // Last resort: point north
      base_ux = 0.0;
      base_uy = 1.0;
    }
  }

  // Search angle offsets and distances
  const double half_span_rad = config_.angle_span_deg * M_PI / 360.0;
  const double exit_threshold = suppression_radius + config_.probe_exit_margin_m;

  // Angle offsets: 0°, ±15°, ±30°, ±45° (up to half_span)
  std::vector<double> angle_offsets_rad = {0.0};
  const double step_rad = 15.0 * M_PI / 180.0;
  for (double a = step_rad; a <= half_span_rad + 1e-6; a += step_rad) {
    angle_offsets_rad.push_back(a);
    angle_offsets_rad.push_back(-a);
  }

  int attempts = 0;
  for (double dist = config_.probe_distance_m;
       attempts < config_.max_attempts;
       dist += config_.probe_distance_step_m, ++attempts)
  {
    for (double angle_offset : angle_offsets_rad) {
      const double cos_a = std::cos(angle_offset);
      const double sin_a = std::sin(angle_offset);
      const double ux = base_ux * cos_a - base_uy * sin_a;
      const double uy = base_ux * sin_a + base_uy * cos_a;

      const Point2D probe = {
        oscillation_center.x + dist * ux,
        oscillation_center.y + dist * uy
      };

      // Must be outside oscillillation radius
      const double dist_to_center = std::hypot(
        probe.x - oscillation_center.x,
        probe.y - oscillation_center.y);
      if (dist_to_center <= exit_threshold) {continue;}

      // Must be free in occupancy grid
      if (!isFree(probe, map)) {continue;}

      // Must have wall clearance
      if (!hasWallClearance(probe, map)) {continue;}

      return probe;
    }
  }

  return std::nullopt;
}

}  // namespace tunnel_frontier_explorer
