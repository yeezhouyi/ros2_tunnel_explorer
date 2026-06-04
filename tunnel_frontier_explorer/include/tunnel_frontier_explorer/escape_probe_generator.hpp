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

#ifndef TUNNEL_FRONTIER_EXPLORER__ESCAPE_PROBE_GENERATOR_HPP_
#define TUNNEL_FRONTIER_EXPLORER__ESCAPE_PROBE_GENERATOR_HPP_

#include <optional>

#include "tunnel_frontier_explorer/frontier_cluster.hpp"
#include "tunnel_frontier_explorer/frontier_detector.hpp"

namespace tunnel_frontier_explorer
{

/// Configuration for the forced escape probe generator.
struct EscapeProbeConfig
{
  double probe_distance_m = 1.25;
  double probe_distance_step_m = 0.5;
  double probe_exit_margin_m = 0.25;
  int max_attempts = 8;
  double angle_span_deg = 90.0;
  double min_wall_distance_m = 0.35;
};

/// Generates forced escape probe points when all candidates are inside
/// the oscillillation zone. Pure C++ class with no ROS dependencies.
class EscapeProbeGenerator
{
public:
  EscapeProbeGenerator() = default;
  explicit EscapeProbeGenerator(EscapeProbeConfig config);

  /// Generate a forced escape probe point.
  ///
  /// Searches outward from the oscillillation center along the preferred
  /// direction (and angle offsets) until finding a free-space point that:
  /// - Is outside the oscillillation radius + margin
  /// - Is free in the occupancy grid
  /// - Has minimum wall clearance
  ///
  /// @return valid probe point, or nullopt if no safe point found
  std::optional<Point2D> generateEscapeProbe(
    const Point2D & robot,
    const Point2D & oscillation_center,
    const Point2D & preferred_direction,
    const GridMap & map,
    double suppression_radius) const;

private:
  EscapeProbeConfig config_;

  /// Check if a point is free in the occupancy grid.
  bool isFree(const Point2D & point, const GridMap & map) const;

  /// Check minimum wall distance (count occupied cells in small radius).
  bool hasWallClearance(const Point2D & point, const GridMap & map) const;
};

}  // namespace tunnel_frontier_explorer

#endif  // TUNNEL_FRONTIER_EXPLORER__ESCAPE_PROBE_GENERATOR_HPP_
