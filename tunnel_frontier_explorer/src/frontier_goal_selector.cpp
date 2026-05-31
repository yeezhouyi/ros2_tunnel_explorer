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

#include "tunnel_frontier_explorer/frontier_goal_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace tunnel_frontier_explorer
{

FrontierGoalSelector::FrontierGoalSelector(
  FrontierGoalSelectorConfig config)
: config_(config)
{
  if (config_.min_goal_distance_meters <= 0.0) {
    throw std::invalid_argument(
      "FrontierGoalSelector: min_goal_distance_meters must be > 0, got " +
      std::to_string(config_.min_goal_distance_meters));
  }
}

// ── gridToWorld ──────────────────────────────────────────────────────────

Point2D FrontierGoalSelector::gridToWorld(
  const GridMap & map, int row, int col)
{
  Point2D p;
  p.x = map.origin_x + (static_cast<double>(col) + 0.5) * map.resolution;
  p.y = map.origin_y + (static_cast<double>(row) + 0.5) * map.resolution;
  return p;
}

// ── select ────────────────────────────────────────────────────────────────

GoalSelectionResult FrontierGoalSelector::select(
  std::vector<FrontierCluster> & clusters,
  const GridMap & map,
  const Point2D & robot_position) const
{
  GoalSelectionResult result;
  std::vector<FrontierCluster *> candidates;

  for (auto & cluster : clusters) {
    // Compute distance from robot to current representative.
    const double dx = cluster.representative_world.x - robot_position.x;
    const double dy = cluster.representative_world.y - robot_position.y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    if (dist >= config_.min_goal_distance_meters) {
      // Representative is already far enough — use as-is.
      cluster.goal_distance_to_robot = dist;
      candidates.push_back(&cluster);
      continue;
    }

    // Representative is too close — search for an alternative cell within
    // this cluster that is >= min_goal_distance_meters away.
    double best_sq_dist = std::numeric_limits<double>::max();
    GridCell best_cell;
    Point2D best_world;
    bool found = false;

    for (const auto & cell : cluster.cells) {
      const Point2D world = gridToWorld(map, cell.row, cell.col);
      const double cd_x = world.x - robot_position.x;
      const double cd_y = world.y - robot_position.y;
      const double cell_dist_sq = cd_x * cd_x + cd_y * cd_y;

      if (cell_dist_sq >= config_.min_goal_distance_meters *
        config_.min_goal_distance_meters)
      {
        // This cell is far enough — prefer the one closest to centroid.
        const double cdx = world.x - cluster.centroid_world.x;
        const double cdy = world.y - cluster.centroid_world.y;
        const double d2 = cdx * cdx + cdy * cdy;
        if (d2 < best_sq_dist) {
          best_sq_dist = d2;
          best_cell = cell;
          best_world = world;
          found = true;
        }
      }
    }

    if (found) {
      cluster.representative_cell = best_cell;
      cluster.representative_world = best_world;
      const double gdx = best_world.x - robot_position.x;
      const double gdy = best_world.y - robot_position.y;
      cluster.goal_distance_to_robot = std::sqrt(gdx * gdx + gdy * gdy);
      candidates.push_back(&cluster);
    } else {
      // Entire cluster is too close — reject.
      result.too_close_goals.push_back(cluster.representative_world);
    }
  }

  if (candidates.empty()) {
    result.selected = std::nullopt;
    return result;
  }

  // Sort candidates by actual goal distance and pick nearest.
  std::sort(candidates.begin(), candidates.end(),
    [](const FrontierCluster * a, const FrontierCluster * b) {
      return a->goal_distance_to_robot < b->goal_distance_to_robot;
    });

  result.selected = *candidates.front();
  return result;
}

}  // namespace tunnel_frontier_explorer
