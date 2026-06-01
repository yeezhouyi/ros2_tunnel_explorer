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

// ── collectAccepted (shared helper) ─────────────────────────────────────────

namespace {

/// Per-cluster filtering: check distance, find alternative if needed.
/// Modifies cluster representative in-place for accepted clusters.
void collectAccepted(
  std::vector<FrontierCluster> & clusters,
  const GridMap & map,
  const Point2D & robot_position,
  double min_goal_distance_meters,
  std::vector<FrontierCluster *> & accepted,
  std::vector<Point2D> & too_close)
{
  const double min_dist_sq =
    min_goal_distance_meters * min_goal_distance_meters;

  for (auto & cluster : clusters) {
    const double dx = cluster.representative_world.x - robot_position.x;
    const double dy = cluster.representative_world.y - robot_position.y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    if (dist >= min_goal_distance_meters) {
      cluster.goal_distance_to_robot = dist;
      accepted.push_back(&cluster);
      continue;
    }

    // Representative too close — search for alternative cell.
    double best_sq_dist = std::numeric_limits<double>::max();
    GridCell best_cell;
    Point2D best_world;
    bool found = false;

    for (const auto & cell : cluster.cells) {
      const Point2D world = FrontierGoalSelector::gridToWorld(map, cell.row, cell.col);
      const double cd_x = world.x - robot_position.x;
      const double cd_y = world.y - robot_position.y;
      const double cell_dist_sq = cd_x * cd_x + cd_y * cd_y;

      if (cell_dist_sq >= min_dist_sq) {
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
      accepted.push_back(&cluster);
    } else {
      too_close.push_back(cluster.representative_world);
    }
  }
}

}  // namespace

// ── select ────────────────────────────────────────────────────────────────

GoalSelectionResult FrontierGoalSelector::select(
  std::vector<FrontierCluster> & clusters,
  const GridMap & map,
  const Point2D & robot_position) const
{
  GoalSelectionResult result;
  std::vector<FrontierCluster *> candidates;

  collectAccepted(
    clusters, map, robot_position, config_.min_goal_distance_meters,
    candidates, result.too_close_goals);

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

// ── selectAll ─────────────────────────────────────────────────────────────

AllCandidatesResult FrontierGoalSelector::selectAll(
  std::vector<FrontierCluster> & clusters,
  const GridMap & map,
  const Point2D & robot_position) const
{
  AllCandidatesResult result;
  std::vector<FrontierCluster *> accepted_ptrs;

  collectAccepted(
    clusters, map, robot_position, config_.min_goal_distance_meters,
    accepted_ptrs, result.too_close_goals);

  // Copy all accepted clusters, sorted by distance.
  std::sort(accepted_ptrs.begin(), accepted_ptrs.end(),
    [](const FrontierCluster * a, const FrontierCluster * b) {
      return a->goal_distance_to_robot < b->goal_distance_to_robot;
    });

  result.candidates.reserve(accepted_ptrs.size());
  for (const auto * ptr : accepted_ptrs) {
    result.candidates.push_back(*ptr);
  }

  return result;
}

}  // namespace tunnel_frontier_explorer
