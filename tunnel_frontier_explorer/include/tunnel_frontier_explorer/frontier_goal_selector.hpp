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

#ifndef TUNNEL_FRONTIER_EXPLORER__FRONTIER_GOAL_SELECTOR_HPP_
#define TUNNEL_FRONTIER_EXPLORER__FRONTIER_GOAL_SELECTOR_HPP_

#include <optional>
#include <vector>

#include "tunnel_frontier_explorer/frontier_cluster.hpp"
#include "tunnel_frontier_explorer/frontier_detector.hpp"

namespace tunnel_frontier_explorer
{

/// Configuration for FrontierGoalSelector.
struct FrontierGoalSelectorConfig
{
  /// Minimum distance (meters) from robot to a goal for it to be considered
  /// navigable.  Should be significantly larger than Nav2's xy_goal_tolerance
  /// (typically 0.25 m) to avoid instant "success" at the start cell.
  double min_goal_distance_meters = 0.50;
};

/// Result of a goal selection pass.
struct GoalSelectionResult
{
  /// The selected cluster with (potentially updated) representative_world.
  std::optional<FrontierCluster> selected;

  /// Representative positions of clusters that were skipped because ALL of
  /// their frontier cells were too close to the robot.
  std::vector<Point2D> too_close_goals;
};

/// Result of selectAll() — all accepted candidates, not just the best one.
struct AllCandidatesResult
{
  /// All clusters that passed the min-goal-distance filter.
  std::vector<FrontierCluster> candidates;

  /// Representative positions of clusters that were rejected because ALL of
  /// their frontier cells were too close to the robot.
  std::vector<Point2D> too_close_goals;
};

/// Pure-algorithm goal selector that filters frontier clusters by minimum
/// distance and finds alternative representative cells when the original is
/// too close to the robot.
///
/// Zero ROS dependencies.  Operates on FrontierCluster and GridMap.
class FrontierGoalSelector
{
public:
  explicit FrontierGoalSelector(FrontierGoalSelectorConfig config);

  /// Select the best goal from @p clusters.
  ///
  /// For each cluster:
  ///   1. If representative_world is >= min_goal_distance_meters from robot,
  ///      keep as-is.
  ///   2. If too close, search the cluster's cells for a suitable alternative
  ///      cell that is >= min_goal_distance_meters away.
  ///   3. If no alternative exists, reject the cluster.
  ///
  /// Among accepted clusters, the one with the smallest goal_distance_to_robot
  /// is returned.
  ///
  /// @param clusters       Non-blacklisted clusters (may be modified in-place).
  /// @param map            Occupancy grid map (for cell-to-world conversion).
  /// @param robot_position Robot position in world coordinates.
  /// @return Selection result.
  GoalSelectionResult select(
    std::vector<FrontierCluster> & clusters,
    const GridMap & map,
    const Point2D & robot_position) const;

  /// Like select() but returns ALL accepted candidates instead of only the
  /// nearest.  Clusters are modified in-place (representative may be updated).
  ///
  /// Candidates are returned with goal_distance_to_robot populated, sorted
  /// by goal distance ascending (nearest first).
  AllCandidatesResult selectAll(
    std::vector<FrontierCluster> & clusters,
    const GridMap & map,
    const Point2D & robot_position) const;

  /// Convert grid (row, col) to world coordinates (mirrors FrontierDetector).
  static Point2D gridToWorld(const GridMap & map, int row, int col);

private:
  FrontierGoalSelectorConfig config_;
};

}  // namespace tunnel_frontier_explorer

#endif  // TUNNEL_FRONTIER_EXPLORER__FRONTIER_GOAL_SELECTOR_HPP_
