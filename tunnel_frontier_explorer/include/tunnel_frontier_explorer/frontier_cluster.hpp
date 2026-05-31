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

#ifndef TUNNEL_FRONTIER_EXPLORER__FRONTIER_CLUSTER_HPP_
#define TUNNEL_FRONTIER_EXPLORER__FRONTIER_CLUSTER_HPP_

#include <cstdint>
#include <vector>

namespace tunnel_frontier_explorer
{

/// A discrete cell in the occupancy grid.
struct GridCell
{
  int row = 0;
  int col = 0;
};

/// A 2-D point in continuous coordinates (grid or world frame).
struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

/// A connected component of frontier cells.
struct FrontierCluster
{
  /// All cells belonging to this cluster (grid coordinates).
  std::vector<GridCell> cells;

  /// Geometric centroid in grid-cell coordinates (average of cell indices).
  Point2D centroid_grid;

  /// Geometric centroid in world coordinates (map frame).
  Point2D centroid_world;

  /// The free cell closest to the centroid — safe to use as navigation goal.
  GridCell representative_cell;

  /// World coordinates of the representative cell.
  Point2D representative_world;

  /// Euclidean distance from robot to centroid_world (populated by detector).
  double centroid_distance_to_robot = 0.0;

  /// Euclidean distance from robot to representative_world (populated by
  /// detector, may be updated by FrontierGoalSelector).
  double goal_distance_to_robot = 0.0;

  /// Number of cells in this cluster.
  std::size_t size() const {return cells.size();}
};

}  // namespace tunnel_frontier_explorer

#endif  // TUNNEL_FRONTIER_EXPLORER__FRONTIER_CLUSTER_HPP_
