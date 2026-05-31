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

#ifndef TUNNEL_FRONTIER_EXPLORER__FRONTIER_DETECTOR_HPP_
#define TUNNEL_FRONTIER_EXPLORER__FRONTIER_DETECTOR_HPP_

#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include "tunnel_frontier_explorer/frontier_cluster.hpp"

namespace tunnel_frontier_explorer
{

/// Lightweight wrapper around raw occupancy grid data for the detector.
struct GridMap
{
  std::size_t width = 0;
  std::size_t height = 0;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<std::int8_t> data;

  /// Returns true when the map is well-formed for frontier detection.
  bool valid() const
  {
    return width > 0 && height > 0 &&
           data.size() == width * height &&
           resolution > 0.0;
  }
};

/// Configuration for FrontierDetector behaviour.
struct FrontierDetectorConfig
{
  /// Minimum number of cells for a cluster to be kept.
  std::size_t min_cluster_size = 5;

  /// Occupancy value threshold for "free" (<= this value is free).
  int free_threshold = 0;

  /// Neighbourhood for detecting frontier cells: 4 or 8.
  int frontier_neighbor_connectivity = 4;

  /// Neighbourhood for clustering: 4 or 8.
  int cluster_connectivity = 8;
};

/// Pure-algorithm frontier detector with BFS clustering.
///
/// Zero ROS dependencies — operates on raw grid data.  Designed to be
/// unit-testable without any ROS infrastructure.
class FrontierDetector
{
public:
  /// @param config  Detection parameters.
  /// @throws std::invalid_argument if connectivity is not 4 or 8.
  explicit FrontierDetector(FrontierDetectorConfig config);

  /// Run the full detection pipeline on @p map.
  ///
  /// @param map             Occupancy grid wrapper (must be valid()).
  /// @param robot_position  Robot location in world coords, used for distance.
  /// @return List of clusters, each with centroid, representative cell, etc.
  /// @throws std::invalid_argument if map is not valid().
  std::vector<FrontierCluster> detect(
    const GridMap & map,
    const Point2D & robot_position) const;

private:
  FrontierDetectorConfig config_;

  /// True when cell (row, col) is a frontier cell:
  ///   - free (value <= free_threshold)
  ///   - at least one unknown neighbour (value == -1)
  bool isFrontierCell(
    const GridMap & map,
    int row,
    int col) const;

  /// BFS starting from (start_row, start_col) to collect a connected cluster.
  void bfsCluster(
    const GridMap & map,
    int start_row,
    int start_col,
    std::vector<bool> & visited,
    FrontierCluster & cluster) const;

  /// Convert grid (row, col) → world (x, y) using map origin + resolution.
  Point2D gridToWorld(const GridMap & map, int row, int col) const;

  /// Find the free cell in the cluster closest to the centroid.
  void computeRepresentative(
    const GridMap & map,
    FrontierCluster & cluster) const;

  /// Neighbour offsets for the configured connectivities.
  static std::vector<std::pair<int, int>> makeNeighbourhood(int connectivity);
};

}  // namespace tunnel_frontier_explorer

#endif  // TUNNEL_FRONTIER_EXPLORER__FRONTIER_DETECTOR_HPP_
