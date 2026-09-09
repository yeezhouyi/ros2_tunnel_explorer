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

#ifndef TUNNEL_MAP_CORE__GRID_MAP_HPP_
#define TUNNEL_MAP_CORE__GRID_MAP_HPP_

#include <cstdint>
#include <cstddef>
#include <vector>

namespace tunnel_map_core
{

/// A discrete cell in the occupancy grid.
///
/// Row 0 is the bottom row of the map (smallest world y when yaw == 0),
/// matching the nav_msgs/OccupancyGrid row-major layout used across this
/// repository (see tunnel_frontier_explorer::GridCell).
struct GridCell
{
  int row = 0;
  int col = 0;
};

/// A 2-D point in continuous grid-cell units.
struct GridPoint
{
  double col = 0.0;
  double row = 0.0;
};

/// A 2-D point in continuous world coordinates.
struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

/// Occupancy values used throughout the coverage track.
/// These match nav_msgs/OccupancyGrid semantics.
enum OccupancyValue : std::int8_t
{
  OCC_UKNOWN = -1,
  OCC_FREE = 0,
  OCC_OCCUPIED = 100
};

/// Lightweight, ROS-free wrapper around raw occupancy grid data.
///
/// Layout: data is row-major, `data[row * width + col]`.  Each cell stores an
/// occupancy value in [-1, 100].  The world pose of a cell centre is
///
///   world = origin + R(yaw) * ((col + 0.5) * resolution, (row + 0.5) * resolution)
///
/// `origin_yaw` is included for generality (map YAML files may carry a yaw);
/// SLAM Toolbox output normally uses yaw == 0.
struct GridMap
{
  std::size_t width = 0;
  std::size_t height = 0;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  std::vector<std::int8_t> data;

  /// Returns true when the map is well-formed for grid algorithms.
  bool valid() const
  {
    return width > 0 && height > 0 &&
           data.size() == width * height &&
           resolution > 0.0;
  }
};

}  // namespace tunnel_map_core

#endif  // TUNNEL_MAP_CORE__GRID_MAP_HPP_
