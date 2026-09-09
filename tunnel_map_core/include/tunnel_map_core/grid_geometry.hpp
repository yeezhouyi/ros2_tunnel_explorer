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

#ifndef TUNNEL_MAP_CORE__GRID_GEOMETRY_HPP_
#define TUNNEL_MAP_CORE__GRID_GEOMETRY_HPP_

#include <cstddef>
#include <string>

#include "tunnel_map_core/grid_map.hpp"

namespace tunnel_map_core
{

/// World <-> grid coordinate bookkeeping for a frozen OccupancyGrid.
///
/// A GridGeometry instance is always created from an already validated
/// GridMap.  All conversions are exact double arithmetic; callers that need a
/// discrete cell index must round/clamp explicitly with worldToGridCell().
///
/// Convention (yaw == 0):
///   - cell (row, col) centre world x = origin_x + (col + 0.5) * resolution
///   - cell (row, col) centre world y = origin_y + (row + 0.5) * resolution
class GridGeometry
{
public:
  /// Default-constructs an invalid geometry (width/height == 0).  Useful as a
  /// member placeholder; real instances are created from a validated GridMap.
  GridGeometry() = default;

  /// @throws std::invalid_argument if @p map is not valid().
  explicit GridGeometry(const GridMap & map);

  /// Cell-centre world coordinates.
  Point2D gridToWorld(int row, int col) const;

  /// Cell-centre world coordinates from continuous grid coordinates.
  Point2D gridToWorld(const GridPoint & p) const;

  /// Continuous grid coordinates of an arbitrary world point.
  GridPoint worldToGrid(const Point2D & p) const;

  /// Rounded cell for a world point; returns false when out of bounds.
  bool worldToGridCell(const Point2D & p, GridCell & cell) const;

  /// Rounded cell; returns false when out of bounds.
  bool gridToCell(const GridPoint & p, GridCell & cell) const;

  /// True when (row, col) lies inside the map.
  bool inBounds(int row, int col) const;

  /// Cell index for (row, col); behaviour undefined when out of bounds.
  std::size_t index(int row, int col) const;

  /// Width of a cell in world metres (= resolution).
  double cellSize() const {return resolution_;}

  std::size_t width() const {return width_;}
  std::size_t height() const {return height_;}

  /// Canonical, deterministic summary of map *content and geometry*.
  ///
  /// Includes width/height/resolution/origin (x, y, yaw) and every data cell.
  /// Used for task_input_id / map-change detection (R12, R23 of the cleaning
  /// track contract).  Not cryptographically strong; collisions are not a
  /// practical concern for gating identical map re-publishes.
  std::string contentDigest(const GridMap & map) const;

private:
  std::size_t width_ = 0;
  std::size_t height_ = 0;
  double resolution_ = 0.0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double cos_yaw_ = 1.0;
  double sin_yaw_ = 0.0;
};

}  // namespace tunnel_map_core

#endif  // TUNNEL_MAP_CORE__GRID_GEOMETRY_HPP_
