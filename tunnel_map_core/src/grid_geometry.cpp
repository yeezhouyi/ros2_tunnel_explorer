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

#include "tunnel_map_core/grid_geometry.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "tunnel_map_core/map_digest.hpp"

namespace tunnel_map_core
{

GridGeometry::GridGeometry(const GridMap & map)
: width_(map.width),
  height_(map.height),
  resolution_(map.resolution),
  origin_x_(map.origin_x),
  origin_y_(map.origin_y),
  cos_yaw_(std::cos(map.origin_yaw)),
  sin_yaw_(std::sin(map.origin_yaw))
{
  if (!map.valid()) {
    throw std::invalid_argument(
      "GridGeometry — invalid map "
      "(width=" + std::to_string(map.width) +
      ", height=" + std::to_string(map.height) +
      ", data.size=" + std::to_string(map.data.size()) +
      ", resolution=" + std::to_string(map.resolution) + ")");
  }
}

Point2D GridGeometry::gridToWorld(int row, int col) const
{
  return gridToWorld(GridPoint{
      static_cast<double>(col) + 0.5,
      static_cast<double>(row) + 0.5});
}

Point2D GridGeometry::gridToWorld(const GridPoint & p) const
{
  // Grid axis x in world frame is (cos_yaw_, sin_yaw_); grid axis y is
  // (-sin_yaw_, cos_yaw_).
  const double gx = p.col * resolution_;
  const double gy = p.row * resolution_;
  Point2D out;
  out.x = origin_x_ + cos_yaw_ * gx - sin_yaw_ * gy;
  out.y = origin_y_ + sin_yaw_ * gx + cos_yaw_ * gy;
  return out;
}

GridPoint GridGeometry::worldToGrid(const Point2D & p) const
{
  const double dx = p.x - origin_x_;
  const double dy = p.y - origin_y_;
  // Project onto the (possibly rotated) grid axes.
  const double gu = cos_yaw_ * dx + sin_yaw_ * dy;
  const double gv = -sin_yaw_ * dx + cos_yaw_ * dy;
  GridPoint out;
  out.col = gu / resolution_;
  out.row = gv / resolution_;
  return out;
}

bool GridGeometry::worldToGridCell(const Point2D & p, GridCell & cell) const
{
  return gridToCell(worldToGrid(p), cell);
}

bool GridGeometry::gridToCell(const GridPoint & p, GridCell & cell) const
{
  const auto col = static_cast<int>(std::floor(p.col));
  const auto row = static_cast<int>(std::floor(p.row));
  if (col < 0 || static_cast<std::size_t>(col) >= width_ ||
    row < 0 || static_cast<std::size_t>(row) >= height_)
  {
    return false;
  }
  cell.row = row;
  cell.col = col;
  return true;
}

bool GridGeometry::inBounds(int row, int col) const
{
  return row >= 0 && static_cast<std::size_t>(row) < height_ &&
         col >= 0 && static_cast<std::size_t>(col) < width_;
}

std::size_t GridGeometry::index(int row, int col) const
{
  return static_cast<std::size_t>(row) * width_ + static_cast<std::size_t>(col);
}

std::string GridGeometry::contentDigest(const GridMap & map) const
{
  return mapContentDigest(map);
}

}  // namespace tunnel_map_core
