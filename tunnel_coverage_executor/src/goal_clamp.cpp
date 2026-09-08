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

#include "tunnel_coverage_executor/goal_clamp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tunnel_coverage_executor
{

GoalClampResult clampGoalEndpoint(
  const std::vector<std::uint8_t> & mask,
  const tunnel_map_core::GridGeometry & geometry,
  double x,
  double y,
  double inset_m)
{
  const auto w = static_cast<int>(geometry.width());
  const auto h = static_cast<int>(geometry.height());
  GoalClampResult out{x, y, false};
  if (w <= 0 || h <= 0 ||
    mask.size() != static_cast<std::size_t>(w) * static_cast<std::size_t>(h))
  {
    return out;  // nothing to clamp against -- leave the goal untouched
  }

  tunnel_map_core::GridCell cell;
  const bool valid_now = geometry.worldToGridCell({x, y}, cell) &&
    mask[static_cast<std::size_t>(cell.row) * static_cast<std::size_t>(w) +
      static_cast<std::size_t>(cell.col)] != 0;
  if (valid_now) {
    return out;  // already on a valid cell -- guard never touches it
  }

  // Bounding box of the valid region, in world coordinates.
  int min_row = -1, max_row = -1, min_col = -1, max_col = -1;
  for (int row = 0; row < h; ++row) {
    for (int col = 0; col < w; ++col) {
      if (mask[static_cast<std::size_t>(row) * static_cast<std::size_t>(w) +
        static_cast<std::size_t>(col)] == 0)
      {
        continue;
      }
      min_row = min_row < 0 ? row : std::min(min_row, row);
      max_row = std::max(max_row, row);
      min_col = min_col < 0 ? col : std::min(min_col, col);
      max_col = std::max(max_col, col);
    }
  }
  if (min_row < 0) {
    return out;  // empty mask -- nothing to clamp to
  }

  const double cell_m = geometry.cellSize();
  const tunnel_map_core::Point2D lo_corner =
    geometry.gridToWorld(min_row, min_col);
  const tunnel_map_core::Point2D hi_corner =
    geometry.gridToWorld(max_row, max_col);
  const double x_lo = lo_corner.x - 0.5 * cell_m;
  const double x_hi = hi_corner.x + 0.5 * cell_m;
  const double y_lo = lo_corner.y - 0.5 * cell_m;
  const double y_hi = hi_corner.y + 0.5 * cell_m;

  // Inset window; on an axis thinner than 2 * inset_m fall back to the
  // axis mid-line so the clamp can never push the goal outside the region.
  const double cx_lo = (x_hi - x_lo >= 2.0 * inset_m) ? x_lo + inset_m : x_lo;
  const double cx_hi = (x_hi - x_lo >= 2.0 * inset_m) ? x_hi - inset_m : x_hi;
  const double cy_lo = (y_hi - y_lo >= 2.0 * inset_m) ? y_lo + inset_m : y_lo;
  const double cy_hi = (y_hi - y_lo >= 2.0 * inset_m) ? y_hi - inset_m : y_hi;

  const double cx = std::clamp(x, cx_lo, cx_hi);
  const double cy = std::clamp(y, cy_lo, cy_hi);

  // Snap to the nearest valid cell centre (the mask may be concave, so the
  // clamped point can still sit on a masked-out cell).  Centres inside the
  // inset window are preferred so the inset guarantee survives the snap;
  // when the window holds no valid centre the snap falls back to the
  // nearest valid centre overall.
  double best_d = std::numeric_limits<double>::max();
  double window_d = std::numeric_limits<double>::max();
  tunnel_map_core::Point2D best{cx, cy};
  tunnel_map_core::Point2D window_best{cx, cy};
  bool found = false;
  bool in_window = false;
  for (int row = min_row; row <= max_row; ++row) {
    for (int col = min_col; col <= max_col; ++col) {
      if (mask[static_cast<std::size_t>(row) * static_cast<std::size_t>(w) +
        static_cast<std::size_t>(col)] == 0)
      {
        continue;
      }
      const tunnel_map_core::Point2D c = geometry.gridToWorld(row, col);
      const double d = std::hypot(c.x - cx, c.y - cy);
      if (d < best_d) {
        best_d = d;
        best = c;
        found = true;
      }
      const bool centre_in_window =
        c.x >= cx_lo && c.x <= cx_hi && c.y >= cy_lo && c.y <= cy_hi;
      if (centre_in_window && d < window_d) {
        window_d = d;
        window_best = c;
        in_window = true;
      }
    }
  }
  if (!found) {
    return out;  // no valid cell at all -- leave the goal untouched
  }
  if (in_window) {
    best = window_best;  // keep the inset guarantee when possible
  }

  out.x = best.x;
  out.y = best.y;
  out.clamped = true;
  return out;
}

}  // namespace tunnel_coverage_executor
