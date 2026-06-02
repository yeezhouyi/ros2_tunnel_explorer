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

#ifndef TUNNEL_CENTERLINE_EXTRACTOR__GRID_TYPES_HPP_
#define TUNNEL_CENTERLINE_EXTRACTOR__GRID_TYPES_HPP_

#include <cstdint>
#include <vector>

namespace tunnel_centerline_extractor
{

/// 2D grid of int values (distance, mask, occupancy).
struct Grid2D
{
  int width = 0;
  int height = 0;
  double resolution = 0.05;   // m / cell
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<int> data;

  bool valid() const { return width > 0 && height > 0 && !data.empty(); }
  int & at(int x, int y) { return data[y * width + x]; }
  int at(int x, int y) const { return data[y * width + x]; }

  void resize(int w, int h) { width = w; height = h; data.resize(w * h, 0); }
};

/// A lightweight 2D point in world or grid coordinates.
struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

/// Convert an occupancy-grid value to a blocked/free classification.
/// Returns true if the cell is blocked (occupied or unknown).
inline bool is_blocked(int8_t occ, int /*free_thresh*/, int occ_thresh, bool unknown_is_blocked)
{
  if (occ < 0) return unknown_is_blocked;
  return occ >= occ_thresh;
}

/// Build a free-mask (1 = free, 0 = blocked) from an occupancy-grid raw data buffer.
inline Grid2D build_free_mask(
  int width, int height, double resolution,
  double origin_x, double origin_y,
  const std::vector<int8_t> & occ_data,
  int free_thresh, int occ_thresh, bool unknown_is_blocked)
{
  Grid2D mask;
  mask.width = width;
  mask.height = height;
  mask.resolution = resolution;
  mask.origin_x = origin_x;
  mask.origin_y = origin_y;
  mask.data.resize(width * height, 0);
  for (int i = 0; i < width * height; ++i) {
    mask.data[i] = is_blocked(occ_data[i], free_thresh, occ_thresh, unknown_is_blocked) ? 0 : 1;
  }
  return mask;
}

}  // namespace tunnel_centerline_extractor

#endif  // TUNNEL_CENTERLINE_EXTRACTOR__GRID_TYPES_HPP_
