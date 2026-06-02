// Copyright 2026 zhouyi
// License: Apache-2.0

#include "tunnel_centerline_extractor/distance_field.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

namespace tunnel_centerline_extractor
{

namespace
{

// 8-neighbor offsets: N, S, W, E, NW, NE, SW, SE
constexpr int dx[8] = {0, 0, -1, 1, -1, 1, -1, 1};
constexpr int dy[8] = {-1, 1, 0, 0, -1, -1, 1, 1};

}  // namespace

Grid2D brushfire_distance_field(const Grid2D & free_mask)
{
  const int w = free_mask.width;
  const int h = free_mask.height;
  const double res = free_mask.resolution;

  Grid2D dist;
  dist.width = w;
  dist.height = h;
  dist.resolution = res;
  dist.origin_x = free_mask.origin_x;
  dist.origin_y = free_mask.origin_y;
  dist.data.resize(w * h, std::numeric_limits<int>::max());

  // Integer costs: horizontal = res * 1000, diagonal = res * 1414
  const int cost_hv = static_cast<int>(res * 1000.0);
  const int cost_diag = static_cast<int>(res * 1414.0);

  // Seed: all blocked cells (mask == 0) → distance 0
  std::deque<std::pair<int, int>> queue;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (free_mask.at(x, y) == 0) {
        dist.at(x, y) = 0;
        queue.push_back({x, y});
      }
    }
  }

  // Multi-source BFS / Dijkstra expansion
  while (!queue.empty()) {
    auto [cx, cy] = queue.front();
    queue.pop_front();
    int cur_d = dist.at(cx, cy);

    for (int i = 0; i < 8; ++i) {
      int nx = cx + dx[i];
      int ny = cy + dy[i];
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;

      int step_cost = (i < 4) ? cost_hv : cost_diag;
      int nd = cur_d + step_cost;

      if (nd < dist.at(nx, ny)) {
        dist.at(nx, ny) = nd;
        // Push cheap expansions to front, expensive to back
        // (approximate 0-1 BFS for better performance)
        if (i < 4)
          queue.push_front({nx, ny});
        else
          queue.push_back({nx, ny});
      }
    }
  }

  return dist;
}

}  // namespace tunnel_centerline_extractor
