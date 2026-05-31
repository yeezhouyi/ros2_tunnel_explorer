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

#include "tunnel_frontier_explorer/frontier_detector.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace tunnel_frontier_explorer
{

// ── helpers ──────────────────────────────────────────────────────────────

std::vector<std::pair<int, int>> FrontierDetector::makeNeighbourhood(
  int connectivity)
{
  if (connectivity == 4) {
    return {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  }
  if (connectivity == 8) {
    return {{-1, -1}, {-1, 0}, {-1, 1},
      {0, -1}, {0, 1},
      {1, -1}, {1, 0}, {1, 1}};
  }
  throw std::invalid_argument(
    "connectivity must be 4 or 8, got " + std::to_string(connectivity));
}

// ── ctor ─────────────────────────────────────────────────────────────────

FrontierDetector::FrontierDetector(FrontierDetectorConfig config)
: config_(config)
{
  // Validate up front (makeNeighbourhood will throw for invalid values).
  makeNeighbourhood(config_.frontier_neighbor_connectivity);
  makeNeighbourhood(config_.cluster_connectivity);
}

// ── gridToWorld ──────────────────────────────────────────────────────────

Point2D FrontierDetector::gridToWorld(
  const GridMap & map, int row, int col) const
{
  Point2D p;
  p.x = map.origin_x + (static_cast<double>(col) + 0.5) * map.resolution;
  p.y = map.origin_y + (static_cast<double>(row) + 0.5) * map.resolution;
  return p;
}

// ── isFrontierCell ───────────────────────────────────────────────────────

bool FrontierDetector::isFrontierCell(
  const GridMap & map, int row, int col) const
{
  // Must be within bounds.
  if (row < 0 || static_cast<std::size_t>(row) >= map.height ||
    col < 0 || static_cast<std::size_t>(col) >= map.width)
  {
    return false;
  }

  // Must be free: value must be exactly 0 (or between 0 and free_threshold).
  const auto value = map.data[static_cast<std::size_t>(row) * map.width +
      static_cast<std::size_t>(col)];
  if (value < 0 || value > config_.free_threshold) {
    return false;
  }

  // Must have at least one unknown neighbour (within bounds).
  const auto offsets = makeNeighbourhood(config_.frontier_neighbor_connectivity);
  for (const auto & d : offsets) {
    const int nr = row + d.first;
    const int nc = col + d.second;
    if (nr < 0 || static_cast<std::size_t>(nr) >= map.height ||
      nc < 0 || static_cast<std::size_t>(nc) >= map.width)
    {
      continue;  // Out-of-bounds neighbours are NOT treated as unknown.
    }
    if (map.data[static_cast<std::size_t>(nr) * map.width +
      static_cast<std::size_t>(nc)] == -1)
    {
      return true;
    }
  }
  return false;
}

// ── bfsCluster ───────────────────────────────────────────────────────────

void FrontierDetector::bfsCluster(
  const GridMap & map,
  int start_row,
  int start_col,
  std::vector<bool> & visited,
  FrontierCluster & cluster) const
{
  const auto offsets = makeNeighbourhood(config_.cluster_connectivity);

  std::deque<std::pair<int, int>> queue;
  queue.emplace_back(start_row, start_col);
  visited[static_cast<std::size_t>(start_row) * map.width +
    static_cast<std::size_t>(start_col)] = true;
  cluster.cells.push_back({start_row, start_col});

  while (!queue.empty()) {
    const auto [cr, cc] = queue.front();
    queue.pop_front();

    for (const auto & d : offsets) {
      const int nr = cr + d.first;
      const int nc = cc + d.second;
      if (nr < 0 || static_cast<std::size_t>(nr) >= map.height ||
        nc < 0 || static_cast<std::size_t>(nc) >= map.width)
      {
        continue;
      }
      const auto idx =
        static_cast<std::size_t>(nr) * map.width + static_cast<std::size_t>(nc);
      if (!visited[idx] && isFrontierCell(map, nr, nc)) {
        visited[idx] = true;
        cluster.cells.push_back({nr, nc});
        queue.emplace_back(nr, nc);
      }
    }
  }
}

// ── computeRepresentative ────────────────────────────────────────────────

void FrontierDetector::computeRepresentative(
  const GridMap & map, FrontierCluster & cluster) const
{
  if (cluster.cells.empty()) {
    return;
  }

  double best_dist = std::numeric_limits<double>::max();
  int best_idx = 0;

  for (std::size_t i = 0; i < cluster.cells.size(); ++i) {
    const auto & cell = cluster.cells[i];
    const double dx = static_cast<double>(cell.col) - cluster.centroid_grid.x;
    const double dy = static_cast<double>(cell.row) - cluster.centroid_grid.y;
    const double d = dx * dx + dy * dy;  // squared — compare is sufficient
    if (d < best_dist) {
      best_dist = d;
      best_idx = static_cast<int>(i);
    }
  }

  const auto & best = cluster.cells[best_idx];
  cluster.representative_cell = best;
  cluster.representative_world = gridToWorld(map, best.row, best.col);
}

// ── detect ───────────────────────────────────────────────────────────────

std::vector<FrontierCluster> FrontierDetector::detect(
  const GridMap & map,
  const Point2D & robot_position) const
{
  if (!map.valid()) {
    throw std::invalid_argument(
      "FrontierDetector::detect — invalid map "
      "(width=" + std::to_string(map.width) +
      ", height=" + std::to_string(map.height) +
      ", data.size=" + std::to_string(map.data.size()) +
      ", resolution=" + std::to_string(map.resolution) + ")");
  }

  const auto total_cells = map.width * map.height;
  std::vector<bool> visited(total_cells, false);
  std::vector<FrontierCluster> clusters;

  // Detect frontier cells and cluster them via BFS.
  for (std::size_t row = 0; row < map.height; ++row) {
    for (std::size_t col = 0; col < map.width; ++col) {
      const auto idx = row * map.width + col;
      if (!visited[idx] && isFrontierCell(map, static_cast<int>(row),
                                           static_cast<int>(col)))
      {
        FrontierCluster cluster;
        bfsCluster(map, static_cast<int>(row), static_cast<int>(col),
                   visited, cluster);

        if (cluster.size() >= config_.min_cluster_size) {
          // Compute centroid in grid coords.
          double sum_row = 0.0, sum_col = 0.0;
          for (const auto & cell : cluster.cells) {
            sum_row += static_cast<double>(cell.row);
            sum_col += static_cast<double>(cell.col);
          }
          cluster.centroid_grid.x = sum_col / static_cast<double>(cluster.size());
          cluster.centroid_grid.y = sum_row / static_cast<double>(cluster.size());

          // Convert centroid to world coords.
          // Centroid in grid coords is not necessarily integer, so convert
          // as a continuous point rather than mapping an integer cell.
          cluster.centroid_world.x =
            map.origin_x + (cluster.centroid_grid.x + 0.5) * map.resolution;
          cluster.centroid_world.y =
            map.origin_y + (cluster.centroid_grid.y + 0.5) * map.resolution;

          // Find the closest free cell to the centroid for navigation.
          computeRepresentative(map, cluster);

          // Distance to robot (squared for comparison, sqrt for storage).
          const double dx = cluster.centroid_world.x - robot_position.x;
          const double dy = cluster.centroid_world.y - robot_position.y;
          cluster.distance_to_robot = std::sqrt(dx * dx + dy * dy);

          clusters.push_back(std::move(cluster));
        }
      }
    }
  }

  return clusters;
}

}  // namespace tunnel_frontier_explorer
