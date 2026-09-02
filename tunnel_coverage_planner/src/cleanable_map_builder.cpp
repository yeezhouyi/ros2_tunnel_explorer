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

#include "tunnel_coverage_planner/cleanable_map_builder.hpp"

#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace tunnel_coverage_planner
{

namespace
{

using tunnel_map_core::GridCell;
using tunnel_map_core::Point2D;

/// Neighbour offsets for 4/8 connectivity.
std::vector<std::pair<int, int>> makeNeighbourhood(int connectivity)
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

}  // namespace

CleanableMapBuilder::CleanableMapBuilder(
  const tunnel_map_core::GridGeometry & geometry,
  CleanableMapBuilderConfig config)
: geometry_(geometry), config_(config)
{
  if (config_.free_max < -1 || config_.occupied_min < 0 ||
    config_.free_max >= config_.occupied_min)
  {
    throw std::invalid_argument(
      "CleanableMapBuilderConfig invalid: require -1 <= free_max < "
      "occupied_min");
  }
  if (config_.connectivity != 4 && config_.connectivity != 8) {
    throw std::invalid_argument("connectivity must be 4 or 8");
  }
}

void CleanableMapBuilder::classify(
  const tunnel_map_core::GridMap & map,
  std::vector<std::uint8_t> & free_mask,
  std::vector<std::uint8_t> & occupied_mask) const
{
  const auto n = map.data.size();
  free_mask.assign(n, 0);
  occupied_mask.assign(n, 0);
  for (std::size_t i = 0; i < n; ++i) {
    const auto v = static_cast<int>(map.data[i]);
    if (v >= 0 && v <= config_.free_max) {
      free_mask[i] = 1;
    }
    if (v >= config_.occupied_min) {
      occupied_mask[i] = 1;
    }
  }
}

std::vector<std::uint8_t> CleanableMapBuilder::erodeByDisc(
  const std::vector<std::uint8_t> & free_mask, double radius_cells) const
{
  const std::size_t w = geometry_.width();
  const std::size_t h = geometry_.height();
  std::vector<std::uint8_t> out(free_mask.size(), 0);

  if (radius_cells <= 0.0) {
    return free_mask;
  }

  const double r2 = radius_cells * radius_cells + 1e-9;
  const int k = static_cast<int>(std::ceil(radius_cells));

  for (std::size_t row = 0; row < h; ++row) {
    for (std::size_t col = 0; col < w; ++col) {
      const auto idx = row * w + col;
      if (free_mask[idx] == 0) {
        continue;
      }
      bool ok = true;
      for (int dr = -k; dr <= k && ok; ++dr) {
        for (int dc = -k; dc <= k; ++dc) {
          const double d2 = static_cast<double>(dr * dr + dc * dc);
          if (d2 > r2) {
            continue;
          }
          const auto rr = static_cast<long long>(row) + dr;
          const auto cc = static_cast<long long>(col) + dc;
          if (rr < 0 || cc < 0 ||
            static_cast<std::size_t>(rr) >= h ||
            static_cast<std::size_t>(cc) >= w)
          {
            // Off-map cells are treated as not free: the chassis may not
            // leave the map area.
            ok = false;
            break;
          }
          if (free_mask[static_cast<std::size_t>(rr) * w +
            static_cast<std::size_t>(cc)] == 0)
          {
            ok = false;
            break;
          }
        }
      }
      if (ok) {
        out[idx] = 1;
      }
    }
  }
  return out;
}

std::vector<std::uint8_t> CleanableMapBuilder::dilateByDisc(
  const std::vector<std::uint8_t> & src, double radius_cells) const
{
  const std::size_t w = geometry_.width();
  const std::size_t h = geometry_.height();
  std::vector<std::uint8_t> out(src.size(), 0);

  if (radius_cells <= 0.0) {
    return src;
  }

  const double r2 = radius_cells * radius_cells + 1e-9;
  const int k = static_cast<int>(std::ceil(radius_cells));

  for (std::size_t row = 0; row < h; ++row) {
    for (std::size_t col = 0; col < w; ++col) {
      const auto idx = row * w + col;
      if (src[idx] == 0) {
        continue;
      }
      for (int dr = -k; dr <= k; ++dr) {
        for (int dc = -k; dc <= k; ++dc) {
          const double d2 = static_cast<double>(dr * dr + dc * dc);
          if (d2 > r2) {
            continue;
          }
          const auto rr = static_cast<long long>(row) + dr;
          const auto cc = static_cast<long long>(col) + dc;
          if (rr < 0 || cc < 0 ||
            static_cast<std::size_t>(rr) >= h ||
            static_cast<std::size_t>(cc) >= w)
          {
            continue;
          }
          out[static_cast<std::size_t>(rr) * w +
            static_cast<std::size_t>(cc)] = 1;
        }
      }
    }
  }
  return out;
}

std::vector<std::uint8_t> CleanableMapBuilder::connectedComponentFrom(
  const std::vector<std::uint8_t> & src, std::size_t seed_index) const
{
  const std::size_t w = geometry_.width();
  const std::size_t h = geometry_.height();
  const auto offsets = makeNeighbourhood(config_.connectivity);

  std::vector<std::uint8_t> out(src.size(), 0);
  if (seed_index >= src.size() || src[seed_index] == 0) {
    return out;
  }

  std::deque<std::size_t> queue;
  out[seed_index] = 1;
  queue.push_back(seed_index);

  while (!queue.empty()) {
    const auto idx = queue.front();
    queue.pop_front();
    const auto row = static_cast<long long>(idx / w);
    const auto col = static_cast<long long>(idx % w);
    for (const auto & d : offsets) {
      const auto rr = row + d.first;
      const auto cc = col + d.second;
      if (rr < 0 || cc < 0 ||
        static_cast<std::size_t>(rr) >= h ||
        static_cast<std::size_t>(cc) >= w)
      {
        continue;
      }
      const auto nidx = static_cast<std::size_t>(rr) * w +
        static_cast<std::size_t>(cc);
      if (src[nidx] != 0 && out[nidx] == 0) {
        out[nidx] = 1;
        queue.push_back(nidx);
      }
    }
  }
  return out;
}

CoverageMasks CleanableMapBuilder::build(
  const tunnel_map_core::GridMap & map,
  const RobotCleaningGeometry & robot,
  const Point2D & seed_world) const
{
  if (!map.valid()) {
    throw std::invalid_argument("CleanableMapBuilder::build — invalid map");
  }
  if (map.width != geometry_.width() || map.height != geometry_.height()) {
    throw std::invalid_argument(
      "CleanableMapBuilder::build — map dimensions differ from geometry");
  }
  const std::string geo_error = robot.validationError();
  if (!geo_error.empty()) {
    throw std::invalid_argument(
      "CleanableMapBuilder::build — " + geo_error);
  }

  // Seed must land inside the map.
  GridCell seed_cell;
  if (!geometry_.worldToGridCell(seed_world, seed_cell)) {
    throw std::invalid_argument(
      "CleanableMapBuilder::build — seed pose outside map");
  }
  const auto seed_idx = geometry_.index(seed_cell.row, seed_cell.col);

  CoverageMasks out;
  out.geometry = geometry_;

  // 1. Free / occupied classification.
  std::vector<std::uint8_t> free_mask;
  std::vector<std::uint8_t> occupied_mask;
  classify(map, free_mask, occupied_mask);
  (void)occupied_mask;  // reserved for future geometry gates

  const double cell_m = geometry_.cellSize();

  // 2. Intended target: everything free.
  out.intended_target = free_mask;

  // 3. Navigable centre: free eroded by the chassis clearance disc.
  out.navigable_center = erodeByDisc(
    free_mask, robot.clearanceRadiusM() / cell_m);

  if (out.navigable_center[seed_idx] == 0) {
    throw std::invalid_argument(
      "CleanableMapBuilder::build — seed pose is not inside navigable "
      "centre (robot would start inside or too close to an obstacle)");
  }

  // 4. Reachable cleanable: navigable component at the seed, dilated by the
  //    tool sweep radius, intersected with the target.
  const auto navigable_component =
    connectedComponentFrom(out.navigable_center, seed_idx);
  const auto reach_dilated = dilateByDisc(
    navigable_component, robot.toolSweepRadiusM() / cell_m);

  const auto n = free_mask.size();
  out.reachable_cleanable.assign(n, 0);
  for (std::size_t i = 0; i < n; ++i) {
    if (free_mask[i] != 0 && reach_dilated[i] != 0) {
      out.reachable_cleanable[i] = 1;
    }
  }

  // 5. Tag cleanable components below the minimum-area gate as islands and
  //    drop them from the reachable set.  Dropped cells keep an auditable
  //    exemption cause (ISLAND) instead of being silently ignored.
  {
    const auto offsets = makeNeighbourhood(config_.connectivity);
    const double min_cells =
      robot.min_cleanable_region_area_m2 / (cell_m * cell_m);
    std::vector<std::uint8_t> visited(n, 0);
    std::vector<std::uint8_t> island_of(n, 0);
    std::deque<std::size_t> queue;

    for (std::size_t start = 0; start < n; ++start) {
      if (out.reachable_cleanable[start] == 0 || visited[start] != 0) {
        continue;
      }
      // Collect one connected component of the *initial* reachable set.
      std::vector<std::size_t> comp;
      visited[start] = 1;
      queue.clear();
      queue.push_back(start);
      while (!queue.empty()) {
        const auto idx = queue.front();
        queue.pop_front();
        comp.push_back(idx);
        const auto row = static_cast<long long>(idx / geometry_.width());
        const auto col = static_cast<long long>(idx % geometry_.width());
        for (const auto & d : offsets) {
          const auto rr = row + d.first;
          const auto cc = col + d.second;
          if (rr < 0 || cc < 0 ||
            static_cast<std::size_t>(rr) >= geometry_.height() ||
            static_cast<std::size_t>(cc) >= geometry_.width())
          {
            continue;
          }
          const auto nidx = static_cast<std::size_t>(rr) * geometry_.width() +
            static_cast<std::size_t>(cc);
          if (out.reachable_cleanable[nidx] != 0 && visited[nidx] == 0) {
            visited[nidx] = 1;
            queue.push_back(nidx);
          }
        }
      }
      if (static_cast<double>(comp.size()) < min_cells) {
        for (const auto idx : comp) {
          island_of[idx] = 1;
        }
      }
    }

    for (std::size_t i = 0; i < n; ++i) {
      if (island_of[i] != 0) {
        out.reachable_cleanable[i] = 0;
      }
    }

    // 6. Exemptions: intended-target cells not in the final reachable set.
    out.exempt.assign(n, 0);
    out.exempt_cause.assign(n, 0);
    for (std::size_t i = 0; i < n; ++i) {
      if (out.intended_target[i] == 0) {
        continue;  // not a target -> never in the exempt denominator
      }
      if (out.reachable_cleanable[i] != 0) {
        continue;
      }
      out.exempt[i] = 1;
      out.exempt_cause[i] =
        island_of[i] != 0 ? static_cast<std::uint8_t>(ExemptCause::ISLAND) :
        static_cast<std::uint8_t>(ExemptCause::UNREACHABLE);
    }
  }

  return out;
}

}  // namespace tunnel_coverage_planner
