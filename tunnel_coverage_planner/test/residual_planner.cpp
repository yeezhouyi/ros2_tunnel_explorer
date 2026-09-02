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

#include "tunnel_coverage_planner/residual_planner.hpp"

#include <algorithm>
#include <deque>
#include <stdexcept>
#include <utility>
#include <vector>

#include "tunnel_map_core/grid_map.hpp"

namespace tunnel_coverage_planner
{

namespace
{

using tunnel_map_core::Point2D;

/// 8-connected neighbourhood offsets.
constexpr int kDr8[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
constexpr int kDc8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

}  // namespace

ResidualPlanner::ResidualPlanner(
  const tunnel_map_core::GridGeometry & geometry,
  const CoverageMasks & masks,
  const RobotCleaningGeometry & robot,
  const ScanlinePlannerConfig & config)
: geometry_(geometry), masks_(masks), robot_(robot), config_(config)
{
  if (!masks_.consistent()) {
    throw std::invalid_argument("ResidualPlanner — inconsistent masks");
  }
  if (!robot_.valid()) {
    throw std::invalid_argument(
      "ResidualPlanner — " + robot_.validationError());
  }
}

ResidualResult ResidualPlanner::planResidual(
  const std::vector<std::int32_t> & visit_counts) const
{
  ResidualResult result;
  const std::size_t w = geometry_.width();
  const std::size_t h = geometry_.height();
  const std::size_t n = w * h;
  if (visit_counts.size() != n) {
    throw std::invalid_argument(
      "ResidualPlanner — visit_counts size mismatch");
  }
  const double cell_area = geometry_.cellSize() * geometry_.cellSize();
  const double min_cells =
    robot_.min_cleanable_region_area_m2 / cell_area;

  // Effective target cells that were not covered.
  std::vector<std::uint8_t> uncovered(n, 0);
  std::size_t uncovered_total = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (masks_.intended_target[i] != 0 && masks_.exempt[i] == 0 &&
      visit_counts[i] == 0)
    {
      uncovered[i] = 1;
      ++uncovered_total;
    }
  }
  result.total_uncovered_cells = uncovered_total;
  if (uncovered_total == 0) {
    return result;
  }

  // Connected components of the uncovered mask.
  std::vector<std::uint8_t> seen(n, 0);
  for (std::size_t start = 0; start < n; ++start) {
    if (uncovered[start] == 0 || seen[start] != 0) {
      continue;
    }
    std::vector<std::size_t> comp;
    std::deque<std::size_t> queue;
    seen[start] = 1;
    queue.push_back(start);
    while (!queue.empty()) {
      const auto idx = queue.front();
      queue.pop_front();
      comp.push_back(idx);
      const auto row = static_cast<std::int64_t>(idx / w);
      const auto col = static_cast<std::int64_t>(idx % w);
      for (int k = 0; k < 8; ++k) {
        const auto rr = row + kDr8[k];
        const auto cc = col + kDc8[k];
        if (rr < 0 || cc < 0 ||
          static_cast<std::size_t>(rr) >= h ||
          static_cast<std::size_t>(cc) >= w)
        {
          continue;
        }
        const auto j = static_cast<std::size_t>(rr) * w +
          static_cast<std::size_t>(cc);
        if (uncovered[j] != 0 && seen[j] == 0) {
          seen[j] = 1;
          queue.push_back(j);
        }
      }
    }

    if (static_cast<double>(comp.size()) < min_cells) {
      // Too small to bother the robot: auditable exemption, never dropped.
      result.exempted_uncovered_cells += comp.size();
      continue;
    }

    // Local masks: sweep exactly this residual component.  Rows are taken
    // from the *global* navigable mask so a wall strip can be swept from the
    // nearest navigable band.
    CoverageMasks local;
    local.geometry = geometry_;
    local.intended_target.assign(n, 0);
    local.navigable_center = masks_.navigable_center;
    local.reachable_cleanable.assign(n, 0);
    local.exempt.assign(n, 0);
    local.exempt_cause.assign(n, 0);
    Point2D seed{0.0, 0.0};
    for (const auto idx : comp) {
      local.intended_target[idx] = 1;
      local.reachable_cleanable[idx] = 1;
      const auto row = static_cast<int>(idx / w);
      const auto col = static_cast<int>(idx % w);
      const auto c = geometry_.gridToWorld(row, col);
      seed.x += c.x;
      seed.y += c.y;
    }
    seed.x /= static_cast<double>(comp.size());
    seed.y /= static_cast<double>(comp.size());

    ScanlinePlannerConfig sub_cfg = config_;
    sub_cfg.min_segment_length_m = std::min(config_.min_segment_length_m, 0.15);
    ScanlinePlanner planner(geometry_, local, robot_, sub_cfg);
    auto patch = planner.planMultiCell(seed);
    if (patch.valid()) {
      result.patches.push_back(std::move(patch));
    } else {
      // Residual region not plannable with the scanline generator (too
      // fragmented); keep it as an auditable exemption for now.
      result.exempted_uncovered_cells += comp.size();
    }
  }
  return result;
}

}  // namespace tunnel_coverage_planner
