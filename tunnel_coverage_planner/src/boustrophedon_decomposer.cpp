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

#include "tunnel_coverage_planner/boustrophedon_decomposer.hpp"

#include <algorithm>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace tunnel_coverage_planner
{

namespace
{

struct Run
{
  int c0 = 0;
  int c1 = 0;
  int label = -1;
};

}  // namespace

std::size_t CoverageCell::cellCount() const
{
  return static_cast<std::size_t>(
    std::count(mask.begin(), mask.end(), static_cast<std::uint8_t>(1)));
}

tunnel_map_core::Point2D CoverageCell::centroid(
  const tunnel_map_core::GridGeometry & g) const
{
  double sx = 0.0;
  double sy = 0.0;
  std::size_t n = 0;
  const std::size_t w = g.width();
  for (std::size_t row = 0; row < g.height(); ++row) {
    for (std::size_t col = 0; col < w; ++col) {
      if (mask[row * w + col] != 0) {
        const auto c = g.gridToWorld(
          static_cast<int>(row), static_cast<int>(col));
        sx += c.x;
        sy += c.y;
        ++n;
      }
    }
  }
  tunnel_map_core::Point2D p;
  if (n > 0) {
    p.x = sx / static_cast<double>(n);
    p.y = sy / static_cast<double>(n);
  }
  return p;
}

BoustrophedonDecomposer::BoustrophedonDecomposer(
  const tunnel_map_core::GridGeometry & geometry,
  const CoverageMasks & masks)
: geometry_(geometry), masks_(masks)
{
  if (!masks_.consistent()) {
    throw std::invalid_argument(
      "BoustrophedonDecomposer — inconsistent masks");
  }
}

std::vector<BoustrophedonDecomposer::Run> BoustrophedonDecomposer::rowRuns(
  int row) const
{
  const std::size_t w = geometry_.width();
  const auto & reach = masks_.reachable_cleanable;
  std::vector<BoustrophedonDecomposer::Run> runs;
  bool in_run = false;
  for (std::size_t col = 0; col < w; ++col) {
    const bool on =
      reach[static_cast<std::size_t>(row) * w + col] != 0;
    if (on && !in_run) {
      runs.push_back(BoustrophedonDecomposer::Run{
          static_cast<int>(col), static_cast<int>(col), -1});
      in_run = true;
    } else if (on) {
      runs.back().c1 = static_cast<int>(col);
    } else {
      in_run = false;
    }
  }
  return runs;
}

std::vector<CoverageCell> BoustrophedonDecomposer::decompose() const
{
  const std::size_t w = geometry_.width();
  const std::size_t h = geometry_.height();

  // Per-label state.  Label index == creation order == final cell index, so
  // the output order is deterministic (row-major scan).
  struct LabelState
  {
    std::vector<std::tuple<int, int, int>> row_runs;  // (row, c0, c1)
    std::set<int> neighbors;
  };
  std::vector<LabelState> states;

  std::vector<BoustrophedonDecomposer::Run> prev;

  for (std::size_t row = 0; row < h; ++row) {
    const auto runs = rowRuns(static_cast<int>(row));
    if (runs.empty()) {
      prev.clear();
      continue;
    }

    // Overlap maps between previous runs and this row's runs.
    std::vector<std::vector<int>> new_over_prev(runs.size());
    std::vector<std::vector<int>> prev_over_new(prev.size());
    for (std::size_t i = 0; i < runs.size(); ++i) {
      for (std::size_t j = 0; j < prev.size(); ++j) {
        if (runs[i].c0 <= prev[j].c1 && prev[j].c0 <= runs[i].c1) {
          new_over_prev[i].push_back(static_cast<int>(j));
          prev_over_new[j].push_back(static_cast<int>(i));
        }
      }
    }

    std::vector<BoustrophedonDecomposer::Run> cur;
    cur.reserve(runs.size());
    for (std::size_t i = 0; i < runs.size(); ++i) {
      const auto & P = new_over_prev[i];
      int label = -1;
      if (P.empty()) {
        // A fresh region appears on this row (new cell, no predecessor).
        label = static_cast<int>(states.size());
        states.emplace_back();
      } else if (P.size() == 1) {
        const auto p = P[0];
        if (prev_over_new[p].size() == 1) {
          // 1:1 continuation of the previous run.
          label = prev[static_cast<std::size_t>(p)].label;
        } else {
          // Split: one previous run fans out into several new runs.
          label = static_cast<int>(states.size());
          states.emplace_back();
          const int old_label =
            prev[static_cast<std::size_t>(p)].label;
          states[static_cast<std::size_t>(label)].neighbors.insert(old_label);
          states[static_cast<std::size_t>(old_label)].neighbors.insert(label);
        }
      } else {
        // Merge: several previous runs fuse into one new run.
        label = static_cast<int>(states.size());
        states.emplace_back();
        for (const auto p : P) {
          const int old_label =
            prev[static_cast<std::size_t>(p)].label;
          states[static_cast<std::size_t>(label)].neighbors.insert(old_label);
          states[static_cast<std::size_t>(old_label)].neighbors.insert(label);
        }
      }
      states[static_cast<std::size_t>(label)].row_runs.emplace_back(
        static_cast<int>(row), runs[i].c0, runs[i].c1);
      cur.push_back(BoustrophedonDecomposer::Run{
          runs[i].c0, runs[i].c1, label});
    }
    prev = cur;
  }

  // Rebuild masks and bounding boxes.
  std::vector<CoverageCell> cells;
  cells.reserve(states.size());
  for (std::size_t li = 0; li < states.size(); ++li) {
    CoverageCell cell;
    cell.id = "c" + std::to_string(li);
    cell.neighbors.assign(
      states[li].neighbors.begin(), states[li].neighbors.end());
    cell.mask.assign(w * h, 0);
    for (const auto & t : states[li].row_runs) {
      const int row = std::get<0>(t);
      const int c0 = std::get<1>(t);
      const int c1 = std::get<2>(t);
      cell.row0 = cell.row0 < 0 ? row : std::min(cell.row0, row);
      cell.row1 = std::max(cell.row1, row);
      cell.col0 = cell.col0 < 0 ? c0 : std::min(cell.col0, c0);
      cell.col1 = std::max(cell.col1, c1);
      for (int col = c0; col <= c1; ++col) {
        cell.mask[static_cast<std::size_t>(row) * w +
          static_cast<std::size_t>(col)] = 1;
      }
    }
    cells.push_back(std::move(cell));
  }
  return cells;
}

}  // namespace tunnel_coverage_planner
