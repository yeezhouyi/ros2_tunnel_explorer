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

#include "tunnel_coverage_planner/segment_orderer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace tunnel_coverage_planner
{

std::vector<int> SegmentOrderer::order(
  const std::vector<std::vector<double>> & cost, int start)
{
  if (cost.empty()) {
    throw std::invalid_argument("SegmentOrderer — empty cost matrix");
  }
  const auto n = cost.size();
  for (const auto & row : cost) {
    if (row.size() != n) {
      throw std::invalid_argument(
        "SegmentOrderer — cost matrix must be square");
    }
  }
  if (start < 0 || static_cast<std::size_t>(start) >= n) {
    throw std::invalid_argument(
      "SegmentOrderer — start index out of range");
  }
  for (const auto & row : cost) {
    for (const auto v : row) {
      if (!std::isfinite(v) || v < 0.0) {
        throw std::invalid_argument(
          "SegmentOrderer — costs must be finite and non-negative");
      }
    }
  }

  std::vector<bool> visited(n, false);
  std::vector<int> order;
  order.reserve(n);
  int cur = start;
  visited[static_cast<std::size_t>(cur)] = true;
  order.push_back(cur);

  for (std::size_t step = 1; step < n; ++step) {
    int best = -1;
    double best_cost = 0.0;
    for (std::size_t j = 0; j < n; ++j) {
      if (visited[j]) {
        continue;
      }
      const double c = cost[static_cast<std::size_t>(cur)][j];
      if (best < 0 || c < best_cost - 1e-12 ||
        (std::abs(c - best_cost) <= 1e-12 && static_cast<int>(j) < best))
      {
        best = static_cast<int>(j);
        best_cost = c;
      }
    }
    cur = best;
    visited[static_cast<std::size_t>(cur)] = true;
    order.push_back(cur);
  }
  return order;
}

}  // namespace tunnel_coverage_planner
