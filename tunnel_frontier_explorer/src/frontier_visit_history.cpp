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

#include "tunnel_frontier_explorer/frontier_visit_history.hpp"

#include <cmath>

namespace tunnel_frontier_explorer
{

void FrontierVisitHistory::recordAcceptedGoal(const Point2D & goal)
{
  accepted_goals_.push_back(goal);
}

std::size_t FrontierVisitHistory::countVisitsNear(
  const Point2D & candidate,
  double radius_meters) const
{
  std::size_t count = 0;
  const double radius_sq = radius_meters * radius_meters;
  for (const auto & g : accepted_goals_) {
    const double dx = g.x - candidate.x;
    const double dy = g.y - candidate.y;
    if (dx * dx + dy * dy <= radius_sq) {
      ++count;
    }
  }
  return count;
}

void FrontierVisitHistory::clear()
{
  accepted_goals_.clear();
}

std::size_t FrontierVisitHistory::size() const
{
  return accepted_goals_.size();
}

}  // namespace tunnel_frontier_explorer
