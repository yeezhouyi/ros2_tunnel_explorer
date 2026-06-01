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

#ifndef TUNNEL_FRONTIER_EXPLORER__FRONTIER_VISIT_HISTORY_HPP_
#define TUNNEL_FRONTIER_EXPLORER__FRONTIER_VISIT_HISTORY_HPP_

#include <cstddef>
#include <vector>

#include "tunnel_frontier_explorer/frontier_cluster.hpp"

namespace tunnel_frontier_explorer
{

/// Tracks dispatched navigation goals to penalise re-visiting recently
/// attempted areas.
///
/// Only goals that were accepted by the Nav2 action server are recorded.
/// Goals rejected by the server or never sent do not create entries.
///
/// Zero ROS dependencies.
class FrontierVisitHistory
{
public:
  /// Record a goal that was accepted by the Nav2 action server.
  void recordAcceptedGoal(const Point2D & goal);

  /// Count how many recorded goals are within @p radius of @p candidate.
  std::size_t countVisitsNear(
    const Point2D & candidate,
    double radius_meters) const;

  /// Remove all recorded goals.
  void clear();

  /// Number of accepted goals recorded.
  std::size_t size() const;

private:
  std::vector<Point2D> accepted_goals_;
};

}  // namespace tunnel_frontier_explorer

#endif  // TUNNEL_FRONTIER_EXPLORER__FRONTIER_VISIT_HISTORY_HPP_
