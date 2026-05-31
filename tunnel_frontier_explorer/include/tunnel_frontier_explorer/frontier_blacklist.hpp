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

#ifndef TUNNEL_FRONTIER_EXPLORER__FRONTIER_BLACKLIST_HPP_
#define TUNNEL_FRONTIER_EXPLORER__FRONTIER_BLACKLIST_HPP_

#include <chrono>
#include <cstddef>
#include <vector>

#include "tunnel_frontier_explorer/frontier_cluster.hpp"

namespace tunnel_frontier_explorer
{

/// Tracks temporarily forbidden frontier regions after navigation failures.
///
/// Entries have a configurable radius and timeout.  The clock is injected
/// (defaulting to std::chrono::steady_clock) so that unit tests can provide
/// deterministic time points without real waits.
class FrontierBlacklist
{
public:
  FrontierBlacklist() = default;

  /// Add a position to the blacklist.
  /// @param position  The centroid of the failed goal (world coords).
  /// @param now       Current time (injectable for testing).
  /// @param radius    Euclidean radius within which positions are blocked.
  /// @param timeout   Duration after which this entry expires.
  void add(
    const Point2D & position,
    std::chrono::steady_clock::time_point now,
    double radius,
    std::chrono::seconds timeout);

  /// Returns true if @p position falls within any non-expired entry's radius.
  bool contains(
    const Point2D & position,
    std::chrono::steady_clock::time_point now) const;

  /// Remove all expired entries.
  void cleanup(std::chrono::steady_clock::time_point now);

  /// Number of entries (may include expired ones — call cleanup first).
  std::size_t size() const {return entries_.size();}

private:
  struct Entry
  {
    Point2D position;
    double radius;
    std::chrono::steady_clock::time_point expires_at;
  };

  std::vector<Entry> entries_;
};

}  // namespace tunnel_frontier_explorer

#endif  // TUNNEL_FRONTIER_EXPLORER__FRONTIER_BLACKLIST_HPP_
