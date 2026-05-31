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

#include "tunnel_frontier_explorer/frontier_blacklist.hpp"

#include <algorithm>
#include <cmath>

namespace tunnel_frontier_explorer
{

void FrontierBlacklist::add(
  const Point2D & position,
  std::chrono::steady_clock::time_point now,
  const double radius,
  const std::chrono::seconds timeout)
{
  entries_.push_back({
      position,
      radius,
      now + timeout
  });
}

bool FrontierBlacklist::contains(
  const Point2D & position,
  std::chrono::steady_clock::time_point now) const
{
  for (const auto & entry : entries_) {
    if (now >= entry.expires_at) {
      continue;  // expired, skip
    }
    const double dx = position.x - entry.position.x;
    const double dy = position.y - entry.position.y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (dist <= entry.radius) {
      return true;
    }
  }
  return false;
}

void FrontierBlacklist::cleanup(std::chrono::steady_clock::time_point now)
{
  entries_.erase(
    std::remove_if(entries_.begin(), entries_.end(),
    [&](const Entry & e) {return now >= e.expires_at;}),
    entries_.end());
}

}  // namespace tunnel_frontier_explorer
