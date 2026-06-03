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

#include "tunnel_frontier_explorer/entrance_oscillation_detector.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <sstream>

namespace tunnel_frontier_explorer
{

// ── Constructor ─────────────────────────────────────────────────────────────

EntranceOscillationDetector::EntranceOscillationDetector(
  EntranceOscillationConfig config)
: config_(config)
{
}

// ── recordGoal ──────────────────────────────────────────────────────────────

void EntranceOscillationDetector::recordGoal(const GoalDispatchEvent & event)
{
  events_.push_back(event);
  while (static_cast<int>(events_.size()) > config_.window_goals) {
    events_.pop_front();
  }
}

// ── getCurrentUniqueBins ────────────────────────────────────────────────────

int EntranceOscillationDetector::getCurrentUniqueBins() const
{
  std::set<int> bins;
  for (const auto & e : events_) {
    bins.insert(e.selected_bin);
  }
  return static_cast<int>(bins.size());
}

// ── evaluate ────────────────────────────────────────────────────────────────

OscillationStatus EntranceOscillationDetector::evaluate()
{
  OscillationStatus status;

  if (static_cast<int>(events_.size()) < config_.min_goals_to_check) {
    return status;
  }

  // ── Spatial radius: max pairwise distance between goals ──────────────
  double max_dist_sq = 0.0;
  for (std::size_t i = 0; i < events_.size(); ++i) {
    for (std::size_t j = i + 1; j < events_.size(); ++j) {
      const double dx = events_[i].goal.x - events_[j].goal.x;
      const double dy = events_[i].goal.y - events_[j].goal.y;
      const double d_sq = dx * dx + dy * dy;
      if (d_sq > max_dist_sq) {max_dist_sq = d_sq;}
    }
  }
  status.spatial_radius_m = std::sqrt(max_dist_sq);

  // ── Unique bins ─────────────────────────────────────────────────────
  std::set<int> bins;
  for (const auto & e : events_) {
    bins.insert(e.selected_bin);
  }
  const int unique_bins = static_cast<int>(bins.size());

  // ── Most-frequent bin count (stagnant_bin_count) ────────────────────
  std::map<int, int> bin_counts;
  for (const auto & e : events_) {
    bin_counts[e.selected_bin]++;
  }
  int max_bin_count = 0;
  for (const auto & kv : bin_counts) {
    if (kv.second > max_bin_count) {max_bin_count = kv.second;}
  }
  status.stagnant_bin_count = max_bin_count;
  status.repeated_goal_count = max_bin_count;

  // ── Average revisit ratio ───────────────────────────────────────────
  double sum_revisit = 0.0;
  for (const auto & e : events_) {
    sum_revisit += e.revisit_ratio;
  }
  const double avg_revisit = sum_revisit /
    static_cast<double>(events_.size());

  // ── Check conditions (3 out of 4 → oscillation) ────────────────────
  int conditions_met = 0;
  std::vector<std::string> reasons;

  // Cond 1: spatial radius small
  if (status.spatial_radius_m < config_.radius_m) {
    conditions_met++;
    reasons.push_back("localized_goals");
  }

  // Cond 2: few unique bins
  if (unique_bins <= config_.max_unique_bins) {
    conditions_met++;
    reasons.push_back("stagnant_bins");
  }

  // Cond 3: most-frequent bin dominates (> half of window)
  if (max_bin_count > config_.window_goals / 2) {
    conditions_met++;
    reasons.push_back("bin_dominance");
  }

  // Cond 4: high average revisit ratio
  if (avg_revisit >= config_.min_revisit_ratio) {
    conditions_met++;
    reasons.push_back("high_revisit");
  }

  // ── Result ──────────────────────────────────────────────────────────
  if (conditions_met >= 3) {
    status.oscillating = true;
    total_oscillation_events_++;
    status.event_count = total_oscillation_events_;
    std::ostringstream oss;
    for (std::size_t i = 0; i < reasons.size(); ++i) {
      if (i > 0) {oss << "+";}
      oss << reasons[i];
    }
    status.reason = oss.str();
  }

  return status;
}

// ── reset ───────────────────────────────────────────────────────────────────

void EntranceOscillationDetector::reset()
{
  events_.clear();
  total_oscillation_events_ = 0;
}

// ── windowSize ──────────────────────────────────────────────────────────────

std::size_t EntranceOscillationDetector::windowSize() const
{
  return events_.size();
}

// ── eventCount ──────────────────────────────────────────────────────────────

std::size_t EntranceOscillationDetector::eventCount() const
{
  return static_cast<std::size_t>(total_oscillation_events_);
}

}  // namespace tunnel_frontier_explorer
