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

// ── getOscillationCenter ────────────────────────────────────────────────────

Point2D EntranceOscillationDetector::getOscillationCenter() const
{
  if (events_.empty()) {return {0.0, 0.0};}
  double sum_x = 0.0, sum_y = 0.0;
  for (const auto & e : events_) {
    sum_x += e.goal.x;
    sum_y += e.goal.y;
  }
  const double n = static_cast<double>(events_.size());
  return {sum_x / n, sum_y / n};
}

// ── detectAlternatingPair ──────────────────────────────────────────────────

AlternatingPairStatus
EntranceOscillationDetector::detectAlternatingPair() const
{
  AlternatingPairStatus status;

  if (!config_.detect_alternating_pair) {return status;}
  if (static_cast<int>(events_.size()) < config_.min_goals_to_check) {
    return status;
  }

  // ── Step 1: Greedy cluster goals by spatial proximity ────────────────
  // Use fixed anchor (first goal) to avoid center drift.
  struct Cluster {
    Point2D anchor;   // first goal position (fixed)
    int count = 0;
  };
  std::vector<Cluster> clusters;

  for (const auto & e : events_) {
    bool joined = false;
    for (auto & c : clusters) {
      const double d = std::hypot(e.goal.x - c.anchor.x, e.goal.y - c.anchor.y);
      if (d <= config_.pair_cluster_radius_m) {
        c.count++;
        joined = true;
        break;
      }
    }
    if (!joined) {
      Cluster new_cluster;
      new_cluster.anchor = e.goal;
      new_cluster.count = 1;
      clusters.push_back(new_cluster);
    }
  }

  status.cluster_count = static_cast<int>(clusters.size());

  // ── Step 2: Check cluster count == 2 ────────────────────────────────
  if (status.cluster_count != 2) {return status;}

  // ── Step 3: Check each cluster has enough goals ─────────────────────
  if (clusters[0].count < config_.pair_min_cluster_count ||
      clusters[1].count < config_.pair_min_cluster_count)
  {
    return status;
  }

  status.cluster_a_center = clusters[0].anchor;
  status.cluster_b_center = clusters[1].anchor;

  // ── Step 4: Check overall spatial radius ─────────────────────────────
  double max_dist_sq = 0.0;
  for (const auto & e : events_) {
    for (const auto & e2 : events_) {
      const double d_sq = std::pow(e.goal.x - e2.goal.x, 2) +
                          std::pow(e.goal.y - e2.goal.y, 2);
      if (d_sq > max_dist_sq) {max_dist_sq = d_sq;}
    }
  }
  const double overall_radius = std::sqrt(max_dist_sq);
  if (overall_radius > config_.pair_max_spatial_radius_m) {return status;}

  // ── Step 5: Check unique bins not growing ───────────────────────────
  std::set<int> first_half_bins, second_half_bins;
  const std::size_t mid = events_.size() / 2;
  for (std::size_t i = 0; i < mid; ++i) {
    first_half_bins.insert(events_[i].selected_bin);
  }
  for (std::size_t i = mid; i < events_.size(); ++i) {
    second_half_bins.insert(events_[i].selected_bin);
  }
  // If second half has more unique bins than first, progress is happening
  if (static_cast<int>(second_half_bins.size()) >
      static_cast<int>(first_half_bins.size()))
  {
    return status;
  }

  // ── Step 6: Compute alternation score ────────────────────────────────
  // Assign each goal to its nearest cluster
  std::vector<int> assignments;
  assignments.reserve(events_.size());
  for (const auto & e : events_) {
    const double d_a = std::hypot(
      e.goal.x - clusters[0].anchor.x, e.goal.y - clusters[0].anchor.y);
    const double d_b = std::hypot(
      e.goal.x - clusters[1].anchor.x, e.goal.y - clusters[1].anchor.y);
    assignments.push_back(d_a <= d_b ? 0 : 1);
  }

  int transitions = 0;
  for (std::size_t i = 1; i < assignments.size(); ++i) {
    if (assignments[i] != assignments[i - 1]) {transitions++;}
  }
  status.alternation_score =
    static_cast<double>(transitions) /
    static_cast<double>(events_.size() - 1);

  if (status.alternation_score < config_.pair_min_alternation_score) {
    return status;
  }

  // ── All conditions met ──────────────────────────────────────────────
  status.detected = true;
  status.reason = "two_cluster_pair+stagnant_bins+localized_goals";
  return status;
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
    status.reason = "type=revisit_heavy " + oss.str();
  }

  // ── Type B: alternating-pair fallback ──────────────────────────────
  if (!status.oscillating && config_.detect_alternating_pair) {
    auto pair = detectAlternatingPair();
    if (pair.detected) {
      status.oscillating = true;
      total_oscillation_events_++;
      status.event_count = total_oscillation_events_;
      status.reason = "type=alternating_pair " + pair.reason;
    }
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
