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

#include "tunnel_frontier_explorer/frontier_explorer_node.hpp"

#include <tf2/exceptions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace tunnel_frontier_explorer
{

// ── Constructor ─────────────────────────────────────────────────────────

TunnelFrontierExplorerNode::TunnelFrontierExplorerNode()
: Node("tunnel_frontier_explorer"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  detector_(FrontierDetectorConfig{}),
  blacklist_(),
  goal_selector_(FrontierGoalSelectorConfig{})
{
  // ── Declare parameters with defaults ─────────────────────────────────
  map_topic_ = declare_parameter<std::string>("map_topic", "/map");
  marker_topic_ = declare_parameter<std::string>(
    "marker_topic", "~/frontier_markers");
  action_server_name_ = declare_parameter<std::string>(
    "navigate_to_pose_action", "/navigate_to_pose");
  global_frame_ = declare_parameter<std::string>("global_frame", "map");
  robot_base_frame_ = declare_parameter<std::string>(
    "robot_base_frame", "base_footprint");

  exploration_period_seconds_ = declare_parameter<double>(
    "exploration_period_seconds", 1.0);
  cooldown_seconds_ = declare_parameter<double>("cooldown_seconds", 5.0);
  goal_timeout_seconds_ = declare_parameter<double>("goal_timeout_seconds", 60.0);
  min_cluster_size_ = declare_parameter<int>("min_cluster_size", 10);
  free_threshold_ = declare_parameter<int>("free_threshold", 0);
  frontier_neighbor_connectivity_ = declare_parameter<int>(
    "frontier_neighbor_connectivity", 4);
  cluster_connectivity_ = declare_parameter<int>(
    "cluster_connectivity", 8);
  blacklist_radius_ = declare_parameter<double>(
    "blacklist_radius_meters", 0.5);
  blacklist_timeout_seconds_ = declare_parameter<double>(
    "blacklist_timeout_seconds", 60.0);
  orient_goal_toward_frontier_ = declare_parameter<bool>(
    "orient_goal_toward_frontier", true);
  min_goal_distance_meters_ = declare_parameter<double>(
    "min_goal_distance_meters", 0.50);
  goal_projection_enabled_ = declare_parameter<bool>(
    "goal_projection_enabled", true);
  goal_projection_distance_ = declare_parameter<double>(
    "goal_projection_distance", 0.4);
  goal_projection_min_remaining_distance_ = declare_parameter<double>(
    "goal_projection_min_remaining_distance", 0.6);
  goal_success_cooldown_seconds_ = declare_parameter<double>(
    "goal_success_cooldown_seconds", 120.0);
  goal_success_cooldown_radius_ = declare_parameter<double>(
    "goal_success_cooldown_radius", 1.0);

  // -- Stage 3D: entrance-loop recovery -------------------------------------
  loop_detection_enabled_ = declare_parameter<bool>(
    "loop_detection_enabled", true);
  loop_window_size_ = declare_parameter<int>(
    "loop_window_size", 6);
  loop_unique_bins_threshold_ = declare_parameter<int>(
    "loop_unique_bins_threshold", 2);
  loop_min_successes_ = declare_parameter<int>(
    "loop_min_successes", 3);
  loop_bin_size_ = declare_parameter<double>(
    "loop_bin_size", 0.75);
  recovery_probe_enabled_ = declare_parameter<bool>(
    "recovery_probe_enabled", true);
  recovery_probe_distances_ = declare_parameter<std::vector<double>>(
    "recovery_probe_distances", {1.2, 1.0, 0.8});
  {
    auto angles_deg = declare_parameter<std::vector<double>>(
      "recovery_probe_angle_offsets_deg", {0.0, 20.0, -20.0, 35.0, -35.0});
    recovery_probe_angle_offsets_rad_.clear();
    for (double deg : angles_deg) {
      recovery_probe_angle_offsets_rad_.push_back(deg * M_PI / 180.0);
    }
  }
  recovery_probe_cooldown_seconds_ = declare_parameter<double>(
    "recovery_probe_cooldown_seconds", 30.0);
  recovery_max_attempts_ = declare_parameter<int>(
    "recovery_max_attempts", 3);

  // -- Stage 4E.2: startup bootstrap probe ──────────────────────────────
  startup_bootstrap_enabled_ = declare_parameter<bool>(
    "startup_bootstrap_enabled", true);
  startup_bootstrap_distance_m_ = declare_parameter<double>(
    "startup_bootstrap_distance_m", 0.75);
  startup_bootstrap_max_attempts_ = declare_parameter<int>(
    "startup_bootstrap_max_attempts", 1);
  startup_bootstrap_no_frontier_cycles_ = declare_parameter<int>(
    "startup_bootstrap_no_frontier_cycles", 10);

  // -- Stage 4B: tunnel geometry ─────────────────────────────────────────
  tunnel_distance_map_topic_ = declare_parameter<std::string>(
    "tunnel_distance_map_topic", "/tunnel_centerline/distance_map");
  tunnel_risk_map_topic_ = declare_parameter<std::string>(
    "tunnel_risk_map_topic", "/tunnel_centerline/risk_map");
  tunnel_geometry_max_age_seconds_ = declare_parameter<double>(
    "tunnel_geometry_max_age_seconds", 3.0);
  geometry_missing_fallback_to_stage3d_ = declare_parameter<bool>(
    "geometry_missing_fallback_to_stage3d", true);

  // -- Stage 4B.2: entrance oscillillation detection ──────────────────────
  entrance_oscillation_enabled_ = declare_parameter<bool>(
    "entrance_oscillation_enabled", true);
  entrance_oscillation_window_goals_ = declare_parameter<int>(
    "entrance_oscillation_window_goals", 6);
  entrance_oscillation_radius_m_ = declare_parameter<double>(
    "entrance_oscillation_radius_m", 2.0);
  entrance_oscillation_min_repeated_goals_ = declare_parameter<int>(
    "entrance_oscillation_min_repeated_goals", 4);
  entrance_oscillation_max_unique_bins_ = declare_parameter<int>(
    "entrance_oscillation_max_unique_bins", 2);
  entrance_oscillation_min_revisit_ratio_ = declare_parameter<double>(
    "entrance_oscillation_min_revisit_ratio", 0.35);
  entrance_oscillation_min_goals_to_check_ = declare_parameter<int>(
    "entrance_oscillation_min_goals_to_check", 4);
  entrance_oscillation_detect_alternating_pair_ = declare_parameter<bool>(
    "entrance_oscillation_detect_alternating_pair", true);
  entrance_oscillation_pair_cluster_radius_m_ = declare_parameter<double>(
    "entrance_oscillation_pair_cluster_radius_m", 0.75);
  entrance_oscillation_pair_max_spatial_radius_m_ = declare_parameter<double>(
    "entrance_oscillation_pair_max_spatial_radius_m", 1.5);
  entrance_oscillation_pair_min_cluster_count_ = declare_parameter<int>(
    "entrance_oscillation_pair_min_cluster_count", 2);
  entrance_oscillation_pair_min_alternation_score_ = declare_parameter<double>(
    "entrance_oscillation_pair_min_alternation_score", 0.5);

  {
    EntranceOscillationConfig osc_cfg;
    osc_cfg.window_goals = entrance_oscillation_window_goals_;
    osc_cfg.radius_m = entrance_oscillation_radius_m_;
    osc_cfg.min_repeated_goals = entrance_oscillation_min_repeated_goals_;
    osc_cfg.max_unique_bins = entrance_oscillation_max_unique_bins_;
    osc_cfg.min_revisit_ratio = entrance_oscillation_min_revisit_ratio_;
    osc_cfg.min_goals_to_check = entrance_oscillation_min_goals_to_check_;
    osc_cfg.detect_alternating_pair = entrance_oscillation_detect_alternating_pair_;
    osc_cfg.pair_cluster_radius_m = entrance_oscillation_pair_cluster_radius_m_;
    osc_cfg.pair_max_spatial_radius_m = entrance_oscillation_pair_max_spatial_radius_m_;
    osc_cfg.pair_min_cluster_count = entrance_oscillation_pair_min_cluster_count_;
    osc_cfg.pair_min_alternation_score = entrance_oscillation_pair_min_alternation_score_;
    entrance_oscillation_detector_ = EntranceOscillationDetector(osc_cfg);
  }

  // -- Stage 4B.3: oscillillation escape mode ─────────────────────────────
  entrance_oscillation_response_enabled_ = declare_parameter<bool>(
    "entrance_oscillation_response_enabled", true);
  entrance_oscillation_escape_goals_ = declare_parameter<int>(
    "entrance_oscillation_escape_goals", 3);
  entrance_oscillation_suppression_radius_m_ = declare_parameter<double>(
    "entrance_oscillation_suppression_radius_m", 1.5);
  entrance_oscillation_escape_penalty_ = declare_parameter<double>(
    "entrance_oscillation_escape_penalty", 0.75);

  // -- Stage 4C.1: forced escape probe ─────────────────────────────────
  entrance_oscillation_force_escape_probe_ = declare_parameter<bool>(
    "entrance_oscillation_force_escape_probe", true);
  entrance_oscillation_probe_distance_m_ = declare_parameter<double>(
    "entrance_oscillation_probe_distance_m", 1.25);
  entrance_oscillation_probe_distance_step_m_ = declare_parameter<double>(
    "entrance_oscillation_probe_distance_step_m", 0.5);
  entrance_oscillation_probe_exit_margin_m_ = declare_parameter<double>(
    "entrance_oscillation_probe_exit_margin_m", 0.25);
  entrance_oscillation_probe_max_attempts_ = declare_parameter<int>(
    "entrance_oscillation_probe_max_attempts", 8);
  entrance_oscillation_probe_angle_span_deg_ = declare_parameter<double>(
    "entrance_oscillation_probe_angle_span_deg", 90.0);
  entrance_oscillation_probe_min_wall_distance_m_ = declare_parameter<double>(
    "entrance_oscillation_probe_min_wall_distance_m", 0.35);
  entrance_oscillation_probe_cooldown_goals_ = declare_parameter<int>(
    "entrance_oscillation_probe_cooldown_goals", 3);

  {
    EscapeProbeConfig ep_cfg;
    ep_cfg.probe_distance_m = entrance_oscillation_probe_distance_m_;
    ep_cfg.probe_distance_step_m = entrance_oscillation_probe_distance_step_m_;
    ep_cfg.probe_exit_margin_m = entrance_oscillation_probe_exit_margin_m_;
    ep_cfg.max_attempts = entrance_oscillation_probe_max_attempts_;
    ep_cfg.angle_span_deg = entrance_oscillation_probe_angle_span_deg_;
    ep_cfg.min_wall_distance_m = entrance_oscillation_probe_min_wall_distance_m_;
    probe_generator_ = EscapeProbeGenerator(ep_cfg);
  }

  // -- Stage 4D.1: conservative fallback gating ─────────────────────────
  entrance_oscillation_stuck_detector_enabled_ = declare_parameter<bool>(
    "entrance_oscillation_stuck_detector_enabled", true);
  entrance_oscillation_no_progress_threshold_m_ = declare_parameter<double>(
    "entrance_oscillation_no_progress_threshold_m", 0.1);
  entrance_oscillation_no_progress_min_steps_ = declare_parameter<int>(
    "entrance_oscillation_no_progress_min_steps", 3);
  entrance_oscillation_repeated_revisit_min_ = declare_parameter<int>(
    "entrance_oscillation_repeated_revisit_min", 2);
  entrance_oscillation_cooldown_goals_ = declare_parameter<int>(
    "entrance_oscillation_cooldown_goals", 5);
  entrance_oscillation_escape_max_goals_ = declare_parameter<int>(
    "entrance_oscillation_escape_max_goals", 5);
  entrance_oscillation_escape_deactivation_distance_m_ = declare_parameter<double>(
    "entrance_oscillation_escape_deactivation_distance_m", 1.0);
  entrance_oscillation_escape_no_oscillation_deactivate_goals_ = declare_parameter<int>(
    "entrance_oscillation_escape_no_oscillation_deactivate_goals", 2);

  // -- Stage 4D.2: cooldown starvation recovery ──────────────────────
  cooldown_starvation_recovery_enabled_ = declare_parameter<bool>(
    "cooldown_starvation_recovery_enabled", true);
  cooldown_starvation_recovery_threshold_ = declare_parameter<int>(
    "cooldown_starvation_recovery_threshold", 10);
  cooldown_starvation_recovery_match_tolerance_m_ = declare_parameter<double>(
    "cooldown_starvation_recovery_match_tolerance_m", 0.2);

  // -- Stage 2B parameters --------------------------------------------------
  selection_strategy_ = declare_parameter<std::string>(
    "selection_strategy", "nearest");
  information_gain_radius_meters_ = declare_parameter<double>(
    "information_gain_radius_meters", 0.75);
  revisit_radius_meters_ = declare_parameter<double>(
    "revisit_radius_meters", 0.50);
  max_revisit_count_ = declare_parameter<int>(
    "max_revisit_count", 3);
  weight_information_gain_ = declare_parameter<double>(
    "weight_information_gain", 1.0);
  weight_distance_ = declare_parameter<double>(
    "weight_distance", 1.0);
  weight_revisit_ = declare_parameter<double>(
    "weight_revisit", 1.5);

  // Validate selection_strategy.
  if (selection_strategy_ != "nearest" &&
    selection_strategy_ != "information_gain_revisit" &&
    selection_strategy_ != "tunnel_aware" &&
    selection_strategy_ != "tunnel_aware_tiebreaker")
  {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid selection_strategy '%s' -- must be 'nearest', "
      "'information_gain_revisit', 'tunnel_aware', or 'tunnel_aware_tiebreaker'",
      selection_strategy_.c_str());
    throw std::runtime_error("Invalid selection_strategy");
  }

  // Construct scorer (validates radii/weights internally).
  {
    FrontierScorerConfig scorer_cfg;
    scorer_cfg.information_gain_radius_meters =
      information_gain_radius_meters_;
    scorer_cfg.revisit_radius_meters = revisit_radius_meters_;
    scorer_cfg.max_revisit_count =
      static_cast<std::size_t>(max_revisit_count_);
    scorer_cfg.weight_information_gain = weight_information_gain_;
    scorer_cfg.weight_distance = weight_distance_;
    scorer_cfg.weight_revisit = weight_revisit_;
    scorer_cfg.weight_centerline_alignment =
      declare_parameter<double>("weight_centerline_alignment", 0.3);
    scorer_cfg.weight_wall_risk =
      declare_parameter<double>("weight_wall_risk", 0.5);
    scorer_cfg.geometry_sampling_radius_meters =
      declare_parameter<double>("geometry_sampling_radius_meters", 0.25);
    scorer_cfg.tiebreaker_epsilon =
      declare_parameter<double>("tiebreaker_epsilon", 0.10);
    scorer_ = FrontierScorer(scorer_cfg);
  }

  if (selection_strategy_ != "nearest") {
    RCLCPP_INFO(
      get_logger(),
      "Stage 2B strategy: %s (gain_w=%.1f dist_w=%.1f revisit_w=%.1f)",
      selection_strategy_.c_str(),
      weight_information_gain_, weight_distance_, weight_revisit_);
  }

  // Reconfigure detector with actual parameters.
  FrontierDetectorConfig cfg;
  cfg.min_cluster_size = static_cast<std::size_t>(min_cluster_size_);
  cfg.free_threshold = free_threshold_;
  cfg.frontier_neighbor_connectivity = frontier_neighbor_connectivity_;
  cfg.cluster_connectivity = cluster_connectivity_;
  detector_ = FrontierDetector(cfg);

  // Configure goal selector.
  FrontierGoalSelectorConfig gcfg;
  gcfg.min_goal_distance_meters = min_goal_distance_meters_;
  goal_selector_ = FrontierGoalSelector(gcfg);

  // ── Map subscription (transient_local + reliable) ────────────────────
  auto map_qos = rclcpp::QoS(1).transient_local().reliable();
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {mapCallback(msg);});

  // ── Stage 4B: tunnel geometry subscribers ──────────────────────────
  dist_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    tunnel_distance_map_topic_, rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { distanceMapCallback(msg); });
  risk_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    tunnel_risk_map_topic_, rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { riskMapCallback(msg); });

  // ── Marker publisher ─────────────────────────────────────────────────
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic_, 10);

  // ── Action client ────────────────────────────────────────────────────
  action_client_ = rclcpp_action::create_client<
    nav2_msgs::action::NavigateToPose>(this, action_server_name_);

  // ── Exploration timer ────────────────────────────────────────────────
  const auto period_ms = std::chrono::milliseconds(
    static_cast<int>(exploration_period_seconds_ * 1000.0));
  exploration_timer_ = create_wall_timer(
    period_ms,
    [this]() {explorationTimerCallback();});

  RCLCPP_INFO(get_logger(), "Started tunnel_frontier_explorer node");
}

// ── mapCallback ──────────────────────────────────────────────────────────

void TunnelFrontierExplorerNode::mapCallback(
  nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_map_ = *msg;

  if (state_ == ExplorationState::WAITING_FOR_MAP) {
    RCLCPP_INFO(get_logger(), "First map received (%ux%u, res=%.3f)",
                msg->info.width, msg->info.height, msg->info.resolution);
    transitionTo(ExplorationState::WAITING_FOR_NAV2);
  } else if (state_ == ExplorationState::COMPLETED) {
    // New map data may reveal new frontiers — re-evaluate.
    RCLCPP_INFO(get_logger(), "New map received while COMPLETED — re-evaluating");
    frontier_empty_count_ = 0;
    transitionTo(ExplorationState::IDLE);
  }
}

// ── Stage 4B: tunnel geometry callbacks ─────────────────────────────────

void TunnelFrontierExplorerNode::distanceMapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & msg)
{
  tunnel_geometry_.distance_map.width = msg->info.width;
  tunnel_geometry_.distance_map.height = msg->info.height;
  tunnel_geometry_.distance_map.resolution = msg->info.resolution;
  tunnel_geometry_.distance_map.origin_x = msg->info.origin.position.x;
  tunnel_geometry_.distance_map.origin_y = msg->info.origin.position.y;
  tunnel_geometry_.distance_map.data = msg->data;
  tunnel_geometry_last_update_ = this->now();
}

void TunnelFrontierExplorerNode::riskMapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & msg)
{
  tunnel_geometry_.risk_map.width = msg->info.width;
  tunnel_geometry_.risk_map.height = msg->info.height;
  tunnel_geometry_.risk_map.resolution = msg->info.resolution;
  tunnel_geometry_.risk_map.origin_x = msg->info.origin.position.x;
  tunnel_geometry_.risk_map.origin_y = msg->info.origin.position.y;
  tunnel_geometry_.risk_map.data = msg->data;
}

// ── applyEscapeModePenalty ────────────────────────────────────────────────

void TunnelFrontierExplorerNode::applyEscapeModePenalty(
  std::vector<ScoredGoal> & scored)
{
  if (!escape_mode_active_ || scored.empty()) {return;}

  // Check if ALL candidates are inside the oscillation radius.
  // If so, skip penalty to avoid having no viable goal.
  int outside_count = 0;
  for (const auto & sg : scored) {
    const double d = std::hypot(
      sg.cluster.representative_world.x - oscillation_center_.x,
      sg.cluster.representative_world.y - oscillation_center_.y);
    if (d >= entrance_oscillation_suppression_radius_m_) {
      outside_count++;
    }
  }
  if (outside_count == 0) {
    RCLCPP_INFO(get_logger(),
      "[OscillationEscape] all %d candidates inside radius — skipping penalty",
      static_cast<int>(scored.size()));
    return;
  }

  // Apply penalty to candidates inside the oscillation radius.
  int penalized = 0;
  for (auto & sg : scored) {
    const double d = std::hypot(
      sg.cluster.representative_world.x - oscillation_center_.x,
      sg.cluster.representative_world.y - oscillation_center_.y);
    if (d < entrance_oscillation_suppression_radius_m_) {
      sg.score -= entrance_oscillation_escape_penalty_;
      penalized++;
    }
  }

  // Re-sort by score descending (same comparator as scorer).
  std::sort(scored.begin(), scored.end(),
    [](const ScoredGoal & a, const ScoredGoal & b) {
      if (std::abs(a.score - b.score) > 1e-9) {return a.score > b.score;}
      if (std::abs(a.goal_distance_meters - b.goal_distance_meters) > 1e-9)
        {return a.goal_distance_meters < b.goal_distance_meters;}
      if (a.raw_information_gain != b.raw_information_gain)
        {return a.raw_information_gain > b.raw_information_gain;}
      if (a.cluster.representative_cell.row != b.cluster.representative_cell.row)
        {return a.cluster.representative_cell.row < b.cluster.representative_cell.row;}
      return a.cluster.representative_cell.col < b.cluster.representative_cell.col;
    });

  RCLCPP_INFO(get_logger(),
    "[OscillationEscape] active=true remaining=%d center=(%.2f,%.2f) "
    "radius=%.1f penalized=%d/%d",
    escape_mode_remaining_goals_, oscillation_center_.x, oscillation_center_.y,
    entrance_oscillation_suppression_radius_m_, penalized,
    static_cast<int>(scored.size()));
}

// ── allCandidatesInsideRadius ───────────────────────────────────────────────

bool TunnelFrontierExplorerNode::allCandidatesInsideRadius(
  const std::vector<ScoredGoal> & scored) const
{
  if (scored.empty() || !escape_mode_active_) {return false;}
  for (const auto & sg : scored) {
    const double d = std::hypot(
      sg.cluster.representative_world.x - oscillation_center_.x,
      sg.cluster.representative_world.y - oscillation_center_.y);
    if (d >= entrance_oscillation_suppression_radius_m_) {
      return false;  // at least one candidate outside
    }
  }
  return true;  // all inside
}

// ── evaluateStuckCondition ──────────────────────────────────────────────────

bool TunnelFrontierExplorerNode::evaluateStuckCondition(
  const OscillationStatus & osc_status,
  int num_valid_clusters,
  double current_goal_distance)
{
  if (!entrance_oscillation_stuck_detector_enabled_) {
    return osc_status.oscillating;
  }

  if (!osc_status.oscillating) {
    no_progress_count_ = 0;
    return false;
  }

  if (escape_cooldown_remaining_ > 0) {
    return false;
  }

  // NOT in branching decision: num_valid_clusters <= 1
  const bool in_branching_decision = (num_valid_clusters > 1);
  if (in_branching_decision) {
    return false;
  }

  // No-progress check (only within oscillillation zone)
  const double improvement = last_goal_distance_ - current_goal_distance;
  if (last_goal_distance_ > 0.0 &&
      improvement < entrance_oscillation_no_progress_threshold_m_)
  {
    no_progress_count_++;
  } else {
    no_progress_count_ = 0;
  }
  last_goal_distance_ = current_goal_distance;

  const bool no_progress =
    no_progress_count_ >= entrance_oscillation_no_progress_min_steps_;

  // Repeated revisit: at least M goals in oscillillation zone
  const bool repeated_revisit =
    static_cast<int>(entrance_oscillation_detector_.windowSize()) >=
    entrance_oscillation_repeated_revisit_min_;

  return no_progress && repeated_revisit;
}

// ── generateBootstrapProbeGoal ───────────────────────────────────────────

std::optional<Point2D> TunnelFrontierExplorerNode::generateBootstrapProbeGoal(
  const Point2D & robot)
{
  // Generate a simple forward probe goal.
  // Move forward by bootstrap_distance_m in the direction the robot is facing.
  // If no good direction found, try cardinal directions.

  if (!latest_map_.has_value()) {return std::nullopt;}

  const auto & map = *latest_map_;
  const double dist = startup_bootstrap_distance_m_;

  // Try forward, then cardinal directions
  const double yaw = getRobotYaw();
  const std::vector<double> angles = {yaw, 0.0, M_PI / 2, -M_PI / 2, M_PI};

  const auto & info = map.info;
  const double res = info.resolution;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;
  const int w = static_cast<int>(info.width);
  const int h = static_cast<int>(info.height);

  for (double angle : angles) {
    const double ux = std::cos(angle);
    const double uy = std::sin(angle);
    const Point2D probe = {robot.x + dist * ux, robot.y + dist * uy};

    // Check if point is in free space
    const int col = static_cast<int>(
      std::round((probe.x - ox) / res - 0.5));
    const int row = static_cast<int>(
      std::round((probe.y - oy) / res - 0.5));

    if (row < 0 || row >= h || col < 0 || col >= w) {
      continue;
    }

    const std::size_t idx = static_cast<std::size_t>(row) * w +
                            static_cast<std::size_t>(col);
    if (map.data[idx] == 0) {
      return probe;
    }
  }

  return std::nullopt;
}

// ── explorationTimerCallback ─────────────────────────────────────────────

void TunnelFrontierExplorerNode::explorationTimerCallback()
{
  switch (state_) {
    case ExplorationState::WAITING_FOR_MAP:
      // Silently waiting for first map.
      return;

    case ExplorationState::WAITING_FOR_NAV2: {
      // Non-blocking check for Nav2 action server availability.
        if (action_client_->wait_for_action_server(std::chrono::seconds(0))) {
          RCLCPP_INFO(get_logger(), "Nav2 action server is available");
          transitionTo(ExplorationState::IDLE);
        } else {
          RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for Nav2 action server '%s'...",
          action_server_name_.c_str());
        }
        return;
      }

    case ExplorationState::IDLE: {
        if (!latest_map_) {
          return;
        }

      // Wrap latest map into GridMap.
        const auto & info = latest_map_->info;
        GridMap gm;
        gm.width = info.width;
        gm.height = info.height;
        gm.resolution = info.resolution;
        gm.origin_x = info.origin.position.x;
        gm.origin_y = info.origin.position.y;
        gm.data = latest_map_->data;

        if (!gm.valid()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Invalid map skipped");
          return;
        }

      // Get robot pose.
        Point2D robot_pose;
        if (!getRobotPose(robot_pose)) {
          return;
        }

      // Detect frontier clusters.
        std::vector<FrontierCluster> clusters;
        try {
          clusters = detector_.detect(gm, robot_pose);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "Frontier detection: %s", e.what());
          return;
        }

      // Filter blacklisted clusters (with 4D.2 starvation recovery).
        const auto now = std::chrono::steady_clock::now();
        blacklist_.cleanup(now);

        std::vector<FrontierCluster> valid;
        std::vector<FrontierCluster> suppressed;
        std::vector<Point2D> blacklisted_positions;
        for (const auto & c : clusters) {
          bool is_recovery_target = false;
          if (cooldown_starvation_recovery_active_) {
            const double d = std::hypot(
              c.representative_world.x - cooldown_recovery_target_.x,
              c.representative_world.y - cooldown_recovery_target_.y);
            is_recovery_target =
              d < cooldown_starvation_recovery_match_tolerance_m_;
          }
          if (!is_recovery_target &&
              blacklist_.contains(c.representative_world, now))
          {
            suppressed.push_back(c);
            blacklisted_positions.push_back(c.representative_world);
          } else {
            valid.push_back(c);
          }
        }

        const bool all_suppressed =
          valid.empty() && !clusters.empty() &&
          suppressed.size() == clusters.size();

        if (all_suppressed) {
          ++all_suppressed_count_;
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
          "All %zu frontiers suppressed by cooldown (%zu/%zu cycles)",
          clusters.size(), all_suppressed_count_,
          k_max_all_suppressed_cycles_);
          frontier_empty_count_ = 0;

          // 4D.2: activate starvation recovery
          if (cooldown_starvation_recovery_enabled_ &&
              !cooldown_starvation_recovery_active_ &&
              all_suppressed_count_ >=
                cooldown_starvation_recovery_threshold_)
          {
            // Find best suppressed frontier by info gain
            int best_idx = -1;
            double best_gain = -1.0;
            for (std::size_t i = 0; i < suppressed.size(); ++i) {
              const double gain =
                static_cast<double>(suppressed[i].cells.size());
              if (gain > best_gain) {
                best_gain = gain;
                best_idx = static_cast<int>(i);
              }
            }
            if (best_idx >= 0) {
              cooldown_starvation_recovery_active_ = true;
              cooldown_recovery_target_ =
                suppressed[best_idx].representative_world;
              all_suppressed_count_ = 0;
              RCLCPP_INFO(get_logger(),
                "[CooldownRecovery] activated target=(%.2f,%.2f) "
                "gain=%.0f",
                cooldown_recovery_target_.x,
                cooldown_recovery_target_.y, best_gain);
            }
          }

          if (all_suppressed_count_ >= k_max_all_suppressed_cycles_) {
            RCLCPP_INFO(get_logger(),
              "No viable frontiers after suppression for %zu cycles — "
              "exploration complete", k_max_all_suppressed_cycles_);
            transitionTo(ExplorationState::COMPLETED);
          }
        } else {
          ++frontier_empty_count_;
          all_suppressed_count_ = 0;
          // Deactivate recovery if active
          if (cooldown_starvation_recovery_active_) {
            cooldown_starvation_recovery_active_ = false;
            RCLCPP_INFO(get_logger(),
              "[CooldownRecovery] deactivated — valid frontier found");
          }
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
          "No frontiers (%zu/%zu empty cycles)",
          frontier_empty_count_, k_max_empty_cycles_);
          if (frontier_empty_count_ >= k_max_empty_cycles_) {
            // Stage 4E.2: startup bootstrap — don't declare completion
            // if no goals have been dispatched yet
            if (startup_bootstrap_enabled_ &&
                goals_dispatched_ == 0 &&
                startup_bootstrap_attempts_ < startup_bootstrap_max_attempts_)
            {
              RCLCPP_INFO(get_logger(),
                "[StartupBootstrap] no frontiers, goals=0 — "
                "attempting bootstrap probe");
              startup_bootstrap_attempts_++;
              auto bootstrap_goal = generateBootstrapProbeGoal(robot_pose);
              if (bootstrap_goal.has_value()) {
                // Dispatch bootstrap probe goal
                auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
                goal_msg.pose.header.frame_id = global_frame_;
                goal_msg.pose.header.stamp = this->now();
                goal_msg.pose.pose.position.x = bootstrap_goal->x;
                goal_msg.pose.pose.position.y = bootstrap_goal->y;
                goal_msg.pose.pose.position.z = 0.0;
                goal_msg.pose.pose.orientation.w = 1.0;

                auto send_opts = rclcpp_action::Client<
                  nav2_msgs::action::NavigateToPose>::SendGoalOptions();
                send_opts.goal_response_callback =
                  [this](const GoalHandle::SharedPtr & h) {goalResponseCallback(h);};
                send_opts.result_callback =
                  [this](const GoalHandle::WrappedResult & r) {resultCallback(r);};

                current_goal_ = *bootstrap_goal;
                navigating_start_time_ = this->now();
                action_client_->async_send_goal(goal_msg, send_opts);
                goals_dispatched_++;
                transitionTo(ExplorationState::NAVIGATING);
                frontier_empty_count_ = 0;
                RCLCPP_INFO(get_logger(),
                  "[StartupBootstrap] dispatched probe to (%.2f, %.2f)",
                  bootstrap_goal->x, bootstrap_goal->y);
                return;
              } else {
                RCLCPP_INFO(get_logger(),
                  "[StartupBootstrap] no valid probe found — "
                  "falling back to completion");
              }
            }
            RCLCPP_INFO(get_logger(), "No frontiers for %zu cycles — "
                        "exploration complete", k_max_empty_cycles_);
            transitionTo(ExplorationState::COMPLETED);
          }
        }
        publishMarkers(clusters, blacklisted_positions, std::nullopt, {});
        return;

      // ── Stage 4D.1: oscillillation evaluation with stuck gating ───
      OscillationStatus osc_status;
      if (entrance_oscillation_enabled_) {
        osc_status = entrance_oscillation_detector_.evaluate();
        if (entrance_oscillation_response_enabled_ && !escape_mode_active_)
        {
          const double dist_to_center = std::hypot(
            robot_pose.x - oscillation_center_.x,
            robot_pose.y - oscillation_center_.y);
          const bool stuck = evaluateStuckCondition(
            osc_status, static_cast<int>(valid.size()), dist_to_center);
          if (stuck) {
            escape_mode_active_ = true;
            escape_goals_dispatched_ = 0;
            no_oscillation_count_ = 0;
            oscillation_center_ =
              entrance_oscillation_detector_.getOscillationCenter();
            RCLCPP_INFO(get_logger(),
              "[OscillationEscape] activated center=(%.2f,%.2f) "
              "radius=%.2fm",
              oscillation_center_.x, oscillation_center_.y,
              entrance_oscillation_suppression_radius_m_);
          }
        }
      }

      // ── Stage 3D: entrance-loop recovery ──────────────────────
      // If recent goal history shows local oscillation (≤2 unique
      // bins with ≥3 recent successes), skip normal frontier
      // selection and send a forward recovery probe instead.
      if (loop_detection_enabled_ && detectLocalLoop()) {
        ++loop_detected_count_;

        const auto now = this->now();
        const double since_last_probe =
          (now - recovery_probe_last_time_).seconds();
        const bool in_cooldown =
          recovery_probe_cooldown_seconds_ > 0.0 &&
          since_last_probe < recovery_probe_cooldown_seconds_;

        if (!in_cooldown) {
          const double robot_yaw = getRobotYaw();
          auto probe = generateRecoveryProbe(
            robot_pose, robot_yaw, *latest_map_);

          if (probe) {
            ++recovery_probe_count_;
            recovery_probe_last_time_ = now;
            ++recovery_attempt_count_;
            current_goal_is_recovery_ = true;

            RCLCPP_INFO(get_logger(),
              "Recovery probe goal: (%.2f, %.2f) d=%.2f a=%.1fdeg "
              "(loop bins≤%d succ≥%d attempt %d/%d)",
              probe->x, probe->y,
              std::hypot(probe->x - robot_pose.x,
                         probe->y - robot_pose.y),
              std::atan2(probe->y - robot_pose.y,
                         probe->x - robot_pose.x) * 180.0 / M_PI,
              loop_unique_bins_threshold_, loop_min_successes_,
              recovery_attempt_count_, recovery_max_attempts_);

            // Dispatch recovery probe — skip frontier selection & projection.
            auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
            goal_msg.pose.header.frame_id = global_frame_;
            goal_msg.pose.header.stamp = this->now();
            goal_msg.pose.pose.position.x = probe->x;
            goal_msg.pose.pose.position.y = probe->y;
            goal_msg.pose.pose.position.z = 0.0;
            const double goal_yaw = std::atan2(
              probe->y - robot_pose.y, probe->x - robot_pose.x);
            goal_msg.pose.pose.orientation.z = std::sin(goal_yaw * 0.5);
            goal_msg.pose.pose.orientation.w = std::cos(goal_yaw * 0.5);

            current_goal_ = *probe;
            navigating_start_time_ = this->now();

            if (entrance_oscillation_enabled_) {
              const int selected_bin = static_cast<int>(
                std::floor(probe->x / loop_bin_size_));
              const int unique_bins =
                entrance_oscillation_detector_.getCurrentUniqueBins();
              const std::size_t raw_revisit =
                visit_history_.countVisitsNear(
                  *probe, revisit_radius_meters_);
              const double revisit_ratio =
                static_cast<double>(raw_revisit) /
                static_cast<double>(max_revisit_count_);

              GoalDispatchEvent osc_event;
              osc_event.goal = *probe;
              osc_event.robot = robot_pose;
              osc_event.stamp_seconds = this->now().seconds();
              osc_event.selected_bin = selected_bin;
              osc_event.revisit_ratio = revisit_ratio;
              osc_event.unique_bins = unique_bins;
              entrance_oscillation_detector_.recordGoal(osc_event);

              auto osc_status = entrance_oscillation_detector_.evaluate();
              if (osc_status.oscillating) {
                RCLCPP_WARN(get_logger(),
                  "[EntranceOscillation] detected=true count=%d "
                  "window=%zu radius=%.2fm unique_bins=%d "
                  "revisit=%.2f reason=%s (recovery probe)",
                  osc_status.event_count,
                  entrance_oscillation_detector_.windowSize(),
                  osc_status.spatial_radius_m,
                  unique_bins,
                  revisit_ratio,
                  osc_status.reason.c_str());
              }
            }

            auto send_opts = rclcpp_action::Client<
              nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            send_opts.goal_response_callback =
              [this](const GoalHandle::SharedPtr & h) {goalResponseCallback(h);};
            send_opts.feedback_callback =
              [this](GoalHandle::SharedPtr handle,
               const std::shared_ptr<
                 const nav2_msgs::action::NavigateToPose::Feedback> & fb)
              {feedbackCallback(handle, fb);};
            send_opts.result_callback =
              [this](const GoalHandle::WrappedResult & r) {resultCallback(r);};

            action_client_->async_send_goal(goal_msg, send_opts);
            goals_dispatched_++;
            transitionTo(ExplorationState::NAVIGATING);

            // Markers: show recovery probe as cyan sphere.
            publishMarkers(
              clusters, blacklisted_positions, *probe, {});
            {
              visualization_msgs::msg::Marker m;
              m.header.frame_id = global_frame_;
              m.header.stamp = this->now();
              m.ns = "recovery_probe";
              m.id = 0;
              m.type = visualization_msgs::msg::Marker::SPHERE;
              m.action = visualization_msgs::msg::Marker::ADD;
              m.pose.position.x = probe->x;
              m.pose.position.y = probe->y;
              m.pose.position.z = 0.0;
              m.pose.orientation.w = 1.0;
              m.scale.x = 0.30; m.scale.y = 0.30; m.scale.z = 0.30;
              m.color.a = 0.9;
              m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0;
              visualization_msgs::msg::MarkerArray arr;
              arr.markers.push_back(std::move(m));
              marker_pub_->publish(std::move(arr));
            }
            return;
          }

          // No safe probe found.
          recovery_probe_last_time_ = now;
          ++recovery_attempt_count_;

          if (recovery_attempt_count_ >= recovery_max_attempts_) {
            RCLCPP_INFO(get_logger(),
              "Exploration stalled: local loop detected, "
              "no safe recovery probe after %d attempts",
              recovery_attempt_count_);
            transitionTo(ExplorationState::STALLED);
            return;
          }

          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
            "Local loop detected — no safe recovery probe "
            "(attempt %d/%d), waiting for map update",
            recovery_attempt_count_, recovery_max_attempts_);
          return;
        }
        // In cooldown — do nothing this cycle.
        return;
      }
      // No loop detected or detection disabled — reset recovery state.
      resetRecoveryState();

      // Select best goal — strategy-dependent.
        Point2D goal_pt;
        std::optional<Point2D> selected_for_marker;
        std::vector<Point2D> too_close_for_marker;
        std::vector<ScoredGoal> scored_frontiers;

        if (selection_strategy_ == "information_gain_revisit") {
          // Phase 1: distance filtering via selectAll()
          AllCandidatesResult all_sel = goal_selector_.selectAll(
            valid, gm, robot_pose);
          too_close_for_marker = all_sel.too_close_goals;

          if (all_sel.candidates.empty()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
              "All %zu frontiers within min_goal_distance=%.2f m "
              "— waiting for map update",
              valid.size(), min_goal_distance_meters_);
            publishMarkers(
              clusters, blacklisted_positions, std::nullopt,
              too_close_for_marker);
            return;
          }

          // Phase 2: second blacklist check on final representative_world.
          const auto steady_now = std::chrono::steady_clock::now();
          std::vector<FrontierCluster> post_blacklist;
          for (const auto & c : all_sel.candidates) {
            if (!blacklist_.contains(c.representative_world, steady_now)) {
              post_blacklist.push_back(c);
            }
          }

          if (post_blacklist.empty()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
              "All %zu distance-candidates blacklisted after selectAll()"
              " — waiting for expiry",
              all_sel.candidates.size());
            publishMarkers(
              clusters, blacklisted_positions, std::nullopt,
              too_close_for_marker);
            return;
          }

          // Phase 3: score and rank.
          scored_frontiers = scorer_.scoreAndRank(
            post_blacklist, gm, robot_pose, visit_history_);
          applyEscapeModePenalty(scored_frontiers);
          const auto & best = scored_frontiers.front();
          goal_pt = best.cluster.representative_world;
          selected_for_marker = goal_pt;

          // Stage 4C.1: forced escape probe when all candidates inside radius
          if (escape_mode_active_ && allCandidatesInsideRadius(scored_frontiers) &&
              forced_probe_cooldown_remaining_ == 0 &&
              entrance_oscillation_force_escape_probe_)
          {
            auto probe = probe_generator_.generateEscapeProbe(
              robot_pose, oscillation_center_, last_selected_goal_,
              gm, entrance_oscillation_suppression_radius_m_);
            if (probe) {
              goal_pt = *probe;
              selected_for_marker = goal_pt;
              forced_probe_cooldown_remaining_ =
                entrance_oscillation_probe_cooldown_goals_;
              RCLCPP_INFO(get_logger(),
                "[ForcedEscapeProbe] triggered=true valid=true "
                "center=(%.2f,%.2f) selected=(%.2f,%.2f) cooldown=%d",
                oscillation_center_.x, oscillation_center_.y,
                goal_pt.x, goal_pt.y,
                forced_probe_cooldown_remaining_);
            } else {
              RCLCPP_INFO(get_logger(),
                "[ForcedEscapeProbe] triggered=true valid=false "
                "reason=no_safe_probe fallback=original_goal");
            }
          }

          last_selected_goal_ = goal_pt;

          RCLCPP_INFO(get_logger(),
            "Goal: (%.2f, %.2f) dist=%.2f "
            "gain=%zu(raw)/%.2f(tr)/%.3f(norm) "
            "revisit=%zu(raw)/%zu(cl)/%.3f(norm) "
            "score=%.4f [%zu cand]",
            goal_pt.x, goal_pt.y,
            best.goal_distance_meters,
            best.raw_information_gain,
            best.transformed_information_gain,
            best.normalized_information_gain,
            best.raw_revisit_count,
            best.clamped_revisit_count,
            best.normalized_revisit_penalty,
            best.score,
            scored_frontiers.size());
        } else if (selection_strategy_ == "tunnel_aware") {
          // Stage 4B: tunnel-aware scoring with geometry fallback.
          // Geometry may not be available for the first goal (centerline
          // takes ~110 s to process the first map).  Falls back to Stage 3D
          // scoring for the initial cycle; subsequent goals use geometry.
          AllCandidatesResult all_sel = goal_selector_.selectAll(
            valid, gm, robot_pose);
          too_close_for_marker = all_sel.too_close_goals;

          if (all_sel.candidates.empty()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
              "All %zu frontiers within min_goal_distance=%.2f m "
              "— waiting for map update",
              valid.size(), min_goal_distance_meters_);
            publishMarkers(
              clusters, blacklisted_positions, std::nullopt,
              too_close_for_marker);
            return;
          }

          // Phase 2: second blacklist check.
          const auto steady_now = std::chrono::steady_clock::now();
          std::vector<FrontierCluster> post_blacklist;
          for (const auto & c : all_sel.candidates) {
            if (!blacklist_.contains(c.representative_world, steady_now)) {
              post_blacklist.push_back(c);
            }
          }
          if (post_blacklist.empty()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
              "All %zu distance-candidates blacklisted — waiting",
              all_sel.candidates.size());
            publishMarkers(
              clusters, blacklisted_positions, std::nullopt,
              too_close_for_marker);
            return;
          }

          // Phase 3: tunnel-aware or fallback scoring.
          bool geometry_valid = tunnel_geometry_.valid();
          const double age =
            (this->now() - tunnel_geometry_last_update_).seconds();
          if (age > tunnel_geometry_max_age_seconds_) geometry_valid = false;

          if (geometry_valid) {
            scored_frontiers = scorer_.scoreAndRankTunnelAware(
              post_blacklist, gm, robot_pose, visit_history_,
              tunnel_geometry_);
          } else if (geometry_missing_fallback_to_stage3d_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
              "Tunnel geometry not available — falling back to Stage 3D scoring");
            scored_frontiers = scorer_.scoreAndRank(
              post_blacklist, gm, robot_pose, visit_history_);
          } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
              "Waiting for tunnel geometry maps...");
            return;
          }

          applyEscapeModePenalty(scored_frontiers);
          const auto & best = scored_frontiers.front();
          goal_pt = best.cluster.representative_world;
          selected_for_marker = goal_pt;

          // Stage 4C.1: forced escape probe when all candidates inside radius
          if (escape_mode_active_ && allCandidatesInsideRadius(scored_frontiers) &&
              forced_probe_cooldown_remaining_ == 0 &&
              entrance_oscillation_force_escape_probe_)
          {
            auto probe = probe_generator_.generateEscapeProbe(
              robot_pose, oscillation_center_, last_selected_goal_,
              gm, entrance_oscillation_suppression_radius_m_);
            if (probe) {
              goal_pt = *probe;
              selected_for_marker = goal_pt;
              forced_probe_cooldown_remaining_ =
                entrance_oscillation_probe_cooldown_goals_;
              RCLCPP_INFO(get_logger(),
                "[ForcedEscapeProbe] triggered=true valid=true "
                "center=(%.2f,%.2f) selected=(%.2f,%.2f) cooldown=%d",
                oscillation_center_.x, oscillation_center_.y,
                goal_pt.x, goal_pt.y,
                forced_probe_cooldown_remaining_);
            } else {
              RCLCPP_INFO(get_logger(),
                "[ForcedEscapeProbe] triggered=true valid=false "
                "reason=no_safe_probe fallback=original_goal");
            }
          }

          last_selected_goal_ = goal_pt;

          RCLCPP_INFO(get_logger(),
            "Goal: (%.2f, %.2f) dist=%.2f "
            "gain=%zu(raw)/%.2f(tr)/%.3f(norm) "
            "revisit=%zu(raw)/%zu(cl)/%.3f(norm) "
            "align=%.3f(norm) risk=%.3f(norm) "
            "score=%.4f [%zu cand]",
            goal_pt.x, goal_pt.y,
            best.goal_distance_meters,
            best.raw_information_gain,
            best.transformed_information_gain,
            best.normalized_information_gain,
            best.raw_revisit_count,
            best.clamped_revisit_count,
            best.normalized_revisit_penalty,
            best.normalized_centerline_alignment,
            best.normalized_wall_risk,
            best.score,
            scored_frontiers.size());
        } else if (selection_strategy_ == "tunnel_aware_tiebreaker") {
          // Stage 4B.1: Stage 3D scoring + wall-risk tiebreaker.
          AllCandidatesResult all_sel = goal_selector_.selectAll(
            valid, gm, robot_pose);
          too_close_for_marker = all_sel.too_close_goals;

          if (all_sel.candidates.empty()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
              "All %zu frontiers within min_goal_distance=%.2f m "
              "— waiting for map update",
              valid.size(), min_goal_distance_meters_);
            publishMarkers(
              clusters, blacklisted_positions, std::nullopt,
              too_close_for_marker);
            return;
          }

          const auto steady_now = std::chrono::steady_clock::now();
          std::vector<FrontierCluster> post_blacklist;
          for (const auto & c : all_sel.candidates) {
            if (!blacklist_.contains(c.representative_world, steady_now)) {
              post_blacklist.push_back(c);
            }
          }
          if (post_blacklist.empty()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
              "All %zu distance-candidates blacklisted — waiting",
              all_sel.candidates.size());
            publishMarkers(
              clusters, blacklisted_positions, std::nullopt,
              too_close_for_marker);
            return;
          }

          // Phase 3: Stage 3D scoring, then apply geometry tiebreaker.
          scored_frontiers = scorer_.scoreAndRank(
            post_blacklist, gm, robot_pose, visit_history_);

          bool geometry_valid = tunnel_geometry_.valid();
          if (geometry_valid) {
            double age =
              (this->now() - tunnel_geometry_last_update_).seconds();
            if (age > tunnel_geometry_max_age_seconds_)
              geometry_valid = false;
          }
          if (geometry_valid) {
            scorer_.applyTunnelRiskTiebreaker(
              scored_frontiers, tunnel_geometry_);
          }

          applyEscapeModePenalty(scored_frontiers);
          const auto & best = scored_frontiers.front();
          goal_pt = best.cluster.representative_world;
          selected_for_marker = goal_pt;

          // Stage 4C.1: forced escape probe when all candidates inside radius
          if (escape_mode_active_ && allCandidatesInsideRadius(scored_frontiers) &&
              forced_probe_cooldown_remaining_ == 0 &&
              entrance_oscillation_force_escape_probe_)
          {
            auto probe = probe_generator_.generateEscapeProbe(
              robot_pose, oscillation_center_, last_selected_goal_,
              gm, entrance_oscillation_suppression_radius_m_);
            if (probe) {
              goal_pt = *probe;
              selected_for_marker = goal_pt;
              forced_probe_cooldown_remaining_ =
                entrance_oscillation_probe_cooldown_goals_;
              RCLCPP_INFO(get_logger(),
                "[ForcedEscapeProbe] triggered=true valid=true "
                "center=(%.2f,%.2f) selected=(%.2f,%.2f) cooldown=%d",
                oscillation_center_.x, oscillation_center_.y,
                goal_pt.x, goal_pt.y,
                forced_probe_cooldown_remaining_);
            } else {
              RCLCPP_INFO(get_logger(),
                "[ForcedEscapeProbe] triggered=true valid=false "
                "reason=no_safe_probe fallback=original_goal");
            }
          }

          last_selected_goal_ = goal_pt;

          RCLCPP_INFO(get_logger(),
            "Goal: (%.2f, %.2f) dist=%.2f "
            "gain=%zu(raw)/%.2f(tr)/%.3f(norm) "
            "revisit=%zu(raw)/%zu(cl)/%.3f(norm) "
            "risk=%.3f(raw) score=%.4f [%zu cand]%s",
            goal_pt.x, goal_pt.y,
            best.goal_distance_meters,
            best.raw_information_gain,
            best.transformed_information_gain,
            best.normalized_information_gain,
            best.raw_revisit_count,
            best.clamped_revisit_count,
            best.normalized_revisit_penalty,
            best.normalized_wall_risk,
            best.score,
            scored_frontiers.size(),
            geometry_valid ? " +tiebreak" : "");
        } else {
          // nearest strategy (Stage 2A baseline, unchanged).
          GoalSelectionResult sel = goal_selector_.select(
            valid, gm, robot_pose);

          if (!sel.selected) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
              "All %zu frontiers within min_goal_distance=%.2f m "
              "— waiting for map update",
              valid.size(), min_goal_distance_meters_);
            publishMarkers(
              clusters, blacklisted_positions, std::nullopt,
              sel.too_close_goals);
            return;
          }

          const auto & target = *sel.selected;
          goal_pt = target.representative_world;
          selected_for_marker = goal_pt;
          too_close_for_marker = sel.too_close_goals;

          RCLCPP_INFO(get_logger(),
            "Goal: (%.2f, %.2f) [%zu cells, centroid=%.1f m, goal=%.1f m]",
            goal_pt.x, goal_pt.y, target.size(),
            target.centroid_distance_to_robot,
            target.goal_distance_to_robot);
        }

      // ── Goal safety projection ─────────────────────────────────
      // Pull the selected frontier point inward toward the robot so
      // Nav2 doesn't try to converge on the exact frontier/unknown
      // boundary where inflation or unknown cells cause timeout.
      Point2D safe_goal_pt = goal_pt;
      std::optional<Point2D> projected_goal_for_marker = std::nullopt;
      if (goal_projection_enabled_ && latest_map_) {
        auto projected = projectGoalTowardRobot(
          goal_pt, robot_pose, *latest_map_);
        if (projected) {
          safe_goal_pt = *projected;
          projected_goal_for_marker = safe_goal_pt;
          const double dist_to_orig = std::hypot(
            goal_pt.x - robot_pose.x, goal_pt.y - robot_pose.y);
          const double projection_offset = std::hypot(
            goal_pt.x - safe_goal_pt.x, goal_pt.y - safe_goal_pt.y);
          RCLCPP_INFO(get_logger(),
            "Projected goal: (%.2f, %.2f) dist=%.2f "
            "→ safe (%.2f, %.2f) offset=%.2f",
            goal_pt.x, goal_pt.y, dist_to_orig,
            safe_goal_pt.x, safe_goal_pt.y, projection_offset);
        } else {
          RCLCPP_DEBUG(get_logger(),
            "Goal projection skipped for (%.2f, %.2f) — using original",
            goal_pt.x, goal_pt.y);
        }
      }

      // Prepare NavigateToPose goal using the (possibly projected) safe point.
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = global_frame_;
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = safe_goal_pt.x;
        goal_msg.pose.pose.position.y = safe_goal_pt.y;
        goal_msg.pose.pose.position.z = 0.0;

        if (orient_goal_toward_frontier_) {
          const double yaw = std::atan2(
          safe_goal_pt.y - robot_pose.y, safe_goal_pt.x - robot_pose.x);
          goal_msg.pose.pose.orientation.z = std::sin(yaw * 0.5);
          goal_msg.pose.pose.orientation.w = std::cos(yaw * 0.5);
        } else {
          goal_msg.pose.pose.orientation.w = 1.0;
        }

      // Store for blacklisting on failure.
        current_goal_ = goal_pt;
        navigating_start_time_ = this->now();

      // Send goal.
        if (entrance_oscillation_enabled_) {
          const int selected_bin = static_cast<int>(
            std::floor(goal_pt.x / loop_bin_size_));
          const int unique_bins =
            entrance_oscillation_detector_.getCurrentUniqueBins();
          const std::size_t raw_revisit =
            visit_history_.countVisitsNear(
              goal_pt, revisit_radius_meters_);
          const double revisit_ratio =
            static_cast<double>(raw_revisit) /
            static_cast<double>(max_revisit_count_);

          GoalDispatchEvent osc_event;
          osc_event.goal = goal_pt;
          osc_event.robot = robot_pose;
          osc_event.stamp_seconds = this->now().seconds();
          osc_event.selected_bin = selected_bin;
          osc_event.revisit_ratio = revisit_ratio;
          osc_event.unique_bins = unique_bins;
          entrance_oscillation_detector_.recordGoal(osc_event);

          // Log oscillation status (use early osc_status to avoid double evaluate)
          if (osc_status.oscillating) {
            RCLCPP_WARN(get_logger(),
              "[EntranceOscillation] detected=true count=%d "
              "window=%zu radius=%.2fm unique_bins=%d "
              "revisit=%.2f reason=%s",
              osc_status.event_count,
              entrance_oscillation_detector_.windowSize(),
              osc_status.spatial_radius_m,
              unique_bins,
              revisit_ratio,
              osc_status.reason.c_str());
          }
        }

        auto send_opts = rclcpp_action::Client<
          nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_opts.goal_response_callback =
          [this](const GoalHandle::SharedPtr & h) {goalResponseCallback(h);};
        send_opts.feedback_callback =
          [this](
          GoalHandle::SharedPtr handle,
          const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>
          & fb) {feedbackCallback(handle, fb);};
        send_opts.result_callback =
          [this](const GoalHandle::WrappedResult & r) {resultCallback(r);};

        action_client_->async_send_goal(goal_msg, send_opts);
        goals_dispatched_++;
        transitionTo(ExplorationState::NAVIGATING);

        // Stage 4D.1: condition-based escape deactivation
        if (escape_mode_active_) {
          escape_goals_dispatched_++;

          // Check no-oscillation counter (updated from early evaluation)
          if (!osc_status.oscillating) {
            no_oscillation_count_++;
          } else {
            no_oscillation_count_ = 0;
          }

          bool should_deactivate = false;

          // Max goals reached
          if (escape_goals_dispatched_ >= entrance_oscillation_escape_max_goals_) {
            should_deactivate = true;
            RCLCPP_INFO(get_logger(),
              "[OscillationEscape] deactivated: max_goals reached (%d)",
              escape_goals_dispatched_);
          }

          // Robot moved outside oscillillation radius
          const double dist_to_center = std::hypot(
            robot_pose.x - oscillation_center_.x,
            robot_pose.y - oscillation_center_.y);
          if (dist_to_center > entrance_oscillation_escape_deactivation_distance_m_) {
            should_deactivate = true;
            RCLCPP_INFO(get_logger(),
              "[OscillationEscape] deactivated: outside radius (%.2fm > %.2fm)",
              dist_to_center, entrance_oscillation_escape_deactivation_distance_m_);
          }

          // No oscillillation for N consecutive goals
          if (no_oscillation_count_ >= entrance_oscillation_escape_no_oscillation_deactivate_goals_) {
            should_deactivate = true;
            RCLCPP_INFO(get_logger(),
              "[OscillationEscape] deactivated: no oscillillation for %d goals",
              no_oscillation_count_);
          }

          if (should_deactivate) {
            escape_mode_active_ = false;
            forced_probe_cooldown_remaining_ = 0;
            escape_cooldown_remaining_ = entrance_oscillation_cooldown_goals_;
            RCLCPP_INFO(get_logger(),
              "[OscillationEscape] cooldown=%d goals",
              escape_cooldown_remaining_);
          }
        }

        // Stage 4C.1: decrement forced probe cooldown
        if (forced_probe_cooldown_remaining_ > 0) {
          forced_probe_cooldown_remaining_--;
        }

        // Stage 4D.1: decrement escape cooldown
        if (escape_cooldown_remaining_ > 0) {
          escape_cooldown_remaining_--;
        }

        publishMarkers(
          clusters, blacklisted_positions, selected_for_marker,
          too_close_for_marker);

        // ── Projected goal marker (blue sphere) ──────────────────
        if (projected_goal_for_marker) {
          visualization_msgs::msg::Marker m;
          m.header.frame_id = global_frame_;
          m.header.stamp = this->now();
          m.ns = "projected_goal";
          m.id = 0;
          m.type = visualization_msgs::msg::Marker::SPHERE;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.pose.position.x = projected_goal_for_marker->x;
          m.pose.position.y = projected_goal_for_marker->y;
          m.pose.position.z = 0.0;
          m.pose.orientation.w = 1.0;
          m.scale.x = 0.25;
          m.scale.y = 0.25;
          m.scale.z = 0.25;
          m.color.a = 0.9;
          m.color.r = 0.0;
          m.color.g = 0.5;
          m.color.b = 1.0;
          visualization_msgs::msg::MarkerArray arr;
          arr.markers.push_back(m);
          marker_pub_->publish(std::move(arr));
        }

        if (!scored_frontiers.empty()) {
          publishScoredFrontierMarkers(scored_frontiers);
        }
        return;
      }

    case ExplorationState::NAVIGATING: {
        if (goal_timeout_seconds_ > 0.0 && current_goal_) {
          const double elapsed = (this->now() - navigating_start_time_).seconds();
          if (elapsed >= goal_timeout_seconds_) {
            RCLCPP_WARN(
              get_logger(), "Goal timed out after %.1f s — cancelling", elapsed);
            // ── Stage 3D recovery tracking ───────────────────────────
            if (current_goal_is_recovery_) {
              ++recovery_failure_count_;
            }
            // ── Goal bin tracking ─────────────────────────────────────
            if (current_goal_) {
              recordGoalBin(*current_goal_, false);
            }
            // Blacklist before cancelling to prevent re-selection
            // (frontier goals only — not recovery probes).
            if (!current_goal_is_recovery_) {
              const auto t = std::chrono::steady_clock::now();
              blacklist_.add(
                *current_goal_, t, blacklist_radius_,
                std::chrono::duration_cast<std::chrono::seconds>(
                  std::chrono::duration<double>(blacklist_timeout_seconds_)));
              // 4D.2: deactivate recovery on goal terminal
              if (cooldown_starvation_recovery_active_) {
                cooldown_starvation_recovery_active_ = false;
                all_suppressed_count_ = 0;
                RCLCPP_INFO(get_logger(),
                  "[CooldownRecovery] deactivated on timeout");
              }
              RCLCPP_INFO(
                get_logger(), "Blacklisted (%.2f, %.2f) for %.0f s",
                current_goal_->x, current_goal_->y, blacklist_timeout_seconds_);
            }
            if (current_goal_handle_) {
              action_client_->async_cancel_goal(current_goal_handle_);
            }
            transitionTo(ExplorationState::COOLDOWN);
          }
        }
        return;
      }

    case ExplorationState::COOLDOWN: {
        const double elapsed = (this->now() - cooldown_start_).seconds();
        if (elapsed >= cooldown_seconds_) {
          transitionTo(ExplorationState::IDLE);
        }
        return;
      }

    case ExplorationState::COMPLETED:
    case ExplorationState::STALLED:
      return;
  }
}

// ── goalResponseCallback ─────────────────────────────────────────────────

void TunnelFrontierExplorerNode::goalResponseCallback(
  const GoalHandle::SharedPtr & handle)
{
  if (!handle) {
    RCLCPP_WARN(get_logger(), "Goal rejected by Nav2 server");
    current_goal_ = std::nullopt;
    transitionTo(ExplorationState::IDLE);
    return;
  }
  current_goal_handle_ = handle;
  RCLCPP_INFO(get_logger(), "Goal accepted by Nav2 server");

  // Record in visit history for revisit penalty (info_revisit strategy only).
  if (selection_strategy_ == "information_gain_revisit" && current_goal_) {
    visit_history_.recordAcceptedGoal(*current_goal_);
  }
}

// ── feedbackCallback ─────────────────────────────────────────────────────

void TunnelFrontierExplorerNode::feedbackCallback(
  GoalHandle::SharedPtr,
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>
  feedback)
{
  const double nav_time = feedback->navigation_time.sec +
    feedback->navigation_time.nanosec / 1.0e9;
  const double est_remaining = feedback->estimated_time_remaining.sec +
    feedback->estimated_time_remaining.nanosec / 1.0e9;
  RCLCPP_DEBUG(get_logger(),
    "nav_time=%.1f remaining=%.1f recoveries=%d dist=%.2f",
    nav_time, est_remaining,
    feedback->number_of_recoveries,
    feedback->distance_remaining);
}

// ── resultCallback ───────────────────────────────────────────────────────

void TunnelFrontierExplorerNode::resultCallback(
  const GoalHandle::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Navigation to goal succeeded");

      // ── Stage 3D recovery metrics ─────────────────────────────
      if (current_goal_is_recovery_) {
        ++recovery_success_count_;
        RCLCPP_INFO(get_logger(),
          "Recovery probe succeeded (%d/%d probes succeeded)",
          recovery_success_count_, recovery_probe_count_);
      }

      // ── Goal bin tracking ─────────────────────────────────────
      if (current_goal_) {
        recordGoalBin(*current_goal_, true);
      }

      // Temporarily blacklist the original frontier point on success
      // to prevent repeatedly re-selecting the same frontier before
      // new map data reveals fresh cells.
      // Do NOT blacklist recovery probes — they are already in
      // known free space.
      if (current_goal_ && !current_goal_is_recovery_ &&
          goal_success_cooldown_seconds_ > 0.0) {
        const auto t = std::chrono::steady_clock::now();
        blacklist_.add(
          *current_goal_, t, goal_success_cooldown_radius_,
          std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::duration<double>(goal_success_cooldown_seconds_)));
        RCLCPP_INFO(get_logger(),
          "Success cooldown: (%.2f, %.2f) r=%.1f for %.0f s",
          current_goal_->x, current_goal_->y,
          goal_success_cooldown_radius_,
          goal_success_cooldown_seconds_);
        // 4D.2: deactivate recovery on success
        if (cooldown_starvation_recovery_active_) {
          cooldown_starvation_recovery_active_ = false;
          all_suppressed_count_ = 0;
          RCLCPP_INFO(get_logger(),
            "[CooldownRecovery] deactivated on success");
        }
      }
      break;

    case rclcpp_action::ResultCode::ABORTED: {
        RCLCPP_WARN(get_logger(), "Navigation aborted");
        if (result.result) {
          RCLCPP_WARN(get_logger(),
          "  error_code=%d  error_msg='%s'",
          result.result->error_code,
          result.result->error_msg.c_str());
        }
        // ── Stage 3D recovery tracking ───────────────────────────
        if (current_goal_is_recovery_) {
          ++recovery_failure_count_;
        }
        // ── Goal bin tracking ─────────────────────────────────────
        if (current_goal_) {
          recordGoalBin(*current_goal_, false);
        }
      // Blacklist the failed goal (frontier goals only).
        if (current_goal_ && !current_goal_is_recovery_) {
          const auto t = std::chrono::steady_clock::now();
          blacklist_.add(
          *current_goal_, t, blacklist_radius_,
          std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::duration<double>(blacklist_timeout_seconds_)));
          RCLCPP_INFO(get_logger(),
          "Blacklisted (%.2f, %.2f) for %.0f s",
          current_goal_->x, current_goal_->y, blacklist_timeout_seconds_);
          // 4D.2: deactivate recovery on abort
          if (cooldown_starvation_recovery_active_) {
            cooldown_starvation_recovery_active_ = false;
            all_suppressed_count_ = 0;
            RCLCPP_INFO(get_logger(),
              "[CooldownRecovery] deactivated on abort");
          }
        }
        break;
      }

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(get_logger(),
        "Navigation canceled — not blacklisting");
      if (current_goal_ && current_goal_is_recovery_) {
        ++recovery_failure_count_;
      }
      break;

    default:
      RCLCPP_WARN(get_logger(),
        "Navigation returned unknown result code %d",
        static_cast<int>(result.code));
      break;
  }

  current_goal_ = std::nullopt;
  current_goal_handle_.reset();
  current_goal_is_recovery_ = false;
  transitionTo(ExplorationState::COOLDOWN);
}

// ── getRobotPose ─────────────────────────────────────────────────────────

bool TunnelFrontierExplorerNode::getRobotPose(Point2D & robot_pose)
{
  try {
    const auto tf = tf_buffer_.lookupTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero);
    robot_pose.x = tf.transform.translation.x;
    robot_pose.y = tf.transform.translation.y;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "TF lookup %s → %s failed: %s",
      global_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
    return false;
  }
}

// ── getRobotYaw ────────────────────────────────────────────────────

double TunnelFrontierExplorerNode::getRobotYaw()
{
  try {
    const auto tf = tf_buffer_.lookupTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero);
    const auto & q = tf.transform.rotation;
    const double siny = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "TF yaw lookup failed: %s", ex.what());
    return 0.0;
  }
}

// ── detectLocalLoop ────────────────────────────────────────────────
//
// Checks whether the recent goal history shows entrance-area
// oscillation: the sliding window is full, unique spatial bins
// within the window are few, and most recent goals succeeded.

bool TunnelFrontierExplorerNode::detectLocalLoop() const
{
  if (!loop_detection_enabled_) return false;
  if (static_cast<int>(recent_goal_bins_.size()) < loop_window_size_)
    return false;

  std::set<std::pair<int, int>> unique;
  int successes = 0;
  for (const auto & rec : recent_goal_bins_) {
    unique.insert({rec.bin_x, rec.bin_y});
    if (rec.succeeded) ++successes;
  }

  return (static_cast<int>(unique.size()) <= loop_unique_bins_threshold_) &&
         (successes >= loop_min_successes_);
}

// ── generateRecoveryProbe ──────────────────────────────────────────
//
// When a local loop is detected, try distance×angle combinations
// forward from the robot's current pose.  Returns the first candidate
// that lands in known free space (occupancy 0).

std::optional<Point2D> TunnelFrontierExplorerNode::generateRecoveryProbe(
  const Point2D & robot, double yaw,
  const nav_msgs::msg::OccupancyGrid & map)
{
  if (!recovery_probe_enabled_) return std::nullopt;

  for (double dist : recovery_probe_distances_) {
    for (double angle_offset : recovery_probe_angle_offsets_rad_) {
      double probe_yaw = yaw + angle_offset;
      Point2D probe;
      probe.x = robot.x + dist * std::cos(probe_yaw);
      probe.y = robot.y + dist * std::sin(probe_yaw);

      const double ox = map.info.origin.position.x;
      const double oy = map.info.origin.position.y;
      const double res = map.info.resolution;
      const int mx = static_cast<int>((probe.x - ox) / res);
      const int my = static_cast<int>((probe.y - oy) / res);
      const int w = static_cast<int>(map.info.width);
      const int h = static_cast<int>(map.info.height);
      if (mx < 0 || mx >= w || my < 0 || my >= h) continue;

      const int idx = my * w + mx;
      if (idx >= 0 && idx < static_cast<int>(map.data.size()) &&
          map.data[idx] == 0) {
        return probe;
      }
    }
  }
  return std::nullopt;
}

// ── recordGoalBin ──────────────────────────────────────────────────

void TunnelFrontierExplorerNode::recordGoalBin(
  const Point2D & goal, bool succeeded)
{
  GoalBinRecord rec;
  rec.bin_x = static_cast<int>(std::floor(goal.x / loop_bin_size_));
  rec.bin_y = static_cast<int>(std::floor(goal.y / loop_bin_size_));
  rec.succeeded = succeeded;
  recent_goal_bins_.push_back(rec);
  while (static_cast<int>(recent_goal_bins_.size()) > loop_window_size_) {
    recent_goal_bins_.pop_front();
  }
}

// ── resetRecoveryState ─────────────────────────────────────────────

void TunnelFrontierExplorerNode::resetRecoveryState()
{
  recovery_attempt_count_ = 0;
  current_goal_is_recovery_ = false;
}

// ── projectGoalTowardRobot ─────────────────────────────────────────
//
// Pull the selected frontier goal inward toward the robot along the
// line from goal → robot. This prevents Nav2 from trying to converge
// on the exact frontier/unknown boundary where the costmap inflation
// or unknown cells cause oscillation or timeout.
//
// Effective projection distance is capped so the projected point
// stays at least min_remaining_distance from the robot — otherwise
// the robot would reach it instantly without revealing new map cells.
//
// Checks the projected point against the occupancy grid: it must be
// in known free space (value == 0). Falls back to shorter projection
// distances (stepped down by 0.1 m) if the current distance lands on
// unknown/occupied cells.

std::optional<Point2D> TunnelFrontierExplorerNode::projectGoalTowardRobot(
  const Point2D & goal, const Point2D & robot,
  const nav_msgs::msg::OccupancyGrid & map)
{
  const double dx = robot.x - goal.x;
  const double dy = robot.y - goal.y;
  const double norm = std::hypot(dx, dy);

  // If robot is already very close to the goal, skip projection
  // to avoid pulling the goal behind the robot.
  if (norm < 0.2) {
    return std::nullopt;
  }

  // Effective projection: apply full projection_distance, but ensure
  // the projected point remains at least min_remaining_distance from
  // the robot so it doesn't sit right on top of the current pose.
  const double effective = std::min(
    goal_projection_distance_,
    std::max(0.0, norm - goal_projection_min_remaining_distance_));

  if (effective < 0.1) {
    return std::nullopt;          // too close to project meaningfully
  }

  const double ux = dx / norm;
  const double uy = dy / norm;

  // Try effective distance, then step down by 0.1 m increments until
  // we find a known-free cell or exhaust the usable range.
  for (double d = effective; d >= 0.1; d -= 0.1) {
    Point2D projected;
    projected.x = goal.x + d * ux;
    projected.y = goal.y + d * uy;

    const double ox = map.info.origin.position.x;
    const double oy = map.info.origin.position.y;
    const double res = map.info.resolution;
    const int mx = static_cast<int>((projected.x - ox) / res);
    const int my = static_cast<int>((projected.y - oy) / res);

    const int w = static_cast<int>(map.info.width);
    const int h = static_cast<int>(map.info.height);
    if (mx < 0 || mx >= w || my < 0 || my >= h) {
      continue;
    }

    const int idx = my * w + mx;
    if (idx >= 0 && idx < static_cast<int>(map.data.size()) &&
        map.data[idx] == 0) {
      return projected;
    }
  }

  return std::nullopt;
}

// ── publishMarkers ───────────────────────────────────────────────────────

void TunnelFrontierExplorerNode::publishMarkers(
  const std::vector<FrontierCluster> & clusters,
  const std::vector<Point2D> & blacklisted_positions,
  const std::optional<Point2D> & selected_goal,
  const std::vector<Point2D> & too_close_positions)
{
  visualization_msgs::msg::MarkerArray markers;
  auto make_deleteall = [](const std::string & ns)
    -> visualization_msgs::msg::Marker
    {
      visualization_msgs::msg::Marker m;
      m.action = visualization_msgs::msg::Marker::DELETEALL;
      m.ns = ns;
      return m;
    };

  // ── Frontier clusters (green POINTS at centroids) ──────────────────
  markers.markers.push_back(make_deleteall("frontier_clusters"));
  if (!clusters.empty()) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_;
    m.header.stamp = this->now();
    m.ns = "frontier_clusters";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.08;
    m.scale.y = 0.08;
    m.color.a = 0.9;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    for (const auto & c : clusters) {
      geometry_msgs::msg::Point p;
      p.x = c.centroid_world.x;
      p.y = c.centroid_world.y;
      p.z = 0.0;
      m.points.push_back(p);
    }
    markers.markers.push_back(m);
  }

  // ── Selected goal (red SPHERE) ─────────────────────────────────────
  markers.markers.push_back(make_deleteall("selected_goal"));
  markers.markers.push_back(make_deleteall("projected_goal"));
  markers.markers.push_back(make_deleteall("recovery_probe"));
  if (selected_goal) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_;
    m.header.stamp = this->now();
    m.ns = "selected_goal";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = selected_goal->x;
    m.pose.position.y = selected_goal->y;
    m.pose.position.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.3;
    m.scale.y = 0.3;
    m.scale.z = 0.3;
    m.color.a = 0.9;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    markers.markers.push_back(m);
  }

  // ── Blacklisted positions (grey SPHERE) ────────────────────────────
  markers.markers.push_back(make_deleteall("blacklisted"));
  for (std::size_t i = 0; i < blacklisted_positions.size(); ++i) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_;
    m.header.stamp = this->now();
    m.ns = "blacklisted";
    m.id = static_cast<int>(i);
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = blacklisted_positions[i].x;
    m.pose.position.y = blacklisted_positions[i].y;
    m.pose.position.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.2;
    m.scale.y = 0.2;
    m.scale.z = 0.2;
    m.color.a = 0.6;
    m.color.r = 0.5;
    m.color.g = 0.5;
    m.color.b = 0.5;
    markers.markers.push_back(m);
  }

  // ── Too-close frontiers (yellow POINTS) ───────────────────────────
  markers.markers.push_back(make_deleteall("too_close_frontiers"));
  if (!too_close_positions.empty()) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_;
    m.header.stamp = this->now();
    m.ns = "too_close_frontiers";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.12;
    m.scale.y = 0.12;
    m.color.a = 0.9;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    for (const auto & pt : too_close_positions) {
      geometry_msgs::msg::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = 0.0;
      m.points.push_back(p);
    }
    markers.markers.push_back(m);
  }

  marker_pub_->publish(std::move(markers));
}

// -- publishScoredFrontierMarkers -----------------------------------------

void TunnelFrontierExplorerNode::publishScoredFrontierMarkers(
  const std::vector<ScoredGoal> & scored)
{
  visualization_msgs::msg::MarkerArray markers;

  auto make_deleteall = [](const std::string & ns)
    -> visualization_msgs::msg::Marker
    {
      visualization_msgs::msg::Marker m;
      m.action = visualization_msgs::msg::Marker::DELETEALL;
      m.ns = ns;
      return m;
    };

  markers.markers.push_back(make_deleteall("scored_frontiers"));

  if (!scored.empty()) {
    double min_score = std::numeric_limits<double>::max();
    double max_score = std::numeric_limits<double>::lowest();
    for (const auto & sg : scored) {
      min_score = std::min(min_score, sg.score);
      max_score = std::max(max_score, sg.score);
    }
    const double score_range = max_score - min_score;

    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_;
    m.header.stamp = this->now();
    m.ns = "scored_frontiers";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.color.a = 0.8;

    for (const auto & sg : scored) {
      double norm = 0.0;
      if (score_range > 1e-9) {
        norm = (sg.score - min_score) / score_range;
      }

      geometry_msgs::msg::Point p;
      p.x = sg.cluster.representative_world.x;
      p.y = sg.cluster.representative_world.y;
      p.z = 0.0;
      m.points.push_back(p);

      m.scale.x = 0.12 + norm * 0.18;
      m.scale.y = 0.12 + norm * 0.18;

      std_msgs::msg::ColorRGBA color;
      color.a = 0.8;
      color.r = 1.0 - norm;
      color.g = norm;
      color.b = 0.0;
      m.colors.push_back(color);
    }
    markers.markers.push_back(m);
  }

  marker_pub_->publish(std::move(markers));
}

// -- transitionTo ----------------------------------------------------------

void TunnelFrontierExplorerNode::transitionTo(ExplorationState new_state)
{
  if (new_state == state_) {
    return;
  }
  RCLCPP_DEBUG(get_logger(), "State: %s → %s",
               stateName(state_), stateName(new_state));
  state_ = new_state;

  if (new_state == ExplorationState::COOLDOWN) {
    cooldown_start_ = this->now();
  }
}

// ── stateName ────────────────────────────────────────────────────────────

const char * TunnelFrontierExplorerNode::stateName(ExplorationState s) const
{
  switch (s) {
    case ExplorationState::WAITING_FOR_MAP:  return "WAITING_FOR_MAP";
    case ExplorationState::WAITING_FOR_NAV2: return "WAITING_FOR_NAV2";
    case ExplorationState::IDLE:             return "IDLE";
    case ExplorationState::NAVIGATING:       return "NAVIGATING";
    case ExplorationState::COOLDOWN:         return "COOLDOWN";
    case ExplorationState::COMPLETED:        return "COMPLETED";
    case ExplorationState::STALLED:          return "STALLED";
    default:                                 return "UNKNOWN";
  }
}

}  // namespace tunnel_frontier_explorer
