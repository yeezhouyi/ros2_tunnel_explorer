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
#include <limits>
#include <memory>
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
    selection_strategy_ != "information_gain_revisit")
  {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid selection_strategy '%s' -- must be 'nearest' or "
      "'information_gain_revisit'", selection_strategy_.c_str());
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

      // Filter blacklisted clusters.
        const auto now = std::chrono::steady_clock::now();
        blacklist_.cleanup(now);

        std::vector<FrontierCluster> valid;
        std::vector<Point2D> blacklisted_positions;
        for (const auto & c : clusters) {
          if (blacklist_.contains(c.representative_world, now)) {
            blacklisted_positions.push_back(c.representative_world);
          } else {
            valid.push_back(c);
          }
        }

        if (valid.empty()) {
          if (!clusters.empty() &&
            clusters.size() == blacklisted_positions.size())
          {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "All %zu frontiers blacklisted — waiting for expiry",
            clusters.size());
            frontier_empty_count_ = 0;
          } else {
            ++frontier_empty_count_;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "No frontiers (%zu/%zu empty cycles)",
            frontier_empty_count_, k_max_empty_cycles_);
            if (frontier_empty_count_ >= k_max_empty_cycles_) {
              RCLCPP_INFO(get_logger(), "No frontiers for %zu cycles — "
                          "exploration complete", k_max_empty_cycles_);
              transitionTo(ExplorationState::COMPLETED);
            }
          }
          publishMarkers(clusters, blacklisted_positions, std::nullopt, {});
          return;
        }
        frontier_empty_count_ = 0;

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
          const auto & best = scored_frontiers.front();
          goal_pt = best.cluster.representative_world;
          selected_for_marker = goal_pt;

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

      // Prepare NavigateToPose goal.
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = global_frame_;
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = goal_pt.x;
        goal_msg.pose.pose.position.y = goal_pt.y;
        goal_msg.pose.pose.position.z = 0.0;

        if (orient_goal_toward_frontier_) {
          const double yaw = std::atan2(
          goal_pt.y - robot_pose.y, goal_pt.x - robot_pose.x);
          goal_msg.pose.pose.orientation.z = std::sin(yaw * 0.5);
          goal_msg.pose.pose.orientation.w = std::cos(yaw * 0.5);
        } else {
          goal_msg.pose.pose.orientation.w = 1.0;
        }

      // Store for blacklisting on failure.
        current_goal_ = goal_pt;
        navigating_start_time_ = this->now();

      // Send goal.
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
        transitionTo(ExplorationState::NAVIGATING);

        publishMarkers(
          clusters, blacklisted_positions, selected_for_marker,
          too_close_for_marker);

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
            // Blacklist before cancelling to prevent re-selection.
            const auto t = std::chrono::steady_clock::now();
            blacklist_.add(
              *current_goal_, t, blacklist_radius_,
              std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::duration<double>(blacklist_timeout_seconds_)));
            RCLCPP_INFO(
              get_logger(), "Blacklisted (%.2f, %.2f) for %.0f s",
              current_goal_->x, current_goal_->y, blacklist_timeout_seconds_);
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
      break;

    case rclcpp_action::ResultCode::ABORTED: {
        RCLCPP_WARN(get_logger(), "Navigation aborted");
        if (result.result) {
          RCLCPP_WARN(get_logger(),
          "  error_code=%d  error_msg='%s'",
          result.result->error_code,
          result.result->error_msg.c_str());
        }
      // Blacklist the failed goal.
        if (current_goal_) {
          const auto t = std::chrono::steady_clock::now();
          blacklist_.add(
          *current_goal_, t, blacklist_radius_,
          std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::duration<double>(blacklist_timeout_seconds_)));
          RCLCPP_INFO(get_logger(),
          "Blacklisted (%.2f, %.2f) for %.0f s",
          current_goal_->x, current_goal_->y, blacklist_timeout_seconds_);
        }
        break;
      }

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(get_logger(),
        "Navigation canceled — not blacklisting");
      break;

    default:
      RCLCPP_WARN(get_logger(),
        "Navigation returned unknown result code %d",
        static_cast<int>(result.code));
      break;
  }

  current_goal_ = std::nullopt;
  current_goal_handle_.reset();
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
    default:                                 return "UNKNOWN";
  }
}

}  // namespace tunnel_frontier_explorer
