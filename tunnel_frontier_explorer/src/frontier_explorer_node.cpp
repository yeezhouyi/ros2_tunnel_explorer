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
  blacklist_()
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
  cooldown_seconds_ = declare_parameter<double>("cooldown_seconds", 1.0);
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

  // Reconfigure detector with actual parameters.
  FrontierDetectorConfig cfg;
  cfg.min_cluster_size = static_cast<std::size_t>(min_cluster_size_);
  cfg.free_threshold = free_threshold_;
  cfg.frontier_neighbor_connectivity = frontier_neighbor_connectivity_;
  cfg.cluster_connectivity = cluster_connectivity_;
  detector_ = FrontierDetector(cfg);

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
          if (blacklist_.contains(c.centroid_world, now)) {
            blacklisted_positions.push_back(c.centroid_world);
          } else {
            valid.push_back(c);
          }
        }

      // Sort by distance to robot (nearest first).
        std::sort(valid.begin(), valid.end(),
          [](const FrontierCluster & a, const FrontierCluster & b) {
            return a.distance_to_robot < b.distance_to_robot;
        });

        if (valid.empty()) {
          if (!clusters.empty() &&
            clusters.size() == blacklisted_positions.size())
          {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "All %zu frontiers blacklisted — waiting for expiry",
            clusters.size());
          } else {
            RCLCPP_INFO(get_logger(),
            "No frontiers — exploration complete");
            transitionTo(ExplorationState::COMPLETED);
          }
          publishMarkers(clusters, blacklisted_positions, std::nullopt);
          return;
        }

      // Pick nearest valid frontier.
        const auto & target = valid.front();
        const Point2D goal_pt = target.representative_world;

        RCLCPP_INFO(get_logger(),
        "Goal: (%.2f, %.2f) [%zu cells, %.1f m]",
        goal_pt.x, goal_pt.y, target.size(), target.distance_to_robot);

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

        publishMarkers(clusters, blacklisted_positions, goal_pt);
        return;
      }

    case ExplorationState::NAVIGATING:
      // Awaiting action result — idle.
      return;

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
    transitionTo(ExplorationState::IDLE);
    return;
  }
  RCLCPP_INFO(get_logger(), "Goal accepted by Nav2 server");
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
  const std::optional<Point2D> & selected_goal)
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

  marker_pub_->publish(std::move(markers));
}

// ── transitionTo ─────────────────────────────────────────────────────────

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
