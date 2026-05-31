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

#ifndef TUNNEL_FRONTIER_EXPLORER__FRONTIER_EXPLORER_NODE_HPP_
#define TUNNEL_FRONTIER_EXPLORER__FRONTIER_EXPLORER_NODE_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "tunnel_frontier_explorer/frontier_blacklist.hpp"
#include "tunnel_frontier_explorer/frontier_cluster.hpp"
#include "tunnel_frontier_explorer/frontier_detector.hpp"

namespace tunnel_frontier_explorer
{

/// Exploration state machine states.
enum class ExplorationState
{
  WAITING_FOR_MAP,
  WAITING_FOR_NAV2,
  IDLE,
  NAVIGATING,
  COOLDOWN,
  COMPLETED
};

/// ROS2 node for nearest-frontier autonomous exploration.
///
/// Subscribes to /map, detects frontier clusters, sends NavigateToPose
/// goals to Nav2, and publishes RViz markers.
class TunnelFrontierExplorerNode : public rclcpp::Node
{
public:
  TunnelFrontierExplorerNode();

private:
  // ── Callbacks ────────────────────────────────────────────────────────
  void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void explorationTimerCallback();

  // ── Action client callbacks ──────────────────────────────────────────
  using GoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

  void goalResponseCallback(const GoalHandle::SharedPtr & handle);
  void feedbackCallback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
  void resultCallback(const GoalHandle::WrappedResult & result);

  // ── Helpers ──────────────────────────────────────────────────────────
  bool getRobotPose(Point2D & robot_pose);
  void publishMarkers(
    const std::vector<FrontierCluster> & clusters,
    const std::vector<Point2D> & blacklisted_positions,
    const std::optional<Point2D> & selected_goal);
  void transitionTo(ExplorationState new_state);
  const char * stateName(ExplorationState s) const;

  // ── Subscriptions / publishers / clients ─────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr exploration_timer_;

  // ── TF ───────────────────────────────────────────────────────────────
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ── Pure algorithm objects ───────────────────────────────────────────
  FrontierDetector detector_;
  FrontierBlacklist blacklist_;

  // ── Parameters ───────────────────────────────────────────────────────
  double exploration_period_seconds_;
  double cooldown_seconds_;
  std::size_t min_cluster_size_;
  int free_threshold_;
  int frontier_neighbor_connectivity_;
  int cluster_connectivity_;
  double blacklist_radius_;
  double blacklist_timeout_seconds_;
  bool orient_goal_toward_frontier_;
  std::string map_topic_;
  std::string global_frame_;
  std::string robot_base_frame_;
  std::string action_server_name_;
  std::string marker_topic_;

  // ── State ────────────────────────────────────────────────────────────
  ExplorationState state_ = ExplorationState::WAITING_FOR_MAP;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_map_;
  rclcpp::Time cooldown_start_;
  std::optional<Point2D> current_goal_;
};

}  // namespace tunnel_frontier_explorer

#endif  // TUNNEL_FRONTIER_EXPLORER__FRONTIER_EXPLORER_NODE_HPP_
