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
#include "tunnel_frontier_explorer/frontier_goal_selector.hpp"
#include "tunnel_frontier_explorer/frontier_scorer.hpp"
#include "tunnel_frontier_explorer/frontier_visit_history.hpp"

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
    const std::optional<Point2D> & selected_goal,
    const std::vector<Point2D> & too_close_positions);
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
  FrontierGoalSelector goal_selector_;
  FrontierScorer scorer_;
  FrontierVisitHistory visit_history_;

  // ── Parameters ───────────────────────────────────────────────────────
  double exploration_period_seconds_;
  double cooldown_seconds_;
  double goal_timeout_seconds_;
  std::size_t min_cluster_size_;
  int free_threshold_;
  int frontier_neighbor_connectivity_;
  int cluster_connectivity_;
  double blacklist_radius_;
  double blacklist_timeout_seconds_;
  bool orient_goal_toward_frontier_;
  double min_goal_distance_meters_;
  bool goal_projection_enabled_;
  double goal_projection_distance_;
  double goal_projection_min_remaining_distance_;
  double goal_success_cooldown_seconds_;
  double goal_success_cooldown_radius_;

  // ── Goal safety projection ──────────────────────────────────────
  std::optional<Point2D> projectGoalTowardRobot(
    const Point2D & goal, const Point2D & robot,
    const nav_msgs::msg::OccupancyGrid & map);
  std::string map_topic_;
  std::string global_frame_;
  std::string robot_base_frame_;
  std::string action_server_name_;
  std::string marker_topic_;

  // Stage 2B parameters
  std::string selection_strategy_;
  double information_gain_radius_meters_;
  double revisit_radius_meters_;
  int max_revisit_count_;
  double weight_information_gain_;
  double weight_distance_;
  double weight_revisit_;

  // ── Scored frontier markers ──────────────────────────────────────────
  void publishScoredFrontierMarkers(
    const std::vector<ScoredGoal> & scored);

  // ── State ────────────────────────────────────────────────────────────
  ExplorationState state_ = ExplorationState::WAITING_FOR_MAP;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_map_;
  rclcpp::Time cooldown_start_;
  rclcpp::Time navigating_start_time_;
  std::optional<Point2D> current_goal_;
  GoalHandle::SharedPtr current_goal_handle_;

  // Consecutive empty frontier cycles before entering COMPLETED.
  std::size_t frontier_empty_count_ = 0;
  static constexpr std::size_t k_max_empty_cycles_ = 10;

  // Consecutive cycles where all frontiers are suppressed (blacklisted
  // or inside success cooldown regions).  After enough consecutive
  // suppressed cycles the explorer declares completion rather than
  // spinning forever re-dispatching the same entrance-area points.
  std::size_t all_suppressed_count_ = 0;
  // Must outlast the success cooldown (120 s / 1 s per cycle = 120)
  // plus margin for the map to resolve, otherwise we declare completion
  // while the cooldown is still active — a false positive.
  static constexpr std::size_t k_max_all_suppressed_cycles_ = 180;
};

}  // namespace tunnel_frontier_explorer

#endif  // TUNNEL_FRONTIER_EXPLORER__FRONTIER_EXPLORER_NODE_HPP_
