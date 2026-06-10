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

#include <deque>
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
#include "tunnel_frontier_explorer/tunnel_geometry_grid.hpp"
#include "tunnel_frontier_explorer/entrance_oscillation_detector.hpp"
#include "tunnel_frontier_explorer/escape_probe_generator.hpp"

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
  COMPLETED,
  STALLED
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
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr dist_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr risk_map_sub_;
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

  // Stage 4B: tunnel geometry subscriber callbacks
  void distanceMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr & msg);
  void riskMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr & msg);

  // Stage 3D: entrance-loop recovery
  bool loop_detection_enabled_;
  int loop_window_size_;
  int loop_unique_bins_threshold_;
  int loop_min_successes_;
  double loop_bin_size_;
  bool recovery_probe_enabled_;
  std::vector<double> recovery_probe_distances_;
  std::vector<double> recovery_probe_angle_offsets_rad_;
  double recovery_probe_cooldown_seconds_;
  int recovery_max_attempts_;

  // Stage 4B: tunnel geometry
  std::string tunnel_distance_map_topic_;
  std::string tunnel_risk_map_topic_;
  double tunnel_geometry_max_age_seconds_;
  bool geometry_missing_fallback_to_stage3d_;
  TunnelGeometryGrid tunnel_geometry_;
  rclcpp::Time tunnel_geometry_last_update_{0, 0, RCL_ROS_TIME};

  // Stage 4B.2: entrance oscillation detection
  EntranceOscillationDetector entrance_oscillation_detector_;
  bool entrance_oscillation_enabled_;
  int entrance_oscillation_window_goals_;
  double entrance_oscillation_radius_m_;
  int entrance_oscillation_min_repeated_goals_;
  int entrance_oscillation_max_unique_bins_;
  double entrance_oscillation_min_revisit_ratio_;
  int entrance_oscillation_min_goals_to_check_;
  // Stage 4B.4: alternating-pair detection
  bool entrance_oscillation_detect_alternating_pair_;
  double entrance_oscillation_pair_cluster_radius_m_;
  double entrance_oscillation_pair_max_spatial_radius_m_;
  int entrance_oscillation_pair_min_cluster_count_;
  double entrance_oscillation_pair_min_alternation_score_;

  // Stage 4B.3: oscillation escape mode
  bool escape_mode_active_ = false;
  int escape_mode_remaining_goals_ = 0;
  Point2D oscillation_center_;
  bool entrance_oscillation_response_enabled_;
  int entrance_oscillation_escape_goals_;
  double entrance_oscillation_suppression_radius_m_;
  double entrance_oscillation_escape_penalty_;
  void applyEscapeModePenalty(std::vector<ScoredGoal> & scored);
  bool allCandidatesInsideRadius(const std::vector<ScoredGoal> & scored) const;

  // Stage 4C.1: forced escape probe
  bool entrance_oscillation_force_escape_probe_;
  double entrance_oscillation_probe_distance_m_;
  double entrance_oscillation_probe_distance_step_m_;
  double entrance_oscillation_probe_exit_margin_m_;
  int entrance_oscillation_probe_max_attempts_;
  double entrance_oscillation_probe_angle_span_deg_;
  double entrance_oscillation_probe_min_wall_distance_m_;
  int entrance_oscillation_probe_cooldown_goals_;
  int forced_probe_cooldown_remaining_ = 0;
  EscapeProbeGenerator probe_generator_;
  Point2D last_selected_goal_;

  // Stage 4E.2: startup bootstrap probe
  std::optional<Point2D> generateBootstrapProbeGoal(const Point2D & robot);
  bool startup_bootstrap_enabled_;
  double startup_bootstrap_distance_m_;
  int startup_bootstrap_max_attempts_;
  int startup_bootstrap_no_frontier_cycles_;
  int startup_bootstrap_attempts_ = 0;
  int goals_dispatched_ = 0;

  // Stage 4D.1: conservative fallback gating
  bool entrance_oscillation_stuck_detector_enabled_;
  double entrance_oscillation_no_progress_threshold_m_;
  int entrance_oscillation_no_progress_min_steps_;
  int entrance_oscillation_repeated_revisit_min_;
  int entrance_oscillation_cooldown_goals_;
  int escape_cooldown_remaining_ = 0;
  int entrance_oscillation_escape_max_goals_;
  double entrance_oscillation_escape_deactivation_distance_m_;
  int entrance_oscillation_escape_no_oscillation_deactivate_goals_;
  int escape_goals_dispatched_ = 0;
  int no_progress_count_ = 0;
  double last_goal_distance_ = 0.0;
  int no_oscillation_count_ = 0;
  bool evaluateStuckCondition(
    const OscillationStatus & osc_status,
    int num_valid_clusters,
    double current_goal_distance);

  // ── Goal safety projection ──────────────────────────────────────
  std::optional<Point2D> projectGoalTowardRobot(
    const Point2D & goal, const Point2D & robot,
    const nav_msgs::msg::OccupancyGrid & map);

  // ── Stage 3D: entrance-loop recovery ────────────────────────────
  double getRobotYaw();
  bool detectLocalLoop() const;
  std::optional<Point2D> generateRecoveryProbe(
    const Point2D & robot, double yaw,
    const nav_msgs::msg::OccupancyGrid & map);
  void recordGoalBin(const Point2D & goal, bool succeeded);
  void resetRecoveryState();
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

  // Stage 4D.2: cooldown starvation recovery
  bool cooldown_starvation_recovery_enabled_;
  int cooldown_starvation_recovery_threshold_;
  double cooldown_starvation_recovery_match_tolerance_m_;
  bool cooldown_starvation_recovery_active_ = false;
  Point2D cooldown_recovery_target_;

  // ── Stage 3D sliding-window loop tracking ──────────────────────
  struct GoalBinRecord {
    int bin_x;
    int bin_y;
    bool succeeded;
  };
  std::deque<GoalBinRecord> recent_goal_bins_;
  bool current_goal_is_recovery_ = false;

  // ── Stage 3D recovery probe state ──────────────────────────────
  int loop_detected_count_ = 0;
  int recovery_probe_count_ = 0;
  int recovery_success_count_ = 0;
  int recovery_failure_count_ = 0;
  rclcpp::Time recovery_probe_last_time_{0, 0, RCL_ROS_TIME};
  int recovery_attempt_count_ = 0;
};

}  // namespace tunnel_frontier_explorer

#endif  // TUNNEL_FRONTIER_EXPLORER__FRONTIER_EXPLORER_NODE_HPP_
