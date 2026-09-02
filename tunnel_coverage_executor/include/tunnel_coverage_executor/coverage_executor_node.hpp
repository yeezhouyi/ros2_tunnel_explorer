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

#ifndef TUNNEL_COVERAGE_EXECUTOR__COVERAGE_EXECUTOR_NODE_HPP_
#define TUNNEL_COVERAGE_EXECUTOR__COVERAGE_EXECUTOR_NODE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "tunnel_coverage_executor/checkpoint_store.hpp"
#include "tunnel_coverage_executor/coverage_task_core.hpp"
#include "tunnel_coverage_msgs/action/execute_coverage.hpp"
#include "tunnel_coverage_msgs/msg/coverage_status.hpp"
#include "tunnel_coverage_planner/cleanable_map_builder.hpp"
#include "tunnel_coverage_planner/coverage_tracker.hpp"
#include "tunnel_coverage_planner/robot_cleaning_geometry.hpp"
#include "tunnel_coverage_planner/scanline_planner.hpp"
#include "tunnel_map_core/grid_geometry.hpp"

namespace tunnel_coverage_executor
{

/// Task-phase values (mirror tunnel_coverage_msgs/CoverageStatus.msg).
enum TaskPhase : int
{
  PHASE_BOOTSTRAP = 0,
  PHASE_WAIT_MAP = 1,
  PHASE_WAIT_LOCALIZATION = 2,
  PHASE_WAIT_NAV2 = 3,
  PHASE_READY_IDLE = 4,
  PHASE_PLANNING = 5,
  PHASE_TRANSITING = 6,
  PHASE_EXECUTING_SEGMENT = 7,
  PHASE_RECOVERY = 8,
  PHASE_REPLANNING = 9,
  PHASE_RESIDUAL_CHECK = 10,
  PHASE_CANCELLING = 11,
  PHASE_TERMINAL = 12
};

/// Coverage-task executor node (U5 first pass).
///
/// Readiness: BOOTSTRAP -> WAIT_MAP -> WAIT_LOCALIZATION -> WAIT_NAV2 ->
/// READY_IDLE.  Only an explicit ExecuteCoverage goal starts motion (R17).
/// Work segments are executed with FollowPath, transitions with
/// NavigateToPose (KTD5); coverage is tracked from the fresh
/// map -> base_footprint transform sampled by a 10 Hz timer (R3, R19).
class CoverageExecutorNode : public rclcpp::Node
{
public:
  CoverageExecutorNode();

private:
  using NavClient = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>;
  using FollowClient = rclcpp_action::Client<nav2_msgs::action::FollowPath>;
  using CovAction = tunnel_coverage_msgs::action::ExecuteCoverage;
  using CovServer = rclcpp_action::Server<CovAction>;
  using CovGoalHandle = rclcpp_action::ServerGoalHandle<CovAction>;

  // ── ROS plumbing ───────────────────────────────────────────────────────
  void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void tickTimerCallback();
  void tfSamplerCallback();

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<tunnel_coverage_msgs::msg::CoverageStatus>::SharedPtr
    status_pub_;
  std::shared_ptr<CovServer> cov_server_;
  std::shared_ptr<NavClient> nav_client_;
  std::shared_ptr<FollowClient> follow_client_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr tf_sampler_timer_;

  // Child-goal handles (single active child invariant).
  NavClient::GoalHandle::SharedPtr nav_gh_;
  FollowClient::GoalHandle::SharedPtr follow_gh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ── Parameters ─────────────────────────────────────────────────────────
  std::string map_topic_;
  std::string nav_action_name_;
  std::string follow_action_name_;
  std::string global_frame_;
  std::string base_frame_;
  double max_tf_age_s_;
  double endpoint_tolerance_m_;
  double stop_velocity_threshold_;
  double stop_confirm_timeout_s_;
  double min_effective_coverage_;
  int max_attempts_per_segment_;
  std::string checkpoint_dir_;
  tunnel_coverage_planner::RobotCleaningGeometry robot_geo_;
  tunnel_coverage_planner::ScanlinePlannerConfig planner_cfg_;
  tunnel_coverage_planner::CleanableMapBuilderConfig builder_cfg_;

  // ── Map state ──────────────────────────────────────────────────────────
  std::optional<nav_msgs::msg::OccupancyGrid> latest_map_;
  std::optional<std::string> frozen_map_digest_;

  // ── Task state ─────────────────────────────────────────────────────────
  int phase_ = PHASE_BOOTSTRAP;
  bool task_active_ = false;
  std::optional<std::shared_ptr<CovGoalHandle>> current_goal_;
  std::unique_ptr<tunnel_coverage_planner::CoverageMasks> masks_;
  std::unique_ptr<tunnel_coverage_planner::CoveragePlan> plan_;
  std::unique_ptr<tunnel_coverage_planner::CoverageTracker> tracker_;
  std::unique_ptr<CoverageTaskCore> core_;
  std::string plan_id_;
  std::string task_input_id_;
  std::string failure_class_;
  std::int32_t terminal_result_ = RESULT_SUCCEEDED_FULL;
  rclcpp::Time task_start_time_;
  double task_duration_s_ = 0.0;
  bool sampling_enabled_ = false;
  std::optional<std::string> resume_path_;

  // One child-goal outcome waiting to be consumed by the tick.
  struct ChildOutcome
  {
    int idx = -1;
    bool ok = false;
  };
  std::optional<ChildOutcome> pending_outcome_;

  // Execution progress inside a segment.
  int exec_index_ = -1;
  int exec_attempt_ = 0;
  bool child_sent_ = false;
  // What the in-flight child goal is doing:
  // 1 = repositioning to a work-line start (approach), 2 = a plan TRANSITION
  // segment, 3 = following a WORK segment.
  int child_mode_ = 0;

  // Cancel/stop-confirmation state.
  bool cancel_requested_ = false;
  bool map_changed_ = false;
  bool cancel_started_ = false;
  rclcpp::Time cancel_start_time_;
  std::optional<tunnel_map_core::Point2D> last_cancel_pose_;

  // ── TF sampling ────────────────────────────────────────────────────────
  std::optional<tunnel_map_core::Point2D> last_tool_pose_;

  // ── Helpers ────────────────────────────────────────────────────────────
  bool getRobotPose(tunnel_map_core::Point2D & pose) const;
  void publishStatus();
  void setPhase(int phase);
  void beginTask(std::shared_ptr<CovGoalHandle> goal_handle);
  void sendNextSegment();
  void processOutcome(int idx, bool ok);
  void tickReadiness();
  void tickExecution();
  void tickCancelling();
  void sendNavigate(const tunnel_coverage_planner::CoverageSegment & seg);
  void sendFollow(const tunnel_coverage_planner::CoverageSegment & seg);
  void saveCheckpoint(const std::string & reason);
  void finishTask();
};

}  // namespace tunnel_coverage_executor

#endif  // TUNNEL_COVERAGE_EXECUTOR__COVERAGE_EXECUTOR_NODE_HPP_
