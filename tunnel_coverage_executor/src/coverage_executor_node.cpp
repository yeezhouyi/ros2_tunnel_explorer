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

#include "tunnel_coverage_executor/coverage_executor_node.hpp"

#include <tf2/exceptions.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace tunnel_coverage_executor
{

namespace
{

double dist2d(double ax, double ay, double bx, double by)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

CoverageExecutorNode::CoverageExecutorNode()
: Node("tunnel_coverage_executor"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // ── Parameters ────────────────────────────────────────────────────────
  map_topic_ = declare_parameter<std::string>("map_topic", "/map");
  nav_action_name_ = declare_parameter<std::string>(
    "navigate_to_pose_action", "/navigate_to_pose");
  follow_action_name_ = declare_parameter<std::string>(
    "follow_path_action", "/follow_path");
  global_frame_ = declare_parameter<std::string>("global_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  max_tf_age_s_ = declare_parameter<double>("max_tf_age_s", 1.0);
  endpoint_tolerance_m_ = declare_parameter<double>(
    "endpoint_tolerance_m", 0.15);
  stop_velocity_threshold_ = declare_parameter<double>(
    "stop_velocity_threshold_mps", 0.05);
  stop_confirm_timeout_s_ = declare_parameter<double>(
    "stop_confirm_timeout_s", 8.0);
  min_effective_coverage_ = declare_parameter<double>(
    "min_effective_coverage", 0.97);
  max_attempts_per_segment_ = declare_parameter<int>(
    "max_attempts_per_segment", 2);
  checkpoint_dir_ = declare_parameter<std::string>(
    "checkpoint_dir", "/tmp/tunnel_coverage_checkpoints");

  robot_geo_.robot_radius_m = declare_parameter<double>(
    "robot_radius_m", robot_geo_.robot_radius_m);
  robot_geo_.safety_margin_m = declare_parameter<double>(
    "safety_margin_m", robot_geo_.safety_margin_m);
  robot_geo_.cleaning_width_m = declare_parameter<double>(
    "cleaning_width_m", robot_geo_.cleaning_width_m);
  robot_geo_.min_cleanable_region_area_m2 = declare_parameter<double>(
    "min_cleanable_region_area_m2", robot_geo_.min_cleanable_region_area_m2);

  planner_cfg_.overlap_eta = declare_parameter<double>(
    "planner.overlap_eta", planner_cfg_.overlap_eta);
  planner_cfg_.loc_err_p95_m = declare_parameter<double>(
    "planner.loc_err_p95_m", planner_cfg_.loc_err_p95_m);
  planner_cfg_.track_err_p95_m = declare_parameter<double>(
    "planner.track_err_p95_m", planner_cfg_.track_err_p95_m);
  planner_cfg_.min_segment_length_m = declare_parameter<double>(
    "planner.min_segment_length_m", planner_cfg_.min_segment_length_m);
  planner_cfg_.endpoint_inset_m = declare_parameter<double>(
    "planner.endpoint_inset_m", planner_cfg_.endpoint_inset_m);

  builder_cfg_.free_max = declare_parameter<int>(
    "builder.free_max", builder_cfg_.free_max);
  builder_cfg_.occupied_min = declare_parameter<int>(
    "builder.occupied_min", builder_cfg_.occupied_min);

  // ── Map subscription (transient_local, same as the frontier node) ─────
  auto map_qos = rclcpp::QoS(1).transient_local().reliable();
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {mapCallback(msg);});

  // ── Publishers ────────────────────────────────────────────────────────
  status_pub_ = create_publisher<tunnel_coverage_msgs::msg::CoverageStatus>(
    "coverage/status", 10);

  // ── Action server: ExecuteCoverage ────────────────────────────────────
  cov_server_ = rclcpp_action::create_server<CovAction>(
    this, "execute_coverage",
    [this](const rclcpp_action::GoalUUID &,
    std::shared_ptr<const CovAction::Goal>)
    {
      if (task_active_) {
        RCLCPP_WARN(get_logger(), "ExecuteCoverage rejected — task active");
        return rclcpp_action::GoalResponse::REJECT;
      }
      if (phase_ != PHASE_READY_IDLE) {
        RCLCPP_WARN(get_logger(), "ExecuteCoverage rejected — not READY_IDLE");
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const std::shared_ptr<CovGoalHandle> &)
    {
      if (task_active_) {
        cancel_requested_ = true;
        setPhase(PHASE_CANCELLING);
        return rclcpp_action::CancelResponse::ACCEPT;
      }
      return rclcpp_action::CancelResponse::REJECT;
    },
    [this](const std::shared_ptr<CovGoalHandle> & goal_handle)
    {
      beginTask(goal_handle);
    });

  // ── Nav2 action clients ───────────────────────────────────────────────
  nav_client_ = rclcpp_action::create_client<
    nav2_msgs::action::NavigateToPose>(this, nav_action_name_);
  follow_client_ = rclcpp_action::create_client<
    nav2_msgs::action::FollowPath>(this, follow_action_name_);

  // ── Timers ────────────────────────────────────────────────────────────
  tick_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() {tickTimerCallback();});
  tf_sampler_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() {tfSamplerCallback();});

  setPhase(PHASE_BOOTSTRAP);
  RCLCPP_INFO(get_logger(), "Started tunnel_coverage_executor");
}

// ── mapCallback ─────────────────────────────────────────────────────────

void CoverageExecutorNode::mapCallback(
  nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (msg->info.width == 0 || msg->info.height == 0 ||
    msg->info.resolution <= 0.0)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Invalid map skipped");
    return;
  }
  tunnel_map_core::GridMap gm;
  gm.width = msg->info.width;
  gm.height = msg->info.height;
  gm.resolution = msg->info.resolution;
  gm.origin_x = msg->info.origin.position.x;
  gm.origin_y = msg->info.origin.position.y;
  gm.origin_yaw = 0.0;
  gm.data = msg->data;
  if (!gm.valid()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Malformed map skipped");
    return;
  }
  tunnel_map_core::GridGeometry geo(gm);
  const std::string digest = geo.contentDigest(gm);

  if (!frozen_map_digest_) {
    // First valid map freezes task identity (R23).
    frozen_map_digest_ = digest;
    latest_map_ = *msg;
    if (phase_ == PHASE_BOOTSTRAP || phase_ == PHASE_WAIT_MAP) {
      RCLCPP_INFO(get_logger(),
        "Static map frozen (%zux%zu res=%.3f digest=%s)",
        gm.width, gm.height, gm.resolution, digest.c_str());
      setPhase(PHASE_WAIT_LOCALIZATION);
    }
    return;
  }

  if (digest == *frozen_map_digest_) {
    latest_map_ = *msg;  // identical re-publish: refresh, keep identity
    return;
  }

  if (!task_active_) {
    RCLCPP_WARN(get_logger(),
      "Map content changed while idle — refreeze (digest %s -> %s)",
      frozen_map_digest_->c_str(), digest.c_str());
    frozen_map_digest_ = digest;
    latest_map_ = *msg;
    return;
  }

  // Content change during a task: safe stop (R23 -> MAP_CHANGED).
  RCLCPP_ERROR(get_logger(),
    "Map changed during task (%s -> %s): safe stop",
    frozen_map_digest_->c_str(), digest.c_str());
  map_changed_ = true;
  if (phase_ != PHASE_CANCELLING) {
    cancel_requested_ = true;
    setPhase(PHASE_CANCELLING);
  }
}

// ── tickTimerCallback ───────────────────────────────────────────────────

void CoverageExecutorNode::tickTimerCallback()
{
  if (!task_active_) {
    tickReadiness();
  } else {
    switch (phase_) {
      case PHASE_TRANSITING:
      case PHASE_EXECUTING_SEGMENT:
        tickExecution();
        break;
      case PHASE_CANCELLING:
        tickCancelling();
        break;
      default:
        break;
    }
  }
  publishStatus();
}

// ── Readiness gates (R17) ───────────────────────────────────────────────

void CoverageExecutorNode::tickReadiness()
{
  if (phase_ == PHASE_BOOTSTRAP) {
    setPhase(PHASE_WAIT_MAP);
    return;
  }
  if (phase_ == PHASE_WAIT_MAP) {
    return;  // mapCallback freezes the map and advances the phase
  }
  if (phase_ == PHASE_WAIT_LOCALIZATION) {
    tunnel_map_core::Point2D pose;
    if (getRobotPose(pose)) {
      RCLCPP_INFO(get_logger(), "Localisation valid (map -> %s)",
        base_frame_.c_str());
      setPhase(PHASE_WAIT_NAV2);
    }
    return;
  }
  if (phase_ == PHASE_WAIT_NAV2) {
    const bool nav_ready =
      nav_client_->wait_for_action_server(std::chrono::seconds(0));
    const bool follow_ready =
      follow_client_->wait_for_action_server(std::chrono::seconds(0));
    if (nav_ready && follow_ready) {
      RCLCPP_INFO(get_logger(),
        "Nav2 action servers available (%s, %s)",
        nav_action_name_.c_str(), follow_action_name_.c_str());
      setPhase(PHASE_READY_IDLE);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for Nav2 action servers (nav=%d follow=%d)",
        nav_ready ? 1 : 0, follow_ready ? 1 : 0);
    }
  }
}

// ── beginTask ───────────────────────────────────────────────────────────

void CoverageExecutorNode::beginTask(
  std::shared_ptr<CovGoalHandle> goal_handle)
{
  current_goal_ = goal_handle;
  task_active_ = true;
  cancel_requested_ = false;
  map_changed_ = false;
  cancel_started_ = false;
  sampling_enabled_ = false;
  child_sent_ = false;
  pending_outcome_.reset();
  failure_class_.clear();
  terminal_result_ = RESULT_SUCCEEDED_FULL;
  task_start_time_ = now();
  task_duration_s_ = 0.0;
  resume_path_ = goal_handle->get_goal()->resume_checkpoint_path;
  if (resume_path_ && resume_path_->empty()) {
    resume_path_.reset();
  }
  setPhase(PHASE_PLANNING);

  tunnel_map_core::Point2D seed{0.0, 0.0};
  if (!getRobotPose(seed)) {
    failure_class_ = "LOCALIZATION_FAILED";
    terminal_result_ = RESULT_PARTIAL_FAILED;
    finishTask();
    return;
  }

  try {
    const auto & map = *latest_map_;
    tunnel_map_core::GridMap gm;
    gm.width = map.info.width;
    gm.height = map.info.height;
    gm.resolution = map.info.resolution;
    gm.origin_x = map.info.origin.position.x;
    gm.origin_y = map.info.origin.position.y;
    gm.origin_yaw = 0.0;
    gm.data = map.data;

    tunnel_map_core::GridGeometry geo(gm);
    task_input_id_ = *frozen_map_digest_;

    tunnel_coverage_planner::CleanableMapBuilder builder(geo, builder_cfg_);
    masks_ = std::make_unique<tunnel_coverage_planner::CoverageMasks>(
      builder.build(gm, robot_geo_, seed));

    tunnel_coverage_planner::ScanlinePlanner planner(
      geo, *masks_, robot_geo_, planner_cfg_);
    auto p = planner.plan();
    if (!p.valid()) {
      failure_class_ = "EMPTY_PLAN";
      terminal_result_ = RESULT_PARTIAL_FAILED;
      RCLCPP_ERROR(get_logger(), "Plan empty (no cleanable area)");
      finishTask();
      return;
    }
    p.task_input_id = task_input_id_;
    plan_id_ = p.plan_id;

    tracker_ = std::make_unique<tunnel_coverage_planner::CoverageTracker>(
      geo, *masks_, robot_geo_.toolSweepRadiusM());

    CoverageTaskCore::Options opts;
    opts.max_attempts_per_segment = max_attempts_per_segment_;
    // Copy the segments into the task core before moving the plan.
    core_ = std::make_unique<CoverageTaskCore>(p.segments, opts);
    plan_ = std::make_unique<tunnel_coverage_planner::CoveragePlan>(
      std::move(p));

    if (resume_path_) {
      CheckpointData cp;
      const auto status = CheckpointStore::loadAndValidate(
        *resume_path_, task_input_id_, plan_id_, cp);
      if (status == CheckpointLoadStatus::OK) {
        core_->applyCheckpointDispositions(cp);
        tracker_->restore(cp.visit_counts, cp.path_length_m);
        RCLCPP_INFO(get_logger(),
          "Resumed from checkpoint %s (covered=%zu)",
          resume_path_->c_str(), core_->countCovered());
      } else if (status == CheckpointLoadStatus::MISMATCH) {
        failure_class_ = "CHECKPOINT_MISMATCH";
        terminal_result_ = RESULT_PARTIAL_FAILED;
        RCLCPP_ERROR(get_logger(),
          "Checkpoint identity mismatch — refusing to resume (R12)");
        finishTask();
        return;
      } else {
        RCLCPP_WARN(get_logger(),
          "Checkpoint %s not usable (%d) — starting fresh",
          resume_path_->c_str(), static_cast<int>(status));
      }
    }

    task_start_time_ = now();
    task_duration_s_ = 0.0;
    last_tool_pose_.reset();
    work_row_start_pose_.reset();
    sampling_enabled_ = true;
    setPhase(PHASE_EXECUTING_SEGMENT);
    RCLCPP_INFO(get_logger(),
      "Coverage task started: task_input_id=%s plan_id=%s work=%zu",
      task_input_id_.c_str(), plan_id_.c_str(), plan_->work_count);
  } catch (const std::exception & e) {
    failure_class_ = "PLANNING_FAILED";
    terminal_result_ = RESULT_PARTIAL_FAILED;
    RCLCPP_ERROR(get_logger(), "Planning failed: %s", e.what());
    finishTask();
  }
}

// ── Segment dispatch ────────────────────────────────────────────────────

void CoverageExecutorNode::sendNextSegment()
{
  if (!core_ || !plan_) {
    return;
  }
  const int idx = core_->nextPendingIndex();
  if (idx < 0) {
    // ── Residual check: coverage gate over the executed plan ──────────
    setPhase(PHASE_RESIDUAL_CHECK);
    const auto m = tracker_->metrics();
    RCLCPP_INFO(get_logger(),
      "Coverage: effective=%.4f gross=%.4f exempt=%.4f repeat=%.4f "
      "covered=%zu failed=%zu",
      m.effective_coverage, m.gross_coverage, m.exempt_ratio, m.repeat_ratio,
      core_->countCovered(), core_->countFailed());

    const bool below_threshold =
      m.effective_coverage < min_effective_coverage_ - 1e-9;
    if (below_threshold && failure_class_.empty()) {
      failure_class_ = "COVERAGE_BELOW_THRESHOLD";
    }
    terminal_result_ = core_->terminalResult();
    if (below_threshold && terminal_result_ == RESULT_SUCCEEDED_FULL) {
      terminal_result_ = m.exempt_ratio > 1e-9 ?
        RESULT_SUCCEEDED_WITH_EXEMPTIONS : RESULT_PARTIAL_FAILED;
    }
    saveCheckpoint("terminal");
    finishTask();
    return;
  }

  const auto & seg = plan_->segments[static_cast<std::size_t>(idx)];
  if (idx != exec_index_) {
    exec_index_ = idx;
    exec_attempt_ = 0;  // fresh attempts for a new segment
  }
  if (seg.type == tunnel_coverage_planner::SegmentType::TRANSITION) {
    setPhase(PHASE_TRANSITING);
    child_mode_ = 2;  // plan transition segment
    sendNavigate(seg);
    return;
  }

  // WORK segment: reposition if not at the start, then follow the line.
  setPhase(PHASE_EXECUTING_SEGMENT);
  tunnel_map_core::Point2D pose;
  if (!getRobotPose(pose) ||
    dist2d(pose.x, pose.y, seg.start_x, seg.start_y) >
    std::max(endpoint_tolerance_m_, 0.05))
  {
    // Approach the work-line start using NavigateToPose (orientation =
    // line direction).
    tunnel_coverage_planner::CoverageSegment approach = seg;
    approach.type = tunnel_coverage_planner::SegmentType::TRANSITION;
    approach.start_x = seg.start_x;
    approach.start_y = seg.start_y;
    approach.end_x = seg.start_x;
    approach.end_y = seg.start_y;
    approach.end_yaw = seg.start_yaw;
    child_mode_ = 1;  // approach to the work start
    sendNavigate(approach);
    return;
  }
  child_mode_ = 3;  // follow the work line
  work_row_start_pose_ = pose;
  sendFollow(seg);
}

void CoverageExecutorNode::tickExecution()
{
  if (pending_outcome_) {
    const auto o = *pending_outcome_;
    pending_outcome_.reset();
    processOutcome(o.idx, o.ok);
    return;
  }
  if (!child_sent_) {
    sendNextSegment();
  }
}

void CoverageExecutorNode::processOutcome(int idx, bool ok)
{
  if (!core_ || !plan_ ||
    idx < 0 || static_cast<std::size_t>(idx) >= plan_->segments.size())
  {
    child_mode_ = 0;
    return;
  }
  const auto & seg = plan_->segments[static_cast<std::size_t>(idx)];
  const int mode = child_mode_;
  child_mode_ = 0;

  if (mode == 1) {
    // Approach to the work-line start succeeded -> now follow the line;
    // failure retries the approach (or fails the segment when exhausted).
    if (ok) {
      child_mode_ = 3;
      work_row_start_pose_ = tunnel_map_core::Point2D{seg.start_x, seg.start_y};
      sendFollow(seg);
      return;
    }
    if (exec_attempt_ < max_attempts_per_segment_) {
      ++exec_attempt_;
      child_mode_ = 1;
      sendNavigate(seg);
      return;
    }
    core_->markFailed(static_cast<std::size_t>(idx));
    if (failure_class_.empty()) {
      failure_class_ = "WORK_APPROACH_FAILED";
    }
    RCLCPP_ERROR(get_logger(), "Segment %s -> FAILED (approach)", seg.id.c_str());
    return;
  }

  if (ok && mode == 3) {
    // Work completes only when the robot actually reached the segment end
    // (R20, geometric gate; real-sweep gate is the global coverage check).
    tunnel_map_core::Point2D pose;
    ok = getRobotPose(pose) &&
      dist2d(pose.x, pose.y, seg.end_x, seg.end_y) <=
      std::max(endpoint_tolerance_m_, 0.10);
    if (!ok) {
      RCLCPP_WARN(get_logger(),
        "Work segment %s reported success but endpoint not reached",
        seg.id.c_str());
    }
  }

  if (ok) {
    // Commit the real swept pass of this work row into the CoverageGrid:
    // from the row-start pose to the robot's current (end) pose.  Nav2
    // transitions never sweep (R3/R9).
    if (mode == 3 && tracker_ && work_row_start_pose_) {
      tunnel_map_core::Point2D end_pose;
      if (getRobotPose(end_pose)) {
        tracker_->addSweepSegment(*work_row_start_pose_, end_pose);
      }
    }
    work_row_start_pose_.reset();
    core_->markCovered(static_cast<std::size_t>(idx));
    RCLCPP_INFO(get_logger(), "Segment %s -> COVERED", seg.id.c_str());
    return;
  }

  work_row_start_pose_.reset();

  // Transition (mode 2) or work (mode 3) failure: bounded retry.
  if (exec_attempt_ < max_attempts_per_segment_) {
    ++exec_attempt_;
    RCLCPP_WARN(get_logger(),
      "Segment %s failed (attempt %d/%d) — retrying",
      seg.id.c_str(), exec_attempt_, max_attempts_per_segment_);
    // Segment stays PENDING; next tick re-sends it.
    return;
  }
  core_->markFailed(static_cast<std::size_t>(idx));
  if (failure_class_.empty()) {
    failure_class_ = mode == 3 ? "WORK_TRACKING_FAILED" : "TRANSITION_FAILED";
  }
  RCLCPP_ERROR(get_logger(), "Segment %s -> FAILED", seg.id.c_str());
}

// ── Child-goal senders ──────────────────────────────────────────────────

void CoverageExecutorNode::sendNavigate(
  const tunnel_coverage_planner::CoverageSegment & seg)
{
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(0))) {
    RCLCPP_ERROR(get_logger(), "NavigateToPose server not ready");
    pending_outcome_ = ChildOutcome{exec_index_, false};
    return;
  }
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.header.frame_id = global_frame_;
  goal.pose.header.stamp = now();
  goal.pose.pose.position.x = seg.end_x;
  goal.pose.pose.position.y = seg.end_y;
  goal.pose.pose.position.z = 0.0;
  const double yaw = seg.end_yaw;
  goal.pose.pose.orientation.z = std::sin(yaw * 0.5);
  goal.pose.pose.orientation.w = std::cos(yaw * 0.5);

  auto send_opts = NavClient::SendGoalOptions();
  send_opts.goal_response_callback =
    [this](const NavClient::GoalHandle::SharedPtr & gh) {nav_gh_ = gh;};
  send_opts.result_callback =
    [this, idx = exec_index_](
    const NavClient::GoalHandle::WrappedResult & r)
    {
      const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED;
      if (!ok && phase_ != PHASE_CANCELLING) {
        RCLCPP_WARN(get_logger(), "NavigateToPose finished code=%d",
          static_cast<int>(r.code));
      }
      nav_gh_.reset();
      child_sent_ = false;
      if (phase_ != PHASE_CANCELLING) {
        pending_outcome_ = ChildOutcome{idx, ok};
      }
    };
  nav_gh_.reset();
  child_sent_ = true;
  nav_client_->async_send_goal(goal, send_opts);
}

void CoverageExecutorNode::sendFollow(
  const tunnel_coverage_planner::CoverageSegment & seg)
{
  if (!follow_client_->wait_for_action_server(std::chrono::seconds(0))) {
    RCLCPP_ERROR(get_logger(), "FollowPath server not ready");
    pending_outcome_ = ChildOutcome{exec_index_, false};
    return;
  }
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = now();
  const double len = seg.lengthM();
  const int n = std::max(2, static_cast<int>(std::ceil(len / 0.10)));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = seg.start_x + t * (seg.end_x - seg.start_x);
    ps.pose.position.y = seg.start_y + t * (seg.end_y - seg.start_y);
    ps.pose.position.z = 0.0;
    const double yaw = seg.start_yaw;
    ps.pose.orientation.z = std::sin(yaw * 0.5);
    ps.pose.orientation.w = std::cos(yaw * 0.5);
    path.poses.push_back(std::move(ps));
  }

  auto goal = nav2_msgs::action::FollowPath::Goal();
  goal.path = std::move(path);
  goal.controller_id = "";

  auto send_opts = FollowClient::SendGoalOptions();
  send_opts.goal_response_callback =
    [this](const FollowClient::GoalHandle::SharedPtr & gh)
    {follow_gh_ = gh;};
  send_opts.result_callback =
    [this, idx = exec_index_](
    const FollowClient::GoalHandle::WrappedResult & r)
    {
      const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED;
      if (!ok && phase_ != PHASE_CANCELLING) {
        RCLCPP_WARN(get_logger(), "FollowPath finished code=%d",
          static_cast<int>(r.code));
      }
      follow_gh_.reset();
      child_sent_ = false;
      if (phase_ != PHASE_CANCELLING) {
        pending_outcome_ = ChildOutcome{idx, ok};
      }
    };
  follow_gh_.reset();
  child_sent_ = true;
  follow_client_->async_send_goal(goal, send_opts);
}

// ── TF sampling ─────────────────────────────────────────────────────────

bool CoverageExecutorNode::getRobotPose(tunnel_map_core::Point2D & pose) const
{
  try {
    const auto ts = tf_buffer_.lookupTransform(
      global_frame_, base_frame_, tf2::TimePointZero,
      std::chrono::milliseconds(80));
    const double age = (now() - ts.header.stamp).seconds();
    if (age > max_tf_age_s_) {
      return false;
    }
    pose.x = ts.transform.translation.x;
    pose.y = ts.transform.translation.y;
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "TF lookup failed: %s", e.what());
    return false;
  }
}

void CoverageExecutorNode::tfSamplerCallback()
{
  // Coverage passes are batched per executed work row in processOutcome():
  // stamping every 10 Hz sample would turn interpolation overlap inside one
  // row into fake "repeat coverage".  This sampler remains as a hook for
  // stall/telemetry logic; it does not write the CoverageGrid (R3).
  (void)0;
}

// ── Cancelling ──────────────────────────────────────────────────────────

void CoverageExecutorNode::tickCancelling()
{
  if (!cancel_started_) {
    cancel_started_ = true;
    cancel_start_time_ = now();
    // Cancel the unique child goal (R21).
    if (nav_gh_) {
      nav_client_->async_cancel_goal(nav_gh_);
    }
    if (follow_gh_) {
      follow_client_->async_cancel_goal(follow_gh_);
    }
    return;
  }

  // Stop confirmation via TF displacement between consecutive ticks.
  tunnel_map_core::Point2D pose;
  const bool have_pose = getRobotPose(pose);
  const bool moving = have_pose && last_cancel_pose_ &&
    dist2d(pose.x, pose.y, last_cancel_pose_->x, last_cancel_pose_->y) >
    stop_velocity_threshold_ * 0.2;
  last_cancel_pose_ =
    have_pose ? std::optional<tunnel_map_core::Point2D>(pose) : std::nullopt;

  const double elapsed = (now() - cancel_start_time_).seconds();
  if (!child_sent_ && !moving && elapsed > 0.5) {
    saveCheckpoint("cancel");
    terminal_result_ = map_changed_ ? RESULT_MAP_CHANGED : RESULT_CANCELLED;
    if (map_changed_) {
      failure_class_ = "MAP_CHANGED";
    }
    finishTask();
    return;
  }
  if (elapsed > stop_confirm_timeout_s_) {
    RCLCPP_ERROR(get_logger(), "Stop confirmation timeout");
    saveCheckpoint("stop-timeout");
    terminal_result_ = RESULT_STOP_FAILED;
    failure_class_ = "STOP_CONFIRMATION_TIMEOUT";
    finishTask();
  }
}

// ── Checkpoint ──────────────────────────────────────────────────────────

void CoverageExecutorNode::saveCheckpoint(const std::string & reason)
{
  if (!core_ || !tracker_ || !plan_) {
    return;
  }
  try {
    std::filesystem::create_directories(checkpoint_dir_);
    CheckpointData cp;
    cp.task_input_id = task_input_id_;
    cp.plan_id = plan_id_;
    cp.map_digest = frozen_map_digest_.value_or("");
    for (const auto & s : plan_->segments) {
      cp.segment_ids.push_back(s.id);
    }
    for (std::size_t i = 0; i < plan_->segments.size(); ++i) {
      cp.dispositions.push_back(core_->disposition(i));
    }
    cp.visit_counts = tracker_->visitCounts();
    cp.path_length_m = tracker_->metrics().path_length_m;
    cp.task_duration_s = (now() - task_start_time_).seconds();
    const std::string path = checkpoint_dir_ + "/" + task_input_id_ +
      "_" + plan_id_ + ".cp";
    CheckpointStore::save(path, cp);
    RCLCPP_INFO(get_logger(), "Checkpoint saved (%s): %s",
      reason.c_str(), path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Checkpoint save failed: %s", e.what());
  }
}

// ── Terminal ────────────────────────────────────────────────────────────

void CoverageExecutorNode::finishTask()
{
  sampling_enabled_ = false;
  task_duration_s_ = (now() - task_start_time_).seconds();
  setPhase(PHASE_TERMINAL);
  publishStatus();

  if (current_goal_) {
    auto result = std::make_shared<CovAction::Result>();
    result->terminal_result = terminal_result_;
    const auto m = tracker_ ?
      tracker_->metrics() : tunnel_coverage_planner::CoverageMetrics{};
    result->gross_coverage = m.gross_coverage;
    result->effective_coverage = m.effective_coverage;
    result->exempt_ratio = m.exempt_ratio;
    result->repeat_ratio = m.repeat_ratio;
    result->path_length_m = m.path_length_m;
    result->duration_s = task_duration_s_;
    result->segments_total = static_cast<int32_t>(
      core_ ? static_cast<int32_t>(core_->size()) : 0);
    result->segments_covered = static_cast<int32_t>(
      core_ ? static_cast<int32_t>(core_->countCovered()) : 0);
    result->segments_exempt = static_cast<int32_t>(
      core_ ? static_cast<int32_t>(core_->countExempt()) : 0);
    result->segments_failed = static_cast<int32_t>(
      core_ ? static_cast<int32_t>(core_->countFailed()) : 0);
    result->segments_pending = static_cast<int32_t>(
      core_ ? static_cast<int32_t>(core_->countPending()) : 0);
    result->failure_class = failure_class_;
    result->task_input_id = task_input_id_;
    result->plan_id = plan_id_;
    if (terminal_result_ == RESULT_CANCELLED ||
      terminal_result_ == RESULT_MAP_CHANGED ||
      terminal_result_ == RESULT_STOP_FAILED)
    {
      (*current_goal_)->canceled(result);
    } else {
      (*current_goal_)->succeed(result);
    }
    RCLCPP_INFO(get_logger(), "Task finished: %s (class=%s)",
      CoverageTaskCore::terminalName(terminal_result_).c_str(),
      failure_class_.c_str());
  }
  task_active_ = false;
  current_goal_.reset();
  cancel_started_ = false;
  last_cancel_pose_.reset();
  last_tool_pose_.reset();
  work_row_start_pose_.reset();
  masks_.reset();
  plan_.reset();
  core_.reset();
  tracker_.reset();
  setPhase(PHASE_READY_IDLE);
}

// ── Status ──────────────────────────────────────────────────────────────

void CoverageExecutorNode::setPhase(int phase)
{
  phase_ = phase;
}

void CoverageExecutorNode::publishStatus()
{
  tunnel_coverage_msgs::msg::CoverageStatus s;
  s.phase = phase_;
  s.task_input_id = task_input_id_;
  s.plan_id = plan_id_;
  s.map_digest = frozen_map_digest_.value_or("");
  if (core_) {
    s.segments_total = static_cast<int32_t>(core_->size());
    s.segments_covered = static_cast<int32_t>(core_->countCovered());
    s.segments_exempt = static_cast<int32_t>(core_->countExempt());
    s.segments_failed = static_cast<int32_t>(core_->countFailed());
    s.segments_pending = static_cast<int32_t>(core_->countPending());
    s.segments_blocked_temp = static_cast<int32_t>(core_->countBlockedTemp());
  }
  if (tracker_) {
    const auto m = tracker_->metrics();
    s.gross_coverage = m.gross_coverage;
    s.effective_coverage = m.effective_coverage;
    s.exempt_ratio = m.exempt_ratio;
    s.repeat_ratio = m.repeat_ratio;
    s.path_length_m = m.path_length_m;
  }
  s.last_update_seconds = (now() - task_start_time_).seconds();
  status_pub_->publish(s);
}

}  // namespace tunnel_coverage_executor
