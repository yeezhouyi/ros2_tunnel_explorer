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

// Single source of truth for the executor's child-goal lifecycle policy
// (generation guard + timeout watchdog + cancel bookkeeping).  Extracted
// from CoverageExecutorNode so the async-cancel behaviours the node claims
// are unit-testable without a live Nav2 action server.
//
// The node's own review finding: "generation check != old action stopped".
// Both mechanisms therefore live here as SEPARATE state:
//   * the generation guard decides whether a late callback belongs to the
//     goal the executor is CURRENTLY waiting on (stale => ignored, and a
//     stale result never touches the state of a newer dispatch);
//   * the watchdog/cancel bookkeeping decides whether the OLD action was
//     actually asked to stop (async_cancel was issued while its handle was
//     known) before it was abandoned (handle dropped, generation advanced).
//
// One instance tracks the single logical child goal (nav OR follow).  The
// node still owns the rclcpp handles/timers and the exec/segment state.
#ifndef TUNNEL_COVERAGE_EXECUTOR__CHILD_GOAL_TRACKER_HPP_
#define TUNNEL_COVERAGE_EXECUTOR__CHILD_GOAL_TRACKER_HPP_

#include <cstdint>

namespace tunnel_coverage_executor
{

/// What the result callback must do with an arrived child-goal result.
enum class ChildResult
{
  kReady,             // current goal finished: node consumes the outcome
  kStaleIgnored,      // late result of an abandoned/superseded goal: drop
  kClearedCancelling  // current goal ended while the task is cancelling:
                      // clear transport state, do NOT queue an outcome
};

/// What the timeout watchdog must do this tick.
enum class WatchAction
{
  kNone,
  kCancel,    // run_s > child_goal_timeout_s_: ask the server to cancel
  kAbandon    // run_s > timeout + grace: force-fail, stop waiting forever
};

/// Child-goal lifecycle policy (see file comment).
class ChildGoalTracker
{
public:
  /// beginTask: drop any in-flight child state.  The generation is kept
  /// monotonic so stale callbacks from a previous task stay stale.
  void clear()
  {
    sent_ = false;
    handle_known_ = false;
    cancel_asked_ = false;
  }

  /// Dispatch a new child goal.  Returns the generation the caller must
  /// capture into its callbacks.  Marks the goal as sent (watchdog armed).
  std::uint64_t dispatch()
  {
    ++gen_;
    sent_ = true;
    handle_known_ = false;
    cancel_asked_ = false;
    return gen_;
  }

  std::uint64_t generation() const {return gen_;}
  bool isCurrent(std::uint64_t g) const {return g == gen_;}
  bool sent() const {return sent_;}

  /// True while the transport handle of the current goal is known (the
  /// server answered goal_response).  Only then can a cancel actually be
  /// delivered; a goal whose response never arrived has nothing to cancel
  /// and can only be abandoned by the watchdog.
  bool handleKnown() const {return handle_known_;}

  /// True once async_cancel was actually issued for the current goal
  /// (noteCancelSent is only called on the transport paths that send it).
  bool cancelAsked() const {return cancel_asked_;}

  /// goal_response_callback: store the handle if the response is current.
  /// A response that arrives after an abandon/supersede (cancel requested
  /// before the server answered) is stale and must be dropped.
  bool onGoalResponse(std::uint64_t g)
  {
    if (g != gen_) {
      return false;
    }
    handle_known_ = true;
    return true;
  }

  /// result_callback.  A stale result never mutates state; a current result
  /// clears the transport state and (outside the cancelling phase) lets the
  /// caller consume the outcome.
  ChildResult onResult(std::uint64_t g, bool cancelling_phase)
  {
    if (g != gen_) {
      return ChildResult::kStaleIgnored;
    }
    sent_ = false;
    handle_known_ = false;
    cancel_asked_ = false;
    if (cancelling_phase) {
      return ChildResult::kClearedCancelling;
    }
    return ChildResult::kReady;
  }

  /// Record that async_cancel was delivered for the current goal (called by
  /// the transport paths that actually sent it).
  void noteCancelSent()
  {
    if (sent_) {
      cancel_asked_ = true;
    }
  }

  /// Timeout-watchdog decision (strict inequalities mirror the node).
  WatchAction watch(double run_s, double timeout_s, double grace_s) const
  {
    if (!sent_) {
      return WatchAction::kNone;
    }
    if (run_s > timeout_s + grace_s) {
      return WatchAction::kAbandon;
    }
    if (run_s > timeout_s) {
      return WatchAction::kCancel;
    }
    return WatchAction::kNone;
  }

  /// Watchdog force-fail: stop waiting for the old goal, drop its handle,
  /// advance the generation so every still-in-flight callback is stale.
  void abandon()
  {
    ++gen_;
    sent_ = false;
    handle_known_ = false;
    cancel_asked_ = false;
  }

private:
  std::uint64_t gen_ = 0;
  bool sent_ = false;
  bool handle_known_ = false;
  bool cancel_asked_ = false;
};

}  // namespace tunnel_coverage_executor

#endif  // TUNNEL_COVERAGE_EXECUTOR__CHILD_GOAL_TRACKER_HPP_
