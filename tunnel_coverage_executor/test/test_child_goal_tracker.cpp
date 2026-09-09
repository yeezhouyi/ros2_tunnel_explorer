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

// Behavior tests for the executor's async-cancel lifecycle policy (review
// P1-2).  These drive the SAME ChildGoalTracker the node now uses -- not a
// mirror of it -- over the three claimed behaviours:
//   1. a cancel requested before the goal handle was returned can never
//      wedge the executor (watchdog abandons, late response is stale);
//   2. an old goal that times out is asked to stop AND then abandoned; the
//      new task starts; the late old result is ignored by the generation
//      guard and never touches the new task's state -- the "asked to stop"
//      and "late result ignored" are verified as separate mechanisms;
//   3. a cancel that is rejected or stalls (no result ever arrives) is
//      still bounded by timeout + grace and the executor moves on.
#include <cstdint>

#include "gtest/gtest.h"
#include "tunnel_coverage_executor/child_goal_tracker.hpp"

namespace tunnel_coverage_executor
{

namespace
{
constexpr double kTimeout = 60.0;
constexpr double kGrace = 20.0;
}  // namespace

TEST(ChildGoalTracker, WatchBoundariesMirrorTheNode)
{
  ChildGoalTracker t;
  // not sent: no watchdog action even for a huge run
  EXPECT_EQ(t.watch(1e9, kTimeout, kGrace), WatchAction::kNone);
  t.dispatch();
  EXPECT_EQ(t.watch(kTimeout - 0.001, kTimeout, kGrace), WatchAction::kNone);
  EXPECT_EQ(t.watch(kTimeout, kTimeout, kGrace), WatchAction::kNone);
  // strictly past the timeout -> cancel
  EXPECT_EQ(t.watch(kTimeout + 0.001, kTimeout, kGrace), WatchAction::kCancel);
  // at exactly timeout+grace the grace window is still open
  EXPECT_EQ(t.watch(kTimeout + kGrace, kTimeout, kGrace), WatchAction::kCancel);
  // strictly past timeout+grace -> abandon (bounded exit)
  EXPECT_EQ(
    t.watch(kTimeout + kGrace + 0.001, kTimeout, kGrace),
    WatchAction::kAbandon);
}

TEST(ChildGoalTracker, DispatchAdvancesGenerationAndArmsTheWatchdog)
{
  ChildGoalTracker t;
  const std::uint64_t g1 = t.dispatch();
  EXPECT_EQ(t.generation(), g1);
  EXPECT_TRUE(t.sent());
  EXPECT_FALSE(t.handleKnown());
  const std::uint64_t g2 = t.dispatch();
  EXPECT_EQ(g2, g1 + 1);
  EXPECT_FALSE(t.isCurrent(g1));
  EXPECT_TRUE(t.isCurrent(g2));
}

TEST(ChildGoalTracker, ClearBetweenTasksKeepsTheGenerationMonotonic)
{
  ChildGoalTracker t;
  t.dispatch();
  const std::uint64_t g2 = t.dispatch();
  t.clear();   // beginTask: in-flight child state dropped ...
  EXPECT_EQ(t.generation(), g2);   // ... but the generation is NOT reset
  EXPECT_FALSE(t.sent());
  EXPECT_EQ(t.dispatch(), g2 + 1);
}

// P1-2(1): cancel requested before the goal handle was returned.
TEST(ChildGoalTracker, CancelBeforeHandleReturnedNeverWedges)
{
  ChildGoalTracker t;
  const std::uint64_t g = t.dispatch();
  // the server never answered goal_response yet; nothing can be cancelled
  EXPECT_FALSE(t.handleKnown());
  // watchdog keeps asking to cancel inside the grace window ...
  EXPECT_EQ(t.watch(kTimeout + 1.0, kTimeout, kGrace), WatchAction::kCancel);
  // ... and abandons once the grace window is past
  EXPECT_EQ(
    t.watch(kTimeout + kGrace + 1.0, kTimeout, kGrace),
    WatchAction::kAbandon);
  t.abandon();
  EXPECT_FALSE(t.sent());
  EXPECT_FALSE(t.handleKnown());
  // the handle finally arrives for the abandoned goal: stale, dropped
  EXPECT_FALSE(t.onGoalResponse(g));
  EXPECT_FALSE(t.handleKnown());
  // a late result of the abandoned goal is ignored
  EXPECT_EQ(t.onResult(g, false), ChildResult::kStaleIgnored);
  // the executor is free to dispatch the next goal and complete it
  const std::uint64_t g2 = t.dispatch();
  EXPECT_TRUE(t.onGoalResponse(g2));
  EXPECT_TRUE(t.handleKnown());
  EXPECT_EQ(t.onResult(g2, false), ChildResult::kReady);
  EXPECT_FALSE(t.sent());
}

// P1-2(2): old goal timeout -> stop asked + abandoned -> new task starts ->
// the late old result is ignored AND the new task state is untouched.
TEST(ChildGoalTracker, LateOldResultIgnoredWhileStopWasSeparatelyAsked)
{
  ChildGoalTracker t;
  const std::uint64_t old_gen = t.dispatch();
  ASSERT_TRUE(t.onGoalResponse(old_gen));   // handle known: a cancel CAN go out
  // watchdog: past the timeout -> ask the server to cancel the old goal
  EXPECT_EQ(t.watch(kTimeout + 0.1, kTimeout, kGrace), WatchAction::kCancel);
  t.noteCancelSent();   // the node transport sent async_cancel_goal
  EXPECT_TRUE(t.cancelAsked());   // mechanism 1: the OLD action was stopped
  // the cancel never completes (rejected/stalled) -> bounded abandon
  EXPECT_EQ(
    t.watch(kTimeout + kGrace + 0.1, kTimeout, kGrace),
    WatchAction::kAbandon);
  t.abandon();
  EXPECT_FALSE(t.sent());
  // the executor starts the NEW task (next tick dispatches another goal)
  const std::uint64_t new_gen = t.dispatch();
  EXPECT_TRUE(t.isCurrent(new_gen));
  EXPECT_TRUE(t.sent());
  // the OLD goal's result arrives late: generation guard drops it ...
  EXPECT_EQ(t.onResult(old_gen, false), ChildResult::kStaleIgnored);
  // ... and the drop must NOT touch the new task's state
  EXPECT_TRUE(t.sent());
  EXPECT_TRUE(t.isCurrent(new_gen));
  EXPECT_FALSE(t.handleKnown());   // new goal's response not yet in
  // the new goal completes normally
  EXPECT_EQ(t.onResult(new_gen, false), ChildResult::kReady);
  EXPECT_FALSE(t.sent());
}

// P1-2(3): cancel rejected / stalls (no result ever arrives) is bounded.
TEST(ChildGoalTracker, StalledCancelIsBoundedAndTheExecutorMovesOn)
{
  ChildGoalTracker t;
  t.dispatch();
  // no handle response, no result: the watchdog still fires abandon exactly
  // once the grace window is past
  EXPECT_EQ(
    t.watch(kTimeout + kGrace + 5.0, kTimeout, kGrace),
    WatchAction::kAbandon);
  t.abandon();
  EXPECT_FALSE(t.sent());
  EXPECT_FALSE(t.handleKnown());
  // subsequent dispatch + completion still work
  const std::uint64_t g = t.dispatch();
  EXPECT_EQ(t.onResult(g, false), ChildResult::kReady);
}

TEST(ChildGoalTracker, NormalSuccessAndFailureBothReportReady)
{
  ChildGoalTracker t;
  const std::uint64_t g = t.dispatch();
  ASSERT_TRUE(t.onGoalResponse(g));
  EXPECT_TRUE(t.handleKnown());
  // the caller decides ok from ResultCode; the tracker reports readiness
  EXPECT_EQ(t.onResult(g, false), ChildResult::kReady);
  EXPECT_FALSE(t.sent());
  EXPECT_FALSE(t.handleKnown());
}

// While the TASK is cancelling, a current result clears the transport state
// (so stop-confirmation may proceed) but must NOT queue an outcome.
TEST(ChildGoalTracker, CancellingPhaseClearsWithoutQueuingAnOutcome)
{
  ChildGoalTracker t;
  const std::uint64_t g = t.dispatch();
  ASSERT_TRUE(t.onGoalResponse(g));
  EXPECT_EQ(t.onResult(g, true), ChildResult::kClearedCancelling);
  EXPECT_FALSE(t.sent());
  EXPECT_FALSE(t.handleKnown());
}

}  // namespace tunnel_coverage_executor
