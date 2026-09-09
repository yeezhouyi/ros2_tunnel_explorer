"""Structural regression tests for executor review fixes (2026-09-09).

The review behaviour itself is unit-tested by the C++ gtests that drive the
same seams the node uses (test_stop_confirm drives classifyStopSampleTimed
from stop_confirm.hpp; test_child_goal_tracker drives ChildGoalTracker from
child_goal_tracker.hpp).  These Python tests therefore only PIN THE NODE
WIRING -- that the ROS callbacks really route through those seams and really
call the transport (async_cancel_goal / noteCancelSent) when the policy says
so.  They must not re-assert implementation text that has been deleted.

#5  Stop confirmation requires consecutive VALID samples (translation AND
    rotation); a missing TF sample must reset the streak and can only end in
    STOP_CONFIRMATION_TIMEOUT -- never in a successful cancel on
    "not observed moving".  Node feeds the timed classifier in stop_confirm.hpp.
#6  Every child-goal dispatch/response/result/watchdog event routes through
    ChildGoalTracker; a late goal handle that arrives AFTER a stop was
    requested must be cancelled immediately on the transport
    (ResponseAction::kStoreAndCancel -> async_cancel_goal), not stored.
    A goal the server REFUSES (null goal_response handle) is routed to
    ResponseAction::kRejectedCurrent BEFORE any cancel branch is reachable,
    so async_cancel_goal is never called on a null handle.
#7  The coverage bar applies to BOTH success classes: a below-bar effective
    coverage collapses SUCCEEDED_WITH_EXEMPTIONS to PARTIAL_FAILED instead
    of letting exempt_ratio excuse the miss.
"""
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
NODE_CPP = REPO / "tunnel_coverage_executor/src/coverage_executor_node.cpp"
NODE_HPP = (
    REPO / "tunnel_coverage_executor/include/tunnel_coverage_executor"
          / "coverage_executor_node.hpp"
)
STOP_CONFIRM_HPP = (
    REPO / "tunnel_coverage_executor/include/tunnel_coverage_executor"
          / "stop_confirm.hpp"
)
CHILD_TRACKER_HPP = (
    REPO / "tunnel_coverage_executor/include/tunnel_coverage_executor"
          / "child_goal_tracker.hpp"
)
TASK_CORE_CPP = (
    REPO / "tunnel_coverage_executor/src/coverage_task_core.cpp"
)


def _text(path):
    return path.read_text(encoding="utf-8")


# ── #5: stop confirmation needs consecutive valid samples ────────────────

def test_cancel_requires_consecutive_quiet_samples():
    src = _text(NODE_CPP)
    # The old "no observation => not moving => cancel ok" pattern is gone.
    assert "!moving && elapsed > 0.5" not in src
    # New logic lives in the timed classifier seam; the node feeds it real
    # stamp intervals and keeps a quiet-streak accumulator.
    assert "classifyStopSampleTimed(" in _text(STOP_CONFIRM_HPP)
    assert "classifyStopSampleTimed(" in src
    assert "stop_samples_quiet_" in src
    assert "getRobotState(cur)" in src
    assert "stop_confirm_samples_" in src


def test_cancel_checks_rotation_too():
    src = _text(NODE_CPP)
    # Translation-only check was the review gap; yaw is sampled & bounded.
    assert "stop_yaw_threshold_radps_" in src
    assert "state.yaw = std::atan2" in src


def test_cancel_observation_gap_resets_streak():
    src = _text(NODE_CPP)
    # A missing sample (have == false => cls == -1) resets the counter.
    assert "stop_samples_quiet_ = (cls == 0) ? (stop_samples_quiet_ + 1) : 0" in src


# ── #6: child-goal lifecycle routes through the tracker seam ─────────────

def test_child_goal_tracker_is_the_dispatch_source():
    hpp = _text(NODE_HPP)
    seam = _text(CHILD_TRACKER_HPP)
    assert "child_goal_tracker.hpp" in hpp
    assert "ChildGoalTracker child_tracker_;" in hpp
    # The policy single-source exposes the late-handle cancel decision.
    assert "enum class ResponseAction" in seam
    assert "kStoreAndCancel" in seam


def test_navigate_dispatch_and_response_route_through_tracker():
    src = _text(NODE_CPP)
    start = src.index("void CoverageExecutorNode::sendNavigate(")
    end = src.index("void CoverageExecutorNode::sendFollow(")
    nav_region = src[start:end]
    assert "child_tracker_.dispatch()" in nav_region
    assert "child_tracker_.onGoalResponse(gen, gh != nullptr)" in nav_region
    assert "Stale NavigateToPose result ignored" in nav_region


def test_navigate_late_handle_after_cancel_is_cancelled_on_transport():
    src = _text(NODE_CPP)
    start = src.index("void CoverageExecutorNode::sendNavigate(")
    end = src.index("void CoverageExecutorNode::sendFollow(")
    nav_region = src[start:end]
    # The response callback must map kStoreAndCancel to a REAL transport
    # cancel of the late handle -- not merely record internal state.
    assert "ResponseAction::kStoreAndCancel" in nav_region
    assert "nav_client_->async_cancel_goal(gh)" in nav_region
    assert "child_tracker_.noteCancelSent()" in nav_region


def test_follow_dispatch_and_late_handle_cancel_route_through_tracker():
    src = _text(NODE_CPP)
    start = src.index("void CoverageExecutorNode::sendFollow(")
    tail = src[start:]
    assert "child_tracker_.dispatch()" in tail
    assert "child_tracker_.onGoalResponse(gen, gh != nullptr)" in tail
    assert "ResponseAction::kStoreAndCancel" in tail
    assert "follow_client_->async_cancel_goal(gh)" in tail
    assert "child_tracker_.noteCancelSent()" in tail
    assert "Stale FollowPath result ignored" in tail


def test_cancel_phase_remembers_stop_request_for_a_late_handle():
    src = _text(NODE_CPP)
    idx = src.index("void CoverageExecutorNode::tickCancelling(")
    body = src[idx:]
    # tickCancelling's first tick: remember the stop even without a known
    # handle (requestCancel) and register every delivered transport cancel.
    assert "child_tracker_.requestCancel()" in body
    assert "child_tracker_.noteCancelSent()" in body


def test_watchdog_abandonment_invalidates_inflight_callbacks():
    src = _text(NODE_CPP)
    idx = src.index("forcing failure")
    force = src[idx:idx + 600]
    assert "child_tracker_.abandon()" in force


def test_watchdog_cancel_without_handle_requests_late_handle_cancel():
    src = _text(NODE_CPP)
    idx = src.index("Child goal timeout after")
    body = src[idx:idx + 1400]
    # The timeout watchdog may also hit a goal whose response never arrived:
    # it must remember the stop request so the late handle is cancelled.
    assert "child_tracker_.requestCancel()" in body
    assert "child_tracker_.noteCancelSent()" in body


def test_rejected_goal_is_handled_before_any_transport_cancel():
    """A refused goal (null goal_response handle) must be routed to
    kRejectedCurrent -- never to kStoreAndCancel -- so async_cancel_goal
    cannot dereference a null handle.  Behaviour is unit-tested in
    test_child_goal_tracker (same seam); this pins the node's wiring."""
    src = _text(NODE_CPP)
    for name, nxt, handle, client in [
        ("void CoverageExecutorNode::sendNavigate(",
         "void CoverageExecutorNode::sendFollow(",
         "nav_gh_", "nav_client_"),
        ("void CoverageExecutorNode::sendFollow(",
         None,
         "follow_gh_", "follow_client_"),
    ]:
        start = src.index(name)
        end = src.index(nxt) if nxt else len(src)
        region = src[start:end]
        # null vs live handle is what the tracker decision is keyed on
        assert "child_tracker_.onGoalResponse(gen, gh != nullptr)" in region
        assert "ResponseAction::kRejectedCurrent" in region
        i_rej = region.index("case ResponseAction::kRejectedCurrent")
        i_sac = region.index("case ResponseAction::kStoreAndCancel")
        i_store = region.index("case ResponseAction::kStoreOnly", i_sac)
        # rejection is decided BEFORE the cancel branch is even reachable
        assert i_rej < i_sac < i_store
        # the rejection case clears the wait / queues the failure and
        # contains NO transport cancel of the (null) handle
        rej_block = region[i_rej:i_sac]
        assert "async_cancel_goal" not in rej_block
        assert "pending_outcome_ = ChildOutcome{idx, false}" in rej_block
        assert handle + ".reset()" in rej_block
        # the only cancel call site lives in the accepted kStoreAndCancel case
        sac_block = region[i_sac:i_store]
        assert client + "->async_cancel_goal(gh)" in sac_block
        assert "child_tracker_.noteCancelSent()" in sac_block


# ── #7: coverage bar applies to both success classes ─────────────────────

def test_result_gate_uses_resolve_coverage_gate():
    src = _text(NODE_CPP)
    gate_region = src[src.index("resolveCoverageGate"):src.index("saveCheckpoint")]
    assert "resolveCoverageGate(" in gate_region
    # Old exemption re-branding is removed.
    assert "exempt_ratio > 1e-9 ?" not in src


def test_resolve_coverage_gate_collapses_exemption_success_below_bar():
    core = _text(TASK_CORE_CPP)
    assert "resolveCoverageGate" in core
    # The money line: below bar => partial failure, exemptions cannot excuse.
    assert "return RESULT_PARTIAL_FAILED;" in core
