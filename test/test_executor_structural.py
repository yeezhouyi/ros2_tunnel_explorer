"""Structural regression tests for executor review fixes (2026-09-09).

These tests pin that the three reviewed fixes stay in place by scanning the
executor sources, because the fixes themselves live inside ROS node
callbacks / state machines that are impractical to drive in a plain gtest:

#5  Stop confirmation requires consecutive VALID samples (translation AND
    rotation); a missing TF sample must reset the streak and can only end in
    STOP_CONFIRMATION_TIMEOUT — never in a successful cancel on
    "not observed moving".
#6  Every child-goal dispatch carries a generation; async goal/result
    callbacks reject stale generations so a late result from a
    watchdog-abandoned goal cannot clobber a new task.
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
    # New logic: per-tick classification + quiet-streak accumulator.
    assert "classifyStopSample(" in src
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


# ── #6: generation guard on child-goal callbacks ─────────────────────────

def test_dispatch_generation_member_exists():
    hpp = _text(NODE_HPP)
    assert "std::uint64_t child_gen_ = 0;" in hpp


def test_navigate_callbacks_reject_stale_generation():
    src = _text(NODE_CPP)
    start = src.index("void CoverageExecutorNode::sendNavigate(")
    end = src.index("void CoverageExecutorNode::sendFollow(")
    nav_region = src[start:end]
    assert "++child_gen_;" in nav_region
    assert "gen != child_gen_" in nav_region  # goal-response guard
    assert "Stale NavigateToPose result ignored" in nav_region


def test_follow_callbacks_reject_stale_generation():
    src = _text(NODE_CPP)
    assert "Stale FollowPath result ignored" in src
    assert src.count("gen != child_gen_") >= 4  # resp+result for nav & follow


def test_watchdog_abandonment_invalidates_inflight_callbacks():
    src = _text(NODE_CPP)
    # The forced-failure branch bumps the generation so any late callback of
    # the abandoned goal is rejected instead of being accepted as current.
    idx = src.index("forcing failure")
    force = src[idx:idx + 600]
    assert "++child_gen_;" in force


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
