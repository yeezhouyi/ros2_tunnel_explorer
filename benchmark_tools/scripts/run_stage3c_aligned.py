#!/usr/bin/env python3
"""Stage 3C aligned 5-run driver (U8, protocol stage3c-aligned-v2).

Unlike the ephemeral original driver (whose timeout/stable-window were
never archived — see stage3c_protocol_aligned_v1.json PROTOCOL_BLOCKED),
this driver DECLARES every protocol value explicitly at startup and writes
them into every run manifest.  The declared values are a NEW freeze (not a
recovery of the lost originals): historical 5/5, 2/5 and the unaligned 1/5
remain separate comparison groups.

Protocol (frozen):
  world        tunnel_worlds/worlds/branching_tunnel_y.sdf
  initial pose x=0.0, y=1.0 (stage0_simulation.launch.py spawn)
  timeout      1500 s per run
  stable window 120 s: frontier markers empty AND no active Nav2 goal
  goals        NavigateToPose via the stage3d explorer stack (recovery ON)
  seeds        0..4
"""
from __future__ import annotations

import argparse
import json
import signal
import subprocess
import time
from pathlib import Path

BRANCH_DIR = Path(__file__).resolve().parents[2]
WORLD = str(BRANCH_DIR / "tunnel_worlds" / "worlds" / "branching_tunnel_y.sdf")
CLEANUP = str(BRANCH_DIR / "scripts" / "cleanup_simulation.sh")
MARKER_TOPIC = "/tunnel_frontier_explorer/frontier_markers"

PROTOCOL = {
    "protocol_version": "stage3c-aligned-v2",
    "declared_by": "author freeze (original driver values unrecoverable)",
    "world": "branching_tunnel_y.sdf",
    "initial_pose": {"x": 0.0, "y": 1.0, "yaw": 0.0},
    "timeout_seconds": 1500,
    "stable_window_s": 120,
    "goal_source": "stage3d explorer stack, entrance-loop recovery ON",
    "seeds": [0, 1, 2, 3, 4],
}


def sh(cmd: str, **kw) -> subprocess.Popen:
    return subprocess.Popen(["bash", "-c", cmd], **kw)


class Monitor:
    def __init__(self):
        # rclpy.init() is owned by the caller (one_run) — a second init
        # here raised Context.init() must only be called once
        import rclpy

        self.rclpy = rclpy
        from rclpy.node import Node

        self.node = Node("stage3c_aligned_monitor")
        from visualization_msgs.msg import MarkerArray
        from action_msgs.msg import GoalStatusArray

        self.marker_ts = []
        self.markers_seen = False
        self.goal_active = False
        self.goal_events = 0
        self.node.create_subscription(
            MarkerArray, MARKER_TOPIC, self._on_markers, 10)
        self.node.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status",
            self._on_status, 10)

    def _on_markers(self, msg):
        now = time.monotonic()
        if any(m.action == 0 for m in msg.markers):
            self.markers_seen = True
            self.marker_ts.append(now)

    def _on_status(self, msg):
        was = self.goal_active
        self.goal_active = any(s.status in (1, 2) for s in msg.status_list)
        if self.goal_active and not was:
            self.goal_events += 1

    def wait_terminal(self, timeout_s: float, stable_s: float) -> dict:
        start = time.monotonic()
        last_progress = start
        while self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=1.0)
            now = time.monotonic()
            if self.marker_ts:
                last_progress = max(last_progress, self.marker_ts[-1])
            if self.goal_active:
                last_progress = now
            elapsed = now - start
            if self.markers_seen and now - last_progress >= stable_s:
                return {"status": "COMPLETED",
                        "elapsed_s": round(elapsed, 1),
                        "goal_events": self.goal_events}
            if elapsed >= timeout_s:
                return {"status": "TIMEOUT",
                        "elapsed_s": round(elapsed, 1),
                        "goal_events": self.goal_events}
        return {"status": "ERROR", "elapsed_s": -1, "goal_events": -1}


def one_run(idx: int, outdir: str, args) -> dict:
    import rclpy

    rclpy.init()
    rundir = Path(outdir) / f"aligned_seed_{idx}"
    rundir.mkdir(parents=True, exist_ok=True)
    sh(f"'{CLEANUP}' >/dev/null 2>&1 || true").wait()

    sim = sh(
        f"source /opt/ros/jazzy/setup.bash && source {BRANCH_DIR}/install/setup.bash && "
        f"ros2 launch tunnel_explorer_bringup stage0_simulation.launch.py "
        f"headless:=True rviz:=False use_composition:=False "
        f"> '{rundir}/simulation.log' 2>&1"
    )
    mon = Monitor()
    t0 = time.monotonic()
    while not mon.marker_ts and time.monotonic() - t0 < 120:
        mon.rclpy.spin_once(mon.node, timeout_sec=1.0)

    bagp = None
    if not args.no_bag:
        bagp = sh(
            f"source /opt/ros/jazzy/setup.bash && ros2 bag record -o "
            f"'{rundir}/bag' /clock /map /odom /tf {MARKER_TOPIC} "
            f"> '{rundir}/bag.log' 2>&1")

    expl = sh(
        f"source /opt/ros/jazzy/setup.bash && source {BRANCH_DIR}/install/setup.bash && "
        f"ros2 launch tunnel_frontier_explorer frontier_explorer.launch.py "
        f"> '{rundir}/frontier_explorer.log' 2>&1")

    result = mon.wait_terminal(args.timeout, args.stable_window)

    for p in (expl, sim, bagp):
        if p:
            p.send_signal(signal.SIGINT)
            try:
                p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                p.kill()
    mon.node.destroy_node()
    mon.rclpy.shutdown()
    sh(f"'{CLEANUP}' >/dev/null 2>&1 || true").wait()

    result["run"] = idx
    result["protocol_version"] = PROTOCOL["protocol_version"]
    with open(rundir / "run_manifest.json", "w") as f:
        json.dump({"protocol": PROTOCOL, "seed": idx, "result": result}, f,
                  indent=1)
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--timeout", type=float, default=PROTOCOL["timeout_seconds"])
    ap.add_argument("--stable-window", type=float,
                    default=PROTOCOL["stable_window_s"])
    ap.add_argument("--outdir", default=str(Path.home() / "stage3c_aligned"))
    ap.add_argument("--no-bag", action="store_true")
    args = ap.parse_args()

    results = [one_run(i + 1, args.outdir, args) for i in range(args.runs)]
    n_ok = sum(1 for r in results if r["status"] == "COMPLETED")
    summary = dict(PROTOCOL)
    summary["timeout_seconds"] = args.timeout
    summary["stable_window_s"] = args.stable_window
    summary["completed"] = n_ok
    summary["total"] = len(results)
    summary["runs"] = results
    out = Path(args.outdir) / "aggregate.json"
    out.write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
