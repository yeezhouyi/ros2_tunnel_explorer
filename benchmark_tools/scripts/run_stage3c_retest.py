#!/usr/bin/env python3
# Copyright 2026 zhouyi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Stage 3C retest driver (plan C6: reproducible experiment for the Stage 3C
# entrance-oscillation failure, run with the Stage 3D recovery stack active).
#
# Protocol per run:
#   1. cleanup_simulation.sh
#   2. launch stage0_simulation.launch.py (branching world, headless)
#   3. record rosbag (/clock /map /tf /odom frontier markers)
#   4. launch frontier_explorer.launch.py (Stage 3D params: recovery ON)
#   5. monitor: completion = frontier markers empty AND no active Nav2 goal
#      for STABLE_WINDOW seconds; otherwise timeout
#   6. write retest_run_NN.json; aggregate all runs at the end
#
# Usage:
#   python3 benchmark_tools/scripts/run_stage3c_retest.py \
#       --runs 5 --outdir ~/stage3c_retest
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time

BRANCH_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WORLD = os.path.join(BRANCH_DIR, "tunnel_worlds", "worlds", "branching_tunnel_y.sdf")
CLEANUP = os.path.join(BRANCH_DIR, "scripts", "cleanup_simulation.sh")
MARKER_TOPIC = "/tunnel_frontier_explorer/frontier_markers"
STABLE_WINDOW_S = 90.0
BAG_TOPICS = ["/clock", "/map", "/tf", "/odom", MARKER_TOPIC]


def sh(cmd: str, **kw) -> subprocess.Popen:
    return subprocess.Popen(["bash", "-c", cmd], **kw)


class RunMonitor:
    """Tracks frontier markers + Nav2 action status to detect completion."""

    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node

        self.rclpy = rclpy
        self.node = Node("stage3c_retest_monitor")
        self.marker_ts: list[float] = []
        self.markers_seen = False
        self.goal_active = False
        self.goal_events = 0
        self.node.create_subscription(
            __import__("visualization_msgs.msg", froml=["MarkerArray"]).MarkerArray
            if False
            else self._marker_type(),
            MARKER_TOPIC,
            self._on_markers,
            10,
        )
        from action_msgs.msg import GoalStatusArray

        self.node.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status", self._on_status, 10
        )

    @staticmethod
    def _marker_type():
        from visualization_msgs.msg import MarkerArray

        return MarkerArray

    def _on_markers(self, msg) -> None:
        now = time.monotonic()
        n_live = sum(1 for m in msg.markers if m.action == 0)  # 0 = ADD/ MODIFY
        if n_live > 0:
            self.markers_seen = True
            self.marker_ts.append(now)

    def _on_status(self, msg) -> None:
        # 1=ACCEPTED,2=EXECUTING => active; terminal >=4
        was = self.goal_active
        self.goal_active = any(s.status in (1, 2) for s in msg.status_list)
        if self.goal_active and not was:
            self.goal_events += 1

    def wait_completion(self, timeout_s: float) -> dict:
        start = time.monotonic()
        last_progress = time.monotonic()
        while self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=1.0)
            now = time.monotonic()
            if self.marker_ts:
                last_progress = max(last_progress, self.marker_ts[-1])
            if self.goal_active:
                last_progress = now
            elapsed = now - start
            if self.markers_seen and now - last_progress >= STABLE_WINDOW_S:
                return {"status": "COMPLETED", "elapsed_s": round(elapsed, 1),
                        "goal_events": self.goal_events}
            if elapsed >= timeout_s:
                return {"status": "TIMEOUT", "elapsed_s": round(elapsed, 1),
                        "goal_events": self.goal_events}
        return {"status": "ERROR", "elapsed_s": -1, "goal_events": self.goal_events}


def one_run(idx: int, outdir: str, timeout_s: float, bag: bool) -> dict:
    import rclpy

    rclpy.init()
    rundir = os.path.join(outdir, f"run_{idx:02d}")
    os.makedirs(rundir, exist_ok=True)
    sh(f"'{CLEANUP}' >/dev/null 2>&1 || true").wait()

    sim = sh(
        f"source /opt/ros/jazzy/setup.bash && source {BRANCH_DIR}/install/setup.bash && "
        f"ros2 launch tunnel_explorer_bringup stage0_simulation.launch.py "
        f"world:='{WORLD}' headless:=True rviz:=False use_composition:=False "
        f"> '{rundir}/simulation.log' 2>&1"
    )
    monitor = RunMonitor()
    # readiness: first /map message or 120 s
    t0 = time.monotonic()
    while not monitor.marker_ts and time.monotonic() - t0 < 120:
        monitor.rclpy.spin_once(monitor.node, timeout_sec=1.0)

    bagp = None
    if bag:
        bagp = sh(
            f"source /opt/ros/jazzy/setup.bash && ros2 bag record -o '{rundir}/bag' "
            + " ".join(BAG_TOPICS)
            + f" > '{rundir}/bag.log' 2>&1"
        )

    expl = sh(
        f"source /opt/ros/jazzy/setup.bash && source {BRANCH_DIR}/install/setup.bash && "
        f"ros2 launch tunnel_frontier_explorer frontier_explorer.launch.py "
        f"> '{rundir}/frontier_explorer.log' 2>&1"
    )
    result = monitor.wait_completion(timeout_s)

    for p in (expl, sim, bagp):
        if p:
            p.send_signal(signal.SIGINT)
            try:
                p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                p.kill()
    monitor.node.destroy_node()
    monitor.rclpy.shutdown()
    sh(f"'{CLEANUP}' >/dev/null 2>&1 || true").wait()

    result["run"] = idx
    with open(os.path.join(rundir, "retest_result.json"), "w") as f:
        json.dump(result, f, indent=1)
    return result


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--timeout", type=float, default=1200.0)
    ap.add_argument("--outdir", default=os.path.expanduser("~/stage3c_retest"))
    ap.add_argument("--no-bag", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    results = [one_run(i + 1, args.outdir, args.timeout, not args.no_bag) for i in range(args.runs)]
    n_ok = sum(1 for r in results if r["status"] == "COMPLETED")
    summary = {
        "protocol": "stage3c_retest_with_stage3d_recovery",
        "world": os.path.basename(WORLD),
        "stable_window_s": STABLE_WINDOW_S,
        "timeout_s": args.timeout,
        "completed": n_ok,
        "total": len(results),
        "runs": results,
    }
    with open(os.path.join(args.outdir, "aggregate.json"), "w") as f:
        json.dump(summary, f, indent=1)
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
