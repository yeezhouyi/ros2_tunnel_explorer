#!/usr/bin/env python3
"""B4 bridge (record side): capture the explorer's executed path as
nav_msgs/Path and persist it to a replayable file.

Subscribes to /odom (robot pose) and optionally /navigate_to_pose/_action/status
(goal events are logged into the file for provenance).  On shutdown (SIGINT)
or --duration, writes a JSON file:

    {"frame_id": ..., "poses": [[x, y, yaw, t], ...],
     "goals": [[x, y, t], ...], "source": "explorer_run"}

The companion replay_path_mpc.py feeds this Path into the MPC reference core
for offline tracking evaluation (plan R14: explorer only provides a path).
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class PathRecorder(Node):
    def __init__(self, topic: str, goal_topic: str, min_step_m: float):
        super().__init__("record_explorer_path")
        self.poses = []
        self.goals = []
        self.min_step_m = min_step_m
        self.t0 = time.monotonic()
        self.last_xy = None
        self.create_subscription(Odometry, topic, self.on_odom, 20)
        self.create_subscription(Path, goal_topic, self.on_goal_path, 5)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        xy = (p.x, p.y)
        if self.last_xy is not None and math.hypot(xy[0] - self.last_xy[0],
                                                   xy[1] - self.last_xy[1]) < self.min_step_m:
            return
        self.last_xy = xy
        self.poses.append([p.x, p.y, yaw, round(time.monotonic() - self.t0, 3)])

    def on_goal_path(self, msg: Path):
        for ps in msg.poses:
            self.goals.append([ps.pose.position.x, ps.pose.position.y,
                               round(time.monotonic() - self.t0, 3)])

    def dump(self, path: str, frame: str):
        with open(path, "w") as f:
            json.dump({"frame_id": frame, "poses": self.poses,
                       "goals": self.goals,
                       "source": "explorer_run",
                       "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")},
                      f, indent=1)
        return len(self.poses)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--odom-topic", default="/odom")
    ap.add_argument("--goal-path-topic", default="/plan",
                    help="optional topic carrying the planner nav_msgs/Path")
    ap.add_argument("--frame", default="map")
    ap.add_argument("--min-step-m", type=float, default=0.05)
    ap.add_argument("--duration", type=float, default=0.0,
                    help="stop after this many seconds (0 = until SIGINT)")
    ap.add_argument("--output", required=True)
    args = ap.parse_args()

    rclpy.init()
    node = PathRecorder(args.odom_topic, args.goal_path_topic, args.min_step_m)

    def on_sigint(sig, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_sigint)
    start = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if args.duration > 0 and time.monotonic() - start > args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        n = node.dump(args.output, args.frame)
        node.get_logger().info("wrote %d poses, %d goals -> %s" % (
            n, len(node.goals), args.output))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
