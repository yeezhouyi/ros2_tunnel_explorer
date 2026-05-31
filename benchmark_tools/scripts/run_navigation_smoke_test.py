#!/usr/bin/env python3
# Copyright 2026 zhouyi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Stage 0B: Navigation Functional Smoke Test.

Reads a set of goal poses from a YAML file, sends them sequentially via
the Nav2 NavigateToPose action server, and reports success/failure per
goal along with aggregate statistics.

Usage:
    ros2 run benchmark_tools run_navigation_smoke_test.py \
        --goals /path/to/stage0b_goals.yaml \
        --output-dir /tmp/stage0b_results

Compatible with ROS2 Jazzy nav2_msgs/NavigateToPose action.
"""

import argparse
from datetime import datetime
import json
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

try:
    import yaml
except ImportError:
    yaml = None

from benchmark_tools.stats import compute_navigation_summary


def load_goals(yaml_path):
    """
    Load goal definitions from a YAML file.

    Expected format:
        goals:
          - description: "Forward 1m"
            frame_id: "map"
            position: {x: 1.0, y: 0.0, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    """
    if yaml is None:
        raise RuntimeError('PyYAML is required: pip install PyYAML')

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    if not data or 'goals' not in data:
        raise ValueError(
            'YAML file must contain a "goals" top-level list'
        )

    return data['goals']


_STATUS_LABELS = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
}


def _status_label(code):
    return _STATUS_LABELS.get(code, 'UNKNOWN')


class NavigationSmokeTest(Node):
    """Sends goals from a YAML file to Nav2 and records results."""

    def __init__(self, goals, output_dir):
        super().__init__('navigation_smoke_test')
        self.goals = goals
        self.output_dir = output_dir
        self.results = []

        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._current_goal_index = 0
        self._current_result = None
        self._goal_start_time = None
        self._goal_handle = None

        self.get_logger().info(
            'Navigation smoke test loaded %d goals' % len(goals)
        )

    def run(self):
        """Execute all goals sequentially and write reports."""
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self._client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(
                'NavigateToPose action server not available after 30s'
            )
            # Still write partial report
            self._finish()
            return

        self.get_logger().info('Action server ready. Starting goals.')
        self._send_next_goal()

    def _send_next_goal(self):
        if self._current_goal_index >= len(self.goals):
            self._finish()
            return

        goal_def = self.goals[self._current_goal_index]
        desc = goal_def.get('description', 'goal %d' % self._current_goal_index)
        self.get_logger().info(
            '[%d/%d] Sending: %s' % (
                self._current_goal_index + 1, len(self.goals), desc
            )
        )

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = goal_def.get('frame_id', 'map')
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal_def['position']['x']
        pose.pose.position.y = goal_def['position']['y']
        pose.pose.position.z = goal_def['position'].get('z', 0.0)
        pose.pose.orientation.x = goal_def['orientation'].get('x', 0.0)
        pose.pose.orientation.y = goal_def['orientation'].get('y', 0.0)
        pose.pose.orientation.z = goal_def['orientation'].get('z', 0.0)
        pose.pose.orientation.w = goal_def['orientation'].get('w', 1.0)
        goal_msg.pose = pose

        self._goal_start_time = time.time()
        send_goal_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        send_goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by action server')
            self._record_result('REJECTED', error_msg='Goal rejected')
            self._current_goal_index += 1
            self._send_next_goal()
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status = _status_label(result.status)
        duration = time.time() - (self._goal_start_time or time.time())

        error_msg = None
        if status == 'ABORTED':
            if result.result and hasattr(result.result, 'result'):
                error_msg = str(result.result.result)
            else:
                error_msg = 'ABORTED (no detail)'

        self._record_result(
            status, duration_s=duration, error_msg=error_msg
        )
        self._current_goal_index += 1
        self._send_next_goal()

    def _feedback_cb(self, feedback_msg):
        # Optional: log distance remaining or other feedback
        pass

    def _record_result(self, status, duration_s=0.0, error_msg=None):
        goal_def = self.goals[self._current_goal_index]
        self.results.append({
            'index': self._current_goal_index,
            'description': goal_def.get('description', ''),
            'pose': {
                'x': goal_def['position']['x'],
                'y': goal_def['position']['y'],
                'frame_id': goal_def.get('frame_id', 'map'),
            },
            'status': status,
            'duration_s': round(duration_s, 1),
            'error_msg': error_msg,
        })
        self.get_logger().info(
            '  Result: %s (%.1fs)' % (status, duration_s)
        )

    def _finish(self):
        summary = compute_navigation_summary(self.results)
        self._write_reports(summary)
        self.get_logger().info('Smoke test complete.')
        raise SystemExit(0)

    def _write_reports(self, summary):
        os.makedirs(self.output_dir, exist_ok=True)

        report = {
            'timestamp': datetime.now().isoformat(),
            'source_goals': self._get_source_description(),
            'summary': summary,
        }

        json_path = os.path.join(self.output_dir, 'stage0b_results.json')
        with open(json_path, 'w') as f:
            json.dump(report, f, indent=2)

        md_path = os.path.join(self.output_dir, 'stage0b_results.md')
        with open(md_path, 'w') as f:
            f.write('# Stage 0B: Navigation Smoke Test Report\n\n')
            f.write('- **Report generated**: %s\n' % report['timestamp'])
            f.write('- **Source goals**: %s\n\n' % report['source_goals'])

            s = summary
            f.write('## Summary\n\n')
            f.write('| Metric | Value |\n')
            f.write('|--------|-------|\n')
            f.write('| Total goals | %d |\n' % s['total'])
            f.write('| Succeeded | %d |\n' % s['succeeded'])
            f.write('| Failed (ABORTED) | %d |\n' % s['failed'])
            f.write('| Canceled | %d |\n' % s['canceled'])
            f.write('| Unknown | %d |\n' % s['unknown'])
            f.write('| Success rate | %s |\n'
                     % ('%.1f%%' % (s['success_rate'] * 100)
                        if s['success_rate'] is not None else 'N/A'))
            f.write('| Total duration | %.1fs |\n' % s['total_duration_s'])
            f.write('| Mean duration per goal | %.1fs |\n\n'
                     % s['mean_duration_s'])

            f.write('## Per-Goal Results\n\n')
            f.write('| # | Description | Status | Duration (s) | Error |\n')
            f.write('|---|-------------|--------|--------------|-------|\n')
            for r in self.results:
                err = r.get('error_msg') or ''
                f.write('| %d | %s | %s | %.1f | %s |\n' % (
                    r['index'], r['description'], r['status'],
                    r['duration_s'], err
                ))

        self.get_logger().info('JSON: %s' % json_path)
        self.get_logger().info('Markdown: %s' % md_path)

    def _get_source_description(self):
        return 'YAML config (inline)' if hasattr(self, 'goals') else 'unknown'


_NODE = None


def main():
    parser = argparse.ArgumentParser(
        description='Stage 0B Navigation Functional Smoke Test'
    )
    parser.add_argument(
        '--goals', type=str,
        default=os.path.join(
            os.path.dirname(__file__), '..', 'config', 'stage0b_goals.yaml'
        ),
        help='Path to YAML goal definitions'
    )
    parser.add_argument(
        '--output-dir', type=str, default='/tmp/stage0b_results',
        help='Output directory for reports'
    )
    args = parser.parse_args()

    # Resolve goals relative to script location if not absolute
    goals_path = args.goals
    if not os.path.isabs(goals_path):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        goals_path = os.path.normpath(os.path.join(script_dir, goals_path))

    if not os.path.exists(goals_path):
        print('ERROR: Goals file not found: %s' % goals_path, file=sys.stderr)
        sys.exit(1)

    try:
        goals = load_goals(goals_path)
    except Exception as e:
        print('ERROR loading goals: %s' % e, file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=sys.argv)
    global _NODE
    _NODE = NavigationSmokeTest(goals, args.output_dir)
    try:
        _NODE.run()
        rclpy.spin(_NODE)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        _NODE.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
