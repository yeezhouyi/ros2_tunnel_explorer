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
Stage 0B-1: Navigation Functional Smoke Test.

Sends goals from a YAML config file to the Nav2 NavigateToPose action
server sequentially. Each goal has an independent timeout; if it expires
the goal is cancelled and, by default, the next goal starts after a brief
cooldown.

Usage:
    ros2 run benchmark_tools run_navigation_smoke_test.py \
        --goals /path/to/stage0b_goals.yaml \
        --goal-timeout-seconds 30 \
        --continue-on-failure true \
        --cooldown-seconds 1.0 \
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
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

try:
    import yaml
except ImportError:
    yaml = None

from benchmark_tools.path_util import resolve_goals_path
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

    def __init__(self, goals, output_dir, goal_timeout=30.0,
                 continue_on_failure=True, cooldown=1.0):
        super().__init__('navigation_smoke_test')
        self.goals = goals
        self.output_dir = output_dir
        self._goal_timeout = goal_timeout
        self._continue_on_failure = continue_on_failure
        self._cooldown = cooldown
        self.results = []

        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._current_goal_index = 0
        self._goal_start_time = None
        self._goal_handle = None
        self._canceled_by_timeout = False

        # State machine: IDLE -> NAVIGATING -> COOLDOWN -> IDLE / FINISHED
        self._state = 'IDLE'
        self._cooldown_until = 0.0

        # Periodic check timer (100 ms) for timeout and cooldown
        self._check_timer = self.create_timer(0.1, self._check_cb)

        self.get_logger().info(
            'Navigation smoke test loaded %d goals' % len(goals)
        )

    def run(self):
        """Start executing goals."""
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self._client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(
                'NavigateToPose action server not available after 30s'
            )
            self._finish()
            return

        self.get_logger().info('Action server ready. Starting goals.')
        self._send_next_goal()

    def _check_cb(self):
        """Periodic check: timeout during NAVIGATING, cooldown expiry."""
        if self._state == 'NAVIGATING' and self._goal_handle is not None:
            elapsed = time.time() - (self._goal_start_time or time.time())
            if elapsed > self._goal_timeout and not self._canceled_by_timeout:
                self.get_logger().info(
                    '  Goal timeout (%.1fs > %.1fs), cancelling...'
                    % (elapsed, self._goal_timeout)
                )
                self._canceled_by_timeout = True
                cancel_future = self._goal_handle.cancel_goal_async()
                # The result callback will fire when cancel completes
                # _canceled_by_timeout flag ensures it's recorded as TIMED_OUT

        elif self._state == 'COOLDOWN':
            if time.time() >= self._cooldown_until:
                self._state = 'IDLE'
                self._send_next_goal()

    def _send_next_goal(self):
        if self._current_goal_index >= len(self.goals):
            self._finish()
            return

        goal_def = self.goals[self._current_goal_index]
        desc = goal_def.get(
            'description', 'goal %d' % self._current_goal_index
        )
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

        self._state = 'NAVIGATING'
        self._goal_start_time = time.time()
        self._goal_handle = None
        send_goal_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        send_goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by action server')
            self._finish_goal('REJECTED', error_msg='Goal rejected')
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        elapsed = time.time() - (self._goal_start_time or time.time())

        if self._canceled_by_timeout:
            status = 'TIMED_OUT'
            self._canceled_by_timeout = False
            error_code = None
            error_msg = 'Goal timed out after %.1fs' % elapsed
        else:
            status = _status_label(result.status)
            error_code = getattr(result.result, 'error_code', None) if result.result else None
            error_msg = getattr(result.result, 'error_msg', None) if result.result else None
            if status == 'ABORTED' and not error_msg:
                if error_code is not None:
                    error_msg = 'ABORTED (error_code=%s)' % error_code
                else:
                    error_msg = 'ABORTED (no detail)'

        self._finish_goal(status, duration_s=elapsed,
                          error_code=error_code, error_msg=error_msg)

    def _feedback_cb(self, feedback_msg):
        pass

    def _finish_goal(self, status, duration_s=0.0, error_code=None, error_msg=None):
        """Record result for the current goal and transition state."""
        self._record_result(status, duration_s=duration_s,
                            error_code=error_code, error_msg=error_msg)
        self._current_goal_index += 1

        if self._current_goal_index >= len(self.goals):
            self._finish()
            return

        # Stop on first failure unless continue_on_failure is set
        if status != 'SUCCEEDED' and not self._continue_on_failure:
            self.get_logger().info(
                'Stopping on %s (continue_on_failure=False)' % status
            )
            while self._current_goal_index < len(self.goals):
                goal_def = self.goals[self._current_goal_index]
                self.results.append({
                    'index': self._current_goal_index,
                    'description': goal_def.get('description', ''),
                    'pose': {
                        'x': goal_def['position']['x'],
                        'y': goal_def['position']['y'],
                        'frame_id': goal_def.get('frame_id', 'map'),
                    },
                    'status': 'SKIPPED',
                    'duration_s': 0.0,
                    'error_code': None,
                    'error_msg': None,
                })
                self._current_goal_index += 1
            self._finish()
            return

        self._start_cooldown()

    def _start_cooldown(self):
        self._state = 'COOLDOWN'
        self._cooldown_until = time.time() + self._cooldown

    def _record_result(self, status, duration_s=0.0, error_code=None, error_msg=None):
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
            'error_code': error_code,
            'error_msg': error_msg,
        })
        self.get_logger().info(
            '  Result: %s (%.1fs)' % (status, duration_s)
        )

    def _finish(self):
        self._check_timer.cancel()
        summary = compute_navigation_summary(self.results)
        self._write_reports(summary)
        self.get_logger().info('Smoke test complete.')
        raise SystemExit(0)

    def _write_reports(self, summary):
        os.makedirs(self.output_dir, exist_ok=True)

        report = {
            'timestamp': datetime.now().isoformat(),
            'source_goals': self._get_source_description(),
            'config': {
                'goal_timeout_seconds': self._goal_timeout,
                'continue_on_failure': self._continue_on_failure,
                'cooldown_seconds': self._cooldown,
            },
            'summary': summary,
        }

        json_path = os.path.join(self.output_dir, 'stage0b_results.json')
        with open(json_path, 'w') as f:
            json.dump(report, f, indent=2)

        md_path = os.path.join(self.output_dir, 'stage0b_results.md')
        with open(md_path, 'w') as f:
            f.write('# Stage 0B: Navigation Smoke Test Report\n\n')
            f.write('- **Report generated**: %s\n' % report['timestamp'])
            f.write('- **Source goals**: %s\n' % report['source_goals'])
            cfg = report['config']
            f.write('- **Goal timeout**: %.1fs\n' % cfg['goal_timeout_seconds'])
            f.write('- **Continue on failure**: %s\n' % cfg['continue_on_failure'])
            f.write('- **Cooldown**: %.1fs\n\n' % cfg['cooldown_seconds'])

            s = summary
            f.write('## Summary\n\n')
            f.write('| Metric | Value |\n')
            f.write('|--------|-------|\n')
            f.write('| Total goals | %d |\n' % s['total'])
            f.write('| Succeeded | %d |\n' % s['succeeded'])
            f.write('| Failed (ABORTED) | %d |\n' % s['failed'])
            f.write('| Canceled | %d |\n' % s['canceled'])
            f.write('| Rejected | %d |\n' % s['rejected'])
            f.write('| Timed out | %d |\n' % s['timed_out'])
            f.write('| Skipped | %d |\n' % s['skipped'])
            f.write('| Unknown | %d |\n' % s['unknown'])
            f.write('| Success rate | %s |\n'
                     % ('%.1f%%' % (s['success_rate'] * 100)
                        if s['success_rate'] is not None else 'N/A'))
            f.write('| Total duration | %.1fs |\n' % s['total_duration_s'])
            f.write('| Mean duration per goal | %.1fs |\n\n'
                     % s['mean_duration_s'])

            f.write('## Per-Goal Results\n\n')
            f.write('| # | Description | Status | Duration (s) | Error code | Error |\n')
            f.write('|---|-------------|--------|--------------|------------|-------|\n')
            for r in self.results:
                err_code = r.get('error_code') or ''
                err_msg = r.get('error_msg') or ''
                f.write('| %d | %s | %s | %.1f | %s | %s |\n' % (
                    r['index'], r['description'], r['status'],
                    r['duration_s'], err_code, err_msg
                ))

        self.get_logger().info('JSON: %s' % json_path)
        self.get_logger().info('Markdown: %s' % md_path)

    def _get_source_description(self):
        return 'YAML config (inline)' if hasattr(self, 'goals') else 'unknown'


_NODE = None


def _str_to_bool(val):
    """Convert a string to boolean for argparse."""
    if isinstance(val, bool):
        return val
    return val.lower() in ('true', '1', 'yes')


def _default_goals_path():
    """Return the default goals path from the package share directory."""
    try:
        share = get_package_share_directory('benchmark_tools')
        return os.path.join(share, 'config', 'stage0b_goals.yaml')
    except Exception:
        # Fallback for development (source tree) environment
        return os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', 'config', 'stage0b_goals.yaml'
        )


def main():
    parser = argparse.ArgumentParser(
        description='Stage 0B Navigation Functional Smoke Test'
    )
    parser.add_argument(
        '--goals', type=str,
        default=_default_goals_path(),
        help='Path to YAML goal definitions (absolute, relative to CWD, or package-share-relative)'
    )
    parser.add_argument(
        '--output-dir', type=str, default='/tmp/stage0b_results',
        help='Output directory for reports'
    )
    parser.add_argument(
        '--goal-timeout-seconds', type=float, default=30.0,
        help='Maximum time per goal before cancellation (default 30.0)'
    )
    parser.add_argument(
        '--continue-on-failure', type=_str_to_bool, default=True,
        help='Continue to next goal after failure (default true)'
    )
    parser.add_argument(
        '--cooldown-seconds', type=float, default=1.0,
        help='Pause between goals (default 1.0)'
    )
    args = parser.parse_args()

    goals_path = resolve_goals_path(args.goals)
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
    _NODE = NavigationSmokeTest(
        goals, args.output_dir,
        goal_timeout=args.goal_timeout_seconds,
        continue_on_failure=args.continue_on_failure,
        cooldown=args.cooldown_seconds,
    )
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
