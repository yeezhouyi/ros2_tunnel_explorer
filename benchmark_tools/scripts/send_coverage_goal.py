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

"""Send one ExecuteCoverage goal and write the result as JSON+Markdown.

Usage:
  ros2 run benchmark_tools send_coverage_goal.py \
      --timeout 1200 --output-dir /tmp/cov_run_01 [--resume PATH]

The executor must already be in READY_IDLE (map + AMCL + Nav2 ready).  The
result JSON carries the terminal result, coverage metrics and failure class,
which is what run_coverage_benchmark.sh aggregates (R14).
"""

import argparse
import json
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from tunnel_coverage_msgs.action import ExecuteCoverage


def _jsonable(value):
    if hasattr(value, '__dict__'):
        return {k: _jsonable(v) for k, v in value.__dict__.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value


class CoverageGoalClient(Node):
    def __init__(self, resume_path: str):
        super().__init__('coverage_goal_client')
        self.client = ActionClient(self, ExecuteCoverage, 'execute_coverage')
        self.resume = resume_path

    def run(self, timeout_s: float):
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('execute_coverage action server not available')
        goal = ExecuteCoverage.Goal()
        goal.resume_checkpoint_path = self.resume or ''
        self.get_logger().info(
            'Sending ExecuteCoverage goal (resume=%s)' % (self.resume or '-'))
        future = self.client.send_goal_async(goal, feedback_callback=None)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        gh = future.result()
        if gh is None or not gh.accepted:
            raise RuntimeError('goal rejected by executor (not READY_IDLE?)')
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_s)
        if not result_future.done():
            self.get_logger().warn('Timed out waiting for result')
            return None
        result = result_future.result().result
        self.get_logger().info('Terminal result: %d class=%s' % (
            result.terminal_result, result.failure_class))
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=1200.0)
    parser.add_argument('--output-dir', default='/tmp')
    parser.add_argument('--resume', default='')
    args = parser.parse_args()

    rclpy.init()
    node = CoverageGoalClient(args.resume)
    try:
        result = node.run(args.timeout)
        os.makedirs(args.output_dir, exist_ok=True)
        data = _jsonable(result)
        with open(os.path.join(args.output_dir, 'coverage_goal_result.json'),
                  'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)
        md = [
            '# Coverage run result',
            '',
            '| field | value |',
            '|---|---|',
        ]
        for key, value in (data or {}).items():
            md.append(f'| {key} | {value} |')
        with open(os.path.join(args.output_dir, 'coverage_goal_result.md'),
                  'w', encoding='utf-8') as f:
            f.write('\n'.join(md) + '\n')
        return 0 if result is not None else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
