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
Record Stage 0 environment feasibility metrics.

Monitors /clock, /scan, and /map topics during a simulation run.
Outputs a JSON report and an optional Markdown summary.

Usage:
    ros2 run benchmark_tools record_stage0_metrics.py \
        --duration 600 \
        --output-dir /tmp/stage0_results
"""

import argparse
from datetime import datetime
import json
import os
import sys
import time

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


class Stage0MetricsRecorder(Node):
    """Records message reception statistics for Stage 0 verification."""

    def __init__(self, duration_sec: float, output_dir: str):
        super().__init__('stage0_metrics_recorder')
        self.duration_sec = duration_sec
        self.output_dir = output_dir
        self.start_time = None

        # Per-message timestamps (seconds since epoch)
        self.clock_timestamps = []
        self.scan_timestamps = []
        self.map_timestamps = []

        # Clock monotonicity check
        self.last_clock_value = None
        self.clock_non_monotonic_count = 0

        # Gap detection threshold (seconds)
        self.clock_gap_threshold = 0.5
        self.scan_gap_threshold = 1.0
        self.map_gap_threshold = 10.0

        # Subscribers
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, qos_reliable
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_sensor
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, qos_map
        )

        # Timer: periodic report and shutdown check
        self.report_timer = self.create_timer(
            max(10.0, duration_sec / 10.0), self.report_timer_cb
        )
        self.shutdown_timer = self.create_timer(duration_sec, self.shutdown_cb)

        self.get_logger().info(
            'Stage 0 metrics recorder started. Duration: %ds' % duration_sec
        )

    def clock_cb(self, msg: Clock):
        now = time.time()
        if self.start_time is None:
            self.start_time = now
        self.clock_timestamps.append(now)

        clock_sec = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.last_clock_value is not None:
            if clock_sec < self.last_clock_value:
                self.clock_non_monotonic_count += 1
        self.last_clock_value = clock_sec

    def scan_cb(self, msg: LaserScan):
        self.scan_timestamps.append(time.time())

    def map_cb(self, msg: OccupancyGrid):
        self.map_timestamps.append(time.time())

    def report_timer_cb(self):
        elapsed = time.time() - (self.start_time or time.time())
        self.get_logger().info(
            'Elapsed: %.0fs | clock: %d msgs | scan: %d msgs | map: %d msgs'
            % (elapsed, len(self.clock_timestamps),
               len(self.scan_timestamps), len(self.map_timestamps))
        )

    def shutdown_cb(self):
        self.get_logger().info('Duration reached. Computing metrics...')
        report = self.compute_metrics()
        self.write_report(report)
        self.get_logger().info('Report written to %s' % self.output_dir)
        raise SystemExit(0)

    def _compute_gaps(self, timestamps, gap_threshold):
        """Count gaps larger than threshold between consecutive timestamps."""
        if len(timestamps) < 2:
            return 0, []
        gaps = []
        for i in range(1, len(timestamps)):
            dt = timestamps[i] - timestamps[i - 1]
            if dt > gap_threshold:
                gaps.append({
                    'index': i,
                    'duration_s': round(dt, 3),
                    'elapsed_s': round(
                        timestamps[i] - (self.start_time or 0), 1
                    ),
                })
        return len(gaps), gaps

    def compute_metrics(self):
        """Compute all Stage 0 metrics from collected timestamps."""
        actual_duration = time.time() - (self.start_time or time.time())

        def stats(ts_list, label):
            if not ts_list:
                return {'count': 0, 'label': label}
            n = len(ts_list)
            mean_hz = n / actual_duration if actual_duration > 0 else 0.0
            intervals = [
                ts_list[i] - ts_list[i - 1] for i in range(1, n)
            ]
            mean_iv = (
                round(sum(intervals) / len(intervals), 4)
                if intervals else None
            )
            return {
                'label': label,
                'count': n,
                'mean_hz': round(mean_hz, 2),
                'min_interval_s': round(min(intervals), 4) if intervals else None,
                'max_interval_s': round(max(intervals), 4) if intervals else None,
                'mean_interval_s': mean_iv,
            }

        clock_stats = stats(self.clock_timestamps, '/clock')
        scan_stats = stats(self.scan_timestamps, '/scan')
        map_stats = stats(self.map_timestamps, '/map')

        # Gap analysis
        clock_gaps_count, clock_gaps = self._compute_gaps(
            self.clock_timestamps, self.clock_gap_threshold
        )
        scan_gaps_count, scan_gaps = self._compute_gaps(
            self.scan_timestamps, self.scan_gap_threshold
        )
        map_gaps_count, map_gaps = self._compute_gaps(
            self.map_timestamps, self.map_gap_threshold
        )

        return {
            'timestamp': datetime.now().isoformat(),
            'duration_s': round(actual_duration, 1),
            'topics': {
                'clock': clock_stats,
                'scan': scan_stats,
                'map': map_stats,
            },
            'monotonicity': {
                'clock_non_monotonic_count': self.clock_non_monotonic_count,
            },
            'gaps': {
                'clock': {
                    'threshold_s': self.clock_gap_threshold,
                    'count': clock_gaps_count,
                    'details': clock_gaps[:10],
                },
                'scan': {
                    'threshold_s': self.scan_gap_threshold,
                    'count': scan_gaps_count,
                    'details': scan_gaps[:10],
                },
                'map': {
                    'threshold_s': self.map_gap_threshold,
                    'count': map_gaps_count,
                    'details': map_gaps[:10],
                },
            },
        }

    def write_report(self, report):
        """Write JSON and Markdown reports."""
        os.makedirs(self.output_dir, exist_ok=True)

        # JSON report
        json_path = os.path.join(self.output_dir, 'stage0_metrics.json')
        with open(json_path, 'w') as f:
            json.dump(report, f, indent=2)

        # Markdown report
        md_path = os.path.join(self.output_dir, 'stage0_metrics.md')
        with open(md_path, 'w') as f:
            f.write('# Stage 0: Environment Feasibility Report\n\n')
            f.write('- **Timestamp**: %s\n' % report['timestamp'])
            f.write('- **Duration**: %ss\n\n' % report['duration_s'])

            f.write('## Topic Statistics\n\n')
            f.write(
                '| Topic | Messages | Mean Hz | '
                'Min Interval | Max Interval | Mean Interval |\n'
            )
            f.write(
                '|-------|----------|---------|'
                '--------------|--------------|---------------|\n'
            )
            for key in ['clock', 'scan', 'map']:
                t = report['topics'][key]
                f.write(
                    '| %s | %s | %s | %s | %s | %s |\n' % (
                        t['label'], t['count'], t['mean_hz'],
                        t['min_interval_s'], t['max_interval_s'],
                        t['mean_interval_s']
                    )
                )

            f.write('\n## Gap Analysis\n\n')
            f.write('| Topic | Threshold | Gap Count |\n')
            f.write('|-------|-----------|----------|\n')
            for key in ['clock', 'scan', 'map']:
                g = report['gaps'][key]
                f.write(
                    '| %s | %ss | %s |\n'
                    % (key, g['threshold_s'], g['count'])
                )

            f.write('\n## Clock Monotonicity\n\n')
            f.write(
                '- Non-monotonic events: %s\n'
                % report['monotonicity']['clock_non_monotonic_count']
            )

        self.get_logger().info('JSON: %s' % json_path)
        self.get_logger().info('Markdown: %s' % md_path)


def main():
    parser = argparse.ArgumentParser(
        description='Record Stage 0 environment feasibility metrics'
    )
    parser.add_argument(
        '--duration', type=float, default=600.0,
        help='Recording duration in seconds (default: 600 = 10 min)'
    )
    parser.add_argument(
        '--output-dir', type=str, default='/tmp/stage0_results',
        help='Output directory for reports'
    )
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    try:
        node = Stage0MetricsRecorder(args.duration, args.output_dir)
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
