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
Outputs a JSON report and Markdown summary with separate warm-up and
steady-state statistics.

Usage:
    ros2 run benchmark_tools record_stage0_metrics.py \
        --duration 600 \
        --warmup-seconds 30 \
        --output-dir /tmp/stage0_results
"""

import argparse
from datetime import datetime
import json
import os
import sys
import time

from benchmark_tools.stats import compute_topic_stats
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


class Stage0MetricsRecorder(Node):
    """Records message reception statistics for Stage 0 verification."""

    def __init__(self, duration_sec: float, warmup_seconds: float,
                 output_dir: str):
        super().__init__('stage0_metrics_recorder')
        self.duration_sec = duration_sec
        self.warmup_seconds = warmup_seconds
        self.output_dir = output_dir
        self.start_time = None
        self.warmup_end_time = None

        # Per-message timestamps (wall-clock seconds since epoch)
        self.clock_timestamps = []
        self.scan_timestamps = []
        self.map_timestamps = []

        # Sim-time samples for RTF: list of (wall_ts, sim_sec)
        self.sim_time_samples = []

        # Clock monotonicity check (strict: every sim_time must be >= previous)
        self.last_clock_value = None
        self.clock_non_monotonic_count = 0
        self.clock_backward_events = []  # (wall_ts, sim_sec, prev_sim_sec)

        # Gap detection thresholds
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
            'Stage 0 metrics recorder started. '
            'Duration: %ds, Warm-up: %ds' % (duration_sec, warmup_seconds)
        )

    def clock_cb(self, msg: Clock):
        now = time.time()
        if self.start_time is None:
            self.start_time = now
            self.warmup_end_time = now + self.warmup_seconds
        self.clock_timestamps.append(now)

        clock_sec = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.sim_time_samples.append((now, clock_sec))

        # Strict monotonicity check
        if self.last_clock_value is not None:
            if clock_sec < self.last_clock_value:
                self.clock_non_monotonic_count += 1
                self.clock_backward_events.append({
                    'wall_ts': now,
                    'sim_sec': clock_sec,
                    'prev_sim_sec': self.last_clock_value,
                    'delta_s': round(
                        self.last_clock_value - clock_sec, 6
                    ),
                })
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

    def _compute_rtf(self):
        """
        Compute Real-Time Factor from sim_time_samples.

        Returns (rtf, total_wall_sec, total_sim_sec).
        Uses the first and last sample to avoid noise from startup.
        """
        if len(self.sim_time_samples) < 2:
            return None, 0.0, 0.0

        first_wall, first_sim = self.sim_time_samples[0]
        last_wall, last_sim = self.sim_time_samples[-1]

        # Filter to steady-state (post-warmup) for primary RTF
        steady_samples = [
            (w, s) for w, s in self.sim_time_samples
            if self.warmup_end_time and w >= self.warmup_end_time
        ]

        def ratio(samples):
            if len(samples) < 2:
                return None, 0.0, 0.0
            fw, fs = samples[0]
            lw, ls = samples[-1]
            dw = lw - fw
            ds = ls - fs
            if dw <= 0:
                return None, dw, ds
            return ds / dw, dw, ds

        full_rtf, full_dw, full_ds = ratio(self.sim_time_samples)
        steady_rtf, steady_dw, steady_ds = (
            ratio(steady_samples) if len(steady_samples) >= 2
            else (None, 0.0, 0.0)
        )

        return {
            'full': {
                'rtf': round(full_rtf, 4) if full_rtf is not None else None,
                'wall_time_s': round(full_dw, 1),
                'sim_time_s': round(full_ds, 1),
            },
            'steady': {
                'rtf': round(steady_rtf, 4) if steady_rtf is not None else None,
                'wall_time_s': round(steady_dw, 1),
                'sim_time_s': round(steady_ds, 1),
            },
        }

    def shutdown_cb(self):
        self.get_logger().info('Duration reached. Computing metrics...')
        report = self.compute_metrics()
        self.write_report(report)
        self.get_logger().info('Report written to %s' % self.output_dir)
        raise SystemExit(0)

    def compute_metrics(self):
        """Compute all Stage 0 metrics from collected timestamps."""
        now = time.time()
        actual_duration = now - (self.start_time or now)
        warmup_end = self.warmup_end_time or now

        # Topic stats per phase
        clock_full, clock_steady = compute_topic_stats(
            self.clock_timestamps, self.start_time, warmup_end,
            '/clock', self.clock_gap_threshold,
        )
        scan_full, scan_steady = compute_topic_stats(
            self.scan_timestamps, self.start_time, warmup_end,
            '/scan', self.scan_gap_threshold,
        )
        map_full, map_steady = compute_topic_stats(
            self.map_timestamps, self.start_time, warmup_end,
            '/map', self.map_gap_threshold,
        )

        # RTF
        rtf = self._compute_rtf()

        # Clock monotonicity summary
        clock_monotonic = self.clock_non_monotonic_count == 0
        backward_details = self.clock_backward_events[:20]

        return {
            'timestamp': datetime.now().isoformat(),
            'duration_s': round(actual_duration, 1),
            'warmup_seconds': self.warmup_seconds,
            'warmup_end_time_iso': (
                datetime.fromtimestamp(warmup_end).isoformat()
                if warmup_end else None
            ),
            'real_time_factor': rtf,
            'topics': {
                'clock': {
                    'full': clock_full,
                    'steady': clock_steady,
                },
                'scan': {
                    'full': scan_full,
                    'steady': scan_steady,
                },
                'map': {
                    'full': map_full,
                    'steady': map_steady,
                },
            },
            'clock_monotonicity': {
                'strictly_monotonic': clock_monotonic,
                'non_monotonic_count': self.clock_non_monotonic_count,
                'backward_events': backward_details,
            },
        }

    @staticmethod
    def _write_topics_md(f, heading, topics_data):
        """Write a topic stats table for full or steady phase."""
        f.write('| Topic | Messages | Mean Hz | Min Interval | Max Interval | '
                'Mean Interval | P95 Interval | P99 Interval | '
                'Gap Count (>%.1fs) |\n' % (topics_data['clock']
                                            ['gaps']['threshold_s']))
        f.write('|-------|----------|---------|'
                '--------------|--------------|'
                '---------------|--------------|--------------|'
                '------------------|\n')
        for key in ['clock', 'scan', 'map']:
            t = topics_data[key]
            f.write(
                '| %s | %d | %s | %s | %s | %s | %s | %s | %d |\n' % (
                    t['label'], t['count'], t['mean_hz'],
                    t['min_interval_s'], t['max_interval_s'],
                    t['mean_interval_s'],
                    t['p95_interval_s'], t['p99_interval_s'],
                    t['gaps']['count'],
                )
            )

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
            now_dt = datetime.now()
            f.write('# Stage 0: Environment Feasibility Report\n\n')
            f.write('- **Report generated**: %s\n' % now_dt.isoformat())
            f.write('- **Recording timestamp**: %s\n'
                    % report['timestamp'])
            f.write('- **Duration**: %ss\n' % report['duration_s'])
            f.write('- **Warm-up period**: %ss\n\n'
                    % report['warmup_seconds'])

            # RTF section
            f.write('## Real-Time Factor\n\n')
            rtf = report.get('real_time_factor')
            if rtf:
                for phase in ['full', 'steady']:
                    p = rtf.get(phase, {})
                    f.write('### %s\n' % phase.capitalize())
                    f.write('- RTF: %s\n' % p.get('rtf', 'N/A'))
                    f.write('- Wall time: %ss\n' % p.get('wall_time_s', '?'))
                    f.write('- Sim time: %ss\n\n' % p.get('sim_time_s', '?'))
            else:
                f.write('- RTF: N/A (insufficient samples)\n\n')

            # Full-run topic statistics
            f.write('## Full Run: Topic Statistics\n\n')
            f.write(
                'Includes the entire recording from t=0 to t=%ss, '
                'including the warm-up transient.\n\n' % report['duration_s']
            )
            self._write_topics_md(f, 'full', {
                key: report['topics'][key]['full']
                for key in ['clock', 'scan', 'map']
            })
            f.write('\n')

            # Steady-state topic statistics
            f.write('## Steady State: Topic Statistics\n\n')
            f.write(
                'Excludes the first %ss warm-up period.\n\n'
                % report['warmup_seconds']
            )
            self._write_topics_md(f, 'steady', {
                key: report['topics'][key]['steady']
                for key in ['clock', 'scan', 'map']
            })
            f.write('\n')

            # Clock monotonicity
            f.write('## Clock Monotonicity\n\n')
            mono = report['clock_monotonicity']
            f.write('- Strictly monotonic: %s\n' % mono['strictly_monotonic'])
            f.write('- Non-monotonic events: %d\n'
                    % mono['non_monotonic_count'])
            if mono['backward_events']:
                f.write('\n### Backward Events (first 20)\n\n')
                f.write('| # | Wall Time | Sim Time | Previous Sim | Delta |\n')
                f.write('|---|-----------|----------|--------------|-------|\n')
                for i, ev in enumerate(mono['backward_events']):
                    f.write(
                        '| %d | %.1f | %s | %s | %ss |\n' % (
                            i + 1,
                            ev['wall_ts'],
                            ev['sim_sec'],
                            ev['prev_sim_sec'],
                            ev['delta_s'],
                        )
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
        '--warmup-seconds', type=float, default=30.0,
        help='Duration of warm-up phase excluded from steady-state stats '
             '(default: 30)'
    )
    parser.add_argument(
        '--output-dir', type=str, default='/tmp/stage0_results',
        help='Output directory for reports'
    )
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    try:
        node = Stage0MetricsRecorder(
            args.duration, args.warmup_seconds, args.output_dir
        )
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
