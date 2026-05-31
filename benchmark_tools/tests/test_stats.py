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

"""Unit tests for benchmark_tools.stats pure functions."""

from benchmark_tools.stats import compute_topic_stats, percentile

import pytest


# ── percentile ──────────────────────────────────────────────────────────────

class TestPercentile:
    """Tests for the percentile function."""

    def test_empty_list(self):
        assert percentile([], 50) is None

    def test_single_element(self):
        assert percentile([42], 50) == 42
        assert percentile([42], 0) == 42
        assert percentile([42], 100) == 42

    def test_median_odd_count(self):
        assert percentile([1, 2, 3, 4, 5], 50) == 3

    def test_median_even_count(self):
        # Linear interpolation: p=50, len=4 -> k=1.5 -> avg(2,3)=2.5
        assert percentile([1, 2, 3, 4], 50) == 2.5

    def test_min_and_max(self):
        data = [10, 20, 30, 40, 50]
        assert percentile(data, 0) == 10
        assert percentile(data, 100) == 50

    def test_known_percentile(self):
        data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        # P50 = 5.5 (avg of 5 and 6)
        assert percentile(data, 50) == 5.5
        # P95: k = 0.95 * 9 = 8.55 -> 9 * 0.45 + 10 * 0.55
        p95 = percentile(data, 95)
        assert p95 == pytest.approx(9.55, rel=1e-4)

    def test_duplicate_values(self):
        data = [1, 1, 1, 2, 2, 2, 3, 3, 3]
        assert percentile(data, 50) == 2
        assert percentile(data, 90) == pytest.approx(3, rel=1e-4)

    def test_realistic_intervals_p95(self):
        # 1900 entries at 0.001s + 100 at 0.010s = 2000 total
        # P95 index: 0.95 * 1999 = 1899.05 -> within 0.001s range (indices 0-1899)
        # sorted[1899]=0.001, sorted[1900]=0.010
        # = 0.001 * 0.95 + 0.010 * 0.05 = 0.00145
        intervals = [0.001] * 1900 + [0.010] * 100
        intervals.sort()
        p95 = percentile(intervals, 95)
        assert p95 == pytest.approx(0.00145, rel=1e-3)

    def test_p99_captures_outliers(self):
        # P99 index: 0.99 * 1999 = 1979.01 -> within the 0.010 zone
        intervals = [0.001] * 1900 + [0.010] * 100
        intervals.sort()
        p99 = percentile(intervals, 99)
        assert p99 == pytest.approx(0.010, rel=1e-2)

    def test_endpoint_percentiles(self):
        data = [1, 2, 3, 4, 5]
        assert percentile(data, 0) == 1
        assert percentile(data, 100) == 5


# ── compute_topic_stats ─────────────────────────────────────────────────────

class TestComputeTopicStats:
    """Tests for the compute_topic_stats function."""

    def test_empty_timestamps(self):
        full, steady = compute_topic_stats(
            [], 1000.0, 1005.0, '/test', 1.0, wall_now=1100.0
        )
        assert full['count'] == 0
        assert full['mean_hz'] == 0.0
        assert steady['count'] == 0

    def test_single_timestamp_no_intervals(self):
        full, steady = compute_topic_stats(
            [1000.0], 1000.0, 1005.0, '/test', 1.0, wall_now=1100.0
        )
        assert full['count'] == 1
        assert full['min_interval_s'] is None
        assert steady['count'] == 0

    def test_steady_state_split(self):
        # 5 timestamps during warmup (at 1000.0-1000.4), then
        # 10 timestamps during steady (at 1005.5-1006.4)
        warmup_ts = [1000.0 + 0.1 * i for i in range(5)]
        steady_ts = [1005.5 + 0.1 * i for i in range(10)]
        all_ts = warmup_ts + steady_ts

        full, steady = compute_topic_stats(
            all_ts, 1000.0, 1005.0, '/test', 1.0, wall_now=1100.0
        )

        assert full['count'] == 15
        assert steady['count'] == 10
        # 15 messages in 100s (1100-1000) = 0.15 Hz
        assert full['mean_hz'] == pytest.approx(0.15, rel=0.01)
        # 10 messages in 95s (1100-1005) = round(0.10526, 2) = 0.11 Hz
        assert steady['mean_hz'] == pytest.approx(0.11, rel=0.01)

    def test_gap_detection(self):
        # 10 timestamps at 0.1s intervals (1001.0-1001.9), then
        # a 1.5s gap, then 10 more at 0.1s intervals (1003.4-1004.3)
        ts_list = [1001.0 + 0.1 * i for i in range(10)]
        ts_start2 = ts_list[-1] + 1.5  # 1.5s after last of first group
        ts_list.extend([ts_start2 + 0.1 * i for i in range(10)])

        full, steady = compute_topic_stats(
            ts_list, 1000.0, 1000.0, '/test', 0.5, wall_now=1100.0
        )
        # The 1.5s gap > 0.5s threshold, should be detected
        assert full['gaps']['count'] >= 1
        assert full['gaps']['details'][0]['duration_s'] >= 1.5

    def test_frequency_calculation(self):
        # 100 timestamps at 0.01s intervals = expected ~10 Hz
        # over 100s (100/100 = 1 Hz mean actually, since intervals < period)
        ts_list = [1000.0 + 0.01 * i for i in range(100)]

        full, steady = compute_topic_stats(
            ts_list, 1000.0, 1000.0, '/test', 0.5, wall_now=1100.0
        )
        # 100 messages / 100s = 1 Hz
        assert full['mean_hz'] == pytest.approx(1.0, rel=0.01)
        # Mean interval should be ~0.01s
        assert full['mean_interval_s'] == pytest.approx(0.01, rel=0.01)

    def test_gap_below_threshold(self):
        # 0.8s gaps with 1.0s threshold
        ts_list = [1000.0 + 0.8 * i for i in range(5)]

        full, steady = compute_topic_stats(
            ts_list, 1000.0, 1000.0, '/test', 1.0, wall_now=1100.0
        )
        assert full['gaps']['count'] == 0

    def test_warmup_empty_steady(self):
        """All timestamps fall within warmup."""
        ts_list = [1000.0 + 0.1 * i for i in range(10)]

        full, steady = compute_topic_stats(
            ts_list, 1000.0, 1100.0, '/test', 1.0, wall_now=1100.0
        )
        assert steady['count'] == 0

    def test_all_timestamps_in_steady(self):
        """No warmup phase (warmup_end = start)."""
        ts_list = [1000.0 + 0.1 * i for i in range(10)]

        full, steady = compute_topic_stats(
            ts_list, 1000.0, 1000.0, '/test', 1.0, wall_now=1100.0
        )
        assert steady['count'] == 10
        assert full['count'] == 10

    def test_p95_p99_values(self):
        # 90 timestamps at 0.01s, then 3 outliers at 0.1s
        ts_list = [1000.0 + 0.01 * i for i in range(90)]
        ts_list.extend([1000.0 + 0.01 * 89 + 0.1,
                        1000.0 + 0.01 * 89 + 0.11,
                        1000.0 + 0.01 * 89 + 0.12])
        ts_list.sort()

        full, steady = compute_topic_stats(
            ts_list, 1000.0, 1000.0, '/test', 1.0, wall_now=1100.0
        )
        # P95 should not include outliers (3/93 = ~3.2% outliers)
        assert full['p95_interval_s'] is not None
        assert full['p95_interval_s'] < 0.05

    def test_label_preserved(self):
        full, steady = compute_topic_stats(
            [1000.0, 1000.1], 1000.0, 1000.0, '/custom_topic', 1.0,
            wall_now=1100.0
        )
        assert full['label'] == '/custom_topic'
        assert steady['label'] == '/custom_topic'

    def test_median_interval(self):
        """With odd number of equal intervals, median equals each interval."""
        ts_list = [1000.0, 1000.5, 1001.0, 1001.5, 1002.0]
        full, steady = compute_topic_stats(
            ts_list, 1000.0, 1000.0, '/test', 1.0, wall_now=1100.0
        )
        assert full['mean_interval_s'] == pytest.approx(0.5, rel=0.01)
