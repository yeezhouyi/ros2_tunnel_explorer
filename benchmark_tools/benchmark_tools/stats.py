"""
Pure statistics functions for Stage 0 metrics analysis.

These functions have no ROS2 or I/O dependencies and can be unit-tested
independently.
"""
import math
import time


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


def check_duration(actual_s, requested_s, tolerance_s=2.0):
    """
    Check whether the actual recording duration meets the requested duration
    within a tolerance.

    This accounts for timing granularity in shutdown timers: a 600 s request
    may record 599.0 s of wall time, which is acceptable.

    Arguments
    ---------
    actual_s -- observed wall-clock recording duration
    requested_s -- target recording duration
    tolerance_s -- allowed shortfall (default 2.0)

    Returns
    -------
    dict with 'pass', 'actual_s', 'requested_s', 'tolerance_s',
    'shortfall_s', and 'shortfall_within_tolerance' keys.

    """
    shortfall = max(0.0, requested_s - actual_s)
    within = shortfall <= tolerance_s
    return {
        'pass': within,
        'actual_s': round(actual_s, 1),
        'requested_s': requested_s,
        'tolerance_s': tolerance_s,
        'shortfall_s': round(shortfall, 1),
        'shortfall_within_tolerance': within,
    }


def percentile(sorted_values, p):
    """
    Compute the p-th percentile of a sorted list.

    Uses linear interpolation between adjacent values when the percentile
    falls between two data points.

    Arguments
    ---------
    sorted_values -- sorted (ascending) list of numbers
    p -- percentile to compute (0 <= p <= 100)

    Returns
    -------
    The p-th percentile value, or None if the list is empty.

    """
    if not sorted_values:
        return None
    k = (p / 100.0) * (len(sorted_values) - 1)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_values[int(k)]
    return sorted_values[int(f)] * (c - k) + sorted_values[int(c)] * (k - f)


def compute_topic_stats(timestamps, start_time, warmup_end_time,
                        label, gap_threshold, wall_now=None):
    """
    Compute per-phase statistics for a single topic.

    Separate statistics are computed for the full recording period and
    for the steady-state (post-warmup) period only. Gap detection flags
    inter-message intervals exceeding gap_threshold.

    Arguments
    ---------
    timestamps -- list of wall-clock timestamps
    start_time -- wall-clock time at recording start
    warmup_end_time -- wall-clock boundary between warmup and steady phases
    label -- topic name for the stats output
    gap_threshold -- inter-message gap (seconds) above which to flag
    wall_now -- wall-clock "now" override (for testability); defaults
                to time.time()

    Returns
    -------
    Tuple of (full_stats, steady_stats) dicts.

    """
    if wall_now is None:
        wall_now = time.time()
    duration = wall_now - start_time if start_time else 0.0

    # Split into warmup and steady phases by wall-clock time
    steady_ts = [ts for ts in timestamps if ts >= warmup_end_time]

    def phase_stats(ts_list, phase_duration):
        if not ts_list:
            return {
                'label': label,
                'count': 0,
                'mean_hz': 0.0,
            }

        n = len(ts_list)
        mean_hz = n / phase_duration if phase_duration > 0 else 0.0

        intervals = [
            ts_list[i] - ts_list[i - 1] for i in range(1, n)
        ]
        if not intervals:
            return {
                'label': label,
                'count': n,
                'mean_hz': round(mean_hz, 2),
                'min_interval_s': None,
                'max_interval_s': None,
                'mean_interval_s': None,
                'p95_interval_s': None,
                'p99_interval_s': None,
                'gaps': {
                    'threshold_s': gap_threshold,
                    'count': 0,
                    'details': [],
                },
            }

        sorted_iv = sorted(intervals)
        min_iv = sorted_iv[0]
        max_iv = sorted_iv[-1]
        mean_iv = sum(intervals) / len(intervals)

        p95 = percentile(sorted_iv, 95)
        p99 = percentile(sorted_iv, 99)

        # Gap detection
        gaps = []
        for i, dt in enumerate(intervals):
            if dt > gap_threshold:
                gaps.append({
                    'index': i,
                    'duration_s': round(dt, 3),
                    'elapsed_s': round(
                        ts_list[i] - start_time, 1
                    ) if start_time else 0.0,
                })

        return {
            'label': label,
            'count': n,
            'mean_hz': round(mean_hz, 2),
            'min_interval_s': round(min_iv, 4),
            'max_interval_s': round(max_iv, 4),
            'mean_interval_s': round(mean_iv, 4),
            'p95_interval_s': round(p95, 4) if p95 is not None else None,
            'p99_interval_s': round(p99, 4) if p99 is not None else None,
            'gaps': {
                'threshold_s': gap_threshold,
                'count': len(gaps),
                'details': gaps[:10],
            },
        }

    full = phase_stats(timestamps, duration)
    steady_duration = max(0.0, wall_now - warmup_end_time)
    steady = phase_stats(steady_ts, steady_duration)

    return full, steady


def compute_navigation_summary(results):
    """
    Compute summary statistics from a list of navigation goal results.

    ``results`` is a list of dicts, each with at minimum:

    - ``status`` (str): one of SUCCEEDED, ABORTED, CANCELED, UNKNOWN
    - ``duration_s`` (float): wall-clock seconds the goal took

    Optional keys: ``description``, ``error_msg``, ``index``.

    Returns a dict with:

    - ``total``
    - ``succeeded``
    - ``failed`` (ABORTED)
    - ``canceled``
    - ``rejected``
    - ``timed_out``
    - ``skipped``
    - ``unknown`` (status not in any recognised category)
    - ``success_rate`` (float 0.0–1.0, or None if total is 0)
    - ``total_duration_s``
    - ``mean_duration_s`` per goal
    - ``results`` (the input list, for serialisation)

    """
    if not results:
        return {
            'total': 0,
            'succeeded': 0,
            'failed': 0,
            'canceled': 0,
            'rejected': 0,
            'timed_out': 0,
            'skipped': 0,
            'unknown': 0,
            'success_rate': None,
            'total_duration_s': 0.0,
            'mean_duration_s': 0.0,
            'results': [],
        }

    succeeded = sum(1 for r in results if r['status'] == 'SUCCEEDED')
    failed = sum(1 for r in results if r['status'] == 'ABORTED')
    canceled = sum(1 for r in results if r['status'] == 'CANCELED')
    rejected = sum(1 for r in results if r['status'] == 'REJECTED')
    timed_out = sum(1 for r in results if r['status'] == 'TIMED_OUT')
    skipped = sum(1 for r in results if r['status'] == 'SKIPPED')
    known = succeeded + failed + canceled + rejected + timed_out + skipped
    unknown = len(results) - known

    durations = [r.get('duration_s', 0.0) for r in results]

    return {
        'total': len(results),
        'succeeded': succeeded,
        'failed': failed,
        'canceled': canceled,
        'rejected': rejected,
        'timed_out': timed_out,
        'skipped': skipped,
        'unknown': unknown,
        'success_rate': round(succeeded / len(results), 3),
        'total_duration_s': round(sum(durations), 1),
        'mean_duration_s': (
            round(sum(durations) / len(durations), 1)
            if durations else 0.0
        ),
        'results': results,
    }
