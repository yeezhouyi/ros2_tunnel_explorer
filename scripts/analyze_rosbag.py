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
#
# analyze_rosbag.py
#
# Post-process a Stage 2A benchmark rosbag to extract:
#   - Travel distance from /odom
#   - Map coverage proxy from /map
#
# Usage:
#   python3 scripts/analyze_rosbag.py \
#     --bag-dir <rosbag_path> \
#     --output-dir <output_dir>
#
# Output:
#   rosbag_metrics.json  — structured metrics
#   rosbag_metrics.md    — human-readable report

import argparse
import json
import math
import os

try:
    import yaml
except ImportError:
    yaml = None


def _detect_storage_id(bag_path: str) -> str:
    """Detect rosbag2 storage format from metadata.yaml (mcap/sqlite3)."""
    if yaml is None:
        return "mcap"
    meta_path = os.path.join(bag_path, "metadata.yaml")
    if not os.path.isfile(meta_path):
        return "mcap"  # sensible default for Jazzy
    try:
        with open(meta_path) as f:
            meta = yaml.safe_load(f)
        storage_id = meta.get("rosbag2_bagfile_information", {}).get(
            "storage_identifier", "mcap"
        )
        return storage_id
    except Exception:
        return "mcap"


def _open_reader(bag_path: str, storage_id: str):
    """Open a SequentialReader with the given storage ID."""
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    storage_opts = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_opts = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage_opts, converter_opts)
    return reader


def extract_odom_travel(bag_path: str, storage_id: str, odom_topic: str = "/odom") -> float:
    """Integrate positional deltas from /odom messages. Returns meters."""
    try:
        from rclpy.serialization import deserialize_message
        from nav_msgs.msg import Odometry
    except ImportError:
        print("rclpy not available; skipping odometry analysis")
        return 0.0

    try:
        reader = _open_reader(bag_path, storage_id)
    except Exception as e:
        print(f"Failed to open rosbag: {e}")
        return 0.0

    prev_x = None
    prev_y = None
    total_dist = 0.0
    samples = 0

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == odom_topic:
            try:
                msg = deserialize_message(data, Odometry)
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                if prev_x is not None:
                    dx = x - prev_x
                    dy = y - prev_y
                    total_dist += math.sqrt(dx * dx + dy * dy)
                prev_x = x
                prev_y = y
                samples += 1
            except Exception:
                pass

    return total_dist


def extract_map_coverage(
    bag_path: str, storage_id: str, map_topic: str = "/map"
) -> dict:
    """Sample /map OccupancyGrid messages and track known/unknown ratios.

    Returns dict with:
      - first_known: known cells at first map
      - last_known: known cells at last map
      - coverage_proxy: last_known / max(first_known, 1) (relative growth)
      - total_slices: number of map messages processed
    """
    try:
        from rclpy.serialization import deserialize_message
        from nav_msgs.msg import OccupancyGrid
    except ImportError:
        print("rclpy not available; skipping map coverage analysis")
        return {
            "first_known": 0,
            "last_known": 0,
            "coverage_proxy": 0.0,
            "total_slices": 0,
        }

    try:
        reader = _open_reader(bag_path, storage_id)
    except Exception as e:
        print(f"Failed to open rosbag: {e}")
        return {
            "first_known": 0,
            "last_known": 0,
            "coverage_proxy": 0.0,
            "total_slices": 0,
        }

    first_known = None
    last_known = 0
    total_slices = 0

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == map_topic:
            try:
                msg = deserialize_message(data, OccupancyGrid)
                free = sum(1 for c in msg.data if c == 0)
                occupied = sum(1 for c in msg.data if c in (100, -1))
                known = free + occupied
                if first_known is None:
                    first_known = known
                last_known = known
                total_slices += 1
            except Exception:
                pass

    if first_known is None:
        first_known = 0

    return {
        "first_known": first_known,
        "last_known": last_known,
        "coverage_proxy": (
            last_known / max(first_known, 1) if first_known > 0 else 1.0
        ),
        "total_slices": total_slices,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Post-process Stage 2A benchmark rosbag"
    )
    parser.add_argument(
        "--bag-dir", required=True, help="Path to rosbag directory"
    )
    parser.add_argument(
        "--output-dir", required=True, help="Output directory for metrics"
    )
    parser.add_argument(
        "--odom-topic", default="/odom", help="Odometry topic (default: /odom)"
    )
    parser.add_argument(
        "--map-topic", default="/map", help="Map topic (default: /map)"
    )
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    # Resolve bag path (rosbag2 stores metadata.yaml inside the directory)
    bag_path = args.bag_dir
    metadata_path = os.path.join(bag_path, "metadata.yaml")
    if not os.path.isdir(bag_path) or not os.path.isfile(metadata_path):
        print(f"Warning: {bag_path} does not contain a rosbag (no metadata.yaml)")
        metrics = {
            "travel_distance_m": 0.0,
            "map_coverage": {
                "first_known": 0,
                "last_known": 0,
                "coverage_proxy": 0.0,
                "total_slices": 0,
            },
            "note": "rosbag not found or unreadable",
        }
    else:
        storage_id = _detect_storage_id(bag_path)
        print(f"Detected rosbag storage format: {storage_id}")
        travel_m = extract_odom_travel(bag_path, storage_id, args.odom_topic)
        coverage = extract_map_coverage(bag_path, storage_id, args.map_topic)

        metrics = {
            "travel_distance_m": round(travel_m, 2),
            "map_coverage": coverage,
        }

    # Write JSON
    json_path = os.path.join(args.output_dir, "rosbag_metrics.json")
    with open(json_path, "w") as f:
        json.dump(metrics, f, indent=2)
    print(f"Wrote {json_path}")

    # Write Markdown summary
    md_path = os.path.join(args.output_dir, "rosbag_metrics.md")
    with open(md_path, "w") as f:
        f.write("### Rosbag Analysis\n\n")
        f.write(f"| Metric | Value |\n")
        f.write(f"|--------|-------|\n")
        f.write(
            f"| Travel distance | {metrics['travel_distance_m']} m |\n"
        )
        mc = metrics["map_coverage"]
        f.write(f"| First map known cells | {mc['first_known']} |\n")
        f.write(f"| Last map known cells | {mc['last_known']} |\n")
        f.write(f"| Coverage proxy | {mc['coverage_proxy']:.3f} |\n")
        f.write(f"| Map slices recorded | {mc['total_slices']} |\n")
        if "note" in metrics:
            f.write(f"| Note | {metrics['note']} |\n")
        f.write("\n")
    print(f"Wrote {md_path}")


if __name__ == "__main__":
    main()
