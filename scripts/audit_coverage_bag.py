#!/usr/bin/env python3
"""U7 coverage audit (offline, from rosbag).

Reconstructs from the recorded bag:
  * known_free / obstacle / unknown masks from the final /map message
  * visited mask = cells within sweep_radius_m of any /odom pose
  * coverage_known_free = visited & free / free
  * required sweep length = free_area / spacing_m vs actual path length
Exports .npy masks + coverage_audit.json (plan R9/R10: both denominators,
no threshold changes).
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid, Odometry
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions


def load_bag(bag_dir: str):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id="mcap"),
                ConverterOptions("", ""))
    topics = reader.get_all_topics_and_types()
    wanted = {t.name: t.type for t in topics}
    maps, odoms = [], []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name == "/map":
            maps.append(deserialize_message(data, OccupancyGrid))
        elif name == "/odom":
            odoms.append(deserialize_message(data, Odometry))
    return wanted, maps, odoms


def masks_from_map(grid: OccupancyGrid, sweep_radius_m: float,
                   odom_xy: list) -> dict:
    w, h = grid.info.width, grid.info.height
    res = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    data = np.array(grid.data, dtype=np.int16).reshape(h, w)

    occupied = (data == 100)
    unknown = (data == -1)
    known_free = (data == 0)

    # visited: cells within sweep_radius of any odom point (footprint model)
    rr = int(math.ceil(sweep_radius_m / res))
    r2 = sweep_radius_m ** 2
    visited = np.zeros((h, w), dtype=bool)
    for (x, y) in odom_xy:
        cx = int((x - ox) / res)
        cy = int((y - oy) / res)
        for dy in range(-rr, rr + 1):
            for dx in range(-rr, rr + 1):
                if dx * dx + dy * dy > r2 * (1.0 / res) ** 2 * (res ** 2) / (res ** 2):
                    pass
                ny, nx = cy + dy, cx + dx
                if 0 <= ny < h and 0 <= nx < w:
                    wx = ox + (nx + 0.5) * res
                    wy = oy + (ny + 0.5) * res
                    if (wx - x) ** 2 + (wy - y) ** 2 <= r2:
                        visited[ny, nx] = True

    return {
        "known_free": known_free,
        "obstacle": occupied,
        "unknown": unknown,
        "visited": visited,
        "meta": {"width": w, "height": h, "resolution": res,
                 "origin": [ox, oy]},
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--sweep-radius-m", type=float, default=0.25)
    ap.add_argument("--spacing-m", type=float, default=0.28)
    ap.add_argument("--actual-path-length-m", type=float, default=22.555)
    args = ap.parse_args()

    rclpy.init()
    try:
        _, maps, odoms = load_bag(args.bag)
    finally:
        rclpy.shutdown()

    if not maps:
        raise SystemExit("no /map messages in bag")
    grid = maps[-1]  # final map
    odom_xy = [(m.pose.pose.position.x, m.pose.pose.position.y) for m in odoms]

    masks = masks_from_map(grid, args.sweep_radius_m, odom_xy)
    meta = masks.pop("meta")

    free = masks["known_free"]
    visited = masks["visited"]
    visited_free = visited & free

    # odom path length (continuous)
    path_len = sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(odom_xy, odom_xy[1:])
    )

    free_area = free.sum() * meta["resolution"] ** 2
    required_len = free_area / args.spacing_m

    audit = {
        "map_meta": meta,
        "odom_samples": len(odom_xy),
        "odom_path_length_m": round(path_len, 3),
        "sweep_radius_m": args.sweep_radius_m,
        "spacing_m": args.spacing_m,
        "known_free_cells": int(free.sum()),
        "visited_free_cells": int(visited_free.sum()),
        "coverage_known_free": round(float(visited_free.sum()) / max(free.sum(), 1), 4),
        "coverage_task_note": "task denominator = intended_target = known_free "
                              "(cleanable_map_builder.cpp:263); exempt mask not "
                              "recorded in this bag -> COVERAGE_METRIC denominator "
                              "shown for known_free only",
        "free_area_m2": round(free_area, 3),
        "required_sweep_length_m": round(required_len, 1),
        "actual_executed_path_length_m": args.actual_path_length_m,
        "plan_undercoverage_ratio": round(args.actual_path_length_m / required_len, 3),
    }

    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)
    np.save(out / "known_free.npy", free)
    np.save(out / "obstacle.npy", masks["obstacle"])
    np.save(out / "unknown.npy", masks["unknown"])
    np.save(out / "visited.npy", visited)
    (out / "coverage_audit.json").write_text(json.dumps(audit, indent=1))
    print(json.dumps(audit, indent=1))


if __name__ == "__main__":
    main()
