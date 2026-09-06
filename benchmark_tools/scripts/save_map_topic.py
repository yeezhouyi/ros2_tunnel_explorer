#!/usr/bin/env python3
"""Save the current /map OccupancyGrid as a map_saver-style PGM+YAML pair."""
import argparse

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class MapGrab(Node):
    def __init__(self):
        super().__init__("save_map_topic")
        self.msg = None
        self.create_subscription(OccupancyGrid, "/map", self.on_map, 1)

    def on_map(self, msg):
        if self.msg is None or msg.info.width * msg.info.height > \
            (self.msg.info.width * self.msg.info.height if self.msg else 0):
            self.msg = msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", required=True, help="output base path (no ext)")
    ap.add_argument("--wait", type=float, default=10.0)
    args = ap.parse_args()
    rclpy.init()
    node = MapGrab()
    import time
    end = time.monotonic() + args.wait
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.msg is not None and time.monotonic() > end - args.wait + 3.0:
            break
    g = node.msg
    if g is None:
        print("no /map received")
        return 2
    import numpy as np
    arr = np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
    # occupancy -> grayscale: -1 unknown -> 205, 100 -> 0, 0 -> 254
    img = np.full_like(arr, 254, dtype=np.uint8)
    img[arr == 100] = 0
    img[arr == -1] = 205
    with open(args.output + ".pgm", "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (g.info.width, g.info.height))
        f.write(img.tobytes())
    with open(args.output + ".yaml", "w") as f:
        f.write("image: %s.pgm\n" % args.output.split("/")[-1])
        f.write("resolution: %s\n" % g.info.resolution)
        f.write("origin: [%s, %s, %s]\n" % (
            g.info.origin.position.x, g.info.origin.position.y,
            g.info.origin.orientation.w))
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    print("saved", args.output + ".pgm/.yaml",
          "free:", int((arr == 0).sum()), "occ:", int((arr == 100).sum()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
