#!/usr/bin/env python3
"""Generate the README demo clip for the tunnel explorer.

extract   -- pull /odom from a local rosbag2 (mcap) bag into a small npz
render    -- render the saved npz as a replay animation of the robot path
             and a disc-coverage growth (0.15 m footprint disc on a
             0.05 m grid -- same footprint/grid CELL geometry as
             seal_results.json's coverage audit, but this clip's DENOMINATOR
             is the grid bounding box of the driven trajectory, NOT the
             served-map executable mask; the two percentages are not
             numerically comparable).

This script's render half depends only on numpy + matplotlib + pillow; the
extract half additionally needs ROS 2 (rosbag2_py + nav_msgs).  The shipped
npz already contains everything render needs; re-running extract is only
useful if you have a fresh bag from the same scenario.

Usage:
  # extract (needs ROS 2 source'd)
  source /opt/ros/jazzy/setup.bash
  python3 scripts/make_explore_demo.py extract \
      --bag /home/zhouyi/b6_chain/explore_map_bag \
      --out  results/demo_20260909/track.npz
  # render (no ROS needed)
  python3 scripts/make_explore_demo.py render \
      --in   results/demo_20260909/track.npz \
      --gif  results/demo_20260909/explore_replay.gif

Honest labeling: the trail and coverage growth are reconstructed from
/odom recorded during a ROS 2 Gazebo simulation run of the canonical b6
chain -- no physical robot and no second Gazebo replay are involved.  The
displayed "covered m^2" is `disc-covered cells * cell_area`; the
percentage divides that by the trajectory bounding-box area.  This is a
visual gauge ONLY and is not comparable with
seal_results.coverage_chain.executor_effective (~0.90 over 4 runs), whose
denominator is the served-map executable mask.
"""
from __future__ import annotations

import argparse
import pathlib
import sys
from typing import Tuple

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]

DISC_M = 0.15
GRID_M = 0.05


def _disk_mask(radial: np.ndarray, radius: float) -> np.ndarray:
    return radial <= radius


def cmd_extract(args: argparse.Namespace) -> int:
    import rosbag2_py  # noqa: F401
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message

    bag = pathlib.Path(args.bag).expanduser()
    storage = rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap")
    conv = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, conv)
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/odom"]))

    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    n = 0
    # pre-allocate for speed
    cap = 100000
    t = np.empty(cap, dtype=np.float64)
    x = np.empty(cap, dtype=np.float32)
    y = np.empty(cap, dtype=np.float32)
    yaw = np.empty(cap, dtype=np.float32)
    while reader.has_next():
        _topic, data, _stamp = reader.read_next()
        msg = deserialize_message(data, Odometry)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # quaternion to yaw (z-axis)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yw = float(np.arctan2(siny, cosy))
        st = msg.header.stamp
        t[n] = st.sec + st.nanosec * 1e-9
        x[n] = float(p.x)
        y[n] = float(p.y)
        yaw[n] = yw
        n += 1
        if n == cap:
            cap *= 2
            t = np.resize(t, cap)
            x = np.resize(x, cap)
            y = np.resize(y, cap)
            yaw = np.resize(yaw, cap)
    t = t[:n]; x = x[:n]; y = y[:n]; yaw = yaw[:n]
    # make t monotonic from 0
    t = t - t[0]
    np.savez_compressed(out, t=t.astype(np.float64), x=x, y=y, yaw=yaw)
    print(f"extract: wrote {n} odom samples to {out} "
          f"(t_end={t[-1]:.1f} s, driven~{np.sum(np.hypot(np.diff(x), np.diff(y))):.1f} m)")
    return 0


def _world_grid(x: np.ndarray, y: np.ndarray, pad_m: float = 0.2
                ) -> Tuple[np.ndarray, np.ndarray, float, float]:
    """Return (xg, yg, x0, y0) for a grid that encloses the path."""
    xmin, xmax = float(x.min() - pad_m), float(x.max() + pad_m)
    ymin, ymax = float(y.min() - pad_m), float(y.max() + pad_m)
    nx = int(np.ceil((xmax - xmin) / GRID_M))
    ny = int(np.ceil((ymax - ymin) / GRID_M))
    xg = xmin + (np.arange(nx) + 0.5) * GRID_M
    yg = ymin + (np.arange(ny) + 0.5) * GRID_M
    return xg, yg, xmin, ymin


def cmd_render(args: argparse.Namespace) -> int:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import PillowWriter

    inp = pathlib.Path(args.inp)
    out = pathlib.Path(args.gif)
    out.parent.mkdir(parents=True, exist_ok=True)
    d = np.load(inp)
    t = d["t"]; x = d["x"]; y = d["y"]; yaw = d["yaw"]
    print(f"render: {len(t)} samples, t_end={t[-1]:.1f} s")
    xg, yg, x0, y0 = _world_grid(x, y)
    nx = len(xg); ny = len(yg)
    cell_area = GRID_M * GRID_M

    covered = np.zeros((ny, nx), dtype=bool)
    frames = args.frames
    # sample target time per frame
    sample_t = np.linspace(0.0, t[-1], frames)
    # index of last odom sample <= sample_t[k]
    idx = np.searchsorted(t, sample_t, side="right")

    fig, (a1, a2) = plt.subplots(
        1, 2, figsize=(9.6, 4.4), dpi=100,
        gridspec_kw={"width_ratios": [2.2, 1.0]})
    fig.subplots_adjust(left=0.06, right=0.985, top=0.90, bottom=0.12)

    a1.set_aspect("equal")
    a1.set_xlim(xg[0] - 0.15, xg[-1] + 0.15)
    a1.set_ylim(yg[0] - 0.15, yg[-1] + 0.15)
    a1.set_title(
        "Tunnel explorer -- real /odom replay (29.5 min, 92.6 k msg)\n"
        "0.15 m disc coverage on 0.05 m grid (seal gauge)",
        fontsize=9)
    a1.grid(alpha=0.2)
    # origin marker
    a1.plot(0, 0, marker="s", color="#475569", ms=5, label="spawn (0,0)")
    cov_img = a1.imshow(np.zeros_like(covered, dtype=float),
                         cmap="Greens", origin="lower",
                         extent=(xg[0] - GRID_M / 2, xg[-1] + GRID_M / 2,
                                 yg[0] - GRID_M / 2, yg[-1] + GRID_M / 2),
                         vmin=0, vmax=1, alpha=0.55, interpolation="nearest")
    (trail,) = a1.plot([], [], color="#2563eb", lw=1.6, label="executed path")
    (head,) = a1.plot([], [], "o", color="#dc2626", ms=5, zorder=6)
    (disc_artist,) = a1.plot([], [], color="#dc2626", lw=0.8, alpha=0.55)
    a1.legend(loc="lower left", fontsize=7.5, framealpha=0.9)

    a2.axis("off")
    txt = a2.text(0.02, 0.97, "", va="top", ha="left",
                  fontfamily="monospace", fontsize=8.6)

    # pre-compute disc masks lazily per-frame (uses memory; acceptable)
    # For 140 frames * full xy grid * idx entries is too much; instead we
    # grow `covered` incrementally per frame using new samples only.

    # precompute disc-pixel offsets once
    pix = int(np.ceil(DISC_M / GRID_M))
    yy_off, xx_off = np.mgrid[-pix: pix + 1, -pix: pix + 1]
    rr = np.sqrt((xx_off * GRID_M) ** 2 + (yy_off * GRID_M) ** 2)
    disc_off_mask = rr <= DISC_M

    fps = 12

    def draw(k):
        kk = int(min(idx[k], len(t) - 1))
        # grow `covered` up to sample kk
        for j in range(prev_k[0], kk + 1):
            cx, cy = x[j], y[j]
            ix = int((cx - x0) / GRID_M)
            iy = int((cy - y0) / GRID_M)
            x0i = max(0, ix - pix); x1i = min(nx, ix + pix + 1)
            y0i = max(0, iy - pix); y1i = min(ny, iy + pix + 1)
            dx0 = x0i - (ix - pix); dx1 = disc_off_mask.shape[1] - ((ix + pix + 1) - x1i)
            dy0 = y0i - (iy - pix); dy1 = disc_off_mask.shape[0] - ((iy + pix + 1) - y1i)
            covered[y0i: y1i, x0i: x1i] |= disc_off_mask[dy0: dy1, dx0: dx1]
        prev_k[0] = kk + 1
        # update visuals
        cov_img.set_data(covered.astype(float))
        trail.set_data(x[: kk + 1], y[: kk + 1])
        head.set_data([x[kk]], [y[kk]])
        # current disc ring
        th = np.linspace(0, 2 * np.pi, 40)
        disc_artist.set_data(x[kk] + DISC_M * np.cos(th),
                             y[kk] + DISC_M * np.sin(th))
        # stats
        t_real = float(t[kk])
        driven = float(np.sum(np.hypot(np.diff(x[: kk + 1]),
                                        np.diff(y[: kk + 1]))))
        cov_area = float(covered.sum()) * cell_area
        total_area = (xg[-1] - xg[0] + GRID_M) * (yg[-1] - yg[0] + GRID_M)
        cov_pct = cov_area / total_area * 100.0
        txt.set_text(
            f"frame = {k + 1:3d} / {frames}\n"
            f"t_real  = {t_real:6.1f} s   (~{t_real / 60:4.1f} min)\n"
            f"driven  = {driven:6.1f} m\n"
            f"covered = {cov_area:5.2f} m^2  ({cov_pct:4.1f}% of box)\n"
            f"samples = {kk + 1:5d} / {len(t)}\n"
            f"gauge   = 0.15 m disc / 0.05 m grid\n"
            f"frame   = odom frame, spawn (0,0)")
        return [cov_img, trail, head, disc_artist, txt]

    prev_k = [0]
    import matplotlib.animation as manim
    anim = manim.FuncAnimation(fig, draw, frames=frames,
                               interval=1000 // fps, blit=False)
    writer = PillowWriter(fps=fps)
    anim.save(out, writer=writer)
    print(f"render: wrote {out} ({out.stat().st_size / 1e6:.2f} MB)")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    pe = sub.add_parser("extract")
    pe.add_argument("--bag", required=True)
    pe.add_argument("--out", required=True)
    pr = sub.add_parser("render")
    pr.add_argument("--in", dest="inp", required=True)
    pr.add_argument("--gif", required=True)
    pr.add_argument("--frames", type=int, default=140)
    args = ap.parse_args()
    return {"extract": cmd_extract, "render": cmd_render}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())