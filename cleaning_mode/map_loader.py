"""Load a map_saver-style map (PGM image + YAML metadata) into a
cleaning_mode grid, bridging [2] saved map -> [3] coverage planner in the
B6 chain.

Accepts the standard SLAM output pair:
  <name>.yaml  : {image, resolution, origin: [x, y, yaw], ...}
  <name>.pgm   : P5 (binary) grayscale; 0 = occupied, 254 = free,
                 205 = unknown (map_saver conventions)
Negated pixels (ROS occupancy convention in some savers) are handled by
--negate.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from cleaning_mode.obstacle_inflation import CleanGrid


def read_pgm(path: str) -> np.ndarray:
    """Minimal P5/P2 PGM reader (no PIL dependency)."""
    with open(path, "rb") as f:
        data = f.read()
    tokens = []
    i = 0
    while len(tokens) < 4 and i < len(data):
        # skip whitespace and comments
        while i < len(data) and data[i : i + 1].isspace():
            i += 1
        if data[i : i + 1] == b"#":
            while i < len(data) and data[i : i + 1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(data) and not data[j : j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    magic = tokens[0].decode()
    width, height = int(tokens[1]), int(tokens[2])
    maxval = int(tokens[3])
    i += 1  # single whitespace after maxval
    if magic == "P5":
        if maxval < 256:
            img = np.frombuffer(data[i : i + width * height], dtype=np.uint8)
        else:
            img = np.frombuffer(data[i : i + 2 * width * height], dtype=">u2")
    elif magic == "P2":
        body = data[i:].split()
        img = np.array([int(t) for t in body], dtype=np.uint16)
    else:
        raise ValueError(f"unsupported PGM magic: {magic}")
    return img.reshape(height, width)


def load_map(yaml_path: str, negate: bool = False) -> CleanGrid:
    import yaml

    meta = yaml.safe_load(Path(yaml_path).read_text())
    image_path = str(Path(yaml_path).parent / meta["image"])
    img = read_pgm(image_path).astype(np.int16)
    if negate:
        img = 255 - img
    # map_saver conventions: p=0 black -> occupied, 254 white -> free,
    # 205 grey -> unknown; some savers emit inverted polarity.
    occ = (img <= 50).astype(np.int16) * 100
    unk = ((img >= 100) & (img <= 220)).astype(np.int16) * -1
    grid = np.where(occ == 100, 100, np.where(unk == -1, -1, 0)).astype(np.int16)
    # polarity guard: if the map is mostly "occupied" the saver used the
    # inverted convention (free=black) — flip it.
    if (grid == 100).sum() > 0.6 * grid.size:
        grid = np.where(img <= 50, 0, np.where(img >= 220, 100, -1)).astype(np.int16)
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    return make_clean(grid, float(meta["resolution"]),
                      (float(origin[0]), float(origin[1])))


def make_clean(grid: np.ndarray, resolution: float, origin) -> CleanGrid:
    from cleaning_mode.obstacle_inflation import make_clean_grid

    return make_clean_grid(grid, resolution, origin, unknown_policy="obstacle")
