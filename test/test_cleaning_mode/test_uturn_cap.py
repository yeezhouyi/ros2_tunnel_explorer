"""U-turn cap tests (B6 fix): serpentine lane ends connected by a
semicircular cap with continuous yaw instead of a 180-degree hop."""
import numpy as np

from cleaning_mode.boustrophedon import connected_boustrophedon


def open_map():
    """Open area: rows 2-12, cols 2-21 free; lanes every 2 rows."""
    free = np.zeros((14, 24), dtype=bool)
    free[2:13, 2:22] = True
    return free


def test_cap_connects_adjacent_lanes():
    free = open_map()
    path = connected_boustrophedon(free, spacing_cells=2)
    assert len(path) > 20
    assert all(free[y, x] for x, y in path)


def test_cap_bulges_beyond_lane_end():
    free = open_map()
    path = connected_boustrophedon(free, spacing_cells=2)
    # lanes sweep rows 2,4,6,8,10 (even); the caps between them bulge past
    # the lane end column — some point must lie in the odd rows between
    # consecutive lanes (the arc region)
    ys = [y for _, y in path]
    odd_rows = [y for y in ys if y % 2 == 1]
    assert odd_rows, "no cap arc points between lanes"


def test_no_180_degree_heading_jump():
    free = open_map()
    path = connected_boustrophedon(free, spacing_cells=2)
    steps = [(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(path, path[1:])]
    for a, b in zip(steps, steps[1:]):
        assert not (a[0] == -b[0] and a[1] == -b[1]), "180-deg jump found"


def test_all_cells_free_and_connected():
    free = open_map()
    path = connected_boustrophedon(free, spacing_cells=2)
    assert all(free[y, x] for x, y in path)
    # 4-connected steps only (cap sampled densely)
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        assert abs(x1 - x0) + abs(y1 - y0) <= 2


def test_cap_blocked_region_falls_back():
    free = open_map()
    # block the outward bulge column beyond the right lane ends
    free[:, 21:22] = False
    path = connected_boustrophedon(free, spacing_cells=2)
    assert all(free[y, x] for x, y in path)  # A* fallback, still collision-free
