"""路径平滑:对 grid_to_world 后的路径做轻量平滑.

方法:对连续三个点,若中间点到两端连线的距离 < threshold,则删除中间点。
不修改起止点。
"""
from __future__ import annotations

from math import hypot


def smooth_path(
    xy: list[tuple[float, float]],
    *,
    collinear_threshold_m: float = 0.01,
) -> list[tuple[float, float]]:
    """删除共线中间点.

    Args:
        xy: 世界坐标点列表
        collinear_threshold_m: 共线判定阈值(点到直线距离)

    Returns:
        平滑后的点列表(同输入若已是最简)
    """
    if len(xy) < 3:
        return list(xy)

    result: list[tuple[float, float]] = [xy[0]]
    i = 0
    while i < len(xy) - 1:
        # 找最远的 j 使得 [i..j] 共线
        j = i + 1
        while j < len(xy) - 1:
            ax, ay = xy[i]
            bx, by = xy[j + 1]
            mx, my = xy[j]
            # 点 (mx, my) 到直线 (a, b) 的距离
            num = abs((by - ay) * mx - (bx - ax) * my + bx * ay - by * ax)
            den = hypot(by - ay, bx - ax)
            if den == 0:
                break
            dist = num / den
            if dist > collinear_threshold_m:
                break
            j += 1
        # 保留 [i, j]
        result.append(xy[j])
        i = j
    return result
