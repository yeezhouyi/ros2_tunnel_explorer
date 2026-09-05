"""障碍物膨胀、未知区策略与栅格掩膜处理.

策略:
- candidate: 应覆盖的目标格子(could-be-cleaned)
- known_free: 已确认无障碍(已探索区域)
- obstacle: 已知障碍(静态)
- unknown: 未知区域(探索前沿,默认按候选处理)
- boundary-excluded: 边界膨胀区(膨胀边界后排除)
"""
from __future__ import annotations

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view


def inflate(blocked: np.ndarray, radius_cells: int) -> np.ndarray:
    """对障碍栅格进行欧氏半径膨胀.

    膨胀后的栅格中,任何距离原始障碍小于等于 radius_cells 的格子都标记为 True。
    """
    blocked = np.asarray(blocked, dtype=bool)
    if blocked.ndim != 2:
        raise ValueError("blocked must be a 2-D mask")
    if radius_cells < 0:
        raise ValueError("radius_cells must be non-negative")
    if radius_cells == 0:
        return blocked.copy()

    size = 2 * radius_cells + 1
    # 零填充(边界外视为障碍)
    padded = np.pad(blocked, radius_cells, mode="constant", constant_values=True)
    windows = sliding_window_view(padded, (size, size))
    return windows.any(axis=(-2, -1))


def unknown_as_candidate(known_free: np.ndarray, unknown: np.ndarray) -> np.ndarray:
    """将未知区并入候选区: candidate = known_free | unknown.

    这样未知区也会被 boustrophedon 覆盖(探索优先)。
    """
    known_free = np.asarray(known_free, dtype=bool)
    unknown = np.asarray(unknown, dtype=bool)
    if known_free.shape != unknown.shape:
        raise ValueError("known_free and unknown must have identical shapes")
    return known_free | unknown


def apply_inflation_and_exclusions(
    obstacle: np.ndarray,
    unknown: np.ndarray,
    known_free: np.ndarray,
    candidate: np.ndarray,
    radius_cells: int,
) -> tuple[np.ndarray, np.ndarray]:
    """膨胀障碍物并应用排除掩膜.

    Returns:
        blocked: 膨胀后的不可行区域
        executable: 可行且待覆盖的区域
    """
    # 合并障碍和未知区,然后膨胀
    uncertain = obstacle | unknown
    blocked = inflate(uncertain, radius_cells)

    # 可行区域 = 候选 & 已知自由 & 未被膨胀阻塞
    executable = candidate & known_free & ~blocked
    return blocked, executable


def boundary_exclusion_mask(shape: tuple[int, int], margin_cells: int) -> np.ndarray:
    """创建边界排除掩膜(边界外 = True).

    地图边界附近通常不需覆盖。
    """
    h, w = shape
    mask = np.zeros((h, w), dtype=bool)
    if margin_cells > 0:
        mask[:margin_cells, :] = True
        mask[-margin_cells:, :] = True
        mask[:, :margin_cells] = True
        mask[:, -margin_cells:] = True
    return mask
