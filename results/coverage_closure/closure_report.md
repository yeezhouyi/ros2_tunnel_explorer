# Residual coverage + budgeted recovery plan (run6)

Gauge: served static map cleaning_room_rect masks (executable 6996), shift (0, 0), footprint radius r0.15.

| quantity | value |
|---|---|
| coverage before (r0.15) | 0.7364 (5152/6996 cells) |
| residual cells / area | 1844 / 4.6100 m^2 |
| residual domains total / kept / dropped | 33 / 21 / 12 |
| dropped cells (too small / no single pass) | 20 |
| recovery route | 1008 waypoints, 48.350 m |
| distance budget (x1.50) | 72.525 m |
| theoretical coverage after perfect recovery (r0.15) | 0.9971 (delta +0.2607) |

Dropped domains (reported, not silently kept):

- id=R12 cells=2 area=0.0050 m^2 centre=[0.675, -0.25] reason=too_small
- id=R14 cells=1 area=0.0025 m^2 centre=[0.625, -0.175] reason=too_small
- id=R15 cells=1 area=0.0025 m^2 centre=[0.575, -0.075] reason=too_small
- id=R16 cells=1 area=0.0025 m^2 centre=[0.525, -0.025] reason=too_small
- id=R19 cells=3 area=0.0075 m^2 centre=[-1.875, 0.325] reason=too_small
- id=R21 cells=1 area=0.0025 m^2 centre=[0.575, 0.425] reason=too_small
- id=R22 cells=1 area=0.0025 m^2 centre=[0.525, 0.475] reason=too_small
- id=R23 cells=1 area=0.0025 m^2 centre=[0.475, 0.525] reason=too_small
- id=R24 cells=1 area=0.0025 m^2 centre=[0.425, 0.575] reason=too_small
- id=R25 cells=3 area=0.0075 m^2 centre=[0.342, 0.642] reason=too_small
- id=R27 cells=3 area=0.0075 m^2 centre=[-0.025, 0.975] reason=too_small
- id=R32 cells=2 area=0.0050 m^2 centre=[0.05, 1.625] reason=too_small

Notes:
- Recovery geometry: residual slivers are one-pass strips, so each kept domain gets its LONGEST in-domain pass; the passes are then
 chained in greedy nearest-neighbour order over the executable free set (transfers inside free cells only).
- Budget rule: stop when the extra distance exceeds the budget (x1.50 of the planned recovery length).  The sim side (budgeted resume execution + before/after audit) is a separate step; its delta stays 待测 until run.
- Cost/effect (planning layer): recovery route 48.350 m for 4.5600 m^2 of extra coverage = 10.60 m/m^2, 0.80x the main plan length (60.75 m).  This is the 'endless loops for the last slivers' risk the round ruling warned about, now made visible.  Execution-layer delta stays 待测 until a resume/recovery sim is run; the budget cap (72.525 m, x1.50) is the agreed stop rule.

