# Coverage strategy comparison (same maps, same metrics)

footprint coverage: executable cells within tool radius of the path; medians over 5 seeds per map.

| planner | map | n | OK | footprint_cov | path_len_m | revisit | turns | planning_ms |
|---|---|---|---|---|---|---|---|---|

| astar_greedy | narrow_corridor | 5 | 5 | 1.000 | 671.9 | 0.000 | 110 | 18.8 |
| astar_greedy | open_room | 5 | 5 | 1.000 | 1199.9 | 0.000 | 198 | 33.5 |
| astar_greedy | room_with_islands | 5 | 5 | 1.000 | 1237.6 | 0.019 | 276 | 55.7 |
| boustrophedon_astar | narrow_corridor | 5 | 5 | 1.000 | 219.3 | 0.000 | 34 | 7.2 |
| boustrophedon_astar | open_room | 5 | 5 | 1.000 | 414.5 | 0.000 | 66 | 14.4 |
| boustrophedon_astar | room_with_islands | 5 | 5 | 1.000 | 517.3 | 0.153 | 139 | 19.9 |
| hybrid_astar_greedy | narrow_corridor | 5 | 5 | 1.000 | 1526.2 | 0.529 | 457 | 1043.1 |
| hybrid_astar_greedy | open_room | 5 | 5 | 1.000 | 2957.8 | 0.558 | 805 | 8019.4 |
| hybrid_astar_greedy | room_with_islands | 5 | 5 | 1.000 | 3420.8 | 0.629 | 1065 | 5044.8 |
| row_by_row | narrow_corridor | 5 | 5 | 1.000 | 416.6 | 0.000 | 34 | 0.3 |
| row_by_row | open_room | 5 | 5 | 1.000 | 797.4 | 0.000 | 66 | 0.5 |
| row_by_row | room_with_islands | 5 | 5 | 1.000 | 797.4 | 0.000 | 90 | 0.6 |
| rrt_greedy | narrow_corridor | 5 | 5 | 1.000 | 671.9 | 0.000 | 110 | 245.9 |
| rrt_greedy | open_room | 5 | 5 | 1.000 | 1199.9 | 0.000 | 198 | 534.6 |
| rrt_greedy | room_with_islands | 5 | 5 | 1.000 | 1174.3 | 0.000 | 270 | 514.9 |
