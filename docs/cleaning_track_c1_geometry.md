# Cleaning Track — C1 Map Geometry and Cleanable-Region Semantics

Referenced by `tunnel_coverage_planner/src/cleanable_map_builder.cpp`.

## Frame and grid conventions

All coverage code uses the same row-major `OccupancyGrid` convention as
`tunnel_frontier_explorer`:

- index `row * width + col`; `row 0` is the bottom row (smallest world y);
- cell centre world pose: `(origin_x + (col+0.5)*res, origin_y + (row+0.5)*res)`
  for yaw = 0;
- `GridGeometry` also supports a non-zero `origin_yaw` for generality.

## Four masks (R1, R19)

| mask | definition | role |
|---|---|---|
| `intended_target` T | free cells (value ∈ [0, free_max]) | gross denominator, cleaning goal |
| `navigable_center` N | free eroded by chassis clearance disc (`robot_radius + safety_margin`) | robot centre may occupy |
| `reachable_cleanable` R | `(N-component(seed) ⊕ tool disc) ∩ T`, minus sub-min-area islands | effective denominator T\E after exempt removal |
| `exempt` E | `T \ R` with per-cell cause (`UNREACHABLE` or `ISLAND`) | auditable exceptions |

Invariants:

- `N ⊆ T`, `R ⊆ T`, `E ⊆ T`, `|T| = |R| + |E|`;
- a corridor narrower than `2 * (robot_radius + safety_margin)` vanishes from
  `N` (the builder rejects a seed inside it instead of emitting an empty plan);
- off-map neighbours are never free (the chassis may not leave the map);
- with a centred footprint whose sweep radius ≥ chassis clearance, plain
  single-room scenes keep `E = ∅` (wall strips are reachable through the
  tool), matching the rect-world MVP gate exempt ratio 0.

## Example numbers (defaults)

| parameter | value |
|---|---|
| `robot_radius_m` | 0.13 |
| `safety_margin_m` | 0.05 → clearance 0.18 m |
| `cleaning_width_m` | 0.50 → sweep radius 0.25 m |
| `min_cleanable_region_area_m2` | 0.04 (island gate ≈ 16 cells @ 0.05 m) |

Seed pose = deterministic robot spawn of the benchmark world
(`tunnel_worlds/maps/initial_poses.yaml`).
