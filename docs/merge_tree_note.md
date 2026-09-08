# Merge tree (Day 1-2) — import record & canonical decisions

Branch: `bline-merge-20260908`
Base: `093ad767785f1b8ca8aeb2f1e597223e14276e17`
  = `stage3d-entrance-loop-recovery` @ 2cddae39
  + `15e498c` (explorer node entrance_hysteresis include — default branch
    compile fix)
  + `093ad76` (.gitignore `src/` anchored to `/src/`)
Imported (file trees from `coverage-cleaning-track` @ `c09074ee`, per
work-instruction §一.2 / delivery-denominator §5 Day 1-2):
- tunnel_coverage_executor
- tunnel_coverage_msgs
- tunnel_coverage_planner
- tunnel_map_core

## Canonical boustrophedon planner: PYTHON (work-instruction §四, decided)

The `stage3d` python `cleaning_mode/boustrophedon.py` (with `_uturn_cap`)
is the canonical plan generator for the sealed full chain:
- the real 69.95 m path (`geo/a8_cap_real.json`, source
  `b6_chain/plan/cleaning_path.json`) was produced by it — all existing
  full-chain numbers keep provenance continuity;
- switching the seal-time numbers to the C++ planner would orphan every
  existing number with no time to rebuild.

Therefore in THIS tree the full chain runs: python planner -> plan JSON ->
C++ `tunnel_coverage_executor` segment execution.  The C++ planner
classes inside `tunnel_coverage_planner`
(`boustrophedon_decomposer.cpp`, `scanline_planner.cpp`,
`residual_planner.cpp`) are IMPORTED for infrastructure but are NOT part
of the canonical chain — they are the "performance rewrite / next phase"
item (README framing, not debt).  `cleanable_map_builder.cpp` in the same
package is canonical infrastructure (mask / exempt-denominator source,
B3).

Any full-chain run script in this tree must reference the python planner
output; do not wire the C++ decomposer into the run path.

## Day-1 regression note

This branch carries no behaviour change to either side's sources beyond
the two base fixes; import only adds files.  The merge is exercised by
building the four imported packages + the stage3d python packages in the
same workspace and running the B6-era chain entry points (Day 6-8).
