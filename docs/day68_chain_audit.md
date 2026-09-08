# Day 6-8: full cleaning-chain runs + dual-denominator coverage audit

Branch: `bline-merge-20260908` @ **e020a6f** (merge tree built from source;
canonical chain = python `cleaning_mode` planner geometry, executor's C++
`ScanlinePlanner` plan is geometry-equivalent on the rect room — same map
digest `6649dcb8` / plan id `2f2c3d0b` across R24-era and these runs).

Method: `scripts/run_chain_audit.sh <run_dir>` — coverage_simulation
(static map `cleaning_room_rect` + AMCL + Nav2 dwb + coverage executor) →
`send_coverage_goal` (2700 s cap, READY retry loop) → `/odom` bag →
`audit_b6_coverage.py` (D-line repo @ `85276e3`, dual denominator,
`footprint_radius_m=0.1`; masks `b6_chain/plan/audit_masks.npz`,
plan_stats read from the same dir).  Three full runs (37/37 segments
covered, 0 failed, terminal COVERAGE_BELOW_THRESHOLD in all).

## Executor ledger (segment / self-tracking denominator)

| run | gross | effective | repeat_ratio | path_m | dur_s | segs |
|---|---|---|---|---|---|---|
| run_v | 0.9018 | 0.9018 | 0.7140 | 156.5 | 809 | 37/37 |
| run4  | 0.8858 | 0.8858 | 0.6857 | 113.3 | 652 | 37/37 |
| run5  | 0.9125 | 0.9125 | 0.7232 | 165.2 | 834 | 37/37 |

mean effective **0.9000**, range 0.8858–0.9125 (±1.5 %).
Ledger is stable across runs and independent of driven distance — the
executor self-tracks "segment endpoints reached", not absolute coverage.

## Offline grid audit (mask denominator; both reported, never swapped)

| run | coverage_task | coverage_known_free | driven_m | overhead | odom_samples | visited_exec_cells |
|---|---|---|---|---|---|---|
| run_v | 0.3674 | 0.3573 | 163.3 | 2.33 | 24772 | 2302 |
| run4  | 0.2815 | 0.2805 | 117.3 | 1.68 | 20207 | 1764 |
| run5  | 0.3197 | 0.3090 | 172.7 | 2.47 | 25600 | 2003 |

mean coverage_task **0.323**, range 0.2815–0.3674 (spread ≈ 27 % of mean);
mean coverage_known_free 0.316.  driven 117–173 m (Nav2 trajectory
tightness varies run to run) → coverage_task correlates weakly with driven
length; residual spread is odom→map alignment / visited-cell sensitivity.

## Findings (B3.2 evidence on the canonical tree)

1. **Ledger ≠ grid**: executor effective ~0.90 vs grid coverage_task ~0.32.
   The executor's own accounting (endpoint reached per segment, /odom
   self-consistency) is not absolute map coverage — reproduced on the
   merged tree exactly as on the old b6 chain.
2. **Ledger stable / grid sensitive**: effective spread ±1.5 % over 3 runs;
   coverage_task spread ≈ 27 %.  Grid audit cell counts are sensitive to
   odom→map alignment and per-run trajectory tightness; report the range,
   never a single run.
3. **Repeat ratio** 0.69–0.72 (row-working inefficiency) is the known open
   item (b6_rerun_rect/NOTE.md); unaffected by the merge.

## Provenance notes

- Runs executed from the merge-tree source build (no stale install): the
  workspace was rebuilt after importing the coverage runtime pieces
  (bringup launch/config `82a7888`, cleaning world assets `e020a6f`).
- Masks npz `b6_chain/plan/audit_masks.npz` (Sep 7) share the map digest /
  plan id with all runs (incl. R24-era): inputs are aligned.
- Audit numbers are NOT gate-thresholded here; Day 9/10 decides the seal
  claim against this JSON set.
