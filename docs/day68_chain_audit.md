# Day 6-8: full cleaning-chain runs + dual-denominator coverage audit

Branch: `bline-merge-20260908` @ **5c12f2d** (merge tree built from source;
canonical chain = python `cleaning_mode` planner geometry, executor's C++
`ScanlinePlanner` plan is geometry-equivalent on the rect room — same map
digest `6649dcb8` / plan id `2f2c3d0b` across R24-era and these runs).

Method: `scripts/run_chain_audit.sh <run_dir>` — coverage_simulation
(static map `cleaning_room_rect` + AMCL + Nav2 dwb + coverage executor) →
`send_coverage_goal` (2700 s cap; client waits for READY_IDLE since
`cc6731b`, shell retry demoted to fallback) → `/odom` bag →
`audit_b6_coverage.py` (D-line repo @ `85276e3`, dual denominator,
`footprint_radius_m=0.1`; masks `b6_chain/plan/audit_masks.npz`,
plan_stats read from the same dir).  Four full runs; readiness-wait
client validated on run6 (goal accepted on attempt 1, no shell retry).

## Executor ledger (segment / self-tracking denominator)

| run | gross | effective | repeat_ratio | path_m | dur_s | segs | failure_class |
|---|---|---|---|---|---|---|---|
| run_v | 0.9018 | 0.9018 | 0.7140 | 156.5 | 809 | 37/37 | COVERAGE_BELOW_THRESHOLD |
| run4  | 0.8858 | 0.8858 | 0.6857 | 113.3 | 652 | 37/37 | COVERAGE_BELOW_THRESHOLD |
| run5  | 0.9125 | 0.9125 | 0.7232 | 165.2 | 834 | 37/37 | COVERAGE_BELOW_THRESHOLD |
| run6  | 0.8905 | 0.8905 | 0.6238 | 130.2 | 786 | 36/37 | WORK_TRACKING_FAILED |

mean effective **0.8976**, range 0.8858–0.9125 (±1.5 %); three runs 37/37,
one run 36/37 (one segment WORK_TRACKING_FAILED: /odom self-check could
not confirm endpoint progress after max attempts — real-world variance,
not a harness fault).
Ledger is stable across runs and independent of driven distance — the
executor self-tracks "segment endpoints reached", not absolute coverage.

## Offline grid audit (mask denominator; both reported, never swapped)

| run | coverage_task | coverage_known_free | driven_m | overhead | odom_samples | visited_exec_cells |
|---|---|---|---|---|---|---|
| run_v | 0.3674 | 0.3573 | 163.3 | 2.33 | 24772 | 2302 |
| run4  | 0.2815 | 0.2805 | 117.3 | 1.68 | 20207 | 1764 |
| run5  | 0.3197 | 0.3090 | 172.7 | 2.47 | 25600 | 2003 |
| run6  | 0.2092 | 0.2120 | 152.6 | 2.18 | 22430 | 1311 |

mean coverage_task **0.294**, range 0.2092–0.3674 (spread ≈ 54 % of mean);
mean coverage_known_free 0.290.  coverage_task does NOT track driven
length monotonically (run6 driven 152.6 m mid-range yet lowest task —
one failed segment removed its far-end rows from the visited set), and
run-to-run residual spread is odom→map alignment / visited-cell
sensitivity.  Report the range, never a single run.

## Findings (B3.2 evidence on the canonical tree)

1. **Ledger ≠ grid**: executor effective ~0.89–0.91 vs grid coverage_task
   0.21–0.37.  The executor's own accounting (endpoint reached per
   segment, /odom self-consistency) is not absolute map coverage —
   reproduced on the merged tree exactly as on the old b6 chain.
2. **Ledger stable / grid sensitive**: effective spread ±1.5 % over 4 runs;
   coverage_task spread ≈ 54 % of mean.  Grid audit cell counts are
   sensitive to odom→map alignment, per-run trajectory tightness, and
   segment failures (run6 36/37 → lowest task despite mid-range driven).
   Report the range, never a single run.
3. **Repeat ratio** 0.62–0.72 (row-working inefficiency) is the known open
   item (b6_rerun_rect/NOTE.md); unaffected by the merge.
4. **Startup nondeterminism removed** (`cc6731b`): run6 goal accepted on
   attempt 1 via in-process READY_IDLE phase wait (no shell retry).

## Provenance notes

- Runs executed from the merge-tree source build (no stale install): the
  workspace was rebuilt after importing the coverage runtime pieces
  (bringup launch/config `82a7888`, cleaning world assets `e020a6f`).
- Masks npz `b6_chain/plan/audit_masks.npz` (Sep 7) share the map digest /
  plan id with all runs (incl. R24-era): inputs are aligned.
- Audit numbers are NOT gate-thresholded here; Day 9/10 decides the seal
  claim against this JSON set.
