# Task resume verification (必做3, 2026-09-09)

Scope of this round ruling: a checkpoint produced by a previous run
must let a fresh process **continue the unfinished segments**, not
restart from scratch and not silently skip malformed input. The fixed
map is cleaning_room_rect (served static map, origin -3/-2); the audit
gauge stays shift(0,0).

## Data flow (3.1: resume data path)

The full chain is in-tree and exercised at unit-test level:

```
 coverage_executor (running)
   -> coverage_task_core.applyCheckpointDispositions(checkpoint)
   -> checkpoint_store.cpp  saveCheckpoint() on cancel / FAILED
                              verifyIdentity()  on next start
   -> send_coverage_goal.py --resume <checkpoint path>
       -> ExecuteCoverage.goal.resume_checkpoint_path
   -> executor loads checkpoint, verifies identity, replays segment
      dispositions + grid, then continues only the pending segments
```

Identity check: `checkpoint_store.verifyIdentity()` validates
`map_digest == served map digest` (MISMATCH -> refuse) and the JSON
shape (CORRUPT -> refuse).  Parameter `max_attempts_per_segment=2`
(default) bounds retries; `min_effective_coverage=0.97` sets the
terminal coverage gate.  Checkpoint directory is fixed by
`coverage_executor_params.yaml::checkpoint_dir`
(`/home/zhouyi/tunnel_coverage_checkpoints`).

## Unit-level acceptance (3.2: three classes)

| case | code | result |
|---|---|---|
| valid checkpoint, plan + grid intact, dispositions match segments | apply loads + segments restart | OK (existing `coverage_executor_node.cpp` resume path) |
| incompatible (map digest differs) | MISMATCH refused | OK (`checkpoint_store::verifyIdentity`) |
| malformed (JSON truncated / not parseable) | CORRUPT refused | OK (`checkpoint_store::verifyIdentity`) |
| **NEW** incomplete (segment_ids match the plan but **dispositions is shorter**) | refuse atomically, no segment is overwritten | OK (commit `4739ed3`, `applyCheckpointDispositions` now checks `dispositions.size() == segments_.size()` **before any write**) |

New test pinned at the core level:
`CoverageTaskCore::IncompleteCheckpointIsRejectedAtomically` — feeds
a 3-segment plan with a 1-element dispositions vector and asserts
`std::invalid_argument` is thrown AND the core's plan/dispositions are
unchanged afterwards (no partial write).

## Build + gtest (already run on this round)

```
 colcon build --packages-select tunnel_coverage_executor    -> exit 0
 colcon test  --packages-select tunnel_coverage_executor    -> exit 0
 ./build/tunnel_coverage_executor/test_task_core
   --gtest_filter='*Checkpoint*'                            -> 3/3 green
```
The pre-existing xmllint env-timeout on the audit step is unrelated
and was retracting before this round (recorded in
docs/day68_chain_audit.md).

## Live smoke (separate from the unit evidence)

A live resume smoke that cancels a run mid-task and reloads through
the executor **was not run in this round** -- it requires the WSL
long-session chain (gz + AMCL + Nav2 + executor) and was deferred in
favour of sealing the unit-level guarantee first.  The data flow and
identity check above are the verified part; the live layer stays
待测 until the next chain window.

## Commit pointers

- core OOB guard + atomic refusal: commit `4739ed3` on branch
  `postseal2-planner-feasibility-20260909`.
- node resume application (existed pre-round, confirmed in
  `coverage_executor_node.cpp` around the `loadCheckpoint()` call site).
- store identity check (existed pre-round, confirmed in
  `tunnel_coverage_executor/src/checkpoint_store.cpp`).