# Advanced round 必做3 — fixed-map checkpoint resume (2026-09-09)

Branch: `postseal2-planner-feasibility-20260909`.

## State at round start (already implemented, post-seal era)
- `checkpoint_store` v1: atomic rename write; `loadAndValidate` returns
  OK / NOT_FOUND / CORRUPT / MISMATCH (identity = task_input_id + plan_id).
- Executor node resume path: goal carries `resume_checkpoint_path`;
  OK -> `core->applyCheckpointDispositions` + `tracker->restore(visit_counts)`;
  MISMATCH -> terminal `CHECKPOINT_MISMATCH` (refusal, no resume);
  CORRUPT / NOT_FOUND -> warn + fresh start (never mis-executes the cp).
- Existing tests: store round-trip / identity mismatch / missing+corrupt /
  atomic-failure; core apply-by-id / id-count mismatch throws.

## Gap found this round
`CoverageTaskCore::applyCheckpointDispositions` validated
`checkpoint.segment_ids` (count + per-index ids) but then indexed
`checkpoint.dispositions[i]` **without a length check**.  A truncated /
incomplete checkpoint whose dispositions vector is shorter than the plan
(out-of-band write into the store file, or a hand-built payload) would
read past the vector end -- UB, and a plausible silent corruption of the
disposition table (mis-execution risk).

## Fix
`src/coverage_task_core.cpp`: reject before any write when
`checkpoint.dispositions.size() != segments_.size()` (std::invalid_argument,
mirrors the segment-count guard).  The node's existing catch turns this into
a refused terminal, so nothing is partially applied.

## Tests
`test/test_task_core.cpp`: `IncompleteCheckpointIsRejectedAtomically` --
dispositions shorter AND longer than the plan are both refused and the core
is provably untouched (0 covered / all pending) afterwards.

gtest result (checkpoint filter, 3/3 PASS):
- CheckpointDispositionsApplyById
- CheckpointIdMismatchThrows
- IncompleteCheckpointIsRejectedAtomically

`colcon test --packages-select tunnel_coverage_executor`: all green except
the pre-existing environment-only `xmllint` schema-download timeout (known,
unrelated).

## Acceptance mapping (必做3)
| item | covered by |
|---|---|
| valid checkpoint resumes | store round-trip OK + core apply-by-id + node OK path (live smoke still scheduled) |
| incompatible checkpoint explicitly refused | store MISMATCH test + core id/count throw + node CHECKPOINT_MISMATCH refusal |
| corrupt / incomplete cp causes no mis-execution | store CORRUPT test + new atomic length-reject test |

## Next
Live resume smoke (mid-run kill + relaunch with `resume_checkpoint_path` on
the static rect map) is scheduled with the next chain-run session; module and
store-level closure is complete here.
