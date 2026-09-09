# chain_day68_evidence/

Post-seal (2026-09-08): the small, git-trackable half of the Day 6-8
chain evidence set, committed so the numbers do not depend on one
machine being alive.  The large half (raw /odom bags, 17-23 MB each)
stays local under `~/chain_day68/` by policy and is reproduced by
re-running the simulation (`scripts/run_chain_audit.sh`).

Layout (all byte-identical copies of the local run directories):

- `rect_plan/` - the gauge inputs: `audit_masks.npz` (served-map
  plan_from_map masks, executable 6996 cells), `cleaning_path.json`,
  `plan_stats.json` (python-canonical plan 60.75 m / 22 waypoints).
- `triple_s0_sealed4/` - `audit_b6_triple.py` output over the SEALED
  4-run set, shift (0,0): the numbers in `docs/seal_results.json`
  (r015 mean 0.628, 0.559-0.736; r010 mean 0.504).
- `triple_s0_v3_6run/` - the same audit extended with run7/run8
  (r015 mean 0.620, 0.476-0.736): the post-seal extension evidence.
- `run_ledger/` - per-run executor result JSONs (segment endpoints,
  effective coverage, failure class).
- `run_audit/` - per-run first-pass (single-radius, inline) audit JSONs;
  these are the 0.11-0.37 "raw family" numbers and are NOT the reported
  gauge (see docs/day68_chain_audit.md "Post-seal attributions" and the
  gauge-correction notes).

Canonical tools: `audit_b6_coverage.py` (@ 85276e3) and
`audit_b6_triple.py` (@ fd1f493, marking logic unchanged through
77fe26c) in linear_mpc_controller.  Reproduce with
`--masks-npz chain_day68_evidence/rect_plan/audit_masks.npz --sx 0 --sy 0`.
