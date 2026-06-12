# Benchmark Summary

## Benchmark Evolution

| Stage | Status | Key Finding |
|-------|--------|-------------|
| Stage 2B | PASS | 100% completion, info+revisit scoring works |
| Stage 3D | PASS | 5/5 completion, entrance-loop recovery works |
| Stage 4B.1 | PASS | wall_risk_weak 60% vs 20% baseline |
| Stage 4C | PASS | baseline 72%, escape 61% on corrected benchmark |
| Stage 4D.3 | PASS | 30/30 completed, bootstrap + cooldown guard works |
| Stage 4F | PASS | 48/48 valid subset, all variants 100% |
| Stage 4F.1 | PASS | baseline sufficient for non-straight topologies |
| Stage 4G.1 | PASS | 20/20 final default validation |

## Benchmark Validity Issues Fixed

| Issue | Stage | Fix |
|-------|-------|-----|
| Startup complete map (goals=0) | 4E.2 | Startup bootstrap probe |
| Cooldown infinite loop | 4D.2 | Cooldown loop guard |
| INVALID_STARTUP_COMPLETE_MAP | 4D.0c | Validity check in benchmark |
| Odom TF timing | 4F.0a | Odom wait gate |

## Final Validation

Stage 4G.1: 20/20 completed, all goals=1, no fake completions
