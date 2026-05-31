# Known Issues

This document tracks pre-existing failures and known issues that are not
related to the currently active development stage.

Last updated: 2026-05-31

---

## benchmark_tools: 21 Pre-existing Test Failures

Discovered during Stage 1A+1B development of `tunnel_frontier_explorer`.

### Failure Summary

| Test Suite | Failures | Type | Root Cause |
|---|---|---|---|
| `test_metrics` | 3 | pytest infrastructure | Missing `lark` Python module in system Python |
| `flake8` | 15 | lint style | Import ordering, undefined names, docstring, indentation |
| `pep257` | 3 | docstring style | D205/D400/D415 in `stats.py` |

**Total**: 21 failures, 1 error (the error is the missing-result error from the
pytest crash, which is a cascade of the missing `lark` module).

### Detail

#### 1. `test_metrics` — pytest crash (3 failures + 1 error)

- **Test name**: `benchmark_tools.test_metrics`
- **Failure type**: pytest infrastructure / missing system dependency
- **Error**: `ModuleNotFoundError: No module named 'lark'`
  ```
  from launch.frontend.parse_substitution import parse_if_substitutions
  from lark import Lark  # ← missing
  ```
  The `lark` module is required by `launch_testing`'s import chain but is not
  installed in the current Python environment.
- **Why unrelated to `tunnel_frontier_explorer`**:
  The error occurs in `launch_testing` → `launch` → `lark` import chain, which
  is only triggered when pytest collects tests for `benchmark_tools`. It is a
  missing pip/system dependency in the workspace Python environment, completely
  independent of the C++ frontier explorer package.
- **Planned fix**: Stage 2 (benchmark baseline tooling), install `python3-lark`
  or add pytest marker to skip launch-dependent tests when lark is absent.

#### 2. `flake8` — 15 style errors

| # | Error Code | File | Description |
|---|---|---|---|
| 1 | D205 | `benchmark_tools/stats.py:27` | 1 blank line required between summary and description |
| 2 | D400 | `benchmark_tools/stats.py:27` | First line should end with a period |
| 3 | I100 | `scripts/run_navigation_smoke_test.py:42` | Import order: `action_msgs` before `rclpy` |
| 4 | I100 | `scripts/run_navigation_smoke_test.py:51` | Import order: `benchmark_tools.stats` before `nav2_msgs` |
| 5 | E127 | `scripts/run_navigation_smoke_test.py:244` | Continuation line over-indented |
| 6 | E127 | `scripts/run_navigation_smoke_test.py:248` | Continuation line over-indented |
| 7-14 | F821 | `tests/test_stats.py:276-347` (8×) | Undefined name `compute_navigation_summary` |
| 15 | W391 | `tests/test_stats.py:349` | Blank line at end of file |

- **Why unrelated to `tunnel_frontier_explorer`**: All errors are in
  `benchmark_tools/` Python source files and test scripts. The
  `tunnel_frontier_explorer` package is pure C++ and has its own clean flake8
  check (0 errors). The F821 errors indicate that `compute_navigation_summary`
  is not imported in `test_stats.py` — either the import was omitted or the
  function was moved/renamed.
- **Planned fix**: Stage 2 (benchmark baseline tooling), fix import statements
  and style issues in the benchmark tools codebase.

#### 3. `pep257` — 3 docstring errors

| # | Error Code | File | Description |
|---|---|---|---|
| 1 | D205 | `benchmark_tools/stats.py:27` | 1 blank line required between summary and description |
| 2 | D400 | `benchmark_tools/stats.py:27` | First line should end with a period |
| 3 | D415 | `benchmark_tools/stats.py:27` | First line should end with a period, question mark, or exclamation |

- **Why unrelated to `tunnel_frontier_explorer`**: Same file
  (`benchmark_tools/stats.py`), same root cause — the `check_duration()`
  function's docstring is missing a blank line after the summary line and a
  period at the end of the first line.
- **Planned fix**: Stage 2 (benchmark baseline tooling), fix docstring format.

---

## TunnelFrontierExplorer Test Status

```
tunnel_frontier_explorer: 26/26 GTest PASS
tunnel_frontier_explorer: 10/10 lint checks PASS
```

All `tunnel_frontier_explorer` unit tests and linter checks pass
independently. The 21 failures above are confined to the `benchmark_tools`
package and do not affect the frontier explorer's correctness or build
validity.
