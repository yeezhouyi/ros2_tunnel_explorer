# Engineering checklist — ros2_tunnel_explorer (核验矩阵)

最终定位：能管理探索/覆盖任务、恢复执行并输出结果的导航应用。
原则同 D 线：只核验不重写。

| # | 能力 | 状态 | 证据 | 缺口 |
|---|---|---|---|---|
| 1 | 模块责任 / 三口径分离 | ✅（本批补齐） | 规划覆盖（plan_covered_frac）、执行完成（executor 段 COVERED）、面积覆盖（audit 工具口径）三者已分开度量：`artifacts/miss_classification/`（plan 96.97% vs visited 56.7-61.6%）+ `docs/day68_chain_audit.md` 双分母口径 | 无 |
| 2 | 任务状态机 | ✅（部分） | `TaskPhase`：BOOTSTRAP→WAIT_MAP→WAIT_LOCALIZATION→WAIT_NAV2→READY_IDLE→EXECUTING→RESIDUAL_CHECK→CANCELLING→终态；转换触发条件在代码 + 契约（R17 只在 READY_IDLE 接受任务） | 暂停/恢复状态未实现——记为缺口（当前只有取消+检查点续跑） |
| 3 | 检查点与恢复 | ✅（部分） | `checkpoint_store`：任务/plan 标识校验（R12，地图或配置变化拒绝恢复）、已完成段恢复、`test_checkpoint_store.cpp` 单测；每段完成即 `saveCheckpoint` | 受控仿真中的"中断一次→恢复"端到端演示未单独录制——记为待办 |
| 4 | 依赖就绪与故障处理 | ✅ | READY 门控（无地图/定位/Nav2 不接受任务）、child_goal 超时 watchdog（有界取消 grace）、每段有界重试（max_attempts）→ 失败分类（WORK_TRACKING_FAILED / APPROACH / TRANSITION） | 无 |
| 5 | 可定位问题的运行记录 | ✅ | `scripts/run_chain_audit.sh`：每 run 独立目录（launch.log / goal.log / odom bag / metrics.json / audit）；代码版本由 git tag/commit 锚定（postseal2 分支） | 无 |
| 6 | 端到端验收 | ✅ | 6-run canonical 集 + run9/10/11（正常完成 + 失败段 + 恢复路径均出现过并被归因）；取消路径有代码级状态机与测试 | 取消/暂停恢复的专项 e2e 未单独录制——记为待办 |
| 7 | CI | ✅（本批新增） | `.github/workflows/ci.yml`：ros:jazzy 容器 colcon build + 核心包单测（CI 有网络，xmllint schema 拉取在 CI 可通过） | 首跑 GREEN（2026-09-09） |

算法对比证据（P0 已完成，见 `docs/coverage_strategy_compare.md`）与漏覆盖分类（`artifacts/miss_classification/`）支撑"三口径不会混为一个覆盖率"的验收。
