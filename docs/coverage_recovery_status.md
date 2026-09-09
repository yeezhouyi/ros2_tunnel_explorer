# Coverage & Recovery Closure Status (2026-09-09)

> Companion to `docs/advanced_round_resume.md`.  Pins the **honest status** of
> the two advanced-round items that have a planning/module half and an
> execution half, so readers cannot mistake a planning module for a closed
> loop.  Baseline: `main` @ `v1.0.0-sealed` (`b162fc1`).
> Coverage 主口径 = `docs/seal_results.json`（r015 封面 / r010 保守带）。

---

## 1. Interruption recovery（任务恢复）— STATUS: **module-level completed；live E2E 明确降级**

Per the range decision this round may deliver *“一个有效检查点成功恢复；一个不兼容检查点
被明确拒绝；一个损坏/不完整检查点不会导致任务误执行”* at unit level and defer a live
kill/resume smoke to a full chain window.  We take that option explicitly.

### 1.1 What is verified (evidence)

| 验收点（range 口径） | 证据 | 层 |
|---|---|---|
| 有效检查点恢复：身份匹配 + 段数/ID 对齐 | `test_checkpoint_store.cpp` VALID load；`test_task_core.cpp` count/ID alignment；executor node 加载 checkpoint 应用 dispositions + grid（`coverage_executor_node.cpp`，code-reviewed） | store / core / node(源码) |
| 不兼容检查点（map/plan MISMATCH）被明确拒绝 | store identity check（MISMATCH）gtest | store |
| 损坏/不完整检查点不导致误执行 | CORRUPT 拒绝 + **`IncompleteCheckpointIsRejectedAtomically`**（`dispositions.size() != segments_.size()` 在任何写入前拒绝，core 状态不变）`4739ed3` | store / core |
| 续跑语义：pending 段从 checkpoint 续跑、访问计数继承 | `coverage_task_core.cpp` 加载段数/ID + node 应用 dispositions；gtest `*Checkpoint*` 3/3 | core / node(源码) |
| 构建/回归 | `colcon build` + `colcon test`（executor 包；已知 xmllint 环境超时除外） | build |

Automated coverage today = **store + core**（checkpoint gtest 3/3 等）；executor **node** 的
加载/应用路径是源码级接线 + 编译通过，尚无自动化 node 测试；**没有任何一次进程级
kill→respawn→续跑**的运行证据。

### 1.2 What a live E2E would add (and why deferred)

在 canonical 链（`cleaning_room_rect` 静态图 + AMCL + Nav2 + executor）上：

1. 运行覆盖任务并在中途（若干段完成后）**kill executor 进程**（保留 checkpoint 文件）；
2. 重新 launch executor，goal 携带 `--resume <checkpoint>`；
3. 验证：仅 pending 段被续跑、已覆盖段不重复计账、ledger/网格延续；
4. 用 `audit_b6_triple.py` 对前后 bag 做 r015 审计，报告恢复后覆盖率无异常。

该步骤需要一次完整链窗口（单次运行 10+ 分钟、含 Nav2/AMCL 启动与段执行），且有
kill-injection 脚本与 checkpoint 保存时点需新增；按本轮范围裁定**延期**，降级为模块级完成。

### 1.3 对外主张

> 中断恢复 = **模块级验证完成**（store/core 原子拒绝 + 应用路径 + 单元测试），
> **未做**进程级端到端 kill/resume 演示。简历/README 不写 “end-to-end recovery verified”。

---

## 2. Residual re-coverage（残余补扫）— STATUS: **已有规划模块（离线验证）；执行层闭环未做**

### 2.1 What exists (planning module, offline)

`scripts/plan_residual_recovery.py` + `results/coverage_closure/`（run6 输入、r015 口径、
与 seal 同源映射）：基线 **0.7364**（= seal JSON `grid_coverage_task_r015.max`，run6）→
残余 **1844 cells / 4.61 m² / 33 域**（kept 21 / dropped 12=≤20 cells）→ 逐域最长单趟 +
executable 内 A* 连接 → 路线 **48.35 m**（预算 72.5 m = ×1.5）→ 理论 r015 **0.9971**
（Δ +0.2607）。**成本/效益 10.6 m/m²**，裁决 **BUDGETED_ACCEPT**——量化暴露“为最后一点
覆盖反复绕行”风险（路线 ≈ 残余面积的 10.6× 当量里程）。几何见 `residual_overlay.png`。

### 2.2 Why execution-layer delta is NOT closed

coverage executor 对 served map **整图自规划**（C++ ScanlinePlanner），不存在
“喂入外部 Python 残余子图计划”的 API；残余补扫的接口耦合按范围裁定**延期**（不重构
执行状态机）。因此：

> 残余补扫 = **“已有规划模块（离线验证）”**，**不是**“已闭环验证的执行增量”。

闭环判据（未来）：executor 增加外部计划/段级 resume 入口 → 携带残余路线执行 →
预算截断 → 新 bag `audit_b6_triple` r015 前后对比 → 报告 Δ 与里程/耗时/失败数。

### 2.3 对外主张

> 覆盖审计 r015 口径（0.7364 max / mean 0.628 / min 0.5587）为**已封板实测**；
> 残余补扫为**规划层成果 + 理论闭合估计**，执行层 Δ 标为 **待测**，不得与实测数并列。
