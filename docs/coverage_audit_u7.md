# U7 覆盖率审计(2026-09-06,本地干净运行)

运行:U6 无 checkpoint 单次连续运行,时钟源 hyperv_clocksource_tsc_page,
终态 result=2 COVERAGE_BELOW_THRESHOLD,有效覆盖率 56.74%(37/37 段全盖,
duration 890.6s)。bag:/map /odom /tf /coverage/status(mcap)。

## 公式(coverage_tracker.cpp:171-215)

    effective_coverage = |target ∧ ¬exempt ∧ visited| / |target ∧ ¬exempt|
    intended_target = known_free(cleanable_map_builder.cpp:263)
    sweep_radius = cleaning_width/2 = 0.25m
    spacing = min(w(1-η), max(w-2(err), .02)) = min(0.45, 0.28) = 0.28m

## 审计发现(scripts/audit_coverage_bag.py,证据 artifacts/coverage_clean/audit/)

- F1 规划欠覆盖(根因):plan 总长 22.55m,而 0.28m 间距全扫需 68.8m
  (free 19.25 m²)→ plan_undercoverage_ratio = 0.328。37 段只铺了
  ~1/3 的必需扫描线 → 即使完美执行也只能到 ~1/3+重叠补偿。
- F2 执行膨胀:odom 实际行驶 178.5m = plan 的 7.9 倍(repeat 16.4%,
  Nav2 跟踪震荡)。执行不是瓶颈,规划才是。
- F3 标记语义差:bag 重建(odom × 0.25m 足迹)得 coverage_known_free
  = 70.2%,executor 报 56.7% → executor 的 visited 标记或 exempt 掩膜
  与朴素足迹模型不一致,需在 U8/U9 对齐(导出 executor 侧 mask)。

## 结论

56.9%(relay)/ 56.74%(干净)都是真实终态;不是接力伪影,不是执行
失败,是**规划器生成的扫描线总长不足**(F1)。修复方向:分解/段生成
覆盖全部 free cell(检查 bcd/scanline 的 cell 划分边界),或提高
spacing 利用率;修复后按 R10 同时报 task/known_free 两个分母。

状态:COVERAGE_BELOW_THRESHOLD 保留;阈值未动;审计 JSON 与 .npy
mask 已归档(known_free/obstacle/unknown/visited)。

## 追加(代码级定位,2026-09-06)

离线复现 plan(/tmp 工具,链 install 库,同地图同参数):
- masks: intended_target = reachable_cleanable = 7700(ratio 1.000)→ 无 mask 级排除;
- planMultiCell: segments_total=37(19 WORK + 18 转移)与线上一致;
- WORK 段 19 × 3.05m = **57.95m,计划本身足以全覆盖**(seed 位姿无关)。

**F1 修订(真根因)**:coverage_executor_node.cpp:575-580 ——
`addSweepSegment(work_row_start_pose_, end_pose)` 只把"行起点→Nav2 报告成功时
位姿"的**直线弦**沿途 stampDisc;Nav2 实际弧线行驶的扫掠不写入
(tfSamplerCallback 明确不写 grid)。弦长合计 22.55m(计划的 39%),这就是
56.7% 的来源;`markCovered` 无条件执行,"37/37 covered" 分母还混入 18 个
转移段,是虚账。

修复方向(单机制,需回归对照):
a) WORK 段执行期间按 odom/TF 采样写入 CoverageGrid(去重后计 repeat),
   替代起点→终点弦;或
b) recordSegment 记**计划段**几何并如实报告弦长/计划长比值。
两案都需重跑 U6 干净对照 + 回归 cleaning_room_rect 的 37 段行为。

---

# U8 对齐 5-run 结果(2026-09-06,stage3c-aligned-v2)

## 执行

frozen driver v2(run_stage3c_aligned.py,856bbb8):timeout 1500s、
稳定窗 120s、初始位姿 (0,1) 全字段声明;每 run manifest 携带完整协议。

| seed | status | elapsed | goals |
|---|---|---|---|
| 1 | TIMEOUT | 1500.4 | 22 |
| 2 | TIMEOUT | 1500.2 | 32 |
| 3 | TIMEOUT | 1500.1 | 32 |
| 4 | TIMEOUT | 1500.0 | 26 |
| 5 | TIMEOUT(goals=1,explorer 启动异常单列) | 1500.0 | 1 |

**0/5 COMPLETED**(判定:frontier markers 清空且无活跃 Nav2 goal
持续 120s)。

## 判定语义差异 = 可比性边界

- 历史 3D 5/5 用的是原始 driver 的 **explorer 内部完成信号**(已失传);
- v2 判定是"全部前沿消费完且静止 120s"——seed 1-4 的 goal 事件流
  显示探索器 25 分钟内持续派发目标(无振荡死锁),是**未在 1500s 内
  消费完前沿**,而非入口振荡复发;
- 结论:历史 5/5、2/5 与 v2 的 0/5 属于**三种不同测量**,按 U8 规则
  分组并列,不互相冒充。

## 后续(真正闭合 U8)

1. 把 explorer 内部完成语义(完成信号/覆盖率阈值)从源码反推并
   写进 driver,重跑对齐;
2. seed 5 类启动异常单列(frontier explorer 就绪性检查);
3. 1500s 不够则按宣告式修订协议版本(v3),仍全字段声明。
