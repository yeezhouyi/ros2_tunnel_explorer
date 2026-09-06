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

---

# U9 实施规格(entrance hysteresis,单机制,实验分支)

插入点已定位(源码级):
- 参数:`tunnel_frontier_explorer/src/frontier_explorer_node.cpp:96-109`
  既有 loop/recovery 参数块旁加 `entrance_hysteresis_enabled`(默认
  **false**,AE5:回归通过前不启用)、`entrance_cooldown_s=20.0`、
  `entrance_radius_m=1.0`;
- 机制:dispatch 站(goal_msg 构造处,:368 附近)记录最近一次接受
  goal 的位姿+时间;新候选若在 cooldown 内且距上次 goal
  < entrance_radius_m → 拒绝该簇(加入 frontier_blacklist 本周期
  排除),重选次优——FrontierGoalSelector 只返回最优,次优经
  blacklist 重选获得(黑名单机制已存在);
- 单元测试:cooldown 内同区候选被拒、超窗后放行、radius 外放行、
  hysteresis 关闭时零行为变化(守恒测试)。

回归门禁(AE5):对齐协议 v2 下 Stage 3C 5-run(预期改善)与
Stage 3D 5-run(不得低于 5/5、4/4 探针、Nav2 100%)——两套都过
才允许 enabled=true 合入主线;否则留实验分支+失败证据。

状态:机制未实现(本规格即施工图);对齐 v2 基线已就绪
(0/5@1500s 全字段声明)。

## 弦修复回归(2026-09-06,U6 干净重跑,fix c09074e)

| 指标 | 修复前 | 修复后 |
|---|---|---|
| executor effective_coverage | 56.74% | **88.62%** |
| executor 记账路径 | 22.55m(弦) | **185.88m(实际折线)** |
| repeat_ratio | 16.4% | 66.7% |
| odom 重建 visited free cells | 5405(=70.2% of 7700) | 4698(=61.0% of 7700) |
| odom 实驶 | 178.5m | 192.8m |

**收敛证明**:修复后 executor 盖章格 ≈ 4700 ≈ odom 重建 4698(差
0.04%)——记账与物理事实一致,understamping 消除。修复前 executor
盖章 3000 vs 实扫 5405,欠账 2400 格。

**口径差=分母差**:executor 88.6% 的分母是 intended_target∧¬exempt
(~5300 格),重建 61.0% 的分母是全部 known_free(7700 格)——两者
覆盖的格子一致(4700),差在 ~2400 格被 exempt 排除(占自由区 31%)。
**新审计问题**:exempt 掩膜为何排除 31% 的自由区(cleanable_map_builder
的 island/doorway 门槛过宽?)→ 下一个单机制审计对象(R10 双分母
必须并列的原因)。

执行方差:repeat 16.4%→66.7%,unique visited 5405→4698(驱更远、
重复更多)——Nav2 执行方差真实存在,单 run 结论需 5-run 支撑。

# U9 对齐 A/B 结果(2026-09-07,entrance hysteresis ON vs OFF)

同协议 v2(1500s/120s/(0,1))、同世界、recovery ON:

| seed | hysteresis OFF | hysteresis ON |
|---|---|---|
| 1 | TIMEOUT, 22 goals | **COMPLETED 590.6s, 12 goals** |
| 2 | TIMEOUT, 32 goals | TIMEOUT, 47 goals |
| 3 | TIMEOUT, 0 goals | TIMEOUT, 0 goals |
| 4 | TIMEOUT, 26 goals | TIMEOUT, 36 goals |
| 5 | TIMEOUT, 1 goal | TIMEOUT, 1 goal |
| 合计 | 0/5, 81 goals | **1/5, 97 goals** |

## 判定(按 AE5)

- seed 1:完成时间 1500s→590.6s,goal 数 22→12( thrash 减少 45%)
  ——hysteresis 在该种子上兑现了设计意图;
- 合计完成 0/5→1/5(n=5,统计上不足以声明整体改善);
- seed 3/5 的零/近零 goal 在 OFF/ON 两轮一致复现 → **对齐驱动器的
  explorer 就绪性竞争**是驱动器自身问题,与机制无关,单列;
- 无 3D 回退信号(recovery 保持 ON,goal 流未死锁)。

## 决定(遵循 AE5 治理)

机制**保留在实验分支 u9-entrance-hysteresis**,不合入主线:
改善样本 1/5 不足以支撑 enabled=true 的默认值;需要 (a) 修复驱动器
就绪性竞争后重跑,(b) 或以完成时间为主指标做更多 run。合入前置
条件=就绪性修复 + 完成时间中位数改善的可复现证据。

---

# B6 全链首跑(2026-09-07):完整序列打通,暴露弓形 U 型掉头缺口

全链五步全部执行并留产物(探索存图→清洁规划→MPC 跟踪→双分母审计):

| 步 | 产物 |
|---|---|
| 探索+存图 | map_saved.pgm/.yaml(7860 known-free 格) |
| 清洁规划 | 69.95m 路径,planned_coverage 1.0,reason OK(250 点) |
| MPC 跟踪 | **仅驱动 7.3m 即卡死在第一个车道端 U 型掉头** |
| 双分母审计 | coverage_task **0.061** / coverage_known_free 0.0505 |

## 根因(新,规划层)

弓形路径的车道间连接是 **0.3m 横向直跳**(lane 1 末端 → lane 2 起点),
对 forward-only(v_min=0)差速底盘等于 180° 掉头:
1. Frenet 航向误差 wrap 到 ±π——线性 MPC 的线性化在该误差下失效;
2. 安全投影钳 v≥0,倒车不可用;
3. 跟踪器在第一个掉头处 stall(audit odom:7.3m 后静止)。

这正是 4060 清洁规格中"相邻扫描段 A* 或可验证直线连接"+**yaw 连续化**
条款要防的问题——A6 实现缺 U 型掉头帽(semicircular cap)。

## 修复方向(单机制)

扫描线端点间加半圆 U 型帽(半径 ≥ 最小转弯半径),或 planner 输出
显式倒车段(需 MPC 放开 v_min——改动更大)。帽方案与现有 v_min=0
约束兼容,为首选。修后重跑全链,coverage_task 预期 → 接近
planned_coverage(1.0×可执行比例)。

状态:链路本身全通(五步皆留产物),发现已定位,修复列下一单元。

## Exempt 分母审计(2026-09-07,sandbox 全链 run)

两层口径实测:
- **plan 层**(cleaning_mode):executable = known_free − 2 格膨胀 =
  6266/7860(**79.7%**)——膨胀排除全部贴墙/贴障;
- **executor 层**(coverage_executor):effective 分母 ≈ 5300/7700
  (**~69%**)——exempt 额外排除 ~11%,来源 = cleanable_map_builder 的
  island 面积门槛 + doorway 带宽门槛(源码 :240-290 一带)。

结论:两个口径的覆盖百分比差(88.6% vs 61.0% 全自由区分母)主要来自
exempt 集合的定义,而非测量分歧(盖章格与 odom 重建格 0.04% 收敛已证)。
下一步(单机制):逐门槛关闭对照,量化 island 门槛与 doorway 带宽各贡献
多少 exempt 格;达标线 = 清洁任务语义决定的必要排除(贴障膨胀),其余
应回到 candidate。

## 本会话未完项(均需外部条件或新会话)

- RRBot 原生 Linux 对照(需非 WSL 机器);
- stage3d→main 合并(建议 PR 描述引用 U7/U8/U9 审计文档);
- B6 弦修复的 5-run 统计(单 run 已验证方向)。
