# Stage 3C 失败分析:分支拓扑泛化(entrance oscillation)

状态:**分析完成,重测已执行(云端,2026-09-04)——结果 1/5 完成,结论受协议差异限制**(见 §5)。

## 1. 失败事实

- 场景:`tunnel_worlds/worlds/branching_tunnel_y.sdf`(Y 型分支隧道)。
- 正式结果:5-run **2/5 explorer 级完成**(README Stage 3C 记录 FAIL,completion 40%)。
- 失败模式:explorer 在 Y 型入口附近在"进入支路"与"返回干道"两个前沿之间反复
  横跳(goal 反复在相邻空间 bin 之间切换),不产生净探索进展,直至超时。
- 3 个失败 run 的共性:入口处存在两个代价接近的前沿簇,information-gain 评分
  差距小于评分噪声(revisit 惩罚不足以覆盖切换成本)。

## 2. 根因链

1. **前沿评分无滞回**:两个前沿簇得分交替领先,每周期最优解翻转。
2. **入口几何放大**:Y 型入口处支路前沿与干道前沿空间距离小于 revisit 半径,
   惩罚项对两者同时生效,拉平了评分差。
3. **无回路检测**:explorer 无法识别"最近 N 个目标都落在同一小区域"这一
   振荡特征,行为上表现为无限重试而非触发恢复。

## 3. Stage 3D 修复(commit e3d80fbb)

滑动窗口回路检测器:最近 6 个 goal 中,若**不同 bin 数 ≤ 2 且成功数 ≥ 3**,
判定入口振荡,改发前向恢复探针(0.8–1.2 m,偏航 0°/±20°/±35°)替代重复
选择同一入口前沿。3D 正式结果:5/5 完成、mean unique bins 4.0→6.0、
revisit 49.3%→34.6%、Nav2 success 100%、恢复探针 4/4 打破振荡。

## 4. 复现实验(run_stage3c_retest.py)

**假设**:3D 恢复栈在 3C 的 Y 型分支场景上同样消除入口振荡,完成率
2/5 → ≥4/5。

- 分支:`stage3d-entrance-loop-recovery`(恢复逻辑默认启用)。
- 世界:`branching_tunnel_y.sdf`,headless,rviz off。
- 每轮:cleanup → stage0_simulation.launch.py → rosbag(/clock /map /tf
  /odom frontier_markers)→ frontier_explorer.launch.py(默认参数即 3D)→
  监控完成(frontier markers 空且无活跃 Nav2 goal 持续 90 s)或 1200 s 超时。
- 判定:explorer 级完成(COMPLETED/TIMEOUT),goal 事件数留档;bag 供
  unique bins / revisit 离线复算。
- 注意:重测与 RRBot 线的 PPO 训练并行(CPU 竞争容忍——本实验指标是
  离散完成判定,不取实时性结论)。

## 5. 重测结果(2026-09-04,云端 Ubuntu,单实例)

| run | status | elapsed_s | goal_events |
|-----|--------|-----------|-------------|
| 1 | COMPLETED | 1076.7 | 12 |
| 2 | TIMEOUT | 1200.2 | 53 |
| 3 | TIMEOUT | 1200.2 | 8 |
| 4 | TIMEOUT(0 goal,疑似 explorer 启动失败) | 1200.5 | 0 |
| 5 | TIMEOUT | 1200.0 | 2 |

聚合:**1/5 COMPLETED**(判定:frontier markers 清空且无活跃 Nav2 goal 持续 90 s;上限 1200 s)。

### 5.1 与基线的可比性限制

- 3D 正式的 5/5、3C 基线的 2/5 使用**当时的临时 runner**(未入库),
  其完成判据/超时/稳定窗与本次不同,2/5 → 1/5 **不构成恢复机制负收益的证据**。
- run_02 全程活跃(53 goal)未收敛、run_04 零 goal(explorer 启动异常),
  方差来源混杂:恢复触发阈值、探索目标数上限、监控判据。
- 每轮 rosbag(/map /tf /odom frontier_markers)保留于云端
  /root/stage3c_retest/run_*/bag,供离线复算 unique bins / revisit。

### 5.2 下一步

1. 将 3D 正式协议的判据(原 runner)复刻为可入库脚本后对齐重测;
2. run_04 类零 goal 失败单列 explorer/Nav2 就绪性检查;
3. 恢复探针计数在重测中未验证(bag 离线分析补)。
