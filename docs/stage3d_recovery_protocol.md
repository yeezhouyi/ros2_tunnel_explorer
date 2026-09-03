# Stage 3D 恢复与长任务会话协议

状态:现行协议(coverage-cleaning-track 分支)。本文合并两件事:
入口回环恢复机制(Stage 3D)与 WSL2 长任务的多会话接力协议。

## 1. 入口回环恢复(Stage 3D,commit e3d80fbb)

**触发**(滑动窗口,`frontier_explorer_node.cpp`):最近 6 个 goal 中
**不同空间 bin 数 ≤ 2**(`loop_bin_size=0.75 m`)且**成功数 ≥ 3** →
判定入口振荡。

**动作**:暂停常规前沿选择,改发前向恢复探针——距离 0.8–1.2 m,
偏航 0°/±20°/±35°,把机器人推出入口吸引域后恢复常规前沿评分。

**效果**(分支隧道 Y 世界,5-run):explorer 级完成 2/5 → 5/5,
mean unique bins 4.0 → 6.0,revisit 49.3% → 34.6%,Nav2 success 100%,
恢复探针 4/4 成功打破振荡。

## 2. 多会话接力协议(R12:短会话绕开 WSL2 稳定性窗口)

单次覆盖任务耗时超过 WSL2 仿真稳定窗口,因此按会话切片:

1. 每会话上限 `SESSION_SECONDS=420`(墙钟),到点后客户端对
   `ExecuteCoverage` action 发**优雅取消**,executor 保存 checkpoint。
2. checkpoint 落在 `~/tunnel_coverage_checkpoints/<task>_<plan>.cp`
   (commit 1ba013e:从 /tmp 迁出,WSL 重启不丢)。
3. 下一会话以 `--resume <checkpoint>` 续扫,直至 terminal result 0/1
   或 `segments_pending=0`。

### 会话环境要求

- 时钟源切换为 `hyperv_clocksource_tsc_page`(默认 `tsc` 在长会话中出现
  TF 时间回跳 / RTF 塌陷;切换命令见 2026-09-04 会话记录)。持久化建议
  systemd oneshot 服务,**不要**依赖 ~/.bashrc(sysfs 写入需要 root)。
- 会话内串行:同一时刻只允许一个 Gazebo 实例;重 CPU 后台任务(如 RL
  训练)在 relay 期间 SIGSTOP 挂起,避免 RTF 下降导致会话有效覆盖缩水。

### 已知问题与防御

- 优雅取消后 executor 不返回 CANCELED 结果,rclpy 客户端可能无限等待
  (见 `known_issues.md`)。防御:客户端 `os._exit(3)` 兜底 +
  relay 以 `timeout` 硬停客户端 + NO_RESULT 时回退最新 checkpoint 续扫。
- relay 脚本支持 `INITIAL_RESUME=<cp 路径>`,relay 自身中断后可从断点
  重启,不重跑已完成会话。

## 3. 待办

- executor 侧 CANCELED 终态返回的 C++ 修复 + 单测(根因)。
- clocksource 修复前后各一轮完整覆盖数据对照,量化时钟源对 RTF 的
  影响(与 mpc_controller 的 deadline-miss 归因共用结论)。
