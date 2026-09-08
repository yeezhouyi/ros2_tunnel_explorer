# B7 (U8) Event Log CSV — Schema v0 (定稿草案, 待 4060 侧确认后实现)

起点: `2cddae399409b7854ace1f2625759eb88e19e5f7` (stage3d-entrance-loop-recovery 头)
实现对象: `tunnel_frontier_explorer` (本次改动只落此包 + 运行侧脚本)

## 0. 设计基线(读代码后定稿)

- 事件流唯一出处: `tunnel_frontier_explorer/src/frontier_explorer_node.cpp`
  (周期 1 s timer → IDLE 检测/选点/发 goal → NAVIGATING → resultCallback)。
- `FrontierCluster` 无任何 id 字段(`frontier_cluster.hpp`),簇身份 = 检测顺序函数。
- 现无 CSV 写出者、无 readiness 信号、无 config/map 溯源设施。
- 目标: 5 种子一次性导出;导出文件路径由参数给定;分析就地跑,不做往返。

## 1. 列定义(每列: 含义 / 来源代码点 / 取值规则)

| 列 | 来源 | 规则 |
|---|---|---|
| schema_version | 常量 | 本次 = 1;列集演进时递增 |
| repo_sha | 启动时 git rev-parse HEAD | 见 §3 repo_dirty 配套,不是独立充分量 |
| repo_dirty | 启动时 git status --porcelain 非空 | 0/1;dirty 时附 diff_sha |
| diff_sha | dirty 时 git diff \| sha256sum | 干净时留空(整行仍定长,用空串) |
| config_hash | 见 §4 | 已解析参数,非 yaml |
| map_digest | 见 §5 | **每行**带该行所在检测周期的地图摘要(非一次 run 一个) |
| seed | 运行参数(launch 注入) | 0..4 |
| t | 事件时刻 sim/墙钟秒 | 单调;heartbeat 行=周期序号×period |
| frontier_count_open | 本周期 detector_.detect() 返回簇数(node ~285) | 含被 blacklist 抑制的 |
| reachable_frontier_count | 同一周期 blacklist 过滤后 `valid.size()`(~295-303) | 供剩余前沿曲线用可达数 |
| goal_id | 单次运行的 goal 流水号(单调递增计数) | 跨 run 不保证一致,不做 join key |
| stable_cluster_id | 见 §2 | 几何定义、检测顺序无关、跨 run 可复现 |
| goal_x / goal_y | 发往 Nav2 的 goal 世界坐标(safe_goal_pt, ~623-624) | frontier goal 用投影后安全点;recovery probe 用探针点;heartbeat 行为空 |
| action | 见 §6 | 受控词表 |
| nav2_result | goalResponseCallback / resultCallback 结果 | 受控词表 |
| nav2_error_code | resultCallback result->error_code | 无错误填 -1;aborted 时填 Nav2 码 |
| event_detail | 事件附注(自由文本) | 默认空;reject 行填抑制原因词 |
| robot_x / robot_y | 本周期 getRobotPose() 结果(~277) | heartbeat 与事件行都带 |
| readiness_status | 见 §7 | 受控词表 |

## 2. stable_cluster_id(核心)

- **绝不使用 vector 下标**: 下标是扫描顺序的函数 —— 地图长大/连通域合并分裂/
  min_cluster_size 过滤都会使下标平移。n=5 配对分析拿下标当稳定 id 会把不同
  frontier 配成一对,且数据表无缺失/无异常/效应量可算 —— 静默假通过里最贵的一种。
- 定义: `qx = floor(rep_world.x / q)`, `qy = floor(rep_world.y / q)`;
  `stable_cluster_id = fnv1a64(qx, qy)` 取低 48 bit 十进制。
- q(量化步长) = 参数 `eventlog_stable_quant_m`,默认 **0.0 = 自动取地图分辨率**
  (latest_map_.info.resolution)。分辨率级量化等价于代表栅格 (row,col),映射单射
  → 无碰撞;显式 >0 时允许粗量化(碰撞风险由使用方承担),此时 q 进 config_hash。
- **护栏测试(实现前必写,唯一能在 B4 长跑前抓住此错的东西)**: 固定地图与机器人位姿,
  把 `detector_.detect()` 的返回向量**打乱后重算全部 id,断言逐簇不变**。
  便宜、且是唯一防线 —— 长跑完才发现 = 10 次全链 explore 全废。
- marker id(现为下标)不用于任何分析列;§2 与该视觉通道无关。

## 3. repo_sha 配套 repo_dirty / diff_sha

- 单独 repo_sha 在脏工作树时是假的: 同一 SHA 的树 + 未提交改动 = 不同程序。
- 启动时: `git rev-parse HEAD`; `git status --porcelain` 非空 → repo_dirty=1 且
  `git diff | sha256sum` 记 diff_sha(干净则空)。首行 header 或首条 CSV 行之前写定,
  一次采样,运行中不变(禁止每行重查 git)。

## 4. config_hash(已解析实参,不是文件)

- ROS 参数可被命令行 / launch / 覆盖文件改写;哈希 yaml 会漏。
- 取节点启动后 `get_parameters()` 全集: 参数名排序 → `name=value` 逐行 →
  整串 sha256 前 16 hex。量化步长(§2 显式值时)作为参数在列,天然进入。
- 计算一次于构造完成时;进程内不变。

## 5. map_digest(每行带当周期 digest,不是一次 run 一个)

- explore 期间地图持续变化;一次 run 一个 digest 无法回答"这一行是在哪张地图上
  产生的"。**每个事件行(含 heartbeat 与事件行)都带该事件所在检测周期的 digest**。
- 实现: 每个 IDLE 检测周期拿到 `latest_map_` 快照后算一次 digest,缓存为
  "当前周期 digest";该周期产生的 heartbeat 行与后续事件行共用此值,直到下一周期
  刷新。语义: "t 时刻生效的地图快照摘要" —— 一个行的 digest = 在 t 或 t 之前最近的
  检测周期的地图摘要(事件恒发生在某检测周期之后、下一周期之前)。
- digest 算法: 对 data 按 stride=8 粗采样(行与列均 stride 8),取每个采样格
  occupancy 值(0/100/-1)组串 → sha256 前 16 hex。stride 计入 schema_version 备注,
  不与 config 挂钩(纯摘要口径,改动只影响可比性标注,不改变任何行为)。
- map_digest 列与 map 时刻的解耦不再需要单独 map_digest_t —— 行自带 t。

## 6. action 受控词表(与 4060 doc `{accept,reject,preempt}` 的关系)

- doc 原词表三值不够覆盖本节点的现实事件;扩展(加粗为新增)。每行恰一 action:

| action | 触发点(代码) | 语义 |
|---|---|---|
| **heartbeat** | 每个 IDLE 检测周期(~285 后) | 前沿计数 + 机器人位姿 + readiness;无 goal 列 |
| accept | 发 frontier goal(~655 async_send_goal) | 选定并派发;nav2_result 随后另列 |
| **reject** | candidate 被抑制: blacklist(~298)/too-close(~468,570)/entrance-hysteresis(~526)/all-suppressed(~309) | 该簇本轮未被选;goal_id 留空或记候选序号 |
| **probe** | recovery probe 派发(~402) | Stage 3D 探针目标(非 frontier);goal_id 留空 |
| **timeout** | NAVIGATING 超时取消(~696-722) | goal 未达时限被取消;nav2_result=timed_out |
| **abort** | resultCallback ABORTED(~822) | Nav2 失败 |
| **preempt** | 取消进行中 goal 以派发新目标(当前代码无此路径;若 B4/B5 引入则在此挂) | 显式抢占;**先记为"无触发点"列,实现时空转** |

- nav2_result 受控词表: `accepted / rejected_by_nav2 / succeeded / aborted /
  canceled / timed_out / (heartbeat 行为空)`;error_code 不单独成列(可加列,待裁)。
## 7. readiness_status(需新增定义,现无信号)

- 词表: `ready` | `waiting_map` | `waiting_nav2` | `navigating` | `cooldown` |
  `completed`。映射 node 状态机(transitionTo/stateName ~1261-1288): 状态即 readiness。
- 建图未收敛/定位丢失等场景当前节点无感知 → 不发明信号;状态机枚举即全部真实状态。

## 8. 文件与周期语义

- 输出路径参数 `eventlog_csv`(默认 `/tmp/exploration_events_<seed>.csv`)。
- heartbeat 每检测周期 1 行(period 默认 1 s);事件行即时追加。CSV 头为首行。
- 5 种子一次性导出: 由运行侧 launch/脚本每种子单独进程导出,不做进程内多 seed。

## 9. 拍板记录(2026-09-08: 按下列推荐值实现,分析前可覆写)

1. action 扩展词表(`heartbeat/accept/reject/probe/timeout/result`)采纳;
   `preempt` 当前代码无触发点 —— 词表预留,导出器不产生该值(B4/B5 引入抢占时挂上)。
2. nav2_result 词表采纳;Nav2 error_code 独立成列 `nav2_error_code`(-1=无)。
3. quant 默认 **0.0 = 自动取地图分辨率**(单射、无碰撞);显式 >0 进 config_hash。
4. map digest stride=8 采纳;**每行带当周期 digest**(§5 改定)。
5. goal_x/goal_y 取**派发值**(投影后安全点 / 探针点)——与 Nav2 实际收敛目标一致;
   分析 revisit 需要地理 rep 时,可再增列 raw_goal_x/raw_goal_y(暂不加,防列膨胀)。
6. reject 行 = 该周期存在 frontier 但未派发 goal(全部被抑制)时输出 1 行,
   原因词入 event_detail(no_frontier / all_blacklisted / all_too_close /
   entrance_hysteresis / loop_no_probe);不逐候选输出(防 CSV 洪泛,诊断需要时再细分)。
7. seed/输出路径/量化步长为节点参数(eventlog_csv / eventlog_seed /
   eventlog_stable_quant_m);eventlog_csv 空 = 导出禁用(默认关,不影响既有行为)。
