
## 首次实跑结果(2026-09-06,branching world,8 分钟探索)

| 环节 | 结果 | 证据 |
|---|---|---|
| 探索+录制 | 8 min,1820 位姿,2.4MB JSON | ~/b6_demo/explorer_path.json |
| MPC live 跟踪(path_file 模式) | **位移 0.395m > 0.05m PASS**(15s 窗口) | ~/b6_demo/motion_result.txt |
| 离线审计(前 300 位姿,16.6m) | **e_y_rms 0.0013m**,e_y_max 0.0126m,smooth 4.6e-4 | replay_metrics_300.json |
| 全长回放 | 因 QP 在噪声转角处反复迭代上限,小时级耗时——已记录为回放器待优化项(向量化 closest_point / 降迭代上限) | — |

结论:B6 链路(探索→Path→MPC 跟踪)首次端到端打通,毫米级跟踪。
未完成项:全链覆盖率审计(需沿 Path 执行完整跟踪跑完 16.6m)。
