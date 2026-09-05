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
