# 动态过滤显示与统计验证

2026-11-13。范围仅为显示和观测数量审计，没有改变候选交集、过滤决策或默认安全阈值。复用现有 rgbd_tum、FrameDrawer 和 `ORB-SLAM3: Current Frame` 窗口。

结果目录：[outputs/dynamic-filter-view-validation](../../outputs/dynamic-filter-view-validation/)。配置及统计定义见 [显示说明](../dynamic-filter-view.md)。所有构建均使用 `-j1`。

## 真实片段结果

使用 TUM freiburg3 walking_xyz 前 120 帧，每组独立进程，顺序为关闭 → simulate → apply → 强制回退 → 强制安全拒绝。后两组分别只将最终支持门限或最少剩余支持提高到 2000，用于覆盖对应分支，没有修改项目默认配置。

| 运行 | 帧数 | 候选总数 | 实际排除优化观测 | 接受帧 | 安全拒绝帧 | 优化回退帧 | 跟踪失败 |
|---|---:|---:|---:|---:|---:|---:|---:|
| off | 120 | 0 | 0 | 0 | 0 | 0 | 0 |
| simulate | 120 | 2 | 0 | 0 | 0 | 0 | 0 |
| apply | 120 | 2 | 2 | 2 | 0 | 0 | 0 |
| rollback | 120 | 2 | 0 | 0 | 0 | 1 | 0 |
| safety | 120 | 2 | 0 | 0 | 2 | 0 | 0 |

每组首帧 before/after 和最终优化内点都为 N/A；实际排除为已知的 0。关闭时不运行候选过滤器，候选数组保持 false，表中 0 不表示已经做过动态真值判定。

### 窗口、CSV 与实际索引的代表帧

| 同次运行 | 帧 | ORB 总数 | 候选 | 构边前→后 | 实际排除 | 最终优化内点 | 状态 |
|---|---:|---:|---:|---:|---:|---:|---|
| off | 10 | 1009 | 0 | 344→344 | 0 | 225 | FILTER_DISABLED |
| simulate | 10 | 1009 | 1 | 341→341 | 0 | 225 | SIMULATE_ONLY |
| apply | 10 | 1009 | 1 | 342→341 | 1 | 222 | ACCEPTED |
| apply | 106 | 1002 | 1 | 403→402 | 1 | 138 | ACCEPTED |
| rollback | 106 | 1002 | 1 | 455→455 | 0 | 167 | FINAL_SUPPORT_FAILED_ROLLBACK |

各组独立建图，不能把不同组的边数差当作过滤效果。每帧的 before/after 都来自同一份实际构边资格快照，变化仅来自该帧接受的过滤掩码。

以下均为通过 X11 抓取的真实 Current Frame 窗口，不是另写绘图脚本制造的演示图。窗口内容与同帧 FrameDrawer 保存图逐像素相同，对应关系记录在各组 `view/window-capture.json`。

- [关闭窗口](../../outputs/dynamic-filter-view-validation/off/view/window_off.png)
- [仅模拟、有黄圈且实际排除为 0](../../outputs/dynamic-filter-view-validation/simulate/view/window_simulate_candidates.png)
- [apply 接受、红叉且实际排除为 1](../../outputs/dynamic-filter-view-validation/apply/view/window_apply_accepted.png)
- [优化回退、只有黄圈且实际排除为 0](../../outputs/dynamic-filter-view-validation/rollback/view/window_rollback.png)
- [安全拒绝、未应用且实际排除为 0](../../outputs/dynamic-filter-view-validation/safety/view/window_safety.png)
- [初始化、无位姿优化结果显示 N/A](../../outputs/dynamic-filter-view-validation/off/view/window_initialization.png)

每个运行的 `view/frames.csv` 是新界面的帧级统计；`view/observations.csv` 给出每个原始索引的候选、资格、实际排除、真实最终边和最终内点。原有 `filter/` 审计日志及轨迹输出独立保留。

汇总见 [summary.json](../../outputs/dynamic-filter-view-validation/summary.json)，完整逐项核对见 [final-audit.log](../../outputs/dynamic-filter-view-validation/final-audit.log)。600 帧逐原始索引检查：after = before − effective_excluded = 实际 final_edge；排除与最终构边无交集；最终内点属于最终构边集合；CSV 数量与各集合基数相同；候选和最终边与旧过滤审计逐项相同；时间戳与原程序 RunFrames 一致。

首次回退测试的截图工具遇到 X11 24 位颜色头与 32 位行步长的兼容差异而中止，未计为完成验证。已修复截图解码并完整重跑 120 帧；第一次不完整输出保留在 `rollback-capture-incomplete/`，上述表格、汇总和最终审计只统计五个完成的运行目录。

## 单元和回归

- `dynamic_filter_test`：调用真实 PoseOptimization，覆盖单目与等效双目边。simulate 保持位姿、外点、描述子、深度、关键点、网格及关联不变；apply 的 96 条边变为 92 条；回退恢复原状态与本次新增的测量元数据。
- 混合资格夹具：96 个 ORB、5 个候选，其中 1 个没有地图点关联；95 条初步边变为 91 条，只显示 4 个红叉，无地图点的候选保持黄圈。见 [截图](../../outputs/dynamic-filter-view-validation/unit/mixed-nomap/current_frame.png) 及对应 frames.csv。
- 关闭、simulate、接受、回退、安全拒绝、初始化均调用实际 FrameDrawer 发布和 DrawFrame 路径；检查 RGB/BGR 转换、输入图像所有权、灰点/黄圈/红叉、显示缩放和原始坐标（刻意令 mvKeysUn 与 mvKeys 不同）。无测量显示 N/A。
- 200 帧生产者/显示读取者并发快照检查：图像像素编码的帧号、点位置、帧号和时间戳一致，没有跨帧拼接。
- 原分割、对极、三维和 LK 回归均通过。日志在 `regression/`，新增单元日志在 `unit.log`。编译日志为 `build.log`、`build-font-fix.log`、`build-regression.log`、`build-test-final.log`。
- 28 帧真实 sitting_static 生命周期检查：第 10/11 输入置黑，原跟踪状态仍为非 OK；第 16 重置活动地图，第 22 切换数据集。全部显示记录与输入时间戳及原始特征数量一致，未复用上一帧优化结果；样本 0、10、11、12、16、22 的最终内点为 N/A。见 [生命周期汇总](../../outputs/dynamic-filter-view-validation/lifecycle/summary.json)。

## 修改位置与调用链

| 文件 | 本次职责 |
|---|---|
| include/DynamicFilterView.h、src/DynamicFilterView.cc | 独立的优化测量元数据、显示快照、同帧戳校验、冻结资格及真实边集合核对 |
| include/Frame.h、src/Frame.cc | 保存/复制优化测量；不调整原特征索引或算法数组 |
| src/Optimizer.cc | 在原构边/分类路径上记录结果是否可测及最终优化内点索引；不修改优化参数或决策 |
| include/DynamicFilter.h、src/DynamicFilter.cc | 导出本轮显示快照；回退恢复测量元数据；旧调试图修正为实际优化排除红叉及对应数量 |
| include/Tracking.h、src/Tracking.cc | 读取显示输出设置，提交前捕获统计，Track 后将对应原 RGB 与值快照发布 |
| include/FrameDrawer.h、src/FrameDrawer.cc、src/DynamicFilterDrawer.cc | 新增完整快照锁、RGB 叠绘、中文图例、CSV 与 PNG；DrawFrame 复用现有窗口 |
| CMakeLists.txt、tests/dynamic_filter_test.cc、tools/validate_dynamic_filter_view.py | 编译接入、真实优化/显示验证、现有 rgbd_tum 驱动及窗口/CSV/实际索引核对 |

真实顺序：GrabImageRGBD 原始 RGB/实例关联与两帧证据 → Track 初步位姿/TrackLocalMap → Track 中 mBeforeDynamicCommit → 初步建边资格快照 → 原三维证据/候选/安全检查 → 原额外优化一次或回退 → 冻结显示统计 → 原运动模型、关键帧/新点、最终位姿与轨迹缓存 → GrabImageRGBD 将该 RGB 与快照整体发布 → 现有 Viewer 的 DrawFrame 读取完整快照。不会在帧末改变掩码来假装本帧重新优化。

## 复现与限制

```bash
cmake --build build-semantic --target dynamic_filter_test rgbd_tum -j1
python3 tools/validate_dynamic_filter_view.py --case all --frames 120 \
  --output outputs/my-new-filter-view-validation --capture-window
python3 tools/validate_dynamic_filter_view.py --case analyze \
  --output outputs/my-new-filter-view-validation
```

`--capture-window` 需要本机 X11 和 xwininfo/xwd，自动抓取的范围仅为本程序窗口。去掉该参数为无 GUI 验证，保存图仍走同一个 FrameDrawer 绘制函数。脚本拒绝覆盖已有运行。实际参数保存在各运行 effective_settings.txt，不新增实验 YAML。

本次检查显示与计数的真实性，不重新评估完整轨迹精度，也不报告动态识别准确率。没有逐点真值，少量候选与实际排除不能证明它们是真动态。字体依赖当前已安装的 OpenCV freetype、Droid 中文字体及 DejaVu 字体；其他机器需要提供这些字体或适配字体路径。
