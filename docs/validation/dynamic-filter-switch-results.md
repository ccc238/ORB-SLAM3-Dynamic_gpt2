# 动态过滤总开关简化：验证结果

2026-11-13。当前仅保留 `Dynamic.UseDynamicFilter` 总开关：整数 **0 直接开启真实过滤**；整数 **1 或缺失关闭**；浮点、字符串、空字符串、容器及其他整数明确报错。分割、关联、对极、三维、LK 各自开关和默认配置保持原状。

`Dynamic.FilterMode` 已废弃。只检测旧键是否存在并打印 warning，不读取其值、不参与决策。残留 simulate/apply、非法字符串、数字或容器均警告并忽略；总开关决定是否真实过滤。旧验证报告保留为历史记录，不能再按旧模式配置运行。

## 实测结果

使用当前工程现有 rgbd_tum 和当前 TUM3 配置，TUM walking_xyz 前 120 帧。结果目录为 [outputs/dynamic-filter-switch-validation](../../outputs/dynamic-filter-switch-validation/)。每组独立进程，输出保留单独目录；缺省关闭另测 5 帧。

| 场景 | 帧 | 候选 | 实际排除优化观测 | applied 帧 | 安全拒绝 | 优化回退 | 跟踪失败 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 显式关闭（1） | 120 | 0 | 0 | 0 | 0 | 0 | 0 |
| 开启（0，无旧键） | 120 | 8 | 2 | 2 | 0 | 0 | 0 |
| 开启，强制最终支持不足 | 120 | 9 | 0 | 0 | 0 | 3 | 0 |
| 开启，强制剩余支持不足 | 120 | 9 | 0 | 0 | 2 | 0 | 0 |
| 开启，残留旧 simulate 键 | 120 | 8 | 3 | 3 | 0 | 0 | 0 |
| 总开关缺失 | 5 | 0 | 0 | 0 | 0 | 0 | 0 |

强制拒绝/回退只在临时测试配置中将 FilterMinRemaining / FilterMinFinalInliers 分别提高到 2000，项目默认安全阈值不变。on 与 legacy 独立建图，不要求逐帧计数相等；弃用旧项的验证依据是配置代码、明确警告以及实际接受过滤，而不是跨进程计数相等。

开启组：114 帧 NO_CANDIDATES，3 帧 NO_OPTIMIZABLE_CANDIDATES；均不执行额外优化。第 10 帧候选 1、边 352→351、实际排除 1、最终优化内点 238；第 106 帧候选 2、边 431→430、实际排除 1、最终优化内点 186。无建边资格的另一个候选保留黄圈，不能计作减少的优化约束。

代表图（现有 FrameDrawer 同一绘制路径保存，本轮为无 GUI 验证，未重新抓取 X11 窗口）：

- [开启接受过滤，2 候选但实际排除 1](../../outputs/dynamic-filter-switch-validation/runs/on/view/frame_106_1341846317.306289.png)
- [回退未应用，实际排除 0](../../outputs/dynamic-filter-switch-validation/runs/rollback/view/frame_10_1341846313.957927.png)
- [安全拒绝，实际排除 0](../../outputs/dynamic-filter-switch-validation/runs/safety/view/frame_63_1341846315.766041.png)

界面不再显示模式，保留 enabled、applied、候选、实际排除、优化观测前后、最终内点、跳过/回退原因。非候选灰点不代表确认静态，黄圈为候选，红叉只表示实际退出最终优化的观测。初始化无优化结果为 N/A。

## 编译和检查

全部受影响目标使用 `-j1` 单线程编译。最终日志：[build-final-j1.log](../../outputs/dynamic-filter-switch-validation/build-final-j1.log)；依赖重建日志为 build-continue-j1.log。首次空配置预检暴露 OpenCV 空根节点 keys() 断言，修复根节点类型检查后，最终单元与全部回归通过；原预检日志保留。

- 总开关缺省、0/1、非法值/类型；旧 FilterMode 各种值均警告且不改变总开关。依赖不兼容检查继续通过。
- 关闭对象调用保持位姿、地图关联、外点、优化索引、优化测量、描述子、深度、关键点和网格不变；正常 Tracking 关闭时根本不创建过滤对象。关闭实测仍有原分割/关联及 LK 诊断记录，未连带关闭这些模块。
- 真实混合单目/等效双目优化 96→92 边，4 个实际排除索引均不在最终边中。5 候选中有 1 个无地图点时，95→91 边，实际排除 4；未压缩原特征数组。
- 最终支持不足完整恢复初步位姿、全部外点、边索引、优化测量和计数，排除标志全部清空；安全拒绝不进入额外优化。KeyFrame / MapPoint 观测入口及新点保护测试继续通过。
- 无候选、无可优化候选、最大比例、剩余支持和空间覆盖拒绝，及不支持的跟踪路径均覆盖。原分割、对极、三维和 LK 回归全部通过，日志在 regression/。
- 605 帧新 CSV 核对：before/after 使用同一份资格快照，after = before − 实际排除 = 真实 final_edge；最终内点单列并属于最终边；事务 CSV 与显示 CSV 的 enabled/applied/actual_excluded 相同，RunFrames 的 enabled 和时间戳一致。无 filter_mode 列。见 [最终审计](../../outputs/dynamic-filter-switch-validation/final-audit.log)。
- 额外 28 帧 sitting_static 生命周期：样本 10/11 输入黑帧，仍为原跟踪失败状态；16 重置地图，22 切换数据集。记录与输入时间戳一致，不复用旧帧优化结果。见 [生命周期结果](../../outputs/dynamic-filter-switch-validation/lifecycle/summary.json)。

源码验证同时比对了改动前后 ConservativeCandidates 函数，以及原 apply 额外优化、安全接受和完整回退事务主体，逐字节相同。见 [校验结果](../../outputs/dynamic-filter-switch-validation/unchanged-algorithm.json)。残差阈值、安全默认值、逐特征规则、关键帧/建点守卫和历史地图点处理策略未修改。

## 修改位置与执行顺序

- include/DynamicFilter.h、src/DynamicFilter.cc：删除模式配置/读取、simulate 专用拷贝和不变性审计分支；总开关开启后直接进入原真实过滤流程；弃用警告，更新事务日志。
- include/DynamicFilterView.h、src/DynamicFilterView.cc、src/DynamicFilterDrawer.cc：删除快照模式字段和模拟显示；帧 CSV 使用 enabled/applied，候选和实际优化排除仍分开。
- include/Tracking.h、src/Tracking.cc：去掉模式缓存和参数传递，原提交钩子位置不变。
- Examples/RGB-D/rgbd_tum.cc：运行元数据只记录 enabled，不读取或输出模式。
- tests/dynamic_filter_test.cc、tools/validate_dynamic_filter*.py、tools/inspect_dynamic_filter.py：关闭不变性、直接真实过滤、弃用兼容及新日志字段验证；验证场景名称不是应用运行模式。
- 运行和显示说明同步更新；此前 simulate/apply 的实测文档明确标为历史语义。

顺序保持：初步跟踪及 TrackLocalMap 优化 → 地图提交前诊断证据/固定候选 → 安全检查 → 至多一次排除观测的额外优化 → 接受或完整回退 → 最终运动模型、关键帧/新点及位姿/轨迹缓存提交。原跟踪失败保持原处理，不会因过滤跳过而伪装成功。

## 新配置与复现

已有分割、关联及几何模块满足依赖时，只设置：

```yaml
Dynamic.UseDynamicFilter: 0  # 真实过滤；改为 1 或删掉该项即关闭
```

删除旧 Dynamic.FilterMode。可选输出目录和原安全参数继续有效，每次运行指定新的独立目录。依赖完整示例见 [集成说明](../conservative-dynamic-filter.md)，界面字段见 [显示说明](../dynamic-filter-view.md)。原默认 YAML 本次未增加实验配置。

```bash
cmake --build build-semantic --target dynamic_filter_test rgbd_tum semantic_mask_test epipolar_motion_test residual3d_test lk_check_test depth_pose_lifecycle_test -j1
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt /tmp/your-settings.yaml \
  /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz \
  /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/associate.txt \
  outputs/your-run 120
python3 tools/validate_dynamic_filter.py --case all --frames 120 --output outputs/your-new-switch-test
python3 tools/validate_dynamic_filter.py --case missing --frames 5 --output outputs/your-new-switch-test
```

没有调整阈值、增加候选数量的规则改动、实例整体传播、HMM 或历史地图点清理。本轮验证配置控制、真实排除、回退和统计的一致性，不报告动态识别准确率或新的轨迹精度结论。
