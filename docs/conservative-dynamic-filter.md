# 第 4 周第 6–7 天：保守动态候选过滤与 SLAM 集成

## 配置及范围

所有模块开关和参数都由 YAML 配置，不需要在命令行传开关。示例 [TUM3-instance-association.yaml](../config/TUM3-instance-association.yaml) 集中配置语义分割、实例关联、对极匹配、RGB-D 三维残差、LK 交叉验证、人体区域补充特征和动态过滤。动态过滤开关显式默认为关闭；其他模块仍按各自 YAML 开关独立启用。验证脚本会在 `/tmp` 创建临时 YAML，完整记录在结果目录的 `effective_settings.txt`。

实验需要已有参数为：

```yaml
Dynamic.Enable: 1                     # 在线分割，1 启用
Dynamic.UseEpipolar: 0                # 0 启用
Dynamic.Use3DResidual: 0              # 0 启用
Dynamic.DepthPoseSource: "tracking"
Residual3D.UseDepthDifferenceGate: 1   # 这里是 1 启用
Dynamic.UseLKCheck: 0                 # 0 启用
```

真实 `SemanticConfig::Read` 在当前工程将 `maskingEnabled` 解析为 false、实例关联与在线分割一起启用。`SemanticMask.Mode: selected` 的存在不代表当前确实进行 person 屏蔽；过滤兼容检查针对实际执行标志，没有修改任何已有开关。

只保留一个总开关：

```yaml
Dynamic.UseDynamicFilter: 1  # 0 开启真实过滤；1 或缺失：关闭
```

严格只接受整数 0/1，其他类型或值明确报错。启用后直接生成候选、检查安全条件并尝试一次排除观测的最终位姿优化，接受或完整回退。`Dynamic.FilterMode` 已废弃：旧配置残留时打印明确警告，不读取其值、不影响总开关，即使旧值为 simulate，开关为 0 也执行真实过滤。请删除旧项以免误解。

启动打印原始开关及 enabled。启用仍需要全部诊断依赖、Tracking 位姿来源、深度差筛选以及语义屏蔽关闭，非纯 RGB-D 拒绝。初始化、重定位、失败/恢复首帧及 localization-only 路径按原条件跳过。关闭时不创建过滤器，不改变原 SLAM 决策；分割、关联及各诊断仍由各自的原开关控制。

可选安全参数（默认值）：

```yaml
Dynamic.FilterMaxRatio: 0.20
Dynamic.FilterMinRemaining: 50
Dynamic.FilterMinFinalInliers: 30
Dynamic.FilterMinCells: 6
Dynamic.FilterOutputDirectory: "outputs/dynamic-filter"
```

最大比例同时检查拟排除优化边 / 原边数和拟排除原内点 / 原有效地图内点。剩余数检查实际具有地图观测的原优化内点，非总 ORB 数；空间覆盖检查这些剩余内点占据 4×3 图像格子的数量。额外优化之后再次检查最终内点覆盖。50 留出原 RGB-D 正常跟踪 30 内点判据的余量；最终内点不得少于原 RGB-D 判据 30，原重定位保护期仍须 50。配置可提高门限，不能把最终门限降低到 30 以下。

准备数据集时，切换过滤只需编辑 YAML 中的 `Dynamic.UseDynamicFilter`：`1` 表示关闭、`0` 表示开启真实过滤。运行命令仍需提供词典、YAML、数据集目录和 TUM `associate.txt`；无需再给终端额外传模块开关：

```bash
./Examples/RGB-D/rgbd_tum \
  Vocabulary/ORBvoc.txt \
  config/TUM3-instance-association.yaml \
  /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz \
  /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/associate.txt \
  outputs/your-unique-run \
  430
```

命令最后的 `430` 是可选的最大帧数，移除后处理整个关联文件。`Run.Viewer: 0` 可用于无显示运行。每次重复运行前，建议复制 YAML 并在副本中改用新的过滤、显示和诊断输出目录，避免覆盖结果；模块开关仍只需在所用 YAML 中维护。

## 调用链与提交位置

原调用链是：`GrabImageRGBD` 构造 Frame、分割/关联、公共诊断匹配、对极、LK → `Track` → 初始匹配及优化 → `TrackLocalMap` 的 `PoseOptimization` → 跟踪判据 → 运动模型更新 → `NeedNewKeyFrame/CreateNewKeyFrame` → 清理帧内关联 → `CaptureDepthTrackingPose` → `mLastFrame` → 轨迹记录 → 返回 Grab 后才进行 Tracking 三维诊断。

开启过滤后的顺序：

1. Grab 内按原顺序产生本帧特征、实例标签、公共匹配、对极结果和 LK 结果；匹配对象和帧对快照在本次调用内独占。
2. `Track` 完成初步跟踪和 `TrackLocalMap` 优化、按原规则判定是否成功。
3. 在 `Track` 内、drawer/运动模型/关键帧之前调用本次 Grab 安装的 `mBeforeDynamicCommit`。
4. 在已有地图更新锁内捕获初步 Tracking 位姿，同时按已有逻辑重基准化上一帧位姿；生成本轮 Tracking 三维残差。
5. `DynamicFilter::Process` 固定本轮候选，记录原边、原外点、原内点；检查剩余支持和覆盖，最多执行一次额外 `PoseOptimization`。
6. 额外优化传播 g2o 初始化/求解失败返回值；失败或最终支持不合格时恢复初步位姿、完整外点数组、优化边索引、清空排除标志；跟踪计数在通过时才更新。原跟踪失败不进入额外过滤优化。
7. 使用确认后的最终结果更新 LocalMapping 的内点数、MapPoint found 统计、drawer、运动模型、关键帧决策和深度建点。
8. `CaptureDepthTrackingPose` 再次捕获最终位姿，复制到 `mLastFrame`，记录轨迹；Grab 返回前提交独立的上一帧诊断位姿缓存。用于本轮诊断的初步快照不被覆盖重算。

没有第二次调用 Track。只有开启过滤时，`TrackLocalMap` 的 MapPoint found 统计推迟到提交钩子，并使用最终内点；初步的局部地图搜索统计、可见次数及初步匹配仍保留。关键帧接近点支持统计会跳过已接受的排除位置。候选可能参与本帧初步匹配与初步优化，只保证不参与接受后的最终优化及新关键帧有效观测/点创建。

关闭时保留原来 Track 返回后的三维诊断时机；启用时诊断前移到提交前。独立运行的 BA 线程时序和轨迹可能不同，不能用跨进程轨迹差异代替关闭路径的状态不变性验证。

## 逐特征候选和优化边

`ConservativeCandidates` 输出与当前帧原始索引等长的数组。要求当前/上一帧匹配的标签均大于 0（无需相等）、公共匹配有效、LK SUPPORTED、可靠对极高残差和有效 Tracking 三维高残差、双端有效深度及深度差未拒绝。沿用已有诊断分类门限，不改变阈值。

检查各模块的当前/上一帧 ID、时间、匹配前帧索引；拒绝缺失、重复 LK 行、未知、冲突或旧缓存结果。类别不是规则输入。数组 `mvbDynamicCandidate` 与 `mvbDynamicExcluded` 独立于 `mvbOutlier`，不压缩或重排关键点、描述子、深度、网格或关联数组。

`Optimizer::PoseOptimization` 在任何单目/等效双目边构造之前检查 `DynamicExcluded(i)`。因此被跳过的位置不会被优化器内部的外点重分类恢复为优化内点。每次优化记录 `mvPoseOptimizationIndices`，过滤模块验证它与接受的排除集合无交集。

候选全部数、其中可构边数、原外点数和原内点数分别统计；全部候选数不是优化约束减少量。无候选或无可优化候选时不重复优化，本帧不接受过滤。

## 地图接口

- `Frame` 的复制保留索引、候选/排除标志及优化索引审计数据。
- `KeyFrame(Frame&)` 复制排除标志，只在新关键帧的有效地图点表清空对应位置；不先破坏当前帧诊断关联。
- `Tracking::CreateNewKeyFrame` 在排序深度之前跳过排除位置，不能由这些深度创建新点。
- `LocalMapping::CreateNewMapPoints` 对两端排除位置都跳过。
- `KeyFrame::GetFeaturesInArea` 不返回排除位置，避免后续投影融合回填；`KeyFrame::AddMapPoint` 和 `MapPoint::AddObservation` 再次守卫入口。
- 关键帧标志有 Boost 序列化版本 1；加载旧版本默认为空（无排除），新版保存/加载保留此字段。
- 不调用 MapPoint::SetBadFlag，不删除旧观测，不推断地图点永久动态身份。

## 输出与验证

`rgbd_tum` 的 `RunFrames.csv` 逐帧记录 enabled。完整显示统计在独立的 `Dynamic.FilterViewOutputDirectory/frames.csv`：enabled、applied、ORB 总数、候选、优化观测前后数量、实际排除、最终优化内点、执行/跳过/回退原因，无优化结果为 N/A。

`Dynamic.FilterOutputDirectory/frames.csv` 保存原事务审计并去除模式字段：enabled、applied、actual_excluded、候选/可优化/原外点/原内点数、初步/最终边与地图支持数、安全状态、原因、位姿变化及耗时。实际排除只计原本可以构边且已接受排除的观测；不能将全部候选数作为实际排除数。回退的最终数来自恢复后的初步结果。

`features.csv` 每帧每个原始索引一行，保存实例、匹配前帧索引、LK/对极/三维证据、候选/实际排除/原因、最终优化边参与、是否允许新关键帧观测与深度建点。对应帧对和原像素/深度详细值也能从同次原诊断 CSV 交叉核对。这里 effective_excluded 保留地图提交许可掩码的含义，可含本来没有优化边的位置；实际退出优化的观测及其数量以显示目录中的 observations.csv 和帧级 actual_excluded 为准。

`poses.csv` 保存 preliminary/final Tcw。RGB 主窗口中灰点为非候选（不是确认静态），黄圈为疑似动态，红叉只代表本帧接受过滤且退出最终优化的观测。安全拒绝或回退明确显示未应用/已回退和实际排除=0。无候选、无可优化候选不做额外优化。保留带锁的同帧快照、原始坐标和逐索引审计，不压缩特征数组。

`compute_ms` 是本模块候选、安全检查和可能的额外优化；`record_ms` 为逐特征审计与绘图；`total_ms` 为两者之和，不包含已有分割、LK 和三维诊断本身。全 Track 时间从 `RunFrames.csv` 比较，不把模块计算时间当作端到端耗时。

验证命令：

```bash
./build-semantic/dynamic_filter_test
python3 tools/validate_dynamic_filter.py --case all --frames 120 --output outputs/your-switch-validation
python3 tools/validate_dynamic_filter.py --case analyze --output outputs/your-switch-validation
```

脚本拒绝覆盖已有运行目录；默认独立输出到 `outputs/dynamic-filter-switch-validation/runs/`。单元测试在真实单目和等效双目边上验证额外优化/回退；运行脚本逐行验证实际排除不在最终边和地图许可集合中。ATE 采用固定尺度 SE(3) 对齐，真值最近时间差 ≤0.02 s；RPE 使用约 1 s 间隔（误差容许 0.05 s），报告平移和旋转 RMSE。不能把独立线程运行间的单次差异归因为过滤效果。

可从同次日志导出所有候选帧（包括 C++ 每 10 帧采样以外的候选）的原始 RGB 审计图与 4×3 空间分布：

```bash
python3 tools/inspect_dynamic_filter.py outputs/your-switch-validation/on /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz
```

该工具只读取同次 CSV 和原图，不重跑匹配、几何或 LK；当前限定本报告使用的 fresh-process、未缩放 640×480 TUM 运行，拒绝覆盖已有 review 输出。图顶黑色信息栏区分候选、实际排除及回退未应用，标号为原始特征索引。

结果见 [实测报告](validation/dynamic-filter-results.md)。这是保守动态候选，不是真实动态标注。交集规则会漏掉沿对极方向运动、几何不可观测和未知证据的点。没有逐点真值，不报告动态识别准确率。不实现 HMM、实例整体传播、稠密过滤或历史地图点清理。

当前单开关实现与测试结果见 [总开关简化验证](validation/dynamic-filter-switch-results.md)。旧验证报告中 simulate/apply 是历史版本行为，不适用于当前版本。
