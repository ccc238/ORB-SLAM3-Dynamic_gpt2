# 第 4 周第二步：背景对极几何诊断

后续独立三维诊断及公共匹配改造见 [RGB-D 三维静态补偿残差](residual3d-diagnostic.md)；下文保留本步骤的对极算法说明。

实现止于两帧独立匹配、背景 F、sqrt Sampson 残差和三态诊断。结果不传给 ORBmatcher、优化器或地图清理。几何一致不等于确认静态；疑似动态也可能来自错误匹配。本步骤不实现三维残差、实例跟踪或 HMM；后续三维模块独立接入，不改变对极分类。

## 实际配置入口与调用链

`SemanticConfig::Read` 使用现有相机 YAML 的 OpenCV FileStorage，调用 `EpipolarConfig::Read`，统一解析后 Tracking 才创建资源。

`Dynamic.UseEpipolar` 严格为整数，**0=启用，1=关闭**；缺失为 1。字符串、浮点数 0.0、负数、2 等均报错。日志打印原始值、ENABLED/DISABLED 以及“0=启用，1=关闭”。仅读取启动配置，不支持热切换；重新启动重新构造，不继承旧缓存。

现有 `Dynamic.Enable=1` 已经统一控制分割开启、关联开启、屏蔽关闭。`Dynamic.MaskOnly` 保留兼容语义，不控制本模块；旧 selected/person 选择策略和屏蔽实现保留。本次没有更改其解析优先级。几何启用而分割/关联关闭或语义屏蔽开启会明确抛错；实时还检查标签图、尺寸、关联标志。第一版仅支持 RGB-D/IMU RGB-D 的 pinhole 相机。

`System::TrackRGBD` 在进入 `Tracking::GrabImageRGBD` 前执行既有缩放；在本帧分割、Frame 特征提取及 `AssociateInstances` 后，`Track()` 前诊断，不使用当前帧位姿。读取 `mvKeysUn`、`mDescriptors`、`mvInstanceIds`，按原索引写入独立 `Tracking::mEpipolarResult`。不修改 Frame 特征、地图点、描述子或原优化外点标志，也不清除任何实例数据。

已有 `config/TUM3-instance-association.yaml` 增加该开关（值为 0）及独立输出配置，没有新增实验 YAML。其他旧配置缺失该键，默认关闭。手动在现有配置中设置：

```yaml
Run.Viewer: 0  # Headless validation; use 1 for the existing viewer
Dynamic.Enable: 1
Dynamic.MaskOnly: 1
Dynamic.UseEpipolar: 0  # 0=启用，1=关闭；缺失默认 1
Epipolar.Debug: 1
Epipolar.Visualize: 1
Epipolar.SaveEvery: 1
Epipolar.DrawMatches: 1
Epipolar.OutputDirectory: "outputs/epipolar"
```

诊断每帧计算，SaveEvery 只影响 CSV、PNG 和专用调试日志。Debug 和 Visualize 独立控制 CSV 与图像；均关闭则不创建输出目录，也不保留前帧 RGB。总开关关闭时不构造检测器、不独立匹配、不估计模型、不计算残差、不输出诊断文件。启动配置状态日志仍正常打印。

| 配置 | 默认值 | 说明 |
|---|---:|---|
| Epipolar.MatchRatio | 0.75 | 双向各自通过的最近/次近邻比值 |
| Epipolar.MinBackground | 30 | 至少 8；实际使用双端标签为 0 的匹配 |
| Epipolar.RansacPx | 1.5 | FM_RANSAC 阈值，去畸变像素 |
| Epipolar.Confidence | 0.99 | 0 < confidence < 1 |
| Epipolar.MinInliers | 20 | 至少 8 |
| Epipolar.MinInlierRatio | 0.5 | 相对全部背景匹配 |
| Epipolar.MinCoverage | 0.15 | 两幅图背景内点的 4×4 网格占用率的较小值 |
| Epipolar.LowPx | 1.0 | r <= low 为 GEOMETRY_CONSISTENT |
| Epipolar.HighPx | 3.0 | r >= high 为 SUSPECTED_DYNAMIC |
| Epipolar.MaxDt | 0.2 | 连续输入两帧最大时间间隔，秒 |
| Epipolar.MinDisplacementPx | 0.5 | 背景 F 内点的原始像素位移中位数下界 |
| Epipolar.HomographyRatio | 0.9 | 背景单应性 RANSAC 支持率风险门限 |
| Epipolar.Debug | 0 | CSV + 保存帧控制台摘要 |
| Epipolar.Visualize | 0 | 保存原图坐标叠加及匹配图，无 GUI 依赖 |
| Epipolar.DrawMatches | 1 | 左前帧/右当前帧并连线；依赖 Visualize |
| Epipolar.SaveEvery | 1 | 正整数，从首帧开始按处理帧计数保存 |
| Epipolar.OutputDirectory | outputs/epipolar | 专用输出目录 |

分类要求 0 <= low < high；中间区间 UNKNOWN/AMBIGUOUS_RESIDUAL。分类阈值与 RANSAC 阈值完全独立。可选参数在启用时校验；关闭只严格校验主开关。

## 坐标、模型与退化风险

标签沿用原始当前 SLAM RGB 像素 `mvKeys` 的既有查询，不重新投影或查询去畸变坐标。匹配使用标准 ORB `CV_8UC1`、32 列描述子的 Hamming、双向 ratio 及互为最近邻，保证一对一；不使用实例 ID 跨帧相等关系，也不预先按待估对极线筛选运动候选。

F 和残差一律用 `mvKeysUn` 去畸变像素，方向 **x_current^T F x_previous = 0**。K 为实际缩放后的 Frame 内参；畸变、K、图像尺寸变动会切断旧帧比较。F 不处理归一化坐标。背景集合另存到全匹配集合的索引映射，背景 RANSAC mask 只经映射写回对应特征。

背景可能漏检，因此用当前 OpenCV 的 `findFundamentalMat(...FM_RANSAC...)`。检查 3×3 形状、有限性、非零范数、mask 长度、背景数、内点数、内点率及两帧空间覆盖。背景不足不退回全部特征；模型失效不复用前帧 F。

保守退化检查采用两道风险门：背景内点原始像素位移中位数过小则 LOW_DISPLACEMENT_RISK；背景单应性支持率 >= 0.9 则 HOMOGRAPHY_DOMINANCE_RISK。单应性用独立 RANSAC、与 F 相同像素阈值和 confidence。单应性检查本身失败也保守保持 UNKNOWN。

这些是风险筛查而非运动类型判定。原始像素位移**不是旋转补偿后的视差**，旋转可以产生很大位移；高单应性支持可能来自平面、近纯旋转、低平移/低视差或远景，不能单独断言纯旋转。检查可拒绝部分仍可用的 F，也无法保证检出所有退化；深度、惯性或更完整的可观测性判定不在本次范围。空间覆盖按两幅图的图像范围计算，畸变校正后在范围外的点不增加网格覆盖。

残差以 double 齐次像素和规范化 F 计算 `abs(q^T F p) / sqrt((Fp)_0²+(Fp)_1²+(F^Tq)_0²+(F^Tq)_1²)`，得到 sqrt Sampson 像素误差。分母接近零、坐标无效均为 NaN/UNKNOWN。模型可靠时计算全部有限匹配的残差（包括背景 RANSAC 外点）；无效实例关联仍 UNKNOWN。背景外点不会直接成为动态。没有匹配为 -1/NaN/UNKNOWN/NO_MATCH；首帧、模型失败等有明确原因码。

## 所有权和生命周期

“上一帧”是当前序列上一次提交给检测器的 RGB-D 输入帧，不是最后成功跟踪帧或关键帧。特征、标签为值拷贝，描述子、K、畸变及需要可视化时的 RGB 深拷贝。超过 MaxDt、时间倒退/重复、帧号倒退、标定/图像尺寸变化时当前帧全 UNKNOWN，再以当前输入建立新缓存。`Reset`、`ResetActiveMap`、`CreateMapInAtlas`、`NewDataset`、`ChangeCalibration` 同时清除诊断缓存及 Tracking 结果。Track 内触发新地图时也会清除刚产生的诊断状态。

没有共享或修改全局随机种子；合成测试还比较 `cv::theRNG().state` 前后相等。检测器没有读取地图点的接口。

## 输出及运行

`features.csv` 包含请求的十列，并附加 `background_ransac_inlier`（-1 非背景集合，0 外点，1 内点）。无残差写 nan。Debug CSV 另附两端原始/去畸变坐标，并按保存频率输出 `model_<frame_id>.json`，保存实际 F、K、畸变和模型状态，便于独立复算。

`frames.csv` 包含时间间隔、N、匹配/背景/内点数量、内点率、覆盖、退化风险统计、模型状态与原因、三态数量及匹配/模型估计（含退化检查）/残差毫秒耗时。

PNG 为 BGR 输出，绿色几何一致、红色疑似动态或几何异常、黄色未知；用原始 `mvKeys` 绘制。顶部标明 `Geometry diagnostic; features NOT removed`（几何诊断，尚未剔除特征），同时显示模型原因、背景数和内点数。可视化开关只保存文件，不开新窗口。

```bash
cmake -S . -B build-semantic -DBUILD_EPIPOLAR_TESTS=ON
cmake --build build-semantic --target epipolar_motion_test rgbd_tum -j1
./build-semantic/epipolar_motion_test
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt config/TUM3-instance-association.yaml /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/associate.txt outputs/epipolar-run 60
python3 tools/validate_epipolar.py 60
```

build-semantic 已配置 ONNX Runtime；新构建还需按原文档指定 WITH_ONNX_RUNTIME 及 ONNXRUNTIME_ROOT。验证脚本仅在 `/tmp` 创建自动清理的临时配置，不添加实验 YAML；原配置保持可审阅。脚本拒绝复用已有 debug 输出目录，避免旧输出混淆验收。

实测记录见 [验收结果](validation/epipolar-results.md)。


## 按帧专项审计

已导出调试模型及关联标签后，可执行 `python3 tools/inspect_epipolar_frame.py outputs/epipolar-inspection 20`。工具按本实验默认分类阈值 1/3 px 复核全部匹配，核对背景集合和原始标签图，导出 10 个人体绿色点的完整明细、真正的当前帧对极线、背景人体边缘候选和黄色原因分解。需要 numpy/Pillow；对极线原图叠加仅接受零畸变，非零畸变需使用去畸变画布。本次 [第 20 帧四项专项检查](validation/epipolar-frame20-inspection.md) 包含实际证据和局限。
