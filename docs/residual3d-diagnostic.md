# RGB-D 三维静态补偿残差（第 4 周诊断阶段）

更新：三维模块现在默认使用 Tracking 位姿来源，也可显式选择 background_pnp 保留下面的原背景 PnP 流程。来源配置、后置执行时机及新验收见 [位姿来源说明](depth-pose-sources.md)。下文深度、残差和原 PnP 实验参数保持适用；“Track 前执行”的描述仅适用于 background_pnp 路径。

本模块只保存三维几何诊断；不删除特征，不改变位姿优化、地图维护、描述子、深度、地图点关联或优化外点标志，不实现 HMM，也不融合出最终动态结论。高残差可能来自运动、遮挡、错误匹配、深度边缘和相机运动误差。

## 真实深度和坐标处理链

- `Examples/RGB-D/rgbd_tum.cc` 用 `IMREAD_UNCHANGED` 加载深度。本地 TUM PNG 为单通道 uint16，**不是毫米**：配置 `RGBD.DepthMapFactor: 5000.0`，原始值 / 5000 得到米。其他设备输入类型/单位取决于数据源，不能按 uint16 推断单位。
- File.version=1.0 的 `Settings::readRGBD` 读取 `RGBD.DepthMapFactor`；旧配置路径 `Tracking::ParseCamParamFile` 读取 `DepthMapFactor`。两者实际含义都是“每米的原始深度数值”；`Tracking` 将它取倒数保存到 `mDepthMapFactor`（旧行为近零值当作 1）。`GrabImageRGBD` 的 `convertTo(CV_32F, mDepthMapFactor)` 只换算一次。浮点深度也按配置乘缩放系数；若输入已经是米，因子应为 1。
- `System::TrackRGBD` 在 GrabImageRGBD 前做既有图像缩放；Settings 按水平比例调整 fx/cx，按垂直比例调整 fy/cy。旧配置路径 Camera.imageScale 调整针孔内参，示例入口同步缩放 RGB 和深度。诊断使用最终 `Frame::mK`，不再次缩放内参。
- 当前工程要求输入已经注册到 RGB 相机像素网格的轴向深度，**没有在线 RGB/depth 外参配准**。同尺寸只检查尺寸，无法证明已对齐；本地 TUM 数据按既有配准输入使用。未对齐的数据不能用于本模块。
- 既有 resize 对深度使用 OpenCV 默认双线性插值，边缘可能出现混合值。为遵循本次“不修改深度”要求，本模块不更改原处理链，也不平滑/填补任何值；本版局部检查可以拒绝部分边缘，不能保证发现所有缩放混合深度。缩放数据的边缘残差仍应谨慎解释。
- `Frame::ComputeStereoFromRGBD` 在 `mvKeys[i]` **原始畸变像素**处以 `int(u),int(v)` 截断查询已转换深度（与 Frame 原代码 float 到 int 的隐式转换一致），`d>0` 写入 `mvDepth[i]`，否则 -1。此原检查不足以排除 Inf、范围外值等，诊断另外严格检查。
- 三维模块复用 `mvDepth` 的米制值；同一个最终米制深度图、同一原始像素用于局部一致性检查。反投影用对应 `mvKeysUn[i]` 去畸变像素、对应 K 的逆矩阵，以 double 计算。PnP 使用当前 `mvKeysUn`、当前 K 和空畸变（零畸变），不再次去畸变。第一版只支持 pinhole。
- 任一端零、负、NaN、Inf、范围外、像素越界或局部窗口越界均无效。局部半径内任一点无效也保守 UNKNOWN；任一点与中心差值大于 `DepthJumpM + DepthJumpRatio*z` 返回 DEPTH_DISCONTINUITY。默认半径 1（3×3），半径 0 仅检查中心；没有最近邻补洞。残差无效为 NaN，未匹配索引为 -1。

## 配置和前提

在现有相机 YAML 配置，无需新建实验配置：

```yaml
Dynamic.Enable: 1                 # 在线分割 + 实例关联，现有语义屏蔽关闭
Dynamic.Use3DResidual: 0           # 严格整数：0 启用、1 关闭、缺失关闭
Dynamic.UseEpipolar: 1            # 可独立关闭对极；0 同时开启
Residual3D.Debug: 1
Residual3D.Visualize: 1
Residual3D.SaveEvery: 1
Residual3D.OutputDirectory: "outputs/residual3d"
```

打印原始开关（缺失打印 MISSING）、实际值和 enabled=(value==0)。其他值/类型抛错；关闭不创建检测器及输出。两模块关闭时不执行公共诊断匹配。没有改变既有 SemanticMask.Mode=selected / Classes=[0:person] 策略。启动检查在线分割、实例关联、语义屏蔽状态，运行时检查实际标签、尺寸及 pinhole 条件；缺前提明确报错。

以下均为 **待验证的实验初值，不是最优参数**；可选参数仅在启用时读取和验证。

| Residual3D.* 参数 | 默认 | 含义 |
|---|---:|---|
| MinDepthM / MaxDepthM | 0.2 / 8 | 有效深度闭区间，米 |
| DepthRadius | 1 | 局部窗口半径，整数 0–20 |
| DepthJumpM / DepthJumpRatio | 0.05 / 0.02 | 深度跳变绝对和相对容差 |
| MatchRatio | 0.75 | 双向 Hamming ratio + 互为最近邻 |
| MaxDt | 0.2 | 两帧时间间隔上限，秒 |
| MinBackground | 30 | 背景 PnP 最少候选（至少 6） |
| RansacPx | 2 | 去畸变像素重投影门限 |
| Confidence / Iterations | 0.99 / 200 | PnP RANSAC confidence / 迭代数 |
| MinInliers / MinInlierRatio | 20 / 0.5 | 背景最少内点和相对候选的支持率 |
| MinCoverage | 0.15 | 两帧背景内点 4×4 网格覆盖率的较小值 |
| MinShapeRatio | 0.001 | 中心化背景三维坐标最小/最大奇异值比下界 |
| MaxReprojectionRms | 2 | 细化背景内点最大重投影 RMS，像素 |
| LowM / HighM | 0.03 / 0.08 | 三维残差低/高阈值，米 |
| Debug / Visualize | 0 / 0 | CSV/模型审计和独立 PNG |
| SaveEvery | 1 | 保存频率，不降低逐帧诊断频率 |
| OutputDirectory | outputs/residual3d | 输出目录 |

两模块启用时共享同一次匹配，两个 MatchRatio 须相等，不相等明确报错；各模块独立执行自己的 MaxDt 检查。匹配不使用对极筛选，也不使用跨帧实例 ID 相等条件。

## 背景运动、残差和局限

只选双端 instance_id=0、前帧深度有效、坐标有限的匹配构造前帧三维点与当前二维点。OpenCV `solvePnPRansac(...SOLVEPNP_EPNP)` 求 **P_current_predicted = R*P_previous+t**，再仅用背景 RANSAC 内点进行 `solvePnP(...SOLVEPNP_ITERATIVE, useExtrinsicGuess=true)` 细化。三维输入和 t 都是米。未读取 SLAM 位姿、优化结果、F 或旧变换；不把实例测试点加入拟合。

检查估计返回值、R/t 有限性、旋转行列式、内点数量/比例、两帧网格覆盖、三维背景形状及细化后重投影 RMS。所有拟合内点须满足正预测深度、有限重投影和 RansacPx 单点上限。失败、异常、支持不足及退化风险全 UNKNOWN。

形状筛查保守拒绝近共线/共面背景；共面背景 PnP 可能仍有解，本版宁可返回 UNKNOWN。网格覆盖、SVD 和重投影是第一版风险筛查，不能证明完整可观测性、排除所有运动背景污染或错误局部极小；未做协方差估计、完整条件数或遮挡建模。RANSAC 内点重投影仅为拟合质量，**不能视为独立精度验证**。

对全部公共匹配（包括对极一致、对极异常及 PnP 外点）计算：

```
P_previous = z_previous * inverse(K_previous) * [u_previous_un,v_previous_un,1]
P_current  = z_current  * inverse(K_current)  * [u_current_un,v_current_un,1]
vector = P_current - (R*P_previous+t)
residual_3d_m = norm(vector)
```

两端都有效、相对运动可靠、预测点正深度、结果有限后，r<=LowM 为 GEOMETRY_CONSISTENT；r>=HighM 为 SUSPECTED_DYNAMIC（疑似动态或几何异常）；中间为 UNKNOWN，保留有限原始残差。无效关联即使有有限残差也保持 UNKNOWN。深度无效、无匹配或运动不可靠的残差为 NaN。固定米制阈值受距离、深度噪声和标定误差影响，本实验不为使人物全红调阈值。

## 接入、所有权和不变量

`Tracking::GrabImageRGBD` 在特征、Frame 深度、实例标签就绪后、`Track()` 前调用。公共 `DiagnosticMatcher` 持有前帧描述子/K/畸变的深拷贝和特征/标签/深度检查结果的值拷贝；每次返回的 DiagnosticPair 保存该次前帧的所有权，替换缓存不会使结果悬空。两模块读取同一匹配及原始索引；三维结果放 `mResidual3DResult`，对极结果仍为 `mEpipolarResult`。

首帧、时间倒退/重复、过大时间间隔、帧号倒退及图像尺寸/标定变化返回 UNKNOWN 后以当前帧重建缓存。Reset、ResetActiveMap、CreateMapInAtlas、NewDataset、ChangeCalibration 一并清空公共匹配缓存、两模块结果和保存序号。运行时不改变 Frame 内部数据，仅创建诊断输入副本；没有实际剔除、优化外点写入或地图点操作。

## 输出和运行

`features.csv` 包含 frame_id、feature_id、previous_feature_id、instance_id、epipolar_residual_px、residual_3d_m、两端深度、depth_valid、epipolar_state、depth_state、reason，并附双端有效性/原因、三维残差分量、pnp_background_inlier 和“对极一致但三维异常”标志。对极关闭时对极字段留空；没有有效对极残差时残差字段也留空。无效三维残差写 nan。

`frames.csv` 保存候选/内点/内点率、运动状态/原因、覆盖/形状/RMS、当前帧有效深度比例、背景/实例残差中位数和 P90、拟合背景与非拟合背景单列统计、公共匹配/PnP/残差耗时。`motion_<frame>.json` 保存实际 R/t、K 和方向/米单位。只输出 raw 残差，不做融合结论。

`residual3d_<frame>.png` 为独立三维视图，绿=三维几何一致、红=疑似异常、黄=未知。标题 `3D diagnostic; features NOT removed` 表示“尚未剔除特征”。方形=PnP 拟合背景，圆形=其他测试点（实例点圆更大），紫色外圈=对极一致但三维异常。CSV 同时保留两个独立状态供其他组合对照。

```bash
cmake -S . -B build-semantic -DBUILD_RESIDUAL3D_TESTS=ON -DBUILD_EPIPOLAR_TESTS=ON
cmake --build build-semantic --target residual3d_test epipolar_motion_test rgbd_tum -j1
./build-semantic/residual3d_test
./build-semantic/epipolar_motion_test
python3 tools/validate_residual3d.py 40
# 直接运行前先在已有 YAML 打开 Debug/Visualize，关闭 GUI 可设置 Run.Viewer: 0
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt config/TUM3-instance-association.yaml /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/associate.txt outputs/residual3d-run 40
```

新构建需按现有分割文档提供 ONNX Runtime 设置；本地 build-semantic 已配置。验收脚本只在 /tmp 自动创建并清理临时配置，不创建专门的实验 YAML；拒绝覆盖已有输出证据。实测及未验证范围见 `docs/validation/residual3d-results.md`。

修改清单：新增 Residual3D.h/.cc、合成测试、真实验收脚本和本文；修改 EpipolarMotionDetector 提取公共匹配并接收共享结果，SemanticMasker 解析独立开关，Tracking 接入和重置生命周期，CMake 注册源文件和测试，现有实例关联 YAML 增加三维开关（selected/person 保持原样）。
