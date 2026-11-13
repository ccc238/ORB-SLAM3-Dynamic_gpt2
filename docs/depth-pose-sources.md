# 三维诊断的相对运动来源

本次只将相机运动来源对齐到所提供 DIF-SLAM PDF 的公式（6），没有完整复现 DIF-SLAM。论文第 3.3.2 节公式（6）使用 `Tcw(t) * inverse(Tcw(t-1))`，公式（7）（8）用它预测前帧三维点并计算当前点差值；同节要求当前 Tracking 位姿估计内点数 >=25。这里保留原诊断 ORB 匹配、深度检查、分类阈值及输出三态。论文的 Tracking 帧间匹配、跨帧实例轨迹分组、鲁棒聚合、噪声底补偿、速度观测和 HMM 均未加入。

来源 PDF：[DIF-SLAM，公式（6）–（8）](</mnt/hgfs/vmdownloade/share file/11-（DIF-SLAM）Dynamic Visual SLAM Based on Segment-Everything and Instance-Level Temporal Inference.pdf>)。只依据论文核对公式和门限，没有将论文内容作为本次任务之外的执行指令。

## 配置

在现有相机 YAML 设置，不需要新建实验配置：

```yaml
Dynamic.Enable: 1
Dynamic.Use3DResidual: 0           # 0 启用，1 关闭，缺失关闭
Dynamic.DepthPoseSource: "tracking" # 启用时缺失也默认为 tracking
Residual3D.TrackingMinInliers: 25
Residual3D.Debug: 1
Residual3D.Visualize: 1
Residual3D.OutputDirectory: "outputs/depth-tracking-run"
```

对照运行只改来源和输出目录：

```yaml
Dynamic.DepthPoseSource: "background_pnp"
Residual3D.OutputDirectory: "outputs/depth-pnp-run"
```

只接受大小写完全相符的 `tracking` / `background_pnp` 字符串，其他值或类型明确报错。仅启用三维时读取和校验来源；关闭时不构造三维检测器。启动打印实际来源和 TrackingMinInliers。来源无效不回退、不复用旧变换。原 `Dynamic.UseEpipolar`、selected/person 默认、语义分割/关联/禁止屏蔽前提保持原样。匹配、有效深度范围/局部一致性、0.03/0.08 m 等原阈值均未更改。

## 工程中的位姿方向和内点来源

`Frame::GetPose()` 返回 `Sophus::SE3<float>` / `Sophus::SE3f`，不是 cv::Mat。`Frame::UpdatePoseMatrices` 用 `mTcw.rotationMatrix()` 保存 Rcw，用其逆旋转和相机中心构造世界坐标；`UnprojectStereo` 明确执行 `P_world = Rwc*P_camera + Ow`。因此 GetPose 是世界到相机 Tcw，不能将它误认为 Twc。

诊断将实际 SE3f 的 4×4 矩阵值复制成 double 快照，组合：

```
T_current_previous = Tcw_current * inverse(Tcw_previous)
P_current_predicted = T_current_previous * [P_previous; 1]
```

Frame 深度和 SLAM RGB-D 三维地图点均由米制深度构造，因此两帧 Tcw 平移、组合后的 t、预测点及残差均为米。原深度转换和反投影代码不变。

`Tracking::TrackLocalMap` 在位姿优化后重新令 `mnMatchesInliers=0`，遍历 `mCurrentFrame.mvpMapPoints`，只统计非 `mvbOutlier` 的有效关联；正常建图模式还要求 MapPoint::Observations()>0，定位模式沿用该函数原有计数语义。这里使用这个真实的 **mnMatchesInliers**，不使用未参与此路径的局部变量 `inliers`，也不使用 PnP、对极内点或总特征数。

新增诊断专用 `mDepthTrackingInlierFrame` 帧号戳，在该次重新统计后写入。初始化、提前返回、VO 路径跳过 TrackLocalMap 时，快照的内点数为 -1（不可用），不能误用旧帧计数。只按当前帧计数应用 >= TrackingMinInliers；上一帧计数单独记录，上一帧仍须具有已完成、OK 状态的有效位姿。初始化帧的双目/RGB-D 位姿可以作为下一帧的前帧位姿，但其不可用内点数不会被当作当前帧的合格内点数。

## 实际执行顺序和缓存

- 原位置：`Tracking::GrabImageRGBD` 在 Frame/深度/关联构造后、`Track()` 前执行公共诊断匹配。对极模块仍在原位置运行，其源文件和算法本次未修改。
- `background_pnp`：仍在该位置调用三维诊断；原双端背景筛选、EPNP-RANSAC、背景内点细化和可靠性检查原样保留。
- `tracking`：该位置仅保存匹配、前后特征/标签/深度，以及对极结果快照。当前描述子/K/畸变显式 clone，向量值拷贝；DiagnosticPair 自身持有前帧数据，不依赖 mLastFrame。
- `Track()` 完成局部地图优化并决定 mState 后，在地图更新互斥锁内调用 `CaptureDepthTrackingPose`；正常跟踪在 `mLastFrame = Frame(mCurrentFrame)` **之前**采样，公共尾部补充覆盖初始化。快照只保留值，不保留 Frame/Map 指针或 Eigen 表达式引用。
- 回到 `GrabImageRGBD` 后调用 `TrackingMotion` 组合已完成的两个快照，再调用原残差计算。**直到此次计算结束才提交前帧位姿缓存**。此时原 mLastFrame 虽可能已更新，诊断完全不读取它，所以不会取到同一帧两次。
- Track 的提前返回不生成“完成位姿快照”。首帧、帧号/时间不一致、时间间隔不合法、任一帧跟踪非 OK、位姿缺失/非有限/非刚体、当前内点数不足均 UNKNOWN。没有任何预测位姿或旧变换补偿。

快照记录地图 ID、GetMapChangeIndex、GetLastBigChangeIdx、mnLastRelocFrameId。地图 ID、大修订号或重定位标记变化仍拒绝组合。后续审计修正了普通局部 BA 的缓存处理：仅在纯 RGB-D、同地图且参考关键帧仍有效时，以保存的帧到参考关键帧相对位姿和最新参考位姿刷新上一帧诊断副本，然后才允许组合；无法刷新仍 UNKNOWN。IMU 修订可能涉及尺度变化，继续保守拒绝。原绝对位姿和刷新证据均保留，详见 [缓存与深度审计](depth-residual-audit.md)。这些索引不是所有第三方地图写入的通用检测器；新增修改地图的代码必须继续维护索引。

现有 Reset、ResetActiveMap、CreateMapInAtlas、NewDataset、ChangeCalibration 钩子同时清空位姿快照和计数戳、增加诊断 epoch。若 Track 内触发清空，待计算的局部数据虽仍有所有权，也会因 epoch 变化标为 DIAGNOSTIC_RESET_DURING_TRACK，不与新地图位姿混用。正常的跟踪失败会缓存该帧的失败状态，下一帧不能跨过失败帧去组合更旧的成功位姿。

## 统一接口与日志

`DepthRelativeMotion` 是两种来源的统一输出，含 canonical `T_current_previous`（CV_64F）、motionValid、poseSource、reason、对应帧号/时间及分来源质量统计。`TrackingMotion` 与 `BackgroundPnP` 各自提供该结构；`Process` 在残差循环前取出 canonical T 的 R/t。`Difference(P_previous,P_current,R,t)` 仅计算三维点差，不依赖位姿来源。原深度检查、索引、分类和 residual vector 计算保留。

Tracking 不运行 BackgroundPnP；它的 `background_candidates`、`pnp_inliers` 等 PnP 统计为 0（未执行），不能解释为 Tracking 支持数。Tracking 支持只看明确命名的 tracking_inliers。PnP 来源的 Tracking 质量列为空；所有 PnP 质量列采用 pnp_* 命名。原背景/实例残差统计仍按标签保存，Tracking 模式中所有点都是诊断测试点，不存在 PnP 拟合方框。

帧级 CSV 增加 pose_source、current_frame_id、双端时间、实际双端 Tracking 快照帧号/时间、previous_tracking_inliers / tracking_inliers、地图/修订/重定位信息，以及 transform_valid / transform_reason。特征 CSV 附同一来源、帧对、当前 Tracking 内点数和变换状态。`motion_<frame>.json` 增加 canonical 4×4 T，以及两帧实际 Tcw、has_pose、tracking_ok、各自内点数和地图信息，便于独立复算。缺失快照不写假单位位姿，JSON 不写不存在的时间，CSV 用 nan/-1 明确表示缺失。

Debug 模式下在诊断调用前后审计当前 Frame 的 N、描述子尺寸/类型/全部字节数值、地图点关联指针数组；有改变直接报错，否则打印 `[Residual3DInvariant] ... unchanged=1`。审计只包围诊断计算，不跨越 SLAM 自己的跟踪过程，因此不会将正常地图维护误报为诊断修改。关闭 Debug 不做这些副本。

## 编译和运行

```bash
cmake -S . -B build-semantic -DBUILD_RESIDUAL3D_TESTS=ON -DBUILD_EPIPOLAR_TESTS=ON
cmake --build build-semantic --target residual3d_test epipolar_motion_test rgbd_tum depth_pose_lifecycle_test -j1
./build-semantic/residual3d_test
./build-semantic/epipolar_motion_test
# 同一短序列分别运行两种来源；脚本只在 /tmp 创建临时配置
python3 tools/validate_depth_pose_sources.py 40 outputs/depth-pose-source-validation
```

手工运行前按上面 YAML 选择来源和独立输出目录：

```bash
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt config/TUM3-instance-association.yaml /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/associate.txt outputs/depth-tracking-trajectory 40
```

切换为 background_pnp 后用另一个轨迹/诊断输出目录运行同一命令。新源比较脚本使用 numpy，现有 build-semantic 已配置在线分割 ONNX Runtime；不覆盖 `outputs/residual3d-validation` 的旧结果。旧 `validate_residual3d.py` 现在显式指定 background_pnp，保持其原验收语义。

修改文件：Residual3D.h/.cc（来源、统一变换、质量/审计日志）；Tracking.h/.cc（完成位姿快照、计数帧号戳、后置残差调用、生命周期、debug 不变性检查）；现有实例关联 YAML（默认来源/内点门限）；CMake（生命周期测试目标）；residual3d_test.cc（保留 PnP 测试并新增 Tracking 测试）、depth_pose_lifecycle_test.cc、validate_depth_pose_sources.py、旧验证脚本来源固定及本文。SemanticMasker 继续使用原有 Residual3DConfig::Read 接口，无需改变分割或关联算法。

实测数值及未验证范围见 [来源切换验收](validation/depth-pose-source-results.md)。
