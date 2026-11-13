# Tracking 三维残差：缓存修订与深度样本审计

本轮按用户图片的四项建议处理，止于诊断，不输出“动态判断质量已通过”的结论。原始输入结果实际位于 `outputs/residual3d-tracking-test`（用户已确认），没有改写原文件。

## 原始 40 帧审计发现

原结果 27 帧有效、12 帧 TRACKING_MAP_REVISION_CHANGED、1 个首帧。被拒绝帧为 6、8、11、14、17、19、23、26、30、33、36、39。12 帧的地图 ID、大地图变化号和重定位标记都没有变化，拒绝仅来自普通 mapChange 增加。这并不意味着地图变化可以一律忽略；原缓存保存的是旧修订下的绝对 Tcw，确实不能直接与当前修订位姿组合。

原结果有 393 个残差 >=0.08 m 的点，其中 113 个原始双帧轴向深度差 >0.20 m。单凭这一统计不能把剩余或排除样本确认为动态/静态；它表明论文门限会改变相当一部分高残差样本。

## 位姿缓存的局部修正

工程的 `Tracking::UpdateLastFrame` 使用 `T_last_reference * T_reference_world_latest` 更新上一帧。诊断现在采用同一表达方式：

1. 在地图更新锁内，当前完成位姿除了 Tcw，还保存参考关键帧 ID、当时参考关键帧位姿和 `T_frame_reference = Tcw_frame * inverse(Tcw_reference)`。
2. 下一帧采样时，若只有普通地图修订号变化，使用该参考关键帧的最新位姿重建**上一帧诊断副本**的 Tcw，并将其修订号更新到当前修订。
3. 本次仅对纯 RGB-D 路径启用刷新：IMU 地图修订可能包含尺度初始化/细化，仍保留 UNKNOWN，不能按普通 BA 刷新。只有同地图、同大变化号、无重定位变化、前帧完成跟踪有效、参考关键帧仍存活且非 bad 并属于该地图时才允许刷新。参考不可用或其他不兼容变化仍 UNKNOWN。没有直接关闭 mapChange 检查。
4. 不写 mLastFrame、Frame 位姿、关键帧、地图点或原优化标记。原缓存绝对 Tcw 保留为 original_Tcw，便于与上一帧输出逐值核对。

关键帧指针只在地图锁内读取，所有 Reset/ResetActiveMap/CreateMapInAtlas/NewDataset/ChangeCalibration 钩子清空诊断引用；其生命周期沿用工程轨迹的参考关键帧保存策略。当前实现不沿 bad 参考关键帧的父链补救，不能验证时保守拒绝。普通 BA 的局部地图变形仍会影响残差；参考表达式解决的是缓存修订一致性，不是独立位姿精度保证。

JSON 增加 `original_Tcw`、`original_map_change`、`reference_id`、`frame_to_reference`、`reference_Tcw`、`rebased`、`rebase_reason`。帧 CSV 增加 `previous_pose_rebased` 和刷新原因/原修订号。可以独立核对刷新表达式与相对变换方向。

## 论文双帧深度差门限

提供的 DIF-SLAM PDF 第 3.3.2 节在式（8）之后写明只保留 `abs(z_current-z_previous) <= 0.20 m` 的匹配。它是**原始两帧轴向深度差**，不是运动补偿后的预测深度差，也不等同于已有的单帧局部 3×3 深度一致性检查。

```yaml
Residual3D.UseDepthDifferenceGate: 1  # 普通布尔开关：1 开、0 关，代码缺失默认 0
Residual3D.MaxDepthDifferenceM: 0.20
```

已有实例关联 YAML 显式开启此门限；新验收同时跑开/关对照。旧来源/PnP 回归脚本显式关闭该新门限，保留原验收语义。只有 Dynamic.Use3DResidual 仍遵循 0=启用的原特殊约定。

零值/非有限值/越界/范围外/局部跳变等已有检查均保留。新门限只影响诊断样本分类，不影响特征数、匹配、背景 PnP 拟合、位姿优化或地图维护。超过门限的可计算样本仍先保存 `raw_residual_3d_m`，但正式 residual_3d_m/残差向量为 NaN、状态 UNKNOWN、原因为 CROSS_FRAME_DEPTH_DIFFERENCE。没有可靠变换时 raw 残差也是 NaN；depth_difference_gate_rejected 仍能记录双端深度有效时的潜在排除条件，不能把这种潜在排除数当成已计算残差数。

附加日志：`depth_difference_m`、`depth_difference_gate_enabled`、`depth_difference_gate_rejected`、筛选前 raw residual；帧级门限和拒绝数量。低/高三维分类阈值仍是原值 0.03/0.08 m，没有为人物变红调参。此门限也可能排除沿相机 Z 方向真实运动或相机快速前进造成的大深度变化，不能把保留后的较小残差当作算法准确性提高。

## 大残差图像与采样审计

新特征 CSV 保存两端原始和去畸变像素。`tools/inspect_depth_residuals.py` 导出背景、人物和其他实例三组的大残差样本，含：

- 两帧原始 RGB 局部图；固定 0–8 m 色标的深度局部图，缺失深度为黑色，不填补。
- 精确查询的 int(u),int(v) 像素标记、原始/去畸变浮点像素、日志米制深度、原 PNG/5000 深度、3×3 窗口范围。
- 对应深度矩阵 `.npy`、逐点 CSV、每点 PNG 和总览 contact_sheet.png。
- 根据保存 K/R/t 和双端像素/深度，独立复算残差向量，验证坐标/方向/单位。

原结果没有保存像素坐标，不能仅凭原 CSV 猜测。工具允许使用相同输入重放的坐标，但先对**全部特征**核对帧号、原始索引、前帧匹配索引、实例标签、双端深度及有效性，再对导出样本复算原日志残差向量（最大分量差须 <1e-10 m），并检查原 PNG 深度采样（误差 <2e-6 m）。任何不一致即停止。原结果的审计图明确记录坐标来自经核验的确定性重放。

本工具只支持此次全分辨率、零畸变、已配准 TUM3 输入；没有用图像尺寸相同来声称任意 RGB-D 已配准。颜色、原始深度矩阵及 patch 只是检查证据，不会自动给遮挡/错误匹配/真实运动下结论。

## 编译和实验

```bash
cmake --build build-semantic --target residual3d_test epipolar_motion_test rgbd_tum depth_pose_lifecycle_test -j1
./build-semantic/residual3d_test
./build-semantic/epipolar_motion_test
python3 tools/validate_depth_audit.py outputs/depth-tracking-audit-v2
```

新脚本仅在 /tmp 创建临时 YAML，不新增专门实验配置。输出根目录已存在则拒绝覆盖。运行修正后的 walking_xyz 相同 40 帧（门限关闭/开启），再运行 sitting_static 120 帧和 walking_xyz 200 帧；保留每次独立 CSV、变换审计、原图采样证据和 SLAM 轨迹。较长片段用于查看分布及失败范围，不能代替逐点动态真值。

修改涉及 Residual3D.h/.cc（深度差门限、raw 残差、审计字段、引用刷新纯函数）、Tracking.h/.cc（诊断参考表达式及重置）、现有 YAML、原回归脚本的显式门限关闭、合成测试，以及两个新审计脚本。没有增加特征剔除、自动融合或 HMM。

实测数据、代表图像和仍未通过的质量检查见 [本轮结果](validation/depth-residual-audit-results.md)。
