# Tracking / background_pnp 位姿来源验收

2026-11-10，当前工程，本地 OpenCV 4.5.4、CPU ONNX Runtime、既有 yolo11s-seg 与 TUM walking_xyz。分割/实例关联开启、特征屏蔽关闭，selected/person 默认未改。仅新增来源选择和必要的位姿采样/执行时机；原匹配、深度检查和三维阈值不变。所有编译使用 **-j1**，未覆盖旧 `outputs/residual3d-validation`。

## 已验证

编译成功：ORB_SLAM3、rgbd_tum、residual3d_test、epipolar_motion_test、depth_pose_lifecycle_test、semantic_mask_test。原工程警告仍存在；没有编译或链接失败。三维测试保留所有背景 PnP 原测试；原对极回归通过；语义/实例关联回归也通过（1013 个特征前后不变，selected/person 默认保留）。

Tracking 合成测试使用非单位的前帧世界到相机位姿（旋转向量 `(0.2,-0.1,0.3)`，t=`(1.2,-0.6,0.9)` m），再用已知相对运动构造当前 Tcw。相对旋转向量 `(0.01,-0.02,0.015)`，相对 t=`(0.04,-0.02,0.03)` m。矩阵组合误差 <1e-12；100 个静态点最大残差 **2.75713e-7 m**，20 个实例测试点加入 0.12 m 位移后残差与该位移一致（误差 <1e-5 m）。旧 PnP 静态最大残差仍为 **2.84656e-7 m**，0.12 m 位移残差误差仍为 **1.36759e-7 m**。

已覆盖：缺失来源默认 tracking，非法字符串/数字/数组拒绝；Tracking 最小内点数 25 接受、24 或 -1 拒绝；两帧缺位姿、非有限位姿、非 OK 跟踪、帧号/时间错配、同帧误用、重置缓存、地图 ID/修订/大修订/重定位标记变化均 UNKNOWN/NaN。没有提供 Tracking motion 时，即使同一组对应能成功估计 PnP，也不会自动回退。

真实脚本 `validate_depth_pose_sources.py 40` 的 **9 个场景全部通过**：

| 场景 | 帧数 | 有效相对运动 | 说明 |
|---|---:|---:|---|
| tracking | 40 | 23 | 16 次 TRACKING_MAP_REVISION_CHANGED，1 次首帧 |
| background_pnp | 40 | 18 | 21 次原 REPROJECTION_UNRELIABLE，1 次首帧 |
| default_tracking | 6 | 5 | 去掉来源配置仍实际运行 tracking |
| tracking_epipolar_off | 6 | 5 | 不依赖对极模块 |
| tracking_quality_rejection | 6 | 0 | 门限 10000，5 次当前内点不足，首帧 UNKNOWN |
| disabled | 5 | 不执行 | 三维/对极关闭，无专用三维输出或诊断匹配调用 |
| invalid_source | 启动退出 | 不执行 | 明确报错，不接受 bad |
| invalid_source_type | 启动退出 | 不执行 | 明确报错，不接受整数 0 |
| lifecycle | 28 | 17 | 空白图像跟踪失败、ResetActiveMap、ChangeDataset |

Tracking 的有效 23 个变换均非单位矩阵。读取保存的真实两帧 Tcw 独立复算 `Tcw_current @ inverse(Tcw_previous)`，最大矩阵元素差 **1.11022e-16**；用 K、匹配像素和米制深度独立复算三维残差向量，最大分量差 **1.33227e-15 m**。PnP 独立残差向量复算最大差 **8.88178e-16 m**。这些数值只验证实现与保存输入的算式一致，不代表相机位姿的真实误差。

具体帧对示例：前帧 0（时间 1341846313.592026）、当前帧 1（时间 1341846313.6541841），当前 Tracking 内点 **330**；组合 t=`(-0.0091321850, 0.00169408065, 0.00383217144)` m。前帧是具有有效初始化位姿但无 TrackLocalMap 内点统计的 RGB-D 初始化帧，前帧内点日志为 -1；门限检查的是当前帧，未误用这个 -1 或总特征数。所有有效帧对均检查帧号不同、时间递增、来源元数据与匹配帧对完全一致。

两种来源在同一 40 帧上的诊断匹配索引、双端深度、有效性/深度原因、实例关联和对极状态/残差 **全部相同**。更强的回归检查：新 background_pnp 的全部原始逐特征列（包括三维向量、残差、状态、原因、拟合内点标志、对极组合字段）与上次 `residual3d-validation/walking_both` 的 40 帧证据逐值完全一致。此次来源分离没有改变原 PnP 结果。

Tracking 有限残差 4832 个，中位数/P90 为 **0.01595/0.05672 m**；PnP 有限残差 3854 个，中位数/P90 为 **0.01803/0.05711 m**。两者有效帧集合不同，这些分布不用于证明某来源更准确，也没有据此调整分类阈值。

所有启用场景共 **126 次** debug 审计，均确认调用前后当前 Frame 的特征数量、描述子尺寸/类型/全部数值和地图点关联指针数组保持一致；这是各次诊断的直接不变性检查。两种来源的关联 CSV 完全一致，关闭三维时前 5 帧关联 CSV 也与开启时前 5 帧一致。没有把正常 SLAM 在两次独立运行中因线程时序产生的地图差异误称为逐点相同地图。

生命周期实测：sample 10–11 将实际 RGB 改为空白输入，sample 10 进入 LOST（state=4）、内点不可用、无匹配，三维 UNKNOWN；下一帧执行工程已有的重置流程，仍 UNKNOWN。sample 16 请求 ResetActiveMap、sample 22 调用 ChangeDataset，这两帧均为 NO_PREVIOUS_FRAME、内点 -1、无效变换；后续正常帧可以重新建立有效帧对。28 帧共 4 次首帧、5 次地图修订拒绝、2 次无匹配、17 次有效。失败分支未复用旧变换，所有无效残差为 NaN。此短序列的 ChangeDataset 实际走工程的少关键帧 ResetActiveMap 分支，不声称它已经覆盖多地图合并或跨地图重定位实测。

## 输出和复现

新输出根目录 `outputs/depth-pose-source-validation/`：

- `tracking/` 和 `background_pnp/`：分别保存两种来源的 `run.log`、轨迹、实例关联和独立 `debug/` CSV/JSON/PNG。
- `default_tracking/`、`tracking_epipolar_off/`、`tracking_quality_rejection/`、`disabled/`、两个非法配置目录和 `lifecycle/`：各自独立证据。
- `summary.json`：逐场景结果及输入不变量/原 PnP 结果精确复现检查。
- `build-single.log`、`residual3d-test.log`、`epipolar-test.log`、`semantic-test.log`、`validation-run.log`：本次编译/测试记录。

```bash
cmake --build build-semantic --target residual3d_test epipolar_motion_test rgbd_tum depth_pose_lifecycle_test -j1
./build-semantic/residual3d_test
./build-semantic/epipolar_motion_test
# 使用新目录；脚本拒绝覆盖旧结果，只在 /tmp 创建和清理临时配置
python3 tools/validate_depth_pose_sources.py 40 outputs/depth-pose-source-validation-repeat
```

配置、两种来源的手动运行方法、修改文件和实际调用位置详见 [位姿来源说明](../depth-pose-sources.md)。

## 未验证及限制

未实测完整长序列、真实跨地图合并、真实闭环后的重定位、IMU RGB-D、其他相机设备或非零畸变真实序列。这些不兼容变换的拒绝条件有合成测试和代码检查，但不能据此声称完整在线路径已全部覆盖。普通局部 BA 也会使修订号变化，导致本版保守拒绝；本次 40 帧中有 16 帧因此 UNKNOWN。线程时序可能改变该次数。

Tracking 位姿仍可能受动态物体和错误匹配影响；当前内点数是支持量门限，不是精度保证。DepthPoseSource=tracking 只对齐论文公式（6）的相机运动来源，未改为论文的 Tracking 匹配、实例鲁棒残差/噪声底/速度/HMM，也没有特征剔除或自动融合。
