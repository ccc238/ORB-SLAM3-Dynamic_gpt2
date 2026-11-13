# 局部图像跟踪交叉验证

LK 是独立的匹配审计证据，不改变 ORB 诊断匹配、特征坐标、深度、实例标签、地图点关联、优化外点标志及对极/三维分类。不输出融合动态结论。

## 开关和参数

在已有配置中加入（本次不新增实验 YAML）：

```yaml
Dynamic.UseLKCheck: 0            # 整数 0 开、1 关；缺失默认关
LKCheck.OutputDirectory: "outputs/lk-check-new-run"
LKCheck.Visualize: 1            # 普通整数标志：1 保存图，0 不保存图
LKCheck.SaveEvery: 10
LKCheck.MaxImagesPerFrame: 4
LKCheck.WinSize: 21             # 正奇数；正反向相同
LKCheck.MaxLevel: 3
LKCheck.Iterations: 30
LKCheck.Epsilon: 0.01
LKCheck.MinEigThreshold: 0.0001
LKCheck.MaxFBErrorPx: 1.0
LKCheck.MaxEndpointErrorPx: 3.0
LKCheck.MaxDt: 0.2
```

这些是实验起点，受分辨率、金字塔、纹理、遮挡、形变和 ORB 定位误差影响，不是通用最优参数。开关以 `enabled=(value==0)` 显式解析；非法整数、浮点、字符串或复合类型报错。启动打印原始值（缺失打印 `<missing>`）、实际状态、参数及输出路径。关闭不构造 LK 对象，不缓存图像、不计算 LK、不创建 LK 输出。

现有几何模块启用时仍需开启在线分割、实例关联，关闭语义特征屏蔽；不改变 selected/person 默认。LK 仅支持本工程 RGB-D 接口（含 RGB-D IMU 图像接口）。它复用任一几何模块已产生的公共匹配，两个几何模块都关闭时记录 `NO_MATCH_INPUT`、候选为零，不调用新匹配器；此时即使分割也关闭，仍可明确记录没有匹配输入。原 `Dynamic.UseEpipolar` 和 `Dynamic.Use3DResidual` 约定不变。

## 图像、索引及执行位置

`Tracking::GrabImageRGBD` 内颜色转灰完成后，`mImGray` 用于原 Frame ORB 提取。LK 使用与该图像对齐的 `mvKeys` 原始像素，不使用 `mvKeysUn`；畸变图像也因此不会误用去畸变坐标。图像缩放沿用 System/Tracking 的现有处理，LK 不另行缩放。

公共匹配就绪后、Track 之前，调用 `LKCheck::Process`；当前图像深拷贝保存，前帧缓存为模块所有的不可变引用计数缓冲区。逐帧核对公共匹配的前帧 ID、时间戳、尺寸与缓存，检查灰度 `CV_8UC1`、标定一致性、严格递增的帧号/时间及最大间隔。无效尺寸/类型、首帧、时间间隔、标定或身份不兼容安全返回不可验证，当前有效图像成为下一次的锚点，不能跨过不兼容帧使用旧图像。

所有已有公共匹配重置位置同时重置 LK：`Reset`、`ResetActiveMap`、`CreateMapInAtlas`、`NewDataset` 和 `ChangeCalibration`。若 Track 内发生重置，待记录结果也标为 `DIAGNOSTIC_RESET_DURING_TRACK`，清空有效端点与误差。LK 缓冲区独立持有，不读取被覆盖的 mLastFrame 图像。

Track 之后、Tracking 来源的三维结果产生后，`LKCheck::Record` 用同一帧保存的输入、公共匹配和对极快照，交叉记录对应三维结果。LK 未开启时不进入这些分支。开启时计算周围检查特征数、描述子、地图点数组、优化外点标志及原灰度图不变。

## 独立测量与状态

按上一帧原索引去重后，一次正向批量 `calcOpticalFlowPyrLK(p→q_LK)`；仅把正向成功且坐标有效的 q_LK 送入反向批量 `calcOpticalFlowPyrLK(q_LK→p_back)`。两次 flags 都为 0，没有 `OPTFLOW_USE_INITIAL_FLOW`，不传 ORB 端点或相机运动预测作为初始终点。

- `fb_vector=p_back-p`，`fb_error=norm(fb_vector)`。
- `endpoint_vector=q_LK-q_ORB`，`endpoint_error=norm(endpoint_vector)`。
- 前/反向 status、有限性、中心坐标越界或 FB 门限失败：`UNVERIFIABLE`。
- FB 有效且 <=1 px，端点差 <=3 px：`SUPPORTED`；端点差 >3 px：`DISAGREES`。

所有量均为输入图像原尺度像素。OpenCV minEigThreshold 提供局部纹理门限；仅检查点中心在图内，窗口边缘沿用 OpenCV 的金字塔边界处理，不声称图像边缘都有完整真实像素支撑。没有导出 OpenCV err：flags=0 下它是平均 L1 补丁亮度差，不是坐标位移误差；status=0 的输出位置/err 均不作为有效结果。

SUPPORTED 不是正确匹配真值，DISAGREES 不是动态标签，UNVERIFIABLE 也没有动静含义。LK 允许局部物体运动，与静态相机运动补偿检查不是同一概念；重复纹理、局部极小值、非刚性形变及遮挡仍可能欺骗或破坏 LK。

## 输出与复现

`features.csv` 覆盖全部原诊断匹配，包括跟踪失败者，记录两帧 ID/时间、原始索引及 p/q_ORB/q_LK/p_back、正反向 status、两种误差向量/标量、独立 LK 状态/原因、两端实例标签以及原几何状态/残差（含 raw 三维残差）。无对应的几何模块字段留空；未计算 status 留空，无效端点/误差为 NaN。实例数字不当作跨帧身份。

`frames.csv` 包含候选数、去重点数、正反向有效数、三状态数量、所有有限误差的中位数/P90、计算及记录耗时、失败原因。计算耗时包含图像缓存和 LK；记录耗时包含逐行格式化及选图保存，结束 CSV 刷新不计入该局部计时。三维不同状态与 LK 的交叉统计由验证工具使用同一次日志计算。

可视化使用白色 p、橙色 ORB 端点、青色 LK 端点、品红色 p_back，不沿用几何红绿。SaveEvery 控制常规图频率，MaxImagesPerFrame 限制每帧数量（0 禁图）；指定的 6→7/32→29、40→41/142→94 对应在存在时优先于频率保存。每图标记两种误差、LK 状态及原几何状态；历史复现是否匹配由验证工具检查，不凭 ID 相同就认定输入相同。

```bash
cmake -S . -B build-semantic -DBUILD_LK_CHECK_TESTS=ON
cmake --build build-semantic --target rgbd_tum lk_check_test residual3d_test epipolar_motion_test depth_pose_lifecycle_test -j1
./build-semantic/lk_check_test
python3 tools/validate_lk_check.py outputs/lk-check-validation-new
```

验证脚本只在 `/tmp` 生成临时 YAML，保存实际设置为输出目录内的文本，拒绝覆盖结果根目录。普通数据集运行继续使用 `rgbd_tum vocabulary existing_settings dataset associate.txt fresh_output frame_count`；须为 LK、三维、对极及关联日志各选择新目录，避免原有日志构造函数覆盖旧文件。
