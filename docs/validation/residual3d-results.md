# RGB-D 三维静态补偿残差验收记录

2026-11-10，本地 OpenCV 4.5.4、CPU ONNX Runtime，现有 yolo11s-seg 模型；使用已有 TUM3 相机配置、selected/person 策略，在线分割和实例关联开启、语义特征屏蔽关闭。使用默认实验初值：低/高三维阈值 0.03/0.08 m，PnP 门限 2 px，200 次迭代，confidence 0.99，最少背景 30、最少内点 20、内点率 >=0.5。没有按人物颜色调整参数。

## 编译和合成测试：已验证

用户要求后，所有编译均使用 `-j1`。ORB_SLAM3、rgbd_tum、residual3d_test、epipolar_motion_test、semantic_mask_test 编译通过。构建仍有工程既有警告，不影响成功链接。

- `residual3d_test`：0 启用，1/缺失关闭；拒绝 2、-1、0.0、字符串、序列、映射。缺分割、缺关联、屏蔽开启均报错。测试不构造对极检测器。
- 已知旋转向量 `(0.01,-0.02,0.015)` rad、t=`(0.04,-0.02,0.03)` m 的 120 个合成匹配（100 背景 + 20 实例）。静态最大残差 **2.84656e-7 m**；实例加入 `(0.12,0,0)` m 位移，残差与 0.12 m 最大差 **1.36759e-7 m**。原始像素变动而去畸变像素不变时三维结果保持一致。
- 验证中间阈值 UNKNOWN、两端缺深度、零/负/NaN/Inf/范围外深度、局部跳变、窗口/像素越界、背景不足、退化输入、无匹配、首帧、重置、时间间隔、尺寸/标定变化、缓存深拷贝及输入不变。
- 随机错误二维对应产生明确 **PNP_FAILED**；成功估计之后紧接失败仍全部 UNKNOWN/NaN，不沿用旧变换。负预测深度返回无效。
- 原对极测试通过：F 方向、残差、索引、坐标、退化、生命周期、RNG 和不变性等。
- 语义/关联回归通过：真实图像 1013 个特征关联前后保持 1013，背景 462、实例 551，其中人物 361；selected/person 默认及其他原行为保持一致。

## 真实序列：已验证

执行 `python3 tools/validate_residual3d.py 40`，12 个场景全部通过：walking_3d_only、sitting_both、walking_both（各 40 帧），disabled、missing、invalid、float、string、prerequisite、insufficient_background、no_depth、resized（有效短场景各 5 帧，错误配置直接退出）。验收只在 /tmp 创建并清理临时配置，没有添加实验 YAML。

下表为**有效残差点的汇总分布**，单位米；不是所有特征的均值，也不是独立定位精度。人物由关联元数据 class_id=0 筛选，未把桌椅/显示器算成人物。拟合背景单列，非拟合背景也不是带静态真值的独立测试集。

| 序列 / 模式 | 可靠运动帧 | 拟合背景 n / 中位数 / P90 | 非拟合背景 n / 中位数 / P90 | 人物 n / 中位数 / P90 |
|---|---:|---|---|---|
| walking_xyz，三维开/对极关 | 18/40 | 932 / 0.01727 / 0.07555 | 315 / 0.02880 / 0.13459 | 1695 / 0.01560 / 0.03525 |
| walking_xyz，三维开/对极开 | 18/40 | 932 / 0.01727 / 0.07555 | 315 / 0.02880 / 0.13459 | 1695 / 0.01560 / 0.03525 |
| sitting_static，两模块开 | 14/40 | 951 / 0.01358 / 0.04634 | 261 / 0.02259 / 0.12747 | 1730 / 0.00943 / 0.02430 |

walking 的 21 帧、sitting 的 25 帧被 REPROJECTION_UNRELIABLE 保守拒绝，各另有 1 首帧 UNKNOWN。检查要求所有细化背景内点都满足 2 px，因而即使 RMS 较低也可能拒绝。本次保留该初版门限，未为增加有效帧数放宽检查。

人物三态（计入未匹配/深度无效/模型不可靠特征）：walking **UNKNOWN 13815，一致 1452，疑似异常 56**；sitting **UNKNOWN 13185，一致 1599，疑似异常 29**。有限人物残差数量包含中间阈值 UNKNOWN，故不等于一致+疑似异常之和。静坐人体也有异常、行走人体多数没有变红；相邻帧运动小、匹配/深度/遮挡影响明显，不能据此确认每个人体点真实动静。静坐序列视图确认存在坐着的人物，但不声称人体各部位具有逐点静止真值。

对极一致但三维异常：walking 全部点 158 个，其中人物 **11** 个；sitting 全部点 **131** 个，其中人物 **18** 个。两种状态独立保存，无融合结论。

walking 的三维开/对极关与两模块开：除对极字段外，**全部逐特征三维输出完全相同**；关联特征 CSV 也逐字节一致。两个模块均关的前 5 帧关联 CSV 与启用时前 5 帧一致。全部有效场景都检查 N、索引连续性、匹配一对一、标签一致性、背景拟合点双端标签均为 0，以及 invalid model => UNKNOWN/NaN。原 SLAM 地图点/优化外点结构未做运行时二进制快照；代码检查确认模块无这些结构的输入和写入接口。

无有效深度场景使用 [20,30] m 范围故意拒绝当前短序列深度，全部 UNKNOWN；背景门限 10000 时全部 UNKNOWN。缩放为 480×360 的 5 帧运行成功，1 帧可靠估计；不把它视为缩放边缘准确性的证明。

深度源核查：第一张 walking 深度 PNG 为 **16 bit 灰度，640×480**；正值中位数 13090，除以 5000 为 **2.618 m**。首帧 **729** 个正深度特征使用实际原始特征像素查询 PNG，与日志 current_depth_m 的最大差 **6.2561e-7 m**（float 换算精度）；原始无深度位置为 -1、valid=0。不存在重复缩放或把原始值当毫米的问题。

用保存 R/t、K、关联像素、双端深度独立复算全部有效残差向量：walking 最大分量差 **1.303e-15 m**，sitting **1.142e-15 m**。按 OpenCV JSON 的 dt=f 恢复 K 的 float32 原值后转 double，避免 JSON 十进制打印误差；这些序列畸变为零，因此关联原始像素与去畸变像素相同。非零畸变像素分离已在合成/对极测试中检查，未在真实非零畸变序列上验收。

平均耗时（含首帧，机器负载相关，不含分割、输入副本/深度窗口检查及文件写入）：walking_3d_only 公共匹配 **5.70 ms**、PnP/检查 **0.663 ms**、残差 **0.149 ms**；walking_both 分别 **3.62 / 0.642 / 0.143 ms**；sitting_both **3.46 / 0.479 / 0.151 ms**。不同运行的匹配时延差不能用于推断开启对极让匹配加速。

## 证据、命令和未验证内容

工程根目录下：

- `outputs/residual3d-validation/summary.json`：完整数值、原因统计、人物统计、独立复算和不变量结果。
- 每个场景下 `run.log`、`RunFrames.csv`、`association/*.csv`；启用场景下 `debug/features.csv`、`debug/frames.csv`、`debug/motion_*.json`、`debug/residual3d_*.png`；两模块开启的场景另有 `epipolar/`。
- `sitting_both/debug/residual3d_20.png` 已检查：独立三维颜色、PnP 拟合方框、测试圆点、组合紫圈及“features NOT removed”标题可见。
- `build-single.log`、`residual3d-test.log`、`epipolar-test.log`、`semantic-test.log`：编译和测试证据。

```bash
cmake -S . -B build-semantic -DBUILD_RESIDUAL3D_TESTS=ON -DBUILD_EPIPOLAR_TESTS=ON
cmake --build build-semantic --target residual3d_test epipolar_motion_test rgbd_tum semantic_mask_test -j1
./build-semantic/residual3d_test
./build-semantic/epipolar_motion_test
./build-semantic/semantic_mask_test /home/aaa/models/yolo/yolo11s-seg.onnx /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/rgb/1341846313.592026.png outputs/residual3d-semantic-check
# 换新输出目录，脚本不会覆盖既有证据
python3 tools/validate_residual3d.py 40 outputs/residual3d-validation-repeat
```

未验证：完整长序列、真实非零畸变/其他设备、IMU RGB-D 实测、未配准输入、全生命周期在线交互复位、逐点动静真值、独立相机位姿精度、阈值最优性及全场景鲁棒性。重置/标定/序列切换接入已做代码检查，公共缓存行为有合成测试；没有冒称上述在线全路径均实测。固定米阈值受距离和噪声影响，背景重投影好也不等于三维独立精度好；此短序列证据支持模块正确接线和计算，尚不足以支持实际剔除。工作停留在三维诊断，未推进 HMM。
