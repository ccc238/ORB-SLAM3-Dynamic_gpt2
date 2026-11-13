# 背景对极几何诊断验收结果

日期：2026-11-10。实现范围：独立两帧匹配、背景 F、sqrt Sampson 残差、三态及调试输出；未接入特征剔除、优化或地图点清理。

## 构建及合成验证：已验证

使用现有 `build-semantic`（Release、ONNX Runtime CPU、OpenCV 4.5.4），按要求单线程完成：

```bash
cmake --build build-semantic --target epipolar_motion_test rgbd_tum -j1
./build-semantic/epipolar_motion_test
```

`libORB_SLAM3.so`、`epipolar_motion_test`、`rgbd_tum` 均成功构建。完整构建日志在 `outputs/epipolar-build.log`；新增坐标测试后的单线程增量构建在 `outputs/epipolar-test-rebuild.log`。现有代码有 Eigen 弃用、未使用变量等警告，构建没有错误。合成验证输出 `outputs/epipolar-unit-test.log` 为 PASS。

合成验证覆盖：

- `UseEpipolar=0` 启用、1/缺失关闭；2、-1、0.0、字符串和数组报错。前提分割关闭、关联关闭、屏蔽开启分别抛错。分类阈值相等、非法 ratio、保存频率 0 均拒绝。
- 独立构造非对称 F=`K^-T [t]x R K^-1`，验证 `x_current^T F x_previous`，正确对应近零，交换前后帧不为零。沿当前极线法向加 8 像素扰动后残差超过 3 像素；与 OpenCV `sqrt(sampsonDistance)` 一致至 1e-9；零分母输出 NaN。
- 120 组唯一描述子/已知非平面对应，双向匹配及 previous/current 索引全部正确，100 个背景用于估计 F，20 个实例特征的垂直运动全部标为疑似动态；中间残差区间 UNKNOWN；无效关联不分类。背景 mask 只映射回背景匹配。
- 前帧实例 ID=42、当前 ID=1 时仍正确匹配，不采用跨帧 ID 相等约束。
- 通过径向/切向畸变生成原始像素，保持已知去畸变坐标；残差和分类与无畸变输入完全一致，验证模块不会混用原图/去畸变坐标。
- 首帧、N=0、空描述子、非 ORB 描述子、无匹配、背景不足、F 失败、低位移、平面和纯旋转风险均返回 UNKNOWN。Reset 后首帧仍 UNKNOWN，时间间隔过长、尺寸/标定变化不使用旧模型。
- 描述子、标签、原始/去畸变特征坐标不变；修改调用方的前帧描述子后缓存仍能匹配，确认深拷贝。OpenCV 全局 RNG 状态不变。检测器接口没有地图点/原优化外点标志输入，源码接入也不写这些字段。

## 真实 TUM：已验证

执行 `python3 tools/validate_epipolar.py 60`，数据：`/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz`，模型：`/home/aaa/models/yolo/yolo11s-seg.onnx`。脚本仅在 `/tmp` 创建并自动清理临时参数文件，没有新增实验 YAML。各模式都使用原 `rgbd_tum` 程序和在线分割/关联；Viewer 关闭，CSV/PNG 验收不依赖窗口。

| 模式 | 实测结果 |
|---|---|
| UseEpipolar=0 | 60 帧，实际匹配/估计/残差；58 帧 OK，首帧 UNKNOWN，另 1 帧 LOW_DISPLACEMENT_RISK |
| UseEpipolar=1 | 5 帧正常 Tracking=2；无诊断目录/专用输出；实例关联 CSV 仍正常生成 |
| 缺失 UseEpipolar | 5 帧正常 Tracking=2；启动打印原始值 1 / DISABLED；无诊断目录/专用输出 |
| UseEpipolar=2 | 明确报错：只接受整数 0/1，说明 0=启用，1=关闭 |
| UseEpipolar=0.0 | 明确报错：必须为整数 |
| UseEpipolar=0，Dynamic.Enable=0 | 明确报错：要求在线分割及实例关联开启、语义屏蔽关闭 |
| 内部缩放到 480×360 | 8 帧正常 Tracking=2；5 帧模型有效，首帧及 2 帧低位移风险 UNKNOWN |
| SaveEvery=2 | 处理 5 帧，保存 0/2/4 三帧；索引指向真实上一输入帧而非上一保存帧 |
| UseEpipolar=0，Debug/Visualize=0 | 5 帧正常 Tracking=2；没有输出目录及专用输出 |

0、1、缺失三种模式的前五帧特征数量完全一致：1013、1008、1007、1009、1007。所有正常运行的帧都为 Tracking=2。全部关联记录满足关联前后特征数量不变；全部诊断记录满足数组长度=N、当前特征索引连续、前帧索引一对一、实例 ID 与原关联 CSV 相符。无效模型下全部 UNKNOWN/NaN。逐帧核对三种状态实际行数与帧级统计完全一致（补充核对结果在 `audit.json`）。

60 帧共 60,415 个特征记录：UNKNOWN=44,908，GEOMETRY_CONSISTENT=14,877，SUSPECTED_DYNAMIC=630。其中实例内部：UNKNOWN=24,983，一致=6,548，疑似动态=532。UNKNOWN 包含大量未通过独立匹配的特征，不能作为动态率。有效残差的背景中位数 0.400 px、P90 1.261 px；实例中位数 0.657 px、P90 2.111 px。背景也有 98 个高残差记录，不能将高残差或背景 RANSAC 外点直接解释为真实动态。

58 个有效模型帧：总匹配 213–498、背景匹配 92–237、背景内点 73–219，内点率 0.786–0.944，网格覆盖 0.5625–0.9375。输入间隔 0.0318–0.0661 s，符合默认 MaxDt=0.2 s。

诊断三部分平均耗时（包含首帧及失效帧）：匹配 3.550 ms、估计及退化检查 0.435 ms、残差 0.029 ms，合计约 4.014 ms。该统计不包含输入构造/深拷贝及调试 CSV、图片生成和写盘，不能当作总跟踪耗时。

已实际查看 `enabled/debug/epipolar_20.png`：原始 RGB 坐标叠加、左右帧连线、绿/红/黄三态、模型原因和背景支持数量显示正常，顶部标明 `Geometry diagnostic; features NOT removed`。

## 文件及输出

实现修改：

- 新增 `include/EpipolarMotionDetector.h`、`src/EpipolarMotionDetector.cc`。
- `include/SemanticMasker.h`、`src/SemanticMasker.cc`：接入统一配置读取，保留原开关解析行为。
- `include/Tracking.h`、`src/Tracking.cc`：关联后/Track 前运行，独立结果及生命周期清理。
- `CMakeLists.txt`：库源文件及可选验证目标。
- 现有 `config/TUM3-instance-association.yaml`：加入反向开关，CSV/图片默认仍关闭。
- 新增 `tests/epipolar_motion_test.cc`、`tools/validate_epipolar.py`、`docs/epipolar-diagnostic.md`；旧动态开关文档加历史说明链接。

真实数据证据：`outputs/epipolar-validation/summary.json`、`audit.json`。各模式的 `run.log`、`RunFrames.csv`、轨迹文件及原实例关联输出都保存在对应子目录。诊断 CSV/PNG 在 `enabled/debug/`、`resized/debug/`、`frequency/debug/`。默认手动运行输出目录为 `outputs/epipolar/`，需在现有配置设置 Epipolar.Debug/Visualize=1。启动命令、全部实际参数和风险检查说明见 [运行说明](../epipolar-diagnostic.md)。

## 尚未验证及边界

- 没有做长序列精度、动态点真值精度/召回率或位姿逐位一致性评估；疑似动态不是运动真值。
- 实际 TUM 配置畸变系数为零；非零畸变坐标分离已做合成验证，尚未用真实非零畸变相机验收。
- 重置、尺寸/标定不兼容及退化风险已做模块测试和接入源码检查，未在运行中的真实 SLAM 序列主动触发所有重置/重标定路径。
- IMU RGB-D 和其他传感器未运行数据验收；本次实际运行仅 RGB-D pinhole。模型可观测性筛查是保守启发式，不保证识别所有退化，也可能拒绝部分可用模型。
- 可视化验收为已保存 PNG，未做交互 GUI 窗口验收。未新增热切换配置能力。
