# LK 局部图像交叉验证结果

2026-11-12。输出：`outputs/lk-check-validation/`。仅增加独立 LK 审计，未修改 ORB 匹配、对极/三维公式与分类、深度门限及地图维护，未引入特征剔除、融合或 HMM。

## 已验证的输入、调用与实现

LK 使用 `GrabImageRGBD` 中提取 ORB 的 `mImGray` 和对应 `mvKeys` 原始像素，不使用 `mvKeysUn`。前帧灰度为模块所有的深拷贝；两帧图像与公共匹配的 ID、时间、尺寸和标定一致才执行。LK 在公共匹配后、Track 前批量计算；在 Tracking 三维残差生成后记录交叉日志，背景 PnP 来源则保留其 Track 前结果快照用于交叉记录。没有用 ORB 端点或预测位姿初始化光流。

按前帧索引去重，一次正向批量 p→q_LK，然后对成功、有限且在图内的 q_LK 一次反向批量 q_LK→p_back。两次 flags=0；minEigThreshold 用于纹理检查，不解释 status=0 时的返回点或误差。先检查 FB，再检查端点差，两种向量和标量都保存。输出独立 SUPPORTED / DISAGREES / UNVERIFIABLE，原几何状态未改写。

首帧、重置、活动地图重置、新地图、新序列和标定变化均有缓存处理。开启时检查 Frame 特征数量、描述子、地图点关联、优化外点标志及输入灰度不被 LK 修改。本次共 **240 次**检查通过；关闭/缺失开关的另外 6 帧没有进入该分支。42 个原始 `residual3d-tracking-test` 文件的 SHA256 与 mtime 与上一轮 manifest 全部一致。

## 历史案例与人工抽查

静坐重放逐行核对 **120812** 条原日志输入，行走前 40 帧核对 **40286** 条；当前/前帧原特征索引、实例标签、双端深度和像素完全一致。两项指定案例因此可以按同一输入比较。新完整 SLAM 重放受 BA 线程时序影响，位姿与三维残差不要求等于历史运行；下表三维值均使用本次同一运行日志，未混用历史数值。

| 场景、帧与原特征 | LK 结果 | FB px | 端点差 px | 本次三维残差 m | 人工检查 |
|---|---|---:|---:|---:|---|
| 静坐 6→7，32→29 | DISAGREES | 0.005825 | 152.183325 | 2.089150 | 原 ORB 横跳到另一处相似隔板接缝；LK 留在原接缝邻域，提示明显可疑错配 |
| 静坐 40→41，142→94 | DISAGREES | 0.012231 | 143.577656 | 0.540315 | ORB 从膝部附近跳到鞋部附近；LK 留在原身体部位邻域，提示明显可疑对应 |
| 静坐 6→7，23→25 | SUPPORTED | 0.004579 | 0.210017 | 0.003312 | 同一背景海报边缘局部区域，人工外观检查相符 |
| 行走 24→25，713→711 | SUPPORTED | 0.035513 | 0.489247 | 0.030155 | 格纹上衣局部外观相符；LK 图像位移约 17.17 px，人体运动未被当成失败直接排除 |
| 行走 45→46，123→41 | SUPPORTED | 0.007592 | 0.847277 | 0.029500 | 头部发际边缘局部外观相符，LK 图像位移约 16.37 px；存在模糊/边界影响，仍不能当作精确真值 |

人工抽查是外观核查，不是逐像素真值。格纹也有重复性，SUPPORTED 仍不能证明物理对应绝对正确；可观察的人体运动不等同于已经标定的三维速度。

图片包含原 RGB 全图、p/q_ORB/q_LK/p_back 四张局部图；LK 本身仍是在对齐灰度上计算：

- [重复背景结构](../../outputs/lk-check-validation/sitting_static120/selected-review/frame_7_feature_29.png)
- [膝部/鞋部对应](../../outputs/lk-check-validation/sitting_static120/selected-review/frame_41_feature_94.png)
- [背景海报对应](../../outputs/lk-check-validation/sitting_static120/selected-review/frame_7_feature_25.png)
- [运动人物衣纹](../../outputs/lk-check-validation/walking80/selected-review/frame_25_feature_711.png)
- [运动人物头部](../../outputs/lk-check-validation/walking80/selected-review/frame_46_feature_41.png)

白色 p、橙色 ORB、青色 LK、品红色 p_back，图内说明没有特征删除，不采用几何分类的红绿颜色。常规图按频率和每帧数量限制，指定历史案例在存在时保留优先槽位。人工选图由 `inspect_lk_check.py` 从同一次 CSV 和原 RGB 导出，没有重跑光流或更改结果。

## 全量匹配与交叉统计

统计覆盖全部公共诊断匹配，共 **88076 对**，不是只检查大残差。原对极状态/残差、三维正式/raw 残差逐行与本次独立几何日志完全一致。几何一致、可疑和 UNKNOWN 都保留；无深度、深度差筛选等不影响该匹配的 LK 证据输出。

| 片段 | 帧数 | 匹配对 | SUPPORTED | DISAGREES | UNVERIFIABLE |
|---|---:|---:|---:|---:|---:|
| sitting_static | 120 | 61088 | 59591 | 1459 | 38 |
| walking_xyz | 80 | 26988 | 25702 | 1166 | 120 |

下表百分比以各行原三维分类样本数为分母；不是准确率或误匹配率。

| 片段及原三维分类 | 样本数 | SUPPORTED | DISAGREES | UNVERIFIABLE |
|---|---:|---:|---:|---:|
| 静坐 GEOMETRY_CONSISTENT | 29148 | 28584（98.07%） | 554（1.90%） | 10（0.03%） |
| 静坐 SUSPECTED_DYNAMIC | 1003 | 952（94.92%） | 46（4.59%） | 5（0.50%） |
| 行走 GEOMETRY_CONSISTENT | 10876 | 10539（96.90%） | 302（2.78%） | 35（0.32%） |
| 行走 SUSPECTED_DYNAMIC | 739 | 547（74.02%） | 178（24.09%） | 14（1.89%） |

中等残差（0.03~0.08 m）同样覆盖：静坐 3272 对中 3074/189/9，行走 3134 对中 2878/228/28，顺序均为 SUPPORTED/DISAGREES/UNVERIFIABLE。完整 UNKNOWN 及各残差区间统计见 `summary.json`。

静坐 38 个不可验证均为 FB_INCONSISTENT；行走为 119 个 FB_INCONSISTENT、1 个 OUT_OF_BOUNDS。静坐另有 **15** 个、行走 **59** 个样本端点差 <=3 px，但 FB>1 px，正确保留 UNVERIFIABLE，没有因为端点接近就判 SUPPORTED。例：静坐 4→5，240→286，端点差 0.78567 px，FB 1.28676 px，状态 UNVERIFIABLE。

**这两个明显错配被提示，不表示所有三维高残差都是错配。** 静坐高残差中 94.92%、行走高残差中 74.02% 仍获 LK 支持；这部分还可能涉及深度、距离/噪声、遮挡、位姿及实际物体运动等因素。几何一致样本中也存在 LK 分歧。没有人工真值，不能由上述状态统计推导错误率或动态检测准确率，也未自动让不可验证样本退出几何分类。

## 耗时

本机本次测量，单位 ms/帧。compute 含图像缓存和批量 LK；record 含逐行格式化、选图编码/写盘，不含最后 CSV flush。额外输入不变性快照不计入 compute，因此不能把 compute 单独视为全部审计开销。

| 片段 | compute 中位数/P90/最大 | record 中位数/P90/最大 |
|---|---|---|
| 静坐 120 | 3.112 / 3.982 / 4.607 | 3.689 / 18.998 / 67.297 |
| 行走 80 | 2.194 / 2.707 / 4.888 | 2.469 / 3.406 / 81.059 |

选图保存有明显开销，不能据此宣称整个 SLAM 的实时性能。这是本机参数和片段的单次记录，不是跨设备性能保证。

## 配置与边界验证

- `Dynamic.UseLKCheck`：整数 0 开、1 关、缺失关；2、-1、0.0、字符串 "0"、序列、映射和 true 均拒绝。内部显式 `value==0`。启动打印实际状态。
- 合成带纹理 640×480 图像，平移 (4,-3) px：最大端点差约 0.001009 px，最大 FB 约 0.000583 px。人为偏移 ORB 端点 (45,20) px 后仍保持小 FB，但得到 DISAGREES。
- 单元测试覆盖独立原始/去畸变坐标、缓存所有权（调用者覆盖图像后仍可用）、重复索引只算一次、低纹理 FORWARD_FAILED、不相关纹理 FB_INCONSISTENT、越界/NaN、无匹配、空图、尺寸/时间/帧对应不兼容、重置及待记录结果失效。可视化测试检查每帧只允许 1 张且 SaveEvery=1 时历史案例仍优先，尺寸不兼容不进入图像拼接。
- 合成测试开发时，较小 320×240 随机纹理中有一个点正向接近平移真值但反向 status=0；它正确成为 BACKWARD_FAILED，未被当作支持。正式已知平移数值测试采用具有充分金字塔边界余量的 640×480 内部点，未降低阈值；这也体现边界支撑与图像纹理会影响 LK。
- 真实 3 帧开关 1、缺失两次都无 LK 目录和计算审计标记；原几何诊断仍运行。两个几何模块都关时 3 帧均 NO_MATCH_INPUT、0 候选；额外关闭分割再测 3 帧也同样安全记录 NO_MATCH_INPUT，未增加全图匹配。
- 对极关/三维开与三维关/对极开各 3 帧，均有 731 对 LK 记录，关闭模块的几何字段为空，未伪造零。
- 28 帧生命周期测试含空白图像造成跟踪失败、ResetActiveMap 和 ChangeDataset。帧级原因：4 次 NO_PREVIOUS_FRAME、22 次 OK、2 次 NO_MATCH；重置/换序列后没有沿用图像缓存。
- 主库、rgbd_tum、LK/三维/对极测试、生命周期目标均 `-j1` 编译通过；LK 测试以及既有三维、对极回归通过。原有项目警告未作无关修改。

真实所有设备畸变/缩放组合、所有重定位/闭环场景及人工逐点真值未全面验证；不声称完整算法或 DIF-SLAM 动态判别复现。

## 文件、配置和复现

修改/新增：`include/LKCheck.h`、`src/LKCheck.cc`（独立模块）；`include/SemanticMasker.h`、`src/SemanticMasker.cc`（配置读取）；`include/Tracking.h`、`src/Tracking.cc`（调用、结果快照和生命周期）；`CMakeLists.txt`、`tests/lk_check_test.cc`；`tools/validate_lk_check.py`、`tools/inspect_lk_check.py`；[实现与全部配置项](../lk-check.md)及本报告。已有相机 YAML 未修改，没有新增专门实验 YAML。

```bash
cmake -S . -B build-semantic -DBUILD_LK_CHECK_TESTS=ON
cmake --build build-semantic --target rgbd_tum lk_check_test residual3d_test epipolar_motion_test depth_pose_lifecycle_test -j1
./build-semantic/lk_check_test
python3 tools/validate_lk_check.py outputs/lk-check-validation-repeat
```

要在已有运行配置启用：`Dynamic.UseLKCheck: 0`，指定新 `LKCheck.OutputDirectory`；其余参数初值和普通数据集运行命令见实现说明。LK 开关仅控制本轮审计，不增加任何 LK 决策门限到原算法。复跑验证脚本仅在 `/tmp` 创建临时 YAML。

根目录 `summary.json` 为全量交叉统计，`verification.json` 为不变性核验；每段 `lk/features.csv`、`lk/frames.csv`、可视化 PNG、`depth/`、`epipolar/`、`association/`、`run.log` 与实际配置文本各自保存。`selected-review/selected_matches.csv` 和样本图便于人工复查。编译及测试日志另置于根目录。

交付停在独立交叉验证；没有创建“匹配不确定样本退出几何分类”开关，是否进一步处理留待实验证据和后续明确指令。
