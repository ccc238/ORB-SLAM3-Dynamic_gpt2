# ORB 特征与实例标签关联验收（2026-11-09）

已完成本步关联与可视化，保留 selected/person 默认屏蔽行为。未推进几何残差或动态特征剔除。

## 实际通过

- `cmake --build build-semantic -j1` 构建全部已配置目标通过，包括新旧 RGB-D、单目、双目、惯性示例及原有 semantic_mask_test。显示文字描边调整后再次单线程增量构建全部目标通过。日志 `outputs/instance-association/build.log`、`build-final.log`。本机 GCC 11.4、OpenCV 4.5.4、CPU ONNX Runtime 1.17.3。
- 现有测试目标扩展后执行通过，日志 `module-test.log`。实际 YOLO 模型及 TUM walking_xyz 首帧得到 11 个 NMS 实例，包含 person、bottle、chair、tv、mouse、keyboard。
- 单帧正常提取 1013 个 ORB 特征，关联后仍为 1013：背景 462、实例内 551、无效 0。其中人物实例内 361（两个实例分别 110、251），其他实例内 190。ID 数组长度=N，三类和=N，正 ID 均存在本帧信息表。
- 受控测试比较关联前后的原始关键点顺序、描述子、深度、右目等效坐标、地图点数组，均不改变。Frame 复制构造及赋值保持标签；副本 ID 向量独立；赋入空 Frame 后不残留标签。禁用时 N 个 ID 为 -1。
- 空检测关联为背景；重叠取最高置信度，相同置信度稳定取原顺序；floor 查询、负坐标、右/下边界、NaN、Infinity、标签类型/尺寸错误及缺失实例表均检查通过。关联开启而推理关闭的配置明确拒绝。
- 关联标签图不受 selected 类别限制；person 模式的关联标签与 all 模式一致。膨胀半径 3 与未膨胀标签逐像素相等。不同尺寸（含竖图、非整比例、小图）推理标签类型及尺寸检查、查询通过；原屏蔽、RGB/BGR、输入不变性和原接口回归通过。
- 真实短序列：关联 12 帧，逐帧/逐特征用 CSV 的 floor(u,v) 与 16 位 PNG 重新读取比较，全相等；三类和、数组长度、实例表均通过。总计人物特征 4300、其他实例特征 2309、背景特征 5482。
- 经 System 内部缩放至 480×360 的真实序列 5 帧，同样逐点验证通过：人物 1859、其他实例 957、背景 2221。
- 关闭分割与原 selected/person 屏蔽各 5 帧均正常跟踪；selected 屏蔽区域中最终特征中心违规数=0。关联模式与关闭分割前 5 帧特征数量逐帧相等：[1013,1008,1007,1009,1007]。
- Xvfb + Mesa 软件 OpenGL 中运行实际窗口：关联且 Debug/AssociationDebug 均关闭时 30 帧全部 OK，观察到 Map Viewer、Current Frame、Semantic Mask，捕获两张不同截图，彩色标签/关联画面随帧更新，无调试图片目录。Visualize=0 时 8 帧只出现原两个窗口；Run.Viewer=0 且移除 DISPLAY 时 5 帧正常、无窗口。各组正常退出。
- 已人工查看实际生成图与窗口截图：人物轮廓对齐、人物内部保留同实例颜色的 ORB 特征，背景特征为灰色。不是通过二值并集反推实例身份。

## 本机测量

| 模式 | 帧数 | 平均 TrackRGBD / ms | 跟踪状态 |
|---|---:|---:|---|
| association | 12 | 343.84 | 全部 OK |
| resized | 5 | 337.45 | 全部 OK |
| selected | 5 | 354.54 | 全部 OK |
| original | 5 | 10.37 | 全部 OK |

关联和 selected 行开启对应调试写盘；original 关闭推理。计时来自 RunFrames.csv，包括 TrackRGBD 内的同步推理、跟踪、关联及其调试写盘，不含读图和回放等待，不是纯关联耗时。短序列均状态 OK 不等于轨迹精度提高；这些结果也不表示达到 30 Hz。完整记录 `outputs/instance-association/summary.json` 与各模式日志/轨迹。

## 输出与限制

- 首帧可视化：`outputs/instance-association/association/debug/association_0_features.png`；对应 `_labels.png` 为 CV_16U 保存。
- CSV：同目录 `association_frames.csv`、`association_features.csv`、`association_instances.csv`。
- 实时窗口截图与 GUI 状态：`outputs/instance-association/live-view/viewer-1.png`、`viewer-2.png`、`gui-summary.json`，日志 `gui-test.log`。
- 控制测试与单帧关联输出：`outputs/instance-association/module`；单元执行日志 `module-test.log`。
- 构建未启用/未安装的可选硬件目标未测试；其他传感器已编译但本次未运行其数据集。未在用户物理桌面手动操作，GUI 验证使用虚拟显示器。
- 未做长序列精度、带非零畸变真实标注集的像素精度评估、运动判定或跨帧实例跟踪；本次查询明确使用 mvKeys，未混用 mvKeysUn。实例 ID 仅在当前帧内有效，模型误分割也会反映在标签中。

配置、完整编译及运行命令、文件作用、切回原配置方式见 [使用说明](../instance-association.md)。

## 逐帧终端统计增补

终端统计增补验证：全部已配置目标 -j1 编译通过（outputs/instance-association/statistics/build.log）。真实 TUM 3 帧输出全部六项统计及各实例 ID/类别/数量，与 CSV 逐项一致；关联前后特征数、标签长度及总数恒等式通过。运行日志 outputs/instance-association/statistics/run.log，CSV 在同目录 debug/。
