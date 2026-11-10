# 动态消融开关验收（2026-11-10）

本次在已有 YOLO11s-seg 和 ORB 实例关联实现上补充开关，没有重新实现模型或关联算法。工作区 `.git` 为空目录，使用源码归档和二进制备份保存修改前版本：`outputs/dynamic-switch/pre-change/`；包含 SHA256SUMS。备份基线运行通过 LD_LIBRARY_PATH 明确加载备份 libORB_SLAM3.so，加载路径记录在该目录 ldd.txt。

## 实际编译与运行

`cmake --build build-semantic -j1` 返回 0，全部已配置目标编译通过，包括原有/当前 RGB-D、单目、双目、惯性示例和 semantic_mask_test。已有编译警告仍存在，无编译错误。环境为 GCC 11.4、OpenCV 4.5.4、CPU ONNX Runtime 1.17.3。日志：`outputs/dynamic-switch/build.log`。

实际使用 `/home/aaa/models/yolo/yolo11s-seg.onnx` 和 TUM `rgbd_dataset_freiburg3_walking_xyz`，执行 `python3 tools/validate_dynamic_switches.py` 返回 0：

| 模式 | 帧数 | Tracking 状态 | 有限且有效的相机轨迹行数 |
|---|---:|---|---:|
| 修改前关闭分割基线 | 100 | 全部 OK | 100 |
| A：Enable=0 | 100 | 全部 OK | 100 |
| B：Enable=1，MaskOnly=1 | 100 | 全部 OK | 100 |
| 缺少 Dynamic 参数 | 5 | 全部 OK | 5 |
| Enable=1，MaskOnly=0 | 5 | 全部 OK | 5 |
| B：内部缩放至 480×360 | 5 | 全部 OK | 5 |

- A 和缺省关闭测试故意设置不存在的 ONNX 路径，并将旧 Enabled/MaskingEnabled/AssociationEnabled/AssociationDebug 设置为 1，仍成功输出相机与关键帧轨迹；没有模型 tensor 初始化日志、关联日志或动态调试目录。表明新总开关优先，未加载模型或执行关联。
- B 每帧均有真实 ONNX 推理耗时、实例关联记录。100 帧共 100,683 个特征：实例内 47,861，背景 52,822。逐特征索引、floor(u,v) 标签 PNG 查询、实例表 ID 和每实例计数均相符。关联前后特征数与标签数组长度逐帧相等，拒绝数和违规数均为 0。
- A 和 B 的 100 帧逐帧特征数量均与备份版本关闭分割的基线完全相等。缺省关闭和 MaskOnly=0 的前 5 帧也与基线一致。此阶段 MaskOnly=0 仅打印几何检测预留提示，没有动态剔除。
- 480×360 内部缩放的 5 帧标签尺寸、逐点查询、实例索引与特征数量检查全部通过；未改变缩放和坐标约定。
- 相机轨迹均逐行检查 8 列、数值有限、行数与帧数相等；关键帧轨迹非空。轨迹与逐帧状态在各模式输出目录，汇总为 `outputs/dynamic-switch/summary.json`，执行日志为 `validation.log`。

## 模块回归

以下命令返回 0，日志 `outputs/dynamic-switch/module-test.log`：

```bash
./build-semantic/semantic_mask_test /home/aaa/models/yolo/yolo11s-seg.onnx /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/rgb/1341846313.592026.png outputs/dynamic-switch/module
```

首帧实际检测 11 个实例，ORB 特征 1013 个，关联后仍 1013 个，背景 462、实例 551、无效 0。关联前后逐项检查特征位置、顺序、描述子、深度、等效右目坐标及地图点关联数组不变。复制、空检测、重叠优先级、floor 查询、越界及非有限坐标、尺寸与类型校验、标签不受类别选择与膨胀影响等原有回归均通过。新增检查包括配置缺省关闭、四种 Enable/MaskOnly 组合、旧开关覆盖保护、非法开关值拒绝。

## 实际窗口验收

沙箱内 xwininfo 不能访问虚拟显示；授权后在沙箱外使用 Xvfb 和 Mesa 软件 OpenGL 执行：

```bash
env SEMANTIC_VIEW_OUT=/home/aaa/Documents/ORB-SLAM3_original_gpt2/outputs/dynamic-switch/live-view LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a python3 tools/verify_semantic_viewer.py /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz association
```

- B 且可视化开启：30 帧全部 OK，实际存在 Map Viewer、Current Frame、Semantic Mask 三个窗口；两张截图像素不同，画面随帧更新。
- Visualize=0：8 帧全部 OK，仅保留两个原有 SLAM 窗口。
- Run.Viewer=0 且移除 DISPLAY：5 帧全部 OK，无窗口。
- Dynamic.Enable=0：8 帧全部 OK，仅保留两个原有 SLAM 窗口，无语义窗口。
- 所有进程正常退出，关闭 Debug/AssociationDebug 时未生成调试图片目录。已查看实际窗口截图，人物边界与标签对齐，人物内部 ORB 特征保留并按实例着色，背景为灰色。

证据：`outputs/dynamic-switch/gui-test.log`、`live-view/gui-summary.json`、`live-view/viewer-1.png` 和 `viewer-2.png`。

## 对原流程的影响与边界

Tracking.cc 本次仅修改 SemanticConfig::Read 的调用参数。`Tracking::Track`、匹配器、优化器、Local Mapping、Frame、ORBextractor、模型推理函数和可视化实现与备份逐文件/函数比较未变化。两个开关统一在原配置加载入口解析，关闭时不创建模块，开启时强制空有效区域并执行原有关联。此前已加入的 Frame 实例字段仍保留；关闭时维持原有 -1 初始化，未触动原有特征及地图点索引结构。

在线旧 SemanticMask.Enabled/MaskingEnabled/AssociationEnabled 已由 Dynamic.* 取代；不带新总开关的旧配置默认关闭动态模块。旧过滤代码仍保留用于历史模块回归，当前在线配置不会启用它。

本次验证对象是本项目修改前关闭分割的可运行基线，未声称与独立上游官方版本逐位一致。B 的同步推理增加运行耗时，线程调度也可能导致轨迹数值差异；特征输入不变不代表轨迹逐位一致。运行部分与编译/窗口测试存在时间重叠，计时不作为性能基准。未进行长序列精度消融，也未运行其他传感器数据集。

## 实际修改文件

| 文件 | 修改内容 |
|---|---|
| include/SemanticMasker.h | 默认关闭，保存 MaskOnly 状态，统一读取接口 |
| src/SemanticMasker.cc | 读取两个 Dynamic 参数，统一决定推理、关联、过滤及调试状态 |
| src/Tracking.cc | 同步现有配置读取接口调用 |
| config/TUM3-instance-association.yaml | 显式 Enable=1、MaskOnly=1 |
| config/TUM3-semantic-off.yaml | 显式 Enable=0、MaskOnly=1 |
| tests/semantic_mask_test.cc | 更新缺省行为与配置优先级回归，保留原模型/关联检查 |
| tools/validate_dynamic_switches.py（新增） | 真实序列 A/B、备份基线、缺省、预留模式与缩放检查 |
| tools/verify_semantic_viewer.py | 使用新总开关，增加关闭动态模块的窗口检查，允许独立输出目录 |
| README.md | 更新当前开关行为与文档入口 |
| docs/instance-association.md | 标注历史配置语义，指向当前文档 |
| docs/semantic-region-masking.md | 标注历史配置语义，指向当前文档 |
| docs/dynamic-switches.md（新增） | 配置默认值、优先级、调用链与复现命令 |
| docs/validation/dynamic-switch-results.md（新增） | 本次实测结果与文件清单 |
