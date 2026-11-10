# 第 4 周第一小步：ORB 特征与本帧实例标签关联

> 配置更新（2026-11-10）：在线总开关已改为 `Dynamic.Enable`，缺省为 0；开启后关联且不屏蔽特征。本文的旧开关、历史屏蔽模式和旧验证脚本描述保留为历史记录，当前用法与验收以 [动态消融开关](dynamic-switches.md) 为准。

此功能只记录语义归属，不判定运动、不删除特征、不修改历史地图点，不建立跨帧 ID。背景标签不表示已验证静态点。

原有屏蔽路径为 `System::TrackRGBD`（先完成内部图像缩放）→ `Tracking::GrabImageRGBD` 同步推理 → 原图大小的实例二值 Mask → selected/all 并集及可选膨胀 → 取反为有效区域 → RGB-D Frame → ORBextractor。在 ORB 候选点进入八叉树分配前过滤，低阈值补充候选也过滤；在方向和描述子计算前再次检查。未修改这些过滤规则。

现在将分割、屏蔽和关联分别控制。旧配置默认保持推理开启、selected/person 屏蔽开启、关联关闭；其他传感器仍默认关闭分割。新配置 `config/TUM3-instance-association.yaml` 开启推理和关联，关闭屏蔽，使 ORB 收到空有效区域参数，正常提取全部区域的特征。

| 配置项 | 默认值 | 作用 |
|---|---|---|
| SemanticMask.Enabled | RGB-D 为 1 | 在线分割总开关 |
| SemanticMask.MaskingEnabled | 1 | 只控制特征屏蔽 |
| SemanticMask.AssociationEnabled | 0 | 记录 Frame 实例归属；要求 Enabled=1 |
| SemanticMask.AssociationDebug | 0 | 保存关联图、16 位标签图和 CSV；要求关联开启 |
| SemanticMask.Visualize | 1 | 在 Run.Viewer=1 时显示实时语义窗口 |

已有 Model、Mode、Classes、Confidence、NmsIoU、MaxDetections、DilationRadius、Debug、OutputDirectory 保持原义。实验仍保留 selected/person 设置，但屏蔽开关关闭时不参与特征筛选。启动日志打印三个实际状态。不合法的关联配置、标签类型/尺寸、实例表或推理错误明确抛出异常。合法空检测生成全零标签图。

分割结果既保留每个 NMS 实例的原始二值 Mask，又提供 `CV_16UC1 instanceLabels` 和 `instanceInfo`。标签图包含全部保留类别，背景为 0，本帧 ID 从 1 开始，对应实例信息表的 ID、COCO 类别、名称、置信度。重叠像素取最高置信度实例，平分时保留原先顺序。标签图在屏蔽膨胀前由实例 Mask 生成，因此不受 selected 类别和膨胀影响。

`Frame::AssociateInstances` 在提取、深度和网格建立完成后、Track 前执行；只写 `mvInstanceIds`、`mvInstanceInfo` 和关联状态。数组逐项对应原来的 `mvKeys`、描述子、深度及地图点索引。各构造路径禁用关联时保留 N 个 -1，默认空 Frame 为 N=0、空数组。复制构造和赋值保留关联，向量独立持有，不保存悬空 Mask 引用。未增加 KeyFrame 或 MapPoint 字段。

查询使用当前 SLAM RGB 图像坐标中的原始 `mvKeys`，不用去畸变后的 `mvKeysUn`。先检查有限性和半开区间边界，再按 `floor(u), floor(v)` 读取 `M.at<uint16_t>(v,u)`；无效点为 -1，不钳制。模型 letterbox 在实例 Mask 恢复时已移除；分割位于 System 缩放之后，因此不需再次缩放标签图。显示缩放使用最近邻插值。

## 编译与启动

在工程根目录执行，所有编译均单线程：

```bash
cmake -S . -B build-semantic -DWITH_ONNX_RUNTIME=ON -DBUILD_SEMANTIC_TESTS=ON -DONNXRUNTIME_ROOT=/home/aaa/ORB-SLAM3_original0/third_party_ort/onnxruntime-linux-x64-1.17.3 -DCMAKE_BUILD_TYPE=Release
cmake --build build-semantic -j1

# 实验：在线推理 + 关联，关闭语义特征屏蔽；实时窗口显示实例边界与彩色特征。
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt config/TUM3-instance-association.yaml /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/associate.txt outputs/instance-association/manual 100

# 切回原有 selected/person 屏蔽行为（关联默认关闭）
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt config/TUM3-semantic-selected.yaml /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/associate.txt outputs/instance-association/person 100
```

ONNX Runtime 路径是本机已安装位置，可替换为其他安装根目录；头文件需在 `include`、动态库在 `lib`。版本和原分割环境一致为 CPU 1.17.3。默认模型路径为 `$HOME/models/yolo/yolo11s-seg.onnx`，也可设置 SemanticMask.Model。完整依赖说明见 [语义屏蔽文档](semantic-region-masking.md)。

## 输出与复现验证

默认实验配置不逐帧写调试文件。将实验配置复制到输出目录，设置 `SemanticMask.AssociationDebug: 1` 可启用写盘；无显示环境还应设置 `Run.Viewer: 0`。输出由 SemanticMask.OutputDirectory 控制，默认 `outputs/instance-association/debug`：

- `association_<frame_id>_features.png`：原始 RGB、实例边界、背景灰色特征、实例同色特征、无效紫色特征；左上列出本帧 ID、类别及特征数量。非有限或越界点无法在图内绘制，其数量仍记录。
- `association_<frame_id>_labels.png`：无损 16 位整数标签，背景 0。
- `association_frames.csv`：帧号、时间戳、总数、背景/实例/无效数量、关联前后数量、坐标空间。
- `association_features.csv`：帧号、时间戳、原始特征索引、u、v、实例 ID、坐标空间。
- `association_instances.csv`：帧号、时间戳、ID、类别、名称、置信度、该实例的特征数。

CSV 坐标空间 `slam_rgb_raw_pixels` 是当前 SLAM 输入图像（完成内部缩放、未经去畸变），像素查询采用 floor。实时语义窗口左侧为关联叠加，右侧为彩色实例标签，黑色表示背景。

```bash
./build-semantic/semantic_mask_test /home/aaa/models/yolo/yolo11s-seg.onnx /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/rgb/1341846313.592026.png outputs/instance-association/module
python3 tools/validate_instance_association.py
```

测量结果及实际执行范围见 [验收记录](validation/instance-association-results.md)。本步结束后不推进几何残差、动态剔除或跨帧实例追踪。

## 本步修改文件

| 文件 | 作用 |
|---|---|
| include/SemanticMasker.h、src/SemanticMasker.cc | 三个控制状态；全部实例标签、信息表、重叠处理、查询和关联调试输出 |
| include/Frame.h、src/Frame.cc | 按原特征索引保存 ID/信息表，构造与复制保留状态 |
| src/Tracking.cc | 控制有效区域传递，在 Track 前执行当前帧关联 |
| include/FrameDrawer.h、src/FrameDrawer.cc | 扩展已有实时窗口，显示实例边界、按实例着色的特征及彩色标签 |
| config/TUM3-instance-association.yaml | 独立实验配置，不改变 selected/person 旧配置 |
| tests/semantic_mask_test.cc | 在现有测试目标内增加关联、复制、边界和真实图像检查 |
| tools/validate_instance_association.py | 复用 rgbd_tum 跑真实序列并逐项核对 CSV 与标签 PNG |
| tools/verify_semantic_viewer.py | 原实时窗口验证脚本增加 association 参数 |
| README.md、本说明、验收记录 | 配置、运行命令及实测证据 |

本次没有新增 SLAM 可执行程序，也没有修改 KeyFrame、MapPoint、地图维护或动态判断算法。当前工作区保留了上一次实时掩膜窗口改动（Viewer/System 等）；上述表格列出的是本次关联工作。

## 手动运行时的终端统计

启用实例关联后，每帧输出一行 `[InstanceAssociation]`，不依赖 AssociationDebug：

```text
[InstanceAssociation] frame_id=0 timestamp=1341846313.592026 关联前特征数=1013 关联后特征数=1013 实例标签数组长度=1013 背景特征数=462 无效关联数=0 各实例特征数=[ID=1(class_id=0:person):110, ID=2(class_id=0:person):251, ...]
```

各实例包含全部本帧保留实例，即使该实例关联特征数为 0，也会列出。关联前后数量在调用前后分别读取 Frame.N；运行时检查前后数量不变、标签数组长度=N，以及背景+各实例+无效=N。合法空检测显示空实例列表。禁用关联不输出此行。AssociationDebug=1 时帧级 CSV 另保存 `instance_labels_length` 列，各实例数量保存在 association_instances.csv。
