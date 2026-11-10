# 动态模块消融开关

## 现有实现与本次范围

本次保留 YOLO11s-seg 与关联实现，仅修改现有配置入口。修改前快照在 `outputs/dynamic-switch/pre-change/source.tar.gz`，同目录保存当时的 `rgbd_tum`、`libORB_SLAM3.so`。当前根目录 `.git` 为空，不能直接创建普通 Git 提交，因此使用备份。

- `SemanticMasker::Impl` 初始化 CPU ONNX Runtime Session；`SemanticMasker::Infer` 做现有 letterbox、推理、NMS、Mask 恢复、标签构建。
- `System::TrackRGBD` 完成内部图像处理后进入 `Tracking::GrabImageRGBD`。后者接收 `SemanticResult`，构造 RGB-D Frame 后、`Track()` 前调用 `Frame::AssociateInstances`。
- `Frame::AssociateInstances` 使用 `QueryInstanceLabels` 查询原始 `mvKeys` 对应标签，只写附加实例字段。ID 顺序、floor 像素查询、标签坐标、描述子及特征索引保持不变。
- 原有代码已扩展 RGB-D Frame 的有效区域参数和 ORBextractor 的候选点 Mask 过滤。旧屏蔽模式确实会改变用于 Tracking、匹配及建图的输入；本次在线配置始终传空有效区域，不触发过滤。
- 匹配器、`Tracking::Track`、优化器及 Local Mapping 算法本次未修改。关闭时不创建 SemanticMasker，不加载模型，不推理、不关联、不显示语义窗口。Frame 中此前已添加的实例字段保留，关闭时仍按已有逻辑初始化为 -1；不改变原有特征、描述子、深度、地图点数组或优化输入。

## 配置及优先级

复用 `SemanticConfig::Read` 的 OpenCV FileStorage 机制，直接读取相机 YAML 中的平面键名：

```yaml
Dynamic.Enable: 0
Dynamic.MaskOnly: 1
```

| 配置项 | 默认值 | 含义 |
|---|---|---|
| Dynamic.Enable | 0 | 总开关，0 不构造动态模块，1 执行现有分割及关联 |
| Dynamic.MaskOnly | 1 | 1 仅分割、关联及按原配置可视化；0 为几何检测预留，目前同样不剔除特征 |

两个键仅接受整数 0/1。开关决策集中在 `SemanticConfig::Read`，不在各处理函数中散布判断。此阶段 `MaskOnly=0` 会打印预留提示，没有新增几何算法。

`Dynamic.*` 优先于历史开关。`SemanticMask.Enabled`、`SemanticMask.MaskingEnabled`、`SemanticMask.AssociationEnabled` 在在线 YAML 路径中不再生效；即使旧配置把它们设为 1，缺少 Dynamic.Enable 仍关闭。显式开启 Dynamic.Enable 时，始终解析为 enabled=true、associationEnabled=true、maskingEnabled=false，避免旧配置意外剔除特征。旧过滤代码保留供历史模块测试，未删除或重写。

模型路径、置信度、NMS、类别、膨胀等仍从现有 `SemanticMask.*` 参数读取，标签构建不受类别屏蔽策略影响。`SemanticMask.Visualize` 仍控制语义窗口（还需 Run.Viewer=1），`SemanticMask.Debug` 和 `SemanticMask.AssociationDebug` 仍控制原调试输出，总开关关闭时全部不执行。

示例 `config/TUM3-instance-association.yaml` 已显式设置 Enable=1、MaskOnly=1。`config/TUM3-semantic-off.yaml` 显式设置 Enable=0、MaskOnly=1。其余旧配置未设置 Dynamic.Enable，因此均关闭动态模块；旧 selected/all 文件名不再代表在线过滤模式。

调用链：`System` → `Tracking` 构造 → `SemanticConfig::Read` 统一解析 → Enable=1 时构造 `SemanticMasker` → `GrabImageRGBD` 推理 → 空有效区域提取 ORB → `Frame::AssociateInstances` → 原 `Track()` → 原日志/可视化。Enable=0 则跳过全部动态调用。

## 编译与复现

```bash
cmake --build build-semantic -j1
./build-semantic/semantic_mask_test /home/aaa/models/yolo/yolo11s-seg.onnx /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/rgb/1341846313.592026.png outputs/dynamic-switch/module
python3 tools/validate_dynamic_switches.py
```

新增验证脚本使用现有 `rgbd_tum`，默认对 TUM walking_xyz 前 100 帧执行 A/B 和备份基线；另测缺失配置、MaskOnly=0、内部缩放各 5 帧。不重写 SLAM 执行程序。输出在 `outputs/dynamic-switch/`，调试目录已存在时脚本会拒绝覆盖，需要先保留旧输出再重新执行。旧 `run_semantic_validation.py`、`validate_instance_association.py` 中的历史在线屏蔽测试不适用于当前开关约定，当前入口为新脚本。

手动运行（将 YAML 中 Dynamic.Enable 改为 0 即测试 A）：

```bash
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt config/TUM3-instance-association.yaml /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/associate.txt outputs/dynamic-switch/manual 100
```

实际编译、运行和窗口验收结果见 [本次验收记录](validation/dynamic-switch-results.md)。短序列开关验收不评价长序列精度，其他传感器仅进行编译回归。
