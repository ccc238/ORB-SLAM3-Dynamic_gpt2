# YOLO11-seg 同步在线语义区域屏蔽

本版本仅屏蔽当前 RGB-D 帧中的语义实例区域，不判断实际运动，不删除历史地图点，不修改地图维护算法。RGB-D 默认启用语义区域屏蔽；默认模式为 `selected`，默认仅屏蔽人（COCO class_id=0）。CPU 同步逐帧执行，无 GPU、队列或跳帧。

## 修改文件

| 文件 | 作用 |
|---|---|
| `include/SemanticMasker.h`, `src/SemanticMasker.cc` | 独立 ONNX Runtime 会话、配置解析、严格模型检查、letterbox、类别内 NMS、实例掩码与并集、调试输出 |
| `include/Tracking.h`, `src/Tracking.cc` | 持有一个可复用模块；RGB-D 当前帧缩放完成后、转灰度前同步推理，把有效区域传给 Frame；跟踪后检查关键点屏蔽不变量并记录 |
| `include/Frame.h`, `src/Frame.cc` | RGB-D 构造函数追加默认空 mask 参数；提取时传入 mask，深度、描述子和网格沿用原有顺序建立；零特征帧初始化安全哨兵 |
| `include/ORBextractor.h`, `src/ORBextractor.cc` | 真正使用有效区域 mask，在 FAST 候选进入八叉树前过滤，包括低阈值补充路径；描述子前补充边界检查；记录候选统计；修正窄图初始节点数和很小层网格除零问题 |
| `src/System.cc` | 未产生任何关键帧时，明确提示并保存空轨迹，避免全屏蔽短序列保存时越界 |
| `Examples/RGB-D/rgbd_tum.cc` | 保留原四个位置参数；新增可选输出目录、帧数上限及 `Run.Viewer`；检查关联文件；输出逐帧耗时与特征数；捕获推理错误并退出 |
| `CMakeLists.txt` | 可选 ONNX Runtime 依赖、可配置根目录和构建期动态库搜索路径；可选验证目标 |
| `config/TUM3-semantic-{off,all,selected}.yaml` | 三种直接使用的 TUM3 示例配置，匹配 OpenCV 的 BGR 输入 |
| `tests/semantic_mask_test.cc` | 真实模型和真实图像的分割、屏蔽、坐标、尺寸、零特征及兼容性验证 |
| `tools/run_semantic_validation.py` | 串行运行验证矩阵并输出汇总 |

## 依赖与编译

本机已有 GCC 11.4、OpenCV 4.5 系列、Eigen、Pangolin、DBoW2/g2o、CPU ONNX Runtime 1.17.3。使用现有 1.17.3，不升级或修改模型。实际版本以 `outputs/semantic-validation/configure.log` 为准。ORT 动态库的系统依赖通过 `ldd` 检查。ORT 1.17.3 的 C++ 头文件需要 C++14，仅 `SemanticMasker.cc` 使用 `-std=c++14`，原工程其余代码仍使用 C++11。

ONNX Runtime 安装目录应包含：

```text
<ORT_ROOT>/include/onnxruntime_cxx_api.h
<ORT_ROOT>/include/onnxruntime_c_api.h
<ORT_ROOT>/include/onnxruntime_cxx_inline.h
<ORT_ROOT>/include/onnxruntime_float16.h
<ORT_ROOT>/lib/libonnxruntime.so
<ORT_ROOT>/lib/libonnxruntime.so.1.17.3
```

可以将已有 CPU Linux x64 1.17.3 官方发行压缩包解压到自选目录：

```bash
mkdir -p third_party_ort
tar -xzf /path/to/onnxruntime-linux-x64-1.17.3.tgz -C third_party_ort
export ORT_ROOT="$PWD/third_party_ort/onnxruntime-linux-x64-1.17.3"
```

本机本次验证无需安装，使用已有安装目录：

```bash
cd /home/aaa/Documents/ORB-SLAM3_original_gpt2
export ORT_ROOT=/home/aaa/ORB-SLAM3_original0/third_party_ort/onnxruntime-linux-x64-1.17.3
cmake -S . -B build-semantic \
  -DCMAKE_BUILD_TYPE=Release \
  -DWITH_ONNX_RUNTIME=ON \
  -DONNXRUNTIME_ROOT="$ORT_ROOT" \
  -DBUILD_SEMANTIC_TESTS=ON
cmake --build build-semantic -j1
```

以上编译所有已配置目标，包括 RGB-D、新旧单目/双目/惯性示例。RealSense 示例只有在系统找到 SDK 时才会生成。已有 DBoW2 和 g2o 构建产物继续使用；如从全新源码构建，先按原 README 构建第三方库，每次均使用 `cmake --build <build-dir> -j1`，避免原 `build.sh` 中的并行构建。

CMake 在 `<ORT_ROOT>/include` 和 `<ORT_ROOT>/lib`、`lib64` 中查找；也可直接传入 `ONNXRUNTIME_INCLUDE_DIR`、`ONNXRUNTIME_LIBRARY`。库通过构建期 RPATH 查找 ORT；移动可执行文件/安装包后，请保留 ORT 库位置，或显式配置：

```bash
export LD_LIBRARY_PATH="$ORT_ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ldd lib/libORB_SLAM3.so
```

不需要 ORT 时可使用 `-DWITH_ONNX_RUNTIME=OFF`（构建选项默认 OFF），同时必须在 RGB-D YAML 中显式设置 `SemanticMask.Enabled: 0`。启用屏蔽而未编译 ORT 时明确抛出错误，不会退回无 mask。

## 配置

下列字段可添加到原相机配置中。未配置模型路径时默认查找 `$HOME/models/yolo/yolo11s-seg.onnx`，本机解析为 `/home/aaa/models/yolo/yolo11s-seg.onnx`。显式配置的相对路径相对于进程工作目录解析，建议使用绝对路径。

```yaml
# OpenCV imread 的三通道图像是 BGR。其他输入源根据真实通道顺序设置。
Camera.RGB: 0
Run.Viewer: 0
SemanticMask.Enabled: 1
SemanticMask.Model: "/home/aaa/models/yolo/yolo11s-seg.onnx"
SemanticMask.Mode: "selected"
SemanticMask.Classes: [0]
SemanticMask.Confidence: 0.25
SemanticMask.NmsIoU: 0.7
SemanticMask.MaxDetections: 300
SemanticMask.DilationRadius: 0
SemanticMask.Debug: 1
SemanticMask.OutputDirectory: "outputs/my-new-run/debug"
```

- `all` 屏蔽所有通过阈值和类别内 NMS 的实例；Classes 在该模式下不限制屏蔽。
- `selected` 仅合并 Classes 列表内的实例；多个 COCO class_id 均支持。未提供列表时明确提示并使用默认 `[0]`（person），显式空列表报错。需要多类时，例如 `[0,2]` 表示 person 和 car。
- Confidence 和 NmsIoU 范围为 [0,1]，MaxDetections 为 1–8400，DilationRadius 为 0–640。
- DilationRadius 是 **当前实际 SLAM 图像坐标**的像素半径，0 关闭；非零使用 `(2r+1) × (2r+1)` 正方形核，在实例并集合并后膨胀。
- Debug 关闭时仍执行最终关键点与 mask 的一致性检查；不写图和 CSV。Debug 打开才写逐帧检测数、特征数、像素比例和耗时。每次创建会话重写该目录的 frames.csv；保留旧实验请使用新的目录。
- TUM 示例会按 Camera.RGB 将 imread 的 BGR/BGRA 转为所需通道顺序，再送给 SLAM 和语义模块。新示例配置使用 BGR=0；原 TUM3 配置 RGB=1 也能正确运行，原配置文件保持不变。
- 不要在同一个 YAML 中重复添加同名键；修改已有键值。重跑请改用新的输出目录。

## 启动

从工程根目录执行，模型和数据集只读。以下三个输出目录互相独立：

```bash
DATA=/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz

# 关闭语义屏蔽
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt config/TUM3-semantic-off.yaml \
  "$DATA" "$DATA/associate.txt" outputs/off 30 > outputs/off.log 2>&1

# 开启 all
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt config/TUM3-semantic-all.yaml \
  "$DATA" "$DATA/associate.txt" outputs/all 30 > outputs/all.log 2>&1

# 开启 selected，默认仅选择 person
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt config/TUM3-semantic-selected.yaml \
  "$DATA" "$DATA/associate.txt" outputs/selected 30 > outputs/selected.log 2>&1
```

可选参数是 `[output_directory] [max_frames]`；去掉 30 就处理整个序列。输出根目录 `outputs` 若不存在先创建。图像输出目录取自 YAML，不会根据命令行的轨迹目录自动改变，运行新实验时同时修改 `SemanticMask.OutputDirectory`。

原始四参数形式仍可用，默认使用当前目录保存轨迹和 RunFrames.csv：

```bash
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml \
  "$DATA" "$DATA/associate.txt"
```

该原配置没有 SemanticMask 字段，现在会默认启用 selected/person 屏蔽；模型从上述 HOME 路径加载，viewer 默认开启。需要原版无屏蔽行为时，在配置副本中显式设置 `SemanticMask.Enabled: 0`。其他传感器模式仍默认关闭语义屏蔽。无显示环境请在配置副本中加 `Run.Viewer: 0`。

## 实现与坐标约定

1. `System::TrackRGBD` 先克隆输入，再执行配置的内部缩放；`Tracking::GrabImageRGBD` 接收到最终图像才推理。RGB、深度、结果与传入时间戳处于同一个同步调用中。模块不修改 RGB 或深度。
2. 遵循 Tracking 的 `mbRGB`，仅在独立预处理缓冲中转 RGB；单通道扩展为 RGB，四通道按 RGBA/BGRA 转换。缩放后图像以 114 补边到 640×640，记录实际整数缩放尺寸与四侧补边，转换为 float32 NCHW /255。
3. 初始化严格检查唯一输入 `images float32 [1,3,640,640]` 和两个输出 `output0 float32 [1,116,8400]`、`output1 float32 [1,32,160,160]`。每帧再次检查输出形状、类型和非有限值。输入向量在同步 Run 完成前存活，输出 Ort::Value 在所有读取和后处理完成前存活，名称使用固定字符串或分配器所有权对象，无临时字符串指针。
4. output0 按通道优先索引 `channel*8400+candidate`；最高 80 类分数就是置信度，无 objectness。使用模型输入坐标的未裁剪框执行类别内 NMS，再限制最大检测数。COCO 80 类表在源码中明确列出。
5. 32 个系数乘 32 个原型得到 logits，先 sigmoid，再把连续概率双线性插值到 640×640，去掉记录的整数 letterbox 补边，双线性恢复当前 SLAM 图像尺寸，最后以 **probability > 0.5** 二值化，按原图浮点检测框进行半开区间像素中心裁剪。缩放回原图使用实际整数缩放后的 x/y 比例，避免舍入误差。此版本不再缩放二值 mask；若后续增加该操作，必须使用最近邻。
6. 返回所有 NMS 后实例的原图框、类别、置信度和全尺寸实例 mask；blocked 是根据 all/selected 策略合并再可选膨胀的并集。有效区域为 `bitwise_not(blocked)`，255 允许提取、0 排除。
7. FAST 的局部候选坐标先加 cell 原点（含 `EDGE_THRESHOLD-3`），乘该层最终关键点使用的缩放比例，再以 floor 访问原图 mask。图像金字塔 ROI 不包含分配的外侧 border，无须额外加 19 像素。被屏蔽候选在八叉树分配前去除，不占用特征配额。被屏蔽后 cell 为空时，低阈值 FAST 也执行相同过滤。
8. 在描述子分配、深度关联和网格建立前额外检查一次按最终坐标计算的中心，防止浮点运算结合顺序的边界差异；因此不需要提取后重排描述子、深度和索引。未启用 RGB-D mask 的其他构造函数和原 extractor 调用签名保持兼容。
9. 跟踪后的 invariant 检查使用原始 `mvKeys` 图像坐标，不使用去畸变后的 `mvKeysUn`；屏蔽发生在原始特征提取图像中。特征不足保持 mask，记录状态，走原版初始化/跟踪失败流程。

## 日志、图像与统计定义

实际本次验证结果写入 `outputs/semantic-validation/`：

- `configure.log`, `build.log`, `build-final.log`, `build-config-validation.log`：配置与单线程编译。
- `module-test.log`, `module-debug/`：单图/尺寸/空检测等模块验证及叠加图。
- `{off,all,selected}.log`、对应子目录中的 `CameraTrajectory.txt`、`KeyFrameTrajectory.txt`、`RunFrames.csv`：同一段真实序列对比。
- `{all,selected}/debug/frames.csv`：逐帧检测、mask 比例、关键点与阶段耗时。
- `N_regions.png`：红色语义屏蔽叠加，黄色检测框仅供显示，不代表整框屏蔽。
- `N_keypoints.png`：在叠加图上用绿色绘制最终关键点。
- `N_blocked.png`：精确二值屏蔽区域，便于逐像素审计。
- `resized/`, `empty/`, `fully-blocked/`, `transition/`, `legacy/`：内部缩放、空检测、全屏蔽、已初始化后跟踪失败及旧启动参数验证。
- `summary.json`：运行后生成实测汇总；`outputs/semantic-validation/RESULTS.md`：验证结论和限制。

`instances` 是所有类别 NMS 后实例数，selected 模式中它包含未被选中屏蔽的实例；`blocked_ratio` 是最终并集（包含膨胀）非零像素比例。`candidates` 是全部金字塔层、cell 和高/低 FAST 阈值路径中提交检查的中心数量，cell 重叠可重复计数；`rejected` 是其中因 mask 被拒绝的候选数，不是最终特征配额前后的唯一点数量。`keypoints` 是最终保留数量，`violations` 必须为 0。

`pre_ms` 包括颜色、letterbox、NCHW；`infer_ms` 是同步 Ort::Session::Run；`post_ms` 包括输出验证、NMS、mask 重建及膨胀；debug CSV 中的 `frame_ms` 从 GrabImageRGBD 开始到 Track 返回，尚未包含调试图片写盘；`RunFrames.csv` 的 `track_ms` 包含完整 TrackRGBD（含 System 缩放和调试输出），不包含读图和回放等待。主程序给出的 mean/median tracking time 采用后者。日志数值用于本机实测，不表示实时保证或轨迹精度提升。

## 复现验证

```bash
python3 tools/run_semantic_validation.py \
  /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz
```

串行运行，不修改原模型、数据集和关联文件；在输出目录生成前 30 帧关联副本。已存在的输出会被重新写入，请先换目录或备份。实际通过项和未覆盖项请看 `outputs/semantic-validation/RESULTS.md`。

默认策略调整后的验证输出位于 `outputs/semantic-person-default/`。`outputs/semantic-validation/RESULTS.md` 中 selected `[0,2]` 是调整前的历史实测配置；当前示例 selected 配置为 `[0]`。
