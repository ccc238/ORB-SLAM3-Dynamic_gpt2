# 动态过滤显示与数量统计

显示复用 `FrameDrawer::DrawFrame` → Viewer 现有的 `ORB-SLAM3: Current Frame` 窗口，仅为纯 RGB-D 发布新的完整显示快照。其他传感器保持原显示路径。未新建 SLAM 程序，未改候选交集规则、安全阈值、优化求解或地图排除规则。

RGB 原图和 `Frame::mvKeys` 同时保存；不使用去畸变坐标 `mvKeysUn` 覆盖原图。灰点表示非候选，不能解读成确认静态；黄圈表示保守候选；红叉只表示接受过滤且确实从最终位姿优化构边集合中排除的观测。原有地图点绿色/VO 蓝色图例不会叠加到这个视图。原分割实例窗口可独立保留。

## 调用与统计口径

1. RGB-D 预处理、分割和原始索引关联；对极/LK 两帧证据。
2. 原 Tracking 初步跟踪、TrackLocalMap 和 PoseOptimization。优化器在实际添加边时记录原始特征索引；另记录该帧是否完成优化以及最后的优化内点索引。
3. 地图提交前的原有动态过滤钩子：冻结初步建边索引作为资格快照，按原规则生成候选、检查安全条件，启用后至多额外优化一次；回退时连同原有算法状态恢复优化测量元数据。
4. 生成只读显示结果：before = 冻结资格索引；after = before 减去接受的排除掩码。程序断言 after 与真实最终构边索引完全相同。内点数来自优化器最后的外点分类，单独计数，不使用 Tracking 的地图支持数冒充优化内点数。
5. 原流程继续运动模型、关键帧与新地图点提交、上一帧/轨迹缓存。GrabImageRGBD 将之前冻结的统计与本次 RGB 图像整体发布。Viewer 可显示上一份完整快照，但不会拼接不同帧的图像、点或数量。

图像和结果由 `mDynamicMutex` 一次性发布/复制，显示线程不读取正在变化的 Frame、MapPoint 或 Atlas。CSV 写入与保存图片由另一个写锁串行化。绘制完成后整体缩放，RGB 点位与图像使用同一缩放。

`orb_total` 永远为原始特征数组长度；没有删除、压缩或重排。`actual_excluded` 只计 before 中被接受排除的位置；有候选但本来没有建边资格的位置不会产生红叉。候选在地图提交接口上的禁止位置可能比这个数多，旧过滤审计中的 `effective_excluded` 表示该地图许可掩码，不能与新视图的实际优化排除观测数混用。

无优化构边测量时，before/after 为 N/A；已经构边但求解无结果时，边数仍可测、最终内点为 N/A。初始化通过 RGB-D 深度直接创建地图的帧不冒称完成位姿优化。回退的最终内点和边索引取恢复后的初步结果，失败的尝试不当作最终结果。

## 配置与运行

模块开关集中放在 [TUM3-instance-association.yaml](../config/TUM3-instance-association.yaml) 中：语义分割/实例关联及各诊断按各自开关运行；动态过滤仅由总开关控制，`0` 开启真实过滤，`1` 或缺失关闭。旧 `Dynamic.FilterMode` 已废弃，残留会警告且不会生效。运行时无需再传模块开关，但命令行仍需提供词典、YAML、数据集和关联文件路径。

```yaml
Dynamic.UseDynamicFilter: 0

Run.Viewer: 1
# 独立输出目录；每次实验指定不同目录，避免覆盖
Dynamic.FilterViewOutputDirectory: "outputs/my-run/filter-view"
# 每 10 帧保存，候选帧和无优化结果帧也保存；0 关闭图片保存
Dynamic.FilterViewSaveEvery: 10
```

缺省输出目录为 `outputs/dynamic-filter-view`，缺省保存间隔为 10；空目录字符串关闭 CSV/图片，实时窗口仍显示。非法目录/保存间隔类型会报错。中文显示使用本机 OpenCV freetype 模块，以及 DroidSansFallbackFull（中文）和 DejaVuSans（数字/英文）字体。Droid 路径为 `/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf`，DejaVu 路径为 `/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf`。

```bash
cmake --build build-semantic --target dynamic_filter_test rgbd_tum -j1
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt /tmp/settings.yaml \
  /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz \
  /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/associate.txt \
  outputs/my-run 120
python3 tools/validate_dynamic_filter_view.py --case all --frames 120 --output outputs/my-view-validation
```

验证脚本只生成临时 YAML 并启动现有 rgbd_tum，默认关闭 GUI，以同一个 FrameDrawer 绘制函数保存代表帧；加 `--viewer` 可同时显示原窗口。rollback/safety 是强制提高支持门限的测试用例，不代表默认门限在正常序列触发了回退。

## 输出

- `frames.csv`：frame_id、timestamp、enabled、orb_total、candidates、observations_before、observations_after、actual_excluded、final_inliers、applied、executed、status、reason、optimization_available。
- `observations.csv`：每帧每个原始 index 的原图坐标、候选、实际优化排除、before/after 资格、真实 final_edge、最终 final_inlier。无测量的字段输出 N/A。
- `frame_<id>_<timestamp>.png`：与现有窗口使用同一绘制函数的 RGB 帧，包含中文状态及图例。

状态区分别显示“过滤已接受，实际排除=n”“未应用，实际排除=0”或“已回退，实际排除=0”。关闭独立显示“过滤关闭”。原因保留机器可检索的原始代码。

新增界面数量的核对以 `Dynamic.FilterViewOutputDirectory` 中的 CSV 为准。原 `Dynamic.FilterOutputDirectory` 保留旧审计格式，其内点字段属于原地图支持计数，不替代新视图中直接取自优化器的最终内点测量。使用 `--capture-window` 可连接 X11 捕获本程序的 Current Frame 窗口，脚本要求对应分支确实出现，并逐像素核对窗口内容与同帧 FrameDrawer 输出。

本机实测、窗口截图和验证限制见 [验证报告](validation/dynamic-filter-view-results.md)。

当前版本没有模式选择或 simulate 路径。验证驱动只设置总开关；on/rollback/safety 是测试场景名称，不是运行模式。新结果见 [单开关验证](validation/dynamic-filter-switch-results.md)。
