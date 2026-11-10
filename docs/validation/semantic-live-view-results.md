# 实时语义掩膜窗口验证（2026-11-09）

已在 C++ RGB-D 流程中增加 `ORB-SLAM3: Semantic Mask` 实时窗口，显示 RGB/红色屏蔽区域/绿色保留特征，以及同帧的黑白掩膜。selected 默认仍仅屏蔽人（class_id=0）。

- 全部已配置目标使用 `cmake --build build-semantic -j1` 构建通过；日志 `outputs/semantic-live-view/build.log`。
- 原有真实模型分割回归通过，新增预览测试通过：右侧二值掩膜逐像素等于实际 blocked mask，RGB/BGR 渲染一致，读取快照不会修改共享数据，显示缩放使用最近邻；空检测右侧全黑，关闭后不保留旧图。测试在未设置 DISPLAY 的环境执行。
- 在 Xvfb 1920×1080 虚拟显示器、Mesa 软件 OpenGL 中实测真实 TUM walking_xyz 数据。不是仅生成静态图，使用实际 Pangolin/OpenCV 窗口运行并通过 X11 观察。
- 开启 Viewer+Visualize、关闭 Debug：30 帧全部跟踪 OK，同时观察到 Map Viewer、Current Frame、Semantic Mask 三个窗口，保存两张不同的窗口截图，确认随帧更新；无调试图片目录生成。
- 开启 Viewer、关闭 Visualize：8 帧全部 OK，只有地图与跟踪帧窗口，仍执行语义屏蔽。
- Run.Viewer=0 且移除 DISPLAY：5 帧全部 OK，无任何窗口；Visualize=1 不会破坏无界面运行。
- 三组均正常退出，Viewer 关闭窗口后退出，未出现自我 join 或卡住。
- 本机虚拟显示测试平均 TrackRGBD 耗时约 352.38 ms，包含分割、跟踪与预览准备，不包含读图或回放等待。窗口显示最新已完成帧，不声称达到 30 Hz，也未评价轨迹精度。

原始记录：`outputs/semantic-live-view/gui-summary.json`、`gui-test.log`、各组 `RunFrames.csv`、`module-test.log`。实际实时窗口截图：`viewer-1.png`、`viewer-2.png`。未在用户物理桌面上手动验证窗口布局，自动 GUI 测试使用虚拟显示器。

复现 GUI 验证（需 Xvfb、xwininfo、xwd、Python NumPy/Pillow，仅测试工具依赖）：

```bash
xvfb-run -a -s '-screen 0 1920x1080x24 +extension GLX' \
  env LIBGL_ALWAYS_SOFTWARE=1 python3 tools/verify_semantic_viewer.py
```

正常运行不需要 Python 或 Xvfb。使用 `config/TUM3-semantic-selected.yaml`，其中 `Run.Viewer: 1` 与 `SemanticMask.Visualize: 1` 已启用。实时显示与 `SemanticMask.Debug` 保存图片开关独立。
