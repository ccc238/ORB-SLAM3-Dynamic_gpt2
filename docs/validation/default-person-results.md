# 默认启用 selected/person 验证

RGB-D 默认启用语义区域屏蔽，模式 selected，类别 [0]（person）。未配置模型时使用 $HOME/models/yolo/yolo11s-seg.onnx；Enabled: 0 可显式关闭。其他传感器仍默认关闭。

- 最终全目标单线程编译通过：build-final.log。TUM 示例最终强制重编译日志：build-rgbd-refresh.log。
- build.log 为收到追加修改要求时的中间构建，其中旧接口链接错误已在最终构建解决；同时修复了编译期间编辑 TUM 示例造成的旧对象复用，并重新运行了全部下述短序列检查。
- module-test.log：默认启用、默认 person 的精确掩码并集、显式空列表报错、默认关闭的其他传感器配置、显式关闭覆盖默认值，以及原有分割/特征验证均通过。
- TUM3-default-person.yaml 和 TUM3-default-person-rgb.yaml 省略 Enabled、Model、Mode、Classes，分别实测 BGR 与 RGB 配置各 5 帧，全部 OK，关键点违规数均为 0。
- 两种配置产生的 5 张二值 mask 逐文件完全一致。TUM 示例读取后按 Camera.RGB 转换通道顺序，原配置 Camera.RGB=1 也能正确输入。
- 显式关闭跑 3 帧，均为 OK，无分割会话创建。
- 本目录为新输出目录，保留了先前 outputs/semantic-validation 中的历史结果。当前结果见 summary.json、各 run 日志/CSV 和 debug 图像。
