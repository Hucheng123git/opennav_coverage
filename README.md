# 地图覆盖规划与 RViz2

目标环境：ROS 2 Humble。所有编译和运行都在 `navigation_sim.sh` 打开的
`ros2-navigation2` Docker 容器内进行。

核心目录：

```text
scripts/                         地图编辑、依赖安装、启动和重规划入口
scripts/result/                  排序算法及 SPIRAL 分组对比图片
maps/coverage_map/               可编辑的 field.json、map.yaml、map.pgm
opennav_coverage/include/        覆盖规划接口与类型
opennav_coverage/src/            边缘预留、条带生成、排序和转弯连接
opennav_coverage/maps/           GML 多边形地图样例
opennav_coverage_msgs/           ROS 消息和 ComputeCoveragePath Action
opennav_coverage_demo/
  opennav_coverage_demo/         地图编辑器、地图导出、规划接口节点
  params/                       规划参数与 RViz2 配置
  launch/coverage_map_launch.py  地图服务、规划服务和 RViz2 启动
```

从宿主机进入容器：

```bash
bash /home/hucheng/shell/navigation_sim.sh
```

容器内执行：

```bash
cd /home/hucheng/hucheng_ws/ros2_ws/opennav_coverage
source /opt/ros/humble/setup.bash

# 首次安装 Fields2Cover v1.2.1；安装在本工程 .deps 中。
./scripts/setup_fields2cover.sh

# 编辑地图。左键添加顶点，Enter 闭合，H 添加障碍物，S 保存。
python3 scripts/edit_coverage_map.py

# 编译并启动，加载地图，打开 RViz2，自动规划一次。
./scripts/run_coverage_map_demo.sh
```

编辑器支持拖动顶点、Backspace 撤销和 R 清空重画；再次打开会读取原地图。
保存同时生成 `field.json`（规划边界及障碍物）、`map.pgm` 和 `map.yaml`
（RViz2 栅格地图）。地图坐标单位为米，固定坐标系为 `map`。

保持启动终端运行，在另一个通过 `navigation_sim.sh` 进入的容器终端编辑并保存后：

```bash
cd /home/hucheng/hucheng_ws/ros2_ws/opennav_coverage
./scripts/replan_coverage_map.sh
```

该脚本调用 `/coverage_map/plan`（`std_srvs/srv/Trigger`），重新加载地图并返回
规划状态、路径点数和长度。路径发布到 `/coverage_map/path`，方向及顺序标记
发布到 `/coverage_map/plan_markers`，RViz2 默认显示这些标记及地图、边界和障碍物。
蓝色表示作业段，橙色表示转弯连接；箭头表示路径行进方向，条带序号表示实际
访问顺序，绿色和品红色分别标出起点、终点。箭头默认间隔 2 米，可通过
`arrow_spacing:=1.0` 调整；`show_swath_numbers:=false` 可隐藏序号。

常用参数：

```bash
# 只编译地图演示的三个核心包。
./scripts/run_coverage_map_demo.sh --build-only

# 使用现有构建启动，或不启动图形界面。
./scripts/run_coverage_map_demo.sh --no-build
./scripts/run_coverage_map_demo.sh --no-rviz

# 修改排序方式和角度；角度单位为弧度。
./scripts/run_coverage_map_demo.sh -- route_type:=BOUSTROPHEDON
./scripts/run_coverage_map_demo.sh -- route_type:=SNAKE
./scripts/run_coverage_map_demo.sh -- route_type:=SPIRAL spiral_n:=4 swath_angle:=1.5708

# 真正的回形覆盖：每次向内缩 0.7 米；使用含两个孔洞的独立样例。
./scripts/run_coverage_map_demo.sh --map-dir maps/coverage_holes -- \
  route_type:=CONCENTRIC contour_spacing:=0.7

# 自定义地图目录；编辑与启动必须使用同一目录。
python3 scripts/edit_coverage_map.py --output /tmp/my_map
./scripts/run_coverage_map_demo.sh --map-dir /tmp/my_map
```

机器人宽度、作业宽度、最小转弯半径和边缘预留宽度在
`opennav_coverage_demo/params/coverage_map_params.yaml` 中配置。
默认 `SPIRAL` 对已有平行条带进行螺旋排序；新增 `CONCENTRIC` 沿区域边界逐圈
向内偏移，生成真正的回形轮廓。两种模式保留各自含义。
`BOUSTROPHEDON` 相邻逐行往返；`SNAKE` 先隔行作业再返回补齐；`SPIRAL`
按 `spiral_n` 条分组，在组内从两侧交替向内访问。图例显示实际使用的排序模式。
`CUSTOM` 按请求提供的完整条带索引排列访问，索引从 0 开始。
客户端支持通过 `custom_order` 整数数组参数设置该排列，数量必须等于实际条带数。
`CONCENTRIC` 使用 `contour_spacing` 指定每层内缩距离（米）；0 采用服务端
`default_contour_spacing`，服务端也为 0 时采用 `operation_width`（当前 0.7 米）。
首圈与原边界的距离取边缘预留宽度和机器人半宽的较大值。每层基于原始含孔洞
多边形重新计算：外边界内缩、孔洞外扩，保留内缩后分裂出的全部区域及孔洞轮廓。
先遍历同一层的轮廓，再进入下一层；圈间优先直连，遇到孔洞或凹边界时通过
可见性图寻找位于机器人半宽净空内的连接路线。净空区域不连通时返回失败，
不会跨越孔洞，也不会只保留其中一片区域后报告成功。

回形路径使用 `POLYLINE`，适用于可以原地转向的机器人；拐点没有 Dubins 圆滑化，
不保证最小转弯半径。显式要求回形路径使用 DUBIN/REEDS_SHEPP 会被拒绝。
作业宽度之外的额外大间距会留下未覆盖带，配置间距时需结合 `operation_width`。
原有四种模式仍走平行条带与 Dubins/Reeds-Shepp 流程。
`PathComponents` 保持原结构，回形的每条轮廓边作为一个 Swath，边间及圈间
连接保存在 turns 中；RViz2 的 `C1、C2…` 表示实际轮廓访问顺序。

切换已启动的新版本客户端：

```bash
source scripts/coverage_demo_env.sh
ros2 param set /coverage_map_client route_type CONCENTRIC
ros2 param set /coverage_map_client contour_spacing 0.7
./scripts/replan_coverage_map.sh
```

`RouteMode` 在原字段后增加了默认值为 0 的 `contour_spacing`，原模式与默认参数
保持兼容。首次更新后请关闭旧演示，再运行启动脚本重新编译和启动，保证客户端、
服务端使用同一版消息；不要在同一 ROS domain 中同时运行旧版和新版节点。
规划输入来自 `field.json`；单独修改 PGM 不会改变规划边界。
原有模式的转弯连接不使用栅格地图进行碰撞检测；回形模式针对多边形及孔洞检查
每条连接线的几何净空，不读取 PGM 作为额外障碍物。

保持一个覆盖规划服务运行，在另一个容器终端导出所有排序算法的实际结果：

```bash
cd /home/hucheng/hucheng_ws/ros2_ws/opennav_coverage
source scripts/coverage_demo_env.sh
python3 scripts/export_coverage_results.py
```

图片保存到 `scripts/result/`：`boustrophedon.png`、`snake.png`、
`spiral_2.png`、`spiral_4.png`、`spiral_8.png`、`custom.png`、`concentric.png`，
以及总览 `coverage_comparison.png`。导出脚本直接调用
`ComputeCoveragePath`，所有曲线来自实际规划结果；每幅图保留方向箭头、
访问序号及路径长度。`CUSTOM` 使用每隔三条访问再补齐的完整排列作为示例。
只导出含孔洞的回形结果：

```bash
python3 scripts/export_coverage_results.py --modes CONCENTRIC \
  --map-dir maps/coverage_holes --output scripts/result/holes --contour-spacing 0.7
```

可通过 `--map-dir`、`--output`、`--swath-angle` 指定地图、结果目录和作业角度；
`python3 scripts/export_coverage_results.py --help` 查看完整选项。
导出端和服务端的 `ROS_DOMAIN_ID` 必须一致，同一个 domain 中只启动一套演示。
图片含义与排序算法详解见 [结果说明](scripts/result/README.md)。

启动脚本只构建 `opennav_coverage_msgs`、`opennav_coverage` 和
`opennav_coverage_demo`。仓库中原有的行覆盖、行为树和 Navigator 扩展保留为
独立模块，不属于本入口的构建范围；Navigator 需要 Iron 以上版本。
这三个可选目录已用 `COLCON_IGNORE` 排除，默认 `colcon build` 也只发现核心三个包。
以后启用扩展时，先移除对应标记并适配其 ROS 版本依赖。
编译输出临时生成在 `.coverage_demo/humble/`。

启动和重规划共用 `scripts/coverage_demo_env.sh`，自动加载本地 Fields2Cover，
并保持 DDS 实现一致。容器默认使用软件 OpenGL；GPU 透传可用时可传入
`software_rendering:=false`。
退出时先使用 RViz2 窗口的关闭按钮，再在启动终端按 Ctrl+C。
