# 地图覆盖规划与 RViz2

目标环境：ROS 2 Humble。所有编译和运行都在 `navigation_sim.sh` 打开的
`ros2-navigation2` Docker 容器内进行。

核心目录：

```text
scripts/                         地图编辑、依赖安装、启动和重规划入口
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
规划状态、路径点数和长度；最终路径发布到 `/coverage_map/path`。
RViz2 已配置地图、边界、障碍物、条带和路径显示。

常用参数：

```bash
# 只编译地图演示的三个核心包。
./scripts/run_coverage_map_demo.sh --build-only

# 使用现有构建启动，或不启动图形界面。
./scripts/run_coverage_map_demo.sh --no-build
./scripts/run_coverage_map_demo.sh --no-rviz

# 修改排序方式和角度；角度单位为弧度。
./scripts/run_coverage_map_demo.sh -- route_type:=BOUSTROPHEDON
./scripts/run_coverage_map_demo.sh -- route_type:=SPIRAL spiral_n:=4 swath_angle:=1.5708

# 自定义地图目录；编辑与启动必须使用同一目录。
python3 scripts/edit_coverage_map.py --output /tmp/my_map
./scripts/run_coverage_map_demo.sh --map-dir /tmp/my_map
```

机器人宽度、作业宽度、最小转弯半径和边缘预留宽度在
`opennav_coverage_demo/params/coverage_map_params.yaml` 中配置。
`SPIRAL` 是已有条带的螺旋排序，非逐圈向内收缩的边界覆盖。
规划输入来自 `field.json`；单独修改 PGM 不会改变规划边界。
转弯连接不使用栅格地图进行碰撞检测，此入口用于规划与可视化。

启动脚本只构建 `opennav_coverage_msgs`、`opennav_coverage` 和
`opennav_coverage_demo`。仓库中原有的行覆盖、行为树和 Navigator 扩展保留为
独立模块，不属于本入口的构建范围；Navigator 需要 Iron 以上版本。
这三个可选目录已用 `COLCON_IGNORE` 排除，默认 `colcon build` 也只发现核心三个包。
以后启用扩展时，先移除对应标记并适配其 ROS 版本依赖。
编译输出临时生成在 `.coverage_demo/humble/`。

启动和重规划共用 `scripts/coverage_demo_env.sh`，自动加载本地 Fields2Cover，
并保持 DDS 实现一致。容器默认使用软件 OpenGL；GPU 透传可用时可传入
`software_rendering:=false`。
