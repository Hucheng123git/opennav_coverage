# 覆盖规划地图演示

使用方法及核心目录见 [工程说明](../README.md)。

主要文件：

- `opennav_coverage_demo/edit_coverage_map.py`：交互式地图编辑。
- `opennav_coverage_demo/coverage_map.py`：多边形校验及地图导出。
- `opennav_coverage_demo/coverage_map_client.py`：地图重载、规划接口和路径发布。
- `opennav_coverage_demo/coverage_visualization.py`：方向箭头、作业与转弯分色、访问序号及起终点标记。
- `launch/coverage_map_launch.py`：启动地图、覆盖规划和 RViz2。
- `params/coverage_map_params.yaml`：规划参数。
- `params/coverage_map.rviz`：RViz2 显示配置。
- `../scripts/export_coverage_results.py`：调用真实规划接口，导出所有排序算法对比图片。
- `../scripts/result/README.md`：图片目录与排序算法说明。

支持 `route_type:=CONCENTRIC contour_spacing:=0.7` 的逐层内缩回形覆盖及孔洞地图。
回形轮廓及其避孔连接由核心 C++ 生成，Python 客户端复用原规划接口和 RViz 标记。
