"""cleaning_mode -- 栅格覆盖式清洁规划包.

模块:
- obstacle_inflation: 障碍物膨胀与未知区处理
- boustrophedon: boustrophedon 分解与蛇形路径生成
- coverage_planner: 覆盖规划器(主入口)
- path_smoother: 路径平滑
- ros_path_publisher: ROS 2 nav_msgs/Path 发布节点

用法:
    from cleaning_mode.coverage_planner import make_plan
    from cleaning_mode.boustrophedon import GridSpec, connected_boustrophedon, inflate
"""
from __future__ import annotations
