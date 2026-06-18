# 任务 2：仿真搭建复用说明

任务 2 不在本目录重复复制机器人描述文件，直接复用当前仓库中已经存在的 A1 Gazebo/ROS2 仿真框架。

## 对应作业要求

| 作业要求 | 当前实现 |
| --- | --- |
| URDF links/joints | `src/rl_sar/src/rl_sar_zoo/a1_description/urdf/a1_description.urdf` |
| 质量、惯量、碰撞 | 同上，A1 trunk、hip、thigh、calf、foot 均已有 inertial/collision |
| ros2_control | `src/rl_sar/src/rl_sar_zoo/a1_description/xacro/gazebo.xacro` |
| Gazebo 插件 | `gazebo_ros2_control`、IMU、LiDAR、P3D ground-truth odom |
| 可视化和控制 | `src/rl_sar/src/rl_sar/launch/a1_nav2_sim.launch.py` |

## 运行命令

```bash
source /opt/ros/humble/setup.bash
source /home/lcw/rl_sar_ws/install/setup.bash
ros2 launch rl_sar a1_nav2_sim.launch.py
```

如需降低负载：

```bash
ros2 launch rl_sar a1_nav2_sim.launch.py use_rviz:=false
```

## 报告写法建议

报告中将 A1 单腿作为 3R 串联机构完成理论建模，将 A1 整机 URDF/Gazebo 模型作为仿真平台。`ros2_control` 实际使用 effort command interface，`robot_joint_controller` 接收 `q/dq/kp/kd/tau` 后在控制器内部计算输出力矩，这与 PDF 中 velocity command interface 示例不同，需要在报告中说明为“关节伺服/力矩接口”。

