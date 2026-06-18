# 任务 2：仿真搭建复用说明

任务 2 复用当前项目已经调通的 A1 Gazebo/ROS2 仿真框架。为满足课程提交要求，和本任务直接相关的 URDF、mesh、ROS 2 control 配置、Gazebo/Nav2 launch 与控制器源码已经抽取到：

```text
source_code/
```

## 对应作业要求

| 作业要求 | 当前实现 |
| --- | --- |
| URDF links/joints | `source_code/a1_description/urdf/a1_description.urdf` |
| 质量、惯量、碰撞 | 同上，A1 trunk、hip、thigh、calf、foot 均已有 inertial/collision |
| ros2_control | `source_code/a1_description/xacro/gazebo.xacro` |
| Gazebo 插件 | `gazebo_ros2_control`、IMU、LiDAR、P3D ground-truth odom |
| 可视化和控制 | `source_code/rl_sar_a1_runtime/launch/a1_nav2_sim.launch.py` |
| 关节控制器 | `source_code/robot_joint_controller/` |
| 控制消息 | `source_code/robot_msgs/` |

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

课程仓库中的 `source_code/` 用于源码审阅；真正运行 Gazebo/Nav2 时仍建议使用已经构建好的 ROS 2 工作区。
