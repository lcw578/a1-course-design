# A1 课程设计源码子集

本目录保存课程设计直接使用的 A1 相关源码，用于满足作业要求中的“源代码（含 URDF 文件及控制程序）”。这里不是完整上游 `rl_sar` 仓库，而是经过裁剪的最小相关子集。

## 目录说明

| 路径 | 作用 |
| --- | --- |
| `a1_description/` | A1 URDF、xacro、mesh、Gazebo/RViz launch、ROS 2 control 配置。 |
| `a1_policy/` | A1 `legged_gym` policy 配置和 `model.pt`。 |
| `rl_sar_a1_runtime/` | A1 Gazebo/Nav2/RL 运行代码，包括 `rl_sim.cpp`、A1 FSM、launch、地图、world 和评估脚本。 |
| `robot_joint_controller/` | ROS 2 关节控制器源码，接收 `RobotCommand` 并输出关节 effort 命令。 |
| `robot_msgs/` | 控制器和 RL runtime 使用的 ROS 2 消息定义。 |

## 课程使用的控制链路

```text
Nav2 /cmd_vel
  -> rl_sar_a1_runtime/src/rl_sim.cpp
  -> a1_policy/legged_gym/model.pt
  -> robot_msgs/RobotCommand
  -> robot_joint_controller
  -> a1_description/urdf/a1_description.urdf
  -> Gazebo A1
```

## 关键文件

| 文件 | 说明 |
| --- | --- |
| `a1_description/urdf/a1_description.urdf` | A1 机器人 URDF，含 links、joints、mass、inertia、collision。 |
| `a1_description/xacro/gazebo.xacro` | Gazebo 插件、ros2_control、IMU、LiDAR、ground-truth odom 等配置。 |
| `a1_description/config/robot_control_ros2.yaml` | ROS 2 controller manager 和 A1 joint controller 配置。 |
| `rl_sar_a1_runtime/src/rl_sim.cpp` | Gazebo 中的 A1 RL 控制运行节点，订阅 `/cmd_vel` 并输出关节命令。 |
| `rl_sar_a1_runtime/fsm_robot/fsm_a1.hpp` | A1 状态机，包含 passive、get-up、RL locomotion、get-down 等状态。 |
| `robot_joint_controller/ros2/src/robot_joint_controller_group.cpp` | A1 12 关节 group controller，向 Gazebo effort interface 写入命令。 |
| `rl_sar_a1_runtime/scripts/gazebo_ground_truth_odom.py` | 将 Gazebo ground-truth odom 发布给 Nav2，用于排除定位误差。 |
| `rl_sar_a1_runtime/scripts/evaluate_nav2_goal_sequence.py` | Nav2 多目标导航误差评估脚本。 |

## 裁剪原则

保留：

- A1 机器人描述文件、mesh 和 ROS 2 控制配置。
- A1 强化学习策略和直接相关 runtime。
- 本课程使用的 Gazebo/Nav2 launch、地图和评估脚本。
- ROS 2 关节控制器与消息定义。

排除：

- 其他机器人平台源码。
- 与本课程无关的硬件 SDK 和大型第三方库。
- `build/`、`install/`、`log/`、`__pycache__`、`.pyc`、运行日志。
- 原始中间训练证据和临时 proxy 图。

## 复现说明

本目录主要用于课程提交与源码审阅。完整运行仍建议在原 ROS 2 Humble 工作区中构建相关包，因为 `rl_sar` 的推理 runtime、ROS 2 依赖和 Gazebo/Nav2 环境需要系统级安装。

对应实验结果保存在仓库根目录的 `results/` 中，任务报告与推导保存在 `task1_kinematics/` 到 `task7_rl_velocity_eval/` 中。
