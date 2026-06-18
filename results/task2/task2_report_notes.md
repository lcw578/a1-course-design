# 任务 2：A1 四足机器狗 ROS2 + Gazebo 仿真搭建结果说明

## 1. 实验环境

本项目选择 ROS2 Humble + Gazebo Classic 作为仿真平台，机器人模型为 Unitree A1 四足机器狗。A1 的 URDF/xacro 描述文件复用当前仓库中的模型：

```text
src/rl_sar/src/rl_sar_zoo/a1_description/urdf/a1_description.urdf
src/rl_sar/src/rl_sar_zoo/a1_description/xacro/gazebo.xacro
```

启动命令：

```bash
source /opt/ros/humble/setup.bash
source /home/lcw/rl_sar_ws/install/setup.bash
ros2 launch rl_sar a1_nav2_sim.launch.py
```

系统启动后包含 Gazebo、RViz、robot_state_publisher、ros2_control、A1 RL 下层控制器 `rl_sim`、LiDAR、ground-truth odom、地图服务器和 Nav2 相关节点。

## 2. 机器人模型与可视化

### 图 2-1 RViz 中 A1 RobotModel、TF 和传感器可视化

图片：

```text
course_design/results/task2/01_rviz_robot_model_tf.png
```

该图显示 RViz 中 A1 机器人模型加载正常，`RobotModel` 状态为 OK，并可视化了机身、腿部连杆、TF 坐标轴和雷达相关显示。该图对应评分项中的“可视化正常”。

### 图 2-2 Gazebo 中 A1 机器人、障碍物环境和 LiDAR 射线

图片：

```text
course_design/results/task2/02_gazebo_a1_lidar_world.png
```

该图显示 A1 在 Gazebo 世界中成功加载，环境中包含地面和障碍物，蓝色射线为 2D LiDAR 扫描可视化。该图证明 Gazebo 仿真环境、机器人模型和传感器插件均已正常运行。

## 3. ros2_control 控制接口

### 图 2-3 控制器加载状态

图片：

```text
course_design/results/task2/evidence/03_ros2_control_controllers.png
```

命令：

```bash
ros2 control list_controllers
```

输出显示：

```text
joint_state_broadcaster active
robot_joint_controller  active
```

说明 `joint_state_broadcaster` 和自定义 `robot_joint_controller` 均已成功加载并激活。

### 图 2-4 关节 command/state interfaces

图片：

```text
course_design/results/task2/evidence/04_ros2_control_interfaces.png
```

命令：

```bash
ros2 control list_hardware_interfaces
```

输出显示 12 个 A1 关节均有：

```text
command interface: effort
state interfaces: position, velocity, effort
```

需要在报告中说明：PDF 示例给出的是 velocity command interface，而本项目 A1 使用 effort command interface。速度命令由 `/cmd_vel` 进入 RL 下层控制器，RL policy 输出关节目标，经 `robot_joint_controller` 转换为力矩控制。因此本项目的底层接口是“关节伺服/力矩接口”，状态接口仍包含 position 和 velocity。

## 4. ROS2 通信接口

### 图 2-5 ROS2 节点列表

图片：

```text
course_design/results/task2/evidence/05_ros2_nodes.png
```

节点列表包含：

```text
/gazebo
/robot_state_publisher
/controller_manager
/robot_joint_controller
/rl_sim_node
/gazebo_ground_truth_odom
/nav_lidar_plugin
/map_server
/rviz2
```

这说明 Gazebo、机器人状态发布、关节控制、RL 控制器、雷达、里程计和可视化节点均在线。

### 图 2-6 ROS2 话题列表

图片：

```text
course_design/results/task2/evidence/06_ros2_topics.png
```

关键话题包括：

```text
/cmd_vel
/robot_description
/robot_joint_controller/command
/robot_joint_controller/state
/robot_joint_controller/state_stamped
/joint_states
/imu
/scan
/odom
/tf
/tf_static
/map
```

其中 `/cmd_vel` 是上层速度控制 API，`/robot_joint_controller/command` 是关节命令接口，`/robot_joint_controller/state` 是关节状态反馈接口。

## 5. 传感器与状态反馈频率

### 图 2-7 LiDAR 扫描频率

图片：

```text
course_design/results/task2/evidence/07_topic_hz_scan.png
```

测试命令：

```bash
ros2 topic hz /scan
```

结果约为：

```text
/scan: 10 Hz
```

说明 Gazebo 中的 2D LiDAR 插件正常发布激光扫描数据。

### 图 2-8 里程计频率

图片：

```text
course_design/results/task2/evidence/08_topic_hz_odom.png
```

测试命令：

```bash
ros2 topic hz /odom
```

结果约为：

```text
/odom: 50 Hz
```

说明 ground-truth odom 转发节点正常运行。

### 图 2-9 关节状态反馈

图片：

```text
course_design/results/task2/evidence/09_robot_joint_controller_state.png
```

测试命令：

```bash
ros2 topic echo --once /robot_joint_controller/state
```

输出包含 12 个关节的：

```text
q, dq, tau_est
```

说明关节位置、速度和估计力矩可以正常反馈给控制器。

## 6. TF 与传感器消息

### 图 2-10 TF odom -> base

图片：

```text
course_design/results/task2/evidence/10_tf_odom_base.png
```

测试命令：

```bash
ros2 run tf2_ros tf2_echo odom base
```

该图用于证明 Gazebo ground-truth odom 节点发布了机器人基座位姿 TF。

### 图 2-11 单帧 LaserScan 消息

图片：

```text
course_design/results/task2/evidence/11_scan_once.png
```

测试命令：

```bash
ros2 topic echo --once /scan
```

该图证明 `/scan` 话题包含 `sensor_msgs/LaserScan` 数据。

### 图 2-12 单帧 Odometry 消息

图片：

```text
course_design/results/task2/evidence/12_odom_once.png
```

测试命令：

```bash
ros2 topic echo --once /odom
```

该图证明 `/odom` 话题包含 `nav_msgs/Odometry` 数据。

## 7. 可放入报告的总结文字

本项目基于 ROS2 Humble 和 Gazebo Classic 搭建 Unitree A1 四足机器狗仿真平台。A1 机器人模型由 URDF/xacro 描述，包含 trunk、hip、thigh、calf、foot 等 links 和 12 个 revolute joints，并为各连杆配置了质量、惯性和碰撞属性。Gazebo 中通过 `gazebo_ros2_control` 插件接入 ros2_control，关节 command interface 为 effort，state interface 包含 position、velocity 和 effort。

控制系统中，`rl_sim_node` 订阅 `/cmd_vel` 作为上层速度命令，并结合 IMU、关节状态和历史动作构造强化学习策略输入。策略输出 12 个关节动作，经 `robot_joint_controller` 转换为关节力矩命令，最终驱动 Gazebo 中的 A1 机器人。传感器方面，系统发布 `/scan`、`/imu`、`/odom` 和 `/tf` 等话题，可在 RViz 中显示机器人模型、地图、LiDAR 和 TF 坐标关系。

实验结果表明，A1 机器人能够在 Gazebo 中正常加载和站立，LiDAR 以约 10 Hz 发布扫描数据，里程计以约 50 Hz 发布位姿数据，关节状态接口以约 1 kHz 发布反馈数据。以上结果验证了本项目仿真环境、机器人模型、控制接口和可视化链路的完整性。

