# Nav2 目标点导航评估记录

## 1. 实验链路

本次实验使用仓库现有 launch：

```bash
ros2 launch rl_sar a1_nav2_estimated_3d_open.launch.py use_rviz:=false
```

完整链路为：

1. Gazebo 提供真值 IMU、足端接触和激光雷达。
2. `a1_contact_preprocessor.py` 将四足接触信息统一成 `/a1/contact_state`。
3. `a1_invariant_ekf_node` 基于 IMU + 接触约束输出 `/odom_3d`。
4. `project_odom_3d_to_2d.py` 将 3D 里程计投影成 Nav2 使用的 `/odom`。
5. AMCL 结合地图和 `/scan` 进行定位修正。
6. Nav2 输出 `/cmd_vel`，由 `rl_sim` 导航模式接管。
7. A1 `legged_gym` policy 将速度命令转换成全身关节动作，`robot_joint_controller` 输出力矩驱动 Gazebo 机器狗。

## 2. 任务级指标

- 平均真值目标点误差：`0.286 m`
- 平均 AMCL 目标点误差：`0.400 m`
- `/odom` 相对 `/odom_gt` 的位置 RMSE：`0.506 m`
- `map->base_footprint` 相对 `/odom_gt` 的位置 RMSE：`0.177 m`
- `cmd_vel` 非零比例：`0.920`

逐目标结果：

- `goal_1m`: 状态 `SUCCEEDED`，真值目标点误差 `0.379 m`，AMCL 目标点误差 `0.545 m`，真值路径长度 `0.499 m`，耗时 `2.17 s`。
- `goal_2m`: 状态 `SUCCEEDED`，真值目标点误差 `0.366 m`，AMCL 目标点误差 `0.576 m`，真值路径长度 `0.878 m`，耗时 `3.36 s`。
- `goal_left`: 状态 `SUCCEEDED`，真值目标点误差 `0.113 m`，AMCL 目标点误差 `0.081 m`，真值路径长度 `1.804 m`，耗时 `12.06 s`。

## 3. 创新点

1. **四足机器人接入 Nav2 的桥接链路**：上层仍是标准 Nav2 目标导航接口，下层不是差速/阿克曼底盘，而是强化学习四足步态控制器。
2. **真值 IMU + 足端接触约束的状态估计链路**：不是直接把 Gazebo 平面位姿喂给导航，而是通过接触预处理和 3D invariant EKF 先构造机身运动估计，再投影到 2D。
3. **全身 RL 运动控制作为导航执行器**：`/cmd_vel` 最终由 A1 全身 locomotion policy 跟踪，而不是传统底盘速度控制器。
4. **3D 到 2D 的导航适配**：四足机身的 3D 姿态估计通过 `project_odom_3d_to_2d.py` 适配 Nav2 的平面定位接口，保留了四足系统的动态特征，又兼容了成熟导航栈。

## 4. 结果文件

- `summary.json`
- `samples.csv`
- `01_path_overview.png`
- `02_goal_error_timeseries.png`
- `03_goal_summary.png`
- `01_ros2_nodes.txt`
- `02_ros2_topics.txt`
- `03_ros2_actions.txt`
- `04_ros2_controllers.txt`
- `05_robot_joint_controller_state.txt`
- `06_contact_state.txt`
- `07_odom.txt`
- `08_odom_gt.txt`
