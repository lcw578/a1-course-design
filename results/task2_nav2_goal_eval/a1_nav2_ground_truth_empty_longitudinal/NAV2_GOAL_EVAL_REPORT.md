# Nav2 目标点导航评估记录

## 1. 实验链路

本次实验使用仓库现有 launch：

```bash
ros2 launch rl_sar a1_nav2_sim.launch.py world:=earth use_rviz:=false
```

完整链路为：

1. Gazebo 提供激光雷达和 `ground_truth/odom_raw`。
2. `gazebo_ground_truth_odom.py` 直接将真值位姿重发布为 Nav2 使用的 `/odom`。
3. 同一真值链路额外发布 `/odom_gt`，只用于离线评估。
4. `map_server + navigation_launch.py` 输出 `/cmd_vel`，由 `rl_sim` 导航模式接管。
5. A1 `legged_gym` policy 将速度命令转换成全身关节动作，`robot_joint_controller` 输出力矩驱动 Gazebo 机器狗。

## 2. 任务级指标

- 平均真值目标点误差：`0.287 m`
- 平均 `/odom` 目标点误差：`0.287 m`
- `/odom` 相对 `/odom_gt` 的位置 RMSE：`0.000 m`
- `cmd_vel` 非零比例：`0.963`

逐目标结果：

- `goal_2m`: 状态 `SUCCEEDED`，真值目标点误差 `0.169 m`，Nav2 使用 `/odom` 的目标点误差 `0.169 m`，真值路径长度 `1.603 m`，耗时 `149.05 s`。
- `goal_3m`: 状态 `SUCCEEDED`，真值目标点误差 `0.338 m`，Nav2 使用 `/odom` 的目标点误差 `0.338 m`，真值路径长度 `0.650 m`，耗时 `2.81 s`。
- `goal_4m`: 状态 `SUCCEEDED`，真值目标点误差 `0.353 m`，Nav2 使用 `/odom` 的目标点误差 `0.353 m`，真值路径长度 `0.887 m`，耗时 `3.76 s`。

## 3. 创新点

1. **四足机器人接入 Nav2 的桥接链路**：上层仍是标准 Nav2 目标导航接口，下层不是差速/阿克曼底盘，而是强化学习四足步态控制器。
2. **ground-truth odom 直连 Nav2 的对照实验**：先把定位问题排除，只评估 `Nav2 + RL 执行器` 的任务级目标点误差，便于和后续状态估计方案做对比。
3. **全身 RL 运动控制作为导航执行器**：`/cmd_vel` 最终由 A1 全身 locomotion policy 跟踪，而不是传统底盘速度控制器。
4. **四足机器人与平面导航接口兼容**：虽然底层是四足全身控制，但上层仍保持标准地图导航接口，便于课程报告说明系统集成思路。

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
