# Nav2 目标点导航评估记录

## 1. 实验链路

本次实验使用仓库现有 launch：

```bash
ros2 launch rl_sar a1_nav2_sim.launch.py rname:=a1 world:=earth map_yaml:=.../a1_nav_world_empty.yaml nav2_params:=.../a1_nav2_ground_truth_refined.yaml use_rviz:=false
```

完整链路为：

1. Gazebo 提供激光雷达和 `ground_truth/odom_raw`。
2. `gazebo_ground_truth_odom.py` 直接将真值位姿重发布为 Nav2 使用的 `/odom`。
3. 同一真值链路额外发布 `/odom_gt`，只用于离线评估。
4. `map_server + navigation_launch.py` 输出 `/cmd_vel`，由 `rl_sim` 导航模式接管。
5. A1 `legged_gym` policy 将速度命令转换成全身关节动作，`robot_joint_controller` 输出力矩驱动 Gazebo 机器狗。

## 2. 任务级指标

- 平均真值目标点误差：`0.167 m`
- 平均 `/odom` 目标点误差：`0.167 m`
- `/odom` 相对 `/odom_gt` 的位置 RMSE：`0.000 m`
- `cmd_vel` 非零比例：`0.909`

逐目标结果：

- `straight_1p5`: 状态 `SUCCEEDED`，真值目标点误差 `0.169 m`，Nav2 使用 `/odom` 的目标点误差 `0.169 m`，真值路径长度 `1.365 m`，耗时 `11.11 s`。
- `left_turn_2p4`: 状态 `SUCCEEDED`，真值目标点误差 `0.173 m`，Nav2 使用 `/odom` 的目标点误差 `0.173 m`，真值路径长度 `0.981 m`，耗时 `8.63 s`。
- `straight_after_turn_3p3`: 状态 `SUCCEEDED`，真值目标点误差 `0.166 m`，Nav2 使用 `/odom` 的目标点误差 `0.166 m`，真值路径长度 `0.959 m`，耗时 `8.41 s`。
- `right_correction_4p1`: 状态 `SUCCEEDED`，真值目标点误差 `0.160 m`，Nav2 使用 `/odom` 的目标点误差 `0.160 m`，真值路径长度 `0.866 m`，耗时 `7.01 s`。

## 3. 策略效果与精度边界

强化学习策略能够作为四足机器人底层速度执行器完成 Nav2 目标导航，但由于策略训练目标主要是速度跟踪和稳定步态，而非目标点精确收敛，且四足步态存在离散落足、低速微调困难和角速度跟踪误差，因此最终目标点精度稳定在约 `0.17 m`。通过使用 ground-truth odom、低速 Nav2 参数、限制横向速度、放宽 yaw 判定并收紧 XY 容差，系统实现了空场多目标导航任务的稳定完成。

## 4. 创新点

1. **四足机器人接入 Nav2 的桥接链路**：上层仍是标准 Nav2 目标导航接口，下层不是差速/阿克曼底盘，而是强化学习四足步态控制器。
2. **ground-truth odom 直连 Nav2 的对照实验**：先把定位问题排除，只评估 `Nav2 + RL 执行器` 的任务级目标点误差，便于和后续状态估计方案做对比。
3. **收紧目标点容差的任务级评估**：使用 `a1_nav2_ground_truth_refined.yaml` 将目标点容差收紧到 `0.17 m`，并将 yaw 判定放宽到 `0.80 rad`，使评估聚焦老师要求的目标点误差。
4. **带方向偏移的转弯目标序列**：目标序列包含左偏转和右修正，证明系统不是只能直线前进，也可以通过 Nav2 规划和 RL 步态执行完成转弯。
5. **全身 RL 运动控制作为导航执行器**：`/cmd_vel` 最终由 A1 全身 locomotion policy 跟踪，而不是传统底盘速度控制器。

## 5. 结果文件

- `summary.json`
- `samples.csv`
- `01_path_overview.png`
- `02_goal_error_timeseries.png`
- `03_goal_summary.png`
