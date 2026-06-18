# 任务 7：基于强化学习的无模型速度跟踪控制

## 1. 任务说明

任务 7 要求在不显式依赖动力学模型参数的前提下，使用强化学习训练控制器完成速度跟踪，并展示训练过程和仿真评估结果。

当前项目的落地方式分成两部分：

1. **训练过程证据**  
   按用户要求，采用 `UniLab` 中的四足机器人 PPO 训练记录作为任务 7 的训练过程证明材料。
2. **A1 仿真评估**  
   使用当前仓库已有的 A1 policy：
   [model.pt](/home/lcw/rl_sar_ws/src/rl_sar/policy/a1/legged_gym/model.pt)
   在 Gazebo + ROS2 仿真中评估 `/cmd_vel -> /odom` 的速度跟踪 RMSE。

这意味着：

```text
训练曲线证据：A1 速度跟踪策略训练过程图
最终推理评估：当前 rl_sar 的 A1 legged_gym policy
```

## 2. 训练过程证据

关键摘要：

```text
algo                 = PPO
task                 = A1JoystickFlat
sim backend          = motrix
completed iterations = 150
total env steps      = 3,710,976
final mean reward    = 45.3652
best mean reward     = 47.1013
mean episode length  = 1000.0
last checkpoint      = model_150.pt
```

训练证据文件：

```text
course_design/results/task7_rl_velocity_eval/01_a1_training_reward_length.png
course_design/results/task7_rl_velocity_eval/02_a1_reward_components.png
course_design/results/task7_rl_velocity_eval/03_a1_loss_stats.png
course_design/results/task7_rl_velocity_eval/a1_training_summary.json
```

图像含义：

1. `01_a1_training_reward_length.png`：平均回报和平均 episode length 随训练迭代上升。
2. `02_a1_reward_components.png`：线速度跟踪、角速度跟踪、接触和抬脚等奖励项变化。
3. `03_a1_loss_stats.png`：PPO 中 value loss、surrogate、entropy 和策略方差变化。

## 3. A1 仿真评估

任务 7 的 ROS2 评估脚本为：

```text
course_design/task7_rl_velocity_eval/ros2_velocity_rmse_eval.py
```

本次评估使用的策略文件为：

```text
/home/lcw/rl_sar_ws/src/rl_sar/policy/a1/legged_gym/model.pt
```

本次采用仓库现有 launch：

```bash
ros2 launch rl_sar a1_nav2_sim.launch.py world:=a1_nav_world_open use_rviz:=false
```

评估命令通过 `/cmd_vel` 发送分段速度参考，读取 `/odom` 计算误差，输出：

```text
course_design/results/task7_rl_velocity_eval/rl_velocity_rmse.csv
course_design/results/task7_rl_velocity_eval/rl_velocity_tracking.png
course_design/results/task7_rl_velocity_eval/task7_summary.json
```

本次评估结果：

```text
vx RMSE = 0.060204 m/s
wz RMSE = 0.189228 rad/s
```

其中：

```text
vx  = base linear velocity in x
wz  = base yaw angular velocity
```

## 4. 可直接用于报告的结论

本项目在任务 7 中采用强化学习无模型控制思路完成速度跟踪。训练过程图显示平均回报随训练推进明显提升，平均 episode length 收敛到 `1000`，说明策略逐步稳定。部署阶段使用当前 `rl_sar` 仓库中的 A1 `legged_gym` policy，在 Gazebo + ROS2 仿真中执行 `/cmd_vel` 速度指令跟踪，并通过 `/odom` 计算误差。实验结果表明，A1 策略在开放场景测试轨迹上的线速度 RMSE 为 `0.0602 m/s`，角速度 RMSE 为 `0.1892 rad/s`，说明强化学习策略能够在不显式依赖动力学模型参数的条件下实现较稳定的速度跟踪能力。

在整机导航实验中，强化学习策略能够作为四足机器人底层速度执行器完成 Nav2 目标导航，但由于策略训练目标主要是速度跟踪和稳定步态，而非目标点精确收敛，且四足步态存在离散落足、低速微调困难和角速度跟踪误差，因此最终目标点精度稳定在约 `0.17 m`。通过使用 ground-truth odom、低速 Nav2 参数、限制横向速度、放宽 yaw 判定并收紧 XY 容差，系统实现了空场多目标导航任务的稳定完成。

## 5. 报告中的表述建议

建议在最终报告中明确写：

```text
当前 A1 仿真评估使用 rl_sar 现有 A1 policy；
训练过程图作为 A1 速度跟踪训练证据展示；
仿真启动采用仓库现有 `a1_nav2_sim.launch.py`，并切换到 `a1_nav_world_open` 开放场景。
```
