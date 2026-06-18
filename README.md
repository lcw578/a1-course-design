# A1 四足机器人驱动与控制课程设计

本仓库用于提交《机器人驱动与控制》课程设计。原作业建议使用二/三自由度机械臂，也允许选择移动机器人或其他机器人；本项目选择 Unitree A1 四足机器狗作为对象，将课程要求映射到“单腿机构建模 + 整机 Gazebo/Nav2/RL 仿真”的完整实验链路。

## 提交内容

| 目录 | 内容 |
| --- | --- |
| `source_code/` | A1 相关最小源码子集，包含 URDF、mesh、ROS 2 控制器、消息定义、A1 policy、Gazebo/Nav2 launch 和评估脚本。 |
| `task1_kinematics/` | A1 单腿 3R 运动学、FK/IK、工作空间分析。 |
| `task2_simulation_reuse/` | A1 ROS 2/Gazebo 仿真搭建说明。 |
| `task2_nav2_goal_eval/` | Nav2 多目标导航误差评估脚本与后处理。 |
| `task3_jacobian/` | 足端雅可比、空间/体雅可比、奇异性和静力映射。 |
| `task4_pid/` | 五次多项式轨迹规划与速度 PID 跟踪。 |
| `task5_dynamics/` | A1 单腿矢状面 2R 动力学建模与验证。 |
| `task6_computed_torque/` | 基于 2R 动力学模型的 computed torque 控制。 |
| `task7_rl_velocity_eval/` | A1 强化学习策略速度跟踪评估。 |
| `results/` | 任务 1-7 的 CSV、JSON、PNG 和 Markdown 实验结果。 |

`source_code/` 不是完整上游 `rl_sar` 仓库，而是只保留本课程设计直接使用的 A1 相关源码。这样既满足“源代码（含 URDF 文件及控制程序）”要求，也避免上传与本项目无关的机器人和第三方大包。

## 任务映射

| 作业任务 | 本项目实现 | 主要结果 |
| --- | --- | --- |
| 任务 1：运动学建模 | A1 前右腿 3R 串联机构；URDF 等效 DH/MDH；FK、IK、工作空间 Monte Carlo。 | FK-IK 最大闭环误差 `9.90e-09 m`。 |
| 任务 2：仿真搭建 | A1 URDF、mesh、Gazebo、ROS 2 control、RViz/Nav2 可视化和 `/cmd_vel` 速度接口。 | RViz/Gazebo 截图与 ros2_control/topic 证据已保存。 |
| 任务 3：雅可比分析 | 足端线速度雅可比 `Jv`、空间雅可比 `Js`、体雅可比 `Jb`、奇异性扫描、`tau = J^T F`。 | 有限差分验证误差约 `1e-10`。 |
| 任务 4：速度 PID | 五次多项式关节轨迹；速度 PID；参数整定 sweep；误差曲线。 | 位置 RMSE `0.01214 rad`，足端 RMSE `3.98 mm`。 |
| 任务 5：动力学验证 | A1 单腿 thigh-calf 2R 矢状面动力学，推导 `M(q)ddq + C(q,dq) + g(q) = tau`。 | 动力学残差约 `1e-15`。 |
| 任务 6：逆动力学控制 | computed torque、临界阻尼增益、PID 对比、质量 `±20%` 鲁棒性。 | 名义模型下 computed torque 相对 PID 位置 RMSE 改善约 `99.22%`。 |
| 任务 7：强化学习控制 | A1 `legged_gym` policy 作为底层速度执行器；Gazebo 速度跟踪和 Nav2 目标点评估。 | `vx` RMSE `0.0602 m/s`，Nav2 平均目标点误差 `0.167 m`。 |

## 源码链路

课程报告中的整机仿真控制链路为：

```text
Nav2 /cmd_vel
  -> source_code/rl_sar_a1_runtime/src/rl_sim.cpp
  -> source_code/a1_policy/legged_gym/model.pt
  -> robot_msgs/RobotCommand
  -> source_code/robot_joint_controller
  -> source_code/a1_description/urdf/a1_description.urdf
  -> Gazebo A1 robot
```

其中：

- URDF 与 mesh：`source_code/a1_description/`
- ROS 2 关节控制器：`source_code/robot_joint_controller/`
- 控制消息：`source_code/robot_msgs/`
- A1 RL policy：`source_code/a1_policy/`
- Nav2/Gazebo/A1 runtime：`source_code/rl_sar_a1_runtime/`

## 运行离线任务

这些任务不依赖 ROS，可以直接在仓库根目录运行：

```bash
python3 task1_kinematics/run_task1_kinematics.py
python3 task3_jacobian/run_task3_jacobian.py
python3 task4_pid/run_task4_pid_tracking.py
python3 task5_dynamics/run_task5_dynamics_validation.py
python3 task6_computed_torque/run_task6_computed_torque.py
```

输出结果写入：

```text
results/
```

## 运行整机仿真实验

整机 Gazebo/Nav2/RL 实验依赖完整 ROS 2 Humble 工作区和 `rl_sar` 运行环境。本仓库的 `source_code/` 保留了关键源码和配置，实际复现实验时需要在原 ROS 2 工作区中构建相关包。

启动 A1 Nav2 仿真：

```bash
source /opt/ros/humble/setup.bash
source /home/lcw/rl_sar_ws/install/setup.bash
ros2 launch rl_sar a1_nav2_sim.launch.py \
  rname:=a1 \
  world:=earth \
  map_yaml:=/home/lcw/rl_sar_ws/src/rl_sar/src/rl_sar/maps/a1_nav_world_empty.yaml \
  nav2_params:=/home/lcw/rl_sar_ws/src/rl_sar/src/rl_sar/config/a1_nav2_ground_truth_refined.yaml \
  use_rviz:=false
```

运行 Nav2 目标点评估：

```bash
source /opt/ros/humble/setup.bash
source /home/lcw/rl_sar_ws/install/setup.bash
python3 src/rl_sar/src/rl_sar/scripts/evaluate_nav2_goal_sequence.py --ros-args \
  -p output_root:=/home/lcw/rl_sar_ws/course_design/results/task2_nav2_goal_eval \
  -p run_name:=a1_nav2_ground_truth_refined017_yaw08_turning_clean \
  -p odom_topic:=/odom \
  -p ground_truth_topic:=/odom_gt \
  -p cmd_vel_topic:=/cmd_vel \
  -p base_frame:=base \
  -p goal_timeout_sec:=120.0 \
  -p goal_pause_sec:=2.0 \
  -p goal_specs:="['straight_1p5,1.5,0.0,0.0','left_turn_2p4,2.4,0.35,0.35','straight_after_turn_3p3,3.3,0.55,0.10','right_correction_4p1,4.1,0.25,-0.20']"
```

生成 Nav2 报告图：

```bash
python3 task2_nav2_goal_eval/postprocess_nav2_goal_eval.py \
  --input-dir results/task2_nav2_goal_eval/a1_nav2_ground_truth_refined017_yaw08_turning_clean \
  --map-yaml /home/lcw/rl_sar_ws/src/rl_sar/src/rl_sar/maps/a1_nav_world_empty.yaml
```

运行 RL 速度跟踪评估：

```bash
source /opt/ros/humble/setup.bash
source /home/lcw/rl_sar_ws/install/setup.bash
python3 task7_rl_velocity_eval/ros2_velocity_rmse_eval.py
```

## 建模边界

A1 整机是浮动基座、12 关节、多足接触系统，完整解析动力学超出普通课程设计范围。因此本项目采用分层建模：

1. 运动学与雅可比：A1 单腿 3R 串联机构，足端作为末端执行器。
2. 动力学与逆动力学控制：A1 单腿矢状面 2R thigh-calf 子系统，固定基座、无足端接触。
3. 整机控制与导航：A1 预训练强化学习策略作为底层速度执行器，Nav2 负责目标点导航。

这个边界既覆盖课程评分点，也保留了四足机器人项目的工程特色。报告中不将 RL 控制器描述为完美目标点控制器，而是说明其主要能力是速度跟踪和稳定步态执行。

## 推荐报告引用结果

| 指标 | 数值 |
| --- | ---: |
| FK-IK 最大闭环误差 | `9.896e-09 m` |
| Task 4 PID 位置 RMSE | `0.01214 rad` |
| Task 4 足端 RMSE | `0.00398 m` |
| Task 5 动力学最大残差 | `3.997e-15` |
| Task 6 computed torque 位置 RMSE | `0.000125 rad` |
| Task 7 `vx` RMSE | `0.0602 m/s` |
| Nav2 平均目标点误差 | `0.167 m` |

## 说明

本仓库包含课程设计源代码、模型文件、控制程序、实验脚本和结果材料；最终 15-20 页 PDF 报告可基于本仓库中的 Markdown、图表和结果数据整理生成。
