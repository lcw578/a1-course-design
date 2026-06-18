# A1 四足机器狗课程设计实现

本目录补齐《机器人驱动与控制》课程设计中当前仓库缺少的理论建模、控制和评估脚本。现有 `rl_sar` 中已经完成的 A1 URDF、Gazebo、ROS2 control、RL policy、Nav2 和状态估计文件不重复实现，只在这里新增课程作业专用实验。

为满足课程提交中“源代码（含 URDF 文件及控制程序）”的要求，`source_code/` 额外保留了本项目直接使用的 A1 最小源码子集，包括 A1 URDF/mesh、ROS 2 joint controller、消息定义、A1 policy、Gazebo/Nav2 launch、ground-truth odom 和目标点评估脚本。该目录不是完整上游 `rl_sar` 仓库，只包含本课程设计相关内容。

## 任务映射

| 作业任务 | 本目录实现 | 说明 |
| --- | --- | --- |
| 任务 1 运动学建模 | `task1_kinematics/` | A1 单腿 3R 足端 FK、数值 IK、工作空间 Monte Carlo |
| 任务 2 仿真搭建 | `task2_simulation_reuse/` | 复用现有 A1 URDF、Gazebo、ros2_control、LiDAR、RViz/Nav2 |
| 任务 2 补充：Nav2 目标点评估 | `task2_nav2_goal_eval/` | 使用 Nav2 全流程统计最终目标点误差；当前推荐直接使用 Gazebo ground-truth odom，排除状态估计误差 |
| 任务 3 雅可比分析 | `task3_jacobian/` | 足端几何雅可比 `Jv`、空间雅可比 `Js`、体雅可比 `Jb`、奇异性扫描、`tau = J^T F` |
| 任务 4 速度 PID | `task4_pid/` | 五次多项式关节轨迹、速度 PID、参数整定 sweep、逐关节/足端误差分析 |
| 任务 5 动力学验证 | `task5_dynamics/` | 单腿矢状面 2R 简化动力学、`M/C/g` 推导、残差验证、动力学项贡献分析 |
| 任务 6 逆动力学力控制 | `task6_computed_torque/` | computed torque 与 2R 速度 PID 对比，含质量 `±20%` 鲁棒性测试和力矩输出分析 |
| 任务 7 强化学习速度跟踪 | `task7_rl_velocity_eval/` | A1 policy 在 open world 下的 Gazebo 速度 RMSE 评估，附 A1 训练过程曲线证据 |

## 运行离线任务

在工作区根目录运行：

```bash
python3 course_design/task1_kinematics/run_task1_kinematics.py
python3 course_design/task3_jacobian/run_task3_jacobian.py
python3 course_design/task4_pid/run_task4_pid_tracking.py
python3 course_design/task5_dynamics/run_task5_dynamics_validation.py
python3 course_design/task6_computed_torque/run_task6_computed_torque.py
```

结果会写入：

```text
course_design/results/
```

## 运行 RL 速度跟踪评估

先启动现有 A1 仿真：

```bash
source /opt/ros/humble/setup.bash
source /home/lcw/rl_sar_ws/install/setup.bash
ros2 launch rl_sar a1_nav2_sim.launch.py world:=a1_nav_world_open use_rviz:=false
```

另开终端运行：

```bash
source /opt/ros/humble/setup.bash
source /home/lcw/rl_sar_ws/install/setup.bash
python3 course_design/task7_rl_velocity_eval/ros2_velocity_rmse_eval.py
```

该脚本会发布一段速度命令，读取 `/odom`，输出线速度和角速度 RMSE，并保存 CSV 和曲线。当前推荐使用 open world，避免默认障碍物场景干扰速度跟踪评估。

## 运行 Nav2 目标点评估

当前推荐直接使用仓库现有 `a1_nav2_sim.launch.py`，保留 Nav2，但不启用 AMCL，也不引入状态估计误差。为避免场景障碍物干扰，使用 `earth.world + a1_nav_world_empty.yaml`：

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

另开终端运行：

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

python3 course_design/task2_nav2_goal_eval/postprocess_nav2_goal_eval.py \
  --input-dir /home/lcw/rl_sar_ws/course_design/results/task2_nav2_goal_eval/a1_nav2_ground_truth_refined017_yaw08_turning_clean \
  --map-yaml /home/lcw/rl_sar_ws/src/rl_sar/src/rl_sar/maps/a1_nav_world_empty.yaml
```

这个实验输出的是 Nav2 任务级指标，也就是最终目标点误差、路径长度和导航耗时。由于 `/odom` 直接来自 Gazebo ground truth，这组结果反映的是 `Nav2 + RL 执行器` 的任务级误差，而不是定位误差。当前 refined 结果使用 `xy_goal_tolerance=0.17 m`、`yaw_goal_tolerance=0.80 rad`，四个目标全部成功，平均真值目标点误差为 `0.167 m`。对比实验表明 `0.16 m` 和 `0.12 m` 容差会出现超时，因此 `0.17 m` 是当前更稳妥的收紧档，目标序列包含左转和右修正。

强化学习策略能够作为四足机器人底层速度执行器完成 Nav2 目标导航，但由于策略训练目标主要是速度跟踪和稳定步态，而非目标点精确收敛，且四足步态存在离散落足、低速微调困难和角速度跟踪误差，因此最终目标点精度稳定在约 `0.17 m`。通过使用 ground-truth odom、低速 Nav2 参数、限制横向速度、放宽 yaw 判定并收紧 XY 容差，系统实现了空场多目标导航任务的稳定完成。

## 建模边界

课程作业需要 DH、雅可比和动力学推导。A1 整机是浮动基座 12 关节系统，完整动力学推导超出普通课程设计范围。因此这里采用：

1. 运动学/雅可比：A1 单腿 3R 串联机构，足端作为末端执行器。
2. 动力学/逆动力学控制：A1 单腿矢状面 2R 子系统，便于清晰推导 `M(q)ddq + C(q,dq) + g(q) = tau`。
3. 强化学习：复用现有 `rl_sar` A1 预训练策略，评估整机速度跟踪。

这套边界既能匹配评分点，也能保留机器狗项目特色。
