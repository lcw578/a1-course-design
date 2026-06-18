# Nav2 目标点评估

这个目录补充课程设计里更接近老师评分口径的**目标点导航误差**实验。它不替代 `task7_rl_velocity_eval/` 的底层速度跟踪 RMSE，而是把整条导航链路跑通后，直接统计 Nav2 导航到目标点的最终位置误差。

## 使用的现有仓库链路

当前推荐直接复用现有 `a1_nav2_sim.launch.py`，保留 Nav2，但不使用 AMCL，也不引入状态估计误差：

课程提交仓库中保留了对应源码子集：

```text
source_code/rl_sar_a1_runtime/launch/a1_nav2_sim.launch.py
source_code/rl_sar_a1_runtime/scripts/gazebo_ground_truth_odom.py
source_code/rl_sar_a1_runtime/scripts/evaluate_nav2_goal_sequence.py
source_code/rl_sar_a1_runtime/config/a1_nav2_ground_truth_refined.yaml
source_code/rl_sar_a1_runtime/maps/a1_nav_world_empty.yaml
```

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

该 launch 的核心链路是：

1. Gazebo `ground_truth/odom_raw`
2. `gazebo_ground_truth_odom.py` 直接发布 `/odom` 给 Nav2
3. 同时发布 `/odom_gt` 供评估脚本做真值对照
4. `map_server + navigation_launch.py`
5. `/cmd_vel -> rl_sim -> A1 legged_gym policy -> robot_joint_controller`

## 运行目标点误差评估

另开终端运行仓库现有脚本：

```bash
source /opt/ros/humble/setup.bash
source /home/lcw/rl_sar_ws/install/setup.bash
python3 src/rl_sar/src/rl_sar/scripts/evaluate_nav2_goal_sequence.py --ros-args \
  -p output_root:=/home/lcw/rl_sar_ws/course_design/results/task2_nav2_goal_eval \
  -p run_name:=a1_nav2_ground_truth_refined017_yaw08_turning_clean \
  -p odom_topic:=/odom \
  -p ground_truth_topic:=/odom_gt \
  -p base_frame:=base \
  -p goal_timeout_sec:=120.0 \
  -p goal_pause_sec:=2.0 \
  -p goal_specs:="['straight_1p5,1.5,0.0,0.0','left_turn_2p4,2.4,0.35,0.35','straight_after_turn_3p3,3.3,0.55,0.10','right_correction_4p1,4.1,0.25,-0.20']"
```

输出：

```text
course_design/results/task2_nav2_goal_eval/a1_nav2_ground_truth_refined017_yaw08_turning_clean/
  summary.json
  samples.csv
```

其中最重要的是：

- `final_gt_goal_xy_error_m`：真值轨迹下最终目标点误差
- `final_raw_goal_xy_error_m`：Nav2 实际使用 `/odom` 时的最终目标点误差

## 生成报告图

```bash
python3 course_design/task2_nav2_goal_eval/postprocess_nav2_goal_eval.py \
  --input-dir /home/lcw/rl_sar_ws/course_design/results/task2_nav2_goal_eval/a1_nav2_ground_truth_refined017_yaw08_turning_clean \
  --map-yaml /home/lcw/rl_sar_ws/src/rl_sar/src/rl_sar/maps/a1_nav_world_empty.yaml
```

输出：

- `01_path_overview.png`
- `02_goal_error_timeseries.png`
- `03_goal_summary.png`
- `NAV2_GOAL_EVAL_REPORT.md`

## 说明

这个实验更适合在最终报告中回答“导航目标点误差”这一类问题；而 `task7_rl_velocity_eval/` 更适合回答“底层强化学习速度跟踪控制效果”这一类问题。两个指标不是一回事，报告里建议同时保留，但目标点误差应作为 Nav2 全流程结果单独呈现。

当前推荐结果为 `a1_nav2_ground_truth_refined017_yaw08_turning_clean`：`xy_goal_tolerance=0.17 m`、`yaw_goal_tolerance=0.80 rad`，四个带转向偏移的目标全部 `SUCCEEDED`，平均真值目标点误差为 `0.167 m`。更紧的 `0.16 m` 和 `0.12 m` 对照组会出现目标超时，不建议作为正式结果。

强化学习策略能够作为四足机器人底层速度执行器完成 Nav2 目标导航，但由于策略训练目标主要是速度跟踪和稳定步态，而非目标点精确收敛，且四足步态存在离散落足、低速微调困难和角速度跟踪误差，因此最终目标点精度稳定在约 `0.17 m`。通过使用 ground-truth odom、低速 Nav2 参数、限制横向速度、放宽 yaw 判定并收紧 XY 容差，系统实现了空场多目标导航任务的稳定完成。
