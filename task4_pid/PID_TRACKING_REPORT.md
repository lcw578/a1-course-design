# 任务 4：多项式轨迹规划与速度 PID 控制

## 1. 任务目标与建模边界

课程任务 4 要求设计理想参考轨迹，实现基于速度控制的速度跟踪，并绘制跟踪误差随时间变化曲线。评分点包括：

```text
轨迹规划 3 分
PID 控制器实现 3 分
跟踪误差分析 2 分
控制器参数整定 2 分
```

本项目使用 A1 四足机器狗的 FR 单腿 3R 机构作为控制对象，关节变量为：

```text
q = [q_hip, q_thigh, q_calf]^T
```

这里不直接用简单 PID 替换整机 A1 行走控制器。原因是 A1 整机包含浮动基座、足端接触和 12 个关节，稳定行走通常需要全身控制或强化学习策略。任务 4 的目标是验证课程要求的“传统速度 PID 控制律”，因此采用单腿关节空间轨迹跟踪作为可复现的课程实验；整机强化学习速度跟踪放在任务 7 中处理。

## 2. 关节空间五次多项式轨迹

初始关节角和目标关节角设置为：

```text
q0 = [0.0, 0.8, -1.5]^T rad
qf = [0.18, 1.05, -1.25]^T rad
T  = 4.0 s
dt = 0.002 s
```

每个关节采用五次多项式：

```text
theta_d(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4 + a5 t^5
```

边界条件为：

```text
theta_d(0)  = theta_0
theta_d(T)  = theta_f
theta_dot_d(0) = theta_dot_d(T) = 0
theta_ddot_d(0) = theta_ddot_d(T) = 0
```

在零初末速度、零初末加速度条件下，系数可写为：

```text
a0 = theta_0
a1 = 0
a2 = 0
a3 = 10(theta_f - theta_0) / T^3
a4 = -15(theta_f - theta_0) / T^4
a5 = 6(theta_f - theta_0) / T^5
```

本实验得到的三个关节多项式系数为：

```text
hip:   [ 0.000000, 0, 0, 0.028125, -0.010547, 0.001055]
thigh: [ 0.800000, 0, 0, 0.039063, -0.014648, 0.001465]
calf:  [-1.500000, 0, 0, 0.039063, -0.014648, 0.001465]
```

## 3. 速度 PID 控制律

任务要求的速度 PID 控制律为：

```text
theta_dot_cmd =
    Kp (theta_d - theta)
  + Kd (theta_dot_d - theta_dot)
  + Ki integral(theta_d - theta) dt
```

本项目的向量形式为：

```text
qdot_cmd = Kp e_q + Kd e_dq + Ki integral(e_q) dt
```

其中：

```text
e_q  = qd - q
e_dq = dqd - dq
```

为了模拟真实速度执行器的响应延迟，加入一阶执行器模型：

```text
dq_dot = (qdot_cmd - dq) / tau
```

其中：

```text
tau = [0.045, 0.055, 0.055] s
```

速度命令限幅为：

```text
|qdot_cmd| <= [4.0, 5.0, 5.0] rad/s
```

积分项使用限幅防止 windup：

```text
|integral(e_q)| <= 0.35
```

## 4. 参数整定实验

本任务对比了 5 组控制参数：

```text
low_pd
medium_pd
medium_pid
final_pid
high_pid
```

参数和误差结果保存在：

```text
course_design/results/task4_pid/pid_tuning_sweep.csv
```

调参对比图为：

```text
course_design/results/task4_pid/pid_tuning_comparison.png
```

对比结果如下：

| profile | q RMSE rad | dq RMSE rad/s | foot RMSE m | final q error rad |
|---|---:|---:|---:|---:|
| low_pd | 0.036105 | 0.029559 | 0.011910 | 0.006249 |
| medium_pd | 0.018344 | 0.015778 | 0.006046 | 0.000817 |
| medium_pid | 0.013629 | 0.012484 | 0.004482 | 0.001496 |
| final_pid | 0.012142 | 0.011457 | 0.003976 | 0.002261 |
| high_pid | 0.008152 | 0.007699 | 0.002666 | 0.001475 |

从名义一阶执行器模型看，增大 PID 参数可以降低跟踪误差，`high_pid` 的 RMSE 最低。但是该模型没有包含真实机器狗中的关节力矩限制、测量噪声、通信延迟、足端接触冲击和机身姿态耦合。因此报告中选择 `final_pid` 作为保守参数，用于展示稳定、平滑的速度 PID 跟踪效果。

最终选用：

```text
Kp = [9.0, 10.0, 8.0]
Ki = [0.6, 0.6, 0.5]
Kd = [0.25, 0.28, 0.22]
```

## 5. 最终跟踪结果

`final_pid` 的误差指标为：

```text
position RMSE       = 0.0121416713 rad
velocity RMSE       = 0.0114574351 rad/s
foot position RMSE  = 0.0039756675 m
max position error  = 0.0193934999 rad
max velocity error  = 0.0164954670 rad/s
final position err  = 0.0022605745 rad
final velocity err  = 0.0032626324 rad/s
final foot err      = 0.0007459231 m
saturation ratio    = 0.0
```

说明在该关节空间轨迹下，速度命令没有触发限幅，关节位置和速度误差均较小，最终足端误差小于 1 mm。

## 6. 输出图表

任务四脚本：

```text
course_design/task4_pid/run_task4_pid_tracking.py
```

输出文件：

```text
course_design/results/task4_pid/task4_summary.json
course_design/results/task4_pid/velocity_pid_tracking.csv
course_design/results/task4_pid/pid_tuning_sweep.csv
course_design/results/task4_pid/pid_tuning_comparison.png
course_design/results/task4_pid/velocity_pid_error.png
course_design/results/task4_pid/joint_position_tracking.png
course_design/results/task4_pid/joint_velocity_tracking.png
course_design/results/task4_pid/velocity_pid_command.png
course_design/results/task4_pid/foot_path_tracking_xz.png
```

建议报告中放入以下图片：

1. `pid_tuning_comparison.png`：展示控制器参数整定过程。
2. `joint_position_tracking.png`：展示三个关节实际位置跟踪期望位置。
3. `joint_velocity_tracking.png`：展示期望速度、实际速度和 PID 速度命令。
4. `velocity_pid_error.png`：展示位置误差、速度误差和足端误差随时间变化。
5. `foot_path_tracking_xz.png`：展示关节跟踪在足端空间产生的轨迹。

## 7. 报告结论

本任务基于 A1 单腿 3R 机构设计了关节空间五次多项式轨迹，并实现速度 PID 闭环跟踪。控制器直接输出关节速度命令，执行器采用一阶速度响应模型。通过多组 PID 参数对比，验证了低增益会导致较大的跟踪滞后，适当提高比例、微分和积分增益可以降低位置与速度误差。最终选用的 `final_pid` 参数在不触发速度限幅的情况下实现了稳定跟踪，关节位置 RMSE 为 `0.01214 rad`，足端位置 RMSE 为 `3.98 mm`，满足任务四对轨迹规划、PID 控制、误差曲线和参数整定分析的要求。
