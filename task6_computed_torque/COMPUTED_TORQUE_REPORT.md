# 任务 6：基于动力学模型的力/力矩控制

## 1. 建模对象与任务边界

任务 6 延续任务 5 的动力学模型，采用 A1 单腿 `thigh-calf` 的 2R 矢状面子系统作为控制对象。这里不直接对整机 A1 做 torque-level 全身控制，原因是整机模型包含浮动基座、足端接触和下层稳定控制链路，不适合在课程设计中用简化 computed torque 直接替代。

因此，本任务比较的是：

```text
同一 2R 动力学模型下的速度 PID 基线
vs
同一 2R 动力学模型下的 computed torque 控制
```

这与任务 4 的 3R 单腿速度 PID 不完全同维，但在任务 6 中必须这样处理，才能保证动力学模型 `M(q), C(q,dq), g(q)` 与控制律严格一致。

## 2. 控制律

逆动力学控制采用课程 PDF 给出的形式：

```text
tau = M(q) a + C(q,dq) + g(q)
```

其中辅助加速度为：

```text
a = ddq_d + Kv (dq_d - dq) + Kp (q_d - q)
```

若采用对角增益，并令闭环为临界阻尼 `zeta = 1`，可令：

```text
Kp = wn^2
Kv = 2 wn
```

本实验选取：

```text
wn = [12, 12] rad/s
Kp = [144, 144]
Kv = [24, 24]
zeta = 1
```

这意味着理想闭环误差动力学接近：

```text
e_ddot + Kv e_dot + Kp e = 0
```

在名义模型准确时，跟踪误差会快速收敛。

## 3. 对比基线

为了与任务 4 的传统控制思想形成对照，这里保留一个 2R 速度 PID 基线：

```text
qdot_cmd = Kp (q_d - q) + Kd (dq_d - dq) + Ki integral(q_d - q) dt
```

然后通过一阶速度执行器近似得到 `ddq`。这不是 torque-level 控制，但可作为传统方法基线，用于体现逆动力学补偿的收益。

## 4. 轨迹与仿真参数

参考轨迹仍为关节空间五次多项式：

```text
q0 = [0.75, -1.35]^T rad
qf = [0.95, -1.05]^T rad
T  = 3.5 s
dt = 0.001 s
```

输出文件由脚本生成：

```text
course_design/task6_computed_torque/run_task6_computed_torque.py
```

## 5. 名义模型对比结果

当前结果见：

```text
course_design/results/task6_computed_torque/task6_summary.json
```

名义模型下：

| controller | q RMSE rad | dq RMSE rad/s | max abs tau Nm |
|---|---:|---:|---:|
| velocity_pid | 0.015978 | 0.016541 | 0.000000 |
| computed_torque | 0.000125 | 0.000067 | 1.101412 |

由此可见：

```text
computed torque / PID position RMSE ratio = 0.00783
position RMSE improvement                 = 99.22%
```

这说明在模型匹配时，computed torque 的跟踪精度明显优于速度 PID。

## 6. 鲁棒性分析：质量不确定度 ±20%

为了满足课程要求，控制器仍使用名义动力学模型，但被控对象质量参数分别扰动为：

```text
m1, m2 -> 1.2 * nominal
m1, m2 -> 0.8 * nominal
```

结果如下：

| controller | q RMSE rad | dq RMSE rad/s | final q error rad |
|---|---:|---:|---:|
| computed_torque | 0.000125 | 0.000067 | 0.000006 |
| computed_torque_mass_plus20 | 0.061749 | 0.065339 | 0.053465 |
| computed_torque_mass_minus20 | 0.051992 | 0.060069 | 0.044330 |

结论：

1. 在名义模型准确时，computed torque 跟踪效果极好。
2. 一旦质量参数失配，控制器性能显著下降。
3. `+20%` 质量失配在本实验中导致更大的位置 RMSE。
4. `-20%` 质量失配虽然略轻，但也会因为逆动力学补偿不准确而引入明显误差。

这说明 computed torque 对模型精度比较敏感，鲁棒性弱于不依赖动力学模型的简单速度控制。

## 7. 图表与报告建议

结果文件：

```text
course_design/results/task6_computed_torque/controller_comparison.csv
course_design/results/task6_computed_torque/controller_comparison.png
course_design/results/task6_computed_torque/computed_torque_robustness.png
course_design/results/task6_computed_torque/control_output_comparison.png
course_design/results/task6_computed_torque/task6_summary.json
```

建议报告放图：

1. `controller_comparison.png`：展示 PID 与 computed torque 的位置/速度误差对比。
2. `computed_torque_robustness.png`：展示 `nominal / +20% / -20%` 的鲁棒性差异。
3. `control_output_comparison.png`：展示 computed torque 的力矩输出以及 PID 的速度命令输出。

## 8. 报告结论

本任务基于任务 5 的 2R 单腿动力学模型实现了逆动力学控制器，控制律满足课程要求的：

```text
tau = M(q)a + C(q,dq) + g(q)
```

并通过 `Kp = wn^2`、`Kv = 2wn` 使闭环近似临界阻尼。仿真结果表明，在名义动力学模型准确时，computed torque 的跟踪性能明显优于速度 PID；但在质量参数存在 `±20%` 不确定度时，跟踪误差显著增大，说明该方法对模型失配较敏感。这一结果符合逆动力学控制“高精度、低鲁棒性”的典型特点。
