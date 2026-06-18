# 任务 5：A1 单腿 2R 矢状面动力学推导与仿真验证

## 1. 建模对象与简化边界

本任务不对整只 A1 四足机器狗建立完整动力学，而选择其单腿在矢状面内的 `thigh-calf` 2R 子系统作为动力学建模对象。这样做的原因是：

1. A1 整机是浮动基座、12 关节、多足接触约束系统，完整动力学应包含基座自由度和接触力，不适合本课程设计直接推导完整解析式。
2. 课程任务 5 要求的是开链动力学模型 `M(q)ddq + C(q,dq) + g(q) = tau`、仿真验证和动力学项分析。
3. A1 单腿的 `thigh` 和 `calf` 关节构成标准 2R 串联机构，可以清晰推导，并与任务 6 的 computed torque 控制自然衔接。

因此本任务采用以下假设：

```text
1. 忽略 hip abduction/adduction 关节，只保留矢状面内两个 pitch 关节。
2. 机器人机身视为固定基座。
3. 不考虑足端与地面接触冲击，动力学验证采用离线开链仿真。
```

## 2. 广义坐标与参数定义

广义坐标定义为：

```text
q = [q1, q2]^T
```

其中：

```text
q1: thigh 关节角
q2: calf 相对 thigh 的关节角
```

模型参数来自 [planar_leg_dynamics.py](/home/lcw/rl_sar_ws/course_design/common/planar_leg_dynamics.py:1)：

```text
l1  = 0.2 m      thigh 长度
l2  = 0.2 m      calf 长度
m1  = 1.013 kg   thigh 等效质量
m2  = 0.166 kg   calf 等效质量
lc1 = 0.1 m      thigh 质心到关节 1 的距离
lc2 = 0.1 m      calf 质心到关节 2 的距离
I1  = 0.005139339 kg m^2
I2  = 0.003014022 kg m^2
g   = 9.81 m/s^2
```

## 3. 动力学方程

动力学方程写为：

```text
M(q) ddq + C(q,dq) + g(q) = tau
```

这里：

```text
q   = [q1, q2]^T
dq  = [dq1, dq2]^T
ddq = [ddq1, ddq2]^T
tau = [tau1, tau2]^T
```

## 4. 质量矩阵 M(q)

定义：

```text
c2 = cos(q2)
```

则质量矩阵为：

```text
M(q) =
[
  I1 + I2 + m1 lc1^2 + m2 (l1^2 + lc2^2 + 2 l1 lc2 c2),   I2 + m2 (lc2^2 + l1 lc2 c2)
  I2 + m2 (lc2^2 + l1 lc2 c2),                             I2 + m2 lc2^2
]
```

即：

```text
M11 = I1 + I2 + m1 lc1^2 + m2 (l1^2 + lc2^2 + 2 l1 lc2 cos q2)
M12 = I2 + m2 (lc2^2 + l1 lc2 cos q2)
M22 = I2 + m2 lc2^2
```

## 5. 科里奥利/离心项 C(q,dq)

定义：

```text
s2 = sin(q2)
h  = m2 l1 lc2 s2
```

则向量形式的科里奥利/离心项写为：

```text
C(q,dq) =
[
  -h (2 dq1 dq2 + dq2^2)
   h dq1^2
]^T
```

也可以理解为把常见的 `C(q,dq)dq` 已经合并为一个 2 维向量。

## 6. 重力项 g(q)

重力项为：

```text
g1 = (m1 lc1 + m2 l1) g cos(q1) + m2 lc2 g cos(q1 + q2)
g2 = m2 lc2 g cos(q1 + q2)
```

即：

```text
g(q) = [g1, g2]^T
```

## 7. 逆动力学与前向动力学

逆动力学：

```text
tau = M(q) ddq + C(q,dq) + g(q)
```

前向动力学：

```text
ddq = M(q)^(-1) [tau - C(q,dq) - g(q)]
```

任务 5 脚本同时实现了这两个方向，并通过数值残差检查二者是否一致。

## 8. 仿真验证方法

验证分为两部分：

### 8.1 随机状态残差验证

随机采样 `(q, dq, tau)`，用前向动力学求得 `ddq`，再回代：

```text
r = M(q) ddq + C(q,dq) + g(q) - tau
```

若模型实现正确，则残差 `r` 应接近 0。

### 8.2 受迫响应与动力学项贡献分析

对系统施加时变关节力矩：

```text
tau1(t) = 1.2 sin(2 pi 0.7 t)
tau2(t) = 0.8 cos(2 pi 0.5 t)
```

然后记录：

```text
q(t), dq(t), ddq(t), tau(t), M(q)ddq, C(q,dq), g(q)
```

并比较：

```text
tau(t)  与  M(q)ddq + C(q,dq) + g(q)
```

这样可以直接展示各动力学项对关节力矩的贡献。

## 9. 已生成结果文件

脚本：

```text
course_design/task5_dynamics/run_task5_dynamics_validation.py
```

结果：

```text
course_design/results/task5_dynamics/task5_summary.json
course_design/results/task5_dynamics/inverse_forward_dynamics_residual.csv
course_design/results/task5_dynamics/forced_response.csv
course_design/results/task5_dynamics/forced_response.png
course_design/results/task5_dynamics/dynamics_term_contributions.png
course_design/results/task5_dynamics/dynamics_residual_validation.png
```

## 10. 报告中应强调的结论

1. 本任务采用 A1 单腿 2R 矢状面动力学模型，而非整机浮动基座动力学。
2. `M(q)`、`C(q,dq)`、`g(q)` 已给出完整解析表达式。
3. 随机状态残差验证可以证明前向动力学与逆动力学实现一致。
4. 受迫响应结果可以说明：重力项对关节静载影响明显，惯性项随加速度变化，低速下科里奥利项相对较小。
5. 这种 2R 简化模型虽然不能描述整机接触和机身耦合，但足以支撑课程任务 5 和任务 6 的动力学控制实验。
