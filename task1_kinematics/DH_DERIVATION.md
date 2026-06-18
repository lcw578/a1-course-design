# 任务 1：A1 单腿 DH/MDH 建模说明

## 1. 网上资料核查结论

没有找到 Unitree 官方直接发布的 A1 单腿标准 DH 表。能找到并可作为可靠依据的是：

1. Unitree A1 Software Developer Guide  
   该文档给出 A1 的腿编号、关节编号、关节轴、关节限位和主要几何参数：hip 绕 x 轴，thigh/calf 绕 y 轴；hip link length 为 `0.0838 m`，thigh/calf link length 均为 `0.2 m`。  
   URL: `https://www.mybotshop.de/Datasheet/Unitree_A1_Software_Guide_v2.0.pdf`

2. Unitree 官方 ROS/URDF 模型  
   该模型给出 A1 每个 link/joint 的 origin、axis、limit、mass、inertia、collision。  
   URL: `https://github.com/unitreerobotics/unitree_ros`

3. 本仓库中的 A1 URDF  
   当前项目已经包含 A1 描述文件，FR 腿关键参数如下：  
   `src/rl_sar/src/rl_sar_zoo/a1_description/urdf/a1_description.urdf`

因此，本项目的 DH/MDH 表不是从网上现成表格复制，而是依据官方软件指南和 URDF 关节定义推导得到。

## 2. 建模对象

选择 A1 右前腿 `FR` 作为 3R 串联机构：

| 关节 | URDF 名称 | 物理含义 | 旋转轴 |
| --- | --- | --- | --- |
| 1 | `FR_hip_joint` | 髋外展/内收 | body/local x |
| 2 | `FR_thigh_joint` | 大腿俯仰 | hip 后 local y |
| 3 | `FR_calf_joint` | 小腿俯仰 | thigh 后 local y |

末端执行器取 `FR_foot`，足端位置在机身 `base/trunk` 坐标系下表示。

## 3. 从 URDF 提取的几何参数

FR 腿在零位姿下的关键 origin 和 axis：

| 项 | URDF 值 |
| --- | --- |
| `FR_hip_joint` origin | `(0.1805, -0.047, 0)` |
| `FR_hip_joint` axis | `(1, 0, 0)` |
| `FR_thigh_joint` origin relative to `FR_hip` | `(0, -0.0838, 0)` |
| `FR_thigh_joint` axis | `(0, 1, 0)` |
| `FR_calf_joint` origin relative to `FR_thigh` | `(0, 0, -0.2)` |
| `FR_calf_joint` axis | `(0, 1, 0)` |
| `FR_foot_fixed` origin relative to `FR_calf` | `(0, 0, -0.2)` |

定义：

```text
p_h = [0.1805, -0.047, 0]^T
h   = -0.0838     # FR thigh lateral offset
l1  = 0.2         # thigh length
l2  = 0.2         # calf length
q   = [q1, q2, q3]^T
```

## 4. URDF 等效 MDH/变换表

A1 的官方 ROS 坐标定义不是常见机械臂教材中“所有关节绕 z_i 轴”的标准 DH 画法：hip 绕 x 轴，thigh/calf 绕 y 轴。为了避免为了套表而引入错误坐标系，本项目采用 **URDF 等效 modified-DH 变换链**。每一行给出一个关节旋转和随后固定连杆偏置，数学上等价于 URDF。

FR 腿等效表：

| i | 关节 | 旋转轴 | 关节变量 | 固定平移 |
| --- | --- | --- | --- | --- |
| base | trunk -> hip | - | - | `p_h = [0.1805, -0.047, 0]^T` |
| 1 | hip | x | `q1` | `[0, -0.0838, 0]^T` |
| 2 | thigh | y | `q2` | `[0, 0, -l1]^T = [0, 0, -0.2]^T` |
| 3 | calf | y | `q3` | `[0, 0, -l2]^T = [0, 0, -0.2]^T` |

对应齐次变换为：

```text
T_base_foot(q)
  = Trans(p_h)
    Rx(q1) Trans(0, -0.0838, 0)
    Ry(q2) Trans(0, 0, -0.2)
    Ry(q3) Trans(0, 0, -0.2)
```

其中：

```text
Rx(q1) =
[ 1      0        0
  0   cosq1   -sinq1
  0   sinq1    cosq1 ]

Ry(q) =
[ cosq   0   sinq
    0    1     0
 -sinq   0   cosq ]
```

## 5. 正运动学足端位置

令：

```text
R1  = Rx(q1)
R2  = Ry(q2)
R3  = Ry(q3)
e_y = [0, 1, 0]^T
e_z = [0, 0, 1]^T
```

则足端位置为：

```text
p_foot(q)
  = p_h
    + R1 [0, h, 0]^T
    + R1 R2 [0, 0, -l1]^T
    + R1 R2 R3 [0, 0, -l2]^T
```

对 FR 腿：

```text
h = -0.0838, l1 = 0.2, l2 = 0.2
```

该公式已经在代码中实现：

```text
course_design/common/a1_leg_model.py
```

## 6. 逆运动学思路

给定足端目标 `p_target`：

1. 先减去 hip 基座偏置：

```text
p = p_target - p_h
```

2. hip 旋转后，thigh/calf 平面中 lateral offset 应满足：

```text
Rx(-q1) p 的 y 分量 = h
```

即：

```text
p_y cos(q1) + p_z sin(q1) = h
```

可得到两个候选 `q1`：

```text
q1 = atan2(p_z, p_y) ± arccos(h / sqrt(p_y^2 + p_z^2))
```

3. 对每个候选 `q1`，将问题转为 thigh/calf 平面 2R IK：

```text
v = Rx(-q1) p - [0, h, 0]^T
x = v_x, z = v_z
```

4. 由余弦定理解 `q3`：

```text
cos(q3) = (x^2 + z^2 - l1^2 - l2^2) / (2 l1 l2)
```

5. 再解 `q2`：

```text
q2 = atan2(-x, -z) - atan2(l2 sin(q3), l1 + l2 cos(q3))
```

6. 将候选解代入关节限位，并用阻尼最小二乘做数值微调，保证闭环误差。

当前 100 组随机测试的最大 FK-IK 闭环误差为：

```text
9.896255409983198e-09 m
```

满足作业要求的 `<= 1e-6`。
