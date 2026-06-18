# 任务 3：A1 单腿雅可比矩阵、空间/体雅可比与奇异性分析

## 1. 是否需要求整只机器狗的 Js/Jb？

课程 PDF 的任务 3 明确要求：

1. 推导几何雅可比或解析雅可比矩阵。
2. 分别计算空间雅可比 `Js(theta)` 和体雅可比 `Jb(theta)`。
3. 分析奇异位形、零空间和力-力矩变换。

因此，对于本课程设计，应该补充 `Js/Jb`。但从四足机器人实际控制角度看，通常不会把整只 A1 机器狗当作一个固定基座串联机械臂来求一个单一的 `Js/Jb`。A1 整机是浮动基座 + 12 个关节 + 地面接触约束系统，完整速度关系应写成浮动基座和各足端接触的约束雅可比。

为了和课程中串联机械臂章节对齐，本项目选择 A1 的单条腿作为 3R 串联机构，足端作为末端执行器。这样可以严格使用《现代机器人学》第 5 章的速度运动学方法，得到：

- 足端位置几何雅可比 `Jv(theta) in R^{3x3}`。
- 空间雅可比 `Js(theta) in R^{6x3}`。
- 体雅可比 `Jb(theta) in R^{6x3}`。

在实际腿式控制中，`Jv` 最常用，因为足端力和关节力矩满足：

```text
tau = Jv(theta)^T f_foot
```

其中 `f_foot` 是足端三维接触力。`Js/Jb` 主要用于完整刚体 twist 表达和课程理论对齐。

## 2. A1 单腿运动学定义

以 FR 右前腿为例，关节变量为：

```text
q = [q1, q2, q3]^T
q1: hip abduction/adduction, 绕局部 x 轴
q2: thigh pitch, 绕局部 y 轴
q3: calf pitch, 绕局部 y 轴
```

几何参数来自仓库中的 A1 URDF：

```text
h = [0.1805, -0.047, 0]^T          trunk -> FR_hip_joint
d = [0, -0.0838, 0]^T              hip -> thigh joint
l1 = 0.2 m                         thigh length
l2 = 0.2 m                         calf length
```

定义旋转矩阵：

```text
R1 = Rx(q1)
R2 = Ry(q2)
R3 = Ry(q3)
```

各关节轴在 trunk/base 坐标系中的表达为：

```text
w1 = [1, 0, 0]^T
w2 = R1 [0, 1, 0]^T
w3 = R1 R2 [0, 1, 0]^T
```

各关节轴上一点的位置为：

```text
p1 = h
p2 = h + R1 d
p3 = p2 + R1 R2 [0, 0, -l1]^T
```

足端位置为：

```text
pe = p3 + R1 R2 R3 [0, 0, -l2]^T
```

## 3. 足端位置几何雅可比 Jv

对 revolute joint，第 `i` 列位置雅可比为：

```text
Jv_i = wi x (pe - pi)
```

所以：

```text
Jv(q) = [
  w1 x (pe - p1),  w2 x (pe - p2),  w3 x (pe - p3)
]
```

它满足：

```text
pe_dot = Jv(q) q_dot
```

本项目使用 `Jv` 做以下分析：

- 有限差分验证。
- 行列式 `det(Jv)` 和条件数分析奇异位形。
- 足端力到关节力矩映射 `tau = Jv^T f`。

注意：这里的 `Jv` 是 `3x3`，因此可以直接计算行列式。`Js/Jb` 是 `6x3`，不是方阵，不能计算普通行列式。

## 4. 空间雅可比 Js

采用《现代机器人学》的 twist 顺序：

```text
V = [omega; v]
```

对于 revolute joint，空间 twist 列为：

```text
Si(q) = [wi; -wi x pi]
```

因此空间雅可比为：

```text
Js(q) = [
  [w1; -w1 x p1],
  [w2; -w2 x p2],
  [w3; -w3 x p3]
]
```

它满足：

```text
Vs = Js(q) q_dot
```

其中 `Vs` 是足端坐标系相对 base 的空间 twist，用 base 坐标系表达。

`Js` 的底部三行不是足端原点速度本身。二者关系为：

```text
pe_dot = v_s + omega_s x pe
```

对每一列也有：

```text
Jv_i = Js_v_i + Js_omega_i x pe
```

这也是代码中验证 `Jv` 与 `Js` 一致性的方式。

## 5. 体雅可比 Jb

末端齐次变换为：

```text
T = T_base_foot(q)
```

空间雅可比和体雅可比满足：

```text
Jb(q) = Ad_{T^{-1}} Js(q)
```

对齐次变换：

```text
T = [ R  p ]
    [ 0  1 ]
```

twist 顺序为 `[omega; v]` 时，伴随矩阵为：

```text
Ad_T = [ R        0 ]
       [ [p]R     R ]
```

其中 `[p]` 是向量 `p` 的反对称矩阵。

## 6. 名义位形数值结果

名义位形：

```text
q = [0.0, 0.8, -1.5] rad
```

足端位置几何雅可比：

```text
Jv =
[[ 0.000000, -0.292310, -0.152968],
 [ 0.292310,  0.000000,  0.000000],
 [-0.083800,  0.014628, -0.128844]]
```

空间雅可比，行顺序为 `[omega_x, omega_y, omega_z, v_x, v_y, v_z]`：

```text
Js =
[[ 1.000000,  0.000000,  0.000000],
 [ 0.000000,  1.000000,  1.000000],
 [ 0.000000,  0.000000,  0.000000],
 [-0.000000, -0.000000,  0.139341],
 [-0.000000, -0.000000, -0.000000],
 [ 0.047000,  0.180500,  0.037029]]
```

体雅可比，行顺序同样为 `[omega_x, omega_y, omega_z, v_x, v_y, v_z]`：

```text
Jb =
[[ 0.764842,  0.000000,  0.000000],
 [ 0.000000,  1.000000,  1.000000],
 [-0.644218,  0.000000,  0.000000],
 [-0.053985, -0.214147, -0.200000],
 [ 0.292310,  0.000000,  0.000000],
 [-0.064094,  0.199499,  0.000000]]
```

## 7. 奇异性与零空间分析

因为 A1 单腿选择的是 3 个关节控制足端 3D 位置，`Jv` 是 `3x3` 方阵。非奇异时：

```text
rank(Jv) = 3
det(Jv) != 0
null(Jv) = {0}
```

此时给定足端速度 `pe_dot`，关节速度解唯一：

```text
q_dot = Jv^{-1} pe_dot
```

奇异或近奇异时：

```text
det(Jv) -> 0
condition_number(Jv) -> large
```

物理意义是：某些方向的足端速度很难通过关节速度产生，或者需要非常大的关节速度；静力学上，相同足端力可能导致较大的关节力矩需求。

由于该模型不是冗余机构，正常位形下没有非零零空间运动。只有在秩下降时，才可能存在非零 `q_dot` 使得：

```text
Jv q_dot ≈ 0
```

这表示关节运动对足端位置几乎不产生有效速度。

## 8. 力-力矩映射

足端力为：

```text
f = [0, 0, 20]^T N
```

名义位形下：

```text
tau = Jv^T f
    = [-1.676000, 0.292554, -2.576871]^T Nm
```

该结果表示：如果足端受到竖直方向 20 N 的接触力，FR 腿三个关节需要产生上述等效力矩来平衡该外力。

## 9. 程序验证结果

任务三脚本：

```text
course_design/task3_jacobian/run_task3_jacobian.py
```

输出结果：

```text
course_design/results/task3_jacobian/fr_task3_summary.json
course_design/results/task3_jacobian/fr_jacobian_scan.csv
course_design/results/task3_jacobian/fr_singularity_scan.png
```

数值验证结果：

```text
max finite-difference Jv error  = 1.1088781459307783e-10
max finite-difference Js error  = 3.1080836960770227e-10
max finite-difference Jb error  = 2.9618137951949864e-10
max Js-to-Jv relation error     = 1.2888427958941859e-16
```

这些误差均在数值有限差分精度范围内，说明解析雅可比、空间雅可比和体雅可比实现正确。
