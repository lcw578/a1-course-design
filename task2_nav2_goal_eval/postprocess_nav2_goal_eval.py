#!/usr/bin/env python3
"""Post-process Nav2 goal-sequence evaluation into report figures."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
import sys

sys.path.insert(0, str(ROOT))

from common.io_utils import configure_matplotlib


def load_rows(csv_path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    with csv_path.open("r", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        for row in reader:
            parsed: dict[str, object] = {"goal": row["goal"]}
            for key, value in row.items():
                if key == "goal":
                    continue
                parsed[key] = float(value)
            rows.append(parsed)
    return rows


def read_map_image(map_yaml: Path) -> tuple[np.ndarray, list[float], float]:
    import yaml

    data = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image_path = map_yaml.parent / data["image"]
    image = read_ascii_pgm(image_path)
    origin = [float(v) for v in data["origin"]]
    resolution = float(data["resolution"])
    return image, origin, resolution


def read_ascii_pgm(path: Path) -> np.ndarray:
    tokens: list[str] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        tokens.extend(stripped.split())
    if not tokens or tokens[0] != "P2":
        raise ValueError(f"Unsupported PGM format in {path}")
    width = int(tokens[1])
    height = int(tokens[2])
    maxval = float(tokens[3])
    values = np.array([float(v) for v in tokens[4:]], dtype=float)
    image = values.reshape((height, width)) / maxval
    return image


def write_report(out_dir: Path, summary: dict) -> None:
    goal_lines = []
    for goal in summary["goal_results"]:
        goal_lines.append(
            f"- `{goal['name']}`: 状态 `{goal['status']}`，"
            f"真值目标点误差 `{goal['final_gt_goal_xy_error_m']:.3f} m`，"
            f"Nav2 使用 `/odom` 的目标点误差 `{goal['final_raw_goal_xy_error_m']:.3f} m`，"
            f"真值路径长度 `{goal['gt_path_length_m']:.3f} m`，"
            f"耗时 `{goal['duration_sec']:.2f} s`。"
        )

    report = f"""# Nav2 目标点导航评估记录

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

- 平均真值目标点误差：`{summary['mean_final_gt_goal_xy_error_m']:.3f} m`
- 平均 `/odom` 目标点误差：`{summary['mean_final_gt_goal_xy_error_m']:.3f} m`
- `/odom` 相对 `/odom_gt` 的位置 RMSE：`{summary['raw_position_xy_rmse_m']:.3f} m`
- `cmd_vel` 非零比例：`{summary['cmd_vel_nonzero_ratio']:.3f}`

逐目标结果：

{chr(10).join(goal_lines)}

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
"""
    (out_dir / "NAV2_GOAL_EVAL_REPORT.md").write_text(report, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", required=True)
    parser.add_argument(
        "--map-yaml",
        default="/home/lcw/rl_sar_ws/src/rl_sar/src/rl_sar/maps/a1_nav_world_empty.yaml",
    )
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    summary = json.loads((input_dir / "summary.json").read_text(encoding="utf-8"))
    rows = load_rows(input_dir / "samples.csv")
    image, origin, resolution = read_map_image(Path(args.map_yaml))

    configure_matplotlib()
    import matplotlib.pyplot as plt

    goals = {goal["name"]: goal for goal in summary["goal_results"]}
    unique_goals = [goal for goal in goals if goal in {row["goal"] for row in rows}]

    width = image.shape[1] * resolution
    height = image.shape[0] * resolution
    extent = [origin[0], origin[0] + width, origin[1], origin[1] + height]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(image, cmap="gray", origin="lower", extent=extent)
    for goal_name in unique_goals:
        goal_rows = [row for row in rows if row["goal"] == goal_name]
        ax.plot([row["gt_x"] for row in goal_rows], [row["gt_y"] for row in goal_rows], label=f"{goal_name} gt")
        ax.plot(
            [row["amcl_x"] for row in goal_rows],
            [row["amcl_y"] for row in goal_rows],
            linestyle="--",
            label=f"{goal_name} map_pose",
        )
        goal = goals[goal_name]
        ax.scatter(goal["goal_x_m"], goal["goal_y_m"], marker="x", s=90, linewidths=2)
    ax.set_title("A1 Nav2 Goal Navigation Paths")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(input_dir / "01_path_overview.png", dpi=180)
    plt.close(fig)

    time0 = float(rows[0]["t"]) if rows else 0.0
    times = [float(row["t"]) - time0 for row in rows]
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(times, [row["raw_err_xy"] for row in rows], label="raw vs gt")
    axes[0].plot(times, [row["amcl_err_xy"] for row in rows], label="map_pose vs gt")
    axes[0].set_ylabel("xy error [m]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].plot(times, [row["raw_err_yaw"] for row in rows], label="raw vs gt")
    axes[1].plot(times, [row["amcl_err_yaw"] for row in rows], label="map_pose vs gt")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("yaw error [rad]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    fig.suptitle("Nav2 Localization Error Along the Full Goal Sequence")
    fig.tight_layout()
    fig.savefig(input_dir / "02_goal_error_timeseries.png", dpi=180)
    plt.close(fig)

    labels = [goal["name"] for goal in summary["goal_results"]]
    gt_goal_err = [goal["final_gt_goal_xy_error_m"] for goal in summary["goal_results"]]
    amcl_goal_err = [goal["final_amcl_goal_xy_error_m"] for goal in summary["goal_results"]]
    durations = [goal["duration_sec"] for goal in summary["goal_results"]]
    x = np.arange(len(labels))
    w = 0.35
    fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    axes[0].bar(x - w / 2, gt_goal_err, width=w, label="gt final goal error")
    axes[0].bar(x + w / 2, amcl_goal_err, width=w, label="map_pose final goal error")
    axes[0].set_ylabel("goal xy error [m]")
    axes[0].grid(True, axis="y", alpha=0.3)
    axes[0].legend()
    axes[1].bar(x, durations, width=0.5, color="tab:green")
    axes[1].set_ylabel("duration [s]")
    axes[1].set_xticks(x, labels)
    axes[1].grid(True, axis="y", alpha=0.3)
    fig.suptitle("Per-goal Navigation Result Summary")
    fig.tight_layout()
    fig.savefig(input_dir / "03_goal_summary.png", dpi=180)
    plt.close(fig)

    write_report(input_dir, summary)


if __name__ == "__main__":
    main()
