#!/usr/bin/env python3
"""Task 7: ROS2 velocity-tracking RMSE evaluator for the existing A1 RL policy.

Run this after launching the existing simulation, for example:

  source /opt/ros/humble/setup.bash
  source /home/lcw/rl_sar_ws/install/setup.bash
  ros2 launch rl_sar a1_nav2_sim.launch.py use_rviz:=false

Then in another terminal:

  python3 course_design/task7_rl_velocity_eval/ros2_velocity_rmse_eval.py
"""

from __future__ import annotations

import argparse
import csv
import math
import json
import sys
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.io_utils import configure_matplotlib, ensure_results_dir


def command_profile(t: float) -> tuple[float, float]:
    if t < 3.0:
        return 0.0, 0.0
    if t < 11.0:
        return 0.22, 0.0
    if t < 17.0:
        return 0.12, 0.35
    if t < 23.0:
        return 0.0, -0.35
    return 0.0, 0.0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=28.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument(
        "--policy-path",
        default="/home/lcw/rl_sar_ws/src/rl_sar/policy/a1/legged_gym/model.pt",
    )
    args = parser.parse_args()

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    class VelocityEvaluator(Node):
        def __init__(self) -> None:
            super().__init__("a1_rl_velocity_rmse_eval")
            self.publisher = self.create_publisher(Twist, args.cmd_topic, 10)
            self.create_subscription(Odometry, args.odom_topic, self.odom_callback, 10)
            self.latest_odom: Optional[Odometry] = None
            self.start_time = self.get_clock().now()
            self.rows = []
            self.timer = self.create_timer(1.0 / args.rate, self.tick)

        def odom_callback(self, msg: Odometry) -> None:
            self.latest_odom = msg

        def tick(self) -> None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            vx_ref, wz_ref = command_profile(elapsed)
            msg = Twist()
            msg.linear.x = vx_ref
            msg.angular.z = wz_ref
            self.publisher.publish(msg)

            vx = math.nan
            wz = math.nan
            if self.latest_odom is not None:
                vx = float(self.latest_odom.twist.twist.linear.x)
                wz = float(self.latest_odom.twist.twist.angular.z)
            self.rows.append(
                {
                    "time": elapsed,
                    "vx_ref": vx_ref,
                    "wz_ref": wz_ref,
                    "vx": vx,
                    "wz": wz,
                    "vx_error": vx_ref - vx if not math.isnan(vx) else math.nan,
                    "wz_error": wz_ref - wz if not math.isnan(wz) else math.nan,
                }
            )

            if elapsed >= args.duration:
                stop = Twist()
                for _ in range(5):
                    self.publisher.publish(stop)
                self.timer.cancel()

    rclpy.init()
    node = VelocityEvaluator()
    try:
        while rclpy.ok() and not node.timer.is_canceled():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        rows = node.rows
        node.destroy_node()
        rclpy.shutdown()

    out_dir = ensure_results_dir("task7_rl_velocity_eval")
    csv_path = out_dir / "rl_velocity_rmse.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    valid = [r for r in rows if not math.isnan(r["vx"])]
    if valid:
        vx_rmse = math.sqrt(sum(r["vx_error"] ** 2 for r in valid) / len(valid))
        wz_rmse = math.sqrt(sum(r["wz_error"] ** 2 for r in valid) / len(valid))
    else:
        vx_rmse = math.nan
        wz_rmse = math.nan

    configure_matplotlib()
    import matplotlib.pyplot as plt

    time = [r["time"] for r in rows]
    fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    axes[0].plot(time, [r["vx_ref"] for r in rows], label="vx_ref")
    axes[0].plot(time, [r["vx"] for r in rows], label="vx")
    axes[0].set_ylabel("linear x [m/s]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].plot(time, [r["wz_ref"] for r in rows], label="wz_ref")
    axes[1].plot(time, [r["wz"] for r in rows], label="wz")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("yaw rate [rad/s]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    fig.suptitle(f"A1 RL velocity tracking, vx RMSE={vx_rmse:.3f}, wz RMSE={wz_rmse:.3f}")
    fig.tight_layout()
    plot_path = out_dir / "rl_velocity_tracking.png"
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)

    summary = {
        "duration_sec": args.duration,
        "rate_hz": args.rate,
        "cmd_topic": args.cmd_topic,
        "odom_topic": args.odom_topic,
        "policy_path": args.policy_path,
        "vx_rmse_m_s": vx_rmse,
        "wz_rmse_rad_s": wz_rmse,
        "outputs": {
            "csv": str(csv_path),
            "plot": str(plot_path),
        },
    }
    summary_path = out_dir / "task7_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(f"CSV: {csv_path}")
    print(f"Plot: {plot_path}")
    print(f"Summary: {summary_path}")
    print(f"vx RMSE: {vx_rmse:.6f} m/s")
    print(f"wz RMSE: {wz_rmse:.6f} rad/s")


if __name__ == "__main__":
    main()
