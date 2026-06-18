#!/usr/bin/env python3

import csv
import json
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quat_xyzw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Pose2D:
    t: float
    x: float
    y: float
    yaw: float


@dataclass
class GoalSpec:
    name: str
    x: float
    y: float
    yaw: float


def parse_goal_specs(raw_specs: List[str]) -> List[GoalSpec]:
    goals: List[GoalSpec] = []
    for raw_spec in raw_specs:
        parts = [part.strip() for part in str(raw_spec).split(",")]
        if len(parts) != 4:
            raise ValueError(
                f"Invalid goal spec '{raw_spec}'. Expected format 'name,x,y,yaw'."
            )
        name = parts[0]
        goals.append(GoalSpec(name, float(parts[1]), float(parts[2]), float(parts[3])))
    return goals


class EvaluateNav2GoalSequence(Node):
    def __init__(self) -> None:
        super().__init__("evaluate_nav2_goal_sequence")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("ground_truth_topic", "/odom_gt")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("max_time_diff_sec", 0.05)
        self.declare_parameter("sample_period_sec", 0.05)
        self.declare_parameter("goal_timeout_sec", 90.0)
        self.declare_parameter("goal_pause_sec", 5.0)
        self.declare_parameter("goal_specs", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("align_initial_pose", True)
        self.declare_parameter("output_root", str(Path.home() / "rl_sar_eval" / "nav2_goal_sequence"))
        self.declare_parameter("run_name", "")

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.ground_truth_topic = str(self.get_parameter("ground_truth_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.base_frame_candidates = []
        for frame in [self.base_frame, "base", "base_footprint"]:
            if frame and frame not in self.base_frame_candidates:
                self.base_frame_candidates.append(frame)
        self.max_time_diff_sec = float(self.get_parameter("max_time_diff_sec").value)
        self.sample_period_sec = float(self.get_parameter("sample_period_sec").value)
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self.goal_pause_sec = float(self.get_parameter("goal_pause_sec").value)
        raw_goal_specs = list(
            self.get_parameter_or(
                "goal_specs",
                Parameter("goal_specs", Parameter.Type.STRING_ARRAY, []),
            ).value
        )
        self.align_initial_pose = bool(self.get_parameter("align_initial_pose").value)
        self.output_root = Path(str(self.get_parameter("output_root").value))
        run_name = str(self.get_parameter("run_name").value).strip()
        if not run_name:
            run_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = self.output_root / run_name
        self.output_dir.mkdir(parents=True, exist_ok=True)

        if raw_goal_specs:
            self.goals = parse_goal_specs(raw_goal_specs)
        else:
            self.goals = [
                GoalSpec("goal_1m", 1.0, 0.0, 0.0),
                GoalSpec("goal_2m", 2.0, 0.0, 0.0),
                GoalSpec("goal_left", 1.5, 1.0, 0.0),
            ]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.latest_odom: Optional[Pose2D] = None
        self.latest_gt: Optional[Pose2D] = None
        self.last_record_time: Optional[float] = None
        self.raw_align: Optional[Pose2D] = None
        self.amcl_align: Optional[Pose2D] = None
        self.gt_align: Optional[Pose2D] = None
        self.samples: List[dict] = []
        self.goal_results: List[dict] = []
        self.tf_failures = 0
        self.cmd_vel_samples = 0
        self.cmd_vel_nonzero_samples = 0
        self.current_goal_name = "idle"

        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 50)
        self.create_subscription(Odometry, self.ground_truth_topic, self._gt_callback, 50)
        self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_vel_callback, 50)
        self.create_timer(self.sample_period_sec, self._sample)

        self.get_logger().info(
            f"Ready to send {len(self.goals)} Nav2 goals and evaluate task-level errors in '{self.output_dir}'"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = self._pose_from_odom(msg)

    def _gt_callback(self, msg: Odometry) -> None:
        self.latest_gt = self._pose_from_odom(msg)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.cmd_vel_samples += 1
        if abs(msg.linear.x) > 1e-3 or abs(msg.linear.y) > 1e-3 or abs(msg.angular.z) > 1e-3:
            self.cmd_vel_nonzero_samples += 1

    def run_sequence(self) -> None:
        self._spin_sleep(2.0)
        if not self.action_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Nav2 action server /navigate_to_pose is not available")
            for goal in self.goals:
                self.goal_results.append(self._goal_result(goal, "FAILED", 0.0))
            self.write_outputs()
            return

        for goal in self.goals:
            self.current_goal_name = goal.name
            result = self._send_goal_and_wait(goal)
            self.goal_results.append(result)
            self.write_outputs()
            self.current_goal_name = "pause"
            self._spin_sleep(self.goal_pause_sec)

        self.current_goal_name = "done"
        self.write_outputs()
        self.get_logger().info("Nav2 goal sequence complete")

    def _send_goal_and_wait(self, goal: GoalSpec) -> dict:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal.x
        goal_msg.pose.pose.position.y = goal.y
        goal_msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(goal.yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        start = self.get_clock().now()
        self.get_logger().info(f"Sending {goal.name}: x={goal.x:.2f}, y={goal.y:.2f}, yaw={goal.yaw:.2f}")
        send_future = self.action_client.send_goal_async(goal_msg)
        while rclpy.ok() and not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)

        if not send_future.done() or send_future.result() is None:
            return self._goal_result(goal, "FAILED", self._elapsed_sec(start))

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"{goal.name} was rejected")
            return self._goal_result(goal, "FAILED", self._elapsed_sec(start))

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._elapsed_sec(start) > self.goal_timeout_sec:
                cancel_future = goal_handle.cancel_goal_async()
                while rclpy.ok() and not cancel_future.done():
                    rclpy.spin_once(self, timeout_sec=0.05)
                self.get_logger().warn(f"{goal.name} timed out after {self.goal_timeout_sec:.1f}s")
                return self._goal_result(goal, "TIMEOUT", self._elapsed_sec(start))

        status = "FAILED"
        if result_future.done() and result_future.result() is not None:
            # action_msgs/msg/GoalStatus: STATUS_SUCCEEDED = 4
            status = "SUCCEEDED" if result_future.result().status == 4 else "FAILED"
        self.get_logger().info(f"{goal.name} finished with status={status}")
        return self._goal_result(goal, status, self._elapsed_sec(start))

    def _goal_result(self, goal: GoalSpec, status: str, duration_sec: float) -> dict:
        goal_samples = [sample for sample in self.samples if sample["goal"] == goal.name]
        latest = goal_samples[-1] if goal_samples else {}
        result = {
            "name": goal.name,
            "status": status,
            "duration_sec": duration_sec,
            "goal_x_m": goal.x,
            "goal_y_m": goal.y,
            "goal_yaw_rad": goal.yaw,
            "final_amcl_xy_error_m": float(latest.get("amcl_err_xy", math.nan)),
            "final_raw_xy_error_m": float(latest.get("raw_err_xy", math.nan)),
            "final_amcl_yaw_error_rad": float(latest.get("amcl_err_yaw", math.nan)),
            "final_raw_yaw_error_rad": float(latest.get("raw_err_yaw", math.nan)),
            "final_raw_goal_xy_error_m": self._goal_xy_error(latest.get("raw_x"), latest.get("raw_y"), goal),
            "final_amcl_goal_xy_error_m": self._goal_xy_error(latest.get("amcl_x"), latest.get("amcl_y"), goal),
            "final_gt_goal_xy_error_m": self._goal_xy_error(latest.get("gt_x"), latest.get("gt_y"), goal),
            "final_raw_goal_yaw_error_rad": self._goal_yaw_error(latest.get("raw_yaw"), goal),
            "final_amcl_goal_yaw_error_rad": self._goal_yaw_error(latest.get("amcl_yaw"), goal),
            "final_gt_goal_yaw_error_rad": self._goal_yaw_error(latest.get("gt_yaw"), goal),
            "raw_path_length_m": self._path_length(goal_samples, "raw_x", "raw_y"),
            "amcl_path_length_m": self._path_length(goal_samples, "amcl_x", "amcl_y"),
            "gt_path_length_m": self._path_length(goal_samples, "gt_x", "gt_y"),
        }
        self.get_logger().info(
            f"{goal.name} final GT goal error = {result['final_gt_goal_xy_error_m']:.3f} m, "
            f"AMCL goal error = {result['final_amcl_goal_xy_error_m']:.3f} m"
        )
        return result

    def _sample(self) -> None:
        if self.latest_odom is None or self.latest_gt is None:
            return
        if abs(self.latest_odom.t - self.latest_gt.t) > self.max_time_diff_sec:
            return
        if self.last_record_time is not None and self.latest_odom.t - self.last_record_time < self.sample_period_sec:
            return

        transform = None
        for base_frame in self.base_frame_candidates:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.02),
                )
                break
            except TransformException:
                continue
        if transform is None:
            self.tf_failures += 1
            return

        tr = transform.transform.translation
        q = transform.transform.rotation
        amcl_pose = Pose2D(
            t=float(transform.header.stamp.sec) + float(transform.header.stamp.nanosec) * 1e-9,
            x=float(tr.x),
            y=float(tr.y),
            yaw=yaw_from_quat_xyzw(float(q.x), float(q.y), float(q.z), float(q.w)),
        )

        raw_pose = self.latest_odom
        gt_pose = self.latest_gt
        if self.align_initial_pose and self.gt_align is None:
            self.gt_align = gt_pose
            self.raw_align = raw_pose
            self.amcl_align = amcl_pose

        raw_eval = self._aligned(raw_pose, self.raw_align, self.gt_align)
        amcl_eval = self._aligned(amcl_pose, self.amcl_align, self.gt_align)
        raw_err_xy = math.hypot(raw_eval.x - gt_pose.x, raw_eval.y - gt_pose.y)
        amcl_err_xy = math.hypot(amcl_eval.x - gt_pose.x, amcl_eval.y - gt_pose.y)
        raw_err_yaw = wrap_to_pi(raw_eval.yaw - gt_pose.yaw)
        amcl_err_yaw = wrap_to_pi(amcl_eval.yaw - gt_pose.yaw)

        self.samples.append(
            {
                "goal": self.current_goal_name,
                "t": raw_pose.t,
                "raw_x": raw_eval.x,
                "raw_y": raw_eval.y,
                "raw_yaw": raw_eval.yaw,
                "amcl_x": amcl_eval.x,
                "amcl_y": amcl_eval.y,
                "amcl_yaw": amcl_eval.yaw,
                "gt_x": gt_pose.x,
                "gt_y": gt_pose.y,
                "gt_yaw": gt_pose.yaw,
                "raw_err_xy": raw_err_xy,
                "amcl_err_xy": amcl_err_xy,
                "raw_err_yaw": raw_err_yaw,
                "amcl_err_yaw": amcl_err_yaw,
            }
        )
        self.last_record_time = raw_pose.t

    def _pose_from_odom(self, msg: Odometry) -> Pose2D:
        q = msg.pose.pose.orientation
        return Pose2D(
            t=float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw_from_quat_xyzw(float(q.x), float(q.y), float(q.z), float(q.w)),
        )

    def _aligned(self, pose: Pose2D, source0: Optional[Pose2D], target0: Optional[Pose2D]) -> Pose2D:
        if not self.align_initial_pose or source0 is None or target0 is None:
            return pose
        dyaw = wrap_to_pi(target0.yaw - source0.yaw)
        c = math.cos(dyaw)
        s = math.sin(dyaw)
        dx = pose.x - source0.x
        dy = pose.y - source0.y
        return Pose2D(
            t=pose.t,
            x=target0.x + c * dx - s * dy,
            y=target0.y + s * dx + c * dy,
            yaw=wrap_to_pi(pose.yaw + dyaw),
        )

    def _elapsed_sec(self, start_time) -> float:
        return (self.get_clock().now() - start_time).nanoseconds * 1e-9

    def _spin_sleep(self, seconds: float) -> None:
        start = self.get_clock().now()
        while rclpy.ok() and self._elapsed_sec(start) < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def write_outputs(self) -> None:
        samples_path = self.output_dir / "samples.csv"
        with samples_path.open("w", newline="") as f:
            if self.samples:
                writer = csv.DictWriter(f, fieldnames=list(self.samples[0].keys()))
                writer.writeheader()
                writer.writerows(self.samples)

        def rmse(key: str) -> float:
            if not self.samples:
                return float("nan")
            return math.sqrt(sum(float(row[key]) ** 2 for row in self.samples) / len(self.samples))

        def nonzero_ratio() -> float:
            if self.cmd_vel_samples == 0:
                return 0.0
            return float(self.cmd_vel_nonzero_samples) / float(self.cmd_vel_samples)

        summary = {
            "goal_results": self.goal_results,
            "samples": len(self.samples),
            "align_initial_pose": self.align_initial_pose,
            "raw_position_xy_rmse_m": rmse("raw_err_xy"),
            "amcl_position_xy_rmse_m": rmse("amcl_err_xy"),
            "raw_yaw_rmse_rad": rmse("raw_err_yaw"),
            "amcl_yaw_rmse_rad": rmse("amcl_err_yaw"),
            "mean_final_gt_goal_xy_error_m": self._mean_goal_metric("final_gt_goal_xy_error_m"),
            "mean_final_amcl_goal_xy_error_m": self._mean_goal_metric("final_amcl_goal_xy_error_m"),
            "mean_final_gt_goal_yaw_error_rad": self._mean_goal_metric("final_gt_goal_yaw_error_rad"),
            "mean_final_amcl_goal_yaw_error_rad": self._mean_goal_metric("final_amcl_goal_yaw_error_rad"),
            "cmd_vel_nonzero_ratio": nonzero_ratio(),
            "tf_failures": self.tf_failures,
        }
        (self.output_dir / "summary.json").write_text(json.dumps(summary, indent=2))

    def _goal_xy_error(self, x: Optional[float], y: Optional[float], goal: GoalSpec) -> float:
        if x is None or y is None:
            return float("nan")
        return math.hypot(float(x) - goal.x, float(y) - goal.y)

    def _goal_yaw_error(self, yaw: Optional[float], goal: GoalSpec) -> float:
        if yaw is None:
            return float("nan")
        return wrap_to_pi(float(yaw) - goal.yaw)

    def _path_length(self, samples: List[dict], x_key: str, y_key: str) -> float:
        if len(samples) < 2:
            return 0.0
        length = 0.0
        for prev, cur in zip(samples[:-1], samples[1:]):
            length += math.hypot(float(cur[x_key]) - float(prev[x_key]), float(cur[y_key]) - float(prev[y_key]))
        return length

    def _mean_goal_metric(self, key: str) -> float:
        values = [float(goal[key]) for goal in self.goal_results if key in goal and math.isfinite(float(goal[key]))]
        if not values:
            return float("nan")
        return sum(values) / len(values)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvaluateNav2GoalSequence()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.write_outputs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
