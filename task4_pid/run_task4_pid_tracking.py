#!/usr/bin/env python3
"""Task 4: quintic joint trajectory with velocity-PID tracking and tuning."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.a1_leg_model import A1LegKinematics
from common.io_utils import configure_matplotlib, ensure_results_dir
from common.pid import VectorPID
from common.trajectory import evaluate_quintic, make_time_grid, quintic_coefficients


JOINT_NAMES = ("hip", "thigh", "calf")


def pid_profiles() -> list[dict]:
    return [
        {
            "name": "low_pd",
            "description": "low proportional/derivative gains, no integral term",
            "kp": np.array([3.0, 3.5, 3.0], dtype=float),
            "ki": np.array([0.0, 0.0, 0.0], dtype=float),
            "kd": np.array([0.08, 0.09, 0.08], dtype=float),
            "integral_limit": 0.35,
        },
        {
            "name": "medium_pd",
            "description": "medium proportional/derivative gains, no integral term",
            "kp": np.array([6.0, 7.0, 6.0], dtype=float),
            "ki": np.array([0.0, 0.0, 0.0], dtype=float),
            "kd": np.array([0.16, 0.18, 0.16], dtype=float),
            "integral_limit": 0.35,
        },
        {
            "name": "medium_pid",
            "description": "medium PID gains with integral compensation",
            "kp": np.array([8.0, 9.0, 7.5], dtype=float),
            "ki": np.array([0.35, 0.35, 0.30], dtype=float),
            "kd": np.array([0.22, 0.25, 0.20], dtype=float),
            "integral_limit": 0.35,
        },
        {
            "name": "final_pid",
            "description": "selected balanced PID gains",
            "kp": np.array([9.0, 10.0, 8.0], dtype=float),
            "ki": np.array([0.6, 0.6, 0.5], dtype=float),
            "kd": np.array([0.25, 0.28, 0.22], dtype=float),
            "integral_limit": 0.35,
        },
        {
            "name": "high_pid",
            "description": "higher gains to evaluate overshoot and command effort",
            "kp": np.array([13.0, 15.0, 12.0], dtype=float),
            "ki": np.array([0.8, 0.8, 0.65], dtype=float),
            "kd": np.array([0.34, 0.38, 0.32], dtype=float),
            "integral_limit": 0.35,
        },
    ]


def simulate_tracking(
    model: A1LegKinematics,
    coeffs: np.ndarray,
    q0: np.ndarray,
    duration: float,
    dt: float,
    profile: dict,
    actuator_tau: np.ndarray,
    max_speed: np.ndarray,
) -> tuple[list[dict], dict]:
    pid = VectorPID(
        kp=profile["kp"],
        ki=profile["ki"],
        kd=profile["kd"],
        integral_limit=float(profile["integral_limit"]),
    )
    q = q0.copy()
    dq = np.zeros(3, dtype=float)
    rows = []
    for t in make_time_grid(duration, dt):
        qd, dqd, _ = evaluate_quintic(coeffs, t)
        qdot_cmd_unsat = pid.update(qd - q, dqd - dq, dt)
        qdot_cmd = np.clip(qdot_cmd_unsat, -max_speed, max_speed)

        ddq = (qdot_cmd - dq) / actuator_tau
        dq = dq + ddq * dt
        q = model.clamp_joints(q + dq * dt)

        foot, _ = model.forward_kinematics(q, "FR")
        foot_desired, _ = model.forward_kinematics(qd, "FR")
        pos_error = qd - q
        vel_error = dqd - dq
        command_saturated = bool(np.any(np.abs(qdot_cmd - qdot_cmd_unsat) > 1e-12))
        rows.append(
            {
                "time": t,
                "q_hip": q[0],
                "q_thigh": q[1],
                "q_calf": q[2],
                "qd_hip": qd[0],
                "qd_thigh": qd[1],
                "qd_calf": qd[2],
                "dq_hip": dq[0],
                "dq_thigh": dq[1],
                "dq_calf": dq[2],
                "dqd_hip": dqd[0],
                "dqd_thigh": dqd[1],
                "dqd_calf": dqd[2],
                "cmd_hip": qdot_cmd[0],
                "cmd_thigh": qdot_cmd[1],
                "cmd_calf": qdot_cmd[2],
                "cmd_unsat_hip": qdot_cmd_unsat[0],
                "cmd_unsat_thigh": qdot_cmd_unsat[1],
                "cmd_unsat_calf": qdot_cmd_unsat[2],
                "foot_x": foot[0],
                "foot_y": foot[1],
                "foot_z": foot[2],
                "foot_desired_x": foot_desired[0],
                "foot_desired_y": foot_desired[1],
                "foot_desired_z": foot_desired[2],
                "pos_error_norm": float(np.linalg.norm(pos_error)),
                "vel_error_norm": float(np.linalg.norm(vel_error)),
                "foot_error_norm_m": float(np.linalg.norm(foot_desired - foot)),
                "command_saturated": command_saturated,
            }
        )

    pos_error = np.array([r["pos_error_norm"] for r in rows], dtype=float)
    vel_error = np.array([r["vel_error_norm"] for r in rows], dtype=float)
    foot_error = np.array([r["foot_error_norm_m"] for r in rows], dtype=float)
    cmd = np.array([[r[f"cmd_{name}"] for name in JOINT_NAMES] for r in rows], dtype=float)
    metrics = {
        "profile": profile["name"],
        "description": profile["description"],
        "kp": profile["kp"].tolist(),
        "ki": profile["ki"].tolist(),
        "kd": profile["kd"].tolist(),
        "integral_limit": float(profile["integral_limit"]),
        "position_error_rmse_rad": float(np.sqrt(np.mean(pos_error**2))),
        "velocity_error_rmse_rad_s": float(np.sqrt(np.mean(vel_error**2))),
        "foot_error_rmse_m": float(np.sqrt(np.mean(foot_error**2))),
        "max_position_error_rad": float(np.max(pos_error)),
        "max_velocity_error_rad_s": float(np.max(vel_error)),
        "max_foot_error_m": float(np.max(foot_error)),
        "final_position_error_rad": float(pos_error[-1]),
        "final_velocity_error_rad_s": float(vel_error[-1]),
        "final_foot_error_m": float(foot_error[-1]),
        "max_abs_command_rad_s": float(np.max(np.abs(cmd))),
        "saturation_ratio": float(np.mean([r["command_saturated"] for r in rows])),
    }
    return rows, metrics


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def plot_final_tracking(rows: list[dict], out_dir: Path, profile_name: str) -> dict:
    import matplotlib.pyplot as plt

    time = np.array([r["time"] for r in rows], dtype=float)
    outputs: dict[str, str] = {}

    pos_error = np.array([r["pos_error_norm"] for r in rows], dtype=float)
    vel_error = np.array([r["vel_error_norm"] for r in rows], dtype=float)
    foot_error = np.array([r["foot_error_norm_m"] for r in rows], dtype=float)
    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    axes[0].plot(time, pos_error)
    axes[0].set_ylabel("position error [rad]")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(time, vel_error)
    axes[1].set_ylabel("velocity error [rad/s]")
    axes[1].grid(True, alpha=0.3)
    axes[2].plot(time, foot_error)
    axes[2].set_xlabel("time [s]")
    axes[2].set_ylabel("foot error [m]")
    axes[2].grid(True, alpha=0.3)
    fig.suptitle(f"A1 FR leg velocity PID error ({profile_name})")
    fig.tight_layout()
    path = out_dir / "velocity_pid_error.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs["error_plot"] = str(path)

    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    for axis, name in zip(axes, JOINT_NAMES):
        axis.plot(time, [r[f"qd_{name}"] for r in rows], label="desired")
        axis.plot(time, [r[f"q_{name}"] for r in rows], label="actual", linestyle="--")
        axis.set_ylabel(f"{name} q [rad]")
        axis.grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    axes[-1].set_xlabel("time [s]")
    fig.suptitle(f"A1 FR leg joint position tracking ({profile_name})")
    fig.tight_layout()
    path = out_dir / "joint_position_tracking.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs["joint_position_plot"] = str(path)

    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    for axis, name in zip(axes, JOINT_NAMES):
        axis.plot(time, [r[f"dqd_{name}"] for r in rows], label="desired")
        axis.plot(time, [r[f"dq_{name}"] for r in rows], label="actual", linestyle="--")
        axis.plot(time, [r[f"cmd_{name}"] for r in rows], label="cmd", alpha=0.65)
        axis.set_ylabel(f"{name} dq [rad/s]")
        axis.grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    axes[-1].set_xlabel("time [s]")
    fig.suptitle(f"A1 FR leg velocity tracking and command ({profile_name})")
    fig.tight_layout()
    path = out_dir / "joint_velocity_tracking.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs["joint_velocity_plot"] = str(path)

    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    for axis, name in zip(axes, JOINT_NAMES):
        axis.plot(time, [r[f"cmd_{name}"] for r in rows])
        axis.set_ylabel(f"{name} cmd [rad/s]")
        axis.grid(True, alpha=0.3)
    axes[-1].set_xlabel("time [s]")
    fig.suptitle(f"A1 FR leg PID velocity commands ({profile_name})")
    fig.tight_layout()
    path = out_dir / "velocity_pid_command.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs["command_plot"] = str(path)

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot([r["foot_desired_x"] for r in rows], [r["foot_desired_z"] for r in rows], label="desired")
    ax.plot([r["foot_x"] for r in rows], [r["foot_z"] for r in rows], label="actual", linestyle="--")
    ax.set_xlabel("foot x in base [m]")
    ax.set_ylabel("foot z in base [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    ax.set_title(f"A1 FR foot path from joint PID tracking ({profile_name})")
    fig.tight_layout()
    path = out_dir / "foot_path_tracking_xz.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs["foot_path_plot"] = str(path)
    return outputs


def plot_tuning(metrics_rows: list[dict], out_dir: Path) -> str:
    import matplotlib.pyplot as plt

    names = [r["profile"] for r in metrics_rows]
    x = np.arange(len(names), dtype=float)
    pos_rmse = np.array([r["position_error_rmse_rad"] for r in metrics_rows], dtype=float)
    vel_rmse = np.array([r["velocity_error_rmse_rad_s"] for r in metrics_rows], dtype=float)
    foot_rmse_mm = np.array([1000.0 * r["foot_error_rmse_m"] for r in metrics_rows], dtype=float)
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].bar(x, pos_rmse)
    axes[0].set_ylabel("q RMSE [rad]")
    axes[0].grid(True, axis="y", alpha=0.3)
    axes[1].bar(x, vel_rmse)
    axes[1].set_ylabel("dq RMSE [rad/s]")
    axes[1].grid(True, axis="y", alpha=0.3)
    axes[2].bar(x, foot_rmse_mm)
    axes[2].set_ylabel("foot RMSE [mm]")
    axes[2].set_xticks(x, names, rotation=18)
    axes[2].grid(True, axis="y", alpha=0.3)
    fig.suptitle("Velocity PID tuning comparison")
    fig.tight_layout()
    path = out_dir / "pid_tuning_comparison.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return str(path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--dt", type=float, default=0.002)
    args = parser.parse_args()

    configure_matplotlib()
    import matplotlib.pyplot as plt

    out_dir = ensure_results_dir("task4_pid")
    model = A1LegKinematics()

    q0 = model.default_q.copy()
    qf = np.array([0.18, 1.05, -1.25], dtype=float)
    coeffs = quintic_coefficients(q0, qf, args.duration)
    actuator_tau = np.array([0.045, 0.055, 0.055], dtype=float)
    max_speed = np.array([4.0, 5.0, 5.0], dtype=float)

    all_results = []
    for profile in pid_profiles():
        rows, metrics = simulate_tracking(
            model=model,
            coeffs=coeffs,
            q0=q0,
            duration=args.duration,
            dt=args.dt,
            profile=profile,
            actuator_tau=actuator_tau,
            max_speed=max_speed,
        )
        all_results.append((profile, rows, metrics))

    # Keep the balanced PID as the selected controller for the report; the
    # sweep remains available to justify the choice.
    selected_profile, selected_rows, selected_metrics = next(
        result for result in all_results if result[0]["name"] == "final_pid"
    )
    tuning_rows = [metrics for _, _, metrics in all_results]

    tracking_csv_path = out_dir / "velocity_pid_tracking.csv"
    write_csv(tracking_csv_path, selected_rows)
    tuning_csv_path = out_dir / "pid_tuning_sweep.csv"
    write_csv(tuning_csv_path, tuning_rows)
    final_plots = plot_final_tracking(selected_rows, out_dir, selected_profile["name"])
    tuning_plot = plot_tuning(tuning_rows, out_dir)

    summary = {
        "q0_rad": q0.tolist(),
        "qf_rad": qf.tolist(),
        "duration_sec": args.duration,
        "dt_sec": args.dt,
        "trajectory": {
            "type": "joint-space quintic polynomial",
            "boundary_conditions": "zero initial/final joint velocity and acceleration",
            "coefficients": coeffs.tolist(),
        },
        "controller": {
            "selected_profile": selected_profile["name"],
            "selection_reason": (
                "conservative gains selected for the report; the higher-gain profile "
                "has lower nominal-model RMSE but less margin for unmodeled delay, "
                "noise, and torque limits on a real quadruped"
            ),
            "kp": selected_profile["kp"].tolist(),
            "ki": selected_profile["ki"].tolist(),
            "kd": selected_profile["kd"].tolist(),
            "integral_limit": float(selected_profile["integral_limit"]),
            "actuator_tau_sec": actuator_tau.tolist(),
            "max_speed_rad_s": max_speed.tolist(),
        },
        "selected_metrics": selected_metrics,
        "tuning_metrics": tuning_rows,
        "outputs": {
            "tracking_csv": str(tracking_csv_path),
            "tuning_csv": str(tuning_csv_path),
            "tuning_plot": tuning_plot,
            **final_plots,
        },
    }
    summary_path = out_dir / "task4_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
