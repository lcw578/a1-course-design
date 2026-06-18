#!/usr/bin/env python3
"""Task 6: computed-torque control compared with velocity PID."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.io_utils import configure_matplotlib, ensure_results_dir
from common.pid import VectorPID
from common.planar_leg_dynamics import PlanarLegDynamics, PlanarLegParams
from common.trajectory import evaluate_quintic, make_time_grid, quintic_coefficients


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def rmse(rows: list[dict], key: str) -> float:
    values = np.array([r[key] for r in rows], dtype=float)
    return float(np.sqrt(np.mean(values**2)))


def simulate_velocity_pid(times: np.ndarray, coeffs: np.ndarray, dt: float) -> dict:
    pid = VectorPID(
        kp=np.array([8.0, 7.0], dtype=float),
        ki=np.array([0.4, 0.3], dtype=float),
        kd=np.array([0.22, 0.20], dtype=float),
        integral_limit=0.35,
    )
    q, _, _ = evaluate_quintic(coeffs, 0.0)
    dq = np.zeros(2, dtype=float)
    actuator_tau = np.array([0.055, 0.055], dtype=float)
    rows = []
    for t in times:
        qd, dqd, _ = evaluate_quintic(coeffs, t)
        qdot_cmd = np.clip(pid.update(qd - q, dqd - dq, dt), -5.0, 5.0)
        ddq = (qdot_cmd - dq) / actuator_tau
        dq = dq + ddq * dt
        q = q + dq * dt
        rows.append(
            (
                t,
                q.copy(),
                dq.copy(),
                qd,
                dqd,
                ddq.copy(),
                qdot_cmd.copy(),
                np.zeros(2),
            )
        )
    return {"name": "velocity_pid", "rows": rows}


def simulate_computed_torque(
    name: str,
    times: np.ndarray,
    coeffs: np.ndarray,
    dt: float,
    plant: PlanarLegDynamics,
    controller_model: PlanarLegDynamics,
    wn: np.ndarray,
) -> dict:
    q, _, _ = evaluate_quintic(coeffs, 0.0)
    dq = np.zeros(2, dtype=float)
    kp = wn**2
    kv = 2.0 * wn
    rows = []
    for t in times:
        qd, dqd, ddqd = evaluate_quintic(coeffs, t)
        a = ddqd + kv * (dqd - dq) + kp * (qd - q)
        tau = controller_model.inverse_dynamics(q, dq, a)
        tau = np.clip(tau, -33.5, 33.5)
        ddq = plant.forward_dynamics(q, dq, tau)
        dq = dq + ddq * dt
        q = q + dq * dt
        rows.append((t, q.copy(), dq.copy(), qd, dqd, ddq.copy(), np.zeros(2), tau.copy()))
    return {"name": name, "rows": rows, "kp": kp, "kv": kv, "wn": wn}


def flatten_rows(sim_result: dict) -> list[dict]:
    rows = []
    for t, q, dq, qd, dqd, ddq, qdot_cmd, tau in sim_result["rows"]:
        rows.append(
            {
                "controller": sim_result["name"],
                "time": t,
                "q1": q[0],
                "q2": q[1],
                "dq1": dq[0],
                "dq2": dq[1],
                "ddq1": ddq[0],
                "ddq2": ddq[1],
                "qd1": qd[0],
                "qd2": qd[1],
                "dqd1": dqd[0],
                "dqd2": dqd[1],
                "qdot_cmd1": qdot_cmd[0],
                "qdot_cmd2": qdot_cmd[1],
                "tau1": tau[0],
                "tau2": tau[1],
                "pos_error_norm": float(np.linalg.norm(qd - q)),
                "vel_error_norm": float(np.linalg.norm(dqd - dq)),
            }
        )
    return rows


def metrics_for_rows(rows: list[dict]) -> dict:
    return {
        "position_rmse_rad": rmse(rows, "pos_error_norm"),
        "velocity_rmse_rad_s": rmse(rows, "vel_error_norm"),
        "max_position_error_rad": float(max(r["pos_error_norm"] for r in rows)),
        "final_position_error_rad": float(rows[-1]["pos_error_norm"]),
        "max_abs_tau_Nm": float(max(max(abs(r["tau1"]), abs(r["tau2"])) for r in rows)),
        "max_abs_ddq_rad_s2": float(max(max(abs(r["ddq1"]), abs(r["ddq2"])) for r in rows)),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=3.5)
    parser.add_argument("--dt", type=float, default=0.001)
    args = parser.parse_args()

    configure_matplotlib()
    import matplotlib.pyplot as plt

    out_dir = ensure_results_dir("task6_computed_torque")
    times = make_time_grid(args.duration, args.dt)
    q0 = np.array([0.75, -1.35], dtype=float)
    qf = np.array([0.95, -1.05], dtype=float)
    coeffs = quintic_coefficients(q0, qf, args.duration)

    base_params = PlanarLegParams()
    nominal = PlanarLegDynamics(base_params)
    plant_plus20 = PlanarLegDynamics(
        PlanarLegParams(
            m1=base_params.m1 * 1.2,
            m2=base_params.m2 * 1.2,
        )
    )
    plant_minus20 = PlanarLegDynamics(
        PlanarLegParams(
            m1=base_params.m1 * 0.8,
            m2=base_params.m2 * 0.8,
        )
    )

    wn = np.array([12.0, 12.0], dtype=float)
    pid_result = simulate_velocity_pid(times, coeffs, args.dt)
    ct_nominal = simulate_computed_torque(
        "computed_torque", times, coeffs, args.dt, nominal, nominal, wn
    )
    ct_plus20 = simulate_computed_torque(
        "computed_torque_mass_plus20", times, coeffs, args.dt, plant_plus20, nominal, wn
    )
    ct_minus20 = simulate_computed_torque(
        "computed_torque_mass_minus20", times, coeffs, args.dt, plant_minus20, nominal, wn
    )

    sim_results = [pid_result, ct_nominal, ct_plus20, ct_minus20]
    all_rows = []
    for result in sim_results:
        all_rows.extend(flatten_rows(result))

    csv_path = out_dir / "controller_comparison.csv"
    write_csv(csv_path, all_rows)

    fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    metrics = {}
    for result in sim_results:
        rows = [r for r in all_rows if r["controller"] == result["name"]]
        time = np.array([r["time"] for r in rows], dtype=float)
        pos_err = np.array([r["pos_error_norm"] for r in rows], dtype=float)
        vel_err = np.array([r["vel_error_norm"] for r in rows], dtype=float)
        axes[0].plot(time, pos_err, label=result["name"])
        axes[1].plot(time, vel_err, label=result["name"])
        metrics[result["name"]] = metrics_for_rows(rows)
    axes[0].set_ylabel("position error norm [rad]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("velocity error norm [rad/s]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    fig.suptitle("Velocity PID vs computed-torque tracking")
    fig.tight_layout()
    tracking_plot = out_dir / "controller_comparison.png"
    fig.savefig(tracking_plot, dpi=180)
    plt.close(fig)

    ct_names = [
        "computed_torque",
        "computed_torque_mass_plus20",
        "computed_torque_mass_minus20",
    ]
    fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    for name, label in zip(ct_names, ("nominal", "mass +20%", "mass -20%")):
        rows = [r for r in all_rows if r["controller"] == name]
        time = np.array([r["time"] for r in rows], dtype=float)
        axes[0].plot(time, [r["pos_error_norm"] for r in rows], label=label)
        axes[1].plot(time, [r["vel_error_norm"] for r in rows], label=label)
    axes[0].set_ylabel("position error norm [rad]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("velocity error norm [rad/s]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    fig.suptitle("Computed-torque robustness under mass uncertainty")
    fig.tight_layout()
    robustness_plot = out_dir / "computed_torque_robustness.png"
    fig.savefig(robustness_plot, dpi=180)
    plt.close(fig)

    nominal_rows = [r for r in all_rows if r["controller"] == "computed_torque"]
    pid_rows = [r for r in all_rows if r["controller"] == "velocity_pid"]
    fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    time = np.array([r["time"] for r in nominal_rows], dtype=float)
    axes[0].plot(time, [r["tau1"] for r in nominal_rows], label="tau1 computed torque")
    axes[0].plot(time, [r["tau2"] for r in nominal_rows], label="tau2 computed torque")
    axes[0].set_ylabel("torque [Nm]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].plot(time, [r["qdot_cmd1"] for r in pid_rows], label="qdot_cmd1 PID")
    axes[1].plot(time, [r["qdot_cmd2"] for r in pid_rows], label="qdot_cmd2 PID")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("velocity cmd [rad/s]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    fig.suptitle("Control outputs: torque vs velocity command")
    fig.tight_layout()
    control_output_plot = out_dir / "control_output_comparison.png"
    fig.savefig(control_output_plot, dpi=180)
    plt.close(fig)

    pid_rmse = metrics["velocity_pid"]["position_rmse_rad"]
    ct_rmse = metrics["computed_torque"]["position_rmse_rad"]
    summary = {
        "model": "2R sagittal thigh-calf subsystem",
        "q0_rad": q0.tolist(),
        "qf_rad": qf.tolist(),
        "duration_sec": args.duration,
        "dt_sec": args.dt,
        "control_law": {
            "computed_torque": "tau = M(q)a + C(q,dq) + g(q)",
            "auxiliary_acceleration": (
                "a = ddq_d + Kv(dq_d - dq) + Kp(q_d - q)"
            ),
            "velocity_pid_baseline": "qdot_cmd = Kp(q_d-q) + Kd(dq_d-dq) + Ki integral(q_d-q)dt",
        },
        "computed_torque_gains": {
            "wn_rad_s": wn.tolist(),
            "kp": (wn**2).tolist(),
            "kv": (2.0 * wn).tolist(),
            "damping_ratio": 1.0,
        },
        "metrics": metrics,
        "performance_summary": {
            "computed_torque_vs_pid_position_rmse_ratio": float(ct_rmse / pid_rmse),
            "computed_torque_position_rmse_improvement_percent": float(
                100.0 * (1.0 - ct_rmse / pid_rmse)
            ),
        },
        "robustness_note": (
            "Mass uncertainty increases tracking error because the controller uses "
            "the nominal inverse-dynamics model. The +20% heavier plant is the "
            "worst case in this experiment, while the -20% case shows a smaller "
            "but still visible degradation."
        ),
        "outputs": {
            "csv": str(csv_path),
            "tracking_plot": str(tracking_plot),
            "robustness_plot": str(robustness_plot),
            "control_output_plot": str(control_output_plot),
        },
    }
    summary_path = out_dir / "task6_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
