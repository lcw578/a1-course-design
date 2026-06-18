#!/usr/bin/env python3
"""Task 5: simplified A1 leg dynamics derivation support and validation."""

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
from common.planar_leg_dynamics import PlanarLegDynamics


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def rms(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(values**2)))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--dt", type=float, default=0.001)
    parser.add_argument("--seed", type=int, default=23)
    args = parser.parse_args()

    configure_matplotlib()
    import matplotlib.pyplot as plt

    out_dir = ensure_results_dir("task5_dynamics")
    dynamics = PlanarLegDynamics()
    rng = np.random.default_rng(args.seed)

    residual_rows = []
    max_residual = 0.0
    for index in range(200):
        q = rng.uniform([-0.4, -1.7], [1.2, -0.7])
        dq = rng.uniform([-1.0, -1.0], [1.0, 1.0])
        tau = rng.uniform([-8.0, -8.0], [8.0, 8.0])
        ddq = dynamics.forward_dynamics(q, dq, tau)
        terms = dynamics.decompose_dynamics(q, dq, ddq)
        residual = terms["tau_model"] - tau
        residual_norm = float(np.linalg.norm(residual))
        max_residual = max(max_residual, residual_norm)
        residual_rows.append(
            {
                "index": index,
                "q1": q[0],
                "q2": q[1],
                "dq1": dq[0],
                "dq2": dq[1],
                "tau1": tau[0],
                "tau2": tau[1],
                "ddq1": ddq[0],
                "ddq2": ddq[1],
                "inertia1": terms["inertia"][0],
                "inertia2": terms["inertia"][1],
                "coriolis1": terms["coriolis"][0],
                "coriolis2": terms["coriolis"][1],
                "gravity1": terms["gravity"][0],
                "gravity2": terms["gravity"][1],
                "residual1": residual[0],
                "residual2": residual[1],
                "residual_norm": residual_norm,
            }
        )

    residual_csv = out_dir / "inverse_forward_dynamics_residual.csv"
    write_csv(residual_csv, residual_rows)

    q = np.array([0.75, -1.25], dtype=float)
    dq = np.zeros(2, dtype=float)
    response_rows = []
    steps = int(args.duration / args.dt) + 1
    for step in range(steps):
        t = step * args.dt
        tau = dynamics.gravity_vector(q) + np.array(
            [
                0.45 * np.sin(2.0 * np.pi * 0.7 * t),
                0.25 * np.cos(2.0 * np.pi * 0.5 * t),
            ],
            dtype=float,
        )
        ddq = dynamics.forward_dynamics(q, dq, tau)
        terms = dynamics.decompose_dynamics(q, dq, ddq)
        residual = terms["tau_model"] - tau
        response_rows.append(
            {
                "time": t,
                "q1": q[0],
                "q2": q[1],
                "dq1": dq[0],
                "dq2": dq[1],
                "ddq1": ddq[0],
                "ddq2": ddq[1],
                "tau1": tau[0],
                "tau2": tau[1],
                "inertia1": terms["inertia"][0],
                "inertia2": terms["inertia"][1],
                "coriolis1": terms["coriolis"][0],
                "coriolis2": terms["coriolis"][1],
                "gravity1": terms["gravity"][0],
                "gravity2": terms["gravity"][1],
                "tau_model1": terms["tau_model"][0],
                "tau_model2": terms["tau_model"][1],
                "residual1": residual[0],
                "residual2": residual[1],
                "residual_norm": float(np.linalg.norm(residual)),
            }
        )
        dq = dq + ddq * args.dt
        q = q + dq * args.dt

    response_csv = out_dir / "forced_response.csv"
    write_csv(response_csv, response_rows)

    time = np.array([r["time"] for r in response_rows], dtype=float)
    q_series = np.array([[r["q1"], r["q2"]] for r in response_rows], dtype=float)
    dq_series = np.array([[r["dq1"], r["dq2"]] for r in response_rows], dtype=float)
    ddq_series = np.array([[r["ddq1"], r["ddq2"]] for r in response_rows], dtype=float)
    tau_series = np.array([[r["tau1"], r["tau2"]] for r in response_rows], dtype=float)
    inertia_series = np.array([[r["inertia1"], r["inertia2"]] for r in response_rows], dtype=float)
    coriolis_series = np.array([[r["coriolis1"], r["coriolis2"]] for r in response_rows], dtype=float)
    gravity_series = np.array([[r["gravity1"], r["gravity2"]] for r in response_rows], dtype=float)
    tau_model_series = np.array([[r["tau_model1"], r["tau_model2"]] for r in response_rows], dtype=float)
    residual_norm = np.array([r["residual_norm"] for r in response_rows], dtype=float)

    fig, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
    axes[0].plot(time, q_series[:, 0], label="q1 thigh")
    axes[0].plot(time, q_series[:, 1], label="q2 calf")
    axes[0].set_ylabel("joint position [rad]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].plot(time, dq_series[:, 0], label="dq1")
    axes[1].plot(time, dq_series[:, 1], label="dq2")
    axes[1].set_ylabel("joint velocity [rad/s]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    axes[2].plot(time, tau_series[:, 0], label="tau1")
    axes[2].plot(time, tau_series[:, 1], label="tau2")
    axes[2].set_xlabel("time [s]")
    axes[2].set_ylabel("applied torque [Nm]")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()
    fig.suptitle("Simplified A1 sagittal leg forced response")
    fig.tight_layout()
    response_plot = out_dir / "forced_response.png"
    fig.savefig(response_plot, dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(time, inertia_series[:, 0], label="M(q)ddq")
    axes[0].plot(time, coriolis_series[:, 0], label="C(q,dq)")
    axes[0].plot(time, gravity_series[:, 0], label="g(q)")
    axes[0].plot(time, tau_series[:, 0], label="tau", linewidth=1.4)
    axes[0].set_ylabel("joint 1 torque terms [Nm]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    axes[1].plot(time, inertia_series[:, 1], label="M(q)ddq")
    axes[1].plot(time, coriolis_series[:, 1], label="C(q,dq)")
    axes[1].plot(time, gravity_series[:, 1], label="g(q)")
    axes[1].plot(time, tau_series[:, 1], label="tau", linewidth=1.4)
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("joint 2 torque terms [Nm]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")
    fig.suptitle("Dynamics term contribution comparison")
    fig.tight_layout()
    term_plot = out_dir / "dynamics_term_contributions.png"
    fig.savefig(term_plot, dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    axes[0].plot(time, ddq_series[:, 0], label="ddq1")
    axes[0].plot(time, ddq_series[:, 1], label="ddq2")
    axes[0].set_ylabel("joint acceleration [rad/s^2]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].semilogy(time, np.maximum(residual_norm, 1e-18))
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("model residual norm")
    axes[1].grid(True, alpha=0.3)
    fig.suptitle("Acceleration and inverse-forward dynamics residual")
    fig.tight_layout()
    residual_plot = out_dir / "dynamics_residual_validation.png"
    fig.savefig(residual_plot, dpi=180)
    plt.close(fig)

    params = dynamics.params
    summary = {
        "model": "2R sagittal thigh-calf subsystem",
        "equation": "M(q)ddq + C(q,dq) + g(q) = tau",
        "assumptions": [
            "The A1 hip abduction/adduction joint is excluded from task-5 dynamics.",
            "The model is restricted to the sagittal thigh-calf plane.",
            "The base/trunk is treated as fixed during the 2R derivation.",
        ],
        "parameters": {
            "l1_m": params.l1,
            "l2_m": params.l2,
            "m1_kg": params.m1,
            "m2_kg": params.m2,
            "lc1_m": params.lc1,
            "lc2_m": params.lc2,
            "i1_kg_m2": params.i1,
            "i2_kg_m2": params.i2,
            "gravity_m_s2": params.gravity,
        },
        "max_residual_norm": max_residual,
        "forced_response_metrics": {
            "joint1": {
                "inertia_rms_Nm": rms(inertia_series[:, 0]),
                "coriolis_rms_Nm": rms(coriolis_series[:, 0]),
                "gravity_rms_Nm": rms(gravity_series[:, 0]),
                "applied_tau_rms_Nm": rms(tau_series[:, 0]),
            },
            "joint2": {
                "inertia_rms_Nm": rms(inertia_series[:, 1]),
                "coriolis_rms_Nm": rms(coriolis_series[:, 1]),
                "gravity_rms_Nm": rms(gravity_series[:, 1]),
                "applied_tau_rms_Nm": rms(tau_series[:, 1]),
            },
            "max_time_series_residual_norm": float(np.max(residual_norm)),
        },
        "analysis_note": (
            "The gravity term dominates the static load, especially on joint 1. "
            "The Coriolis term remains relatively small in this low-speed forced "
            "response, while the inertia term varies with commanded acceleration."
        ),
        "duration_sec": args.duration,
        "dt_sec": args.dt,
        "outputs": {
            "residual_csv": str(residual_csv),
            "forced_response_csv": str(response_csv),
            "forced_response_plot": str(response_plot),
            "term_contribution_plot": str(term_plot),
            "residual_plot": str(residual_plot),
        },
    }
    summary_path = out_dir / "task5_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
