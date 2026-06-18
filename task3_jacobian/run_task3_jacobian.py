#!/usr/bin/env python3
"""Task 3: Jacobian validation, singularity scan, and force mapping."""

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


def matrix_rank(jacobian: np.ndarray, tol: float = 1e-5) -> int:
    return int(np.linalg.matrix_rank(jacobian, tol=tol))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--leg", default="FR", choices=["FR", "FL", "RR", "RL"])
    parser.add_argument("--seed", type=int, default=13)
    parser.add_argument("--samples", type=int, default=3000)
    args = parser.parse_args()

    configure_matplotlib()
    import matplotlib.pyplot as plt

    model = A1LegKinematics()
    out_dir = ensure_results_dir("task3_jacobian")

    rng = np.random.default_rng(args.seed)
    q_samples = rng.uniform(model.lower_limits, model.upper_limits, size=(args.samples, 3))
    rows = []
    max_fd_error = 0.0
    for q in q_samples:
        jac = model.jacobian(q, args.leg)
        jac_fd = model.finite_difference_jacobian(q, args.leg)
        js = model.spatial_jacobian(q, args.leg)
        jb = model.body_jacobian(q, args.leg)
        js_fd = model.finite_difference_spatial_jacobian(q, args.leg)
        jb_fd = model.finite_difference_body_jacobian(q, args.leg)
        fd_error = float(np.linalg.norm(jac - jac_fd))
        js_fd_error = float(np.linalg.norm(js - js_fd))
        jb_fd_error = float(np.linalg.norm(jb - jb_fd))
        point_relation_error = float(
            np.linalg.norm(jac - model.point_velocity_jacobian_from_spatial(q, args.leg))
        )
        det = float(np.linalg.det(jac))
        singular_values = np.linalg.svd(jac, compute_uv=False)
        condition = float(singular_values[0] / max(singular_values[-1], 1e-12))
        rank = matrix_rank(jac)
        max_fd_error = max(max_fd_error, fd_error)
        rows.append(
            {
                "q_hip": q[0],
                "q_thigh": q[1],
                "q_calf": q[2],
                "determinant": det,
                "condition_number": condition,
                "rank": rank,
                "fd_jacobian_error": fd_error,
                "fd_spatial_jacobian_error": js_fd_error,
                "fd_body_jacobian_error": jb_fd_error,
                "spatial_to_point_jacobian_error": point_relation_error,
            }
        )

    csv_path = out_dir / f"{args.leg.lower()}_jacobian_scan.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    det_values = np.array([r["determinant"] for r in rows], dtype=float)
    cond_values = np.array([r["condition_number"] for r in rows], dtype=float)
    q_calf_values = np.array([r["q_calf"] for r in rows], dtype=float)

    fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    axes[0].scatter(q_calf_values, det_values, s=3, alpha=0.45)
    axes[0].axhline(0.0, color="black", linewidth=0.8)
    axes[0].set_ylabel("det(J)")
    axes[0].grid(True, alpha=0.3)
    axes[1].scatter(q_calf_values, np.clip(cond_values, 0.0, 500.0), s=3, alpha=0.45)
    axes[1].set_xlabel("calf joint q3 [rad]")
    axes[1].set_ylabel("condition number clipped at 500")
    axes[1].grid(True, alpha=0.3)
    fig.suptitle(f"A1 {args.leg} Jacobian singularity scan")
    fig.tight_layout()
    plot_path = out_dir / f"{args.leg.lower()}_singularity_scan.png"
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)

    q_nominal = np.array([0.0, 0.8, -1.5], dtype=float)
    jac_nominal = model.jacobian(q_nominal, args.leg)
    js_nominal = model.spatial_jacobian(q_nominal, args.leg)
    jb_nominal = model.body_jacobian(q_nominal, args.leg)
    foot_force = np.array([0.0, 0.0, 20.0], dtype=float)
    torque = jac_nominal.T @ foot_force
    singular_index = int(np.argmin(np.abs(det_values)))

    summary = {
        "leg": args.leg,
        "max_finite_difference_jacobian_error": max_fd_error,
        "max_finite_difference_spatial_jacobian_error": max(
            float(r["fd_spatial_jacobian_error"]) for r in rows
        ),
        "max_finite_difference_body_jacobian_error": max(
            float(r["fd_body_jacobian_error"]) for r in rows
        ),
        "max_spatial_to_point_jacobian_error": max(
            float(r["spatial_to_point_jacobian_error"]) for r in rows
        ),
        "minimum_abs_determinant_sample": rows[singular_index],
        "nominal_q_rad": q_nominal.tolist(),
        "nominal_rank": matrix_rank(jac_nominal),
        "nominal_point_velocity_jacobian_Jv": jac_nominal.tolist(),
        "nominal_space_jacobian_Js_omega_then_v": js_nominal.tolist(),
        "nominal_body_jacobian_Jb_omega_then_v": jb_nominal.tolist(),
        "jacobian_definition_note": (
            "Jv is the 3x3 foot-origin linear velocity Jacobian used for "
            "singularity determinant and tau = Jv^T f mapping. Js and Jb are "
            "6x3 Modern Robotics twist Jacobians ordered as [omega; v]. Since "
            "the A1 leg has only 3 actuated joints, Js/Jb are not square and "
            "do not have determinants; rank and singular values are the proper "
            "full-twist diagnostics."
        ),
        "force_mapping_example": {
            "foot_force_N": foot_force.tolist(),
            "joint_torque_Nm": torque.tolist(),
        },
        "nullspace_note": (
            "For the selected 3D foot-position task and 3 actuated joints, J is square. "
            "Away from singular configurations the nullspace is zero-dimensional; near "
            "singular samples the rank drops and non-zero joint velocity directions can "
            "produce little foot velocity."
        ),
        "outputs": {
            "csv": str(csv_path),
            "plot": str(plot_path),
        },
    }
    summary_path = out_dir / f"{args.leg.lower()}_task3_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
