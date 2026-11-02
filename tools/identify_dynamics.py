#!/usr/bin/env python3
"""Record real joint feedback and assess Pinocchio inertial identifiability.

Recording is passive. Run collision-checked trajectories through the existing
controller while this tool records; fitting never changes the live URDF.
"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import time

import numpy as np


NAMES = [f"joint{i}" for i in range(1, 7)]
FIELDS = ["stamp_s"] + [f"{kind}{i}" for kind in ("q", "v", "tau") for i in range(1, 7)]
FIELDS += ["q_gripper_joint", "q_right_joint"]
URDF = Path(__file__).resolve().parents[1] / "src/zekeep_bringup/description/urdf/sixaxis.urdf"


def record(path, seconds):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    if seconds <= 0 or not np.isfinite(seconds):
        raise ValueError("seconds must be finite and positive")
    rclpy.init()
    node = rclpy.create_node("dynamics_feedback_recorder")
    count = 0
    last_stamp = -np.inf
    try:
        with path.open("x", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(FIELDS)

            def callback(msg):
                nonlocal count, last_stamp
                required = NAMES + ["gripper_joint", "right_joint"]
                if len(msg.name) != len(set(msg.name)) or any(name not in msg.name for name in required):
                    return
                indexes = [msg.name.index(name) for name in NAMES]
                finger_indexes = [msg.name.index(name) for name in required[6:]]
                if any(len(values) <= max(indexes) for values in (msg.position, msg.velocity, msg.effort)):
                    return
                if len(msg.position) <= max(finger_indexes):
                    return
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                values = [float(seq[i]) for seq in (msg.position, msg.velocity, msg.effort) for i in indexes]
                values += [float(msg.position[i]) for i in finger_indexes]
                if stamp <= last_stamp or not np.all(np.isfinite(values)):
                    return
                writer.writerow([stamp, *values])
                last_stamp = stamp
                count += 1

            node.create_subscription(JointState, "/zekeep/joint_states", callback, qos_profile_sensor_data)
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if count < 20:
        raise RuntimeError(f"only {count} valid samples; controller may be offline")
    print(f"Recorded {count} samples: {path}")


def load_samples(paths):
    runs = []
    for path in paths:
        with path.open(newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != FIELDS:
                raise ValueError(f"{path}: expected columns {FIELDS}")
            rows = [[float(row[key]) for key in FIELDS] for row in reader]
        data = np.asarray(rows, dtype=float)
        if data.ndim != 2 or data.shape[0] < 21 or not np.all(np.isfinite(data)):
            raise ValueError(f"{path}: need at least 21 finite samples")
        dt = np.diff(data[:, 0])
        if np.any(dt <= 0) or np.max(dt) > 0.1:
            raise ValueError(f"{path}: nonmonotonic timestamp or gap over 100 ms")
        if np.median(dt) > 0.025:
            raise ValueError(f"{path}: feedback below 40 Hz")
        runs.append(data)
    return runs


def design(model, run):
    import pinocchio as pin
    from scipy.signal import savgol_filter

    t = run[:, 0]
    q, v, tau = run[:, 1:7], run[:, 7:13], run[:, 13:19]
    # Differentiate measured velocity; smooth over roughly 110 ms at 100 Hz.
    dt = np.median(np.diff(t))
    window = max(7, int(round(0.11 / dt)) | 1)
    window = min(window, len(t) if len(t) % 2 else len(t) - 1)
    smooth_v = savgol_filter(v, window, 3, axis=0)
    a = np.gradient(smooth_v, t, axis=0)
    data = model.createData()
    q_full = pin.neutral(model)
    blocks, targets = [], []
    for k in range(window // 2, len(t) - window // 2, 3):
        q_full[:6] = q[k]
        q_full[6:8] = run[k, 19:21]
        v_full = np.zeros(model.nv)
        a_full = np.zeros(model.nv)
        v_full[:6], a_full[:6] = smooth_v[k], a[k]
        regressor = pin.computeJointTorqueRegressor(model, data, q_full, v_full, a_full)
        # Leave the gripper's fixed geometry and its unmeasured joints at URDF values.
        blocks.append(np.asarray(regressor[:6, 10:70]).copy())
        targets.append(tau[k] - pin.rnea(model, data, q_full, v_full, a_full)[:6])
    return np.vstack(blocks), np.concatenate(targets)


def fit(paths, urdf, output):
    import pinocchio as pin

    runs = load_samples(paths)
    model = pin.buildModelFromUrdf(str(urdf))
    if (model.nq != 8 or model.nv != 8 or
            [str(model.names[i]) for i in range(1, 9)] != NAMES + ["gripper_joint", "right_joint"]):
        raise ValueError("URDF joint order differs from controller feedback")
    train = [design(model, run) for run in runs[:-1]]
    check = design(model, runs[-1])
    if not train:
        raise ValueError("provide at least two separate runs; final run is holdout validation")
    X, y = np.vstack([part[0] for part in train]), np.concatenate([part[1] for part in train])
    Xcheck, ycheck = check
    # Fit motor torque bias, viscous and Coulomb friction as nuisance parameters.
    # These are not link inertias and must not be baked into the URDF.
    def nuisance(runs_):
        rows = []
        for run in runs_:
            dt = np.median(np.diff(run[:, 0]))
            window = min(max(7, int(round(0.11 / dt)) | 1), len(run) - (len(run) + 1) % 2)
            from scipy.signal import savgol_filter
            v = savgol_filter(run[:, 7:13], window, 3, axis=0)
            for velocity in v[window // 2:len(run) - window // 2:3]:
                rows.append(np.hstack((np.eye(6), np.diag(velocity), np.diag(np.tanh(velocity / 0.02)))))
        return np.vstack(rows)

    F, Fcheck = nuisance(runs[:-1]), nuisance(runs[-1:])
    if len(X) != len(F) or len(Xcheck) != len(Fcheck):
        raise RuntimeError("design and friction samples differ")
    A = np.hstack((X, F))
    scale = np.linalg.norm(A, axis=0)
    active = scale > 1e-10
    U, singular, Vt = np.linalg.svd(A[:, active] / scale[active], full_matrices=False)
    rank = int(np.sum(singular > singular[0] * 1e-3))
    delta_scaled = Vt[:rank].T @ ((U[:, :rank].T @ y) / singular[:rank])
    delta = np.zeros(A.shape[1])
    delta[active] = delta_scaled / scale[active]
    predicted = np.hstack((Xcheck, Fcheck)) @ delta
    rmse_before = float(np.sqrt(np.mean(ycheck**2)))
    rmse_after = float(np.sqrt(np.mean((ycheck - predicted)**2)))
    result = {
        "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
        "train_samples": len(X) // 6,
        "holdout_samples": len(Xcheck) // 6,
        "regressor_columns": int(A.shape[1]),
        "identifiable_rank": rank,
        "holdout_rmse_nominal_Nm": rmse_before,
        "holdout_rmse_fit_Nm": rmse_after,
        "fully_identifiable": rank == A.shape[1],
        "note": "Estimates refer to Pinocchio joint rigid bodies (joint6 includes fixed gripper parts), not isolated URDF links. Motor torque calibration, friction, contact and model errors can bias them. Never apply automatically.",
    }
    if rmse_after < rmse_before:
        candidate = {}
        for i, name in enumerate(NAMES):
            values = model.inertias[i + 1].toDynamicParameters() + delta[10 * i:10 * (i + 1)]
            inertia = pin.Inertia.FromDynamicParameters(values)
            moments = np.linalg.eigvalsh(inertia.inertia)
            if inertia.mass <= 0 or min(moments) <= 0 or 2 * max(moments) >= sum(moments):
                candidate = {}
                result["candidate_rejected"] = f"{name}: nonphysical mass or inertia"
                break
            candidate[name] = {
                "mass_kg": float(inertia.mass),
                "center_m_in_joint_frame": np.asarray(inertia.lever).tolist(),
                "inertia_kg_m2_at_center_in_joint_frame": np.asarray(inertia.inertia).tolist(),
            }
        if candidate:
            result["minimum_norm_candidate_not_unique"] = candidate
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    recorder = commands.add_parser("record", help="passively record six-axis feedback")
    recorder.add_argument("output", type=Path)
    recorder.add_argument("--seconds", type=float, default=60)
    fitting = commands.add_parser("fit", help="fit and check identifiability; final CSV is holdout")
    fitting.add_argument("csv", type=Path, nargs="+", help="at least two separate runs")
    fitting.add_argument("--urdf", type=Path, default=URDF)
    fitting.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "record":
        record(args.output, args.seconds)
    else:
        fit(args.csv, args.urdf, args.output)


if __name__ == "__main__":
    main()
