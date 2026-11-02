from __future__ import annotations

import numpy as np
import pinocchio as pin


def rotate(vector, quaternion):
    x, y, z, w = quaternion
    qv = np.array([x, y, z], dtype=np.float64)
    v = np.asarray(vector, dtype=np.float64)
    return v + 2.0 * np.cross(qv, np.cross(qv, v) + w * v)


def joint_velocity(jacobian, position_error, current_rotation, desired_rotation,
                   previous_velocity, max_speed=0.455):
    """Translation first, orientation in its joint nullspace (world frame)."""
    j = np.asarray(jacobian, dtype=float)
    error = np.asarray(position_error, dtype=float).copy()
    if j.shape != (6, 6) or error.shape != (3,) or not np.all(np.isfinite(j)) or not np.all(np.isfinite(error)):
        raise ValueError("invalid Jacobian or target error")
    error[np.abs(error) < 0.005] = 0.0
    linear = np.clip(2.4 * error, -0.12, 0.12)
    rotation_error = current_rotation @ pin.log3(current_rotation.T @ desired_rotation)
    if np.linalg.norm(rotation_error) < np.deg2rad(2.0):
        rotation_error[:] = 0.0
    if not np.any(error) and not np.any(rotation_error):
        return np.zeros(6)
    angular = np.clip(1.5 * rotation_error, -0.3, 0.3)

    jp, jw = j[:3], j[3:]
    smallest = np.linalg.svd(jp, compute_uv=False)[-1]
    damping = 0.01 + max(0.0, 0.05 - smallest)
    primary = jp.T @ np.linalg.solve(jp @ jp.T + damping**2 * np.eye(3), linear)
    nullspace = np.eye(6) - np.linalg.pinv(jp, rcond=1e-4) @ jp
    secondary_j = jw @ nullspace
    secondary = secondary_j.T @ np.linalg.solve(
        secondary_j @ secondary_j.T + damping**2 * np.eye(3), angular - jw @ primary)
    speed = (primary + nullspace @ secondary) * min(1.0, smallest / 0.05)
    speed = np.clip(1.3 * speed, -max_speed, max_speed)
    return 0.25 * speed + 0.75 * np.asarray(previous_velocity, dtype=float)


if __name__ == "__main__":
    q = [0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)]
    assert np.allclose(rotate([1, 0, 0], q), [0, 1, 0])
    assert np.all(joint_velocity(np.eye(6), np.zeros(3), np.eye(3), np.eye(3), np.ones(6)) == 0)
    j = np.eye(6)
    velocity = joint_velocity(j, np.array([0.1, 0.0, 0.0]), np.eye(3), np.eye(3), np.zeros(6))
    assert np.isclose(velocity[0], 0.039, atol=1e-4) and np.allclose(velocity[1:], 0)
    j[3, 0] = 1.0
    velocity = joint_velocity(j, np.array([0.1, 0.0, 0.0]), np.eye(3), np.eye(3), np.zeros(6))
    assert abs((j @ velocity)[3]) < 0.001
