#!/usr/bin/env python3
"""Supervised, collision-checked small joint sweeps with torque logging.

Preview is the default. Requires an already enabled controller and MoveIt scene.
"""

import argparse
import csv
import math
from pathlib import Path
import time

from identify_dynamics import FIELDS, NAMES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--amplitude", type=float, default=0.06, help="rad, at most 0.10")
    parser.add_argument("--duration", type=float, default=2.0, help="seconds per leg, at least 2")
    parser.add_argument("--reverse", action="store_true", help="reverse axis order for holdout run")
    parser.add_argument("--execute", action="store_true", help="send trajectories to hardware")
    args = parser.parse_args()
    if not (math.isfinite(args.amplitude) and 0 < args.amplitude <= 0.10):
        parser.error("amplitude must be in (0, 0.10] rad")
    if not (math.isfinite(args.duration) and args.duration >= 2):
        parser.error("duration must be at least 2 seconds")

    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.srv import GetStateValidity
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectoryPoint
    from zekeep_msgs.msg import ArmStatus

    rclpy.init()
    node = rclpy.create_node("dynamics_supervised_trial")
    latest = {"joint": None, "status": None, "stamp": 0.0, "feedback": 0.0}
    output = None
    writer = None
    handle = None
    stopping = False

    def joint_callback(msg):
        if any(name not in msg.name for name in NAMES + ["gripper_joint", "right_joint"]):
            return
        latest["joint"], latest["stamp"] = msg, time.monotonic()
        if writer:
            indexes = [msg.name.index(name) for name in NAMES]
            finger = [msg.name.index(name) for name in ("gripper_joint", "right_joint")]
            values = [float(seq[i]) for seq in (msg.position, msg.velocity, msg.effort) for i in indexes]
            values += [float(msg.position[i]) for i in finger]
            if len(values) == 20 and all(map(math.isfinite, values)):
                writer.writerow([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, *values])

    node.create_subscription(JointState, "/zekeep/joint_states", joint_callback, qos_profile_sensor_data)
    node.create_subscription(ArmStatus, "/zekeep/arm_status", lambda msg: latest.__setitem__("status", msg),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE))
    scene = node.create_client(GetStateValidity, "/check_state_validity")
    action = ActionClient(node, FollowJointTrajectory, "/zekeep/follow_joint_trajectory")

    def wait(future, seconds):
        deadline = time.monotonic() + seconds
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError("ROS response timeout")
            rclpy.spin_once(node, timeout_sec=0.05)
        return future.result()

    def ready():
        msg, status = latest["joint"], latest["status"]
        if msg is None or status is None or time.monotonic() - latest["stamp"] > 0.3:
            raise RuntimeError("feedback missing or stale")
        if not status.enabled or not status.control_loop_active or status.state_machine != "IDLE" or status.error_codes:
            raise RuntimeError("controller must be enabled, active, IDLE and fault-free")
        indexes = [msg.name.index(name) for name in NAMES]
        q = [float(msg.position[i]) for i in indexes]
        v = [float(msg.velocity[i]) for i in indexes]
        if not all(map(math.isfinite, q + v)) or max(map(abs, v)) > 0.03:
            raise RuntimeError("joint feedback invalid or arm moving")
        return q

    def check_pose(source, target):
        for fraction in range(17):
            request = GetStateValidity.Request()
            request.robot_state.joint_state.name = list(source.name)
            request.robot_state.joint_state.position = list(source.position)
            for joint, name in enumerate(NAMES):
                index = source.name.index(name)
                request.robot_state.joint_state.position[index] += (
                    fraction * (target[joint] - source.position[index]) / 16
                )
            request.robot_state.is_diff = True
            response = wait(scene.call_async(request), 3)
            if not response.valid:
                raise RuntimeError(f"MoveIt rejected path at sample {fraction}/16")

    try:
        end = time.monotonic() + 4
        while (latest["joint"] is None or latest["status"] is None) and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        baseline = ready()
        if not scene.wait_for_service(timeout_sec=3) or not action.wait_for_server(timeout_sec=3):
            raise RuntimeError("MoveIt scene or trajectory action unavailable")
        order = list(range(6))
        if args.reverse:
            order.reverse()
        for axis in order:
            for direction in (-1, 1):
                target = list(baseline)
                target[axis] += direction * args.amplitude
                check_pose(latest["joint"], target)
        print(f"Validated ±{args.amplitude:.3f} rad on J1-J6; start={baseline}; duration={args.duration}s/leg", flush=True)
        if not args.execute:
            print("Preview only; --execute moves hardware.")
            return
        output = args.output.open("x", newline="")
        writer = csv.writer(output)
        writer.writerow(FIELDS)
        for axis in order:
            for direction in (-1, 1):
                for target in (baseline[axis] + direction * args.amplitude, baseline[axis]):
                    current = ready()
                    if max(abs(current[i] - baseline[i]) for i in range(6) if i != axis) > 0.03:
                        raise RuntimeError("other joint drifted over 0.03 rad")
                    target_positions = list(baseline)
                    target_positions[axis] = target
                    check_pose(latest["joint"], target_positions)
                    goal = FollowJointTrajectory.Goal()
                    goal.trajectory.joint_names = NAMES
                    point = JointTrajectoryPoint()
                    point.positions = target_positions
                    point.velocities = [0.0] * 6
                    point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(args.duration * 1e9), 10**9)
                    goal.trajectory.points.append(point)
                    latest["feedback"] = time.monotonic()

                    def feedback(event):
                        if stopping:
                            return
                        latest["feedback"] = time.monotonic()
                        error = list(event.feedback.error.positions)
                        if len(error) != 6 or not all(map(math.isfinite, error)) or max(map(abs, error)) > 0.06:
                            raise RuntimeError("trajectory tracking error over 0.06 rad")

                    sent_at = time.monotonic()
                    handle = wait(action.send_goal_async(goal, feedback_callback=feedback), 5)
                    if not handle.accepted:
                        raise RuntimeError("trajectory goal rejected")
                    result_future = handle.get_result_async()
                    deadline = time.monotonic() + args.duration + 8
                    while not result_future.done():
                        now = time.monotonic()
                        if (now > deadline or now - latest["stamp"] > 0.5 or
                                (now - sent_at < args.duration + 0.5 and now - latest["feedback"] > 1)):
                            raise RuntimeError("trajectory feedback/result timeout")
                        rclpy.spin_once(node, timeout_sec=0.05)
                    result = result_future.result()
                    if result.status != 4 or result.result.error_code != 0:
                        raise RuntimeError(f"trajectory failed: {result.result.error_string}")
                    handle = None
                    output.flush()
                    print(f"{NAMES[axis]} target={target:.4f} rad completed", flush=True)
        print(f"Trial complete: {args.output}")
    finally:
        stopping = True
        if handle is not None and handle.accepted:
            try:
                wait(handle.cancel_goal_async(), 3)
                result = wait(handle.get_result_async(), 4)
                if result.status not in (4, 5, 6):
                    raise RuntimeError("trajectory stop not confirmed")
                print("STOP: trajectory terminal state confirmed", flush=True)
            except Exception as exc:
                print(f"STOP NOT CONFIRMED: use physical emergency stop! {exc}", flush=True)
        if output:
            output.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        raise SystemExit(f"Stopped: {exc}")
