#!/usr/bin/env python3
"""One supervised J1 speed trial. Preview by default; never changes limits."""
import argparse
import csv
import math
from pathlib import Path
import statistics
import time


def profile(start, end, speed, acceleration):
    """Return (seconds, position, velocity) trapezoid knots for Hermite control."""
    distance = abs(end - start)
    if distance < 1e-6:
        return [(0.0, start, 0.0)]
    direction = 1.0 if end > start else -1.0
    peak = min(speed, math.sqrt(distance * acceleration))
    ramp = peak / acceleration
    cruise = max(0.0, distance / peak - ramp)
    knots = [(0.0, start, 0.0),
             (ramp, start + direction * peak * ramp / 2, direction * peak)]
    if cruise > 1e-8:
        knots.append((ramp + cruise, end - direction * peak * ramp / 2,
                      direction * peak))
    knots.append((2 * ramp + cruise, end, 0.0))
    return knots


def cancel_and_confirm(handle, wait):
    """Cancel this goal and wait until the server finishes its hold operation."""
    wait(handle.cancel_goal_async(), 3)
    result = wait(handle.get_result_async(), 4)
    if result.status not in (4, 5, 6):  # succeeded, canceled, aborted
        raise RuntimeError('Trajectory stop not confirmed')


def self_test():
    for start, end, speed, acceleration in [(-2, 2, .8, .2), (2, -2, .8, .2),
                                             (0, .01, 1.5, .2)]:
        knots = profile(start, end, speed, acceleration)
        assert knots[0] == (0.0, start, 0.0) and knots[-1][1:] == (end, 0.0)
        for (t0, p0, v0), (t1, p1, v1) in zip(knots, knots[1:]):
            duration = t1 - t0
            assert duration > 0
            for i in range(101):
                s = i / 100
                p = ((2*s**3-3*s**2+1)*p0 + (s**3-2*s**2+s)*duration*v0
                     + (-2*s**3+3*s**2)*p1 + (s**3-s**2)*duration*v1)
                v = ((6*s*s-6*s)*p0 + (3*s*s-4*s+1)*duration*v0
                     + (-6*s*s+6*s)*p1 + (3*s*s-2*s)*duration*v1) / duration
                assert min(start, end)-1e-9 <= p <= max(start, end)+1e-9
                assert abs(v) <= speed+1e-9
            assert abs((v1-v0)/duration) <= acceleration+1e-9
    assert profile(1, 1, .2, .2) == [(0.0, 1, 0.0)]
    from types import SimpleNamespace
    calls = []
    handle = SimpleNamespace(
        cancel_goal_async=lambda: calls.append('cancel') or 'cancel_future',
        get_result_async=lambda: calls.append('result') or 'result_future')
    def fake_wait(future, timeout):
        calls.append(('wait', future))
        return SimpleNamespace(status=5)
    cancel_and_confirm(handle, fake_wait)
    assert calls == ['cancel', ('wait', 'cancel_future'), 'result', ('wait', 'result_future')]
    try:
        cancel_and_confirm(handle, lambda future, timeout: SimpleNamespace(status=2))
    except RuntimeError:
        pass
    else:
        raise AssertionError('Executing goal incorrectly accepted as stopped')
    print('PASS: trajectory limits and cancellation confirmation (no /zekeep/stop needed)')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', type=float, default=.2, help='rad/s')
    parser.add_argument('--accel', type=float, default=.2, help='rad/s^2')
    parser.add_argument('--repeat', type=int, default=1, help='round trips')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not all(math.isfinite(x) and x > 0 for x in (args.speed, args.accel)):
        parser.error('speed and accel must be finite and positive')
    if args.repeat < 1:
        parser.error('repeat must be positive')
    cruise = 4 / args.speed - args.speed / args.accel
    if cruise < .5:
        parser.error('4 rad travel cannot provide 0.5 s cruise at these settings; '
                     'this is a travel/acceleration limit, not a measured motor limit')

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from moveit_msgs.srv import GetStateValidity

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('j1_speed_test')
    names = [f'joint{i}' for i in range(1, 7)]
    latest = [None, 0.0]
    def state_callback(msg):
        latest[:] = [msg, time.monotonic()]
    subscription = node.create_subscription(
        JointState, '/zekeep/joint_states', state_callback, qos_profile_sensor_data)
    action = ActionClient(node, FollowJointTrajectory, '/zekeep/follow_joint_trajectory')
    validity = node.create_client(GetStateValidity, '/check_state_validity')
    active = False
    handle = send_future = None
    output = None

    def wait(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError('ROS response timeout')
            rclpy.spin_once(node, timeout_sec=.05)
        return future.result()

    def positions():
        deadline = time.monotonic() + 3
        previous = latest[1]
        while latest[1] <= previous:
            if time.monotonic() > deadline:
                raise RuntimeError('No fresh /zekeep/joint_states')
            rclpy.spin_once(node, timeout_sec=.05)
        msg = latest[0]
        indexes = [msg.name.index(name) for name in names]
        q = [msg.position[i] for i in indexes]
        v = [msg.velocity[i] for i in indexes]
        if not all(math.isfinite(x) for x in q + v) or max(map(abs, v)) > .03:
            raise RuntimeError('Arm feedback invalid or arm is moving')
        if abs(q[0]) > 2.58:
            raise RuntimeError('J1 outside allowed range')
        return q

    try:
        fixed = positions()
        if not validity.wait_for_service(timeout_sec=5):
            raise RuntimeError('Start MoveIt: /check_state_validity is unavailable')
        if not action.wait_for_server(timeout_sec=5):
            raise RuntimeError('/zekeep/follow_joint_trajectory action unavailable')
        # Check both the approach and the entire test sweep against the loaded scene.
        snapshot = latest[0]
        low, high = min(-2.0, fixed[0]), max(2.0, fixed[0])
        count = math.ceil((high - low) / .02)
        for i in range(count + 1):
            request = GetStateValidity.Request()
            request.robot_state.joint_state.name = list(snapshot.name)
            request.robot_state.joint_state.position = list(snapshot.position)
            request.robot_state.joint_state.position[snapshot.name.index('joint1')] = low + (high-low)*i/count
            request.robot_state.is_diff = True
            if not wait(validity.call_async(request), 3).valid:
                raise RuntimeError('Path collision/constraint check failed; no motion sent')
        print(f'J2-J6 held at {fixed[1:]}; J1: approach -2 at <=0.1 rad/s, '
              f'then -2 <-> +2; speed={args.speed}, accel={args.accel}, '
              f'cruise={cruise:.2f}s; repeats={args.repeat}', flush=True)
        if not args.execute:
            print('Preview only. --execute moves hardware. Scene checks do not model cables/unmodeled objects.')
            return
        path = Path.cwd() / f'j1_speed_{time.time_ns()}.csv'
        output = path.open('x', newline='')
        writer = csv.writer(output)
        writer.writerow(['leg', 'monotonic_s', 'desired_q1', 'actual_q1',
                         'desired_v1', 'actual_v1', 'max_all_joint_error_rad'])
        print(f'CSV: {path}', flush=True)
        legs = [(-2.0, .1, min(.2, args.accel))]
        legs += [(target, args.speed, args.accel) for _ in range(args.repeat) for target in (2.0, -2.0)]
        for leg, (target, speed, acceleration) in enumerate(legs):
            current = positions()
            if max(abs(a-b) for a, b in zip(current[1:], fixed[1:])) > .03:
                raise RuntimeError('Other joints moved; refusing next leg')
            knots = profile(current[0], target, speed, acceleration)
            if len(knots) == 1:
                continue
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = names
            for seconds, q1, v1 in knots:
                point = JointTrajectoryPoint()
                point.positions = [q1] + fixed[1:]
                point.velocities = [v1, 0., 0., 0., 0., 0.]
                ns = round(seconds * 1e9)
                point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 10**9)
                goal.trajectory.points.append(point)
            samples, errors = [], []
            feedback_time = [time.monotonic()]
            def feedback_callback(event):
                f = event.feedback
                values = list(f.actual.positions) + list(f.actual.velocities) + list(f.error.positions)
                if len(values) != 18 or not all(math.isfinite(x) for x in values):
                    raise RuntimeError('Invalid trajectory feedback')
                feedback_time[0] = time.monotonic()
                error = max(map(abs, f.error.positions))
                errors.append(error)
                writer.writerow([leg, feedback_time[0], f.desired.positions[0], f.actual.positions[0],
                                 f.desired.velocities[0], f.actual.velocities[0], error])
                if active and (error > .05 or abs(f.actual.positions[0]) >= 2.58):
                    raise RuntimeError('Tracking error >0.05 rad or J1 reached boundary')
                if abs(abs(f.desired.velocities[0])-speed) < 1e-5:
                    samples.append(abs(f.actual.velocities[0]))
            active = True  # Also stop if goal acceptance times out.
            handle = None
            send_future = action.send_goal_async(goal, feedback_callback=feedback_callback)
            handle = wait(send_future, 5)
            if not handle.accepted:
                active = False
                raise RuntimeError('Controller rejected goal; check controller state/authorization')
            result_future = handle.get_result_async()
            deadline = time.monotonic() + knots[-1][0] + 10
            while not result_future.done():
                rclpy.spin_once(node, timeout_sec=.05)
                if time.monotonic()-feedback_time[0] > 1 or time.monotonic() > deadline:
                    raise RuntimeError('Trajectory feedback/result timeout')
            result = result_future.result()
            if result.status != 4 or result.result.error_code != 0:
                raise RuntimeError(f'Trajectory failed: {result.result.error_string}')
            active = False
            output.flush()
            if leg:
                if len(samples) < 10:
                    raise RuntimeError('Not enough cruise samples; speed not verified')
                measured = statistics.median(samples)
                print(f'Leg {leg}: target={speed:.3f}, actual cruise median={measured:.3f} rad/s, '
                      f'max tracking error={max(errors):.4f} rad', flush=True)
                if abs(measured-speed) > .1*speed:
                    raise RuntimeError('Cruise speed error >10%; do not increase speed')
        print('Trial complete. Arm remains at J1=-2 rad; no automatic speed increase.')
    finally:
        if active:
            active = False  # Allow cancellation feedback without repeated monitor exceptions.
            try:
                if handle is None:
                    handle = wait(send_future, 3)
                if handle.accepted:
                    cancel_and_confirm(handle, wait)
                    print('STOP: trajectory terminal state confirmed', flush=True)
                else:
                    print('STOP: goal was rejected; no trajectory started', flush=True)
            except BaseException as exc:
                print(f'STOP NOT CONFIRMED: use physical emergency stop! {exc}', flush=True)
        if output:
            output.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, Exception) as exc:
        raise SystemExit(f'Stopped: {exc}')
