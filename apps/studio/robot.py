"""ROS-free model and adapter for the existing calibrated hardware manager."""
from pathlib import Path
import sys
import threading
import time

import numpy as np

ROOT = Path(sys._MEIPASS) if getattr(sys, "frozen", False) else Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "zekeeparm_SDK"))
sys.path.insert(0, str(ROOT / "src" / "zekeepcontroller"))
URDF = ROOT / "src/zekeep_bringup/description/urdf/sixaxis.urdf"
SRDF = ROOT / "src/zekeep_moveit_config/config/zekeep.srdf"
HARDWARE = ROOT / "src/zekeep_bringup/config/zekeep_hardware.yaml"
# Studio's initial/read pose: J1–J6 = 0°, 40°, 40°, -35°, 0°, 0°.
READY = [0.0, 0.698131701, 0.698131701, -0.610865238, 0.0, 0.0]
WORKTABLE = "fixed_worktable"


class CollisionError(ValueError):
    def __init__(self, kind, pairs, links):
        self.kind = kind
        self.links = sorted(links)
        label = {"environment": "环境", "self": "机械臂自", "mixed": "机械臂自与环境"}[kind]
        super().__init__(f"{label}碰撞：" + " / ".join(pairs))


class RobotModel:
    def __init__(self):
        import pinocchio as pin
        from zekeeparm_SDK.kinematics import load_robot_model

        self.pin = pin
        self.model = load_robot_model(str(URDF))
        self.lock = threading.RLock()
        self.names = [f"joint{i}" for i in range(1, 7)]
        self.indices = [self.model.joints[self.model.getJointId(n)].idx_q for n in self.names]
        self.limits = [(float(self.model.lowerPositionLimit[i]),
                        float(self.model.upperPositionLimit[i])) for i in self.indices]
        self.frame = "official_tcp"
        from zekeepcontroller.hardware_config import resolve_hardware_config
        _, config = resolve_hardware_config(str(HARDWARE), "", "COM3")
        self.home = list(config["safe_home"]["positions"])
        self.geometry = pin.buildGeomFromUrdf(
            self.model, str(URDF), pin.GeometryType.COLLISION,
            package_dirs=[str(ROOT / "src")])
        self.geometry.addAllCollisionPairs()
        pin.removeCollisionPairs(self.model, self.geometry, str(SRDF))
        table_frame = self.model.getFrameId("base_link")
        table = pin.GeometryObject(WORKTABLE, self.model.frames[table_frame].parentJoint,
                                   table_frame, pin.SE3.Identity(),
                                   pin.hppfcl.Box(.9, 1.2, .8))
        table.placement.translation = np.array([.388, 0, -.4])
        self.table_id = self.geometry.addGeometryObject(table, self.model)
        self.table_pairs = set()
        self.geometry_links = []
        for index, obj in enumerate(self.geometry.geometryObjects):
            frame_name = self.model.frames[obj.parentFrame].name if obj.name != WORKTABLE else WORKTABLE
            self.geometry_links.append(frame_name)
            if index != self.table_id and frame_name != "base_link":
                self.geometry.addCollisionPair(pin.CollisionPair(index, self.table_id))
                self.table_pairs.add((index, self.table_id))

    def configuration(self, joints, opening=0):
        q = self.pin.neutral(self.model)
        q[self.indices] = joints
        for name in ("gripper_joint", "right_joint"):
            q[self.model.joints[self.model.getJointId(name)].idx_q] = opening / 2000
        return q

    def fk(self, joints, frame=None):
        from zekeeparm_SDK.kinematics import compute_fk

        with self.lock:
            frame = frame or self.frame
            if frame not in [f.name for f in self.model.frames]:
                raise ValueError("坐标系不存在")
            position, rotation, transform = compute_fk(self.model, self.configuration(joints), frame)
            return {"position_m": position.tolist(),
                    "rpy_rad": self.pin.rpy.matrixToRpy(rotation).tolist(), "frame": frame,
                    "rotation": rotation.tolist(), "transform": transform.tolist()}

    def ik(self, position, rpy, seed, max_retries=0, checkpoint=lambda: None):
        from zekeeparm_SDK.kinematics.inverse_kinematics import solve_ik, IKParams

        target = self.pin.SE3(self.pin.rpy.rpyToMatrix(*rpy), np.array(position))
        with self.lock:
            for attempt in range(max_retries + 1):
                checkpoint()
                q = seed if attempt == 0 else [float(np.random.uniform(lo, hi)) for lo, hi in self.limits]
                result = solve_ik(self.model, self.model.createData(), self.model.getFrameId(self.frame),
                                  target, self.configuration(q), IKParams(max_iter=500), controlled_joints=6)
                if result.success:
                    break
            if not result.success:
                raise ValueError("目标位姿无有效 IK 解，请调整位置或姿态")
            return result.q.tolist()

    def info(self):
        return {"dof": 6, "joint_names": self.names, "limits_rad": self.limits,
                "default_frame": self.frame, "frame_names": [f.name for f in self.model.frames]}

    def trajectory_stats(self, start, end, duration, profile, dt, accel_ratio, checkpoint=lambda: None):
        from dataclasses import asdict
        from zekeeparm_SDK.kinematics import compute_fk
        from zekeeparm_SDK.trajectory.sampler import TrajPlanParams, TrajProfile, plan_cartesian_geodesic_trajectory
        from zekeeparm_SDK.trajectory.clik_tracker import track_trajectory
        from zekeeparm_SDK.trajectory.trajectory_planner import compute_traj_stats

        params = TrajPlanParams(dt=dt, profile=TrajProfile(profile), accel_ratio=accel_ratio)
        with self.lock:
            a, b = self.configuration(start), self.configuration(end)
            first = self.pin.SE3(compute_fk(self.model, a, self.frame)[2])
            last = self.pin.SE3(compute_fk(self.model, b, self.frame)[2])
            trajectory = plan_cartesian_geodesic_trajectory(first, last, duration, params).trajectory
            # Track one interval at a time so stop/heartbeat checks remain responsive.
            from zekeeparm_SDK.trajectory.sampler import CartesianTrajectory
            points, q = [], a
            for point in trajectory.points():
                checkpoint()
                segment = CartesianTrajectory()
                segment.add_point(point.time, point.pose)
                tracked = track_trajectory(self.model, self.model.getFrameId(self.frame), segment, q)
                points.extend(tracked)
                q = tracked[-1].q
            return asdict(compute_traj_stats(self.model, self.model.getFrameId(self.frame),
                                            points, first, last, duration, params))

    def check_path(self, start, end, opening=0):
        """Sample the same monotone joint path used by the executor."""
        with self.lock:
            data, geometry_data = self.model.createData(), self.geometry.createData()
            count = max(1, int(np.ceil(np.max(np.abs(np.subtract(end, start))) / 0.02)))
            for q in np.linspace(start, end, count + 1):
                if self.pin.computeCollisions(self.model, data, self.geometry, geometry_data,
                                              self.configuration(q, opening), False):
                    pairs, links, environment, self_collision = [], set(), False, False
                    for pair, result in zip(self.geometry.collisionPairs, geometry_data.collisionResults):
                        if result.isCollision():
                            first, second = pair.first, pair.second
                            pairs.append(f"{self.geometry.geometryObjects[first].name} / {self.geometry.geometryObjects[second].name}")
                            is_environment = tuple(sorted((first, second))) in self.table_pairs
                            environment |= is_environment
                            self_collision |= not is_environment
                            for index in (first, second):
                                if index != self.table_id:
                                    links.add(self.geometry_links[index])
                    if pairs:
                        kind = "mixed" if environment and self_collision else "environment" if environment else "self"
                        raise CollisionError(kind, pairs, links)


class SDKDevice:
    def __init__(self, channel, model):
        from zekeepcontroller.hardware_manager import HardwareManager

        self.manager = HardwareManager(str(HARDWARE), channel=channel, auto_enable=False)
        self.model = model
        self.manager.validate_home_path = lambda samples: self._check_home(samples)
        self.manager.connect()

    def _check_home(self, samples):
        for first, second in zip(samples, samples[1:]):
            self.model.check_path(first, second, self.state()["opening_mm"])

    def state(self):
        manager = self.manager
        if manager.enabled and not manager.control_loop_active:
            raise RuntimeError("设备控制循环已停止")
        joints, velocity, _ = manager.get_joint_state()
        if not np.all(np.isfinite(joints)) or not np.all(np.isfinite(velocity)):
            raise RuntimeError("设备反馈包含无效数值")
        errors = manager.get_motor_feedback_errors()
        if errors:
            raise RuntimeError("电机反馈异常：" + "; ".join(errors))
        position = manager.get_gripper_state()[0]
        return {"joints": joints.tolist(), "velocity": velocity.tolist(),
                "opening_mm": float(np.clip((position - manager.gripper_close_position) /
                    (manager.gripper_open_position - manager.gripper_close_position), 0.0, 1.0)
                    * manager.gripper_max_width * 1000), "enabled": manager.enabled}

    def validate_lifecycle(self, mode):
        if mode != self.manager._arm_control_mode:
            raise ValueError("真机控制模式必须匹配硬件配置；请失能断开后修改 control.arm_control_mode 并重连")

    def lifecycle(self, mode):
        self.validate_lifecycle(mode)
        self.enable()

    def enable(self):
        self.manager._validate_startup_feedback()
        self.manager.enable()

    def begin(self):
        self.manager.begin_servo_stream()

    def target(self, joints, velocity):
        self.manager.set_servo_target(joints, velocity)

    def end(self):
        self.manager.stop_servo_stream()

    def gripper(self, opening):
        manager = self.manager
        opening = float(opening)
        max_width_mm = manager.gripper_max_width * 1000
        if not np.isfinite(opening) or not 0.0 <= opening <= max_width_mm:
            raise ValueError("夹爪开口超出硬件配置范围")
        manager.set_gripper_target(manager.gripper_close_position + opening / max_width_mm
                                   * (manager.gripper_open_position - manager.gripper_close_position))

    def stop(self):
        if self.manager.enabled:
            self.manager.stop_and_hold()
            deadline, settled = time.monotonic() + 3, 0
            while settled < 3:
                state = self.state()
                settled = settled + 1 if max(abs(v) for v in state["velocity"]) < .05 else 0
                if time.monotonic() > deadline:
                    raise RuntimeError("停止后关节仍在运动")
                time.sleep(.02)

    def disable(self):
        self.manager.disable()

    def close(self):
        if self.manager.enabled:
            raise RuntimeError("请支撑机械臂并显式失能后再断开设备")
        self.manager.shutdown()
