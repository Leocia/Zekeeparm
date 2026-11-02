import http.client
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import numpy as np
from unittest.mock import patch, Mock, call
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import Session, compile_program, sample
from robot import READY, ROOT, RobotModel, SDKDevice
from studio import App, make_server


class StudioTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = RobotModel()

    def test_kinematics_roundtrip(self):
        pose = self.model.fk(READY)
        seed = list(READY)
        seed[0] += .05
        result = self.model.ik(pose["position_m"], pose["rpy_rad"], seed)
        actual = self.model.fk(result)
        self.assertLess(max(abs(a-b) for a,b in zip(actual["position_m"], pose["position_m"])), 1e-4)
        self.assertLess(max(abs(a-b) for a,b in zip(actual["rpy_rad"], pose["rpy_rad"])), 1e-3)

    def test_program_expansion_and_limits(self):
        body = [{"id":"point", "type":"point", "name":"A", "duration":1}]
        program = [{"id":"loop", "type":"repeat", "count":2, "body":body}]
        result = compile_program(program, {"A":READY}, self.model.limits)
        self.assertEqual([s["joints"] for s in result], [READY,READY])
        for bad in (float("nan"), float("inf"), True, -10):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                compile_program([{"id":"x", "type":"wait", "seconds":bad}], {}, self.model.limits)
        with self.assertRaises(ValueError):
            compile_program([{"id":"x", "type":"python", "code":"print(1)"}], {}, self.model.limits)
        with self.assertRaises(ValueError):
            compile_program(body, {}, self.model.limits)
        empty = []
        for _ in range(5):
            empty=[{"id":"x", "type":"repeat", "count":100, "body":empty}]
        with self.assertRaises(ValueError):
            compile_program(empty, {}, self.model.limits)

    def test_simulation_and_cancel_do_not_continue(self):
        session = Session(self.model)
        session.connect("simulation")
        session.enable(True)
        steps=[{"id":"wait", "type":"wait", "seconds":3},
               {"id":"gripper", "type":"gripper", "opening":0}]
        session.start(steps)
        with self.assertRaises(RuntimeError):
            session.start(steps)
        session.stop()
        self.assertFalse(session.busy)
        self.assertEqual(session.phase,"stopped")
        self.assertEqual(session.opening,40)
        session.start([{"id":"done", "type":"wait", "seconds":0}])
        session.worker.join(2)
        self.assertEqual(session.phase,"completed")

    def test_move_reaches_target(self):
        session=Session(self.model)
        session.connect("simulation")
        session.enable(True)
        target=list(READY);target[0]+=.02
        session.start([{"id":"move", "type":"move", "joints":target, "duration":.2}])
        session.worker.join(3)
        self.assertEqual(session.phase,"completed",session.error)
        self.assertEqual(session.q,target)

    def test_self_collision_is_not_ignored(self):
        # Fixed configuration with known intersecting non-adjacent CAD links.
        bad=[-1.5877828599,2.2618585977,.4154121296,-.8658102239,.3797391187,1.1801035138]
        with self.assertRaisesRegex(ValueError,"碰撞"):
            self.model.check_path(bad,bad,40)

    def test_new_blocks_prepare_and_capture_actual_pose(self):
        session = Session(self.model)
        session.connect("simulation")
        program = [
            dict(id="start", type="lifecycle", action="start", mode="posvel"),
            dict(id="profile", type="profile", profile="min_jerk", dt=.02, accel_ratio=.25),
            dict(id="delta", type="delta", joint=0, delta=.01, duration=.2),
            dict(id="capture", type="save_point", name="P1"),
            dict(id="point", type="point", name="P1", duration=.2),
            dict(id="fk", type="show", value=dict(type="fk", joints=dict(type="current_joints"), frame="official_tcp", output="transform")),
        ]
        steps = compile_program(program, {}, self.model.limits)
        checked = session.prepare(steps, points={})
        self.assertEqual(checked[2]["profile"], "min_jerk")
        self.assertEqual(checked[2]["joints"][0], READY[0]+.01)
        self.assertEqual(checked[2]["joints"], checked[4]["joints"])
        self.assertEqual(len(checked[-1]["result"]), 4)
        self.assertFalse(session.enabled)
        self.assertEqual(session.points, {})
        session.start(steps, {})
        session.worker.join(3)
        self.assertEqual(session.phase, "completed", session.error)
        self.assertEqual(session.points["P1"], session.q)
        self.assertEqual(session.point_revision, 1)
        self.assertTrue(any("已记录示教点" in e["message"] for e in session.events))
        with self.assertRaisesRegex(ValueError, "已存在"):
            session.prepare(steps, points=session.points)

    def test_value_blocks_and_both_condition_branches(self):
        session = Session(self.model)
        pose = self.model.fk(READY)
        reachable = dict(type="reachable", pose=pose["position_m"]+pose["rpy_rad"], max_retries=0)
        condition = dict(id="if", type="if", condition=reachable,
                         then=[dict(id="yes", type="wait", seconds=0)],
                         **{"else": [dict(id="no", type="wait", seconds=.1)]})
        checked = session.prepare(compile_program([condition], {}, self.model.limits))
        self.assertEqual(checked[0]["id"], "yes")
        condition["condition"]["pose"] = [2, 2, 2, 0, 0, 0]
        checked = session.prepare(compile_program([condition], {}, self.model.limits))
        self.assertEqual(checked[0]["id"], "no")
        self.assertEqual(session.evaluate(dict(type="model_info"), READY, lambda: None)["dof"], 6)
        target = session.evaluate(dict(type="ik", pose=pose["position_m"]+pose["rpy_rad"], max_retries=1), READY, lambda: None)
        self.assertEqual(len(target), 6)
        for field, size in (("position_m", 3), ("rotation", 3), ("transform", 4)):
            result = session.evaluate(dict(type="fk", joints=dict(type="current_joints"), frame="link6", output=field), READY, lambda: None)
            self.assertEqual(len(result), size)
        with self.assertRaisesRegex(ValueError, "坐标系不存在"):
            self.model.fk(READY, "missing_frame")

    def test_profiles_stats_and_sdk_frame_consistency(self):
        from zekeeparm_SDK.trajectory.sampler import _apply_profile, TrajProfile
        for curve in ("cubic", "min_jerk", "trapezoid", "linear"):
            samples = [sample(i/100, curve) for i in range(101)]
            self.assertAlmostEqual(samples[0][0], 0)
            self.assertAlmostEqual(samples[-1][0], 1)
            self.assertTrue(all(a[0] <= b[0] for a,b in zip(samples, samples[1:])))
            self.assertTrue(all(0 <= p <= 1 and v >= 0 for p,v in samples))
            if curve != "cubic":
                for i, (position, _) in enumerate(samples):
                    self.assertAlmostEqual(position, _apply_profile(i/100, TrajProfile(curve), .25))
                stats = self.model.trajectory_stats(READY, READY, .2, curve, .05, .25)
                self.assertEqual(stats["total_points"], 5)
                self.assertEqual(stats["success_rate"], 1)
                self.assertLess(stats["max_ik_error"], 1e-10)

    def test_new_block_limits_and_safe_lifecycle(self):
        session = Session(self.model)
        session.connect("simulation")
        session.enable(True)
        home = dict(id="home", type="home", max_vel=.1, settle_thresh=.01, timeout=3)
        checked = session.prepare(compile_program([home], {}, self.model.limits))
        self.assertEqual(checked[0]["joints"], self.model.home)
        self.assertEqual(checked[0]["profile"], "min_jerk")
        self.assertGreaterEqual(checked[0]["duration"], max(abs(a-b) for a,b in zip(READY,self.model.home))*1.875/.1)
        end = dict(id="end", type="lifecycle", action="end", mode="posvel")
        self.assertEqual([s["type"] for s in session.prepare([end])], ["move", "lifecycle"])
        with self.assertRaisesRegex(ValueError, "位置控制"):
            session.prepare([end, dict(id="move", type="move", joints=READY, duration=.2)])
        with patch.object(self.model, "check_path"):
            checked = session.prepare([dict(id="delta", type="delta", joint=0, delta=10, duration=.2)])
            self.assertEqual(checked[0]["joints"][0], self.model.limits[0][1])
        bad = [dict(id="x", type="delta", joint=True, delta=1, duration=1),
               dict(id="x", type="profile", profile="bad", dt=.02, accel_ratio=.25),
               dict(id="x", type="show", value=dict(type="ik", pose=[0]*6, max_retries=True)),
               dict(id="x", type="show", value=dict(type="traj_stats", target=dict(type="current_joints"), duration=60, profile="linear", dt=.02, accel_ratio=.25)),
               dict(id="x", type="show", value=dict(type="python", code="print(1)"))]
        for step in bad:
            with self.subTest(step=step), self.assertRaises(ValueError):
                compile_program([step], {}, self.model.limits)
        with self.assertRaisesRegex(InterruptedError, "停止"):
            self.model.trajectory_stats(READY, READY, .2, "min_jerk", .05, .25,
                                        lambda: (_ for _ in ()).throw(InterruptedError("停止")))

    def test_disable_block_checks_and_execution(self):
        disable = dict(id="disable", type="disable")
        start = dict(id="start", type="lifecycle", action="start", mode="posvel")
        move = dict(id="move", type="move", joints=READY, duration=.2)
        for sdk in (False, True):
            with self.subTest(sdk=sdk):
                session = Session(self.model)
                session.connect("simulation")
                session.enable(True)
                if sdk:
                    session.device = Mock()
                    session.device.state.side_effect = lambda: dict(joints=READY, opening_mm=40, enabled=session.enabled)
                steps = compile_program([disable], {}, self.model.limits)
                self.assertEqual(session.prepare(steps), steps)
                self.assertTrue(session.enabled)
                if sdk:
                    session.device.disable.assert_not_called()
                for motion in (move, dict(id="gripper", type="gripper", opening=0)):
                    with self.assertRaisesRegex(ValueError, "运动前必须启动"):
                        session.prepare(steps + [motion])
                self.assertEqual(len(session.prepare(steps + [start, move])), 3)
                session.start(steps)
                session.worker.join(2)
                self.assertEqual(session.phase, "completed", session.error)
                self.assertFalse(session.enabled)
                if sdk:
                    session.device.disable.assert_called_once_with()
                    session.device.lifecycle.assert_not_called()

    def test_disable_failure_does_not_report_success_or_continue(self):
        session = Session(self.model)
        session.connect("simulation")
        session.enable(True)
        session.device = Mock()
        session.device.state.return_value = dict(joints=READY, opening_mm=40, enabled=True)
        session.device.disable.side_effect = RuntimeError("失能失败")
        session.start([dict(id="disable", type="disable"),
                       dict(id="start", type="lifecycle", action="start", mode="posvel")])
        session.worker.join(2)
        self.assertEqual(session.phase, "error")
        self.assertEqual(session.error, "失能失败")
        self.assertTrue(session.enabled)
        session.device.lifecycle.assert_not_called()

    def test_enabled_motor_feedback_updates_session(self):
        adapter = SDKDevice.__new__(SDKDevice)
        adapter.manager = Mock(enabled=True, control_loop_active=True,
                               gripper_open_position=1.15, gripper_close_position=0.0,
                               gripper_max_width=.070)
        adapter.manager.get_joint_state.return_value = (np.array(READY), np.zeros(6), np.zeros(6))
        adapter.manager.get_joint_status_codes.return_value = [1]*6
        adapter.manager.get_motor_feedback_errors.return_value = []
        adapter.manager.get_gripper_state.return_value = (0, 0, 0, 1)
        session = Session(self.model)
        session.device = adapter
        session.connected = True
        self.assertFalse(session.enabled)
        self.assertTrue(session.state()["enabled"])
        adapter.manager.get_motor_feedback_errors.return_value = ["joint2: status=0x09"]
        with self.assertRaisesRegex(RuntimeError, "joint2.*0x09"):
            session.state()

    def test_motor_feedback_validation_keeps_faults_and_missing_data(self):
        from zekeepcontroller.hardware_manager import HardwareManager
        manager = HardwareManager.__new__(HardwareManager)
        joint = SimpleNamespace(name="joint1", vendor="damiao")
        motor = Mock()
        manager._robot = SimpleNamespace(_all_joints=[joint], _motor_map={"joint1": motor}, has_gripper=False)
        manager._robot_get_state = Mock()
        manager._arm_group = Mock()
        manager._joint_mapping = Mock()
        for code in (0, 1, 2, 8, 9, 10, 11, 12, 13, 14, 15, None):
            with self.subTest(code=code):
                motor.get_state.return_value = None if code is None else SimpleNamespace(status_code=code)
                self.assertEqual(bool(manager.get_motor_feedback_errors()), code not in (0, 1))
                if code in (0, 1):
                    manager._validate_startup_feedback()
        joint.vendor = "robstride"
        motor.get_state.return_value = SimpleNamespace(status_code=1)
        self.assertTrue(manager.get_motor_feedback_errors())

    def test_hardware_lifecycle_guards_and_home_timeout_without_hardware(self):
        adapter = SDKDevice.__new__(SDKDevice)
        adapter.manager = Mock(_arm_control_mode="posvel")
        with self.assertRaisesRegex(ValueError, "匹配硬件配置"):
            adapter.validate_lifecycle("mit")
        adapter.validate_lifecycle("posvel")
        adapter.lifecycle("posvel")
        self.assertEqual(adapter.manager.mock_calls, [call._validate_startup_feedback(), call.enable()])
        session = Session(self.model)
        session.device = Mock()
        session.device.state.return_value = dict(joints=READY, opening_mm=40, enabled=True, velocity=[.1]*6)
        session.connected = session.enabled = True
        session.start([dict(id="home", type="move", joints=READY, duration=.01, settle_thresh=.001, timeout=.001)])
        session.worker.join(3)
        self.assertEqual(session.phase, "error")
        self.assertIn("未在规定时间内到位", session.error)
        session.device.stop.assert_called_once()
        session.device.disable.assert_not_called()
        session.device.state.return_value["velocity"] = [0]*6
        target = list(READY); target[0] += .001
        session.start([dict(id="profile", type="profile", profile="linear", dt=.02, accel_ratio=.25),
                       dict(id="linear", type="move", joints=target, duration=.02)])
        session.worker.join(3)
        self.assertEqual(session.phase, "completed", session.error)
        self.assertEqual(session.device.target.call_args.args[1], [0]*6)
        calculation = dict(id="calc", type="show", value=dict(type="ik", pose=[0]*6, max_retries=8))
        with self.assertRaisesRegex(ValueError, "计算量过大"):
            compile_program([dict(id="repeat", type="repeat", count=100, body=[calculation])], {}, self.model.limits)

    def test_controller_end_holds_without_compensation_calls(self):
        session = Session(self.model)
        session.device = Mock()
        session.device.state.return_value = dict(joints=self.model.home, opening_mm=40, enabled=True, velocity=[0]*6)
        session.connected = session.enabled = True
        session.start(compile_program([dict(id="end", type="lifecycle", action="end", mode="posvel")], {}, self.model.limits))
        session.worker.join(2)
        self.assertEqual(session.phase, "completed", session.error)
        session.device.stop.assert_called_once()
        self.assertEqual(session.device.manager.mock_calls, [])
        session.device.disable.assert_not_called()
        self.assertNotIn("gravity", session.state())

    def test_logic_variables_lists_functions_and_motion(self):
        literal = lambda v: dict(type="literal", value=v)
        variable = lambda n: dict(type="variable", name=n)
        session = Session(self.model)
        session.connect("simulation"); session.enable(True)
        function = dict(id="function", type="function", name="微调", body=[
            dict(id="increment", type="change", name="次数", value=literal(1)),
            dict(id="replace", type="list_set", name="目标", index=literal(1), value=dict(type="arithmetic", op="add",
                 a=dict(type="list_get", list=variable("目标"), index=literal(1)),
                 b=dict(type="unary", op="to_rad", value=literal(1))))])
        program = [function, dict(id="count", type="set", name="次数", value=literal(0)),
                   dict(id="target", type="set", name="目标", value=dict(type="current_joints")),
                   dict(id="original", type="set", name="原始", value=variable("目标")),
                   dict(id="repeat", type="repeat", count=2, body=[dict(id="call", type="call", name="微调")]),
                   dict(id="info", type="set", name="模型", value=dict(type="model_info")),
                   dict(id="condition", type="if", condition=dict(type="logic", op="and",
                        a=dict(type="compare", op="eq", a=variable("次数"), b=literal(2)),
                        b=dict(type="compare", op="eq", a=dict(type="dict_get", dict=variable("模型"), key="dof"), b=literal(6))),
                        then=[dict(id="move", type="move_value", value=variable("目标"), duration=.2)],
                        **{"else": [dict(id="wrong", type="gripper", opening=0)]}),
                   dict(id="old", type="show", value=variable("原始")),
                   dict(id="result", type="show", value=variable("目标"))]
        steps = compile_program(program, {}, self.model.limits)
        prepared = session.prepare(steps)
        target = list(READY); target[0] += 2 * 3.141592653589793 / 180
        self.assertAlmostEqual(prepared[-1]["result"][0], target[0])
        self.assertEqual(prepared[-2]["result"], READY)
        self.assertNotIn("wrong", [s["id"] for s in prepared])
        self.assertEqual(session.q, READY)
        session.start(steps)
        session.worker.join(3)
        self.assertEqual(session.phase, "completed", session.error)
        self.assertAlmostEqual(session.q[0], target[0])
        self.assertEqual(session.opening, 40)
        with self.assertRaisesRegex(ValueError, "未赋值"):
            session.prepare(compile_program([dict(id="read", type="show", value=variable("次数"))], {}, self.model.limits))

    def test_logic_operators_short_circuit_and_data(self):
        literal = lambda v: dict(type="literal", value=v)
        session = Session(self.model)
        def evaluate(value):
            from core import expression
            return session.evaluate(expression(value, self.model.limits), READY, lambda: None)
        for op, expected in (("add", 7), ("sub", 3), ("mul", 10), ("div", 2.5), ("mod", 1)):
            self.assertEqual(evaluate(dict(type="arithmetic", op=op, a=literal(5), b=literal(2))), expected)
        for op, operand, expected in (("round", 2.5, 2), ("floor", -1.2, -2), ("ceil", -1.2, -1),
                                      ("abs", -3, 3), ("neg", 3, -3), ("to_deg", 3.141592653589793, 180)):
            self.assertAlmostEqual(evaluate(dict(type="unary", op=op, value=literal(operand))), expected)
        missing = dict(type="variable", name="不存在")
        self.assertFalse(evaluate(dict(type="logic", op="and", a=literal(False), b=missing)))
        self.assertTrue(evaluate(dict(type="logic", op="or", a=literal(True), b=missing)))
        self.assertTrue(evaluate(dict(type="not", value=literal(False))))
        self.assertFalse(evaluate(dict(type="compare", op="eq", a=literal(True), b=literal(1))))
        for op in ("lt", "le", "ne"):
            self.assertTrue(evaluate(dict(type="compare", op=op, a=literal(1), b=literal(2))))
        items = dict(type="list", items=[literal(1), literal(2), literal("三")])
        self.assertEqual(evaluate(dict(type="length", value=items)), 3)
        self.assertEqual(evaluate(dict(type="list_get", list=items, index=literal(3))), "三")
        self.assertEqual(evaluate(dict(type="length", value=literal("机械臂"))), 3)
        self.assertEqual(evaluate(dict(type="list", items=[])), [])
        stats = dict(type="traj_stats", target=dict(type="current_joints"), duration=.2, profile="min_jerk", dt=.05, accel_ratio=.25)
        self.assertEqual(evaluate(dict(type="dict_get", dict=stats, key="success_rate")), 1)
        bad = [dict(type="arithmetic", op="div", a=literal(1), b=literal(0)),
               dict(type="arithmetic", op="mul", a=literal(1e12), b=literal(1e12)),
               dict(type="arithmetic", op="add", a=literal(True), b=literal(1)),
               dict(type="logic", op="and", a=literal(True), b=literal(1)),
               dict(type="dict_get", dict=dict(type="model_info"), key="missing"),
               dict(type="length", value=literal(1))]
        bad.extend(dict(type="list_get", list=items, index=literal(i)) for i in (0, -1, 1.5, 4, True))
        for value in bad:
            with self.subTest(value=value), self.assertRaises(ValueError):
                evaluate(value)

    def test_function_validation_and_logic_fail_before_motion(self):
        literal = lambda v: dict(type="literal", value=v)
        call = lambda n: dict(id="call", type="call", name=n)
        function = lambda n, body: dict(id="function", type="function", name=n, body=body)
        bad = [[call("缺失")], [function("A", [call("A")]), call("A")],
               [function("A", [call("B")]), function("B", [call("A")]), call("A")],
               [function("A", []), function("A", []), dict(id="w", type="wait", seconds=0)]]
        for program in bad:
            with self.subTest(program=program), self.assertRaises(ValueError):
                compile_program(program, {}, self.model.limits)
        session = Session(self.model)
        session.connect("simulation"); session.enable(True)
        target = list(READY); target[0] += .01
        steps = compile_program([dict(id="move", type="move", joints=target, duration=.2),
                                 dict(id="bad", type="set", name="x", value=dict(type="arithmetic", op="div", a=literal(1), b=literal(0)))], {}, self.model.limits)
        session.start(steps); session.worker.join(2)
        self.assertEqual(session.phase, "error")
        self.assertIn("除数不能为 0", session.error)
        self.assertEqual(session.q, READY)
        for value in ([literal(1)]*33,):
            with self.assertRaises(ValueError):
                compile_program([dict(id="list", type="set", name="x", value=dict(type="list", items=value))], {}, self.model.limits)
        expanded = [function("A", [dict(id="wait", type="wait", seconds=0)]*30),
                    dict(id="repeat", type="repeat", count=100, body=[call("A")])]
        with self.assertRaises(ValueError):
            compile_program(expanded, {}, self.model.limits)

    def test_no_ros_imports(self):
        code = """
import builtins
original=builtins.__import__
def blocked(name,*a,**kw):
    if name.split('.')[0] in ('rclpy','ament_index_python','moveit_msgs','geometry_msgs'):
        error=ModuleNotFoundError(name);error.name=name.split('.')[0];raise error
    return original(name,*a,**kw)
builtins.__import__=blocked
from robot import RobotModel,HARDWARE
from zekeepcontroller.hardware_config import resolve_hardware_config
model=RobotModel()
path,config=resolve_hardware_config(str(HARDWARE),'','COM3')
assert config['channel']=='COM3'
assert config['joint_calibration']['joint2']['direction']==-1
"""
        result=subprocess.run([sys.executable,"-c",code],cwd=Path(__file__).resolve().parents[1],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_windows_and_linux_serial_selection(self):
        from zekeeparm_SDK.actuator.rebotarm import RebotArm
        arm=RebotArm.__new__(RebotArm)
        with patch('zekeeparm_SDK.actuator.rebotarm.Controller') as controller:
            for channel in ('COM3','com12',r'\\.\COM10','/dev/ttyACM0'):
                arm._channel=channel
                arm._make_controller('damiao')
                controller.from_dm_serial.assert_called_with(channel,921600)
            arm._channel='can0';arm._make_controller('damiao')
            controller.assert_called_with('can0')

    def test_stop_race_does_not_dispatch_gripper(self):
        class Device:
            def state(self):
                return dict(joints=READY, opening_mm=0, enabled=True)
            def gripper(self, _opening):
                calls.append('gripper')
            def stop(self):
                calls.append('stop')
        session=Session(self.model)
        session.device=Device();session.connected=session.enabled=True
        calls=[]
        checkpoint=session.checkpoint
        interrupted=False
        def cancel_after_check():
            nonlocal interrupted
            checkpoint()
            if session.phase=='running' and not interrupted:
                interrupted=True
                session.cancelled.set()
                session.device.stop()
        session.checkpoint=cancel_after_check
        session.start([{'id':'race','type':'gripper','opening':0}])
        session.worker.join(3)
        self.assertEqual(session.phase,'stopped')
        self.assertNotIn('gripper',calls)

    def test_successful_stop_retry_recovers(self):
        class Device:
            attempts=0
            def stop(self):
                self.attempts+=1
                if self.attempts==1:
                    raise RuntimeError('temporary feedback failure')
            def disable(self):
                pass
        session=Session(self.model)
        session.device=Device();session.connected=session.enabled=True
        with self.assertRaisesRegex(RuntimeError,'停止保持未确认'):
            session.stop()
        self.assertTrue(session.blocked)
        session.stop()
        self.assertFalse(session.blocked)
        session.enable(False)
        self.assertFalse(session.enabled)


class HTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory()
        cls.app=App(cls.temp.name)
        cls.server=make_server(cls.app,0)
        cls.thread=threading.Thread(target=cls.server.serve_forever,daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown();cls.server.server_close();cls.thread.join();cls.temp.cleanup()

    def request(self,route,body=None,headers=None):
        connection=http.client.HTTPConnection('127.0.0.1',self.server.server_port)
        default={'Content-Type':'application/json','X-Studio-Request':'1'}
        default.update(headers or {})
        connection.request('GET' if body is None else 'POST',route,None if body is None else json.dumps(body),default)
        response=connection.getresponse();data=response.read();status=response.status;connection.close()
        return status,data

    def test_clear_logs_keeps_session_state_and_accepts_new_events(self):
        session = self.app.session
        session.log("清空前的日志")
        before = session.state()
        self.assertEqual(self.request('/api/logs/clear', {'extra': 1})[0], 400)
        self.assertEqual(self.request('/api/logs/clear', {}, headers={'X-Studio-Request': ''})[0], 403)
        self.assertEqual(session.state()['events'], before['events'])
        self.assertEqual(self.request('/api/logs/clear', {})[0], 200)
        after = json.loads(self.request('/api/state')[1])
        self.assertEqual(after.pop('events'), [])
        before.pop('events')
        self.assertEqual(after, before)
        session.log("清空后的新日志")
        events = json.loads(self.request('/api/state')[1])['events']
        self.assertEqual([e['message'] for e in events], ["清空后的新日志"])

    def test_local_boundary_and_static_paths(self):
        self.assertEqual(self.request('/api/state',headers={'Host':'attacker.example'})[0],403)
        self.assertEqual(self.request('/api/heartbeat',{},headers={'Origin':'https://attacker.example'})[0],403)
        self.assertEqual(self.request('/api/heartbeat',{},headers={'X-Studio-Request':''})[0],403)
        self.assertEqual(self.request('/%2e%2e/core.py')[0],404)
        self.assertEqual(self.request('/model/description/urdf/sixaxis.urdf')[0],200)
        self.assertEqual(self.request('/model/description/../config/zekeep_hardware.yaml')[0],404)

    def test_program_errors_include_block_location(self):
        cases = [dict(id="variable-block", type="show", value=dict(type="variable", name="未赋值")),
                 dict(id="call-block", type="call", name="缺失函数"),
                 dict(id="python-block", type="python", name="结果", code="result = ["),
                 dict(id="python-runtime", type="python", name="结果", code="result = 1 / 0")]
        for item in cases:
            with self.subTest(item=item):
                status, data = self.request('/api/check', dict(program=[item], points={}))
                self.assertEqual(status, 400)
                body = json.loads(data)
                self.assertEqual(body['block_id'], item['id'])
                self.assertTrue(body['error'])

    def test_project_persistence_and_request_validation(self):
        body={'name':'项目 ../ A','workspace':{'blocks':{'languageVersion':0,'blocks':[]}},'points':{'位置':READY}}
        self.assertEqual(self.request('/api/projects/save',body)[0],200)
        status,data=self.request('/api/projects/load',{'name':body['name']})
        self.assertEqual(status,200)
        self.assertEqual(json.loads(data)['points'],body['points'])
        self.assertEqual(len(list(Path(self.temp.name).glob('*.json'))),1)
        self.assertEqual(self.request('/api/enable',{'enabled':'yes'})[0],400)
        self.assertEqual(self.request('/api/connect',{'mode':'unknown','channel':''})[0],400)
        self.assertEqual(self.request('/api/ik',{'position_m':[0,0,0],'rpy_rad':[], 'seed':READY})[0],400)


if __name__=='__main__':
    unittest.main()
