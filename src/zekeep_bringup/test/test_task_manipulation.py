"""Offline task, geometry and execution checks; no nodes, camera or motors."""
import json
import math
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from rclpy.action import GoalResponse
from zekeep_msgs.action import WebManipulate
from zekeep_shadow.stacking import block_geometry, stack_poses, support_collision_box, validate_stack
from zekeep_teach.web_session import DetectionStore, WebSession
from zekeep_teach.web_tasks import WebTasks
from zekeep_llm.planner import Plan, Step, validate_arguments
from zekeep_llm.robot import RobotTools
from zekeep_llm.web_agent import AgentSession

from test_web_visual_execution import make_runner


def block(color, x=.25, y=-.10, yaw=0, size=.030, identity='a'*64):
    return dict(id=('b' if color == 'red' else 'c')*32, backend='yolo-graspnet',
        class_name=color+' block', color=color+' block', calibration_identity=identity,
        position=[x, y, size/2], position_reference='measured_object_center',
        geometry=dict(size_m=[size]*3, yaw_rad=yaw, top_z_m=size))


def config():
    return dict(safety={'table_z_m': 0}, robot=dict(workspace={
        'x': [-.7, .7], 'y': [-.7, .7], 'z': [.005, .6]}),
        place=dict(enabled=True, mode='fixed_joint', named_joint_targets_rad={'blue_place': [.1]*6}))


class StackingGeometryTest(unittest.TestCase):
    def test_perception_measures_height_and_footprint_instead_of_guessing(self):
        from zekeep_shadow.perception_pipeline import PerceptionPipeline
        from utils.cube_perception import assess_cube_geometry
        points = np.array([[x, y, .03] for x in np.linspace(.23, .26, 5)
                           for y in np.linspace(-.015, .015, 5)])
        pipeline = PerceptionPipeline.__new__(PerceptionPipeline)
        pipeline.cfg = {'safety': {'table_z_m': 0}}
        pipeline.route = SimpleNamespace(assess_cube_geometry=assess_cube_geometry,
            masked_depth_points_base=lambda *args, **kwargs: points,
            graspnet_utils=SimpleNamespace(build_target_sample_mask=lambda *args, **kwargs: None))
        measured = pipeline.measure_object(None, np.zeros((5, 5)), None, None)
        np.testing.assert_allclose(measured['position'], [.245, 0, .015], atol=1e-7)
        np.testing.assert_allclose(measured['geometry']['size_m'], [.03]*3, atol=1e-7)
        self.assertAlmostEqual(measured['geometry']['top_z_m'], .03)
        self.assertTrue(math.isfinite(measured['geometry']['yaw_rad']))

    def test_measured_offset_yaw_and_height_are_preserved(self):
        cfg = config()
        source, support = block('red', yaw=.3), block('blue', x=.35, y=.10, yaw=-.4)
        _, source_pose, _ = block_geometry(source, cfg)
        _, support_pose, top = block_geometry(support, cfg)
        grasp = source_pose.copy()
        grasp[:3, 3] += [.007, -.004, .020]
        above, release = stack_poses(source, support, grasp, cfg)
        placed = release @ np.linalg.inv(grasp) @ source_pose
        expected = support_pose.copy()
        expected[2, 3] = top + .010 + .015
        np.testing.assert_allclose(placed, expected, atol=1e-12)
        np.testing.assert_allclose(above[:3, 3]-release[:3, 3], [0, 0, .080])
        np.testing.assert_allclose(above[:3, :3], release[:3, :3])

    def test_rejects_missing_geometry_non_cubes_oversized_and_ambiguous_targets(self):
        source, support = block('red'), block('blue', x=.35, y=.1)
        invalid = [dict(source, geometry={}), dict(source, class_name='pen'),
                   block('red', size=.07), dict(source, position=[math.nan, 0, .015])]
        for item in invalid:
            with self.subTest(item=item), self.assertRaises(ValueError):
                validate_stack(item, support, config())
        for item in (block('red', x=.35), block('blue'), block('blue', x=.35, size=.022)):
            with self.subTest(item=item), self.assertRaises(ValueError):
                validate_stack(source, item, config())

    def test_clearance_workspace_and_collision_envelope(self):
        source, support = block('red'), block('blue', x=.35, y=.1, yaw=math.pi/4)
        for gap in (-1, 0, .005, .03, math.nan):
            cfg = config()
            cfg['stacking'] = {'release_clearance_m': gap}
            with self.subTest(gap=gap), self.assertRaises(ValueError):
                stack_poses(source, support, np.eye(4), cfg)
        cfg = config()
        cfg['robot']['workspace']['z'][1] = .050
        with self.assertRaisesRegex(ValueError, 'workspace'):
            stack_poses(source, support, np.eye(4), cfg)
        center, size = support_collision_box(support, config(), added_height=.04)
        np.testing.assert_allclose(size, [.033*math.sqrt(2)+.01]*2 + [.073])
        np.testing.assert_allclose(center, [.35, .1, .0365])

    def test_collision_box_covers_verification_tolerance_and_larger_payload(self):
        support = block('blue', x=.35, y=.1)
        center, extent = map(np.asarray, support_collision_box(support, config(), added_height=.042,
            payload_size_m=[.032]*3))
        # A 32 mm payload on a 30 mm support can rotate and shift within verification bounds.
        for yaw in np.linspace(0, math.pi, 30):
            rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
            for x in (-.0175, .0175):
                for y in (-.0175, .0175):
                    point = rotation @ [x, y] + center[:2] + [.010, -.010]
                    self.assertTrue(np.all(np.abs(point-center[:2]) <= extent[:2]/2 + 1e-12))


class ManipulationTaskTest(unittest.TestCase):
    def test_new_snapshot_checks_geometry_and_duplicate_class(self):
        original = block('blue')
        node = SimpleNamespace(verify_target=Mock(), detection_lock=threading.RLock(),
            detections=SimpleNamespace(latest=[original]))
        self.assertIs(WebTasks.verify_snapshot(node, original, lambda: None), original)
        for changes in ({'position': [.26, -.1, .015]}, {'geometry': {}},
                        {'geometry': dict(original['geometry'], yaw_rad=.2)},
                        {'geometry': dict(original['geometry'], size_m=[.04]*3)}):
            node.detections.latest = [dict(original, **changes)]
            with self.subTest(changes=changes), self.assertRaises(RuntimeError):
                WebTasks.verify_snapshot(node, original, lambda: None)
        node.detections.latest = [original, dict(original, id='d'*32)]
        with self.assertRaisesRegex(RuntimeError, 'ambiguous'):
            WebTasks.verify_snapshot(node, original, lambda: None)

    def test_target_listing_is_read_only_and_explicit_names_never_fall_back(self):
        from utils.fixed_placement import resolve_joint_placement_target
        cfg = config()
        cfg['place']['class_targets'] = {'red block': 'missing'}
        name, target = resolve_joint_placement_target(cfg['place'], 'red block', target_name='blue_place')
        self.assertEqual(name, 'blue_place')
        np.testing.assert_allclose(target, [.1]*6)
        with self.assertRaisesRegex(ValueError, 'unknown placement target'):
            resolve_joint_placement_target(cfg['place'], target_name='missing')
        node = SimpleNamespace(perception_config=lambda: (None, cfg), backend='yolo-graspnet')
        response = SimpleNamespace(success=False, status_json='', message='')
        WebTasks.teach_command(node, SimpleNamespace(command='placement_targets'), response)
        self.assertTrue(response.success)
        self.assertEqual(json.loads(response.status_json)['targets'], ['blue_place'])
        # A read-only query never reaches session reservation or robot preparation.
        RobotTools.prepare(SimpleNamespace(_autostart=True), 'list_placement_targets')
        session = AgentSession(SimpleNamespace(plan=lambda _: Plan('列出放置点', (Step('list_placement_targets'),))),
            SimpleNamespace(execute=lambda *args: response.status_json))
        plan = session.plan('有哪些放置点')
        self.assertFalse(plan['requires_confirmation'])
        self.assertTrue(session.execute(plan['plan_id'], True)['ok'])

    def node(self, *, held=False, authorized=True):
        runner, plan = make_runner()
        runner.cfg['robot'].update(config()['robot'])
        runner.cfg['place'].update(config()['place'])
        source = block('red', identity=runner.calibration_identity)
        support = block('blue', x=.35, y=.1, identity=runner.calibration_identity)
        for pose in plan['end_poses']:
            pose[0][3], pose[1][3] = source['position'][:2]
        runner.client._wait_future.return_value.status_json = json.dumps(plan)
        runner.held, runner.lift_completed = held, held
        runner.held_detection = source
        runner.grasp_end_pose = np.asarray(plan['end_poses'][1])
        runner.placement_class_name = 'red block'
        leases = []
        session = WebSession(lambda: None, '/tmp/zekeep-test-no-session-files', authorized=authorized,
            acquire=lambda owner: leases.append(('acquire', owner)))
        store = DetectionStore()
        store.latest = [source, support]
        store.targets = {item['id']: (store.clock(), item) for item in store.latest}
        node = SimpleNamespace(runner=runner, backend='yolo-graspnet', session=session,
            grasp_lock=threading.Lock(), detection_lock=threading.RLock(), detections=store,
            status=lambda: {'grasp_readiness': 'READY'}, perception_config=lambda: (runner.root, runner.cfg),
            get_logger=lambda: Mock(), verify_snapshot=Mock(side_effect=lambda target, check: target),
            lease=lambda owner, acquire: leases.append(('acquire' if acquire else 'release', owner)))
        return node, source, support, leases

    def run_goal(self, node, goal):
        handle = SimpleNamespace(request=goal, is_cancel_requested=False, publish_feedback=Mock(),
            succeed=Mock(), abort=Mock(), canceled=Mock())
        with patch('utils.calibration_identity.validate_calibration_identity'), \
                patch('utils.calibration_identity.calibration_identity', return_value={'test': True}):
            result = WebTasks.grasp(node, handle, manipulation=True)
        return result, handle

    def test_named_sort_and_stacking_keep_one_lease_until_completion(self):
        for stacking in (False, True):
            with self.subTest(stacking=stacking):
                node, source, support, leases = self.node()
                goal = WebManipulate.Goal(detection_id=source['id'],
                    support_detection_id=support['id'] if stacking else '',
                    target_name='' if stacking else 'blue_place')
                self.assertEqual(WebTasks.accept_manipulation(node, goal), GoalResponse.ACCEPT)
                result, handle = self.run_goal(node, goal)
                self.assertTrue(result.success, result.message)
                self.assertFalse(node.runner.held)
                self.assertEqual(leases, [('acquire', 'grasp'), ('release', 'grasp')])
                self.assertFalse(node.grasp_lock.locked())
                handle.succeed.assert_called_once()
                if stacking:
                    self.assertEqual(node.verify_snapshot.call_count, 4)
                    node.runner.client.apply_world_collision_box.assert_called()
                    self.assertIn('stacked red block on blue block', result.message)
                else:
                    np.testing.assert_allclose(node.runner.client.plan_joint_goal.call_args_list[1].args[1], [.1]*6)
                    self.assertIn('blue_place', result.message)

    def test_separate_named_place_uses_held_object_without_a_detection(self):
        node, _, _, leases = self.node(held=True)
        node.status = lambda: {'grasp_readiness': 'WAITING_TARGET'}
        node.detections.latest = []
        goal = WebManipulate.Goal(target_name='blue_place')
        self.assertEqual(WebTasks.accept_manipulation(node, goal), GoalResponse.ACCEPT)
        result, _ = self.run_goal(node, goal)
        self.assertTrue(result.success, result.message)
        node.runner.client.grasp.assert_not_called()
        self.assertEqual(len(leases), 2)

    def test_rejects_bad_destination_stale_detection_and_missing_authorization_before_motion(self):
        for reason in ('unknown', 'both', 'neither', 'stale', 'authorization', 'held'):
            node, source, support, leases = self.node(held=reason=='held', authorized=reason!='authorization')
            goal = WebManipulate.Goal(detection_id=source['id'], target_name='blue_place')
            if reason == 'unknown': goal.target_name = 'missing'
            if reason == 'both': goal.support_detection_id = support['id']
            if reason == 'neither': goal.target_name = ''
            if reason == 'stale': node.detections.targets[source['id']] = (node.detections.clock()-2, source)
            with self.subTest(reason=reason):
                self.assertEqual(WebTasks.accept_manipulation(node, goal), GoalResponse.REJECT)
                self.assertFalse(node.grasp_lock.locked())
                self.assertEqual(leases, [])
                node.runner.client.open_gripper.assert_not_called()

    def test_moved_target_empty_grasp_and_cancellation_never_release(self):
        for reason in ('moved', 'empty', 'cancel'):
            node, source, support, _ = self.node()
            goal = WebManipulate.Goal(detection_id=source['id'], support_detection_id=support['id'])
            self.assertEqual(WebTasks.accept_manipulation(node, goal), GoalResponse.ACCEPT)
            if reason == 'moved': node.verify_snapshot.side_effect = RuntimeError('target changed')
            if reason == 'empty': node.runner.client.grasp.return_value = False
            if reason == 'cancel': node.session.cancelled.set()
            with self.subTest(reason=reason):
                result, handle = self.run_goal(node, goal)
                self.assertFalse(result.success)
                handle.abort.assert_called_once()
                # At most the initial pregrasp opening; never a release after grasp.
                self.assertLessEqual(node.runner.client.open_gripper.call_count, 1)
                if reason != 'empty': node.runner.client.grasp.assert_not_called()

    def test_failed_stack_planning_keeps_payload_attached(self):
        node, source, support, _ = self.node(held=True)
        goal = WebManipulate.Goal(support_detection_id=support['id'])
        self.assertEqual(WebTasks.accept_manipulation(node, goal), GoalResponse.ACCEPT)
        node.runner.client.solve_end_pose_ik.return_value = (None, -31)
        result, _ = self.run_goal(node, goal)
        self.assertFalse(result.success)
        self.assertTrue(node.runner.held)
        node.runner.client.open_gripper.assert_not_called()
        node.runner.client.remove_attached_collision_box.assert_not_called()

    def test_failed_scene_update_does_not_release_the_payload(self):
        node, _, support, _ = self.node(held=True)
        goal = WebManipulate.Goal(support_detection_id=support['id'])
        self.assertEqual(WebTasks.accept_manipulation(node, goal), GoalResponse.ACCEPT)
        node.runner.client.apply_world_collision_box.side_effect = [None, None, RuntimeError('scene update failed')]
        result, _ = self.run_goal(node, goal)
        self.assertFalse(result.success)
        self.assertIn('scene update failed', result.message)
        self.assertTrue(node.runner.held)
        node.runner.client.open_gripper.assert_not_called()
        node.runner.client.remove_attached_collision_box.assert_not_called()

    def test_model_tools_are_confirmed_and_parameters_are_bounded(self):
        args = {'object_name': 'red block', 'target_name': 'blue_place'}
        for tool, arguments in (('pick_and_place', args), ('stack_blocks', {'source_color': 'red', 'target_color': 'blue'})):
            planner = SimpleNamespace(plan=lambda _: Plan('执行任务', (Step('return_ready'), Step(tool, arguments))))
            calls = []
            robot = SimpleNamespace(execute=lambda name, values: calls.append(name) or 'done')
            session = AgentSession(planner, robot, motion_authorized=True, lease=lambda acquire: calls.append(acquire))
            plan = session.plan('任务')
            self.assertTrue(plan['requires_confirmation'])
            session.execute(plan['plan_id'], True)
            self.assertEqual(calls, [True, 'return_ready', False, tool])
            with self.assertRaises(ValueError):
                session.rpc({'jsonrpc':'2.0','id':1,'method':'tools/call','params':{'name':tool,'arguments':arguments}})
        for arguments in ({'source_color':'red','target_color':'red'}, {'source_color':'red','target_color':'orange'}):
            with self.assertRaises(ValueError): validate_arguments('stack_blocks', arguments)
        for name in ('../file', '', 'x'*65, 'blue_place; stop'):
            with self.assertRaises(ValueError): validate_arguments('pick_and_place', dict(args, target_name=name))

    def test_adapter_selects_unique_sources_and_supports_without_sending_coordinates(self):
        targets = [block('red'), block('blue', x=.35)]
        robot = SimpleNamespace(detect_blocks=lambda: {'detections': targets}, _manipulate='action', run_web_task=Mock(return_value='done'))
        self.assertEqual(RobotTools.manipulate(robot, 'stack_blocks', {'source_color':'red','target_color':'blue'}), 'done')
        goal = robot.run_web_task.call_args.args[1]
        self.assertEqual((goal.detection_id, goal.support_detection_id, goal.target_name), ('b'*32, 'c'*32, ''))
        targets.append(dict(targets[0], id='d'*32))
        with self.assertRaisesRegex(RuntimeError, '不唯一'):
            RobotTools.manipulate(robot, 'pick_and_place', {'object_name':'red block','target_name':'blue_place'})
        self.assertEqual(robot.run_web_task.call_count, 1)


if __name__ == '__main__':
    unittest.main()
