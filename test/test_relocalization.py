"""Tests for geometric acceptance and the operator-assisted CLI contract."""
import math
from types import SimpleNamespace as NS
import unittest

from armatron.relocalization_checks import nearby, planar_pose, stationary
from armatron.map_manager import parser


def pose(x=0., y=0., yaw=0., frame='map'):
    return NS(header=NS(frame_id=frame), pose=NS(pose=NS(
        position=NS(x=x, y=y), orientation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)))))


class RelocalizationTest(unittest.TestCase):
    def test_map_pose_and_wrapped_heading(self):
        self.assertEqual(planar_pose(pose(2., 3.)), (2., 3., 0.))
        self.assertTrue(nearby((0., 0., math.pi-.01), (0., 0., -math.pi+.01), .1, .03))
        self.assertFalse(nearby((.4, 0., 0.), (0., 0., 0.), .3, .2))
        self.assertFalse(nearby((0., 0., .4), (0., 0., 0.), .3, .2))

    def test_invalid_poses_cannot_be_accepted(self):
        for msg in (pose(frame='odom'), pose(x=float('nan'))):
            with self.assertRaises(ValueError):
                planar_pose(msg)
        msg = pose()
        msg.pose.pose.orientation.w = 0.
        with self.assertRaises(ValueError):
            planar_pose(msg)

    def test_stationary_requires_valid_low_translation_and_rotation(self):
        msg = NS(twist=NS(twist=NS(linear=NS(x=0., y=0.), angular=NS(z=0.))))
        self.assertTrue(stationary(msg))
        msg.twist.twist.linear.y = .1
        self.assertFalse(stationary(msg))
        msg.twist.twist.linear.y = 0.
        msg.twist.twist.angular.z = .1
        self.assertFalse(stationary(msg))
        msg.twist.twist.angular.z = float('nan')
        self.assertFalse(stationary(msg))

    def test_cli_is_offline_importable_and_bounds_are_explicit(self):
        args = parser().parse_args(['relocalize', '--timeout', '60', '--distance-tolerance', '.2'])
        self.assertEqual(args.timeout, 60.)
        self.assertEqual(args.distance_tolerance, .2)
        self.assertEqual(args.func.__name__, 'command_relocalize')

class OrchestrationTest(unittest.TestCase):
    def exercise(self, bad_slam=False, competing=False, existing_tf=False, transient_tf=False, bad_tf=False, lost_heading=False, save_failure=False, missing_grid=False, export_failure=False, transient_gap=False, motion=False, missing_scan=False):
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        import sys
        import signal
        import yaml
        from armatron import relocalization as recovery
        from armatron.map_state import save_state, load_state
        clock = [100.]
        callbacks, services, events, children = {}, {}, [], []
        states = {}
        class Message:
            def __init__(self):
                self.transition = NS(id=0)
        class Service:
            Request = Message
        class Future:
            def __init__(self, result): self.value = result
            def done(self): return True
            def result(self): return self.value
        class Client:
            def __init__(self, name): self.name = name
            def service_is_ready(self): return True
            def call_async(self, request):
                events.append(self.name)
                prefix = self.name.rsplit('/', 1)[0]
                if self.name.endswith('change_state'):
                    states[prefix] = {1: 2, 3: 3, 4: 2}[request.transition.id]
                return Future(NS(success=True, current_state=NS(id=states.get(prefix, 0))))
        def stamp():
            seconds = int(clock[0])
            return NS(sec=seconds, nanosec=int((clock[0]-seconds)*1e9))
        class Node:
            def __init__(self, name): pass
            def create_subscription(self, kind, topic, callback, qos):
                callbacks[topic] = callback
                return callback
            def create_service(self, kind, name, callback):
                services[name] = callback
                return callback
            def create_publisher(self, *args):
                return NS(get_subscription_count=lambda: 1,
                          publish=lambda msg: events.append('seed_slam'))
            def create_client(self, kind, name): return Client(name)
            def destroy_client(self, client): pass
            def destroy_node(self): pass
            def get_clock(self):
                return NS(now=lambda: NS(nanoseconds=int(clock[0]*1e9), to_msg=stamp))
            def get_node_names(self): return ['bt_navigator'] if competing else []
            def get_service_names_and_types(self):
                return [('/reinitialize_global_localization', ['std_srvs/srv/Empty'])]
        class Buffer:
            def clear(self): events.append('clear_tf')
            def lookup_transform(self, *args):
                slam_start = next(c.started for c in children if c.executable == 'localization_slam_toolbox_node')
                wrong = bad_tf or (transient_tf and clock[0]-slam_start < 1.5)
                return NS(transform=NS(translation=NS(x=4. if wrong else 0., y=0.), rotation=NS(x=0., y=0., z=0., w=1.)))
        class Child:
            def __init__(self, command, **kwargs):
                self.pid = len(children)+1000
                self.active = True
                self.started = clock[0]
                self.executable = command[3]
                if Path(command[-1]).name == 'render.yaml':
                    self.executable = 'grid_renderer'
                    params = yaml.safe_load(Path(command[-1]).read_text())['slam_toolbox']['ros__parameters']
                    assert params['transform_publish_period'] == 0.
                    assert params['scan_topic'] == '/armatron/relocalize/disabled_scan'
                children.append(self)
                events.append('start:'+self.executable)
            def poll(self): return None if self.active else 0
            def wait(self, **kwargs): self.active = False
        def killpg(pid, sig):
            child = next(c for c in children if c.pid == pid)
            if not child.active: raise ProcessLookupError()
            if sig:
                child.active = False
                events.append('stop:'+child.executable)
        def spin(node, **kwargs):
            clock[0] += .1
            # Match Humble's actual one-argument subscription dispatch.
            callbacks['/tf'](NS(transforms=[NS(
                header=NS(frame_id='map' if existing_tf else 'odom'),
                child_frame_id='odom' if existing_tf else 'base_link')]))
            slam = next((c for c in children if c.executable == 'localization_slam_toolbox_node'), None)
            gap = transient_gap and slam and .2 < clock[0]-slam.started < 1.2
            if gap:
                events.append('data_gap')
            else:
                callbacks['/odometry/heading_ready'](NS(data=not (lost_heading and 'accepted_saved' in events)))
                vx = .1 if motion and 'accepted_saved' in events else 0.
                callbacks['/odometry/filtered'](NS(twist=NS(twist=NS(linear=NS(x=vx, y=0.), angular=NS(z=0.)))))
                if not (missing_scan and 'accepted_saved' in events):
                    callbacks['/scan'](None)
            for child in children:
                if child.active and child.executable == 'grid_renderer':
                    callbacks['/map'](NS(info=NS(width=1, height=1), data=[0]))
                if child.active and child.executable in ('amcl', 'localization_slam_toolbox_node'):
                    is_slam = child.executable != 'amcl'
                    msg = pose(x=3. if bad_slam and is_slam else 0.)
                    msg.header.stamp = stamp()
                    callbacks['/pose' if is_slam else '/amcl_pose'](msg)
            response = services['/armatron/relocalize/accept'](None, NS())
            if response.success:
                # Acceptance must be durable before the service replies, even
                # before AMCL has been stopped or SLAM started.
                self.assertEqual(load_state(root)['last_pose']['x'], 0.)
                self.assertEqual(load_state(root)['mode'], 'localization')
                self.assertNotIn('start:localization_slam_toolbox_node', events)
                events.append('accepted_saved')
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            revision = root/'maps/test/current'
            revision.mkdir(parents=True)
            for name in ('map.posegraph', 'map.data'):
                (revision/name).write_bytes(b'test artifact')
            def grid_export(directory, timeout):
                events.append('export_grid')
                if export_failure:
                    raise RuntimeError('Grid export failed')
                directory.mkdir()
                (directory/'map.pgm').write_bytes(b'grid image')
                (directory/'map.yaml').write_text('image: map.pgm\n')
            if not missing_grid:
                grid_export(revision/'grid', 20.)
                events.clear()
            share = root/'share'
            (share/'config/nav2').mkdir(parents=True)
            (share/'config/nav2/navigation.yaml').write_text('amcl:\n  ros__parameters: {}\n')
            (share/'config/mapper_params_localization.yaml').write_text('slam_toolbox:\n  ros__parameters: {}\n')
            save_state({'active_profile': 'test', 'mode': 'mapping'}, root)
            modules = {
                'rclpy': NS(init=lambda: None, shutdown=lambda: None, ok=lambda: True, spin_once=spin),
                'rclpy.node': NS(Node=Node), 'rclpy.time': NS(Time=NS(from_msg=lambda m: m)),
                'rclpy.qos': NS(qos_profile_sensor_data=1, QoSProfile=lambda **kw: kw, DurabilityPolicy=NS(TRANSIENT_LOCAL=1)),
                'geometry_msgs.msg': NS(PoseWithCovarianceStamped=Message),
                'nav_msgs.msg': NS(Odometry=Message, OccupancyGrid=Message), 'sensor_msgs.msg': NS(LaserScan=Message),
                'tf2_msgs.msg': NS(TFMessage=Message), 'std_msgs.msg': NS(Bool=Message),
                'std_srvs.srv': NS(Empty=Service, Trigger=Service),
                'lifecycle_msgs.srv': NS(ChangeState=Service, GetState=Service),
                'tf2_ros': NS(Buffer=Buffer, TransformListener=lambda *a: None, TransformException=LookupError),
                'ament_index_python.packages': NS(get_package_share_directory=lambda p: str(share)),
                'fcntl': NS(flock=lambda *a: None, LOCK_EX=1, LOCK_NB=2),
            }
            args = NS(root=root, timeout=10., distance_tolerance=.3, angle_tolerance=.25)
            with patch.dict(sys.modules, modules), patch.object(recovery, 'os', NS(name='posix', killpg=killpg)), \
                 patch.object(recovery.time, 'monotonic', side_effect=lambda: clock[0]), \
                 patch.object(recovery.subprocess, 'Popen', side_effect=Child), \
                 patch.object(recovery, 'export_grid', side_effect=grid_export), \
                 patch.object(recovery, 'save_state', side_effect=OSError('read-only') if save_failure else save_state):
                if bad_slam or competing or existing_tf or bad_tf or lost_heading or save_failure or export_failure or motion or missing_scan:
                    reason = ('odom reports vx' if motion else 'scan age=' if missing_scan
                              else 'heading guard reports not ready' if lost_heading else '')
                    with self.assertRaisesRegex(RuntimeError, reason): recovery.run(args)
                    if competing or existing_tf or save_failure or export_failure:
                        self.assertEqual(load_state(root)['mode'], 'mapping')
                        self.assertIsNone(load_state(root)['last_pose'])
                    else:
                        self.assertEqual(load_state(root)['mode'], 'localization')
                        self.assertEqual(load_state(root)['last_pose']['x'], 0.)
                else:
                    recovery.run(args)
                    self.assertEqual(load_state(root)['mode'], 'localization')
                    self.assertEqual(load_state(root)['last_pose']['x'], 0.)
            self.assertTrue(all(not c.active for c in children))
            for name in ('map.posegraph', 'map.data'):
                self.assertEqual((revision/name).read_bytes(), b'test artifact')
            if missing_grid:
                self.assertEqual((revision/'grid/map.yaml').is_file(), not export_failure)
            return events

    def test_amcl_stops_before_slam_and_verified_pose_is_saved(self):
        events = self.exercise()
        self.assertLess(events.index('stop:amcl'), events.index('start:localization_slam_toolbox_node'))
        self.assertLess(events.index('clear_tf'), events.index('seed_slam'))

    def test_bad_takeover_retains_operator_accepted_pose(self):
        self.exercise(bad_slam=True)

    def test_existing_navigation_refuses_before_starting_children(self):
        self.assertFalse(any(e.startswith('start:') for e in self.exercise(competing=True)))

    def test_existing_map_tf_refuses_with_humble_single_argument_callback(self):
        events = self.exercise(existing_tf=True)
        self.assertFalse(any(e.startswith('start:') for e in events))

    def test_startup_tf_disagreement_can_settle_before_deadline(self):
        self.exercise(transient_tf=True)

    def test_persistent_tf_disagreement_retains_operator_accepted_pose(self):
        self.exercise(bad_tf=True)

    def test_heading_loss_after_acceptance_retains_pose(self):
        self.exercise(lost_heading=True)

    def test_save_failure_refuses_acceptance_and_does_not_start_slam(self):
        events = self.exercise(save_failure=True)
        self.assertNotIn('accepted_saved', events)
        self.assertNotIn('start:localization_slam_toolbox_node', events)

    def test_missing_grid_generated_before_amcl_without_changing_graph(self):
        events = self.exercise(missing_grid=True)
        self.assertLess(events.index('export_grid'), events.index('stop:grid_renderer'))
        self.assertLess(events.index('stop:grid_renderer'), events.index('start:amcl'))

    def test_grid_export_failure_preserves_profile_and_cleans_up(self):
        events = self.exercise(missing_grid=True, export_failure=True)
        self.assertNotIn('start:amcl', events)

    def test_brief_data_gap_during_slam_verification_recovers(self):
        events = self.exercise(transient_gap=True)
        self.assertIn('data_gap', events)

    def test_actual_motion_after_acceptance_still_fails(self):
        self.exercise(motion=True)

    def test_missing_scans_are_not_misreported_as_gyro_failure(self):
        self.exercise(missing_scan=True)
