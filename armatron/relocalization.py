"""Stage-one AMCL bootstrap: operator acceptance, verified SLAM handoff.

Owns only its child processes. Normal navigation must already be stopped.
"""
import copy
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import uuid

import yaml

from .map_state import load_state, profile_dir, save_state
from .map_manager import export_grid
from .relocalization_checks import planar_pose, nearby, stationary


def run(args):
    # Lazy ROS imports keep the map CLI usable for offline profile management.
    if not all(math.isfinite(v) and v > 0 for v in
               (args.timeout, args.distance_tolerance, args.angle_tolerance)):
        raise ValueError("Timeout and handoff tolerances must be finite and positive")
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry, OccupancyGrid
    from tf2_msgs.msg import TFMessage
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool
    from std_srvs.srv import Empty, Trigger
    from lifecycle_msgs.srv import ChangeState, GetState
    from tf2_ros import Buffer, TransformListener, TransformException
    from ament_index_python.packages import get_package_share_directory

    if os.name != 'posix':
        raise RuntimeError('Run relocalize on the Linux robot with ROS sourced')
    state = load_state(args.root)
    revision = profile_dir(state['active_profile'], args.root) / 'current'
    for name in ('map.posegraph', 'map.data'):
        if not (revision/name).is_file():
            raise RuntimeError('Selected profile needs saved map.posegraph and map.data files')
    def revision_identity():
        return [(revision/name).stat().st_mtime_ns if (revision/name).exists() else None
                for name in ('map.posegraph', 'map.data', 'grid/map.yaml')]
    original_revision = revision_identity()
    args.root.mkdir(parents=True, exist_ok=True)
    # Prevent two operators from racing through the discovery preflight.
    import fcntl
    lock = (args.root/'relocalization.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError('Another relocalization session is running')
    directory = args.root/'relocalization_runs'/uuid.uuid4().hex
    directory.mkdir(parents=True)
    children, streams = [], []
    rclpy.init()
    node = Node('armatron_relocalize')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    values = {'odom': None, 'odom_at': -math.inf, 'scan_at': -math.inf,
              'heading': False, 'heading_at': -math.inf, 'amcl': None,
              'amcl_at': -math.inf, 'amcl_min_stamp': -math.inf, 'slam': None, 'slam_at': -math.inf,
              'still_since': None, 'motion_after_accept': None, 'verification_interrupted': False, 'accepted': None, 'searching': False, 'map_ready': False}

    tf_sources = {}

    def transforms(msg):
        if any(t.header.frame_id == 'map' and t.child_frame_id == 'odom' for t in msg.transforms):
            # Humble rclpy supplies only the message, not publisher metadata.
            # This detects an existing TF owner during preflight. Runtime
            # duplicate checks below use the known localization node names.
            tf_sources['map_odom'] = time.monotonic()

    def status(text):
        print(text, flush=True)
        with (directory/'status.log').open('a') as stream:
            stream.write(text+'\n')

    def pose(which, msg):
        try:
            planar_pose(msg)
        except ValueError:
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age = node.get_clock().now().nanoseconds*1e-9-stamp
        if which == 'amcl' and stamp <= values['amcl_min_stamp']:
            return
        if not -0.1 <= age <= 2.:
            return
        values[which], values[which+'_at'] = msg, time.monotonic()

    def odom(msg):
        now = time.monotonic()
        if values['accepted'] is not None and not stationary(msg):
            v = msg.twist.twist
            values['motion_after_accept'] = (
                f'odom reports vx={v.linear.x:.3f}, vy={v.linear.y:.3f} m/s, '
                f'wz={v.angular.z:.3f} rad/s after acceptance')
        if not stationary(msg) or now-values['odom_at'] > .5:
            values['still_since'] = None
        if stationary(msg) and values['still_since'] is None:
            values['still_since'] = now
        values['odom'], values['odom_at'] = msg, now

    def heading(msg):
        values['heading'], values['heading_at'] = msg.data, time.monotonic()

    def health_problem():
        now = time.monotonic()
        problems = []
        for label, key, limit in (('heading-ready heartbeat', 'heading_at', .3),
                                   ('filtered odometry', 'odom_at', .5),
                                   ('scan', 'scan_at', 1.)):
            age = now-values[key]
            if age >= limit:
                problems.append(f'{label} age={age:.2f}s (limit {limit:.2f}s)')
        if not values['heading']:
            problems.append('heading guard reports not ready (independent of gyro/status)')
        return '; '.join(problems)

    def healthy():
        return not health_problem()

    def handoff_problem():
        return health_problem() or 'waiting for one second of stationary odometry'

    def still():
        return (healthy() and values['still_since'] is not None and
                time.monotonic()-values['still_since'] >= 1.)

    def checked_state():
        current = load_state(args.root)
        if (current['active_profile'] != state['active_profile'] or current['mode'] != state['mode'] or
                original_revision != revision_identity()):
            raise RuntimeError('Selected profile, mode or map revision changed during relocalization')
        return current

    def accept(request, response):
        if not values['searching']:
            response.success, response.message = False, 'Not searching'
        elif not still() or time.monotonic()-values['amcl_at'] > 2.:
            response.success, response.message = False, 'Need fresh AMCL pose and one second stationary with healthy odometry'
        else:
            accepted = copy.deepcopy(values['amcl'])
            try:
                current = checked_state()
                x, y, yaw = planar_pose(accepted)
                current.update(mode='localization', last_pose={'x': x, 'y': y, 'yaw': yaw})
                save_state(current, args.root)
            except (OSError, ValueError, RuntimeError) as error:
                response.success, response.message = False, 'Could not save accepted pose: '+str(error)
                return response
            state['mode'] = 'localization'
            values['accepted'] = accepted
            values['searching'] = False
            status('ACCEPTED_SAVED: operator AMCL pose saved as localization restart hint; SLAM handoff not yet verified')
            response.success, response.message = True, 'Accepted pose saved; verifying SLAM handoff'

        return response

    subscriptions = [
        node.create_subscription(OccupancyGrid, '/map',
            lambda msg: values.update(map_ready=bool(msg.info.width and msg.info.height and msg.data)),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(TFMessage, "/tf", transforms, 100),
        node.create_subscription(Odometry, '/odometry/filtered', odom, 10),
        node.create_subscription(Bool, '/odometry/heading_ready', heading, 1),
        node.create_subscription(LaserScan, '/scan', lambda m: values.update(scan_at=time.monotonic()), qos_profile_sensor_data),
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: pose('amcl', m), 10),
        node.create_subscription(PoseWithCovarianceStamped, '/pose', lambda m: pose('slam', m), 10),
    ]
    service = node.create_service(Trigger, '/armatron/relocalize/accept', accept)
    initial = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)

    def wait(predicate, timeout, reason, require_still=False):
        deadline = time.monotonic()+timeout
        last_wait_log = -math.inf
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            if sum(name in ('amcl', 'slam_toolbox') for name in node.get_node_names()) > 1:
                raise RuntimeError('Multiple AMCL/slam_toolbox localization nodes detected')
            for child in children:
                if child.poll() is not None:
                    raise RuntimeError('Localization process exited; inspect '+str(directory))
            if values['motion_after_accept']:
                raise RuntimeError(values['motion_after_accept']+'; accepted AMCL pose retained')
            if require_still and not still():
                # Scheduling gaps are not evidence of motion. Keep spinning to
                # collect all topics, but never count verification across a gap.
                values['verification_interrupted'] = True
                now = time.monotonic()
                detail = handoff_problem()
                if now-last_wait_log >= 1.:
                    status('HANDOFF_WAIT: '+detail)
                    last_wait_log = now
                if now >= deadline:
                    raise RuntimeError('Handoff data did not recover: '+detail)
                continue
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(reason() if callable(reason) else reason)
        raise RuntimeError('ROS shutdown')

    def call(kind, name, request):
        client = node.create_client(kind, name)
        try:
            wait(client.service_is_ready, 15., 'Service unavailable: '+name)
            future = client.call_async(request)
            wait(future.done, 15., 'Service timed out: '+name)
            return future.result()
        finally:
            node.destroy_client(client)

    def lifecycle(name, transition, expected):
        req = ChangeState.Request()
        req.transition.id = transition
        if not call(ChangeState, name+'/change_state', req).success:
            raise RuntimeError('Lifecycle transition failed: '+name)
        result = call(GetState, name+'/get_state', GetState.Request())
        if result.current_state.id != expected:
            raise RuntimeError('Unexpected lifecycle state: '+name)

    def spawn(package, executable, params):
        log_name = 'grid_renderer' if params.name == 'render.yaml' else executable
        stream = (directory/(log_name+'.log')).open('w')
        streams.append(stream)
        child = subprocess.Popen(['ros2', 'run', package, executable, '--ros-args',
                                  '--params-file', str(params)], stdout=stream,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        children.append(child)
        return child

    def stop(child):
        # Wait for the entire process group, not only ros2's launcher wrapper.
        try:
            os.killpg(child.pid, signal.SIGINT)
        except ProcessLookupError:
            child.poll()
            children.remove(child)
            return
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline:
            child.poll()
            try:
                os.killpg(child.pid, 0)
            except ProcessLookupError:
                children.remove(child)
                return
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.05)
            else:
                time.sleep(.05)
        os.killpg(child.pid, signal.SIGKILL)
        child.wait(timeout=5.)
        children.remove(child)
        # No new TF owner may start until the former group has disappeared.
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline:
            try:
                os.killpg(child.pid, 0)
            except ProcessLookupError:
                return
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.05)
            else:
                time.sleep(.05)
        raise RuntimeError('Could not confirm process group stopped')

    try:
        status('Preflight: stop armatron-navigation.service and all manual navigation launches. Logs: '+str(directory))
        # Give discovery time to populate; this is not a lifecycle timing assumption.
        start = time.monotonic()
        wait(lambda: time.monotonic()-start >= 2., 3., 'Discovery timeout')
        conflicts = set(node.get_node_names()) & {
            'amcl', 'map_server', 'slam_toolbox', 'bt_navigator', 'controller_server',
            'lifecycle_manager_navigation', 'map_pose_persistence'}
        if tf_sources:
            raise RuntimeError("An existing map -> odom publisher is still active; stop it first")
        if conflicts:
            raise RuntimeError('Stop existing navigation/localization nodes first: '+', '.join(sorted(conflicts)))
        wait(healthy, 10., 'Fresh scan, heading and filtered odometry required')
        shutil.copytree(revision, directory/'revision')
        if not (directory/'revision/grid/map.yaml').is_file():
            if (revision/'grid').exists():
                raise RuntimeError('Incomplete grid directory already exists: '+str(revision/'grid'))
            values['map_ready'] = False
            status('EXPORTING_GRID: rendering occupancy grid from the saved posegraph; live scans and TF publication disabled')
            render_config = yaml.safe_load((Path(get_package_share_directory('slam_toolbox'))/
                                           'config/mapper_params_localization.yaml').read_text())
            render_config['slam_toolbox']['ros__parameters'].update(
                use_sim_time=False, map_file_name=str(directory/'revision/map'),
                map_start_pose=[0.0, 0.0, 0.0], map_start_at_dock=False,
                scan_topic='/armatron/relocalize/disabled_scan',
                transform_publish_period=0.0, map_update_interval=0.2)
            render_params = directory/'render.yaml'
            render_params.write_text(yaml.safe_dump(render_config))
            renderer = spawn('slam_toolbox', 'localization_slam_toolbox_node', render_params)
            wait(lambda: values['map_ready'], 30., 'Saved posegraph did not produce an occupancy grid')
            export_grid(directory/'revision/grid', 20.)
            stop(renderer)
            wait(lambda: 'slam_toolbox' not in node.get_node_names(), 10., 'Grid renderer did not stop')
            checked_state()
            # Grid-only migration preserves posegraph bytes and the previous revision.
            destination = revision/'grid'
            if destination.exists():
                raise RuntimeError('Incomplete grid directory already exists: '+str(destination))
            staging_grid = revision/('.grid-'+uuid.uuid4().hex)
            shutil.copytree(directory/'revision/grid', staging_grid)
            staging_grid.replace(destination)
            original_revision = revision_identity()
            status('GRID_SAVED: existing profile is now ready for AMCL recovery')
            wait(healthy, 10., 'Waiting for fresh odometry after grid export')
        config = yaml.safe_load((Path(get_package_share_directory('armatron'))/
                                 'config/nav2/navigation.yaml').read_text())
        config['amcl']['ros__parameters'].update(set_initial_pose=True, initial_pose={'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}, tf_broadcast=True)
        grid = directory/'revision/grid/map.yaml'
        metadata = yaml.safe_load(grid.read_text())
        image = grid.parent/Path(metadata['image']).name
        if not image.is_file():
            raise RuntimeError('Profile grid image is missing')
        metadata['image'] = str(image.resolve())
        grid.write_text(yaml.safe_dump(metadata))
        config['map_server'] = {'ros__parameters': {'yaml_filename': str(grid), 'use_sim_time': False}}
        params = directory/'amcl.yaml'
        params.write_text(yaml.safe_dump(config))
        map_process = spawn('nav2_map_server', 'map_server', params)
        amcl_process = spawn('nav2_amcl', 'amcl', params)
        for name in ('/map_server', '/amcl'):
            lifecycle(name, 1, 2)
            lifecycle(name, 3, 3)
        def discover_global():
            matches = [name for name, types in node.get_service_names_and_types()
                       if name.endswith('/reinitialize_global_localization') and 'std_srvs/srv/Empty' in types]
            if len(matches) > 1:
                raise RuntimeError('Multiple AMCL global-localization services')
            return matches
        wait(discover_global, 10., 'AMCL global-localization service not found')
        # A first pose proves AMCL has actually received the map and a scan.
        # Then discard this arbitrary startup seed with a uniform global reset.
        wait(lambda: values['amcl'] is not None, 15., 'AMCL has not processed its map/scan')
        global_service = discover_global()[0]
        call(Empty, global_service, Empty.Request())
        values['amcl_min_stamp'] = node.get_clock().now().nanoseconds*1e-9
        values['amcl'] = None
        values['amcl_at'] = -math.inf
        values['searching'] = True
        status('SEARCHING: inspect /amcl_pose and /particle_cloud in RViz. No autonomous motion.\n'
               'When aligned, stop moving and accept in another terminal:\n'
               "ros2 service call /armatron/relocalize/accept std_srvs/srv/Trigger '{}' ")
        last_update = [-math.inf]
        def search_complete():
            if values['accepted'] is not None:
                return True
            if time.monotonic()-last_update[0] >= 1.:
                # AMCL normally skips stationary scans. Request updates so the
                # operator can accept a fresh pose after releasing teleop.
                call(Empty, global_service.rsplit('/', 1)[0]+'/request_nomotion_update', Empty.Request())
                last_update[0] = time.monotonic()
            return False
        wait(search_complete, args.timeout,
             'AMCL search timed out; use manual initialization or retry')
        values['searching'] = False
        accepted = values['accepted']
        target = planar_pose(accepted)
        status('HANDOFF: accepted map pose '+str(target))
        wait(still, 5., handoff_problem, True)
        lifecycle('/amcl', 4, 2)
        stop(amcl_process)
        stop(map_process)
        wait(lambda: not any(name in ('amcl', 'map_server') for name in node.get_node_names()),
             10., 'AMCL/map server still visible after shutdown')
        tf_sources.clear()
        buffer.clear()  # Discard the previous owner's future-dated TF cache.
        wait(still, 5., handoff_problem, True)
        # Start with upstream complete localization defaults, then our profile.
        slam_config = yaml.safe_load((Path(get_package_share_directory('slam_toolbox'))/
                                      'config/mapper_params_localization.yaml').read_text())
        slam_config['slam_toolbox']['ros__parameters'].update(
            use_sim_time=False, map_file_name=str(directory/'revision/map'),
            map_start_pose=list(target), map_start_at_dock=False,
            scan_topic='/scan', odom_frame='odom', map_frame='map', base_frame='base_link',
            minimum_travel_distance=0.0, minimum_travel_heading=0.0,
            restamp_tf=True, transform_publish_period=0.05, transform_timeout=0.5,
            scan_buffer_size=3, map_update_interval=5.0, enable_interactive_mode=False,
            mode='localization')
        slam_params = directory/'slam.yaml'
        slam_params.write_text(yaml.safe_dump(slam_config))
        spawn('slam_toolbox', 'localization_slam_toolbox_node', slam_params)
        wait(lambda: initial.get_subscription_count() > 0, 15., 'SLAM initialpose subscriber missing', True)
        values['slam'] = None
        accepted.header.stamp = node.get_clock().now().to_msg()
        initial.publish(accepted)
        seed_stamp = accepted.header.stamp.sec+accepted.header.stamp.nanosec*1e-9
        verified = []
        diagnostic = ['No post-seed SLAM pose received']
        last_diagnostic_at = [-math.inf]
        def mismatch(label, actual):
            distance = math.hypot(actual[0]-target[0], actual[1]-target[1])
            angle = abs(math.atan2(math.sin(actual[2]-target[2]), math.cos(actual[2]-target[2])))
            detail = (f'{label}: accepted={tuple(round(v, 3) for v in target)}, '
                      f'observed={tuple(round(v, 3) for v in actual)}, '
                      f'error={distance:.3f} m / {angle:.3f} rad '
                      f'(limits {args.distance_tolerance:.3f} m / {args.angle_tolerance:.3f} rad)')
            if detail != diagnostic[0] and time.monotonic()-last_diagnostic_at[0] >= 1.:
                status('VERIFYING: '+detail)
                last_diagnostic_at[0] = time.monotonic()
            diagnostic[0] = detail
            verified.clear()
            return False

        def takeover():
            if values['verification_interrupted']:
                verified.clear()
                values['verification_interrupted'] = False
            msg = values['slam']
            if msg is None:
                return False
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if stamp <= seed_stamp or (verified and stamp <= verified[-1]):
                return False
            if not nearby(planar_pose(msg), target, args.distance_tolerance, args.angle_tolerance):
                return mismatch('SLAM pose disagrees', planar_pose(msg))
            try:
                transform = buffer.lookup_transform('map', 'base_link', Time.from_msg(msg.header.stamp))
            except TransformException as error:
                diagnostic[0] = 'Waiting for scan-time TF: '+str(error)
                verified.clear()
                return False
            t, q = transform.transform.translation, transform.transform.rotation
            actual = (t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            if not nearby(actual, target, args.distance_tolerance, args.angle_tolerance):
                return mismatch('SLAM scan-time TF disagrees', actual)
            verified.append(stamp)
            return len(verified) >= 3 and verified[-1]-verified[0] >= 1.
        wait(takeover, 30., lambda: 'SLAM takeover not verified; accepted AMCL pose retained. '+diagnostic[0], True)
        current = checked_state()
        x, y, yaw = planar_pose(values['slam'])
        current.update(mode='localization', last_pose={'x': x, 'y': y, 'yaw': yaw})
        save_state(current, args.root)
        status('VERIFIED: recovered pose saved. Ending bootstrap; keep robot stationary, then start armatron-navigation.service.')
    except (RuntimeError, OSError) as error:
        if values['accepted'] is not None:
            status('HANDOFF_FAILED: '+str(error)+
                   '. Operator-accepted pose was saved; navigation remains stopped.')
        raise
    finally:
        cleanup_errors = []
        for child in list(reversed(children)):
            try:
                stop(child)
            except (OSError, subprocess.TimeoutExpired, RuntimeError) as error:
                cleanup_errors.append(str(error))
        for stream in streams:
            stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.close()
        if cleanup_errors:
            raise RuntimeError('Bootstrap cleanup incomplete; do not start navigation: '+ '; '.join(cleanup_errors))
