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
from .relocalization_checks import planar_pose, nearby, stationary


def run(args):
    # Lazy ROS imports keep the map CLI usable for offline profile management.
    if not all(math.isfinite(v) and v > 0 for v in
               (args.timeout, args.distance_tolerance, args.angle_tolerance)):
        raise ValueError("Timeout and handoff tolerances must be finite and positive")
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
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
    for name in ('map.posegraph', 'map.data', 'grid/map.yaml'):
        if not (revision/name).is_file():
            raise RuntimeError('Selected profile needs a posegraph and grid from save --with-grid')
    original_revision = [(revision/name).stat().st_mtime_ns for name in ('map.posegraph', 'map.data', 'grid/map.yaml')]
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
              'still_since': None, 'accepted': None, 'searching': False}

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
        if not stationary(msg) or now-values['odom_at'] > .5:
            values['still_since'] = None
        if stationary(msg) and values['still_since'] is None:
            values['still_since'] = now
        values['odom'], values['odom_at'] = msg, now

    def heading(msg):
        values['heading'], values['heading_at'] = msg.data, time.monotonic()

    def healthy():
        now = time.monotonic()
        return (values['heading'] and now-values['heading_at'] < .3 and
                now-values['odom_at'] < .5 and now-values['scan_at'] < 1.)

    def still():
        return (healthy() and values['still_since'] is not None and
                time.monotonic()-values['still_since'] >= 1.)

    def accept(request, response):
        if not values['searching']:
            response.success, response.message = False, 'Not searching'
        elif not still() or time.monotonic()-values['amcl_at'] > 2.:
            response.success, response.message = False, 'Need fresh AMCL pose and one second stationary with healthy odometry'
        else:
            values['accepted'] = copy.deepcopy(values['amcl'])
            response.success, response.message = True, 'Pose accepted by operator; verifying SLAM handoff'
        return response

    subscriptions = [
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
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            if sum(name in ('amcl', 'slam_toolbox') for name in node.get_node_names()) > 1:
                raise RuntimeError('Multiple AMCL/slam_toolbox localization nodes detected')
            for child in children:
                if child.poll() is not None:
                    raise RuntimeError('Localization process exited; inspect '+str(directory))
            if require_still and not still():
                raise RuntimeError('Motion or odometry interruption during handoff; pose not saved')
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(reason)
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
        stream = (directory/(executable+'.log')).open('w')
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
        if not still():
            raise RuntimeError('Robot must remain stationary for handoff')
        lifecycle('/amcl', 4, 2)
        stop(amcl_process)
        stop(map_process)
        wait(lambda: not any(name in ('amcl', 'map_server') for name in node.get_node_names()),
             10., 'AMCL/map server still visible after shutdown')
        tf_sources.clear()
        buffer.clear()  # Discard the previous owner's future-dated TF cache.
        if not still():
            raise RuntimeError('Motion during AMCL shutdown; retry')
        # Start with upstream complete localization defaults, then our profile.
        slam_config = yaml.safe_load((Path(get_package_share_directory('slam_toolbox'))/
                                      'config/mapper_params_localization.yaml').read_text())
        slam_config['slam_toolbox']['ros__parameters'].update(
            use_sim_time=False, map_file_name=str(directory/'revision/map'),
            map_start_pose=list(target), map_start_at_dock=False,
            scan_topic='/scan', odom_frame='odom', map_frame='map', base_frame='base_link',
            minimum_travel_distance=0.0, minimum_travel_heading=0.0)
        slam_params = directory/'slam.yaml'
        slam_params.write_text(yaml.safe_dump(slam_config))
        spawn('slam_toolbox', 'localization_slam_toolbox_node', slam_params)
        wait(lambda: initial.get_subscription_count() > 0, 15., 'SLAM initialpose subscriber missing', True)
        values['slam'] = None
        accepted.header.stamp = node.get_clock().now().to_msg()
        initial.publish(accepted)
        seed_stamp = accepted.header.stamp.sec+accepted.header.stamp.nanosec*1e-9
        verified = []
        def takeover():
            msg = values['slam']
            if msg is None:
                return False
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if stamp <= seed_stamp or (verified and stamp <= verified[-1]):
                return False
            if not nearby(planar_pose(msg), target, args.distance_tolerance, args.angle_tolerance):
                raise RuntimeError('SLAM localized away from accepted AMCL pose')
            try:
                transform = buffer.lookup_transform('map', 'base_link', Time.from_msg(msg.header.stamp))
            except TransformException:
                return False
            t, q = transform.transform.translation, transform.transform.rotation
            actual = (t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            if not nearby(actual, target, args.distance_tolerance, args.angle_tolerance):
                raise RuntimeError('SLAM TF disagrees with accepted AMCL pose')
            verified.append(stamp)
            return len(verified) >= 3 and verified[-1]-verified[0] >= 1.
        wait(takeover, 30., 'SLAM takeover not verified; recovered pose not saved', True)
        current = load_state(args.root)
        if (current['active_profile'] != state['active_profile'] or current['mode'] != state['mode'] or
                original_revision != [(revision/name).stat().st_mtime_ns for name in ('map.posegraph', 'map.data', 'grid/map.yaml')]):
            raise RuntimeError('Selected profile, mode or map revision changed during relocalization')
        x, y, yaw = planar_pose(values['slam'])
        current.update(mode='localization', last_pose={'x': x, 'y': y, 'yaw': yaw})
        save_state(current, args.root)
        status('VERIFIED: recovered pose saved. Ending bootstrap; keep robot stationary, then start armatron-navigation.service.')
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
