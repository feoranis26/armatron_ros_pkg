"""Launch Nav2 with the ARMATRON baseline configuration.

Pass ``holonomic:=true`` only when the mecanum base is healthy.  The default
keeps lateral motion disabled while retaining a single authoritative Nav2 file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from armatron.launch_parameters import RewrittenYaml

from armatron.map_state import load_state, profile_dir


def generate_launch_description():
    holonomic = LaunchConfiguration('holonomic')
    state = load_state()
    active_profile = state['active_profile']
    if not active_profile:
        raise RuntimeError(
            'No active ARMATRON map profile. Run: ros2 run armatron armatron-map select <profile>')
    selected_profile = profile_dir(active_profile)
    map_file = selected_profile / 'current' / 'map'
    amcl = state['mode'] == 'amcl'
    localization = state['mode'] == 'localization'
    grid_file = selected_profile / 'current' / 'grid' / 'map.yaml'
    if amcl and not grid_file.is_file():
        raise RuntimeError('AMCL needs a grid export. While mapping, run: '
                           'ros2 run armatron armatron-map save --with-grid')
    last_pose = state.get('last_pose')
    map_start_pose = ([last_pose['x'], last_pose['y'], last_pose['yaw']]
                      if last_pose and map_file.with_suffix('.posegraph').exists()
                      else [0.0, 0.0, 0.0])
    if state['mode'] == 'localization' and not map_file.with_suffix('.posegraph').exists():
        raise RuntimeError(
            f'Profile {active_profile!r} has no saved posegraph. Select mapping mode to create one.')

    params = RewrittenYaml(
        source_file=PathJoinSubstitution([
            FindPackageShare('armatron'), 'config', 'nav2', 'navigation.yaml'
        ]),
        param_rewrites={
            'amcl.ros__parameters.set_initial_pose': str(bool(last_pose)),
            'amcl.ros__parameters.initial_pose.x': str(float(last_pose['x']) if last_pose else 0.0),
            'amcl.ros__parameters.initial_pose.y': str(float(last_pose['y']) if last_pose else 0.0),
            'amcl.ros__parameters.initial_pose.yaw': str(float(last_pose['yaw']) if last_pose else 0.0),
            'controller_server.ros__parameters.FollowPath.min_vel_y': PythonExpression(
                ["'-0.075' if '", holonomic, "'.lower() == 'true' else '0.0'"]
            ),
            'controller_server.ros__parameters.FollowPath.max_vel_y': PythonExpression(
                ["'0.075' if '", holonomic, "'.lower() == 'true' else '0.0'"]
            ),
            # Humble RewrittenYaml addresses array elements, not whole arrays.
            'velocity_smoother.ros__parameters.max_velocity.1': PythonExpression(
                ["'0.5' if '", holonomic, "'.lower() == 'true' else '0.0'"]
            ),
            'velocity_smoother.ros__parameters.min_velocity.1': PythonExpression(
                ["'-0.5' if '", holonomic, "'.lower() == 'true' else '0.0'"]
            ),
            'slam_toolbox.ros__parameters.mode': state['mode'],
            'slam_toolbox.ros__parameters.map_file_name': (
                str(map_file) if map_file.with_suffix('.posegraph').exists() else ''
            ),
            **{f'slam_toolbox.ros__parameters.map_start_pose.{i}': str(float(value))
               for i, value in enumerate(map_start_pose)},
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': PathJoinSubstitution([
                FindPackageShare('armatron'), 'behavior_trees', 'navigate_to_pose_no_backup.xml'
            ]),
            'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': PathJoinSubstitution([
                FindPackageShare('armatron'), 'behavior_trees', 'navigate_through_poses_no_backup.xml'
            ]),
        },
        convert_types=True,
    )

    # Humble bringup's slam=True always includes online_sync_launch.py.
    # The localization executable implements a different scan-processing path;
    # a YAML mode string cannot turn the mapping executable into that node.
    localization_nodes = []
    if localization:
        localization_nodes.append(Node(
            package='slam_toolbox', executable='localization_slam_toolbox_node',
            name='slam_toolbox', output='screen',
            parameters=[params, {'enable_interactive_mode': False}],
        ))
    nav_arguments = {
        'params_file': params, 'use_sim_time': 'false',
    }
    if localization:
        # Launch navigation alone: neither another SLAM node nor AMCL/map_server.
        nav_arguments.update(use_composition='False', autostart='True')
    else:
        nav_arguments.update(map=LaunchConfiguration('map'),
                             slam='False' if amcl else LaunchConfiguration('slam'))

    return LaunchDescription([
        DeclareLaunchArgument('holonomic', default_value='false'),
        DeclareLaunchArgument('slam', default_value='False' if amcl else 'True'),
        DeclareLaunchArgument(
            'map',
            default_value=str(grid_file) if amcl else PathJoinSubstitution([
                FindPackageShare('armatron'), 'maps', 'ai_room.map.yaml'
            ]),
        ),
        Node(
            package='armatron',
            executable='pose_persistence',
            name='map_pose_persistence',
            parameters=[{'pose_topic': '/amcl_pose' if amcl else '/pose'}],
        ),
        *localization_nodes,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch',
                'navigation_launch.py' if localization else 'bringup_launch.py'
            ])),
            launch_arguments=nav_arguments.items(),
        ),
    ])
