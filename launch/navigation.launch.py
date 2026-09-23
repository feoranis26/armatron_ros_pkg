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
from nav2_common.launch import RewrittenYaml

from armatron.map_state import load_state, profile_dir


def generate_launch_description():
    holonomic = LaunchConfiguration('holonomic')
    state = load_state()
    active_profile = state['active_profile']
    if not active_profile:
        raise RuntimeError(
            'No active ARMATRON map profile. Run: armatron-map select <profile>')
    selected_profile = profile_dir(active_profile)
    map_file = selected_profile / 'current' / 'map'
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
            'controller_server.ros__parameters.FollowPath.min_vel_y': PythonExpression(
                ["'-0.075' if '", holonomic, "'.lower() == 'true' else '0.0'"]
            ),
            'controller_server.ros__parameters.FollowPath.max_vel_y': PythonExpression(
                ["'0.075' if '", holonomic, "'.lower() == 'true' else '0.0'"]
            ),
            'velocity_smoother.ros__parameters.max_velocity': PythonExpression(
                ["'[2.0, 0.5, 3.0]' if '", holonomic,
                 "'.lower() == 'true' else '[2.0, 0.0, 3.0]'"]
            ),
            'velocity_smoother.ros__parameters.min_velocity': PythonExpression(
                ["'[-2.0, -0.5, -3.0]' if '", holonomic,
                 "'.lower() == 'true' else '[-2.0, 0.0, -3.0]'"]
            ),
            'slam_toolbox.ros__parameters.mode': state['mode'],
            'slam_toolbox.ros__parameters.map_file_name': (
                str(map_file) if map_file.with_suffix('.posegraph').exists() else ''
            ),
            'slam_toolbox.ros__parameters.map_start_pose': str(map_start_pose),
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': PathJoinSubstitution([
                FindPackageShare('armatron'), 'behavior_trees', 'navigate_to_pose_no_backup.xml'
            ]),
        },
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('holonomic', default_value='false'),
        DeclareLaunchArgument('slam', default_value='True'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('armatron'), 'maps', 'ai_room.map.yaml'
            ]),
        ),
        Node(
            package='armatron',
            executable='pose_persistence',
            name='map_pose_persistence',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'
            ])),
            launch_arguments={
                'params_file': params,
                'map': LaunchConfiguration('map'),
                'slam': LaunchConfiguration('slam'),
                'use_sim_time': 'false',
            }.items(),
        ),
    ])
