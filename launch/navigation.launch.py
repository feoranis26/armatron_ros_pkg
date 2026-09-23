"""Launch Nav2 with the ARMATRON baseline configuration.

Pass ``holonomic:=true`` only when the mecanum base is healthy.  The default
keeps lateral motion disabled while retaining a single authoritative Nav2 file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    holonomic = LaunchConfiguration('holonomic')
    params = RewrittenYaml(
        source_file=PathJoinSubstitution([
            FindPackageShare('armatron'), 'config', 'nav2', 'navigation.yaml'
        ]),
        param_rewrites={
            'controller_server.ros__parameters.FollowPath.min_vel_y': PythonExpression(
                ["'-0.075' if '", holonomic, "' == 'true' else '0.0'"]
            ),
            'controller_server.ros__parameters.FollowPath.max_vel_y': PythonExpression(
                ["'0.075' if '", holonomic, "' == 'true' else '0.0'"]
            ),
            'velocity_smoother.ros__parameters.max_velocity': PythonExpression(
                ["'[2.0, 0.5, 3.0]' if '", holonomic,
                 "' == 'true' else '[2.0, 0.0, 3.0]'"]
            ),
            'velocity_smoother.ros__parameters.min_velocity': PythonExpression(
                ["'[-2.0, -0.5, -3.0]' if '", holonomic,
                 "' == 'true' else '[-2.0, 0.0, -3.0]'"]
            ),
        },
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('holonomic', default_value='false'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('armatron'), 'maps', 'ai_room.map.yaml'
            ]),
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
