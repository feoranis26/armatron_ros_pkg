from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                ])
            ]),
            launch_arguments={
                'params_file': PathJoinSubstitution([
                    FindPackageShare('armatron'),
                    'params',
                    'localization_params.yaml'
                ]),
                'map': PathJoinSubstitution([
                    FindPackageShare('armatron'),
                    'maps',
                    'room.map.yaml'
                ])
            }.items()
        ),

        Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="rplidar_pub",
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'inverted': False,
                'angle_compensate': True,
            }],
        ),

        Node(
            package="armatron",
            executable="scan_trimmer",
            name="scan_trimmer"
        ),

        # Node(
        #     package="armatron",
        #     executable="armatron_drive",
        #     name="drive",
        #     remappings= [("cmd_vel", "cmd_vel_smoothed")]
        # ),

        # Node(
        #     package="joy_teleop",
        #     executable="joy_teleop",
        #     name="joy_teleop",
        #     remappings= [("cmd_vel", "cmd_vel_smoothed")]
        # ),

        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('armatron_description'),
                    'launch',
                    'rsp.launch.py'
                ])
            ])
        ),
    ])
