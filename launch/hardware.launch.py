from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="rplidar_composition",
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'inverted': False,
                'angle_compensate': True,
            }],
            remappings=[
                ('/scan', '/scan_raw'),
            ],
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

        Node(
            package="armatron",
            executable="scan_filter",
            name="scan_filter"
        ),

        # Local sensor odometry must remain available even when no map profile
        # is selected and the navigation service intentionally refuses to run.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('armatron'),
                    'launch',
                    'odometry.launch.py'
                ])
            ])
        ),
    ])
