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
        #Node(
        #    package="armatron",
        #    executable="armatron_drive",
        #    name="drive",
        #),

        #Node(
        #    package="joy_teleop",
        #    executable="joy_teleop",
        #    name="joy_teleop"
        #),

        Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="rplidar_pub",
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'inverted': False,
                'angle_compensate': True,
            }],
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
            executable="scan_trimmer",
            name="scan_trimmer"
        ),
    ])