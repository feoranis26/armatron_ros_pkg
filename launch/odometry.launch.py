"""Independent RF2O and authoritative robot_localization local odometry."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    share = FindPackageShare("armatron")
    return LaunchDescription([
        Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            parameters=[PathJoinSubstitution([share, "config", "odometry", "rf2o.yaml"])],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=[PathJoinSubstitution([share, "config", "odometry", "ekf.yaml"])],
        ),
        Node(
            package="armatron",
            executable="motion_consistency",
            name="motion_consistency_monitor",
        ),
    ])
