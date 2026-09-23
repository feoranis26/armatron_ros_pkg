from setuptools import setup
import os
from glob import glob

package_name = 'armatron'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config', 'nav2'), glob(os.path.join('config', 'nav2', '*yaml'))),
        (os.path.join('share', package_name, 'config', 'odometry'), glob(os.path.join('config', 'odometry', '*yaml'))),
        (os.path.join('share', package_name, 'behavior_trees'), glob(os.path.join('behavior_trees', '*xml'))),
        (os.path.join('share', package_name, 'dependencies'),
         [path for path in glob(os.path.join('dependencies', '*')) if os.path.isfile(path)]),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*'))),
        (os.path.join('lib', package_name), ['scripts/run-armatron']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='feoranis',
    maintainer_email='feoranis@todo.todo',
    description='ARMATRON ROS bridge, sensor bringup, and navigation tools.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive = armatron.drive_bridge:main',
            'scan_filter = armatron.scan_filter:main',
            'heading_guard = armatron.heading_guard:main',
            'lidar_demand = armatron.lidar_demand:main',
            'lidar_confidence = armatron.lidar_confidence:main',
            'gyro_pub = armatron.gyro_pub:main',
            'armatron-map = armatron.map_manager:main',
            'motion_consistency = armatron.motion_consistency:main',
            'pose_persistence = armatron.pose_persistence:main',
        ],
    },
)
