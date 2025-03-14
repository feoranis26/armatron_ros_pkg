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
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*yaml'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='feoranis',
    maintainer_email='feoranis@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive = armatron.node:main',
            'drive_2wd = armatron.node_udp_2wd:main',
            'drive_serial = armatron.node_i2c:main',
            'drive_tcp = armatron.node_i2c:main',
            'drive_udp_i2c = armatron.node_i2c:main',
            'drive_udp = armatron.node_i2c:main',
            'scan_trimmer = armatron.scan_trimmer:main',
            'goto_pose = armatron.goto_pose:main',
            'gyro_pub = armatron.gyro_pub:main'
        ],
    },
)
