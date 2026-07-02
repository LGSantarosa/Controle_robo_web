from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@robot.com',
    description='Navigation and LiDAR integration for hoverboard robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_wheels = robot_nav.cmd_vel_to_wheels:main',
            'mega_bridge = robot_nav.mega_bridge:main',
            'pose_estimator = robot_nav.pose_estimator:main',
            'cone_detector = robot_nav.cone_detector:main',
            'trekking_runner = robot_nav.trekking_runner:main',
            'unstuck_supervisor = robot_nav.unstuck_supervisor:main',
            'scan_sanitizer = robot_nav.scan_sanitizer:main',
            'door_crossing = robot_nav.door_crossing:main',
            'path_follower = robot_nav.path_follower:main',
            'freeze_capture = robot_nav.freeze_capture:main',
            'sim_actuator_model = robot_nav.sim_actuator_model:main',
            'motion_guard = robot_nav.motion_guard:main',
        ],
    },
)
