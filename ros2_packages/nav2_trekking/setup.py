from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav2_trekking'

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
    description='Trekking via Nav2 — clone independente do robot_nav (evolui separado)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_wheels = nav2_trekking.cmd_vel_to_wheels:main',
            'mega_bridge = nav2_trekking.mega_bridge:main',
            'pose_estimator = nav2_trekking.pose_estimator:main',
            'cone_detector = nav2_trekking.cone_detector:main',
            'trekking_runner = nav2_trekking.trekking_runner:main',
            'sim_trekking_pose = nav2_trekking.sim_trekking_pose:main',
            'unstuck_supervisor = nav2_trekking.unstuck_supervisor:main',
            'scan_sanitizer = nav2_trekking.scan_sanitizer:main',
            'door_crossing = nav2_trekking.door_crossing:main',
            'path_follower = nav2_trekking.path_follower:main',
            'freeze_capture = nav2_trekking.freeze_capture:main',
            'sim_actuator_model = nav2_trekking.sim_actuator_model:main',
        ],
    },
)
