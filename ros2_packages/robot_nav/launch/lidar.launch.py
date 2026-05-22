#!/usr/bin/env python3
"""
LiDAR launch file — LDROBOT LD06 (LiDAR principal do robô).

O LD06 usa o driver ldlidar_stl_ros2 com product_name 'LDLiDAR_LD06',
baudrate 230400. Orientação validada em RViz: 0° = +X (frente),
ângulos crescem anti-horário → +Y (esquerda). Por isso laser_scan_dir=True.

Parâmetros:
  - lidar_port: porta serial (default /dev/lidar, criado pelo setup_udev.sh)
  - lidar_product: 'LDLiDAR_LD06' (use 'LDLiDAR_LD19' para o antigo FHL-LD20)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/lidar',
        description='Porta serial do LiDAR (default: /dev/lidar criado pelo setup_udev.sh)'
    )
    product_arg = DeclareLaunchArgument(
        'lidar_product', default_value='LDLiDAR_LD06',
        description='Nome do produto LiDAR: LDLiDAR_LD06 (padrão) ou LDLiDAR_LD19'
    )

    # ---- LiDAR driver node ----
    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ld06',
        output='screen',
        parameters=[
            {'product_name': LaunchConfiguration('lidar_product')},
            {'topic_name': 'scan'},
            {'frame_id': 'base_laser'},
            {'port_name': LaunchConfiguration('lidar_port')},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0},
        ]
    )

    return LaunchDescription([
        port_arg,
        product_arg,
        lidar_node,
    ])
