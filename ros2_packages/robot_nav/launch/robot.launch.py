#!/usr/bin/env python3
"""
Robot base launch.

Sobe:
  1. robot_state_publisher (URDF/TF)
  2. mega_bridge          (USB ↔ Arduino MEGA ↔ 2 hoverboards + sensores)
  3. odom_publisher       (4 RPMs → /odom + TF odom→base_link)
  4. cmd_vel_to_wheels    (/cmd_vel_filtered → /wheel_vel_setpoints)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('robot_nav')
    urdf_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius', default_value='0.085',
        description='Raio das rodas em metros'
    )
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base', default_value='0.50',
        description='Bitola (distância entre centros das rodas L-R) em metros'
    )
    linear_scale_arg = DeclareLaunchArgument(
        'linear_scale', default_value='400.0',
        description='Unidades do hoverboard por m/s'
    )
    angular_scale_arg = DeclareLaunchArgument(
        'angular_scale', default_value='150.0',
        description='Unidades do hoverboard por rad/s'
    )
    mega_port_arg = DeclareLaunchArgument(
        'mega_port', default_value='/dev/mega',
        description='Porta serial USB da Arduino MEGA'
    )
    mega_baud_arg = DeclareLaunchArgument(
        'mega_baud', default_value='230400',
        description='Baud rate da USB MEGA <-> PC'
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    mega_bridge = Node(
        package='robot_nav',
        executable='mega_bridge',
        name='mega_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('mega_port'),
            'baud': LaunchConfiguration('mega_baud'),
        }],
    )

    odom_publisher = Node(
        package='robot_nav',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        parameters=[{
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_base': LaunchConfiguration('wheel_base'),
        }],
    )

    cmd_vel_to_wheels = Node(
        package='robot_nav',
        executable='cmd_vel_to_wheels',
        name='cmd_vel_to_wheels',
        output='screen',
        parameters=[{
            'linear_scale': LaunchConfiguration('linear_scale'),
            'angular_scale': LaunchConfiguration('angular_scale'),
            'cmd_vel_topic': 'cmd_vel',
        }],
    )

    return LaunchDescription([
        wheel_radius_arg,
        wheel_base_arg,
        linear_scale_arg,
        angular_scale_arg,
        mega_port_arg,
        mega_baud_arg,
        robot_state_publisher,
        mega_bridge,
        odom_publisher,
        cmd_vel_to_wheels,
    ])
