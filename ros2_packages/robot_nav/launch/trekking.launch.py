#!/usr/bin/env python3
"""
Launcher do modo TREKKING.

Sobe os 2 nós específicos do controle ponto-a-ponto da competição:
  1. cone_detector     — clusteriza /scan + /trekking/pose → /trekking/cones
  2. trekking_runner   — máquina de estado IDLE/RECORD/PLAY com PID

Pré-requisito: robot.launch.py já está rodando — ele sobe o `pose_estimator`
(que publica /trekking/pose + /odom + TF) além de mega_bridge + URDF +
cmd_vel_to_wheels, e o LiDAR está publicando /scan. O trekking consome
/trekking/pose direto (mais preciso pelo flow), sem depender do TF.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    v_max_arg = DeclareLaunchArgument(
        'v_max', default_value='0.35',
        description='Velocidade linear máxima do PID (m/s)'
    )
    lidar_offset_x_arg = DeclareLaunchArgument(
        'lidar_offset_x', default_value='0.10',
        description='Deslocamento x do base_laser em relação a base_link (m)'
    )
    enable_cone_pose_fix_arg = DeclareLaunchArgument(
        'enable_cone_pose_fix', default_value='true',
        description='Liga a correção persistente de pose por cone-âncora (A/B em campo)'
    )

    cone_detector = Node(
        package='robot_nav',
        executable='cone_detector',
        name='cone_detector',
        output='screen',
        parameters=[{
            'lidar_offset_x': LaunchConfiguration('lidar_offset_x'),
        }],
    )

    trekking_runner = Node(
        package='robot_nav',
        executable='trekking_runner',
        name='trekking_runner',
        output='screen',
        parameters=[{
            'v_max': LaunchConfiguration('v_max'),
            'enable_cone_pose_fix': LaunchConfiguration('enable_cone_pose_fix'),
        }],
        # Saída do PID vai pra nav_vel (entrada de menor prioridade do twist_mux
        # em robot.launch.py) — assim o PS4 pode assumir por cima do autônomo.
        remappings=[('cmd_vel', 'nav_vel')],
    )

    return LaunchDescription([
        v_max_arg,
        lidar_offset_x_arg,
        enable_cone_pose_fix_arg,
        cone_detector,
        trekking_runner,
    ])
