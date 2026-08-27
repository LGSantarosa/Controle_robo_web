#!/usr/bin/env python3
"""
SLAM launch: slam_toolbox em modo online async.

Usa /scan (LiDAR) + /odom + TF (base_link, base_laser) para construir
um mapa 2D em tempo real. Depois de mapear, salve pelo botão 'Salvar mapa'
da UI web (chama MapBridge.save_map → map_saver_cli) — gera
maps/<nome>.yaml + .pgm. Alternativa por terminal:

    ros2 run nav2_map_server map_saver_cli -f ~/Workspace/Controle_robo_web/maps/meu_mapa

E use o arquivo .yaml gerado com nav2.launch.py.

No modo sim (Gazebo), passe use_sim_time:=true para que o slam_toolbox
consuma o /clock simulado em vez do wall clock.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='true no modo sim (usa /clock do Gazebo)',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'resolution': 0.05,
            'max_laser_range': 12.0,
            'minimum_time_interval': 0.1,   # era 0.2 — processa ~10 Hz (taxa do LD06) no giro
            'transform_publish_period': 0.05,
            'map_update_interval': 1.0,
            'use_lifecycle_manager': True,
            'transform_timeout': 0.5,
            # Afinação contra "parede fantasma" na curva: processa scan em
            # incrementos pequenos pra o matcher (Ceres) convergir mesmo com a
            # semente de yaw de roda ruim — em vez de esperar 0.5 rad (~28°) e
            # confiar no odom no meio do giro. Ver spec 2026-06-01-odometria-fundida.
            'use_scan_matching': True,
            'minimum_travel_distance': 0.15,
            'minimum_travel_heading': 0.10,   # era 0.12 — mais scans no giro (passo menor → erro de seed menor)
            'scan_buffer_size': 20,
            # Robô SEM IMU: o seed de yaw vem só da roda e é torto no giro do
            # skid-steer. Deixa o scan matcher do slam recuperar o match mesmo com
            # seed ruim, em vez de se perder. Ver spec 2026-06-02-slam-recupera-prior-giro.
            'use_response_expansion': True,         # match fraco → expande a janela de busca até achar (CPU só quando precisa)
            'coarse_search_angle_offset': 0.6,      # ±~34° (era ±~20° default) — cobre seed de yaw torto no spin
            'minimum_angle_penalty': 0.7,           # era 0.9 default — penaliza menos correção angular grande vs seed ruim
            'correlation_search_space_dimension': 0.6,  # era 0.5 default — folga de busca linear
        }]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 4.0,
        }]
    )

    return LaunchDescription([use_sim_time_arg, slam, lifecycle_manager])
