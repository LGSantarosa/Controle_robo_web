#!/usr/bin/env python3
"""
Launch Nav2 completo (AMCL + planner + controller + bt_navigator +
waypoint_follower + behavior_server + velocity_smoother + costmaps),
consumindo um mapa estático previamente gerado com slam_toolbox.

Uso:
    ros2 launch robot_nav nav2.launch.py map:=$HOME/Workspace/Controle_robo_web/maps/meu_mapa.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_nav')
    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    # BT custom: recovery reordenado p/ BackUp ANTES de Spin (robô encurralado
    # entre paredes "volta de onde veio" em vez de girar sem espaço). Ver
    # behavior_trees/navigate_w_backup_first_recovery.xml. Caminho via share dir
    # (não-hardcoded). Override do default_nav_to_pose_bt_xml no bt_navigator.
    bt_xml = os.path.join(pkg, 'behavior_trees', 'navigate_w_backup_first_recovery.xml')

    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Caminho para o arquivo .yaml do mapa gerado pelo slam_toolbox'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='true no modo sim (usa /clock do Gazebo)',
    )
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Caminho do YAML de params Nav2 (use nav2_params_pi.yaml na Pi)',
    )
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        # Reflexo de segurança — ativa por último (depende da saída do smoother).
        'collision_monitor',
    ]

    sim_time_param = {'use_sim_time': use_sim_time}

    # AUDITORIA_2026-05-27 M3: nós internos do Nav2 só em 'log'. Com 'screen' o
    # terminal do launch.sh vira fluxo contínuo de [controller_server] [INFO] a
    # cada tick. Os logs continuam em ~/.ros/log/<ts>/<node>-N-*.log (e em
    # controle_web/logs/nav2.log via redirect do launch.sh). O lifecycle_manager
    # fica em 'screen' — é o único que sinaliza visualmente "Nav2 ativou".
    nav_output = 'log'

    nodes = [
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output=nav_output,
            parameters=[params_file, sim_time_param, {'yaml_filename': map_yaml}],
        ),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            output=nav_output, parameters=[params_file, sim_time_param],
        ),
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', output=nav_output,
            parameters=[params_file, sim_time_param],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', output=nav_output,
            parameters=[params_file, sim_time_param],
        ),
        Node(
            package='nav2_behaviors', executable='behavior_server',
            name='behavior_server', output=nav_output,
            parameters=[params_file, sim_time_param],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', output=nav_output,
            parameters=[params_file, sim_time_param,
                        {'default_nav_to_pose_bt_xml': bt_xml}],
        ),
        Node(
            package='nav2_waypoint_follower', executable='waypoint_follower',
            name='waypoint_follower', output=nav_output,
            parameters=[params_file, sim_time_param],
        ),
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', output=nav_output,
            parameters=[params_file, sim_time_param],
            # Saída do smoother vai pra nav_vel_raw, que entra no collision_monitor
            # (reflexo de segurança). O collision_monitor é quem publica nav_vel
            # (entrada de menor prioridade do twist_mux em robot.launch.py E em
            # sim.launch.py) — assim o PS4 (joy_vel) e o web (web_vel) podem
            # assumir por cima da navegação.
            remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'nav_vel_raw')],
        ),
        # Collision Monitor: lê /scan cru e freia nav_vel_raw -> nav_vel ANTES do
        # twist_mux. Topicos in/out definidos no YAML (cmd_vel_in/out_topic).
        Node(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', output=nav_output,
            parameters=[params_file, sim_time_param],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
                'bond_timeout': 4.0,
            }],
        ),
    ]

    return LaunchDescription([map_arg, use_sim_time_arg, params_arg, *nodes])
