#!/usr/bin/env python3
"""
Launcher do modo TREKKING.

Sobe os 3 nós que compõem o controle ponto-a-ponto da competição:
  1. pose_estimator    — funde IMU + flow + rodas em /trekking/pose
  2. cone_detector     — clusteriza /scan + /trekking/pose → /trekking/cones
  3. trekking_runner   — máquina de estado IDLE/RECORD/PLAY com PID

Pré-requisito: robot.launch.py já está rodando (mega_bridge + URDF +
cmd_vel_to_wheels), e o LiDAR está publicando /scan.

Observação importante: o `odom_publisher` do robot.launch.py continua
rodando e publicando o TF `odom→base_link` baseado só nas rodas. Isso
serve pro restante do sistema (rviz, etc.). O modo trekking ignora esse
TF e usa /trekking/pose direto — mais preciso pelo flow.
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
    flow_height_arg = DeclareLaunchArgument(
        'flow_height', default_value='0.12',
        description='Altura do PMW3901 ao chão (m)'
    )
    lidar_offset_x_arg = DeclareLaunchArgument(
        'lidar_offset_x', default_value='0.10',
        description='Deslocamento x do base_laser em relação a base_link (m)'
    )
    enable_cone_pose_fix_arg = DeclareLaunchArgument(
        'enable_cone_pose_fix', default_value='true',
        description='Liga a correção persistente de pose por cone-âncora (A/B em campo)'
    )

    pose_estimator = Node(
        package='robot_nav',
        executable='pose_estimator',
        name='pose_estimator',
        output='screen',
        parameters=[{
            'flow_height': LaunchConfiguration('flow_height'),
            # Mapeamento PMW3901 → body frame, calibrado no robô 2026-05-29
            # (dirigindo reto firme): frente = dy NEGATIVO do sensor, dx≈0.
            #   swap_xy=True   → frente entra pelo eixo dy (não dx)
            #   flow_x_sign=-1 → frente (dy<0) vira +vx (pra frente positivo)
            # flow_y_sign fica no default (+1): eixo lateral = dx do sensor, não
            # calibrável num skid-steer (não translada de lado).
            'flow_swap_xy': True,
            'flow_x_sign': -1.0,
        }],
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
        flow_height_arg,
        lidar_offset_x_arg,
        enable_cone_pose_fix_arg,
        pose_estimator,
        cone_detector,
        trekking_runner,
    ])
