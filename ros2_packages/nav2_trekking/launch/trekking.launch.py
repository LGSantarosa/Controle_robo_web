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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    v_max_arg = DeclareLaunchArgument(
        # 0.35 -> 0.90 (2026-08-25). Trekking e prova de VELOCIDADE e 76% do
        # tempo de uma rota era reta a 0.33 m/s. Varredura no sim com a rota
        # gravada pelo dono (0.35/0.50/0.70/0.90/1.20), com o giro ja rapido:
        # 18.4 s -> 10.2 s (-45%), pior desvio da trilha 9.1 -> 16.2 cm, dentro
        # do orcamento de 40 cm que ele definiu. 1.20 dava so 0.7 s a mais e
        # gastava metade da margem que sobrava — e alta velocidade e justamente
        # onde o sim e menos confiavel (o atrito de skid do Gazebo e
        # aproximacao; o robo real derrapa MAIS, nao menos).
        # ATENCAO: nao validado no robo real. A deriva assimetrica do giro
        # (+13.4 cm por curva a direita contra -3.1 a esquerda, medida hoje a
        # 24°/s) tende a CRESCER a 61°/s, e a odometria nao a enxerga.
        'v_max', default_value='0.90',
        description='Velocidade linear máxima do PID (m/s)'
    )
    lidar_offset_x_arg = DeclareLaunchArgument(
        'lidar_offset_x', default_value='0.0',
        description='Deslocamento x do base_laser em relação a base_link (m); '
                    'LiDAR fica no CENTRO do robô (0.10 antigo era falso)'
    )
    enable_cone_pose_fix_arg = DeclareLaunchArgument(
        'enable_cone_pose_fix', default_value='true',
        description='Liga a correção persistente de pose por cone-âncora (A/B em campo)'
    )
    aim_freeze_arg = DeclareLaunchArgument(
        'aim_freeze_radius', default_value='0.40',
        description='m — para de perseguir a mira a menos disto do alvo. 0 desliga')
    aim_freeze_final_arg = DeclareLaunchArgument(
        'aim_freeze_radius_final', default_value='0.25',
        description='idem, no ultimo ponto. 0 desliga')
    final_arr_arg = DeclareLaunchArgument(
        'final_arrival_tolerance', default_value='0.08',
        description='m — anel de chegada do ULTIMO ponto (o do meio e o '
                    'arrival_tolerance, que e passagem)')
    use_imu_yaw_arg = DeclareLaunchArgument(
        'use_imu_yaw', default_value='true',
        description='SIM: yaw da IMU (como o robô real). false = yaw das RODAS, '
                    'que patinam no pivô — é o comportamento de antes de '
                    '2026-08-26, mantido só para o A/B')
    cone_fix_repeat_arg = DeclareLaunchArgument(
        'cone_fix_repeat', default_value='false',
        description='corrige a pose REPETIDO no mesmo cone (instrumentação do '
                    'diagnóstico de associação; foi revertido por instabilidade)')
    sim_pose_arg = DeclareLaunchArgument(
        'sim_pose_from_odom', default_value='false',
        description='SÓ SIM: publica /trekking/pose a partir da /odom do Gazebo. '
                    'No real quem publica é o pose_estimator (robot.launch.py), '
                    'que o --sim não sobe. Ver sim_trekking_pose.py.'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Relógio do /clock (Gazebo). O launch.sh passa true em --sim; '
                    'sem isto os nós do trekking rodavam no relógio de PAREDE '
                    'enquanto o resto do sim estava no tempo simulado.'
    )

    cone_detector = Node(
        package='nav2_trekking',
        executable='cone_detector',
        name='cone_detector',
        output='screen',
        parameters=[{
            'lidar_offset_x': LaunchConfiguration('lidar_offset_x'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    trekking_runner = Node(
        package='nav2_trekking',
        executable='trekking_runner',
        name='trekking_runner',
        output='screen',
        parameters=[{
            'v_max': LaunchConfiguration('v_max'),
            'enable_cone_pose_fix': LaunchConfiguration('enable_cone_pose_fix'),
            'cone_fix_repeat': LaunchConfiguration('cone_fix_repeat'),
            'aim_freeze_radius': LaunchConfiguration('aim_freeze_radius'),
            'aim_freeze_radius_final':
                LaunchConfiguration('aim_freeze_radius_final'),
            'final_arrival_tolerance':
                LaunchConfiguration('final_arrival_tolerance'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        # Saída vai pra auto_vel = a entrada de AUTONOMIA do twist_mux (prio 10),
        # abaixo de PS4/teclado/web — o dono sempre assume por cima.
        #
        # Era `nav_vel` e estava MORTO: na refatoração de 2 muxes (06-26) o mux
        # final passou a escutar `auto_vel` (saída do collision_monitor) e
        # ninguém mais consome `nav_vel` — quem fazia essa ponte é o
        # `twist_mux_auto`, que só sobe no nav2.launch.py. O trekking, congelado
        # em 06-12, seguiu publicando no vazio: medido com o robô em `play`,
        # `/nav_vel` tinha 1 publisher e 0 subscribers, e o robô não saía do
        # lugar (no real também).
        #
        # auto_vel entra DEPOIS do collision_monitor de propósito: o trekking
        # não tem reflexo de colisão por desenho — ele não liga se for bater.
        remappings=[('cmd_vel', 'auto_vel')],
    )

    sim_trekking_pose = Node(
        package='nav2_trekking',
        executable='sim_trekking_pose',
        name='sim_trekking_pose',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_imu_yaw': LaunchConfiguration('use_imu_yaw'),
        }],
        condition=IfCondition(LaunchConfiguration('sim_pose_from_odom')),
    )

    return LaunchDescription([
        v_max_arg,
        lidar_offset_x_arg,
        enable_cone_pose_fix_arg,
        cone_fix_repeat_arg,
        use_imu_yaw_arg,
        aim_freeze_arg,
        aim_freeze_final_arg,
        final_arr_arg,
        sim_pose_arg,
        use_sim_time_arg,
        sim_trekking_pose,
        cone_detector,
        trekking_runner,
    ])
