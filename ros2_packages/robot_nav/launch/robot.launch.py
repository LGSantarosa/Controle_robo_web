#!/usr/bin/env python3
"""
Robot base launch.

Sobe:
  1. robot_state_publisher (URDF/TF)
  2. mega_bridge          (USB ↔ Arduino MEGA ↔ 2 hoverboards + sensores)
  3. pose_estimator       (funde 4 RPMs + IMU + flow → /odom + TF odom→base_link,
                           com degradação graciosa; também publica /trekking/*)
  4. cmd_vel_to_wheels    (/cmd_vel → /wheel_vel_setpoints)
  5. joy_node            (PS4 em /dev/input/js0 → /joy)
  6. teleop_twist_joy    (/joy → joy_vel, com dead-man no L1)
  7. twist_mux           (joy_vel > key_vel > web_vel > nav_vel → /cmd_vel)

Publishers do twist_mux que NÃO sobem aqui (rodam à parte):
  - key_vel: bin/robot-key em terminal SSH separado (WASD via teclado)
  - web_vel: controle_web/app.py quando WEB_TELEOP=on
  - nav_vel: nav2.launch.py (velocity_smoother) ou trekking.launch.py

Requer os pacotes apt: joy, teleop_twist_joy, twist_mux (instalados pelo
setup_pi.sh). Sem eles a launch falha — ver PLANO_HEADLESS_2026-05-22 §2.3.
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
    left_wheel_sign_arg = DeclareLaunchArgument(
        'left_wheel_sign', default_value='1.0',
        description='Polaridade do lado esquerdo (-1.0 inverte). Aplicado em cmd_vel_to_wheels E pose_estimator.'
    )
    right_wheel_sign_arg = DeclareLaunchArgument(
        'right_wheel_sign', default_value='1.0',
        description='Polaridade do lado direito (-1.0 inverte). Aplicado em cmd_vel_to_wheels E pose_estimator.'
    )
    imu_yaw_sign_arg = DeclareLaunchArgument(
        'imu_yaw_sign', default_value='-1.0',
        description='Sinal da taxa de yaw da MPU6050. -1.0 = montada de ponta-cabeca '
                    '(Z pra baixo, default). Trocar p/ 1.0 se o giro vier invertido '
                    'na bancada — sem reflashear a MEGA.'
    )
    use_flow_arg = DeclareLaunchArgument(
        'use_flow', default_value='false',
        description='Funde o optical flow (PMW3901) na translação. OFF por padrao: '
                    'o sensor cospe lixo por EMI do motor ao dirigir e infla a pose '
                    '(ver project_pmw3901_emi_motor). Religar com use_flow:=true quando '
                    'o HW do shifter for corrigido.'
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
            # Placa traseira: motores invertidos + cabos L/R trocados (confirmado
            # em bancada 2026-05-30, a olho). rear_invert_speed=True acerta
            # frente/ré; o giro sai CERTO sem inverter o steer (o swap L/R já
            # cancela a necessidade — por isso rear_invert_steer fica False).
            # NÃO inverter o steer: testado, faz a traseira CONTRA-GIRAR.
            # O FEEDBACK da traseira é corrigido à parte no mega_bridge (_fb_map,
            # swap L↔R) pro /odom não cancelar no giro — ver AUDITORIA_2026-05-29b.
            'rear_invert_speed': True,
        }],
    )

    pose_estimator = Node(
        package='robot_nav',
        executable='pose_estimator',
        name='pose_estimator',
        output='screen',
        parameters=[{
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            # wheel_base aqui é a bitola EFETIVA (calibrada no skid-steer) usada
            # pra estimar o yaw de roda quando não há IMU. Default geométrico até
            # calibrar (ver plano, Task 5).
            'wheel_base': LaunchConfiguration('wheel_base'),
            'left_wheel_sign': LaunchConfiguration('left_wheel_sign'),
            'right_wheel_sign': LaunchConfiguration('right_wheel_sign'),
            # Janela de freshness da IMU: sem /imu/data nesse tempo → cai pro
            # yaw de roda (degradação graciosa).
            'imu_timeout': 0.3,
            # Sinal do yaw da MPU6050 (montagem de ponta-cabeça → -1.0). Override
            # de bancada via `imu_yaw_sign:=1.0` se o giro vier invertido.
            'imu_yaw_sign': LaunchConfiguration('imu_yaw_sign'),
            # Flow OFF por padrão (EMI do PMW3901 infla a pose ao dirigir).
            'use_flow': LaunchConfiguration('use_flow'),
            # Calibração do PMW3901 → body frame (movida do trekking.launch.py:
            # frente entra por dy negativo do sensor). Vale pra TODOS os modos
            # agora que a fusão é a odometria base.
            'flow_swap_xy': True,
            'flow_x_sign': -1.0,
        }],
    )

    cmd_vel_to_wheels = Node(
        package='robot_nav',
        executable='cmd_vel_to_wheels',
        name='cmd_vel_to_wheels',
        output='screen',
        parameters=[{
            'wheel_base': LaunchConfiguration('wheel_base'),
            'linear_scale': LaunchConfiguration('linear_scale'),
            'left_wheel_sign': LaunchConfiguration('left_wheel_sign'),
            'right_wheel_sign': LaunchConfiguration('right_wheel_sign'),
            # Continua assinando cmd_vel — agora é a SAÍDA do twist_mux. Nada muda aqui.
            'cmd_vel_topic': 'cmd_vel',
        }],
    )

    teleop_ps4_cfg = os.path.join(pkg, 'config', 'teleop_ps4.yaml')
    twist_mux_cfg = os.path.join(pkg, 'config', 'twist_mux.yaml')

    # joy_node — lê o PS4 em /dev/input/js0 e publica /joy.
    # Se o controle não estiver conectado, o nó fica tentando abrir o device
    # (loga aviso); manda o stderr pro log file pra não poluir o terminal
    # principal a cada ~1 s.
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output={'stdout': 'screen', 'stderr': 'log'},
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
    )

    # teleop_twist_joy — /joy → joy_vel (entrada de maior prioridade do mux).
    # require_enable_button (L1) faz o dead-man: só publica enquanto segurado,
    # então soltar o L1 deixa o mux cair pro nav_vel (Nav2/trekking assume).
    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[teleop_ps4_cfg],
        remappings=[('cmd_vel', 'joy_vel')],
    )

    # twist_mux — arbitra joy_vel/key_vel/web_vel/nav_vel → cmd_vel (resolve B20).
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_cfg],
        remappings=[('cmd_vel_out', 'cmd_vel')],
    )

    return LaunchDescription([
        wheel_radius_arg,
        wheel_base_arg,
        linear_scale_arg,
        left_wheel_sign_arg,
        right_wheel_sign_arg,
        imu_yaw_sign_arg,
        use_flow_arg,
        mega_port_arg,
        mega_baud_arg,
        robot_state_publisher,
        mega_bridge,
        pose_estimator,
        cmd_vel_to_wheels,
        joy_node,
        teleop_twist_joy,
        twist_mux,
    ])
