#!/usr/bin/env python3
"""
Robot base launch.

Sobe:
  1. robot_state_publisher (URDF/TF)
  2. mega_bridge          (USB ↔ Arduino MEGA ↔ 2 hoverboards + sensores)
  3. pose_estimator       (funde 4 RPMs + IMU (MPU) + IMU #2 (BNO055, com
                           heading absoluto do mag) + flow → /odom + TF
                           odom→base_link, com degradação graciosa; também
                           publica /trekking/*)
  4. cmd_vel_to_wheels    (/cmd_vel → /wheel_vel_setpoints)
  5. joy_node            (PS4 ou Xbox em /dev/input/jsN → /joy; ver detect_joystick)
  6. teleop_twist_joy    (/joy → joy_vel, com dead-man no L1/LB)
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


def detect_joystick():
    """Descobre QUAL controle está plugado e devolve (device_id, arquivo de config).

    Por que auto-detecção em vez de uma flag: o PS4 e o Xbox Series têm números
    de botão diferentes (L1=4/R1=5 contra LB=6/RB=7 — medido em 2026-08-11), e
    o teleop_twist_joy só aceita um `enable_button`. Config errado = dead-man no
    botão errado = robô que não anda. Como quem sobe a stack no dia a dia não
    passa flag nenhuma (`robot-connect` e pronto), a escolha tem que ser
    automática.

    Lê o nome que o driver publica em /dev/input/jsN (ioctl JSIOCGNAME) e casa
    por substring. Sem controle conectado, cai no PS4 em js0 — comportamento
    idêntico ao de antes desta função existir.

    Override manual, se algum dia a heurística errar:
        ROBOT_JOY_CONFIG=teleop_xbox.yaml ROBOT_JOY_DEVICE=1 ./launch.sh
    """
    import fcntl
    import glob

    forced_cfg = os.environ.get('ROBOT_JOY_CONFIG')
    forced_dev = os.environ.get('ROBOT_JOY_DEVICE')
    if forced_cfg or forced_dev:
        cfg = forced_cfg or 'teleop_ps4.yaml'
        dev = int(forced_dev or 0)
        print(f'[robot.launch] joystick forçado por env: {cfg} em js{dev}')
        return dev, cfg

    JSIOCGNAME_128 = 0x80806A13  # _IOR('j', 0x13, char[128])

    for path in sorted(glob.glob('/dev/input/js*')):
        try:
            with open(path, 'rb') as fh:
                raw = fcntl.ioctl(fh, JSIOCGNAME_128, bytes(128))
            name = raw.rstrip(b'\0').decode('utf-8', 'replace')
        except OSError:
            continue  # device sumiu no meio, ou sem permissão — tenta o próximo

        dev_id = int(path.rsplit('js', 1)[1])
        if 'xbox' in name.lower():
            print(f'[robot.launch] controle Xbox em {path} ({name}) → teleop_xbox.yaml')
            return dev_id, 'teleop_xbox.yaml'
        print(f'[robot.launch] controle em {path} ({name}) → teleop_ps4.yaml')
        return dev_id, 'teleop_ps4.yaml'

    print('[robot.launch] nenhum /dev/input/js* — assumindo PS4 em js0')
    return 0, 'teleop_ps4.yaml'


def generate_launch_description():
    pkg = get_package_share_directory('nav2_trekking')
    urdf_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius', default_value='0.082',
        description='Raio das rodas em metros (calibrado 2026-06-08: 0.085 dava +3.7% longo)'
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
        description='Sinal da taxa de yaw da IMU. 2026-07-01: voltou o MPU6050 '
                    'antigo, montado de PONTA-CABEÇA (Z pra baixo) -> -1.0. (Era '
                    '+1.0 com o 6500 montado plano.) Confirmar na bancada: girando '
                    'p/ esquerda o yaw do /odom tem que SUBIR; se descer, +1.0.'
    )
    imu2_yaw_sign_arg = DeclareLaunchArgument(
        'imu2_yaw_sign', default_value='1.0',
        description='Sinal da taxa de yaw E do heading da BNO055 (IMU #2), pra '
                    'casar a montagem dela. Valide na bancada com '
                    'tools/imu2_check.py: girando p/ ESQUERDA, o gz das DUAS '
                    'IMUs tem que ter o MESMO sinal. Se sairem opostos, use '
                    'imu2_yaw_sign:=-1.0 (o pose_estimator ignora a #2 e loga '
                    'erro enquanto discordarem, entao o robo nao anda torto).'
    )
    use_imu2_arg = DeclareLaunchArgument(
        'use_imu2', default_value='true',
        description='Funde a BNO055 (2a taxa de giro + heading absoluto do '
                    'magnetometro). use_imu2:=false volta a pose a ser '
                    'exatamente a de antes dela (so MPU + rodas).'
    )
    use_imu2_heading_arg = DeclareLaunchArgument(
        'use_imu2_heading', default_value='true',
        description='Ancora o yaw no norte magnetico (corrige a deriva do yaw '
                    'integrado, devagar e com teto). Desligue com '
                    'use_imu2_heading:=false se o local tiver ferro/ima demais '
                    '(galpao com estrutura metalica, por exemplo) — a BNO055 '
                    'continua entrando como 2a taxa de giro.'
    )
    use_flow_arg = DeclareLaunchArgument(
        'use_flow', default_value='false',
        description='Funde o optical flow (PMW3901) na translacao. OFF por padrao '
                    'desde 2026-08-26: o SENSOR NAO ESTA NO ROBO (arrancado em '
                    '2026-07-01, commit 33647e4). O codigo fica: a escala '
                    '(0.200mm/count), o corte de EMI e o flow_yaw_gate seguem '
                    'validos e use_flow:=true reativa tudo no dia que o PMW3901 '
                    'voltar pro chassi. Ate la, ligado ele so rende dois warns '
                    'por minuto (flow stale / alpha=0.000) sobre um sensor '
                    'ausente — a fusao ja cai pra roda sozinha (flow_age=inf '
                    '-> alpha=0), entao isto e ruido, nao correcao de conta.'
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
        package='nav2_trekking',
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
        package='nav2_trekking',
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
            # Sinal do yaw da MPU9250 (montagem PLANA, Z pra cima → +1.0). Override
            # de bancada via `imu_yaw_sign:=-1.0` se o giro vier invertido.
            'imu_yaw_sign': LaunchConfiguration('imu_yaw_sign'),
            # IMU #2 (BNO055, 9 eixos). Entra em dois caminhos: a MAIOR PARTE
            # do peso na taxa de yaw (ela recalibra o próprio bias do giro, o
            # MPU só calibra uma vez no boot) e a âncora de heading magnético
            # que tira a deriva do yaw integrado — o que mais pesa no trekking.
            # Os defaults do nó já são estes; ficam explícitos aqui porque são
            # os knobs que se mexe na bancada.
            'use_imu2': LaunchConfiguration('use_imu2'),
            'imu2_yaw_sign': LaunchConfiguration('imu2_yaw_sign'),
            # 0.8: o MPU segue na conta só como cross-check e fallback quente,
            # não como metade da verdade. Cai sozinho pra 0.5 enquanto o giro da
            # BNO055 não estiver calibrado (janela de boot).
            'imu2_rate_weight': 0.8,
            'imu2_gyro_calib_min': 2,
            'use_imu2_heading': LaunchConfiguration('use_imu2_heading'),
            'heading_gain': 0.2,
            'heading_max_rate': 0.15,
            'mag_calib_min': 2,
            # Flow OFF por padrão desde 2026-08-26 — o PMW3901 saiu do robô.
            # use_flow:=true reativa quando ele voltar (ver use_flow_arg).
            'use_flow': LaunchConfiguration('use_flow'),
            # Calibração do PMW3901 → body frame (movida do trekking.launch.py:
            # frente entra por dy negativo do sensor). Vale pra TODOS os modos
            # agora que a fusão é a odometria base.
            'flow_swap_xy': True,
            'flow_x_sign': -1.0,
        }],
    )

    cmd_vel_to_wheels = Node(
        package='nav2_trekking',
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

    joy_device_id, joy_cfg_name = detect_joystick()
    teleop_joy_cfg = os.path.join(pkg, 'config', joy_cfg_name)
    twist_mux_cfg = os.path.join(pkg, 'config', 'twist_mux.yaml')

    # joy_node — lê o controle e publica /joy.
    # Se o controle não estiver conectado, o nó fica tentando abrir o device
    # (loga aviso); manda o stderr pro log file pra não poluir o terminal
    # principal a cada ~1 s.
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output={'stdout': 'screen', 'stderr': 'log'},
        parameters=[{
            'device_id': joy_device_id,
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
        parameters=[teleop_joy_cfg],
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
        imu2_yaw_sign_arg,
        use_imu2_arg,
        use_imu2_heading_arg,
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
