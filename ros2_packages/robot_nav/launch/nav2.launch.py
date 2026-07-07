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
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('robot_nav')
    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    # BT custom: recovery reordenado p/ BackUp ANTES de Spin (robô encurralado
    # entre paredes "volta de onde veio" em vez de girar sem espaço). Ver
    # behavior_trees/navigate_w_backup_first_recovery.xml. Caminho via share dir
    # (não-hardcoded). Override do default_nav_to_pose_bt_xml no bt_navigator.
    bt_xml = os.path.join(pkg, 'behavior_trees', 'navigate_w_backup_first_recovery.xml')
    # Mux de AUTONOMIA (1º estágio do 2-mux): arbitra nav_vel/follow_vel/door_vel
    # numa fonte só (auto_vel_raw) que entra no collision_monitor.
    twist_mux_auto_cfg = os.path.join(pkg, 'config', 'twist_mux_auto.yaml')

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
    # Pose inicial do AMCL. SIM passa o spawn explícito (init_x/y/yaw). REAL não
    # passa nada -> cai no default 'true' + (0,0,0), restaurando a auto-localização
    # na ORIGEM que existia antes do 57c8b13 (regressão: com default 'false' o real
    # nascia NÃO-localizado e o /initialpose precisava ser setado na mão toda vez,
    # e sem pose o ponto pré-porta nem saía). Quem quiser nascer off passa
    # set_initial_pose:=false e seta a pose pela web.
    init_pose_arg = DeclareLaunchArgument('set_initial_pose', default_value='true')
    init_x_arg = DeclareLaunchArgument('init_x', default_value='0.0')
    init_y_arg = DeclareLaunchArgument('init_y', default_value='0.0')
    init_yaw_arg = DeclareLaunchArgument('init_yaw', default_value='0.0')

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    amcl_init = {
        'set_initial_pose': ParameterValue(
            LaunchConfiguration('set_initial_pose'), value_type=bool),
        'initial_pose.x': ParameterValue(
            LaunchConfiguration('init_x'), value_type=float),
        'initial_pose.y': ParameterValue(
            LaunchConfiguration('init_y'), value_type=float),
        'initial_pose.z': 0.0,
        'initial_pose.yaw': ParameterValue(
            LaunchConfiguration('init_yaw'), value_type=float),
    }

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
            output=nav_output, parameters=[params_file, sim_time_param, amcl_init],
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
            # 2-mux (2026-06-26): a saída do smoother (nav_vel) entra no mux de
            # AUTONOMIA (twist_mux_auto) junto com follow_vel/door_vel; a saída
            # do mux passa pelo motion_guard (auto_vel_pre->auto_vel_raw,
            # 2026-07-02) e o collision_monitor filtra depois (->auto_vel).
            # Antes o collision filtrava só a saída do smoother (nav_vel_raw).
            remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'nav_vel')],
        ),
        # Sanitizador do scan PRA O COLLISION MONITOR: o LD06 cospe retornos
        # fantasmas <15 cm (dentro do chassi!) em ~2% dos scans e, com
        # min_points=2 na PolygonStop, 2 pontinhos congelavam o robô no meio
        # da PORTA (captura 2026-06-12). /scan -> /scan_safe só troca esses
        # por inf; SLAM/costmaps/cone_detector seguem no /scan cru.
        Node(
            package='robot_nav', executable='scan_sanitizer',
            name='scan_sanitizer', output=nav_output,
            parameters=[sim_time_param],
        ),
        # Diagnóstico "congela perto do goal" (2026-06-24): grava a cadeia
        # cmd_vel_nav/nav_vel/cmd_vel + odom num CSV (controle_web/logs/
        # freeze_capture.csv) pra eu ler DEPOIS. Read-only, não interfere.
        Node(
            package='robot_nav', executable='freeze_capture',
            name='freeze_capture', output=nav_output,
            parameters=[sim_time_param],
        ),
        # Seguidor decisivo (2026-06-25): segue o /plan (Theta*) como RETO+giro-no-
        # lugar — o robô NÃO arqueia (arc_calib), então o tracking do
        # controller_server (DWB/RotationShim) derivava/zigzagueava. Publica
        # follow_vel (prio 15 no twist_mux, > nav_vel: ignora o controller_server).
        Node(
            package='robot_nav', executable='path_follower',
            name='path_follower', output=nav_output,
            parameters=[sim_time_param],
        ),
        # Travessia de porta: alinha no eixo de porta MARCADA e atravessa
        # reto vigiando o vão (door_vel, prio 20 no twist_mux). Publica
        # /door_zone = gate da máscara de batente no scan_sanitizer.
        # Spec: docs/superpowers/specs/2026-06-12-zonas-de-porta-design.md
        # 2026-06-26 DESATIVADO TEMPORARIAMENTE: o path_follower atravessa a porta
        # NATIVAMENTE (reto+giro-no-lugar pelo /plan do Theta*) — validado 4/4 no
        # real, com a porta deletada do mapa, sem ponto pré-porta. O door_crossing
        # era gambiarra pro DWB velho (que não threadava o vão) e virou obsoleto.
        # Re-habilitar = descomentar este bloco (e colcon build robot_nav).
        # Node(
        #     package='robot_nav', executable='door_crossing',
        #     name='door_crossing', output=nav_output,
        #     parameters=[sim_time_param],
        # ),
        # Mux de AUTONOMIA (1º estágio do 2-mux, 2026-06-26): arbitra as fontes
        # autônomas (nav_vel, follow_vel, door_vel) numa fonte só, auto_vel_pre,
        # que passa pelo motion_guard e entra no collision_monitor. unstuck/
        # humano NÃO entram aqui — ficam no mux FINAL (robot/sim.launch.py),
        # a jusante do collision.
        Node(
            package='twist_mux', executable='twist_mux',
            name='twist_mux_auto', output=nav_output,
            parameters=[twist_mux_auto_cfg, sim_time_param],
            remappings=[('cmd_vel_out', 'auto_vel_pre')],
        ),
        # Cautela com objeto EM MOVIMENTO (2026-07-02): diff temporal do
        # /scan_safe no frame odom -> móvel perto = desacelera; móvel no
        # corredor à frente = para e retoma sozinho. Filtra SÓ a autonomia
        # (auto_vel_pre -> auto_vel_raw); unstuck/manual ficam fora. Failsafe:
        # sem TF/scan -> pass-through (nunca mata a nav). wz passa intocado.
        # respawn=True (8a auditoria A1): o guard esta na ARTERIA da autonomia
        # (auto_vel_pre->auto_vel_raw); o pass-through cobre TF/scan faltando,
        # mas NAO cobre o processo morto — sem respawn, um crash aqui mata a
        # autonomia inteira (auto_vel_raw some) ate alguem relancar a stack.
        # Spec: docs/superpowers/specs/2026-07-02-motion-guard-design.md
        Node(
            package='robot_nav', executable='motion_guard',
            name='motion_guard', output=nav_output,
            parameters=[sim_time_param],
            respawn=True, respawn_delay=1.0,
        ),
        # Collision Monitor: lê /scan_safe (sanitizado acima) e freia
        # auto_vel_raw -> auto_vel ANTES do mux FINAL. Agora protege TODA a
        # autonomia (nav+seguidor+porta), não só o controller_server. Topicos
        # in/out definidos no YAML (cmd_vel_in/out_topic; fonte scan nos params).
        Node(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', output=nav_output,
            parameters=[params_file, sim_time_param],
        ),
        # Watchdog de desencalhe: se o robô NÃO SE DESLOCA (>stuck_radius) por
        # >stuck_timeout com o nav2 comandando, publica unstuck_vel (entrada
        # prio 30 do twist_mux, ACIMA do nav_vel) e dá RÉ furando o collision;
        # depois solta e o nav2 replaneja. SEM GIRO (não vence o atrito do
        # skid-steer). Não toca no collision nem na curva (RotationShim/DWB).
        # Ver docs/superpowers/specs/2026-06-10-unstuck-supervisor-design.md
        Node(
            package='robot_nav', executable='unstuck_supervisor',
            name='unstuck_supervisor', output='screen',
            parameters=[sim_time_param, {
                'stuck_timeout': 5.0,
                # Recovery contextual (2026-06-22): bloqueio à frente que bate no
                # /map cru (parede mapeada) -> ré aos 2s (não os 5s); novo (só no
                # LiDAR) segue os 5s. Os 2s são tb a mini-confirmação. Ver
                # docs/superpowers/specs/2026-06-22-unstuck-recovery-contextual-design.md
                'stuck_timeout_mapped': 2.0,
                'block_range': 0.5,
                'map_occ_threshold': 65,
                'map_neighborhood': 0.22,
                'stuck_radius': 0.05,
                'reverse_distance': 0.30,
                'reverse_speed': 0.25,
                'reverse_time_cap': 6.0,
                'grace': 2.0,
                'nav_latch': 15.0,
                'escalate_after': 2,
                'same_spot_radius': 0.5,
                'escalate_window': 120.0,
                'spin_speed': 3.0,
                'spin_angle': 0.26,
                'spin_time_cap': 4.0,
                'spin_left_boost': 1.4,
                # Geometria da ré (batida 2026-06-11: clearance medida do
                # LiDAR + setor estreito = ré cega). Vão medido do PARA-CHOQUE
                # traseiro num corredor da largura do robô (carcaça 50x50,
                # LiDAR no CENTRO — todos os sensores são centrais). Aborta a
                # ré se o vão cair < margem.
                'rear_lidar_x': 0.0,
                'rear_tail_x': -0.25,
                'rear_half_width': 0.30,
                # 2026-06-15: 0.10 -> 0.20. Bateu numa lata de lixo dando ré:
                # lê vão 0.20m, recua e PARA em 0.08m -> com lata afunilada (o
                # LiDAR pega o topo, a base é mais perto) + overshoot a 0.25 m/s,
                # 8cm "no papel" = encostado. Com 0.20: algo a 20cm -> target=0
                # -> NÃO dá ré (pedido do usuário "não recuar 30cm se há algo a
                # 20cm"); só recua com folga real (>=0.30) parando 20cm antes.
                'rear_stop_margin': 0.20,
                'reverse_min': 0.10,
                'scan_stale': 2.0,
                'nav_move_lin': 0.01,
                'nav_move_ang': 0.05,
                'rate_hz': 10.0,
            }],
            # /scan -> /scan_safe (2026-06-28): o unstuck lia o /scan CRU; no REAL o
            # LD06 cospe fantasmas <0.15m que envenenam near_r/side_clear/gaps (gate
            # e direção do giro) -> bloqueariam giro / disparariam à toa. O scan_safe
            # (scan_sanitizer) zera só os retornos <0.15m (dentro do footprint ±0.25 =
            # fantasma), mantém tudo real >=0.15m. No SIM é no-op (laser limpo).
            remappings=[('scan', 'scan_safe')],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
                # Pi lenta: o velocity_smoother demora >4s pra confirmar o bond e o
                # lifecycle_manager derrubava a stack INTEIRA no meio do bringup
                # (collision_monitor às vezes nem ativava → nav sobe pela metade,
                # comportamento muda, parece bug). Folga grande p/ bringup atômico.
                'bond_timeout': 20.0,
            }],
        ),
    ]

    return LaunchDescription([map_arg, use_sim_time_arg, params_arg,
                              init_pose_arg, init_x_arg, init_y_arg, init_yaw_arg,
                              *nodes])
