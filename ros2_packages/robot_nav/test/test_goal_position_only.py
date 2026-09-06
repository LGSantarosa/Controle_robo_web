"""Contrato de yaw: goal comum e' (x,y); porta continua exigindo heading."""
import math
from pathlib import Path

import yaml

from robot_nav.door_crossing import DoorCrossConfig, DoorCrossing


ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / 'ros2_packages' / 'robot_nav' / 'config'


def _params(name):
    with (CONFIG / name).open(encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_goal_checker_aceita_qualquer_yaw_em_todos_os_perfis():
    for name in ('nav2_params_arena.yaml', 'nav2_params_pi.yaml',
                 'nav2_params_legacy.yaml'):
        goal = _params(name)['controller_server']['ros__parameters']['goal_checker']
        assert goal['yaw_goal_tolerance'] >= 6.28, name


def test_planner_descarta_o_yaw_informado_pelo_usuario():
    for name in ('nav2_params_arena.yaml', 'nav2_params_pi.yaml',
                 'nav2_params_legacy.yaml'):
        planner = _params(name)['planner_server']['ros__parameters']['GridBased']
        assert planner['use_final_approach_orientation'] is True, name


def test_rotation_shim_nao_faz_giro_final():
    for name in ('nav2_params_arena.yaml', 'nav2_params_pi.yaml'):
        follower = _params(name)['controller_server']['ros__parameters']['FollowPath']
        assert follower['rotate_to_goal_heading'] is False, name


def test_door_crossing_continua_exigindo_e_corrigindo_yaw():
    """Ignorar yaw do GOAL nunca pode desligar o alinhamento da PORTA.

    A porta horizontal exige heading +pi/2. Entregue no eixo mas com yaw=0, o
    door_crossing precisa comandar giro; so' depois das leituras alinhadas ele
    pode comitar a travessia.
    """
    cfg = DoorCrossConfig(zone_radius=1.2, stage_dist=0.6)
    crossing = DoorCrossing(cfg)
    door = {'id': 1, 'a': [1.0, 2.0], 'b': [2.0, 2.0]}

    def step(now, yaw):
        return crossing.update(
            now, (1.5, 1.4, yaw), [door],
            goal_active=True, nav_forward=True,
            gap=math.inf, scan_fresh=True, front_gap=math.inf,
            goal_succeeded=True,
        )

    cmd = step(0.0, 0.0)
    assert cmd.state == 'rotating'
    assert cmd.wz != 0.0

    for tick in range(cfg.align_stable):
        cmd = step(0.1 + tick * 0.05, math.pi / 2)
    assert cmd.state == 'crossing'
