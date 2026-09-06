"""Contrato de chegada: goal comum e' (x, y), nunca orientacao final."""
from pathlib import Path

import yaml


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
