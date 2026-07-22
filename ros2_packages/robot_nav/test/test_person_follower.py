"""Testes da lógica pura do person_follower (sem ROS)."""
import math

from robot_nav.person_follower import FollowConfig, PersonFollower, Target, _rel

POSE = (0.0, 0.0, 0.0)   # robô na origem olhando +x (frame odom)


def _pf(**kw):
    return PersonFollower(FollowConfig(**kw))


def test_rel_bearing_frente_esquerda_direita():
    d, b = _rel(2.0, 0.0, POSE)
    assert abs(d - 2.0) < 1e-6 and abs(b) < 1e-6
    _, b = _rel(2.0, 2.0, POSE)
    assert abs(b - 45.0) < 1e-6
    _, b = _rel(2.0, -2.0, POSE)
    assert abs(b + 45.0) < 1e-6


def test_acquire_pega_o_mais_proximo_no_cone():
    pf = _pf(acquire_range=3.0, acquire_cone_deg=60.0)
    clusters = [(2.5, 0.0), (1.2, 0.2), (2.0, 5.0)]  # 2º é o mais perto; 3º fora do cone
    t = pf.acquire(clusters, POSE)
    assert t == Target(1.2, 0.2)


def test_acquire_none_se_fora_do_alcance_ou_cone():
    pf = _pf(acquire_range=3.0, acquire_cone_deg=60.0)
    assert pf.acquire([(4.0, 0.0)], POSE) is None          # longe
    assert pf.acquire([(1.0, 3.0)], POSE) is None          # fora do cone (~72°)
    assert pf.acquire([], POSE) is None                    # vazio
