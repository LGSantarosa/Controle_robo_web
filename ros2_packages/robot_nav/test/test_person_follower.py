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


def test_associate_segue_salto_pequeno():
    pf = _pf(assoc_gate=0.6)
    pf.target = Target(2.0, 0.0)
    t = pf.associate([(2.3, 0.1), (5.0, 5.0)])   # 0.32m de salto < gate
    assert t == Target(2.3, 0.1) and pf.target == Target(2.3, 0.1)


def test_associate_none_se_salto_grande():
    pf = _pf(assoc_gate=0.6)
    pf.target = Target(2.0, 0.0)
    assert pf.associate([(3.0, 0.0)]) is None    # 1.0m > gate
    assert pf.associate([]) is None
