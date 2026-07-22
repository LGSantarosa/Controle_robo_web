"""Testes da lógica pura do person_follower (sem ROS)."""
import json
import math

from robot_nav.person_follower import (FollowConfig, FollowFaceFile,
                                        PersonFollower, Target, _rel)

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


def test_acquire_ignora_estatico_so_trava_movel():
    # clusters chegam como (cx, cy, movendo). Só trava quem se MOVE (parede
    # estática perto não vira alvo); ausência do flag = trata como móvel.
    pf = _pf(acquire_range=3.0, acquire_cone_deg=60.0)
    clusters = [(1.0, 0.0, 0.0), (2.0, 0.0, 1.0)]   # parado perto + móvel longe
    assert pf.acquire(clusters, POSE) == Target(2.0, 0.0)   # ignora o parado


def test_associate_rastreia_alvo_que_parou():
    # depois de travado, o alvo que PAROU (movendo=0) continua sendo rastreado
    pf = _pf(assoc_gate=0.6)
    pf.target = Target(2.0, 0.0)
    t = pf.associate([(2.1, 0.0, 0.0)])   # parou, mas dentro do gate
    assert t == Target(2.1, 0.0) and pf.target == Target(2.1, 0.0)


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


def test_control_encara_sem_andar_se_desalinhado():
    pf = _pf()
    vx, wz = pf.control(dist=3.0, bearing_deg=40.0)   # fora do drive_align 20°
    assert vx == 0.0 and wz > 0.0                     # gira p/ esquerda, não anda


def test_control_wz_zero_na_zona_morta_e_cap():
    pf = _pf(face_deadband_deg=8.0, wz_cap=2.4)
    assert pf.control(3.0, 5.0)[1] == 0.0             # dentro da zona morta
    assert pf.control(3.0, 179.0)[1] == 2.4           # satura no cap (esq)
    assert pf.control(3.0, -179.0)[1] == -2.4         # satura no cap (dir)


def test_control_anda_alinhado_e_para_em_1_5m():
    pf = _pf(stop_dist=1.5, stop_hyst=0.2, vx_max=0.25)
    vx, wz = pf.control(dist=3.0, bearing_deg=0.0)    # longe e alinhado
    assert 0.0 < vx <= 0.25 and wz == 0.0
    pf._driving = True
    vx, _ = pf.control(dist=1.4, bearing_deg=0.0)     # colou -> para
    assert vx == 0.0


def test_control_histerese_nao_pulsa_em_1_5m():
    pf = _pf(stop_dist=1.5, stop_hyst=0.2)
    pf._driving = False
    assert pf.control(1.6, 0.0)[0] == 0.0             # dentro de stop+hyst, parado segue parado
    assert pf.control(1.8, 0.0)[0] > 0.0              # acima de stop+hyst -> anda


def _clusters_at(dist, bearing_deg=0.0, pose=POSE):
    """cluster odom a (dist, bearing) do robô em `pose`."""
    rx, ry, ryaw = pose
    a = ryaw + math.radians(bearing_deg)
    return [(rx + dist * math.cos(a), ry + dist * math.sin(a))]


def test_tick_start_trava_e_fala():
    pf = _pf()
    pf.start()
    pf.tick(0.0, _clusters_at(2.5), POSE)
    assert pf.state == 'following' and pf.just_spoke == 'start' and pf.target is not None


def test_tick_start_sem_ninguem_fica_idle():
    pf = _pf()
    pf.start()
    pf.tick(0.0, [], POSE)
    assert pf.state == 'idle' and pf.no_target is True


def test_tick_following_mantem_rumo_no_gap_curto():
    # alvo pisca (guard só vê movimento): dentro do grace, segue indo pro
    # último alvo conhecido em vez de congelar -> seguimento liso, sem flap
    pf = _pf(lost_grace=2.0, stop_dist=1.5, stop_hyst=0.2, vx_max=0.25)
    pf.start(); pf.tick(0.0, _clusters_at(3.0), POSE)   # trava a 3m
    assert pf.state == 'following'
    vx, wz = pf.tick(0.5, [], POSE)                      # sumiu, dentro do grace
    assert pf.state == 'following' and vx > 0.0          # continua indo pro alvo


def test_tick_perde_alvo_vira_lost_e_fala():
    pf = _pf(lost_grace=1.0)
    pf.start(); pf.tick(0.0, _clusters_at(2.5), POSE)
    pf.tick(0.5, [], POSE)                      # sumiu, mas dentro do grace
    assert pf.state == 'following'
    pf.tick(1.6, [], POSE)                      # >lost_grace sem match
    assert pf.state == 'lost' and pf.just_spoke == 'lost'


def test_tick_lost_reaparece_volta_following():
    pf = _pf(lost_grace=1.0)
    pf.start(); pf.tick(0.0, _clusters_at(2.5), POSE); pf.tick(1.6, [], POSE)
    assert pf.state == 'lost'
    pf.tick(2.0, _clusters_at(2.4), POSE)       # reaparece perto do último
    assert pf.state == 'following'


def test_tick_lost_timeout_vira_ending():
    pf = _pf(lost_grace=1.0, lost_timeout=12.0)
    pf.start(); pf.tick(0.0, _clusters_at(2.5), POSE); pf.tick(1.6, [], POSE)
    pf.tick(1.6 + 12.1, [], POSE)
    assert pf.state == 'ending'


def test_stop_de_following_vai_ending_e_reset_volta_idle():
    pf = _pf()
    pf.start(); pf.tick(0.0, _clusters_at(2.5), POSE)
    pf.stop(); assert pf.state == 'ending'
    pf.reset(); assert pf.state == 'idle' and pf.target is None


def test_followfacefile_grava_estado_e_fala(tmp_path):
    p = str(tmp_path / 'ff.json')
    ff = FollowFaceFile(path=p, min_period=0.0)
    assert ff.update(1.0, 'following', speak='start', bearing_deg=12) is True
    d = json.load(open(p))
    assert d['follow_state'] == 'following' and d['speak'] == 'start' and d['cbear_deg'] == 12


def test_followfacefile_io_error_nao_propaga():
    ff = FollowFaceFile(path='/proc/nao_pode/x.json', min_period=0.0)
    assert ff.update(1.0, 'following') is False and ff.last_error is not None
