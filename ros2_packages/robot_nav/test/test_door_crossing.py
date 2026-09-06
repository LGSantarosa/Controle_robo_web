import math
import os

import pytest
import yaml

from robot_nav.door_crossing import (
    DoorGeom,
    door_geometry,
    door_progress_lateral,
    crossing_yaw,
)


def test_door_geometry_axis_horizontal_wall():
    # parede ao longo de x (porta "olhando" pra cima/baixo)
    g = door_geometry((1.0, 2.0), (2.0, 2.0))
    assert (g.cx, g.cy) == pytest.approx((1.5, 2.0))
    assert g.half_width == pytest.approx(0.5)
    assert (g.tx, g.ty) == pytest.approx((1.0, 0.0))
    assert (g.nx, g.ny) == pytest.approx((0.0, 1.0))


def test_progress_lateral_and_side():
    g = door_geometry((1.0, 2.0), (2.0, 2.0))
    # robô 1 m "abaixo" da porta, 0.2 m à direita do centro
    s, d = door_progress_lateral(g, 1.7, 1.0, side=+1)
    assert s == pytest.approx(-1.0)   # ainda não cruzou (progresso negativo)
    assert d == pytest.approx(0.2)    # offset lateral ao longo da parede
    # mesmo ponto com side=-1: progresso inverte, lateral mantém o sinal de t
    s2, _ = door_progress_lateral(g, 1.7, 1.0, side=-1)
    assert s2 == pytest.approx(1.0)


def test_crossing_yaw_faces_normal():
    g = door_geometry((1.0, 2.0), (2.0, 2.0))
    assert crossing_yaw(g, side=+1) == pytest.approx(math.pi / 2)   # +n = +y
    assert crossing_yaw(g, side=-1) == pytest.approx(-math.pi / 2)


def test_door_geometry_diagonal():
    g = door_geometry((0.0, 0.0), (1.0, 1.0))
    assert g.half_width == pytest.approx(math.sqrt(2) / 2)
    # n perpendicular a t, ambos unitários
    assert g.tx * g.nx + g.ty * g.ny == pytest.approx(0.0)
    assert math.hypot(g.nx, g.ny) == pytest.approx(1.0)


from robot_nav.door_crossing import gap_ahead


def _scan_one_point(x_robot, y_robot):
    # constrói um scan de 8 feixes com UM ponto em (x,y) no frame do robô
    a = math.atan2(y_robot, x_robot)
    r = math.hypot(x_robot, y_robot)
    angle_min, inc = -math.pi, math.pi / 4
    ranges = [float('inf')] * 8
    idx = int(round((a - angle_min) / inc)) % 8
    ranges[idx] = r
    return ranges, angle_min, inc


def test_gap_ahead_sees_obstacle_in_corridor():
    ranges, amin, ainc = _scan_one_point(0.5, 0.0)   # bem na frente
    g = gap_ahead(ranges, amin, ainc, pose=(0.0, 0.0, 0.0),
                  jambs=[], jamb_r=0.30)
    assert g == pytest.approx(0.5, abs=0.15)  # discretização de 8 feixes


def test_gap_ahead_ignores_lateral_and_behind():
    for px, py in [(0.0, 1.0), (-0.5, 0.0), (0.5, 0.6)]:
        ranges, amin, ainc = _scan_one_point(px, py)
        g = gap_ahead(ranges, amin, ainc, pose=(0.0, 0.0, 0.0),
                      jambs=[], jamb_r=0.30)
        assert math.isinf(g)


def test_gap_ahead_excludes_marked_jamb():
    # ponto na frente, mas que em coordenadas do MAPA cai no disco do batente
    ranges, amin, ainc = _scan_one_point(0.5, 0.0)
    pose = (3.0, 4.0, 0.0)                      # robô no mapa
    jamb = (3.5, 4.0)                            # batente exatamente ali
    g = gap_ahead(ranges, amin, ainc, pose=pose,
                  jambs=[jamb], jamb_r=0.30)
    assert math.isinf(g)                         # batente não conta como vão


from robot_nav.door_crossing import DoorCrossing, DoorCrossConfig

DOOR = {'id': 1, 'a': [1.0, 2.0], 'b': [2.0, 2.0]}   # parede em x, vão 1.0 m
# Config FIXA do teste (independente da afinação de produção, que muda em campo:
# stage_dist/zone_radius/align_timeout foram retunados 2026-06-15). Estes testes
# verificam a MÁQUINA DE ESTADOS, não os números de campo.
CFG = DoorCrossConfig(zone_radius=1.2, stage_dist=0.6, align_timeout=15.0,
                      total_timeout=40.0)


def mk():
    return DoorCrossing(CFG)


def step(dc, t, pose, goal=True, nav=True, gap=math.inf, fresh=True, cleared=True):
    # cleared=True simula "ponto pré-porta cumprido" (libera o arme — pendência C).
    # Default True pra os testes da máquina de estados seguirem armando.
    return dc.update(t, pose, [DOOR], goal, nav, gap, fresh,
                     goal_succeeded=cleared)


def test_idle_sem_goal_ou_fora_da_zona():
    dc = mk()
    # na zona mas sem goal
    assert step(dc, 0.0, (1.5, 1.2, math.pi/2), goal=False).state == 'idle'
    # com goal mas longe (>zone_radius do centro)
    assert step(dc, 0.1, (1.5, -1.0, math.pi/2)).state == 'idle'
    # sem pose (TF caiu) nunca arma
    assert step(dc, 0.2, None).state == 'idle'


def test_arma_e_vai_pro_rotating():
    # 2026-06-19: a web (nav2 via ponto-pré-porta) entrega o robô centrado na
    # frente da porta -> o door arma DIRETO no rotating (só alinha o ângulo),
    # sem staging (que perseguia o centro do vão e estragava a posição boa).
    dc = mk()
    c = step(dc, 0.0, (1.5, 1.0, math.pi/2))
    assert c.state == 'rotating'
    assert c.door_id == 1
    assert dc.side == +1


def test_staging_converge_e_rotaciona():
    # staging não é mais o caminho do arme; só é alcançado como recuperação
    # pós-escape. Testado direto aqui (força o estado).
    dc = mk()
    step(dc, 0.0, (1.7, 1.2, 0.0))     # arma -> rotating
    dc.state = 'staging'               # recuperação: staging
    dc._align_t0 = 0.0
    dc._align_anchor = (1.7, 1.2)
    c = step(dc, 0.5, (1.7, 1.2, 0.0))
    assert c.state == 'staging'        # ainda indo pro ponto no eixo
    # teleporta pro ponto de staging (simula chegada): vira ROTATING
    stage_y = 2.0 - CFG.stage_dist
    c = step(dc, 1.0, (1.5, stage_y, 0.0))
    assert c.state == 'rotating'
    assert c.vx == pytest.approx(0.0)
    assert c.wz != 0.0   # girando pra encarar pi/2


def test_staging_nao_solta_para_rotating_se_ainda_nao_cabe_na_porta():
    # Regressão de 2026-09-03: na porta 2 da arena o staging chegava "perto"
    # da linha, mas com lateral ainda grande demais para a boca. Soltar cedo
    # para o rotating devolvia immediately ao will_clear e travava o trecho.
    dc = mk()
    step(dc, 0.0, (1.7, 1.2, 0.0))     # arma -> rotating
    dc.state = 'staging'
    dc._align_t0 = 0.0
    dc._align_anchor = (1.7, 1.2)
    stage_y = 2.0 - CFG.stage_dist
    c = step(dc, 1.0, (1.58, stage_y, math.pi / 2))   # d=0.08, ainda fora do fit
    assert c.state == 'staging'
    assert c.vx >= 0.0


def test_rotating_estavel_vira_crossing():
    dc = mk()
    stage_y = 2.0 - CFG.stage_dist
    step(dc, 0.0, (1.5, stage_y - 0.3, math.pi/2))    # arma (staging)
    step(dc, 0.1, (1.5, stage_y, math.pi/2))          # chegou -> rotating
    # já alinhado: precisa de align_stable ticks estáveis pra promover
    t = 0.2
    for _ in range(CFG.align_stable):
        c = step(dc, t, (1.5, stage_y, math.pi/2))
        t += 0.05
    assert c.state == 'crossing'


def _ate_crossing(dc):
    stage_y = 2.0 - CFG.stage_dist
    step(dc, 0.0, (1.5, stage_y - 0.3, math.pi/2))
    step(dc, 0.1, (1.5, stage_y, math.pi/2))
    t = 0.2
    for _ in range(CFG.align_stable):
        c = step(dc, t, (1.5, stage_y, math.pi/2))
        t += 0.05
    assert c.state == 'crossing'
    return t


def test_crossing_anda_reto_e_solta_depois_da_porta():
    dc = mk()
    t = _ate_crossing(dc)
    c = step(dc, t, (1.5, 1.9, math.pi/2))
    assert c.state == 'crossing' and c.vx == pytest.approx(CFG.cross_speed)
    # passou do centro + exit_margin -> solta
    c = step(dc, t + 1.0, (1.5, 2.0 + CFG.exit_margin + 0.05, math.pi/2))
    assert c.state == 'idle'


def test_crossing_para_correcao_lateral_apos_o_centro():
    # 2026-06-19: a correção lateral persegue o EIXO dos 2 cliques (doors.json),
    # não o corredor real -> dava uma curvinha no fim que deixava o robô torto no
    # corredor pós-porta. Antes do centro (s<0) corrige lateral; depois (s>=0) NÃO.
    dc = mk()
    _ate_crossing(dc)                              # side=+1, centro da porta (1.5,2.0)
    # offset lateral d=0.1, AINDA antes do centro (s=-0.1) -> corrige (wz!=0)
    c = step(dc, 1.0, (1.6, 1.9, math.pi/2))
    assert c.state == 'crossing' and c.wz < 0.0
    # mesmo offset, PASSADO o centro (s=+0.1) -> NÃO corrige lateral; yaw alinhado -> wz~0
    c = step(dc, 1.1, (1.6, 2.1, math.pi/2))
    assert c.state == 'crossing' and c.wz == pytest.approx(0.0)


def test_crossing_aborta_se_vao_fecha_ou_goal_morre():
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 1.9, math.pi/2), gap=0.3).state == 'idle'
    dc2 = mk()
    t2 = _ate_crossing(dc2)
    assert step(dc2, t2, (1.5, 1.9, math.pi/2), goal=False).state == 'idle'


def test_align_timeout_aborta_e_respeita_cooldown():
    dc = mk()
    step(dc, 0.0, (1.5, 1.0, math.pi/2))                       # arma
    c = step(dc, CFG.align_timeout + 0.1, (1.5, 1.0, math.pi/2))
    assert c.state == 'idle'
    # cooldown: tick seguinte ainda não rearma
    assert step(dc, CFG.align_timeout + 0.2, (1.5, 1.0, math.pi/2)).state == 'idle'
    # passado o cooldown, rearma
    t = CFG.align_timeout + CFG.retrigger_cooldown + 0.3
    assert step(dc, t, (1.5, 1.0, math.pi/2)).state == 'rotating'


def test_scan_velho_aborta_crossing():
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 1.9, math.pi/2), fresh=False).state == 'idle'


def test_default_rot_speed_is_3():
    # 2026-06-19: 4.0 -> 3.0. As fitas nas rodas deram grip; a 4.0 o giro passava
    # do alvo (apontava pro batente). Teto do proporcional (rot_k/rot_min seguem).
    assert DoorCrossConfig().rot_speed == 3.0


def test_rotating_is_proportional_slows_near_target():
    # 2026-06-16 (3b40817), re-aplicado 2026-06-19: o giro no lugar era bang-bang
    # (sempre rot_speed) -> a ~11.5°/tick passava da janela de ±5° e ficava
    # caçando direita/esquerda ("doidinho na frente da porta", relato de campo).
    # O sentido-único do 1a0fe30 também girava a vel. cheia -> seguia caçando.
    # Agora é proporcional: teto longe, desacelera perto, piso pra não stallar.
    dc = mk()
    c = step(dc, 0.0, (1.5, 1.0, math.pi))               # arma -> rotating
    # LONGE do alvo (yaw_des=pi/2) -> velocidade no teto
    c = step(dc, 0.1, (1.5, 1.0, math.pi/2 - 1.0))
    assert c.state == 'rotating'
    assert abs(c.wz) == pytest.approx(CFG.rot_speed)
    # PERTO do alvo -> proporcional, mais devagar que o teto, mas >= piso.
    # Banda proporcional = err em (rot_min/rot_k, rot_speed/rot_k) = (0.417, 0.5)
    # com rot_speed=3, rot_k=6, rot_min=2.5 -> uso 0.45 (mag=2.7).
    c = step(dc, 0.2, (1.5, 1.0, math.pi/2 - 0.45))
    assert CFG.rot_min <= abs(c.wz) < CFG.rot_speed
    # MUITO perto (mas fora de align_yaw) -> piso (não para de girar)
    c = step(dc, 0.3, (1.5, 1.0, math.pi/2 - 0.1))
    assert abs(c.wz) == pytest.approx(CFG.rot_min)


def test_default_rot_k_e_rot_min():
    assert DoorCrossConfig().rot_k == 6.0
    assert DoorCrossConfig().rot_min == 2.5


from robot_nav.door_crossing import nav_engaging


def test_nav_engaging_true_when_rotating_or_forward():
    # girando pra alinhar (linear ~0) ou indo pra frente -> engajado (arma)
    assert nav_engaging(0.0, 0.02) is True
    assert nav_engaging(0.30, 0.02) is True
    # ruído de ré minúsculo dentro da banda ainda conta como engajado
    assert nav_engaging(-0.01, 0.02) is True


def test_nav_engaging_false_only_on_real_reverse():
    # ré sustentada (abaixo de -nav_move_lin) -> NÃO arma
    assert nav_engaging(-0.05, 0.02) is False


from robot_nav.door_crossing import nearest_door_in_zone


def test_nearest_door_in_zone_proximity_only():
    doors = [DOOR]                      # centro em (1.5, 2.0)
    # dentro da zona, mas de COSTAS pra porta (cone não importa aqui)
    d = nearest_door_in_zone((1.5, 1.0, -math.pi / 2), doors, zone_radius=1.2)
    assert d is not None and d['id'] == 1
    # fora da zona -> None
    assert nearest_door_in_zone((1.5, -1.0, 0.0), doors, zone_radius=1.2) is None
    # sem pose -> None
    assert nearest_door_in_zone(None, doors, zone_radius=1.2) is None


def test_nearest_door_in_zone_empty_list_is_none():
    assert nearest_door_in_zone((0.0, 0.0, 0.0), [], zone_radius=1.2) is None


def test_nearest_door_in_zone_picks_closest():
    doors = [DOOR, {'id': 2, 'a': [1.0, 5.0], 'b': [2.0, 5.0]}]  # centro (1.5,5)
    d = nearest_door_in_zone((1.5, 4.5, 0.0), doors, zone_radius=1.2)
    assert d is not None
    assert d['id'] == 2


# ---- ré de escape (2026-06-16) -----------------------------------------------

ECFG = DoorCrossConfig(zone_radius=1.2, stage_dist=0.6, align_timeout=15.0,
                       total_timeout=40.0)
P_STAGE = (1.5, 1.0, math.pi / 2)   # na zona, encarando a porta (centro 1.5,2.0)


def estep(dc, t, pose, front_gap=math.inf, rear_gap=math.inf,
          goal=True, nav=True, gap=math.inf, fresh=True, cleared=True):
    return dc.update(t, pose, [DOOR], goal, nav, gap, fresh, front_gap, rear_gap,
                     goal_succeeded=cleared)


def test_escape_reverse_on_front_block():
    dc = DoorCrossing(ECFG)
    assert estep(dc, 0.0, P_STAGE).state == 'rotating'        # arma -> rotating
    c = estep(dc, 0.1, P_STAGE, front_gap=0.10)               # parede perto -> ré
    assert c.state == 'reversing'
    assert c.vx < 0.0 and c.wz == pytest.approx(0.0)          # ré RETA, nunca arco


def test_escape_reverse_on_substuck_timeout():
    # substuck (parado sem progredir) só vale no staging (recuperação); no
    # rotating é giro no lugar e NÃO conta como travado.
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)            # arma -> rotating
    dc.state = 'staging'               # recuperação: staging
    dc._align_t0 = 0.0
    dc._align_anchor = (P_STAGE[0], P_STAGE[1])
    c = estep(dc, ECFG.escape_substuck_time + 0.1, P_STAGE)   # não progrediu -> ré
    assert c.state == 'reversing'


def test_escape_aborts_when_rear_blocked():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    # parede na frente E sem vão atrás -> não força, larga pro nav2/unstuck
    c = estep(dc, 0.1, P_STAGE, front_gap=0.10, rear_gap=0.05)
    assert c.state == 'idle'


def test_escape_target_capped_by_rear_gap():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    estep(dc, 0.1, P_STAGE, front_gap=0.10, rear_gap=0.25)
    # alvo = min(escape_reverse_dist, rear_gap - escape_rear_margin) = min(0.30,0.15)
    assert dc._esc_target == pytest.approx(0.15)


def test_reverse_returns_to_staging_after_distance():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    estep(dc, 0.1, P_STAGE, front_gap=0.10)                   # -> reversing (alvo 0.30)
    # recuou 0.4 m (afastou da porta, y caiu) -> volta pro staging
    c = estep(dc, 0.5, (1.5, 0.6, math.pi / 2))
    assert c.state == 'staging'
    assert dc._align_t0 == pytest.approx(0.5)   # relógio do substuck reiniciado


def test_reverse_returns_to_staging_if_rear_closes():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    estep(dc, 0.1, P_STAGE, front_gap=0.10)                   # -> reversing
    # algo entrou atrás no meio da ré -> para e volta pro staging
    c = estep(dc, 0.2, (1.5, 0.95, math.pi / 2), rear_gap=0.05)
    assert c.state == 'staging'
    assert c.vx == pytest.approx(0.0)


def test_reverse_that_does_not_move_aborts_instead_of_burning_the_timeout():
    """Campo 2026-09-06, porta 2 da arena: a ré saiu ZERADA (o linear_limit 0.0
    do PolygonFront vale pros dois sentidos) e o `reversing`, que só saía por
    deslocamento, ficou 35 s comandando ré parado até o total_timeout de 40 s —
    segurando prio 20 e pendurando as recoveries do nav2. Ré que não anda tem
    que largar o osso rápido."""
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    assert estep(dc, 0.1, P_STAGE, front_gap=0.10).state == 'reversing'
    # comandando ré, pose CONGELADA (é o que o collision zerando produz)
    assert estep(dc, 0.1 + ECFG.escape_stall_time, P_STAGE).state == 'reversing'
    c = estep(dc, 0.2 + ECFG.escape_stall_time, P_STAGE)
    assert c.state == 'idle'          # larga pro nav2, que sabe pivotar
    assert c.vx == pytest.approx(0.0) and c.wz == pytest.approx(0.0)


def test_reverse_that_moves_is_not_cut_by_the_stall_clock():
    """A ré NORMAL não pode morrer pelo relógio novo: enquanto o robô se desloca
    a âncora reseta, igual ao substuck do staging."""
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    estep(dc, 0.1, P_STAGE, front_gap=0.10)               # -> reversing (alvo 0.30)
    y, t = P_STAGE[1], 0.1
    # recua devagar, bem mais tempo que escape_stall_time, sempre progredindo
    for _ in range(int(ECFG.escape_stall_time / 0.5) + 4):
        t += 0.5
        y -= 0.06                                          # > align_progress_radius
        c = estep(dc, t, (P_STAGE[0], y, P_STAGE[2]))
        if c.state != 'reversing':
            break
    assert c.state == 'staging'        # terminou pela DISTÂNCIA, não pelo relógio
    assert P_STAGE[1] - y >= ECFG.escape_reverse_dist


def test_escape_max_count_then_abort():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    t = 0.1
    for _ in range(ECFG.escape_max_count):
        assert estep(dc, t, P_STAGE, front_gap=0.10).state == 'reversing'
        # completa a ré (recua bastante) -> staging
        assert estep(dc, t + 0.05, (1.5, 0.5, math.pi / 2)).state == 'staging'
        t += 0.2
    # estourou o nº de escapes -> próximo bloqueio aborta (larga pro unstuck)
    assert estep(dc, t, P_STAGE, front_gap=0.10).state == 'idle'


def test_moving_approach_does_not_trigger_substuck():
    # aproximação LEGÍTIMA no staging (recuperação): o robô se desloca a cada
    # tick -> a âncora de progresso reseta o relógio, NÃO dispara a ré.
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, (1.5, 1.0, math.pi / 2))   # arma -> rotating
    dc.state = 'staging'                        # recuperação: staging
    dc._align_t0 = 0.0
    dc._align_anchor = (1.5, 1.0)
    # caminha de 1.0 -> 1.35 em y, ao longo de 7 s (bem além do substuck de 5 s)
    t, y = 0.5, 1.0
    last = None
    while t <= 7.0:
        y = min(1.35, y + 0.03)
        last = estep(dc, t, (1.5, y, math.pi / 2))
        t += 0.5
    assert last.state != 'reversing'   # nunca deu ré de escape durante o avanço


def test_escape_from_rotating_on_front_block():
    dc = DoorCrossing(ECFG)
    stage_y = 2.0 - ECFG.stage_dist
    estep(dc, 0.0, (1.5, stage_y - 0.3, math.pi / 2))   # arma (staging)
    c = estep(dc, 0.1, (1.5, stage_y, math.pi / 2))     # chegou -> rotating
    assert c.state == 'rotating'
    c = estep(dc, 0.2, (1.5, stage_y, math.pi / 2), front_gap=0.10)  # parede perto
    assert c.state == 'reversing'
    assert c.wz == pytest.approx(0.0)                   # ré RETA, nunca arco


def test_no_substuck_escape_while_rotating():
    # 2026-06-16: girar parado pra alinhar NÃO é "estar travado". O substuck por
    # TEMPO não deve disparar a ré no rotating (senão a ré reta, com a traseira
    # apontada pra porta, parecia que o robô "entrava de ré na sala"). align_timeout
    # segue como rede de segurança; obstáculo real à frente ainda dispara.
    dc = DoorCrossing(ECFG)
    stage_y = 2.0 - ECFG.stage_dist
    yaw = math.pi / 2 - 0.3                              # 17° fora do eixo -> NÃO alinha
    estep(dc, 0.0, (1.5, stage_y - 0.3, yaw))           # arma (staging)
    c = estep(dc, 0.1, (1.5, stage_y, yaw))             # chegou -> rotating
    assert c.state == 'rotating'
    # girando parado por > substuck_time, frente livre -> NÃO pode dar ré
    c = estep(dc, ECFG.escape_substuck_time + 1.0, (1.5, stage_y, yaw))
    assert c.state == 'rotating'


# ---- trava "passo aqui?" geométrica com yaw (2026-06-22, pendência A) ---------

from robot_nav.door_crossing import will_clear

# DOOR tem vão 1.0 m -> half_width=0.5. Com robot_half_width=0.25, fit_margin=0.13
# a folga lateral útil (fit) = 0.5 - 0.25 - 0.13 = 0.12 m.
WC_DOOR = door_geometry((1.0, 2.0), (2.0, 2.0))


def test_will_clear_centered_and_straight_passes():
    assert will_clear(WC_DOOR, s=-1.0, d=0.0, yaw_err=0.0, side=+1,
                      robot_half_width=0.25, fit_margin=0.13) is True


def test_will_clear_angled_into_jamb_fails():
    # centrado AGORA, mas apontando 10° -> projetado 1 m à frente desvia
    # 1.0*tan(10°)=0.176 > fit 0.12 -> não passa (era a falha de campo: lat OK, yaw ruim)
    assert will_clear(WC_DOOR, s=-1.0, d=0.0, yaw_err=math.radians(10), side=+1,
                      robot_half_width=0.25, fit_margin=0.13) is False


def test_will_clear_lateral_offset_too_big_fails():
    assert will_clear(WC_DOOR, s=-0.2, d=0.20, yaw_err=0.0, side=+1,
                      robot_half_width=0.25, fit_margin=0.13) is False


def test_will_clear_past_jamb_always_passes():
    # s>=0 já passou do ponto mais estreito -> sempre "passa"
    assert will_clear(WC_DOOR, s=0.1, d=5.0, yaw_err=math.radians(40), side=+1,
                      robot_half_width=0.25, fit_margin=0.13) is True


def test_will_clear_yaw_can_compensate_offset():
    # offset d=+0.1 mas apontando de volta pro eixo: a 1 m a projeção fecha em ~0
    # -> PASSA. Prova que é projeção real, não só |d| nem só |yaw|.
    assert will_clear(WC_DOOR, s=-1.0, d=0.10, yaw_err=math.radians(5.71), side=+1,
                      robot_half_width=0.25, fit_margin=0.13) is True


def test_will_clear_side_minus_one():
    # aproximando de cima (side=-1): mesma projeção, sinal coerente
    assert will_clear(WC_DOOR, s=-1.0, d=0.0, yaw_err=math.radians(10), side=-1,
                      robot_half_width=0.25, fit_margin=0.13) is False


# ---- re-estágio quando "não passo" (2026-06-22) ------------------------------

def test_restage_when_aligned_but_wont_fit():
    # 2026-09-03, porta 2 da arena: ficar alternando rotating->reversing com
    # espaço de sobra antes da boca era desperdício. Se ainda há pista para o
    # staging corrigir o lateral andando, volta para staging em vez de dar ré.
    dc = DoorCrossing(CFG)
    estep(dc, 0.0, (1.8, 1.4, math.pi / 2))            # arma -> rotating (d=0.3)
    t, last = 0.1, None
    for _ in range(CFG.align_stable + 1):
        last = estep(dc, t, (1.8, 1.4, math.pi / 2))
        t += 0.05
    assert last.state == 'staging'
    assert last.vx == pytest.approx(0.0)
    assert last.wz == pytest.approx(0.0)


def test_restage_when_aligned_but_wont_fit_near_boca_still_reverses():
    # Perto demais da boca já não dá para "costurar" lateral andando: aqui a
    # ré segue sendo o último recurso seguro.
    dc = mk()
    _ate_crossing(dc)
    dc.state = 'rotating'
    dc._stable = CFG.align_stable - 1
    c = estep(dc, 1.0, (1.8, 1.8, math.pi / 2), rear_gap=3.0)  # s=-0.2, d=0.3
    assert c.state == 'reversing'
    assert c.wz == pytest.approx(0.0)


def test_crossing_restages_on_yaw_drift_before_jamb():
    # no meio da travessia (s<0) o yaw deriva e a projeção bate no batente ->
    # re-estágio em vez de raspar.
    dc = mk()
    _ate_crossing(dc)                                  # centrado, side=+1
    c = step(dc, 1.0, (1.8, 1.7, math.pi / 2))         # s=-0.3, d=0.3 > fit 0.20
    assert c.state == 'reversing'


def test_restage_gives_up_to_nav2_after_max_escapes():
    # esgotou as re-tentativas -> larga pro nav2 (idle), não fica eterno
    dc = mk()
    _ate_crossing(dc)
    dc._escape_count = CFG.escape_max_count            # já gastou todas
    c = step(dc, 1.0, (1.8, 1.7, math.pi / 2))         # s=-0.3, d=0.3 > fit 0.20
    assert c.state == 'idle'


def test_crossing_centered_still_crosses():
    # regressão: centrado e reto NÃO re-estagia (segue cruzando)
    dc = mk()
    t = _ate_crossing(dc)
    c = step(dc, t, (1.5, 1.9, math.pi / 2))
    assert c.state == 'crossing'


# ---- ponto de não-retorno / commit_s (capengada de campo 2026-06-22) ---------

def test_crossing_commits_in_final_stretch_despite_offset():
    # CAPENGADA: >metade do corpo no vão (s passou de commit_s) mas com offset
    # lateral residual que o will_clear reprovaria -> NÃO dá ré, COMITA pra frente.
    dc = mk()
    _ate_crossing(dc)                                  # centrado, side=+1
    # s=-0.1 (> commit_s=-0.15), d=0.3 (> fit 0.20, will_clear reprovaria)
    c = step(dc, 1.0, (1.8, 1.9, math.pi / 2))
    assert c.state == 'crossing'


def test_crossing_still_restages_before_commit_point():
    # regressão: ANTES do commit_s (s bem negativo) a trava segue valendo -> ré.
    dc = mk()
    _ate_crossing(dc)
    c = step(dc, 1.0, (1.8, 1.7, math.pi / 2))         # s=-0.3 (< commit_s), d=0.3
    assert c.state == 'reversing'


# ---- C: armar só DEPOIS do ponto pré-porta cumprido (campo 2026-06-22) --------

def test_nao_arma_sem_pre_porta_cumprido():
    # O BUG DE CAMPO: porta na zona, todos os gates ok, MAS o ponto pré-porta
    # ainda não foi cumprido -> a door NÃO assume (fica idle), deixa o nav2 levar.
    dc = mk()
    c = step(dc, 0.0, (1.5, 1.1, math.pi / 2), cleared=False)
    assert c.state == 'idle'


def test_pre_porta_cumprido_libera_e_arma():
    # goal do nav2 deu succeeded com o robô na zona (= pré-porta cumprido) -> arma
    dc = mk()
    c = step(dc, 0.0, (1.5, 1.1, math.pi / 2), cleared=True)
    assert c.state == 'rotating'
    assert DOOR['id'] in dc._cleared


def test_succeeded_fora_da_zona_nao_libera():
    dc = mk()
    # succeeded longe da porta (fora da zona) -> não libera nada
    step(dc, 0.0, (1.5, -1.5, math.pi / 2), cleared=True)   # dist 3.5 > zona
    assert DOOR['id'] not in dc._cleared
    # chega na zona SEM novo succeeded -> não arma
    c = step(dc, 0.1, (1.5, 1.1, math.pi / 2), cleared=False)
    assert c.state == 'idle'


def test_cleared_reseta_ao_cruzar():
    dc = mk()
    t = _ate_crossing(dc)
    assert DOOR['id'] in dc._cleared
    c = step(dc, t + 1.0, (1.5, 2.0 + CFG.exit_margin + 0.05, math.pi / 2),
             cleared=False)
    assert c.state == 'idle'
    assert DOOR['id'] not in dc._cleared        # cruzou -> exige pré-porta de novo


def test_cleared_reseta_ao_sair_da_zona():
    dc = mk()
    step(dc, 0.0, (1.5, 1.1, math.pi / 2), cleared=True)   # libera
    assert DOOR['id'] in dc._cleared
    step(dc, 0.1, (1.5, -1.5, 0.0), cleared=False)         # saiu da zona
    assert DOOR['id'] not in dc._cleared


# ---- não re-armar após cruzar / cooldown pós-travessia (campo 2026-06-22) -----

def test_no_rearm_right_after_crossing():
    # #2: depois de cruzar, a ré pós-porta trazia o robô de volta pra zona com a
    # porta na frente -> re-armava indevidamente (estava do outro lado!). Agora a
    # travessia bem-sucedida arma um cooldown -> NÃO re-arma logo em seguida.
    dc = mk()
    t = _ate_crossing(dc)
    c = step(dc, t + 1.0, (1.5, 2.0 + CFG.exit_margin + 0.05, math.pi / 2))
    assert c.state == 'idle'                            # cruzou e soltou
    # ré pós-porta: volta pra zona (s<0), porta na frente -> SEM o cooldown re-armaria
    c = step(dc, t + 1.2, (1.5, 1.7, math.pi / 2))
    assert c.state == 'idle'


def test_passou_nao_rearma_nem_com_o_cooldown_vencido():
    """2026-09-06, DECISÃO DO DONO: depois de PASSAR, não tenta de novo.

    Este teste dizia o contrário até hoje ("cooldown é temporário — re-aproximar
    re-arma normal"), e era o comportamento que o campo mostrou de perto: o robô
    passava a porta 2 e voltava por ela. Agora quem quer atravessar de novo tem
    que SAIR da zona e voltar (test_sair_da_zona_e_voltar_e_travessia_nova) —
    o cooldown vencido, sozinho, não basta."""
    dc = mk()
    t = _ate_crossing(dc)
    step(dc, t + 1.0, (1.5, 2.0 + CFG.exit_margin + 0.05, math.pi / 2))   # cruzou
    c = step(dc, t + 1.0 + CFG.crossing_cooldown + 0.5, (1.5, 1.4, math.pi / 2))
    assert c.state == 'idle'


# ---- profundidade do vão + pivô limitado (§2H.25/§2H.26) -------------------
# Toda a geometria daqui é MEDIDA, não estimada: robô 0,50 × 0,50 roda-a-roda,
# cone da arena R=0,17 (tools/gera_arena_galpao.py:28), larguras das 4 portas
# 0,90 / 0,70 / 0,60 / 0,80 m. Números conferidos na §2H.26 do DIARIO_ARENA.md.

from robot_nav.door_crossing import (          # noqa: E402
    entry_yaw_budget,
    exit_s_min,
    pivot_max_yaw,
    will_clear,
)


def test_door_geometry_depth_default_zero_e_retrocompativel():
    # porta sem `depth` declarado = parede fina, comportamento de hoje
    g = door_geometry((1.0, 2.0), (2.0, 2.0))
    assert g.depth == pytest.approx(0.0)


def test_door_geometry_aceita_depth():
    g = door_geometry((1.0, 2.0), (2.0, 2.0), depth=0.34)
    assert g.depth == pytest.approx(0.34)
    # profundidade NÃO mexe em centro, largura nem eixos
    assert (g.cx, g.cy) == pytest.approx((1.5, 2.0))
    assert g.half_width == pytest.approx(0.5)


def test_door_geometry_rejeita_depth_negativa():
    with pytest.raises(ValueError):
        door_geometry((1.0, 2.0), (2.0, 2.0), depth=-0.1)


# -- pivot_max_yaw: quanto o robô pode PIVOTAR dentro do vão -----------------

def test_pivot_max_yaw_vao_070_cabe_13_graus():
    # 0,25*(cos+sin) <= 0.70/2 - 0.05  ->  ~13,1°
    assert math.degrees(pivot_max_yaw(0.70)) == pytest.approx(13.1, abs=0.1)


def test_pivot_max_yaw_vao_060_nao_cabe_nem_parado():
    # meia-largura 0,30 - margem 0,05 = 0,25 = exatamente o meio-corpo
    assert pivot_max_yaw(0.60) == pytest.approx(0.0)


def test_pivot_max_yaw_vao_080():
    assert math.degrees(pivot_max_yaw(0.80)) == pytest.approx(36.9, abs=0.1)


def test_pivot_max_yaw_vao_largo_e_sem_limite():
    # se o envelope de 45° (meia-diagonal 0,354) já cabe, qualquer pivô cabe
    assert pivot_max_yaw(0.90) >= math.pi / 4
    assert pivot_max_yaw(1.20) >= math.pi / 4


def test_pivot_max_yaw_e_monotono_na_largura():
    larguras = (0.60, 0.70, 0.80, 0.90, 1.20)
    vals = [pivot_max_yaw(w) for w in larguras]
    assert vals == sorted(vals)


def test_pivot_max_yaw_vao_estreito_demais_e_zero_nao_negativo():
    assert pivot_max_yaw(0.30) == pytest.approx(0.0)
    assert pivot_max_yaw(0.0) == pytest.approx(0.0)


# -- entry_yaw_budget: erro de entrada tolerável SEM correção dentro ---------

def test_entry_yaw_budget_porta_fina_070_tolera_14_graus():
    assert math.degrees(entry_yaw_budget(0.70, 0.20)) == pytest.approx(14.0, abs=0.1)


def test_entry_yaw_budget_tunel_2m_070_exige_1_4_graus():
    assert math.degrees(entry_yaw_budget(0.70, 2.0)) == pytest.approx(1.4, abs=0.1)


def test_entry_yaw_budget_cai_com_a_profundidade():
    # mesma largura, mais fundo -> menos tolerância. Este é o teto FÍSICO.
    b = [entry_yaw_budget(0.80, d) for d in (0.2, 0.5, 1.0, 2.0)]
    assert b == sorted(b, reverse=True)


def test_entry_yaw_budget_vao_sem_folga_util_e_zero():
    # 0,60 m: 0,30 - 0,25 - 0,05 = 0 -> impossível em qualquer profundidade
    assert entry_yaw_budget(0.60, 0.0) == pytest.approx(0.0)
    assert entry_yaw_budget(0.60, 2.0) == pytest.approx(0.0)


def test_entry_yaw_budget_porta_de_espessura_zero_e_livre():
    # d=0: sem braço de alavanca, o yaw não desloca nada -> sem limite
    assert entry_yaw_budget(0.70, 0.0) >= math.pi / 4


# -- exit_s_min: onde é seguro SOLTAR o robô ---------------------------------

def test_exit_s_min_porta_de_cone_da_arena():
    # cones R=0,17 -> depth 0,34; meia-diagonal 0,354 + 0,17 = 0,524 m
    assert exit_s_min(0.34) == pytest.approx(0.524, abs=0.001)


def test_exit_s_min_prova_que_o_exit_margin_de_hoje_era_curto():
    # §2H.23: soltou em 0,50 e faltaram 2,4 cm pra um pivô caber
    assert exit_s_min(0.34) > 0.50
    assert (exit_s_min(0.34) - 0.50) == pytest.approx(0.024, abs=0.001)


def test_exit_s_min_cresce_com_a_profundidade():
    assert exit_s_min(2.0) == pytest.approx(1.0 + 0.354, abs=0.001)


def test_exit_s_min_parede_fina_e_meia_diagonal():
    assert exit_s_min(0.0) == pytest.approx(0.354, abs=0.001)


# -- will_clear com profundidade --------------------------------------------

def test_will_clear_depth_zero_nao_muda_nada():
    # regressão: com depth=0 a conta é a de hoje (projeta até s=0)
    g = door_geometry((0.0, 0.0), (0.0, 1.09))   # vão largo de 06-23
    assert will_clear(g, -0.5, 0.0, math.radians(2.0), +1, 0.25, 0.05) is True
    assert will_clear(g, 0.1, 0.30, math.radians(20.0), +1, 0.25, 0.05) is True


def test_will_clear_em_tunel_nao_libera_so_por_ter_passado_do_centro():
    # o bug da §2H.25: hoje `s >= 0` devolve True incondicional. Num túnel de
    # 1 m, s=+0.1 ainda está DENTRO e apontando pro batente -> tem que reprovar.
    g = door_geometry((0.0, 0.0), (0.0, 0.70), depth=1.0)
    assert will_clear(g, 0.1, 0.04, math.radians(25.0), +1, 0.25, 0.05) is False


def test_will_clear_em_tunel_libera_depois_da_boca_de_saida():
    g = door_geometry((0.0, 0.0), (0.0, 0.70), depth=1.0)
    assert will_clear(g, 0.6, 0.04, math.radians(25.0), +1, 0.25, 0.05) is True


# ---- fiação da ré de escape (2026-09-06) ------------------------------------
# Campo, porta 2 da arena: o `PolygonFront` é `limit` com `linear_limit: 0.0`, e
# o Nav2 aplica esse teto ao MÓDULO da velocidade linear — o sinal não importa.
# Com o batente na caixa frontal a ré da porta saía ZERADA, e como ela comanda
# `wz = 0.0` por desenho o robô ficava imóvel, sem nunca limpar a própria caixa.
# 35 s parado até o total_timeout. Ver docs/baselines/2026-09-06-porta2-deadlock-re/.
# Estes testes prendem a fiação que corrige isso — sem eles a correção some numa
# refatoração e o defeito volta calado.

_RAIZ = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
_PKG = os.path.join(_RAIZ, 'ros2_packages', 'robot_nav')


def _mux_final():
    with open(os.path.join(_PKG, 'config', 'twist_mux.yaml')) as f:
        return yaml.safe_load(f)['twist_mux']['ros__parameters']['topics']


def test_re_de_escape_tem_canal_no_mux_final():
    """A ré tem que sair A JUSANTE do collision_monitor. O mux FINAL é o único
    lugar onde isso acontece (é de lá que unstuck e humano já furam)."""
    t = _mux_final()
    canais = {v['topic']: v['priority'] for v in t.values()}
    assert 'door_escape_vel' in canais


def test_prioridade_da_re_fica_entre_o_unstuck_e_a_autonomia():
    """Abaixo do unstuck (30): um resgate de verdade ainda tem que vencer a ré da
    porta. Acima da autonomia (10): senão o nav2 anula a ré e nada mudou."""
    canais = {v['topic']: v['priority'] for v in _mux_final().values()}
    assert canais['auto_vel'] < canais['door_escape_vel'] < canais['unstuck_vel']


def test_so_a_re_fura_o_collision__o_avanco_da_porta_NAO():
    """O escopo é o ponto todo: `staging`/`rotating`/`crossing` são movimento pra
    FRENTE e continuam atrás do collision, que é o que impede atropelar alguém
    parado no vão. Se um dia o nó publicar outro estado no canal furador, ou
    deixar de zerar o door_vel durante a ré, este teste tem que cair."""
    with open(os.path.join(_PKG, 'robot_nav', 'door_crossing.py')) as f:
        src = f.read()
    i = src.index('if cmd.state == \'reversing\':')
    bloco = src[i:i + 700]
    # a ré vai pro canal furador...
    assert 'self.pub_escape.publish(t)' in bloco
    # ...e o door_vel fica ZERO (não mudo): segura a prio 20 do mux de autonomia
    assert 'self.pub.publish(Twist())' in bloco
    # o canal furador NÃO carrega o resto da travessia
    for st in ('staging', 'rotating', 'crossing'):
        assert f'cmd.state == \'{st}\'' not in bloco


def test_o_canal_da_re_nao_e_o_do_unstuck():
    """Reusar `unstuck_vel` funcionaria hoje (o unstuck fica em standdown na
    porta), mas mente no log e coloca dois publishers no mesmo tópico — quebra
    calado se o standdown mudar."""
    with open(os.path.join(_PKG, 'robot_nav', 'door_crossing.py')) as f:
        src = f.read()
    assert "'door_escape_vel'" in src
    assert 'unstuck_vel' not in src


# ---- "passou -> não passa de novo" (2026-09-06, campo) -----------------------
# O robô atravessava a porta 2 e, no waypoint seguinte, tentava atravessar de
# VOLTA por ela. Causa: o waypoint logo atrás da porta caía DENTRO da zona
# (0.598 m do centro, zona 1.1), então concluir esse goal contava como
# "pré-porta cumprido" e rearmava a travessia — agora ao contrário.

D2 = {'id': 2, 'a': [-6.2413, -5.2871], 'b': [-5.4960, -5.2871]}   # porta real
NORTE = (-5.8686, -4.4871, -math.pi / 2)   # pré-porta, encarando a porta
SUL = (-5.8786, -5.8851, -math.pi / 2)     # waypoint pós-porta DA ROTA (na zona)
LONGE = (-5.8786, -9.0, -math.pi / 2)      # fora da zona


def _passo(dc, t, pose, cleared=False):
    return dc.update(t, pose, [D2], True, True, math.inf, True,
                     math.inf, math.inf, goal_succeeded=cleared)


def test_depois_de_passar_nao_tenta_passar_de_novo():
    dc = DoorCrossing(DoorCrossConfig())
    _passo(dc, 0.0, NORTE, cleared=True)       # cumpriu o pré-porta -> pode armar
    assert dc._cleared == {2}
    _passo(dc, 1.0, SUL)                        # PASSOU (mudou de lado)
    assert 2 in dc._crossed and dc._cleared == set()
    # a manobra acaba (do jeito que for) e vem o waypoint seguinte
    dc.state, dc.door, dc.geom = 'idle', None, None
    # o goal do waypoint pós-porta conclui DENTRO da zona (0.598 m do centro, é o
    # da rota real): NÃO vale como pré-porta cumprido
    t = 2.0 + DoorCrossConfig().crossing_cooldown + 1.0     # cooldown já vencido
    c = _passo(dc, t, SUL, cleared=True)
    assert dc._cleared == set()
    assert c.state == 'idle'                    # não rearma pra voltar


def test_sair_da_zona_e_voltar_e_travessia_nova():
    """A trava é 'não passa DE NOVO agora', não 'nunca mais': quem sai da zona e
    volta com o pré-porta cumprido atravessa outra vez."""
    dc = DoorCrossing(DoorCrossConfig())
    _passo(dc, 0.0, NORTE, cleared=True)
    _passo(dc, 1.0, SUL)
    assert 2 in dc._crossed
    _passo(dc, 2.0, LONGE)                      # saiu da zona -> esquece tudo
    assert dc._crossed == set() and dc._cleared == set()
    _passo(dc, 3.0, SUL, cleared=True)          # volta e cumpre o pré-porta
    assert dc._cleared == {2}


def test_travessia_conta_mesmo_quando_quem_dirigiu_foi_o_nav2():
    """A porta 2 do campo foi atravessada pelo nav2 no braço, com o `crossing`
    abortado. Mesmo assim tem que contar como passada."""
    dc = DoorCrossing(DoorCrossConfig())
    _passo(dc, 0.0, NORTE, cleared=True)
    dc.state = 'idle'                            # a máquina fina nunca conduziu
    _passo(dc, 1.0, SUL)
    assert 2 in dc._crossed


def test_tremida_em_cima_do_vao_nao_conta_como_travessia():
    """Parado no meio do vão o sinal de `s` treme; sem deadband uma travessia
    viraria várias e a porta se trancaria sozinha no meio da manobra."""
    cfg = DoorCrossConfig()
    dc = DoorCrossing(cfg)
    _passo(dc, 0.0, NORTE, cleared=True)
    meio = cfg.side_deadband * 0.5
    for i, dy in enumerate((+meio, -meio, +meio, -meio)):
        _passo(dc, 1.0 + i, (-5.8686, -5.2871 + dy, -math.pi / 2))
    assert dc._crossed == set() and dc._cleared == {2}
