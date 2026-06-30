import math

import pytest

from robot_nav.unstuck_supervisor import (
    UnstuckConfig,
    UnstuckSupervisor,
    clearest_heading_offset,
    freer_side,
    front_min_gap,
    rear_min_gap,
    side_clearance,
)


# ---- helpers puros ---------------------------------------------------------

def _scan_with_obstacle_at(angle_rad, dist, n=360):
    # scan de -pi..pi, tudo livre (inf) menos um feixe no ângulo pedido
    angle_min = -math.pi
    angle_increment = 2 * math.pi / n
    ranges = [float("inf")] * n
    i = int(round((angle_rad - angle_min) / angle_increment)) % n
    ranges[i] = dist
    return ranges, angle_min, angle_increment


# Geometria real do robô (carcaça 50x50; LiDAR e demais sensores no CENTRO,
# confirmado pelo usuário 2026-06-11): o vão é medido do PARA-CHOQUE traseiro
# (tail_x), não do LiDAR.
GEO = dict(lidar_x=0.0, tail_x=-0.25, half_width=0.30)
# Espelho frontal: o vão de avanço é medido do PARA-CHOQUE dianteiro (head_x).
GEO_FRONT = dict(lidar_x=0.0, head_x=0.25, half_width=0.30)


def _scan_with_obstacle_at_base(x_b, y_b, n=360):
    # converte um ponto no frame base_link pro feixe equivalente do LiDAR
    dx, dy = x_b - GEO['lidar_x'], y_b
    return _scan_with_obstacle_at(math.atan2(dy, dx), math.hypot(dx, dy), n)


def test_rear_gap_measures_from_bumper():
    # feixe a 180°/0.60m do LiDAR (centro) → vão REAL de 0.35m atrás do
    # para-choque (o código antigo media do LiDAR: dizia "0.60 de folga")
    ranges, amin, ainc = _scan_with_obstacle_at(math.pi, 0.60)
    assert rear_min_gap(ranges, amin, ainc, **GEO) == pytest.approx(0.35, abs=0.02)


def test_rear_gap_inf_when_only_obstacle_in_front():
    ranges, amin, ainc = _scan_with_obstacle_at(0.0, 0.20)
    assert math.isinf(rear_min_gap(ranges, amin, ainc, **GEO))


def test_rear_gap_sees_corner_obstacle():
    # BUG DA BATIDA (2026-06-11): obstáculo na QUINA traseira (x=-0.30,
    # y=0.28) está a ~43° do eixo traseiro — o setor antigo de ±30° não via
    # e o robô deu ré em cima. O corredor retangular tem que ver (vão ~0.05m).
    ranges, amin, ainc = _scan_with_obstacle_at_base(-0.30, 0.28)
    assert rear_min_gap(ranges, amin, ainc, **GEO) == pytest.approx(0.05, abs=0.02)


def test_rear_gap_ignores_lateral_outside_corridor():
    # atrás mas 0.5m de lado: fora do corredor que o corpo varre na ré
    ranges, amin, ainc = _scan_with_obstacle_at_base(-0.50, 0.50)
    assert math.isinf(rear_min_gap(ranges, amin, ainc, **GEO))


def test_rear_gap_invalid_returns_are_free():
    ranges = [0.0] * 180 + [float('nan')] * 90 + [float('inf')] * 90
    assert math.isinf(rear_min_gap(ranges, -math.pi, 2 * math.pi / 360, **GEO))


def test_front_gap_measures_from_bumper():
    # feixe a 0°/0.60m do LiDAR (centro) → vão REAL de 0.35m à frente do
    # para-choque dianteiro (head_x=0.25)
    ranges, amin, ainc = _scan_with_obstacle_at(0.0, 0.60)
    assert front_min_gap(ranges, amin, ainc, **GEO_FRONT) == pytest.approx(0.35, abs=0.02)


def test_front_gap_inf_when_only_obstacle_behind():
    ranges, amin, ainc = _scan_with_obstacle_at(math.pi, 0.20)
    assert math.isinf(front_min_gap(ranges, amin, ainc, **GEO_FRONT))


def test_front_gap_sees_corner_obstacle():
    # quina DIANTEIRA (x=0.30, y=0.28): fora de um cone estreito, dentro do
    # corredor retangular que o corpo varre avançando (vão ~0.05m)
    ranges, amin, ainc = _scan_with_obstacle_at_base(0.30, 0.28)
    assert front_min_gap(ranges, amin, ainc, **GEO_FRONT) == pytest.approx(0.05, abs=0.02)


def test_front_gap_ignores_lateral_outside_corridor():
    ranges, amin, ainc = _scan_with_obstacle_at_base(0.50, 0.50)
    assert math.isinf(front_min_gap(ranges, amin, ainc, **GEO_FRONT))


def test_front_gap_invalid_returns_are_free():
    ranges = [0.0] * 180 + [float('nan')] * 90 + [float('inf')] * 90
    assert math.isinf(front_min_gap(ranges, -math.pi, 2 * math.pi / 360, **GEO_FRONT))


def test_freer_side_left_when_obstacle_on_right():
    # obstáculo a 45° à DIREITA -> esquerda mais livre -> +1
    ranges, amin, ainc = _scan_with_obstacle_at(-math.pi / 4, 0.30)
    assert freer_side(ranges, amin, ainc) == 1


def test_freer_side_right_when_obstacle_on_left():
    ranges, amin, ainc = _scan_with_obstacle_at(math.pi / 4, 0.30)
    assert freer_side(ranges, amin, ainc) == -1


# ---- máquina de estados ----------------------------------------------------

def _cfg(**kw):
    base = dict(
        stuck_timeout=10.0,
        stuck_radius=0.05,
        reverse_distance=0.30,
        reverse_speed=0.25,
        reverse_time_cap=6.0,
        grace=2.0,
        nav_latch=15.0,
        escalate_after=3,
        same_spot_radius=0.5,
        escalate_window=120.0,
        spin_speed=3.0,
        spin_angle=0.44,
        spin_time_cap=4.0,
        spin_left_boost=1.0,
        reverse_min=0.10,
        rear_stop_margin=0.10,
        forward_distance=0.20,
        forward_speed=0.15,
        forward_time_cap=6.0,
        front_stop_margin=0.10,
        forward_min=0.10,
    )
    base.update(kw)
    return UnstuckConfig(**base)


def _tick(sup, t, pos=(0.0, 0.0), nav=True, gap=math.inf, front_gap=0.0,
          near=0.0):
    # front_gap=0.0 (frente BLOQUEADA) por padrão: sob a Opção A (2026-06-28) a
    # recovery só dispara com obstáculo à frente; os testes de recovery assumem isso.
    # O teste do PORTÃO (frente livre -> suprime) passa front_gap=inf explicitamente.
    # near=0.0 (PINÇADO) por padrão: sem folga pra girar -> recovery cai na RÉ (o
    # contrato dos testes de ré). Os testes de GIRO passam near alto explicitamente.
    return sup.update(t, nav_wants_move=nav, position=pos, rear_gap=gap,
                      front_gap=front_gap, nearest=near)


def test_no_action_before_timeout():
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0)
    cmd = _tick(sup, 9.9)
    assert cmd.active is False


def test_reverses_after_timeout_when_not_displacing():
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0)
    cmd = _tick(sup, 10.1)
    assert cmd.active is True
    assert cmd.lin == pytest.approx(-0.25)
    assert cmd.ang == pytest.approx(0.0)


def test_point_turn_does_not_trigger_reverse():
    # BO 06-27: robô girando no lugar (yaw muda, posição NÃO) = progresso ->
    # NÃO pode dar ré. Antes, o point-turn legítimo do path_follower disparava
    # o unstuck (que só media deslocamento) e fodia o nav2.
    sup = UnstuckSupervisor(_cfg())
    yaw = 0.0
    cmd = None
    for i in range(40):                      # 20 s girando no lugar (FRENTE bloqueada)
        yaw += 0.2                           # ~11°/tick > stuck_yaw (0.15)
        cmd = sup.update(i * 0.5, nav_wants_move=True, position=(0.0, 0.0),
                         rear_gap=math.inf, front_gap=0.0, yaw=yaw)
    assert cmd.active is False               # nunca disparou a manobra


def test_frozen_yaw_still_reverses():
    # comanda mas nem desloca nem gira (yaw congelado) E frente bloqueada = travado
    # DE VERDADE -> ré.
    sup = UnstuckSupervisor(_cfg())
    sup.update(0.0, nav_wants_move=True, position=(0.0, 0.0), front_gap=0.0,
               yaw=0.0, nearest=0.0)
    cmd = sup.update(10.1, nav_wants_move=True, position=(0.0, 0.0),
                     rear_gap=math.inf, front_gap=0.0, yaw=0.0, nearest=0.0)
    assert cmd.active is True
    assert cmd.lin == pytest.approx(-0.25)   # pinçado (sem folga p/ girar) -> ré


def test_front_clear_defers_then_fires():
    # OPÇÃO A (refinada 06-28): frente LIVRE -> DEFERE a recovery (dá tempo pro nav),
    # mas não pra sempre: passou de front_clear_timeout travado mesmo assim (bloqueio
    # lateral/no giro que o front reto não vê) -> age (senão ficaria preso eterno).
    sup = UnstuckSupervisor(_cfg())
    sup.update(0.0, nav_wants_move=True, position=(0.0, 0.0), front_gap=math.inf, yaw=0.0)
    cmd = sup.update(12.0, nav_wants_move=True, position=(0.0, 0.0),
                     rear_gap=math.inf, front_gap=math.inf, yaw=0.0)
    assert cmd.active is False               # 12s < front_clear_timeout(15) -> defere
    cmd = sup.update(16.0, nav_wants_move=True, position=(0.0, 0.0),
                     rear_gap=math.inf, front_gap=math.inf, yaw=0.0)
    assert cmd.active is True                # 16s > 15 -> age
    # frente LIVRE -> AVANÇA (passa o batente), NÃO ré (ré seria loop). Mesmo com
    # vão atrás (rear=inf), a direção certa é pra frente quando a frente é o caminho.
    assert cmd.lin == pytest.approx(0.15)
    assert cmd.ang == pytest.approx(0.0)


def test_lateral_pinch_fires_fast_when_map_misses_wall():
    # 2026-06-28: near_mapped FALSO (offset de registro AMCL<->mapa perde a parede)
    # + frente livre -> ANTES esperava os 15s cautelosos à toa num mapa conhecido.
    # Espremido de LADO (side_clear=0.05, do LiDAR) = encurralado -> age RÁPIDO
    # (~stuck_timeout_mapped 2s), sem o defer. O "ABERTO de verdade" (test acima)
    # segue cauteloso.
    sup = UnstuckSupervisor(_cfg())   # stuck_timeout=10, front_clear_timeout=15
    sup.update(0.0, nav_wants_move=True, position=(0.0, 0.0), front_gap=2.0,
               side_clear=0.05, near_mapped=False, nearest=0.0, yaw=0.0)
    assert sup.update(1.9, nav_wants_move=True, position=(0.0, 0.0), front_gap=2.0,
                      side_clear=0.05, near_mapped=False, nearest=0.0).active is False
    cmd = sup.update(2.2, nav_wants_move=True, position=(0.0, 0.0), front_gap=2.0,
                     side_clear=0.05, near_mapped=False, nearest=0.0)
    assert cmd.active is True                 # agiu em ~2s (era ~15s)
    assert cmd.lin == pytest.approx(0.15)     # frente livre -> avança


def test_known_obstacle_acts_faster():
    # "CONHEÇO esse obstáculo?" (06-28): parede MAPEADA perto (near_mapped, ex. batente)
    # -> age em ~3s (front_clear_timeout_mapped), não espera os 15s do desconhecido.
    sup = UnstuckSupervisor(_cfg())
    sup.update(0.0, nav_wants_move=True, position=(0.0, 0.0), front_gap=math.inf,
               near_mapped=True, yaw=0.0)
    cmd = sup.update(2.5, nav_wants_move=True, position=(0.0, 0.0),
                     rear_gap=math.inf, front_gap=math.inf, near_mapped=True, yaw=0.0)
    assert cmd.active is False               # 2.5s < 3 (mapped) -> ainda defere
    cmd = sup.update(3.5, nav_wants_move=True, position=(0.0, 0.0),
                     rear_gap=math.inf, front_gap=math.inf, near_mapped=True, yaw=0.0)
    assert cmd.active is True                # 3.5s > 3 -> JÁ age (vs 15s do desconhecido)
    assert cmd.lin == pytest.approx(0.15)    # frente livre -> avança


def test_door_active_suppresses_maneuver():
    # door_crossing (door_vel, prio 20) está conduzindo a travessia; o unstuck
    # (unstuck_vel, prio 30) SOBREPÕE e sabotava a manobra -> door_crossing
    # nunca alinhava e abortava em loop ("5 min na porta", campo 2026-06-15).
    # Com door_active a manobra fica SUPRIMIDA mesmo travado além do timeout.
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0)
    cmd = sup.update(10.1, nav_wants_move=True, position=(0.0, 0.0),
                     rear_gap=math.inf, front_gap=math.inf, door_active=True)
    assert cmd.active is False


def test_unstuck_acts_again_after_door_clears():
    # quando a porta sai (door_crossing solta/aborta), o unstuck re-ancora e
    # volta a poder agir se continuar travado.
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0)
    sup.update(10.1, nav_wants_move=True, position=(0.0, 0.0), door_active=True)
    sup.update(10.2, nav_wants_move=True, position=(0.0, 0.0), door_active=False)
    cmd = sup.update(20.4, nav_wants_move=True, position=(0.0, 0.0),
                     rear_gap=math.inf, front_gap=0.0, door_active=False,
                     nearest=0.0)
    assert cmd.active is True
    assert cmd.lin == pytest.approx(-0.25)   # pinçado -> ré


def test_never_spins():
    # GIRO REMOVIDO (decisão 2026-06-10): a manobra é SEMPRE ré, nunca angular.
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0)
    for dt in (10.1, 10.2, 11.0, 12.0):
        cmd = _tick(sup, dt)
        assert cmd.ang == pytest.approx(0.0)


def test_micro_wiggle_still_fires():
    # BUG REAL: robô "tentando girar" sem sair do lugar (RotationShim/recoveries
    # do nav2) mexia uns mm e resetava o relógio. Deslocamento <stuck_radius
    # NÃO pode resetar — tem que disparar a ré mesmo assim.
    sup = UnstuckSupervisor(_cfg())
    fired = False
    t = 0.0
    while t <= 12.0:
        wiggle = (0.02 * math.sin(t * 7.0), 0.015 * math.cos(t * 5.0))
        cmd = _tick(sup, t, pos=wiggle)
        if cmd.active:
            fired = True
            break
        t += 0.1
    assert fired is True
    assert t <= 11.0


def test_robot_actually_moving_never_fires():
    # robô andando de verdade (>stuck_radius continuamente) nunca dispara
    sup = UnstuckSupervisor(_cfg())
    t = 0.0
    while t <= 20.0:
        cmd = _tick(sup, t, pos=(0.1 * t, 0.0))  # 0.1 m/s
        assert cmd.active is False
        t += 0.5


def test_no_fire_without_nav_goal():
    # sem o nav2 comandar (goal cancelado/atingido), parado NÃO é travado
    sup = UnstuckSupervisor(_cfg())
    t = 0.0
    while t <= 20.0:
        cmd = _tick(sup, t, nav=False)
        assert cmd.active is False
        t += 0.5


def test_nav_latch_tolerates_abort_gaps():
    # nav2 aborta e fica ~1-2s sem comandar; o latch segura e a ré ainda sai
    sup = UnstuckSupervisor(_cfg())
    fired = False
    t = 0.0
    while t <= 12.0:
        nav = (t % 9.0) < 8.0  # comanda 8s, cala 1s, repete
        cmd = _tick(sup, t, nav=nav)
        if cmd.active:
            fired = True
            break
        t += 0.5
    assert fired is True


def test_goal_gone_stops_firing():
    # nav silencia de vez (goal cancelado) -> depois do nav_latch, nada dispara
    sup = UnstuckSupervisor(_cfg(nav_latch=5.0))
    _tick(sup, 0.0, nav=True)
    t = 0.5
    while t <= 30.0:
        cmd = _tick(sup, t, nav=False)
        if t > 5.0:
            assert cmd.active is False, f"disparou em t={t} sem goal"
        t += 0.5


def test_boxed_in_both_sides_holds_then_reverses_when_rear_clears():
    # encurralado dos DOIS lados (traseira E frente bloqueadas): segura. Quando
    # a traseira libera, a ré é a manobra preferida (frente segue bloqueada).
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0, gap=0.12, front_gap=0.12)
    # vão 0.12 dos dois lados: alvo útil = 0.12-0.10 = 0.02 < min -> segura
    cmd = _tick(sup, 10.1, gap=0.12, front_gap=0.12)
    assert cmd.active is False  # boxed in: não ré, não avança, não gira
    cmd = _tick(sup, 10.5, gap=math.inf, front_gap=0.12)  # traseira liberou -> ré
    assert cmd.active is True
    assert cmd.lin == pytest.approx(-0.25)


def test_front_clear_suppresses_even_with_rear_blocked():
    # OPÇÃO A (2026-06-28): SUPERA o pedido de 2026-06-15 (avançar com frente livre).
    # Se a frente está LIVRE, a parada não é obstáculo -> NÃO intervém (nem ré nem
    # avanço); o path_follower/nav é que dirige pra frente. (Antes: avançava.)
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0, gap=0.05, front_gap=math.inf)
    cmd = _tick(sup, 10.1, gap=0.05, front_gap=math.inf)
    assert cmd.active is False              # frente livre -> não mexe


def test_reverse_preferred_when_rear_clear_even_if_front_blocked():
    # obstáculo na FRENTE (frente bloqueada) e traseira livre -> ré (caso comum,
    # como sempre foi). Ré é a manobra preferida quando há vão atrás.
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0, gap=math.inf, front_gap=0.05)
    cmd = _tick(sup, 10.1, gap=math.inf, front_gap=0.05)
    assert cmd.active is True
    assert cmd.lin == pytest.approx(-0.25)  # ré


def test_partial_advance_when_front_gap_limited():
    # frente apertada (0.28): avanço PARCIAL até (vão-margem)=0.18, não os 0.20
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0, gap=0.05, front_gap=0.28)
    cmd = _tick(sup, 10.1, gap=0.05, front_gap=0.28)
    assert cmd.active is True and cmd.lin == pytest.approx(0.15)
    cmd = _tick(sup, 10.5, pos=(0.10, 0.0), gap=0.05, front_gap=0.28)  # não chegou
    assert cmd.lin == pytest.approx(0.15)
    cmd = _tick(sup, 11.0, pos=(0.19, 0.0), gap=0.05, front_gap=0.28)  # >=0.18 -> STOP
    assert cmd.active is True and cmd.lin == pytest.approx(0.0)


def test_advance_aborts_when_obstacle_enters_front():
    # algo entra/aparece na FRENTE durante o avanço -> STOP imediato (respeita
    # o collision via front_gap: nunca avança em cima de obstáculo)
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0, gap=0.05, front_gap=0.35)
    cmd = _tick(sup, 10.1, gap=0.05, front_gap=0.35)  # frente parcial -> dispara avanço
    assert cmd.lin == pytest.approx(0.15)
    cmd = _tick(sup, 10.6, pos=(0.05, 0.0), gap=0.05, front_gap=0.08)  # entrou algo
    assert cmd.active is True and cmd.lin == pytest.approx(0.0)  # STOP
    assert cmd.ang == pytest.approx(0.0)
    cmd = _tick(sup, 10.8, pos=(0.05, 0.0), gap=0.05, front_gap=0.08)
    assert cmd.active is False   # grace, canal solto


def test_advance_ends_with_explicit_stop():
    # fim do avanço manda ZERO explícito (mesmo motivo da ré: cmd_vel_to_wheels
    # segura o último comando)
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0, gap=0.05, front_gap=0.35)
    _tick(sup, 10.1, gap=0.05, front_gap=0.35)  # entra em AVANÇO (frente parcial)
    cmd = _tick(sup, 11.0, pos=(0.20, 0.0), gap=0.05, front_gap=0.35)  # completou
    assert cmd.active is True and cmd.lin == pytest.approx(0.0)
    cmd = _tick(sup, 11.2, pos=(0.20, 0.0), gap=0.05, front_gap=0.35)
    assert cmd.active is False   # grace


def test_advance_stops_after_time_cap():
    sup = UnstuckSupervisor(_cfg(forward_time_cap=6.0))
    _tick(sup, 0.0, gap=0.05, front_gap=0.35)
    _tick(sup, 10.1, gap=0.05, front_gap=0.35)  # entra em AVANÇO (frente parcial)
    cmd = _tick(sup, 16.3, pos=(0.0, 0.0), gap=0.05, front_gap=0.35)  # cap sem andar
    assert cmd.active is True and cmd.lin == pytest.approx(0.0)  # STOP explícito


# ---- avanço ADAPTATIVO (2026-06-28): folga lateral abre -> saiu do pinch ------

def test_side_clearance_measures_tightest_side():
    # obstáculo a 0.40 m do centro do lado ESQUERDO, dentro da faixa do corpo
    # -> folga = 0.40 - half_width(0.30) = 0.10. Nada à direita -> esse é o min.
    ranges, amin, ainc = _scan_with_obstacle_at_base(0.0, 0.40)
    c = side_clearance(ranges, amin, ainc, lidar_x=0.0, x_lo=-0.25, x_hi=0.25,
                       half_width=0.30)
    assert c == pytest.approx(0.10, abs=0.02)


def test_side_clearance_ignores_returns_outside_body_band():
    # retorno à FRENTE (x≈0.5, fora da faixa [-0.25,0.25]) não é pinch lateral
    ranges, amin, ainc = _scan_with_obstacle_at_base(0.5, 0.40)
    c = side_clearance(ranges, amin, ainc, lidar_x=0.0, x_lo=-0.25, x_hi=0.25,
                       half_width=0.30)
    assert c == math.inf


def _adv_tick(sup, t, pos, side, front_gap=math.inf, near=True):
    # avanço acontece com FRENTE LIVRE (front_gap>front_clear); near_mapped=True
    # pra disparar rápido (~3s) sem esperar os 15s do defer. rear bloqueado pra
    # não preferir ré.
    return sup.update(t, nav_wants_move=True, position=pos, rear_gap=0.05,
                      front_gap=front_gap, near_mapped=near, side_clear=side)


def test_adaptive_advance_extends_while_pinch_tight():
    # frente livre + pinch lateral apertado (0.10): avança ALÉM do nudge fixo
    # (0.20) e só para quando a folga lateral ABRE (saiu do batente).
    sup = UnstuckSupervisor(_cfg())
    _adv_tick(sup, 0.0, (0.0, 0.0), side=0.10)          # ancora
    cmd = _adv_tick(sup, 3.1, (0.0, 0.0), side=0.10)    # near_mapped -> dispara avanço
    assert cmd.active and cmd.lin == pytest.approx(0.15)
    # passou do nudge 0.20 mas pinch ainda apertado -> CONTINUA (antes parava aqui)
    cmd = _adv_tick(sup, 3.5, (0.30, 0.0), side=0.10)
    assert cmd.lin == pytest.approx(0.15)
    # folga lateral abriu (0.30 >= 0.10+0.15) -> STOP (saiu do obstáculo)
    cmd = _adv_tick(sup, 3.9, (0.40, 0.0), side=0.30)
    assert cmd.active and cmd.lin == pytest.approx(0.0)


def test_adaptive_advance_only_nudge_when_no_pinch():
    # sem aperto lateral no início (0.50 >= side_open 0.40): só o nudge (0.20),
    # comportamento antigo — não estende à toa.
    sup = UnstuckSupervisor(_cfg())
    _adv_tick(sup, 0.0, (0.0, 0.0), side=0.50)
    cmd = _adv_tick(sup, 3.1, (0.0, 0.0), side=0.50)
    assert cmd.lin == pytest.approx(0.15)
    cmd = _adv_tick(sup, 3.4, (0.15, 0.0), side=0.50)   # antes do nudge -> continua
    assert cmd.lin == pytest.approx(0.15)
    cmd = _adv_tick(sup, 3.7, (0.20, 0.0), side=0.50)   # nudge feito + sem pinch -> STOP
    assert cmd.active and cmd.lin == pytest.approx(0.0)


def test_adaptive_advance_capped_when_pinch_never_opens():
    # pinch nunca abre -> para no teto forward_distance_max (0.6), nunca infinito.
    sup = UnstuckSupervisor(_cfg())
    _adv_tick(sup, 0.0, (0.0, 0.0), side=0.10)
    _adv_tick(sup, 3.1, (0.0, 0.0), side=0.10)
    cmd = _adv_tick(sup, 4.0, (0.45, 0.0), side=0.10)   # < teto -> continua
    assert cmd.lin == pytest.approx(0.15)
    cmd = _adv_tick(sup, 4.5, (0.60, 0.0), side=0.10)   # >= teto 0.6 -> STOP
    assert cmd.active and cmd.lin == pytest.approx(0.0)


def test_partial_reverse_when_gap_limited():
    # vão de 0.28 < reverse_distance+margem: ré PARCIAL até 0.18 (vão-margem),
    # não os 0.30 de sempre — encurralado recua o que DÁ, sem bater.
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0, gap=0.28)
    cmd = _tick(sup, 10.1, gap=0.28)
    assert cmd.active is True and cmd.lin == pytest.approx(-0.25)
    cmd = _tick(sup, 10.5, pos=(-0.10, 0.0), gap=0.28)   # ainda não chegou
    assert cmd.lin == pytest.approx(-0.25)
    cmd = _tick(sup, 11.0, pos=(-0.19, 0.0), gap=0.28)   # >= alvo 0.18 -> STOP
    assert cmd.active is True and cmd.lin == pytest.approx(0.0)


def test_reverse_aborts_when_obstacle_enters_behind():
    # BUG DA BATIDA (2026-06-11): durante a ré ninguém olhava mais o scan e o
    # robô recuou em cima do obstáculo. Vão <= margem no MEIO da manobra tem
    # que dar STOP imediato (e sem giro — tem coisa colada atrás).
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0)
    cmd = _tick(sup, 10.1)                                # dispara, tudo livre
    assert cmd.lin == pytest.approx(-0.25)
    cmd = _tick(sup, 10.6, pos=(-0.05, 0.0), gap=0.08)    # entrou algo atrás
    assert cmd.active is True and cmd.lin == pytest.approx(0.0)  # STOP
    assert cmd.ang == pytest.approx(0.0)
    cmd = _tick(sup, 10.8, pos=(-0.05, 0.0), gap=0.08)
    assert cmd.active is False                            # grace, canal solto


def test_reverse_ends_with_explicit_stop():
    # FIM DA RÉ TEM QUE MANDAR ZERO: o cmd_vel_to_wheels segura o último
    # comando; sem um Twist 0 explícito o robô continuaria de ré até alguém
    # publicar de novo (nav2 pode estar mudo, abortado).
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0)
    _tick(sup, 10.1)  # entra em RÉ na origem
    cmd = _tick(sup, 11.0, pos=(-0.30, 0.0))  # completou a distância
    assert cmd.active is True   # ainda publica...
    assert cmd.lin == pytest.approx(0.0)  # ...mas é o STOP
    assert cmd.ang == pytest.approx(0.0)
    cmd = _tick(sup, 11.2, pos=(-0.30, 0.0))  # grace: solta o canal
    assert cmd.active is False


def test_reverse_stops_after_time_cap():
    sup = UnstuckSupervisor(_cfg(reverse_time_cap=6.0))
    _tick(sup, 0.0)
    _tick(sup, 10.1)  # entra em RÉ
    cmd = _tick(sup, 16.3, pos=(0.0, 0.0))  # passou do cap sem recuar
    assert cmd.active is True and cmd.lin == pytest.approx(0.0)  # STOP explícito
    cmd = _tick(sup, 16.5, pos=(0.0, 0.0))
    assert cmd.active is False


def test_grace_before_rearming():
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0)
    _tick(sup, 10.1)  # RÉ
    _tick(sup, 11.0, pos=(-0.30, 0.0))  # completou -> STOP -> grace
    cmd = _tick(sup, 11.5, pos=(-0.30, 0.0))
    assert cmd.active is False  # ainda no grace


# ---- gate por STATUS do goal (autoritativo quando disponível) ---------------

def test_goal_inactive_blocks_fire_even_with_nav_msgs():
    # goal cancelado/atingido (status diz INATIVO) -> NUNCA dá ré póstuma,
    # mesmo que o flag de nav_vel_raw tenha ficado True pra trás
    sup = UnstuckSupervisor(_cfg())
    t = 0.0
    while t <= 25.0:
        cmd = sup.update(t, nav_wants_move=True, position=(0.0, 0.0),
                         rear_gap=math.inf, goal_active=False)
        assert cmd.active is False
        t += 0.5


def test_goal_active_fires_even_with_controller_silent():
    # BT em recovery (controller mudo, nav_vel_raw sem msg) mas goal ATIVO:
    # robô sem se deslocar 10s -> ré mesmo assim
    sup = UnstuckSupervisor(_cfg())
    fired = False
    t = 0.0
    while t <= 12.0:
        cmd = sup.update(t, nav_wants_move=False, position=(0.0, 0.0),
                         rear_gap=math.inf, front_gap=0.0, goal_active=True)
        if cmd.active:
            fired = True
            break
        t += 0.5
    assert fired is True


def test_goal_status_none_falls_back_to_latch():
    # sem status (tópico não visto): comportamento antigo via nav_latch
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0)
    cmd = _tick(sup, 10.1)
    assert cmd.active is True


# ---- escalada: 3 travamentos no mesmo ponto -> ré + GIRO FORTE --------------

def _stuck_cycle(sup, t0, pos, open_side=1, near=0.0):
    """Um ciclo completo: arma -> manobra inteira -> sai do grace.

    near=0.0 (PINÇADO) por padrão -> a manobra é RÉ (sem folga pra girar).
    Retorna (lista de comandos ativos do ciclo, t_pronto_pro_próximo).
    """
    cmds = []
    t = t0
    # arma e roda até voltar pra MONITORING com grace vencido (posição parada)
    deadline = t0 + 60.0
    fired = False
    while t < deadline:
        cmd = sup.update(t, nav_wants_move=True, position=pos,
                         rear_gap=math.inf, front_gap=0.0, open_side=open_side,
                         nearest=near)
        if cmd.active:
            fired = True
            cmds.append(cmd)
        elif fired and sup.state == "monitoring":
            break  # manobra acabou e o grace venceu
        t += 0.1
    return cmds, t + 0.1


def test_spin_only_as_last_resort_when_boxed():
    # GIRO = ÚLTIMO RECURSO (2026-06-28): só ENCURRALADO (sem ré nem avanço) e com
    # folga lateral -> gira. NÃO preempta ir reto/dar ré (o dono: "ta priorizando
    # sempre o giro... mesmo podendo ir reto"). Giro FORTE no lugar.
    sup, _, cmd = _spin_sup()
    assert cmd.lin == pytest.approx(0.0)       # giro é NO LUGAR
    assert cmd.ang == pytest.approx(3.0)       # forte


def test_no_spin_when_reverse_possible():
    # traseira ABERTA (dá ré) -> NÃO gira, dá ré (prioriza sair, não girar parado).
    # Era o bug: girava parado mesmo com vao_re=8.4. front bloqueado + folga lateral.
    sup = UnstuckSupervisor(_cfg(grace=0.5))
    t = 0.0
    cmd = None
    while t < 60.0:
        cmd = sup.update(t, nav_wants_move=True, position=(0.0, 0.0),
                         rear_gap=math.inf, front_gap=0.0, nearest=1.0,
                         nearest_deg=-126.0, yaw=0.0)
        if cmd.active:
            break
        t += 0.1
    assert cmd.lin == pytest.approx(-0.25) and cmd.ang == pytest.approx(0.0)  # ré, não giro


def test_spin_turns_toward_open_side():
    # obstáculo ~reto à frente (near_deg=0, ambíguo) -> usa o freer_side
    _, _, cmd = _spin_sup(open_side=-1)
    assert cmd.ang == pytest.approx(-3.0)       # gira pro lado mais livre (direita)


def test_spin_turns_away_from_nearest_obstacle():
    # LADO ERRADO (2026-06-28): obstáculo claramente à DIREITA (near_deg=-126°,
    # ex. traseira-direita) -> gira pra ESQUERDA (longe dele), IGNORANDO o
    # freer_side (que só via os setores frontais e mandava pro lado errado).
    _, _, cmd = _spin_sup(open_side=-1, near_deg=-126.0)
    assert cmd.ang == pytest.approx(3.0)        # esquerda (oposto ao obstáculo)
    # espelho: obstáculo à esquerda -> gira pra direita
    _, _, cmd = _spin_sup(open_side=1, near_deg=120.0)
    assert cmd.ang == pytest.approx(-3.0)


def _spin_sup(open_side=1, near_deg=0.0, **cfg_kw):
    """Leva ao 1º comando de GIRO (ENCURRALADO: frente E traseira bloqueadas, mas
    com folga lateral pra girar -> giro é a única saída = último recurso)."""
    sup = UnstuckSupervisor(_cfg(grace=0.5, **cfg_kw))
    deadline, t = 60.0, 0.0
    while t < deadline:
        cmd = sup.update(t, nav_wants_move=True, position=(0.0, 0.0),
                         rear_gap=0.05, front_gap=0.0, open_side=open_side,
                         yaw=0.0, nearest=1.0,  # nearest>=spin_clear -> há folga
                         nearest_deg=near_deg)
        if cmd.active and cmd.ang != 0.0:
            return sup, t, cmd
        t += 0.1
    raise AssertionError("nunca chegou no giro")


def test_spin_closed_loop_stops_at_target_yaw():
    # MALHA FECHADA: roda patina (comanda 30°, vira 5°) -> só para quando o
    # YAW MEDIDO (IMU) acumular spin_angle, não por tempo
    sup, t, cmd = _spin_sup()
    yaw = 0.0
    ticks = 0
    while cmd.ang != 0.0 and ticks < 200:
        yaw += cmd.ang * 0.1 * 0.3  # patinagem feia: só 30% do comandado vira
        t += 0.1
        cmd = sup.update(t, nav_wants_move=True, position=(0.0, 0.0),
                         rear_gap=math.inf, yaw=yaw)
        ticks += 1
    assert abs(yaw) >= 0.44  # girou os 25° DE VERDADE antes de parar
    assert cmd == (0.0, 0.0, True)  # e termina com STOP explícito


def test_spin_time_cap_when_yaw_frozen():
    # patinagem total (yaw não sai do lugar): teto de tempo encerra o giro
    sup, t, cmd = _spin_sup(spin_time_cap=1.0)
    ticks = 0
    while cmd.ang != 0.0 and ticks < 200:
        t += 0.1
        cmd = sup.update(t, nav_wants_move=True, position=(0.0, 0.0),
                         rear_gap=math.inf, yaw=0.0)
        ticks += 1
    assert ticks <= 12  # ~1s de cap, não ficou girando pra sempre


def test_spin_left_speed_boost():
    # esquerda escorrega -> comanda mais força nesse lado
    _, _, cmd_l = _spin_sup(open_side=1, spin_left_boost=1.4)
    _, _, cmd_r = _spin_sup(open_side=-1, spin_left_boost=1.4)
    assert cmd_l.ang == pytest.approx(3.0 * 1.4)   # esquerda: 4.2
    assert cmd_r.ang == pytest.approx(-3.0)        # direita: sem boost


def _drive_escalation(sup, t, nearest, n=80):
    """Conduz a 3ª manobra (escalada) passando `nearest`; devolve se ENTROU no
    giro em algum momento e o t final."""
    entered_spin = False
    for _ in range(n):
        sup.update(t, nav_wants_move=True, position=(0.0, 0.0),
                   rear_gap=math.inf, front_gap=0.0, nearest=nearest)
        if sup.state == "spinning":
            entered_spin = True
        t += 0.1
    return entered_spin, t


def test_spin_skipped_when_too_tight_to_turn():
    # 3ª vez no mesmo ponto (escalada) com obstáculo a 0.30m (< spin_clear 0.40):
    # NÃO gira — a quina varreria a parede (BATIDA 2026-06-28). Faz só a ré.
    sup = UnstuckSupervisor(_cfg(stuck_timeout=1.0, reverse_time_cap=1.0,
                                 grace=0.5))
    t = 0.0
    for _ in range(2):
        _, t = _stuck_cycle(sup, t, pos=(0.0, 0.0))
    entered_spin, _ = _drive_escalation(sup, t, nearest=0.30)
    assert not entered_spin


def test_spin_allowed_when_clearance_ok():
    # mesma escalada, mas com folga (1.0m > spin_clear): gira normal.
    sup = UnstuckSupervisor(_cfg(stuck_timeout=1.0, reverse_time_cap=1.0,
                                 grace=0.5))
    t = 0.0
    for _ in range(2):
        _, t = _stuck_cycle(sup, t, pos=(0.0, 0.0))
    entered_spin, _ = _drive_escalation(sup, t, nearest=1.0)
    assert entered_spin


def test_spin_aborts_when_obstacle_enters_sweep():
    # já girando (folga no disparo) e algo entra no raio de varredura da quina
    # DURANTE o giro -> STOP imediato (espelha o abort da ré/avanço).
    sup, t, cmd = _spin_sup()
    assert cmd.ang != 0.0 and sup.state == "spinning"
    cmd = sup.update(t + 0.1, nav_wants_move=True, position=(0.0, 0.0),
                     rear_gap=math.inf, yaw=0.0, nearest=0.30)
    assert cmd == (0.0, 0.0, True)
    assert sup.state == "grace"


def test_escape_reverse_extends_past_normal_until_room_then_spins():
    # LIVELOCK no canto (2026-06-28): escalada num canal apertado (sem folga pra
    # girar). A ré NORMAL pararia em 0.30; o ESCAPE REVERSE recua mais fundo pelo
    # rear aberto até a folga abrir, aí gira. (rear_gap grande o tempo todo.)
    sup = UnstuckSupervisor(_cfg(stuck_timeout=0.5, reverse_time_cap=10.0,
                                 grace=0.5))
    t = 0.0
    for _ in range(2):
        _, t = _stuck_cycle(sup, t, pos=(0.0, 0.0))
    # dispara a 3ª ré (escalada), frente bloqueada + canal apertado (near<spin_clear)
    t0 = t
    while sup.state != "reversing" and t < t0 + 5.0:
        sup.update(t, nav_wants_move=True, position=(0.0, 0.0), rear_gap=2.0,
                   front_gap=0.0, nearest=0.30)
        t += 0.1
    assert sup.state == "reversing"
    # recuou 0.40 (> ré normal 0.30) mas AINDA apertado -> escape reverse CONTINUA
    # (a ré normal teria ido pra grace aqui = o livelock)
    cmd = sup.update(t, nav_wants_move=True, position=(0.0, -0.40), rear_gap=2.0,
                     front_gap=0.0, nearest=0.30)
    assert cmd.lin == pytest.approx(-0.25) and sup.state == "reversing"
    # recuou mais e a folga ABRIU (saiu do canal) -> GIRA
    cmd = sup.update(t + 0.2, nav_wants_move=True, position=(0.0, -0.55),
                     rear_gap=2.0, front_gap=0.0, nearest=1.0)
    assert cmd.ang != 0.0 and sup.state == "spinning"


def _spin_cycle(sup, t0, pos, rear_gap, front_gap, nearest, open_side=1):
    """Um ciclo de recovery no cenário de GIRO (frente bloqueada, folga lateral pra
    girar). yaw congelado -> o giro acaba pelo spin_time_cap. Retorna (1º comando
    ativo do ciclo, t pronto pro próximo)."""
    t = t0
    deadline = t0 + 60.0
    first = None
    fired = False
    while t < deadline:
        cmd = sup.update(t, nav_wants_move=True, position=pos,
                         rear_gap=rear_gap, front_gap=front_gap,
                         open_side=open_side, nearest=nearest, yaw=0.0)
        if cmd.active:
            fired = True
            if first is None:
                first = cmd
        elif fired and sup.state == "monitoring":
            break  # manobra acabou e o grace venceu
        t += 0.1
    return first, t + 0.1


def test_forces_reverse_after_two_spins_same_spot():
    # ANTI-LIVELOCK (pedido do dono 2026-06-30): point-turn não muda a POSIÇÃO ->
    # gira no mesmo ponto, re-dispara, gira de novo ("preso fazendo só um movimento").
    # 2 giros no mesmo ponto sem sair -> a 3ª recovery TROCA pra translação (ré, que
    # é a direção mais aberta aqui: rear_gap 0.15 > front_gap 0.0).
    sup = UnstuckSupervisor(_cfg(stuck_timeout=1.0, grace=0.5, spin_time_cap=1.0,
                                 spin_escape_after=2))
    t = 0.0
    c1, t = _spin_cycle(sup, t, (0.0, 0.0), rear_gap=0.15, front_gap=0.0, nearest=1.0)
    c2, t = _spin_cycle(sup, t, (0.0, 0.0), rear_gap=0.15, front_gap=0.0, nearest=1.0)
    c3, t = _spin_cycle(sup, t, (0.0, 0.0), rear_gap=0.15, front_gap=0.0, nearest=1.0)
    assert c1.ang == pytest.approx(3.0) and c2.ang == pytest.approx(3.0)  # giros 1 e 2
    assert c3.ang == pytest.approx(0.0) and c3.lin == pytest.approx(-0.25)  # 3ª = RÉ


def test_forces_forward_when_front_more_open_after_two_spins():
    # mesma ideia, mas a FRENTE é a direção mais aberta -> avança em vez de girar.
    sup = UnstuckSupervisor(_cfg(stuck_timeout=1.0, grace=0.5, spin_time_cap=1.0,
                                 spin_escape_after=2, forward_speed=0.15))
    t = 0.0
    for _ in range(2):
        _, t = _spin_cycle(sup, t, (0.0, 0.0), rear_gap=0.0, front_gap=0.15, nearest=1.0)
    c3, _ = _spin_cycle(sup, t, (0.0, 0.0), rear_gap=0.0, front_gap=0.15, nearest=1.0)
    assert c3.ang == pytest.approx(0.0) and c3.lin == pytest.approx(0.15)  # FRENTE


def test_keeps_spinning_when_boxed_both_sides():
    # emparedado dos 2 lados (sem vão útil pra ré nem frente): mantém o giro — é a
    # única saída, não há como transladar contra parede.
    sup = UnstuckSupervisor(_cfg(stuck_timeout=1.0, grace=0.5, spin_time_cap=1.0,
                                 spin_escape_after=2))
    t = 0.0
    for _ in range(2):
        _, t = _spin_cycle(sup, t, (0.0, 0.0), rear_gap=0.05, front_gap=0.0, nearest=1.0)
    c3, _ = _spin_cycle(sup, t, (0.0, 0.0), rear_gap=0.05, front_gap=0.0, nearest=1.0)
    assert c3.ang == pytest.approx(3.0)  # ainda gira


def test_spin_escape_count_is_per_spot():
    # o contador é por PONTO (same_spot_radius): 2 giros em (0,0) não forçam
    # translação num lugar diferente.
    sup = UnstuckSupervisor(_cfg(stuck_timeout=1.0, grace=0.5, spin_time_cap=1.0,
                                 spin_escape_after=2))
    t = 0.0
    for _ in range(2):
        _, t = _spin_cycle(sup, t, (0.0, 0.0), rear_gap=0.15, front_gap=0.0, nearest=1.0)
    # longe (2m): sem histórico de giro nesse ponto -> a recovery lá é GIRO normal
    c, _ = _spin_cycle(sup, t, (2.0, 0.0), rear_gap=0.15, front_gap=0.0, nearest=1.0)
    assert c.ang == pytest.approx(3.0)


def test_no_escalation_when_stuck_at_different_spots():
    # travou em lugares DIFERENTES (>same_spot_radius): sempre ré reta
    sup = UnstuckSupervisor(_cfg(reverse_time_cap=2.0, grace=0.5))
    t = 0.0
    for k in range(4):
        cmds, t = _stuck_cycle(sup, t, pos=(2.0 * k, 0.0))
        assert all(c.ang == pytest.approx(0.0) for c in cmds)


def test_escalation_window_expires():
    # travamentos antigos (fora da janela) não contam pra escalada
    sup = UnstuckSupervisor(_cfg(reverse_time_cap=2.0, grace=0.5,
                                 escalate_window=30.0))
    t = 0.0
    _, t = _stuck_cycle(sup, t, pos=(0.0, 0.0))
    _, t = _stuck_cycle(sup, t, pos=(0.0, 0.0))
    t += 40.0  # janela de 30s expira os 2 eventos
    cmds, _ = _stuck_cycle(sup, t, pos=(0.0, 0.0))
    assert all(c.ang == pytest.approx(0.0) for c in cmds)  # ré reta de novo


def test_refires_repeatedly_if_still_stuck():
    # "nunca para sem razão": continua travado -> ré de novo, e de novo
    sup = UnstuckSupervisor(_cfg(reverse_time_cap=2.0, grace=1.0))
    fires = 0
    t = 0.0
    while t <= 60.0 and fires < 3:
        was_idle = not sup_active(sup)
        cmd = _tick(sup, t)  # posição parada pra sempre (ré não surte efeito)
        if cmd.active and was_idle:
            fires += 1
        t += 0.5
    assert fires >= 3


def sup_active(sup):
    return sup.state != "monitoring"


from robot_nav.unstuck_supervisor import door_zone_active


def test_door_zone_active_includes_approaching():
    # 2026-06-16: 'approaching' entra no standdown — o unstuck sabotava a
    # APROXIMAÇÃO da porta (ré+giro) antes do door_crossing assumir.
    # 'reversing' tb: a ré de escape do door_crossing é manobra dele, o unstuck
    # não pode atropelar.
    for st in ('approaching', 'staging', 'rotating', 'crossing', 'reversing'):
        assert door_zone_active(st) is True
    for st in ('idle', '', 'whatever'):
        assert door_zone_active(st) is False


# ---- recovery contextual: mapeado vs novo (2026-06-22) -----------------------

from robot_nav.unstuck_supervisor import MapGrid, map_occupied

# grid 5x5, resolução 0.1 m, origem (0,0) -> mundo [0,0.5)x[0,0.5).
# célula (row,col) tem centro ((col+0.5)*0.1, (row+0.5)*0.1). index = row*5+col.
def _grid_with(occ_cells, unknown_cells=()):
    data = [0] * 25
    for (r, c) in occ_cells:
        data[r * 5 + c] = 100
    for (r, c) in unknown_cells:
        data[r * 5 + c] = -1
    return MapGrid(data=data, width=5, height=5, resolution=0.1,
                   origin_x=0.0, origin_y=0.0)


def test_map_occupied_on_occupied_cell():
    g = _grid_with([(2, 2)])                      # centro (0.25, 0.25)
    assert map_occupied(g, 0.25, 0.25, 0.02, 65) is True


def test_map_occupied_on_free_cell():
    g = _grid_with([(2, 2)])
    assert map_occupied(g, 0.05, 0.05, 0.02, 65) is False   # célula (0,0) livre


def test_map_occupied_unknown_is_not_occupied():
    g = _grid_with([], unknown_cells=[(2, 3)])    # centro (0.35, 0.25) = -1
    assert map_occupied(g, 0.35, 0.25, 0.02, 65) is False


def test_map_occupied_out_of_bounds_is_false():
    g = _grid_with([(2, 2)])
    assert map_occupied(g, 5.0, 5.0, 0.02, 65) is False
    assert map_occupied(g, -1.0, 0.25, 0.02, 65) is False


def test_map_occupied_neighborhood_reaches_occupied():
    g = _grid_with([(2, 2)])                      # ocupada em (0.25, 0.25)
    # ponto a 0.06 m da célula ocupada
    assert map_occupied(g, 0.25, 0.19, 0.08, 65) is True    # vizinhança 0.08 alcança
    assert map_occupied(g, 0.25, 0.19, 0.03, 65) is False   # vizinhança 0.03 não


def test_map_occupied_below_threshold_is_free():
    g = MapGrid(data=[50] * 25, width=5, height=5, resolution=0.1,
                origin_x=0.0, origin_y=0.0)
    assert map_occupied(g, 0.25, 0.25, 0.02, 65) is False   # 50 < 65


def _umap(sup, t, mapped, pos=(0.0, 0.0), gap=math.inf, front_gap=0.3, near=0.0):
    # near=0.0 (pinçado) -> a manobra do bloqueio mapeado é RÉ (sem folga p/ girar)
    return sup.update(t, nav_wants_move=True, position=pos, rear_gap=gap,
                      front_gap=front_gap, obstacle_mapped=mapped, nearest=near)


def test_mapped_block_reverses_at_short_timeout():
    sup = UnstuckSupervisor(_cfg(stuck_timeout=10.0, stuck_timeout_mapped=2.0))
    _umap(sup, 0.0, True)
    cmd = _umap(sup, 2.1, True)
    assert cmd.active is True
    assert cmd.lin == pytest.approx(-0.25)   # ré (não esperou os 10 s)


def test_mapped_block_does_not_fire_before_short_timeout():
    sup = UnstuckSupervisor(_cfg(stuck_timeout=10.0, stuck_timeout_mapped=2.0))
    _umap(sup, 0.0, True)
    assert _umap(sup, 1.9, True).active is False


def test_new_block_still_waits_full_timeout():
    sup = UnstuckSupervisor(_cfg(stuck_timeout=10.0, stuck_timeout_mapped=2.0))
    _umap(sup, 0.0, False)
    assert _umap(sup, 2.1, False).active is False     # novo: NÃO encurta
    assert _umap(sup, 10.1, False).active is True     # mas dispara aos 10 s


def test_late_mapped_flip_respects_confirmation_window():
    # parado desde 0; vira mapeado só aos 9 s -> precisa 2 s contínuos de mapeado,
    # não dispara na hora. stuck_timeout alto isola o caminho mapeado.
    sup = UnstuckSupervisor(_cfg(stuck_timeout=100.0, stuck_timeout_mapped=2.0))
    _umap(sup, 0.0, False)
    _umap(sup, 8.9, False)
    _umap(sup, 9.0, True)                              # mapped_since = 9.0
    assert _umap(sup, 9.5, True).active is False       # só 0.5 s de mapeado
    assert _umap(sup, 11.05, True).active is True      # 2.05 s de mapeado -> dispara


def test_mapped_since_resets_when_unmapped():
    sup = UnstuckSupervisor(_cfg(stuck_timeout=100.0, stuck_timeout_mapped=2.0))
    _umap(sup, 0.0, True)                              # mapped_since = 0
    _umap(sup, 1.0, False)                             # deixou de ser mapeado -> reset
    _umap(sup, 2.0, True)                              # mapped_since = 2.0 (não carrega do 0)
    assert _umap(sup, 2.5, True).active is False        # só 0.5 s -> não dispara


def test_obstacle_mapped_defaults_to_todays_behavior():
    # sem passar obstacle_mapped -> default False -> espera os 10 s (inalterado)
    sup = UnstuckSupervisor(_cfg(stuck_timeout=10.0, stuck_timeout_mapped=2.0))
    sup.update(0.0, nav_wants_move=True, position=(0.0, 0.0))
    assert sup.update(2.1, nav_wants_move=True, position=(0.0, 0.0)).active is False


# ---- bloqueio FORA do eixo: usa o ponto de contato REAL (2026-06-22) ----------
# Campo: o robô encostou torto (contato a -32°); projetar "reto à frente" caía ao
# lado da parede mapeada (mapped=False). Fix: usar o (x,y) REAL do contato (com
# offset lateral) + vizinhança maior pra absorver o registro (~0.2 m de pose).

from robot_nav.unstuck_supervisor import front_block_point, block_point_mapped


def _scan_pt(x, y, n=72):
    """scan de n feixes (360°) com UM ponto em (x,y) no frame do robô."""
    a = math.atan2(y, x)
    r = math.hypot(x, y)
    amin, ainc = -math.pi, 2 * math.pi / n
    ranges = [float('inf')] * n
    i = int(round((a - amin) / ainc)) % n
    ranges[i] = r
    return ranges, amin, ainc


def test_front_block_point_returns_offaxis_contact():
    ranges, amin, ainc = _scan_pt(0.40, -0.20)        # contato à direita do eixo
    bp = front_block_point(ranges, amin, ainc, lidar_x=0.0, head_x=0.25,
                           half_width=0.30)
    assert bp is not None
    assert bp[0] == pytest.approx(0.40, abs=0.06)
    assert bp[1] == pytest.approx(-0.20, abs=0.06)     # mantém o lateral real


def test_front_block_point_none_when_corridor_clear():
    ranges, amin, ainc = _scan_pt(0.0, 1.0)            # só de lado, nada na frente
    assert front_block_point(ranges, amin, ainc, 0.0, 0.25, 0.30) is None


def test_block_point_mapped_uses_real_lateral_offset():
    # contato FORA do eixo (y=-0.20). Reto à frente cairia em (0.45,0.45) [livre];
    # usar o (x,y) real cai em (0.45,0.25) = parede mapeada. Robô em (0.05,0.45).
    g = _grid_with([(2, 4)])                            # ocupada em (0.45,0.25)
    assert block_point_mapped(g, (0.05, 0.45), 0.0, (0.40, -0.20), head_x=0.25,
                              block_range=0.5, neighborhood=0.10,
                              occ_threshold=65) is True


def test_block_point_mapped_gated_by_block_range():
    g = _grid_with([(2, 4)])
    # contato longe (forward_dist = 0.90-0.25 = 0.65 > block_range 0.5) -> False
    assert block_point_mapped(g, (0.05, 0.45), 0.0, (0.90, 0.0), head_x=0.25,
                              block_range=0.5, neighborhood=0.10,
                              occ_threshold=65) is False


def test_block_point_mapped_none_inputs_false():
    g = _grid_with([(2, 4)])
    assert block_point_mapped(g, (0.05, 0.45), 0.0, None, 0.25, 0.5, 0.10, 65) is False
    assert block_point_mapped(None, (0.05, 0.45), 0.0, (0.40, -0.20), 0.25, 0.5, 0.10, 65) is False


# ---- clearest_heading_offset (giro CALCULADO de menor correção) ------------
# Em vez de girar um valor fixo "pra longe do obstáculo", MEDE qual o menor
# ajuste de heading que abre um corredor reto pra frente (caso do dono:
# "faltavam 5° pra esquerda pra ir reto"). prefer_bearing = rumo do /plan
# (desempate). None = nenhuma rotação pequena abre -> cai na ré.

def _clear_args(depth=0.6, max_off=math.radians(60), step=math.radians(2)):
    return dict(depth=depth, max_offset=max_off, step=step, **GEO_FRONT)


def test_clearest_heading_offset_zero_when_front_already_clear():
    # obstáculo só ATRÁS -> a frente já está livre, nenhuma correção precisa
    ranges, amin, ainc = _scan_with_obstacle_at(math.pi, 0.50)
    assert clearest_heading_offset(ranges, amin, ainc, **_clear_args()) == 0.0


def test_clearest_heading_offset_finds_a_turn_that_clears_the_front():
    # obstáculo reto à frente, dentro do depth -> deve achar uma rotação que abre
    ranges, amin, ainc = _scan_with_obstacle_at_base(0.40, 0.0)
    o = clearest_heading_offset(ranges, amin, ainc, **_clear_args())
    assert o is not None
    # POSTCONDIÇÃO: girando por o, o corredor frontal abre >= depth
    assert front_min_gap(ranges, amin - o, ainc, **GEO_FRONT) >= 0.6


def test_clearest_heading_offset_is_the_smallest_correction():
    # nenhum ajuste de MAGNITUDE MENOR que o resultado abre a frente
    ranges, amin, ainc = _scan_with_obstacle_at_base(0.40, 0.0)
    step = math.radians(2)
    o = clearest_heading_offset(ranges, amin, ainc, **_clear_args(step=step))
    assert o is not None
    for k in range(0, int(round(abs(o) / step))):
        m = k * step
        assert front_min_gap(ranges, amin - m, ainc, **GEO_FRONT) < 0.6
        assert front_min_gap(ranges, amin + m, ainc, **GEO_FRONT) < 0.6


def test_clearest_heading_offset_none_when_front_is_boxed():
    # parede densa cobrindo todo o arco frontal perto -> nenhuma rotação abre
    n = 360
    amin = -math.pi
    ainc = 2 * math.pi / n
    ranges = [float("inf")] * n
    for i in range(n):
        a = amin + i * ainc
        if abs(a) <= math.radians(80):
            ranges[i] = 0.35  # perto, dentro do depth, em todo o arco frontal
    assert clearest_heading_offset(ranges, amin, ainc, **_clear_args()) is None


def test_clearest_heading_offset_breaks_ties_toward_the_plan():
    # frente reta bloqueada -> abre simétrico p/ os 2 lados; o plano à ESQUERDA
    # (prefer_bearing>0) desempata pro lado esquerdo (offset positivo)
    ranges, amin, ainc = _scan_with_obstacle_at_base(0.40, 0.0)
    o = clearest_heading_offset(ranges, amin, ainc,
                                prefer_bearing=math.radians(50), **_clear_args())
    assert o is not None and o > 0
    # espelho: plano à DIREITA -> offset negativo
    o2 = clearest_heading_offset(ranges, amin, ainc,
                                 prefer_bearing=math.radians(-50), **_clear_args())
    assert o2 is not None and o2 < 0


# ---- giro CALCULADO na decisão (clear_offset -> gira em vez de dar ré) ------
# Dono 2026-06-29: "às vezes falta 5° pra ir reto e ele dá ré, gira fixo, erra,
# dá ré de novo". Se há um giro PEQUENO (computado no nó, <= cap) que abre a
# frente rumo ao plano, gira só isso em vez de dar ré. clear_offset vem do nó.

def _stuck_then(sup, clear_offset=None, yaw=0.0, front_gap=0.0, rear_gap=math.inf):
    # ancora no t=0 e dispara a recovery no t=10.1 (passou o stuck_timeout=10)
    sup.update(0.0, nav_wants_move=True, position=(0.0, 0.0), rear_gap=rear_gap,
               front_gap=front_gap, yaw=yaw, nearest=0.0, clear_offset=clear_offset)
    return sup.update(10.1, nav_wants_move=True, position=(0.0, 0.0),
                      rear_gap=rear_gap, front_gap=front_gap, yaw=yaw,
                      nearest=0.0, clear_offset=clear_offset)


def test_clear_offset_turns_instead_of_reversing():
    # frente bloqueada, ré DISPONÍVEL (rear inf), mas cabe um giro de 0.20 rad ->
    # prefere o giro pequeno à ré
    sup = UnstuckSupervisor(_cfg())
    cmd = _stuck_then(sup, clear_offset=0.20)
    assert cmd.active is True
    assert cmd.lin == pytest.approx(0.0)   # giro no lugar, NÃO ré
    assert cmd.ang > 0                       # +offset -> gira pra esquerda
    assert sup.state == "turning"


def test_clear_turn_direction_follows_offset_sign():
    sup = UnstuckSupervisor(_cfg())
    cmd = _stuck_then(sup, clear_offset=-0.20)
    assert cmd.lin == pytest.approx(0.0)
    assert cmd.ang < 0                       # -offset -> gira pra direita
    assert sup.state == "turning"


def test_clear_turn_reaches_target_then_grace():
    sup = UnstuckSupervisor(_cfg())
    _stuck_then(sup, clear_offset=0.20, yaw=0.0)   # alvo de yaw = 0.20
    assert sup.state == "turning"
    # yaw chegou no alvo -> encerra a manobra (grace)
    cmd = sup.update(10.2, nav_wants_move=True, position=(0.0, 0.0),
                     rear_gap=math.inf, front_gap=0.0, yaw=0.20, nearest=0.0)
    assert sup.state == "grace"
    assert cmd.lin == pytest.approx(0.0) and cmd.ang == pytest.approx(0.0)


def test_tiny_clear_offset_still_reverses():
    # giro calculado abaixo do mínimo (no-op) -> NÃO vira o caminho da ré
    sup = UnstuckSupervisor(_cfg())
    cmd = _stuck_then(sup, clear_offset=0.01)
    assert cmd.lin == pytest.approx(-0.25)   # ré normal
    assert sup.state == "reversing"


def test_no_clear_offset_reverses_as_before():
    # regressão: sem clear_offset, o comportamento é o de antes (ré)
    sup = UnstuckSupervisor(_cfg())
    cmd = _stuck_then(sup, clear_offset=None)
    assert cmd.lin == pytest.approx(-0.25)
    assert sup.state == "reversing"
