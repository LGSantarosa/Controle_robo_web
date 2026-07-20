import math

import pytest

from robot_nav.path_follower import (
    wrap,
    closest_index,
    carrot_point,
    straight_deviation,
    FollowConfig,
    DecisiveFollower,
)


def test_wrap():
    assert wrap(0.0) == pytest.approx(0.0)
    assert wrap(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert wrap(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)


def test_closest_index():
    path = [(0, 0), (1, 0), (2, 0), (3, 0)]
    assert closest_index(path, 2.1, 0.05) == 2
    assert closest_index(path, -0.4, 0.0) == 0


def test_carrot_at_lookahead_distance():
    # caminho reto em +x, passos de 0.1 m; carrot a 1.0 m do índice 0
    path = [(i * 0.1, 0.0) for i in range(40)]
    ci, (cx, cy) = carrot_point(path, 0, lookahead=1.0)
    assert cx == pytest.approx(1.0, abs=0.1)
    assert cy == pytest.approx(0.0)


def test_carrot_follows_the_bend_not_the_goal():
    # L: reto +x até (1.0,0), depois sobe +y. Carrot de 0.5 m do começo
    # deve cair AINDA no trecho +x (não pular pro goal lá em cima).
    path = [(i * 0.1, 0.0) for i in range(11)]            # (0,0)..(1.0,0)
    path += [(1.0, j * 0.1) for j in range(1, 11)]        # sobe
    ci, (cx, cy) = carrot_point(path, 0, lookahead=0.5)
    assert (cx, cy) == pytest.approx((0.5, 0.0), abs=0.1)
    # carrot longo (1.5 m) já entra no trecho de subida
    ci2, (cx2, cy2) = carrot_point(path, 0, lookahead=1.5)
    assert cy2 > 0.1


def test_carrot_clamps_to_goal_when_path_short():
    path = [(0, 0), (0.2, 0)]
    ci, p = carrot_point(path, 0, lookahead=1.0)
    assert p == (0.2, 0)


def _fol():
    return DecisiveFollower(FollowConfig())


def test_idle_when_no_goal_or_no_path():
    f = _fol()
    assert f.update((0, 0, 0), [(1, 0), (2, 0)], goal_active=False,
                    goal_yaw=0.0).state == 'idle'
    assert f.update((0, 0, 0), None, goal_active=True, goal_yaw=0.0).state == 'idle'


def test_drives_straight_when_aligned():
    f = _fol()
    path = [(x * 0.1, 0.0) for x in range(40)]   # reto +x, robô alinhado
    cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True, goal_yaw=0.0)
    assert cmd.state == 'driving'
    assert cmd.vx > 0.0 and cmd.wz == pytest.approx(0.0)


def test_turns_in_place_when_misaligned_shortest_angle():
    f = _fol()
    path = [(0.0, y * 0.1) for y in range(40)]   # caminho +y, robô olha +x
    cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True, goal_yaw=math.pi / 2)
    assert cmd.state == 'turning'
    assert cmd.vx == pytest.approx(0.0)
    assert cmd.wz > 0.0       # menor ângulo p/ +90° é girar +


def test_hysteresis_keeps_driving_through_small_error():
    # erro ~8° (entre turn_exit 3° e turn_enter 12°): estando DRIVING, continua
    # dirigindo (não cai em pulinho). path levemente inclinado 8°.
    f = _fol()
    f.state = 'driving'
    ang = math.radians(8)
    path = [(math.cos(ang) * x * 0.1, math.sin(ang) * x * 0.1) for x in range(40)]
    cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True, goal_yaw=ang)
    assert cmd.state == 'driving'


def test_hysteresis_keeps_turning_until_well_aligned():
    # estando TURNING com erro ~8° (acima do exit 3°), continua girando.
    f = _fol()
    f.state = 'turning'
    ang = math.radians(8)
    path = [(math.cos(ang) * x * 0.1, math.sin(ang) * x * 0.1) for x in range(40)]
    cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True, goal_yaw=ang)
    assert cmd.state == 'turning'


def test_turn_magnitude_respects_min_and_max():
    f = _fol()
    cfg = f.cfg
    assert abs(f._turn_cmd(math.pi)) == pytest.approx(cfg.rot_max)
    assert abs(f._turn_cmd(math.radians(5))) == pytest.approx(cfg.rot_min)


def test_rot_min_default_beats_deadzone_crawl():
    # 2026-07-02: rot_min 2.0 comandado ≈ 10°/s real (zona-morta 1.7 +
    # resposta 0.6·(cmd−1.7)) = rastejo que parece parada. 2.4 ≈ 25°/s.
    assert FollowConfig().rot_min == pytest.approx(2.4)


def test_turn_target_frozen_while_plan_shifts():
    # entra girando pra +90° (path +y); no meio do giro o plano vira pra -y.
    # SEM freeze ele inverteria o giro (caça alvo móvel); COM freeze segue +.
    f = _fol()
    path_up = [(0.0, y * 0.1) for y in range(40)]
    cmd = f.update((0.0, 0.0, 0.0), path_up, goal_active=True, goal_yaw=math.pi / 2)
    assert cmd.state == 'turning' and cmd.wz > 0.0
    path_down = [(0.0, -y * 0.1) for y in range(40)]
    cmd2 = f.update((0.0, 0.0, math.radians(45)), path_down, goal_active=True,
                    goal_yaw=-math.pi / 2)
    assert cmd2.state == 'turning'
    assert cmd2.wz > 0.0          # continua no alvo congelado (+90°), não flipa


def test_turn_target_cleared_after_alignment():
    # alinhou com o alvo congelado -> driving e o próximo giro re-mira o plano novo.
    f = _fol()
    path_up = [(0.0, y * 0.1) for y in range(40)]
    f.update((0.0, 0.0, 0.0), path_up, goal_active=True, goal_yaw=math.pi / 2)
    cmd = f.update((0.0, 0.0, math.pi / 2), path_up, goal_active=True,
                   goal_yaw=math.pi / 2)
    assert cmd.state == 'driving'
    assert f._turn_target is None


def test_turn_target_reset_when_goal_lost():
    f = _fol()
    path_up = [(0.0, y * 0.1) for y in range(40)]
    f.update((0.0, 0.0, 0.0), path_up, goal_active=True, goal_yaw=math.pi / 2)
    cmd = f.update((0.0, 0.0, 0.0), path_up, goal_active=False, goal_yaw=None)
    assert cmd.state == 'idle'
    assert f._turn_target is None


# ---- saída PREDITIVA do giro (07-17: overshoot na run real) --------------
# Campo 07-17: giros saíam do turning já com ±16° do OUTRO lado (deviam sair
# em ≤7°) — entre medir o yaw e o robô parar existe atraso (pose lagada +
# inércia), então 52% dos giros só desfaziam o anterior. Fix: parar quando o
# yaw PREVISTO daqui a turn_stop_tau segundos cruza a banda de saída.

def _spin_towards(f, target, rate, ticks, start_err):
    """Gira a `rate` rad/s rumo ao alvo congelado; devolve (herr, cmd) por tick."""
    dt = f.cfg.tick_dt
    path = [(x * 0.1, 0.0) for x in range(80)]     # reto +x, longe do goal
    f.state = 'turning'
    f._turn_target = target
    out = []
    yaw = target - start_err
    for _ in range(ticks):
        cmd = f.update((0.0, 0.0, yaw), path, goal_active=True, goal_yaw=0.0)
        out.append((target - yaw, cmd))
        if cmd.state != 'turning':
            break
        yaw += rate * dt
    return out


def test_predictive_exit_stops_before_the_band():
    # girando a 1 rad/s com tau 0.25: tem que soltar o giro ~0.25 rad ANTES
    # da banda velha (0.12), i.e. com herr ainda ~0.3-0.4.
    cfg = FollowConfig(turn_stop_tau=0.25, tick_dt=0.05)
    f = DecisiveFollower(cfg)
    hist = _spin_towards(f, target=0.9, rate=1.0, ticks=40, start_err=0.9)
    herr_exit, cmd_exit = hist[-1]
    assert cmd_exit.state == 'driving'
    assert herr_exit > cfg.turn_exit + 0.1        # saiu bem antes da banda
    assert herr_exit < cfg.turn_exit + 0.35       # mas não cedo demais


def test_tau_zero_keeps_old_exit_behavior():
    cfg = FollowConfig(turn_stop_tau=0.0, tick_dt=0.05)
    f = DecisiveFollower(cfg)
    hist = _spin_towards(f, target=0.9, rate=1.0, ticks=40, start_err=0.9)
    herr_exit, cmd_exit = hist[-1]
    assert cmd_exit.state == 'driving'
    assert herr_exit <= cfg.turn_exit + 0.06      # banda velha (1 tick de folga)


def test_predictive_exit_ignores_wrong_direction_rate():
    # yaw ANDANDO PRA LONGE do alvo (ex.: pose corrigida pelo AMCL): a
    # previsão não pode soltar o giro mais cedo.
    cfg = FollowConfig(turn_stop_tau=0.25, tick_dt=0.05)
    f = DecisiveFollower(cfg)
    hist = _spin_towards(f, target=0.9, rate=-0.8, ticks=10, start_err=0.5)
    assert all(cmd.state == 'turning' for _, cmd in hist)


# ---- MIRA FILTRADA (07-17: replan 1Hz balança a mira ±14° em corredor) ----
# Sim hotmilk: Theta* nasce no robô e pivota ora numa parede inflada ora na
# outra — a mira salta 13-15° por replan. Alinhamento justo (exit 3°) fez o
# robô perseguir o balanço (vai-e-volta 4->15). EMA na direção da mira, só
# com carrot esticado (trecho reto); curva = cru + reset.

def _tilted(ang, n=40):
    return [(math.cos(ang) * k * 0.1, math.sin(ang) * k * 0.1)
            for k in range(n)]


def test_aim_filter_ignores_replan_wobble():
    # alinhado no corredor; replans alternam a mira ±20° (acima do turn_enter
    # 16°) a cada 1s. Filtrado (tau 2s) o swing vira ~±5° -> NUNCA gira.
    f = _fol()
    for _ in range(10):                             # seed: corredor reto +x
        cmd = f.update((0.0, 0.0, 0.0), _tilted(0.0), goal_active=True,
                       goal_yaw=0.0)
        assert cmd.state == 'driving'
    ang = math.radians(20)
    for cycle in range(6):                          # 6s de wobble ±20°
        path = _tilted(ang if cycle % 2 == 0 else -ang)
        for _ in range(20):                         # 1s por replan
            cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True,
                           goal_yaw=0.0)
            assert cmd.state == 'driving'           # sem filtro viraria turning


def test_aim_filter_seeds_raw_on_first_sight_of_curve():
    # canto de verdade a 0.3m visto do ZERO: o filtro semeia com a mira crua
    # (sem histórico não há lag) -> gira no MESMO tick.
    f = _fol()
    path = [(k * 0.1, 0.0) for k in range(4)]       # reto +x até 0.3
    path += [(0.3, k * 0.1) for k in range(1, 30)]  # canto 90° sobe +y
    cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True,
                   goal_yaw=math.pi / 2)
    assert cmd.state == 'turning'


def test_aim_filter_short_mode_damps_wobble_but_takes_the_corner():
    # modo CURTO forçado (straight_tol=0 desliga o estico): replans balançando
    # ±20° a 1Hz não viram giro (tau_short 0.8 -> ~±11°); canto REAL
    # persistente de 45° entra no giro em <1s.
    f = DecisiveFollower(FollowConfig(straight_tol=0.0))
    for _ in range(10):                              # semeia alinhado
        st = f.update((0.0, 0.0, 0.0), _tilted(0.0), goal_active=True,
                      goal_yaw=0.0).state
    assert st == 'driving'
    ang = math.radians(20)
    for cycle in range(4):                           # 4s de wobble ±20°
        path = _tilted(ang if cycle % 2 == 0 else -ang)
        for _ in range(20):
            cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True,
                           goal_yaw=0.0)
            assert cmd.state == 'driving'            # cru dispararia turn_enter
    corner = _tilted(math.radians(45))               # canto real persistente
    states = [f.update((0.0, 0.0, 0.0), corner, goal_active=True,
                       goal_yaw=0.0).state for _ in range(20)]  # 1s
    assert 'turning' in states


def test_aim_filter_tracks_slow_real_change():
    # mudança REAL e persistente de 30° (não wobble): o filtro converge e o
    # robô entra no giro em ~2-3s — atraso aceitável, não paralisia.
    f = _fol()
    for _ in range(10):
        f.update((0.0, 0.0, 0.0), _tilted(0.0), goal_active=True,
                 goal_yaw=0.0)
    path = _tilted(math.radians(30))
    states = [f.update((0.0, 0.0, 0.0), path, goal_active=True,
                       goal_yaw=0.0).state for _ in range(80)]  # 4s
    assert 'turning' in states


def test_microsim_lagged_plant_predictive_cuts_reversals():
    # Planta com atraso de atuação (0.3s): o wz comandado só vira yaw depois
    # de 6 ticks — reproduz o overshoot do campo. Com tau=0.3 os flips de
    # sinal do wz têm que despencar vs tau=0.
    def run(tau):
        cfg = FollowConfig(turn_stop_tau=tau, tick_dt=0.05)
        f = DecisiveFollower(cfg)
        path = [(x * 0.1, 0.0) for x in range(200)]
        x = y = 0.0
        yaw = 0.6                                  # começa torto
        delay = [0.0] * 6                          # 0.3s de atraso
        flips = 0
        prev_wz = 0.0
        for _ in range(400):
            cmd = f.update((x, y, yaw), path, goal_active=True, goal_yaw=0.0)
            if cmd.wz * prev_wz < 0:
                flips += 1
            if abs(cmd.wz) > 1e-9:
                prev_wz = cmd.wz
            delay.append(cmd.wz * 0.4)             # ~resposta real 0.6·(cmd−1.7)
            wz_real = delay.pop(0)
            yaw += wz_real * cfg.tick_dt
            x += cmd.vx * math.cos(yaw) * cfg.tick_dt
            y += cmd.vx * math.sin(yaw) * cfg.tick_dt
        return flips

    assert run(0.3) < run(0.0)


def test_straight_deviation():
    straight = [(i * 0.1, 0.0) for i in range(20)]
    assert straight_deviation(straight, 0, 19) == pytest.approx(0.0)
    bent = [(i * 0.1, 0.0) for i in range(11)]          # reto até (1,0)...
    bent += [(1.0, j * 0.1) for j in range(1, 11)]      # ...canto de 90°
    assert straight_deviation(bent, 0, len(bent) - 1) > 0.4
    assert straight_deviation(bent, 0, 0) == pytest.approx(0.0)   # degenerado


def test_far_carrot_on_straight_path():
    # ZIGUE-ZAGUE da run hotmilk 07-08: carrot 0.6 + ruído lateral de 13cm =
    # herr 12° = turn_enter -> 184 giros no lugar, 127 <10°, L/R alternado.
    # Em trecho RETO o carrot estica (lookahead_far): o MESMO desvio de 13cm
    # vira ~4.6° -> continua driving, corredor sai numa reta só.
    f = _fol()
    path = [(x * 0.05, 0.0) for x in range(80)]     # corredor reto de 4m
    cmd = f.update((0.0, 0.13, 0.0), path, goal_active=True, goal_yaw=0.0)
    assert cmd.state == 'driving'                   # não gira por migalha
    assert f.dbg['la'] == pytest.approx(f.cfg.lookahead_far)
    assert f.dbg['dist_aim'] > 1.0                  # mirou LONGE de fato


def test_near_carrot_with_short_lookahead_would_turn():
    # contraprova do cenário acima: com o adaptativo DESLIGADO (straight_tol=0)
    # um desvio lateral que estoura a banda (20cm a 0.6m = ~18° > turn_enter
    # 16°) dispara turning; esticado, os mesmos 20cm seriam ~7.6° e seguiriam
    # driving — o mecanismo do zigue-zague antigo.
    f = DecisiveFollower(FollowConfig(straight_tol=0.0))
    path = [(x * 0.05, 0.0) for x in range(80)]
    cmd = f.update((0.0, 0.20, 0.0), path, goal_active=True, goal_yaw=0.0)
    assert cmd.state == 'turning'
    assert f.dbg['la'] == pytest.approx(f.cfg.lookahead)


def test_near_carrot_kept_when_corner_ahead():
    # BO de 06-27 que NÃO pode voltar: lookahead longo cortava o arco/raspava
    # na porta. Com canto DENTRO do alcance far, o desvio da corda estoura o
    # straight_tol -> mantém o carrot 0.6 validado (não corta a curva).
    f = _fol()
    path = [(i * 0.05, 0.0) for i in range(17)]         # reto até (0.8, 0)
    path += [(0.8, j * 0.05) for j in range(1, 25)]     # canto 90° sobe
    cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True, goal_yaw=math.pi / 2)
    assert f.dbg['la'] == pytest.approx(f.cfg.lookahead)
    assert f.dbg['dist_aim'] < 1.0                      # mira PERTO, pré-canto
    assert cmd.state == 'driving'                       # alinhado c/ o trecho reto


def test_far_carrot_after_rounding_the_corner():
    # passou o canto -> o que sobra do plano é reto -> volta a mirar longe.
    f = _fol()
    path = [(i * 0.05, 0.0) for i in range(17)]
    path += [(0.8, j * 0.05) for j in range(1, 41)]     # perna longa pós-canto
    f.update((0.8, 0.1, math.pi / 2), path, goal_active=True, goal_yaw=math.pi / 2)
    assert f.dbg['la'] == pytest.approx(f.cfg.lookahead_far)


def test_near_carrot_when_passage_tight():
    # FRESTA do sim hotmilk_portas 07-10: plano reto atravessa uma passagem
    # apertada; com carrot 1.5m a banda morta de giro tolera ±31cm de drift
    # lateral (la*sin(12°)) -> chega na boca 15cm fora do eixo e raspa a quina
    # (preso 262s alinha-desalinha; a v1 do fix por offset-ao-plano falhou pq
    # o plano NASCE no robô a cada replan). Parede perto (front_clear <
    # stretch_clearance) -> carrot CURTO: mesmo drift vira ~27° -> giro
    # decidido, entra alinhado.
    f = _fol()
    path = [(x * 0.05, 0.0) for x in range(80)]     # plano reto de 4m
    cmd = f.update((0.0, 0.30, 0.0), path, goal_active=True, goal_yaw=0.0,
                   front_clear=0.5)
    assert f.dbg['la'] == pytest.approx(f.cfg.lookahead)
    assert cmd.state == 'turning'                   # corrige em vez de raspar


def test_far_carrot_back_in_open_space():
    # passagem venceu / espaço abriu (front_clear >= stretch_clearance) ->
    # volta a esticar (o anti-zigue-zague das retas continua valendo).
    f = _fol()
    path = [(x * 0.05, 0.0) for x in range(80)]
    f.update((1.0, 0.10, 0.0), path, goal_active=True, goal_yaw=0.0,
             front_clear=2.0)
    assert f.dbg['la'] == pytest.approx(f.cfg.lookahead_far)


def test_stretch_survives_missing_scan():
    # failsafe: sem scan o default é inf -> gate não atua (igual pré-gate);
    # o scan nunca pode matar o estico da nav inteira.
    f = _fol()
    path = [(x * 0.05, 0.0) for x in range(80)]
    f.update((0.0, 0.05, 0.0), path, goal_active=True, goal_yaw=0.0)
    assert f.dbg['la'] == pytest.approx(f.cfg.lookahead_far)


def test_goal_turn_then_arrived():
    f = _fol()
    path = [(0.0, 0.0), (0.05, 0.0)]   # goal coladinho
    cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True, goal_yaw=math.pi / 2)
    assert cmd.state == 'goal_turn' and cmd.wz > 0.0
    cmd = f.update((0.0, 0.0, math.pi / 2), path, goal_active=True,
                   goal_yaw=math.pi / 2)
    assert cmd.state == 'arrived' and (cmd.vx, cmd.wz) == (0.0, 0.0)
