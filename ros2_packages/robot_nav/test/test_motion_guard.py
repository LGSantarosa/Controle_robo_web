"""Testes da lógica pura do motion_guard (sem ROS)."""
import math

from robot_nav.motion_guard import GuardConfig, MapGhostFilter, MotionGuard

POSE = (0.0, 0.0, 0.0)   # robô na origem olhando +x (frame odom)
WALL = [(2.0, y * 0.1 - 1.0) for y in range(20)]   # parede estática em x=2


def _guard(**kw):
    return MotionGuard(GuardConfig(**kw))


def _feed_static(g, t0=0.0, n=8, dt=0.1, pts=WALL):
    """alimenta n scans estáticos p/ encher o histórico (lookback 0.5s)."""
    for i in range(n):
        g.observe(t0 + i * dt, pts, POSE, 0.0)
    return t0 + n * dt


def _feed_mover(g, t, obj, frames=None, dt=0.1, wz=0.0, pose=POSE):
    """alimenta o móvel por `frames` scans consecutivos (default = o mínimo
    p/ latchar, persist_frames). Retorna o t do último scan."""
    n = frames if frames is not None else g.cfg.persist_frames
    for i in range(n):
        g.observe(t + i * dt, WALL + obj, pose, wz)
    return t + (n - 1) * dt


def test_static_wall_not_moving():
    g = _guard()
    _feed_static(g)
    assert g.moving_clusters == []
    assert g.nearest_moving == math.inf


def test_moving_object_detected_and_clustered():
    g = _guard()
    t = _feed_static(g)
    # objeto NOVO (célula livre 0.5s atrás) com 4 pontos juntos a ~1m
    obj = [(1.0, 0.8), (1.0, 0.9), (1.1, 0.8), (1.1, 0.9)]
    g.observe(t, WALL + obj, POSE, 0.0)
    assert len(g.moving_clusters) == 1
    assert len(g.moving_clusters[0]) == 4
    assert g.nearest_moving < 1.5


def test_small_cluster_is_noise():
    g = _guard()   # min_cluster_points=3
    t = _feed_static(g)
    g.observe(t, WALL + [(1.0, 0.8), (1.05, 0.85)], POSE, 0.0)
    assert g.moving_clusters == []


def test_beyond_guard_radius_ignored():
    g = _guard()   # guard_radius=2.5
    t = _feed_static(g)
    obj_far = [(4.0, 3.0), (4.0, 3.1), (4.1, 3.0)]
    g.observe(t, WALL + obj_far, POSE, 0.0)
    assert g.moving_clusters == []


def test_no_history_no_detection():
    g = _guard()
    g.observe(0.0, WALL + [(1.0, 0.8), (1.0, 0.9), (1.1, 0.8)], POSE, 0.0)
    assert g.moving_clusters == []   # sem snapshot >= lookback atrás


def test_corridor_flag():
    g = _guard()
    t = _feed_static(g)
    # móvel BEM na frente (xb ~1.0, |yb| < 0.35)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    g.observe(t, WALL + obj, POSE, 0.0)
    assert g.in_corridor is True


def test_corridor_respects_robot_yaw():
    g = _guard()
    pose = (0.0, 0.0, math.pi / 2)   # olhando +y
    for i in range(8):
        g.observe(i * 0.1, WALL, pose, 0.0)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]   # à DIREITA do robô
    g.observe(0.8, WALL + obj, pose, 0.0)
    assert len(g.moving_clusters) == 1
    assert g.in_corridor is False


def test_filter_idle_passes_command():
    g = _guard()
    t = _feed_static(g)
    vx, wz, st = g.filter(t, 0.30, 1.0)
    assert (vx, wz, st) == (0.30, 1.0, 'idle')


def test_occlusion_reveal_is_not_moving():
    # FALSO POSITIVO residual de campo 07-03 (2ª rodada, sem ninguém perto:
    # 71% do tempo freado): o robô anda, um trecho de parede que estava na
    # SOMBRA de um objeto aparece -> célula "ausente" no snapshot velho era
    # tratada como "estava livre" = móvel. Raycast: só é móvel se o feixe
    # velho ATRAVESSOU a célula (alcance antigo > distância + margem).
    g = _guard()
    # occluder: blob a 1.0m (bearing ±11°) sombreia a parede x=2 em |y|<0.4;
    # o scan velho NÃO vê esse trecho (nem células vizinhas dele)
    occluder = [(1.0, -0.2 + y * 0.05) for y in range(9)]   # x=1, y=-0.2..0.2
    wall_shadowed = [p for p in WALL if not (-0.4 < p[1] < 0.4)]
    _feed_static(g, pts=wall_shadowed + occluder)
    # o trecho sombreado "aparece" (robô moveu / borda da sombra varre)
    revealed = [(2.0, 0.0), (2.0, 0.05), (2.0, -0.05), (2.0, 0.1)]
    t = 0.8
    for i in range(g.cfg.persist_frames):
        g.observe(t + i * 0.1, wall_shadowed + occluder + revealed, POSE, 0.0)
    assert g.moving_clusters == []          # feixe velho batia no occluder
    assert g.filter(t + 0.2, 0.30, 0.0)[2] == 'idle'


def _polar_of(pts, bin_deg=1.0, drop_bins=()):
    """mapa polar como o nó monta do scan COMPLETO: bin->maior alcance;
    feixe dropado/inválido = 0.0 (desconhecido, nunca 'livre')."""
    pol = {}
    for p in pts:
        b = int(math.floor(math.degrees(math.atan2(p[1], p[0])) / bin_deg))
        pol[b] = max(pol.get(b, 0.0), math.hypot(p[0], p[1]))
    for b in drop_bins:
        pol[b] = 0.0
    return pol


def test_beam_dropout_reappearing_wall_not_moving():
    # FALSO residual de campo 07-03 (CSV diagnóstico): 25% das detecções com o
    # robô PARADO, clusters atrás/do lado (rasante), nos MESMOS lugares =
    # feixe do LD06 dropa em superfície rasante e VOLTA segundos depois. Ao
    # voltar, a célula estava ausente e o bin do raycast vazio ("livre") ->
    # virava móvel sustentado. Com o polar do scan COMPLETO o dropout entra
    # como alcance 0.0 = DESCONHECIDO -> não valida movimento.
    g = _guard()
    wall_gap = [p for p in WALL if not (-0.2 < p[1] < 0.2)]   # setor dropado
    drop = range(-8, 8)          # bins ~bearing 0° (onde a parede sumiu)
    for i in range(8):
        g.observe(i * 0.1, wall_gap, POSE, 0.0,
                  polar=_polar_of(wall_gap, drop_bins=drop))
    t = 0.8
    for i in range(g.cfg.persist_frames):     # o feixe volta: parede reaparece
        g.observe(t + i * 0.1, WALL, POSE, 0.0, polar=_polar_of(WALL))
    assert g.moving_clusters == []
    assert g.filter(t + 0.2, 0.30, 0.0)[2] == 'idle'


def test_mover_detected_with_full_polar():
    # contraprova com polar explícito: pessoa aparece onde o feixe velho
    # ATRAVESSAVA (batia na parede atrás) -> detecta normal
    g = _guard()
    for i in range(8):
        g.observe(i * 0.1, WALL, POSE, 0.0, polar=_polar_of(WALL))
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    t = 0.8
    for i in range(g.cfg.persist_frames):
        g.observe(t + i * 0.1, WALL + obj, POSE, 0.0,
                  polar=_polar_of(WALL + obj))
    assert len(g.moving_clusters) == 1
    assert g.filter(t + 0.2, 0.30, 0.0)[2] == 'blocked'


def test_mover_in_observed_free_space_still_detected():
    # contraprova do raycast: pessoa aparece onde o feixe velho PASSAVA
    # (batia na parede bem atrás / não batia em nada) -> segue detectada
    g = _guard()
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]   # na frente da parede x=2
    for i in range(g.cfg.persist_frames):
        g.observe(t + i * 0.1, WALL + obj, POSE, 0.0)
    assert len(g.moving_clusters) == 1
    assert g.filter(t + 0.2, 0.30, 0.0)[2] == 'blocked'


def test_flicker_single_frame_does_not_latch():
    # FALSO POSITIVO de campo 07-03: TF atrasado + borda de oclusão fazem
    # parede MAPEADA "piscar" como móvel por 1 frame enquanto o robô anda ->
    # o guard ficava 100% do tempo em slowing/blocked sem ninguém perto.
    # 1 frame isolado (ou não-consecutivo) = ruído, NÃO latcha.
    g = _guard()
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    g.observe(t, WALL + obj, POSE, 0.0)            # 1 frame só
    assert g.filter(t, 0.30, 0.0)[2] == 'idle'
    g.observe(t + 0.1, WALL, POSE, 0.0)            # sumiu
    g.observe(t + 0.2, WALL + obj, POSE, 0.0)      # voltou (não-consecutivo)
    assert g.filter(t + 0.2, 0.30, 0.0)[2] == 'idle'


def test_persistent_mover_latches_after_persist_frames():
    g = _guard(persist_frames=3)
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]     # no corredor
    g.observe(t, WALL + obj, POSE, 0.0)
    g.observe(t + 0.1, WALL + obj, POSE, 0.0)
    assert g.filter(t + 0.1, 0.30, 0.0)[2] == 'idle'    # 2 < persist_frames
    g.observe(t + 0.2, WALL + obj, POSE, 0.0)
    assert g.filter(t + 0.2, 0.30, 0.0)[2] == 'blocked'  # 3º consecutivo latcha


def test_filter_slowing_scales_vx_only():
    g = _guard()
    t = _feed_static(g)
    obj = [(0.5, -1.5), (0.5, -1.4), (0.6, -1.5)]   # móvel perto, FORA do corredor
    t = _feed_mover(g, t, obj)
    vx, wz, st = g.filter(t, 0.30, 2.4)
    assert st == 'slowing'
    assert 0.30 * 0.25 < vx < 0.30        # escala fica entre o piso e o cheio
    assert wz == 2.4                      # wz NUNCA é escalado (só o cap corta)


def test_filter_slowing_caps_wz():
    # giro CALMO perto de gente (dono 07-10: girava a 4.0-4.5 do lado de
    # pessoa). CAP em slow_wz_cap — nunca escala (zona-morta do skid).
    g = _guard()
    t = _feed_static(g)
    obj = [(0.5, -1.5), (0.5, -1.4), (0.6, -1.5)]
    t = _feed_mover(g, t, obj)
    assert g.filter(t, 0.30, 4.5)[1] == 2.4
    assert g.filter(t, 0.30, -4.5)[1] == -2.4
    assert g.filter(t, 0.30, 1.0)[1] == 1.0     # abaixo do cap: intacto


def test_filter_slow_proportional_to_distance():
    # mais PERTO = mais devagar (o "vindo na minha direção" vira freio
    # progressivo), na faixa entre a bolha (freeze_dist) e o raio.
    def vx_with_obj_at(d):
        g = _guard()
        t = _feed_static(g)
        obj = [(0.0, -d), (0.0, -d - 0.1), (0.1, -d)]   # ao LADO, fora do corredor
        t = _feed_mover(g, t, obj)
        vx, _, st = g.filter(t, 0.30, 0.0)
        assert st == 'slowing'
        return vx

    # borda do raio subiu p/ 3.5 (dono 07-09): a 3.2m quase não freia; a
    # 2.2m/1.3m já freia progressivo (faixa de cautela agora maior).
    far, mid, near = vx_with_obj_at(3.2), vx_with_obj_at(1.7), vx_with_obj_at(1.3)
    assert far > mid > near               # monotônico com a distância
    assert far > 0.30 * 0.7               # perto da borda do raio quase não freia


def test_defaults_catch_path_crossers_and_settle():
    # dono 07-02 (3ª rodada real): cruzava o CAMINHO além do corredor de 1.5m
    # -> o follower saía atrás do desvio-fantasma do planner. Corredor cobre
    # o raio (2.5). Retomada 3.0→5.0s (dono 07-09): se é gente, espera mais
    # antes de voltar a andar.
    cfg = GuardConfig()
    assert cfg.corridor_len == 2.5
    assert cfg.clear_time == 5.0


def test_filter_freeze_bubble_full_stop_any_direction():
    # BOLHA (dono 07-02, 2ª rodada real): móvel se mexendo a <freeze_dist
    # (1.2m) em QUALQUER direção -> parada total, mesmo fora do corredor.
    # Antes: pessoa do LADO deixava o giro liberado (slowing) e o robô
    # rodava atrás do plano-contorno enquanto ela passava ("ficar maluco").
    g = _guard()
    t = _feed_static(g)
    obj = [(0.0, -0.9), (0.0, -1.0), (0.1, -0.9)]   # do LADO, colado
    t = _feed_mover(g, t, obj)
    vx, wz, st = g.filter(t, 0.30, 2.4)
    assert (vx, wz, st) == (0.0, 0.0, 'blocked')


def test_filter_blocked_full_stop_including_wz():
    # blocked = parada TOTAL (dono 07-02: com wz liberado o replan do nav2
    # balançava o caminho e o robô ficava GIRANDO no lugar enquanto a pessoa
    # ainda passava — "para de pensar" até o corredor limpar). Zerar wz é
    # seguro (o perigo da zona-morta é ESCALAR, não zerar).
    g = _guard()
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]      # no corredor
    t = _feed_mover(g, t, obj)
    vx, wz, st = g.filter(t, 0.30, 2.4)
    assert (vx, wz, st) == (0.0, 0.0, 'blocked')


def test_filter_blocked_does_not_zero_reverse():
    g = _guard()
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    t = _feed_mover(g, t, obj)
    vx, wz, st = g.filter(t, -0.25, 1.0)  # ré (afasta do móvel) não é bloqueada
    assert st == 'blocked' and vx == -0.25
    assert wz == 0.0                      # mas o giro para mesmo assim


def test_filter_resumes_after_clear_time():
    g = _guard(clear_time=1.5)   # timing do teste independe do default
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    tl = _feed_mover(g, t, obj)
    assert g.filter(tl + 0.8, 0.30, 0.0)[2] == 'blocked'   # dentro do clear_time
    g.observe(tl + 0.8, WALL, POSE, 0.0)                    # corredor limpo
    g.observe(tl + 2.4, WALL, POSE, 0.0)                    # scans seguem chegando
    vx, _, st = g.filter(tl + 2.4, 0.30, 0.0)               # >clear_time s/ móvel
    assert st == 'idle' and vx == 0.30


def test_filter_wz_gate_holds_then_decays():
    g = _guard(clear_time=1.5)   # timing do teste independe do default
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    tl = _feed_mover(g, t, obj)                             # blocked
    g.observe(tl + 0.1, WALL + obj, POSE, 2.0)              # girando: NÃO avalia
    assert g.filter(tl + 0.2, 0.30, 2.0)[2] == 'blocked'    # decisão segurada
    # muito tempo girando sem avaliação -> decai pra livre (clear_time 1.5
    # depois da última vista do móvel; gated não re-avista)
    for i in range(30):
        g.observe(tl + 0.2 + i * 0.1, WALL + obj, POSE, 2.0)
    vx, _, st = g.filter(tl + 3.5, 0.30, 2.0)
    assert st == 'idle' and vx == 0.30


def test_filter_passthrough_when_scan_stale():
    g = _guard()
    t = _feed_static(g)
    vx, wz, st = g.filter(t + 5.0, 0.30, 1.0)   # 5s sem scan > scan_stale 1.0
    assert (vx, wz, st) == (0.30, 1.0, 'passthrough')


def test_filter_passthrough_when_disabled():
    g = _guard(enabled=False)
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    g.observe(t, WALL + obj, POSE, 0.0)
    vx, wz, st = g.filter(t, 0.30, 1.0)
    assert (vx, wz, st) == (0.30, 1.0, 'passthrough')


def _grid_map(wall_x=None, w=100, h=100, res=0.05, ox=-2.5, oy=-2.5):
    """OccupancyGrid sintético: livre (0) com coluna de parede em x=wall_x
    (2 células ≈ 10cm, como as paredes virtuais desenhadas no hotmilk)."""
    grid = [0] * (w * h)
    if wall_x is not None:
        cx = int((wall_x - ox) / res)
        for cy in range(h):
            grid[cy * w + cx] = 100
            grid[cy * w + cx + 1] = 100
    return MapGhostFilter(grid, w, h, res, ox, oy)


def test_ghost_filter_sees_through_wall():
    f = _grid_map(wall_x=1.0)
    assert f.sees_through_wall((0.0, 0.0), (2.0, 0.0)) is True      # atravessa
    assert f.sees_through_wall((0.0, 0.0), (0.8, 0.0)) is False     # aquém
    assert f.sees_through_wall((0.0, 0.0), (0.0, 2.0)) is False     # paralelo


def test_ghost_filter_person_against_wall_kept():
    # pessoa REAL encostada na parede mapeada: o raio corre no livre e só
    # tocaria a parede na ponta final (end_margin) -> NÃO descarta.
    f = _grid_map(wall_x=1.0)
    assert f.sees_through_wall((0.0, 0.0), (0.97, 0.0)) is False


def test_ghost_filter_unknown_and_offmap_not_wall():
    f = _grid_map(wall_x=None)          # sem parede nenhuma
    assert f.sees_through_wall((0.0, 0.0), (2.0, 0.0)) is False
    assert f.sees_through_wall((0.0, 0.0), (50.0, 0.0)) is False    # sai do mapa
    g = _grid_map(wall_x=None)
    g.grid = [-1] * (g.w * g.h)         # tudo DESCONHECIDO
    assert g.sees_through_wall((0.0, 0.0), (2.0, 0.0)) is False


def _identity_tf():
    return (0.0, 0.0, 1.0, 0.0)         # odom == map (tx, ty, cos, sin)


def test_mover_behind_glass_wall_ignored():
    # O FANTASMA da run hotmilk 07-08 (29/52 paradas): LD06 vê pessoa ATRAVÉS
    # do vidro; no mapa ali é parede virtual -> guard descarta, segue idle.
    # (móvel a 1.0m = o mesmo dos testes de detecção; a ÚNICA diferença é a
    # parede do MAPA em x=0.5 no meio do raio)
    g = _guard()
    g.ghost_map = _grid_map(wall_x=0.5)
    g.map_tf = _identity_tf()
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    t = _feed_mover(g, t, obj)
    assert g.moving_clusters == []
    assert g.ghost_dropped == 3           # os 3 pts caíram no anti-vidro
    assert g.filter(t, 0.30, 0.0)[2] == 'idle'


def test_mover_in_free_space_unaffected_by_map():
    # contraprova: mesma pessoa SEM parede no meio -> comportamento intacto
    g = _guard()
    g.ghost_map = _grid_map(wall_x=None)
    g.map_tf = _identity_tf()
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    t = _feed_mover(g, t, obj)
    assert len(g.moving_clusters) == 1
    assert g.ghost_dropped == 0
    assert g.filter(t, 0.30, 0.0)[2] == 'blocked'


def test_ghost_filter_inert_without_map_or_tf():
    # failsafe: sem /map (ou sem TF map<-odom) o guard age como pré-07-08 —
    # detecta normal, nunca deixa de frear por falta de mapa.
    for ghost_map, map_tf in ((None, _identity_tf()),
                              (_grid_map(wall_x=0.5), None)):
        g = _guard()
        g.ghost_map, g.map_tf = ghost_map, map_tf
        t = _feed_static(g)
        obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
        t = _feed_mover(g, t, obj)
        assert len(g.moving_clusters) == 1
        assert g.filter(t, 0.30, 0.0)[2] == 'blocked'


def test_ghost_filter_disabled_by_param():
    g = _guard(map_filter=False)
    g.ghost_map = _grid_map(wall_x=0.5)
    g.map_tf = _identity_tf()
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    t = _feed_mover(g, t, obj)
    assert len(g.moving_clusters) == 1    # param desliga o filtro, não o guard


def test_ghost_filter_respects_map_tf():
    # map deslocado do odom: ponto em odom (1,0) vira (1,-2) no map — a
    # parede em x=0.5 do MAP continua no meio do raio -> descarta do mesmo
    # jeito (a conta é toda no frame map).
    g = _guard()
    g.ghost_map = _grid_map(wall_x=0.5)
    g.map_tf = (0.0, -2.0, 1.0, 0.0)      # map = odom deslocado -2 em y
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    t = _feed_mover(g, t, obj)
    assert g.moving_clusters == []


def test_no_snapshot_while_turning():
    """8ª auditoria A4: girando (|wz| > wz_gate) o scan é BORRADO (TF atrasa) e
    NÃO pode virar snapshot — antes ele entrava no deque e virava a referência
    "old" logo após o giro (falso móvel pós-giro). Consequência observável:
    quem APARECE durante o giro é comparado com o snapshot PRÉ-giro (limpo) e
    detectado como móvel assim que o giro termina."""
    g = _guard()
    t = _feed_static(g)                      # histórico limpo, sem obj
    obj = [(1.0, 0.8), (1.0, 0.9), (1.1, 0.8), (1.1, 0.9)]
    n_snaps = len(g._snaps)
    # pessoa chega DURANTE o giro: gated (não avalia) e agora nem snapshotta
    for i in range(5):
        g.observe(t + i * 0.1, WALL + obj, POSE, 2.0)
    assert len(g._snaps) == n_snaps          # nada snapshotado girando
    assert g.moving_clusters == []           # nem avaliado
    # giro terminou: o "old" é o pré-giro (obj ausente) -> obj = MÓVEL
    g.observe(t + 0.5, WALL + obj, POSE, 0.0)
    assert len(g.moving_clusters) == 1


# ---------------------------------------------------------------- vigília
# "PAROU-MAS-ESTÁ-LÁ" (dono 07-10): móvel que BLOQUEOU (bolha/corredor) e
# parou de se mexer NÃO some — o guard vigia o lugar e segura o blocked
# enquanto o scan mostrar ocupação ali (teto hold_still_max); saiu -> solta
# pelo clear_time normal. Campo 07-10: pessoa parada ficava invisível em ~1s
# e o robô voltava a empurrar pra cima dela (unstuck bateu no tênis do dono).

PERSON = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]      # no corredor, dentro da bolha


def test_stopped_blocker_still_there_keeps_blocking():
    g = _guard()                                     # clear_time default 5.0
    t = _feed_static(g)
    tl = _feed_mover(g, t, PERSON)
    # pessoa PAROU no lugar: 8s (>clear_time) de scans com ela imóvel
    for i in range(1, 81):
        g.observe(tl + i * 0.1, WALL + PERSON, POSE, 0.0)
    vx, wz, st = g.filter(tl + 8.0, 0.30, 1.0)
    assert (vx, wz, st) == (0.0, 0.0, 'blocked')


def test_stopped_blocker_leaving_releases_by_clear_time():
    g = _guard(clear_time=1.5)
    t = _feed_static(g)
    tl = _feed_mover(g, t, PERSON)
    for i in range(1, 31):                           # parada 3s (>clear_time)
        g.observe(tl + i * 0.1, WALL + PERSON, POSE, 0.0)
    assert g.filter(tl + 3.0, 0.30, 0.0)[2] == 'blocked'    # vigília segurando
    td = tl + 3.1
    for i in range(25):                              # pessoa SAIU
        g.observe(td + i * 0.1, WALL, POSE, 0.0)
    vx, _, st = g.filter(td + 2.0, 0.30, 0.0)        # clear_time depois: anda
    assert st == 'idle' and vx == 0.30


def test_stopped_blocker_watch_has_ceiling():
    g = _guard(clear_time=1.5, hold_still_max=3.0)
    t = _feed_static(g)
    tl = _feed_mover(g, t, PERSON)
    for i in range(1, 81):                           # pessoa fica 8s, imóvel
        g.observe(tl + i * 0.1, WALL + PERSON, POSE, 0.0)
    assert g.filter(tl + 3.0, 0.30, 0.0)[2] == 'blocked'    # teto ainda não
    vx, _, st = g.filter(tl + 6.0, 0.30, 0.0)        # teto+clear_time passados
    assert st == 'idle' and vx == 0.30


def test_stopped_far_lateral_mover_not_watched():
    # móvel que só causou SLOWING (longe da bolha, fora do corredor) não
    # ganha vigília: parou -> decai pelo clear_time como sempre.
    g = _guard(clear_time=1.5)
    t = _feed_static(g)
    obj = [(0.5, -2.0), (0.5, -2.1), (0.6, -2.0)]
    tl = _feed_mover(g, t, obj)
    assert g.filter(tl, 0.30, 0.0)[2] == 'slowing'
    for i in range(1, 31):
        g.observe(tl + i * 0.1, WALL + obj, POSE, 0.0)
    assert g.filter(tl + 3.0, 0.30, 0.0)[2] == 'idle'


def test_watch_ignores_mapped_wall_points():
    # pessoa parou COLADA numa parede do MAPA e depois saiu: o que sobra no
    # raio da vigília é parede mapeada, não presença -> solta pelo
    # clear_time, sem esperar o teto.
    g = _guard(clear_time=1.5)
    g.ghost_map = _grid_map(wall_x=1.3)
    g.map_tf = _identity_tf()
    wall2 = [(1.28, -0.2), (1.28, -0.1), (1.28, 0.1)]   # retornos da parede
    # pessoa a ~0.35m da parede: célula NÃO-vizinha da wall2 (vizinha, o diff
    # de grade come pontos do cluster e nem latcha)
    person = [(0.9, 0.0), (0.9, 0.1), (0.95, 0.05)]
    t = _feed_static(g, pts=WALL + wall2)
    for i in range(g.cfg.persist_frames):            # pessoa chega (latcha)
        g.observe(t + i * 0.1, WALL + wall2 + person, POSE, 0.0)
    tl = t + (g.cfg.persist_frames - 1) * 0.1
    assert g.filter(tl, 0.30, 0.0)[2] == 'blocked'
    for i in range(1, 11):                           # parada 1s
        g.observe(tl + i * 0.1, WALL + wall2 + person, POSE, 0.0)
    td = tl + 1.1
    for i in range(25):                              # pessoa SAIU; parede fica
        g.observe(td + i * 0.1, WALL + wall2, POSE, 0.0)
    vx, _, st = g.filter(td + 2.0, 0.30, 0.0)
    assert st == 'idle' and vx == 0.30


# ------------------------------------------------------- fantasma de parede
# Campo 07-10 (corredor reto do hotmilk): transladando rápido, feixe rasante
# + erro de pose faz trecho da PAREDE cair em bin "livre 0.5s atrás" -> vira
# móvel a <1m -> bolha -> parada SECA repetida (cluster ACOMPANHAVA o robô,
# colado na parede). Cluster com quase todos os pontos EM CIMA de parede
# MAPEADA não é gente -> descarta antes de latchar.


def test_wall_ghost_cluster_dropped():
    g = _guard()
    g.ghost_map = _grid_map(wall_x=1.0)
    g.map_tf = _identity_tf()
    t = _feed_static(g)
    # "móvel" novo com TODOS os pontos na linha da parede mapeada (x=1.0)
    ghost = [(1.02, -0.1), (1.02, 0.0), (1.03, 0.1)]
    for i in range(g.cfg.persist_frames):
        g.observe(t + i * 0.1, WALL + ghost, POSE, 0.0)
    assert g.moving_clusters == []
    assert g.wall_dropped == 1
    assert g.filter(t + 0.2, 0.30, 0.0)[2] == 'idle'   # NÃO para seco


def test_person_near_wall_still_blocks():
    # pessoa ENCOSTADA na parede: o corpo sobra pra fora da linha do mapa
    # (fração na parede < limiar) -> continua latchando (bolha protege).
    g = _guard()
    g.ghost_map = _grid_map(wall_x=1.0)
    g.map_tf = _identity_tf()
    t = _feed_static(g)
    person = [(0.75, -0.05), (0.75, 0.05), (0.8, 0.0), (0.97, 0.0)]
    for i in range(g.cfg.persist_frames):
        g.observe(t + i * 0.1, WALL + person, POSE, 0.0)
    assert len(g.moving_clusters) == 1
    assert g.filter(t + 0.2, 0.30, 0.0)[2] == 'blocked'


def test_wall_ghost_kept_without_map():
    # failsafe: sem /map (ou sem TF) o filtro não atua — comportamento antigo
    # (melhor freio falso que freio nenhum de verdade).
    g = _guard()
    t = _feed_static(g)
    ghost = [(1.02, -0.1), (1.02, 0.0), (1.03, 0.1)]
    for i in range(g.cfg.persist_frames):
        g.observe(t + i * 0.1, WALL + ghost, POSE, 0.0)
    assert len(g.moving_clusters) == 1
    assert g.wall_dropped == 0


# ---- FaceStateFile (cara fase 2: rumo da pessoa pro face_web) -----------

def test_face_state_file_grava_e_throttla(tmp_path):
    import json
    from robot_nav.motion_guard import FaceStateFile
    p = str(tmp_path / 'face.json')
    w = FaceStateFile(path=p, min_period=0.2)
    assert w.update(10.0, 30) is True
    assert json.load(open(p)) == {'ts': 10.0, 'cbear_deg': 30, 'state': None}
    assert w.update(10.1, 35) is False           # dentro do throttle
    assert w.update(10.3, 35) is True            # passou 0.2s
    assert json.load(open(p))['cbear_deg'] == 35


def test_face_state_file_grava_estado_do_guard(tmp_path):
    # 'blocked' vai junto -> face_web pede "com licença" (cara fase 2.1)
    import json
    from robot_nav.motion_guard import FaceStateFile
    p = str(tmp_path / 'face.json')
    w = FaceStateFile(path=p, min_period=0.2)
    assert w.update(10.0, 20, state='blocked') is True
    assert json.load(open(p)) == \
        {'ts': 10.0, 'cbear_deg': 20, 'state': 'blocked'}
    assert w.update(10.3, 20, state='slowing') is True
    assert json.load(open(p))['state'] == 'slowing'


def test_face_state_file_transicao_null_uma_vez(tmp_path):
    import json
    from robot_nav.motion_guard import FaceStateFile
    p = str(tmp_path / 'face.json')
    w = FaceStateFile(path=p, min_period=0.2)
    assert w.update(10.0, None) is False         # sem pessoa antes: nada
    w.update(10.0, 30)
    assert w.update(10.05, None) is True         # transição FURA o throttle
    assert json.load(open(p))['cbear_deg'] is None
    assert w.update(10.1, None) is False         # já silenciou


def test_face_state_file_io_error_nao_propaga(tmp_path):
    from robot_nav.motion_guard import FaceStateFile
    w = FaceStateFile(path=str(tmp_path / 'nao_existe' / 'face.json'))
    assert w.update(10.0, 30) is False           # dir não existe: engole
    assert w.last_error
