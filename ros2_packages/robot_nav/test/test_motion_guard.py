"""Testes da lógica pura do motion_guard (sem ROS)."""
import math

from robot_nav.motion_guard import GuardConfig, MotionGuard

POSE = (0.0, 0.0, 0.0)   # robô na origem olhando +x (frame odom)
WALL = [(2.0, y * 0.1 - 1.0) for y in range(20)]   # parede estática em x=2


def _guard(**kw):
    return MotionGuard(GuardConfig(**kw))


def _feed_static(g, t0=0.0, n=8, dt=0.1, pts=WALL):
    """alimenta n scans estáticos p/ encher o histórico (lookback 0.5s)."""
    for i in range(n):
        g.observe(t0 + i * dt, pts, POSE, 0.0)
    return t0 + n * dt


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


def test_filter_slowing_scales_vx_only():
    g = _guard()
    t = _feed_static(g)
    obj = [(0.5, -1.5), (0.5, -1.4), (0.6, -1.5)]   # móvel perto, FORA do corredor
    g.observe(t, WALL + obj, POSE, 0.0)
    vx, wz, st = g.filter(t, 0.30, 2.4)
    assert st == 'slowing'
    assert 0.30 * 0.25 < vx < 0.30        # escala fica entre o piso e o cheio
    assert wz == 2.4                      # wz NUNCA muda


def test_filter_slow_proportional_to_distance():
    # mais PERTO = mais devagar (o "vindo na minha direção" vira freio
    # progressivo), na faixa entre a bolha (freeze_dist) e o raio.
    def vx_with_obj_at(d):
        g = _guard()
        t = _feed_static(g)
        obj = [(0.0, -d), (0.0, -d - 0.1), (0.1, -d)]   # ao LADO, fora do corredor
        g.observe(t, WALL + obj, POSE, 0.0)
        vx, _, st = g.filter(t, 0.30, 0.0)
        assert st == 'slowing'
        return vx

    far, mid, near = vx_with_obj_at(2.2), vx_with_obj_at(1.7), vx_with_obj_at(1.3)
    assert far > mid > near               # monotônico com a distância
    assert far > 0.30 * 0.7               # na borda do raio quase não freia


def test_filter_freeze_bubble_full_stop_any_direction():
    # BOLHA (dono 07-02, 2ª rodada real): móvel se mexendo a <freeze_dist
    # (1.2m) em QUALQUER direção -> parada total, mesmo fora do corredor.
    # Antes: pessoa do LADO deixava o giro liberado (slowing) e o robô
    # rodava atrás do plano-contorno enquanto ela passava ("ficar maluco").
    g = _guard()
    t = _feed_static(g)
    obj = [(0.0, -0.9), (0.0, -1.0), (0.1, -0.9)]   # do LADO, colado
    g.observe(t, WALL + obj, POSE, 0.0)
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
    g.observe(t, WALL + obj, POSE, 0.0)
    vx, wz, st = g.filter(t, 0.30, 2.4)
    assert (vx, wz, st) == (0.0, 0.0, 'blocked')


def test_filter_blocked_does_not_zero_reverse():
    g = _guard()
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    g.observe(t, WALL + obj, POSE, 0.0)
    vx, wz, st = g.filter(t, -0.25, 1.0)  # ré (afasta do móvel) não é bloqueada
    assert st == 'blocked' and vx == -0.25
    assert wz == 0.0                      # mas o giro para mesmo assim


def test_filter_resumes_after_clear_time():
    g = _guard()
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    g.observe(t, WALL + obj, POSE, 0.0)
    assert g.filter(t + 1.0, 0.30, 0.0)[2] == 'blocked'   # dentro do clear_time
    g.observe(t + 1.0, WALL, POSE, 0.0)                    # corredor limpo
    g.observe(t + 2.6, WALL, POSE, 0.0)                    # scans seguem chegando
    vx, _, st = g.filter(t + 2.6, 0.30, 0.0)               # >clear_time s/ móvel
    assert st == 'idle' and vx == 0.30


def test_filter_wz_gate_holds_then_decays():
    g = _guard()
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    g.observe(t, WALL + obj, POSE, 0.0)                    # blocked
    g.observe(t + 0.1, WALL + obj, POSE, 2.0)              # girando: NÃO avalia
    assert g.filter(t + 0.2, 0.30, 2.0)[2] == 'blocked'    # decisão segurada
    # muito tempo girando sem avaliação -> decai pra livre (clear_time 1.5
    # depois da última vista do móvel; gated não re-avista)
    for i in range(30):
        g.observe(t + 0.2 + i * 0.1, WALL + obj, POSE, 2.0)
    vx, _, st = g.filter(t + 3.5, 0.30, 2.0)
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
