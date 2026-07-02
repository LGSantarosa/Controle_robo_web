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
