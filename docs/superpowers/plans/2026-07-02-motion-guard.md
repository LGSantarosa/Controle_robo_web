# motion_guard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Nó `motion_guard` que detecta coisa EM MOVIMENTO perto do robô (diff temporal do `/scan_safe` no frame odom), desacelera a autonomia e para se o móvel estiver no corredor à frente, retomando sozinho.

**Architecture:** Lógica pura `MotionGuard` (testável sem ROS) + `main()` de cola, padrão do repo (scan_sanitizer/path_follower). Filtro de velocidade na cadeia: `twist_mux_auto → auto_vel_pre → motion_guard → auto_vel_raw → collision_monitor`. Spec: `docs/superpowers/specs/2026-07-02-motion-guard-design.md`.

**Tech Stack:** Python/rclpy, numpy, pytest, gz Harmonic (validação com caixa móvel via VelocityControl).

## Global Constraints

- Commits SEM rodapé de co-autoria (preferência do dono).
- `angular.z` passa INTOCADO sempre (zona-morta 1.7 congela point-turn).
- Failsafe: TF/scan indisponível ou `enabled=false` → pass-through. O guard NUNCA mata a autonomia.
- NÃO mexer: YAML do collision_monitor, unstuck, mux final, path_follower.
- Defaults da spec: `guard_radius=2.5, slow_scale=0.5, corridor_half_w=0.35, corridor_len=1.5, clear_time=1.5, grid_res=0.15, lookback=0.5, min_cluster_points=3, wz_gate=0.3, scan_stale=1.0` (o decaimento pós-gate de giro é o próprio `clear_time` — sem param extra).
- Robô real fica FORA (deploy/validação real é decisão do dono).

---

### Task 1: Lógica pura — detecção de movimento + clusters

**Files:**
- Create: `ros2_packages/robot_nav/robot_nav/motion_guard.py`
- Test: `ros2_packages/robot_nav/test/test_motion_guard.py`

**Interfaces:**
- Produces: `GuardConfig` (dataclass mutável, campos = defaults da spec); `MotionGuard(cfg)` com `observe(t: float, pts: list[tuple[float,float]], pose: tuple[float,float,float], wz: float) -> None` (pts e pose no frame odom) e atributos pós-observe: `moving_clusters: list[list[tuple]]`, `nearest_moving: float` (inf se nada), `in_corridor: bool`.

- [ ] **Step 1: Testes de detecção (falhando)**

```python
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
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `cd ros2_packages/robot_nav && python3 -m pytest test/test_motion_guard.py -v`
Expected: FAIL/ERROR em todos (`ModuleNotFoundError: robot_nav.motion_guard`).

- [ ] **Step 3: Implementar `GuardConfig` + `MotionGuard.observe`**

Criar `ros2_packages/robot_nav/robot_nav/motion_guard.py`:

```python
#!/usr/bin/env python3
"""motion_guard — cautela com objeto EM MOVIMENTO perto do robô.

Por que existe (pedido do dono pós-run 2026-07-02): nada na stack distingue
móvel de estático — o collision_monitor é reativo instantâneo (freia quando
algo JÁ está na frente). Este nó compara scans no frame ODOM: o que é
estático (parede, móvel parado) fica na mesma célula; célula que estava LIVRE
~0.5s atrás e agora tem retorno = borda de ataque de coisa se movendo.

Atuação (filtro de velocidade, só autonomia):
    twist_mux_auto -> auto_vel_pre -> [motion_guard] -> auto_vel_raw
        -> collision_monitor -> auto_vel -> mux final
  - móvel no raio guard_radius  -> linear.x *= slow_scale   (slowing)
  - móvel no corredor à frente  -> linear.x = 0 até limpar clear_time (blocked)
  - angular.z passa INTOCADO SEMPRE (escalar wz cai na zona-morta 1.7 e
    congela o point-turn — lição do rot_min 07-02).
  - TF/scan indisponível ou enabled=false -> PASS-THROUGH (nunca mata a nav).

SEM predição de cruzamento por enquanto (proposta B da spec): os pontos
móveis já saem clusterizados pra plugar velocidade+predição depois se a
versão A reagir tarde em campo.

Spec: docs/superpowers/specs/2026-07-02-motion-guard-design.md
A lógica (MotionGuard) é pura p/ testar sem ROS; main() é a cola de I/O.
"""
import math
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

Pt = Tuple[float, float]


@dataclass
class GuardConfig:
    enabled: bool = True
    guard_radius: float = 2.5       # m — só olha móvel até aqui
    slow_scale: float = 0.5         # fator no vx com móvel no raio
    corridor_half_w: float = 0.35   # m — meia-largura do corredor à frente
    corridor_len: float = 1.5       # m — alcance do corredor
    clear_time: float = 1.5         # s — corredor limpo por isso -> retoma
    grid_res: float = 0.15          # m — célula da grade de comparação
    lookback: float = 0.5           # s — compara com snapshot desta idade
    min_cluster_points: int = 3     # cluster menor = ruído
    cluster_gap: float = 0.3        # m — distância máx p/ mesmo cluster
    wz_gate: float = 0.3            # rad/s — girando acima disso não avalia
    scan_stale: float = 1.0         # s sem scan -> pass-through


class MotionGuard:
    """Detector de movimento por diff temporal em grade (frame odom).

    observe() processa um scan; filter() (Task 2) aplica a decisão no comando.
    """

    def __init__(self, cfg: GuardConfig):
        self.cfg = cfg
        self._snaps = deque()            # (t, frozenset de células)
        self.moving_clusters: List[List[Pt]] = []
        self.nearest_moving: float = math.inf
        self.in_corridor: bool = False
        self._last_moving_t: float = -math.inf
        self._last_corridor_t: float = -math.inf
        self._last_scan_t: float = -math.inf

    def _cell(self, p: Pt) -> Tuple[int, int]:
        r = self.cfg.grid_res
        return (int(math.floor(p[0] / r)), int(math.floor(p[1] / r)))

    def _old_snapshot(self, t: float):
        """último snapshot com idade >= lookback (descarta os mais velhos)."""
        c = self.cfg
        old = None
        while self._snaps and t - self._snaps[0][0] >= c.lookback:
            old = self._snaps.popleft()
        if old is not None:
            self._snaps.appendleft(old)   # mantém p/ os próximos ticks
        return old

    def observe(self, t: float, pts: List[Pt],
                pose: Tuple[float, float, float], wz: float) -> None:
        c = self.cfg
        self._last_scan_t = t
        cells = frozenset(self._cell(p) for p in pts)
        self._snaps.append((t, cells))

        # GATE DE GIRO: girando, o scan inteiro "anda" (pose/TF atrasam) ->
        # não avalia; a decisão anterior decai via hold_timeout (filter).
        if abs(wz) > c.wz_gate:
            return
        old = self._old_snapshot(t)
        if old is None:
            return                      # histórico curto demais ainda
        _, old_cells = old

        px, py, pyaw = pose
        r2 = c.guard_radius ** 2
        moving: List[Pt] = []
        for p in pts:
            if (p[0] - px) ** 2 + (p[1] - py) ** 2 > r2:
                continue
            cx, cy = self._cell(p)
            # célula (ou vizinha imediata) ocupada antes -> estático
            if any((cx + dx, cy + dy) in old_cells
                   for dx in (-1, 0, 1) for dy in (-1, 0, 1)):
                continue
            moving.append(p)

        clusters = self._cluster(moving)
        clusters = [cl for cl in clusters
                    if len(cl) >= c.min_cluster_points]
        self.moving_clusters = clusters
        self.nearest_moving = min(
            (math.hypot(p[0] - px, p[1] - py) for cl in clusters for p in cl),
            default=math.inf)

        # corredor à frente em base_link: xb à frente, yb lateral
        cos_y, sin_y = math.cos(pyaw), math.sin(pyaw)
        self.in_corridor = False
        for cl in clusters:
            for p in cl:
                dx, dy = p[0] - px, p[1] - py
                xb = dx * cos_y + dy * sin_y
                yb = -dx * sin_y + dy * cos_y
                if 0.0 < xb <= c.corridor_len and abs(yb) <= c.corridor_half_w:
                    self.in_corridor = True
                    break
            if self.in_corridor:
                break
        if clusters:
            self._last_moving_t = t
        if self.in_corridor:
            self._last_corridor_t = t

    def _cluster(self, pts: List[Pt]) -> List[List[Pt]]:
        """agrupamento single-link por distância <= cluster_gap (N pequeno)."""
        gap2 = self.cfg.cluster_gap ** 2
        clusters: List[List[Pt]] = []
        for p in pts:
            hits = [cl for cl in clusters
                    if any((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 <= gap2
                           for q in cl)]
            if not hits:
                clusters.append([p])
            else:
                hits[0].append(p)
                for other in hits[1:]:      # p uniu clusters -> merge
                    hits[0].extend(other)
                    clusters.remove(other)
        return clusters
```

- [ ] **Step 4: Rodar os testes da Task 1**

Run: `cd ros2_packages/robot_nav && python3 -m pytest test/test_motion_guard.py -v`
Expected: 7 PASS.

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/motion_guard.py ros2_packages/robot_nav/test/test_motion_guard.py
git commit -m "motion_guard: deteccao de movimento por diff temporal no frame odom (logica pura + clusters)"
```

### Task 2: Lógica pura — decisão/filtro de velocidade

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/motion_guard.py` (classe `MotionGuard`)
- Test: `ros2_packages/robot_nav/test/test_motion_guard.py`

**Interfaces:**
- Consumes: estado interno pós-`observe` (`_last_moving_t`, `_last_corridor_t`, `_last_scan_t`, `_last_eval_t`).
- Produces: `MotionGuard.filter(t: float, vx: float, wz: float) -> tuple[float, float, str]` — retorna `(vx_out, wz_out, state)` com `state in ('idle','slowing','blocked','passthrough')`; `wz_out == wz` SEMPRE.

- [ ] **Step 1: Testes do filtro (falhando)**

Acrescentar em `test/test_motion_guard.py`:

```python
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
    assert vx == 0.15                     # slow_scale 0.5
    assert wz == 2.4                      # wz NUNCA muda


def test_filter_blocked_zeroes_forward_keeps_wz():
    g = _guard()
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]      # no corredor
    g.observe(t, WALL + obj, POSE, 0.0)
    vx, wz, st = g.filter(t, 0.30, 2.4)
    assert (vx, wz, st) == (0.0, 2.4, 'blocked')


def test_filter_blocked_does_not_zero_reverse():
    g = _guard()
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    g.observe(t, WALL + obj, POSE, 0.0)
    vx, _, st = g.filter(t, -0.25, 0.0)   # ré (unstuck-like) não é bloqueada
    assert st == 'blocked' and vx == -0.25


def test_filter_resumes_after_clear_time():
    g = _guard()
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    g.observe(t, WALL + obj, POSE, 0.0)
    assert g.filter(t + 1.0, 0.30, 0.0)[2] == 'blocked'   # ainda dentro do clear_time
    g.observe(t + 1.0, WALL, POSE, 0.0)                    # corredor limpo
    vx, _, st = g.filter(t + 1.0 + 1.6, 0.30, 0.0)         # 1.6s > clear_time 1.5
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
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `cd ros2_packages/robot_nav && python3 -m pytest test/test_motion_guard.py -k filter -v`
Expected: 8 FAIL (`AttributeError: filter`).

- [ ] **Step 3: Implementar `filter`**

Acrescentar na classe `MotionGuard` (o decaimento durante o gate de giro sai
de graça: gated não re-avista o móvel → o latch expira sozinho em `clear_time`):

```python
    def filter(self, t: float, vx: float, wz: float
               ) -> Tuple[float, float, str]:
        """aplica a decisão no comando. wz NUNCA muda (zona-morta do giro)."""
        c = self.cfg
        if not c.enabled or t - self._last_scan_t > c.scan_stale:
            return vx, wz, 'passthrough'
        if t - self._last_corridor_t < c.clear_time:
            return (0.0 if vx > 0.0 else vx), wz, 'blocked'
        if t - self._last_moving_t < c.clear_time:
            return vx * c.slow_scale, wz, 'slowing'
        return vx, wz, 'idle'
```

- [ ] **Step 4: Suíte inteira**

Run: `cd ros2_packages/robot_nav && python3 -m pytest test/ -q`
Expected: tudo PASS (214 antigos + 15 novos).

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/motion_guard.py ros2_packages/robot_nav/test/test_motion_guard.py
git commit -m "motion_guard: filtro de velocidade (slowing/blocked/passthrough) — vx escala/zera, wz intocado"
```

### Task 3: Nó ROS (cola de I/O) + wiring no launch

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/motion_guard.py` (adicionar `main()`)
- Modify: `ros2_packages/robot_nav/setup.py` (entry point)
- Modify: `ros2_packages/robot_nav/launch/nav2.launch.py` (remap do mux + nó novo)

**Interfaces:**
- Consumes: `MotionGuard.observe/filter` (Tasks 1-2); `utils.spin_node`.
- Produces: executável `motion_guard`; tópicos `auto_vel_pre` (in) → `auto_vel_raw` (out), `/motion_guard/state` (String latched), CSV `controle_web/logs/motion_guard.csv`.

- [ ] **Step 1: Escrever o `main()`**

Acrescentar no fim de `motion_guard.py` (padrão do path_follower; params live via callback, lição do `04bcf86`):

```python
def main(args=None):  # pragma: no cover - cola de I/O, validar no sim
    import csv as _csv
    import os as _os

    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from rcl_interfaces.msg import SetParametersResult
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException

    from .utils import quat_to_yaw, spin_node

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    class MotionGuardNode(Node):
        # afináveis ao vivo: mutam a MESMA ref de cfg que observe/filter leem
        _CFG_PARAMS = ('enabled', 'guard_radius', 'slow_scale',
                       'corridor_half_w', 'corridor_len', 'clear_time',
                       'grid_res', 'lookback', 'min_cluster_points',
                       'cluster_gap', 'wz_gate', 'scan_stale')

        def __init__(self):
            super().__init__('motion_guard')
            cfg = GuardConfig()
            for name in self._CFG_PARAMS:
                self.declare_parameter(name, getattr(cfg, name))
                setattr(cfg, name, self.get_parameter(name).value)
            self.guard = MotionGuard(cfg)
            self.add_on_set_parameters_callback(self._on_set_params)

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self._wz = 0.0
            self._last_state = None

            self.pub = self.create_publisher(Twist, 'auto_vel_raw', 10)
            self.pub_state = self.create_publisher(
                String, 'motion_guard/state', latched)
            self.create_subscription(LaserScan, 'scan_safe', self._on_scan,
                                     qos_profile_sensor_data)
            self.create_subscription(Odometry, 'odom', self._on_odom,
                                     qos_profile_sensor_data)
            self.create_subscription(Twist, 'auto_vel_pre', self._on_cmd, 10)

            d = 'controle_web/logs'
            _os.makedirs(d, exist_ok=True)
            self._csv_f = open(_os.path.join(d, 'motion_guard.csv'),
                               'w', newline='')
            self._csv = _csv.writer(self._csv_f)
            self._csv.writerow(['t', 'state', 'n_moving', 'nearest',
                                'in_corridor', 'vx_in', 'vx_out'])
            self.get_logger().info(
                'motion_guard ativo: raio %.1fm, corredor %.2fx%.1fm, '
                'slow %.0f%%, clear %.1fs' % (
                    cfg.guard_radius, cfg.corridor_half_w * 2,
                    cfg.corridor_len, cfg.slow_scale * 100, cfg.clear_time))

        def _on_set_params(self, params):
            for p in params:
                if p.name in self._CFG_PARAMS:
                    setattr(self.guard.cfg, p.name, p.value)
                    self.get_logger().info(
                        'param %s = %s (live)' % (p.name, p.value))
            return SetParametersResult(successful=True)

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def _on_odom(self, msg: Odometry):
            self._wz = msg.twist.twist.angular.z

        def _pose_odom(self):
            try:
                tf = self.tf_buffer.lookup_transform(
                    'odom', 'base_link', rclpy.time.Time())
            except TransformException:
                return None
            t = tf.transform.translation
            q = tf.transform.rotation
            return (t.x, t.y, quat_to_yaw(q.x, q.y, q.z, q.w))

        def _on_scan(self, msg: LaserScan):
            # pontos do scan -> frame odom (TF mais recente; a 10Hz e objeto
            # lento a defasagem é < grid_res). TF faltando -> NÃO alimenta o
            # guard -> scan_stale -> pass-through (failsafe da spec).
            try:
                tf = self.tf_buffer.lookup_transform(
                    'odom', msg.header.frame_id, rclpy.time.Time())
            except TransformException:
                self.get_logger().warn('sem TF odom<-%s; pass-through'
                                       % msg.header.frame_id,
                                       throttle_duration_sec=5.0)
                return
            pose = self._pose_odom()
            if pose is None:
                return
            r = np.asarray(msg.ranges, dtype=np.float32)
            # corta em guard_radius + 1m: barato e o guard re-filtra pelo robô
            ok = np.isfinite(r) & (r > 0.0) & \
                (r <= self.guard.cfg.guard_radius + 1.0)
            if not np.any(ok):
                self.guard.observe(self._now(), [], pose, self._wz)
                return
            a = msg.angle_min + np.arange(r.size) * msg.angle_increment
            xl, yl = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
            tt, q = tf.transform.translation, tf.transform.rotation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            c, s = math.cos(yaw), math.sin(yaw)
            pts = list(zip((tt.x + xl * c - yl * s).tolist(),
                           (tt.y + xl * s + yl * c).tolist()))
            self.guard.observe(self._now(), pts, pose, self._wz)

        def _on_cmd(self, msg: Twist):
            t = self._now()
            vx, wz, state = self.guard.filter(t, msg.linear.x, msg.angular.z)
            out = Twist()
            out.linear.x = vx
            out.angular.z = wz
            self.pub.publish(out)
            if state != self._last_state:
                self._last_state = state
                self.pub_state.publish(String(data=state))
                if state == 'passthrough':
                    self.get_logger().warn(
                        'pass-through (scan/TF indisponível ou disabled)',
                        throttle_duration_sec=5.0)
            self._csv.writerow([
                round(t, 3), state, len(self.guard.moving_clusters),
                round(self.guard.nearest_moving, 2)
                if math.isfinite(self.guard.nearest_moving) else '',
                int(self.guard.in_corridor),
                round(msg.linear.x, 3), round(vx, 3)])
            self._csv_f.flush()

    rclpy.init(args=args)
    node = MotionGuardNode()
    try:
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Entry point no setup.py**

Em `ros2_packages/robot_nav/setup.py`, na lista `console_scripts`, depois de `sim_actuator_model`:

```python
            'motion_guard = robot_nav.motion_guard:main',
```

- [ ] **Step 3: Wiring no launch**

Em `ros2_packages/robot_nav/launch/nav2.launch.py`:

(a) No nó `twist_mux_auto`, trocar o remap:

```python
            remappings=[('cmd_vel_out', 'auto_vel_pre')],
```

(b) Logo APÓS o bloco do `twist_mux_auto` (antes do collision_monitor), inserir:

```python
        # Cautela com objeto EM MOVIMENTO (2026-07-02): diff temporal do
        # /scan_safe no frame odom -> móvel perto = desacelera; móvel no
        # corredor à frente = para e retoma sozinho. Filtra SÓ a autonomia
        # (auto_vel_pre -> auto_vel_raw); unstuck/manual ficam fora. Failsafe:
        # sem TF/scan -> pass-through (nunca mata a nav). wz passa intocado.
        # Spec: docs/superpowers/specs/2026-07-02-motion-guard-design.md
        Node(
            package='robot_nav', executable='motion_guard',
            name='motion_guard', output=nav_output,
            parameters=[sim_time_param],
        ),
```

(c) Atualizar o comentário do velocity_smoother (linha ~136) e do twist_mux_auto (linha ~182) que citam `auto_vel_raw` como saída direta do mux: agora `mux -> auto_vel_pre -> motion_guard -> auto_vel_raw -> collision`.

- [ ] **Step 4: Suíte + build + smoke**

Run: `cd ros2_packages/robot_nav && python3 -m pytest test/ -q`
Expected: tudo PASS.

Run: `source /opt/ros/jazzy/setup.bash && colcon build --base-paths ros2_packages --packages-select robot_nav --symlink-install`
Expected: `Finished <<< robot_nav`.

Run: `source install/setup.bash && timeout 5 ros2 run robot_nav motion_guard; echo "exit=$?"`
Expected: log `motion_guard ativo: ...` e `exit=124` (vivo 5s sem crash — lição 06-28: teste unitário não pega bug de `self.X`).

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/motion_guard.py ros2_packages/robot_nav/setup.py ros2_packages/robot_nav/launch/nav2.launch.py
git commit -m "motion_guard: no ROS + wiring auto_vel_pre->auto_vel_raw (filtra so a autonomia, antes do collision)"
```

### Task 4: Validação no sim + docs + push

**Files:**
- Modify: `ESTADO_PROJETO.md` (seção 07-02)
- Modify: `docs/superpowers/plans/2026-07-02-motion-guard.md` (checkboxes)

**Interfaces:**
- Consumes: stack do sim (`./launch.sh --sim --nav2 --world=worlds/sala_grande.sdf --map=maps/sala_grande.yaml`), executável `motion_guard` (Task 3).

- [ ] **Step 1: Relançar o sim (COORDENAR com o dono — não derrubar run em andamento)**

O dono relança: `./launch.sh --sim --nav2 --world=worlds/sala_grande.sdf --map=maps/sala_grande.yaml --spawn-x=3.0 --spawn-y=0.0`.
Verificar: `ros2 topic echo /motion_guard/state --once` → `idle` (ou `passthrough` só até o 1º scan+TF).

- [ ] **Step 2: Spawnar a caixa móvel (ator animado do gz NÃO aparece no gpu_lidar)**

```bash
gz service -s /world/sala_grande/create --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean --timeout 2000 --req 'sdf: "<sdf version=\"1.9\"><model name=\"moving_box\"><pose>2.0 -2.5 0.25 0 0 0</pose><link name=\"body\"><collision name=\"c\"><geometry><box><size>0.4 0.4 0.5</size></box></geometry></collision><visual name=\"v\"><geometry><box><size>0.4 0.4 0.5</size></box></geometry></visual><inertial><mass>5</mass><inertia><ixx>0.2</ixx><iyy>0.2</iyy><izz>0.2</izz></inertia></inertial></link><plugin filename=\"gz-sim-velocity-control-system\" name=\"gz::sim::systems::VelocityControl\"><topic>/model/moving_box/cmd_vel</topic></plugin></model>"'
```

Expected: `data: true`. Ajustar `<pose>` p/ ~2m à frente da rota do robô no momento do teste.

- [ ] **Step 3: Mandar um goal e cruzar a caixa na frente do robô**

Goal simples (robô andando reto):

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -2.0, y: 0.0}, orientation: {w: 1.0}}}}" --feedback > /dev/null &
```

Com o robô em movimento, atravessar a caixa na frente dele (ida e volta):

```bash
gz topic -t /model/moving_box/cmd_vel -m gz.msgs.Twist -p 'linear: {y: 0.5}'
sleep 6
gz topic -t /model/moving_box/cmd_vel -m gz.msgs.Twist -p 'linear: {y: 0.0}'
```

- [ ] **Step 4: EU leio o CSV e o estado (dono não interpreta nada)**

Run: `python3 - <<'EOF'` (resumo por estado do `controle_web/logs/motion_guard.csv`: contagem de ticks por estado, vx_in vs vx_out nos trechos slowing/blocked, timestamps das transições)`EOF`
Expected: sequência `idle → slowing` (caixa se movendo no raio) `→ blocked` (caixa no corredor, vx_out=0 com vx_in>0) `→ idle` (~1.5s após a caixa sair); robô RETOMA e chega no goal (SUCCEEDED). Parede/mobília estática NUNCA gera slowing com o robô parado ou andando reto.
Regressão (falso positivo constante, nunca retoma, nav morta) → investigar antes de seguir; se insanável, `ros2 param set /motion_guard enabled false` desativa ao vivo.

- [ ] **Step 5: ESTADO_PROJETO.md + checkboxes + commit + push**

Atualizar `ESTADO_PROJETO.md` (seção 07-02): motion_guard implementado + resultado da validação sim + "⏳ real: decisão do dono". Marcar checkboxes deste plano.

```bash
git add ESTADO_PROJETO.md docs/superpowers/plans/2026-07-02-motion-guard.md
git commit -m "docs(estado): 07-02 motion_guard implementado + validado no sim"
git push
```

Expected: main atualizada; deploy na Pi fica a cargo do dono (fora do escopo).
