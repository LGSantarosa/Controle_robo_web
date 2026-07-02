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
  - móvel no raio guard_radius  -> linear.x escala pela distância (slowing);
    angular.z passa INTOCADO (continua navegando/girando perto de gente)
  - móvel no corredor à frente  -> PARADA TOTAL vx=0 E wz=0 até limpar
    clear_time (blocked). wz zerado a pedido do dono 07-02: com wz liberado o
    replan balançava o caminho e o robô GIRAVA no lugar enquanto a pessoa
    passava. NUNCA escalar wz parcialmente (zona-morta 1.7 = comando fraco
    que não gira); zerar é seguro.
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
from typing import List, Tuple

Pt = Tuple[float, float]


@dataclass
class GuardConfig:
    enabled: bool = True
    guard_radius: float = 2.5       # m — só olha móvel até aqui
    slow_scale: float = 0.25        # PISO do fator no vx (móvel colado)
    slow_dist: float = 0.6          # m — abaixo disso o fator satura no piso
                                    # (entre slow_dist e guard_radius a escala
                                    # sobe linear até 1.0: perto=lento, longe=
                                    # quase cheio — feedback do dono 07-02:
                                    # 50% uniforme era imperceptível de lado)
    corridor_half_w: float = 0.35   # m — meia-largura do corredor à frente
    corridor_len: float = 2.5       # m — alcance do corredor (1.5→2.5 dono
                                    # 07-02: cruzava o caminho ALÉM do corredor
                                    # e o follower saía atrás do desvio)
    freeze_dist: float = 1.2        # m — BOLHA: móvel mais perto que isso em
                                    # QUALQUER direção = parada total (dono
                                    # 07-02: do lado, o giro liberado rodava
                                    # atrás do plano-contorno; "para de pensar")
    clear_time: float = 3.0         # s — limpo por isso -> retoma (1.5→3.0
                                    # dono 07-02: gap p/ ~3 replans do nav2
                                    # endireitarem o plano antes de andar)
    grid_res: float = 0.15          # m — célula da grade de comparação
    lookback: float = 0.5           # s — compara com snapshot desta idade
    min_cluster_points: int = 3     # cluster menor = ruído
    cluster_gap: float = 0.3        # m — distância máx p/ mesmo cluster
    wz_gate: float = 0.3            # rad/s — girando acima disso não avalia
    scan_stale: float = 1.0         # s sem scan -> pass-through


class MotionGuard:
    """Detector de movimento por diff temporal em grade (frame odom).

    observe() processa um scan; filter() aplica a decisão no comando.
    """

    def __init__(self, cfg: GuardConfig):
        self.cfg = cfg
        self._snaps = deque()            # (t, frozenset de células)
        self.moving_clusters: List[List[Pt]] = []
        self.nearest_moving: float = math.inf
        self.in_corridor: bool = False
        self._last_moving_t: float = -math.inf
        self._last_nearest: float = math.inf   # dist do móvel na última vista
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
        # não avalia; a decisão anterior decai sozinha (clear_time no filter).
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

        clusters = [cl for cl in self._cluster(moving)
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
            self._last_nearest = self.nearest_moving
        if self.in_corridor:
            self._last_corridor_t = t

    def filter(self, t: float, vx: float, wz: float
               ) -> Tuple[float, float, str]:
        """aplica a decisão no comando. wz nunca é ESCALADO (zona-morta do
        giro); no blocked ele é ZERADO junto (parada total). Os latches
        expiram sozinhos pelo relógio (clear_time) — cobre também o
        decaimento durante o gate de giro (gated não re-avista o móvel)."""
        c = self.cfg
        if not c.enabled or t - self._last_scan_t > c.scan_stale:
            return vx, wz, 'passthrough'
        freeze = (t - self._last_moving_t < c.clear_time
                  and self._last_nearest < c.freeze_dist)
        if freeze or t - self._last_corridor_t < c.clear_time:
            # parada TOTAL: wz TAMBÉM zera (dono 07-02: com wz liberado o
            # replan do nav2 balançava o caminho e o robô girava no lugar
            # enquanto a pessoa ainda passava). Zerar é seguro — o perigo da
            # zona-morta é ESCALAR wz (comando fraco que não gira), não zerar.
            # Ré (vx<0, afasta do móvel à frente) continua passando.
            return (0.0 if vx > 0.0 else vx), 0.0, 'blocked'
        if t - self._last_moving_t < c.clear_time:
            # escala PROPORCIONAL à distância do móvel: colado (<=slow_dist)
            # freia no piso slow_scale; na borda do raio quase não freia.
            span = max(c.guard_radius - c.slow_dist, 1e-6)
            k = min(1.0, max(0.0, (self._last_nearest - c.slow_dist) / span))
            return vx * (c.slow_scale + (1.0 - c.slow_scale) * k), wz, 'slowing'
        return vx, wz, 'idle'

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
        # afináveis ao vivo (lição 04bcf86): mutam a MESMA ref de cfg que
        # observe/filter leem -> `ros2 param set` pega no tick seguinte
        _CFG_PARAMS = ('enabled', 'guard_radius', 'slow_scale', 'slow_dist',
                       'freeze_dist', 'corridor_half_w', 'corridor_len',
                       'clear_time',
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
                'slow %.0f%%@%.1fm..100%%@%.1fm, clear %.1fs' % (
                    cfg.guard_radius, cfg.corridor_half_w * 2,
                    cfg.corridor_len, cfg.slow_scale * 100, cfg.slow_dist,
                    cfg.guard_radius, cfg.clear_time))

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
