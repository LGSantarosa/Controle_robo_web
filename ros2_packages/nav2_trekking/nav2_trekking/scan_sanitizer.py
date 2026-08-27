#!/usr/bin/env python3
"""Filtro de retornos impossíveis do LiDAR — /scan -> /scan_safe.

Por que existe (captura ao vivo 2026-06-12, bag /tmp/porta_bag): ~2% dos scans
do LD06 trazem 2+ retornos a <15 cm do sensor (mediana 5,7 cm — DENTRO do
chassi do robô, obstáculo real ali é fisicamente impossível). Os fantasmas
concentram em setores fixos (~200-260° e ~80-120° no frame do laser = algo
físico raspando o feixe) e NÃO correlacionam com movimento (não é EMI). Como o
LD06 publica range_min=0.02 e a PolygonStop do collision monitor tem
min_points=2, dois pontinhos fantasmas congelavam o robô — inclusive parado no
meio da PORTA (17 dos 22 freezes da captura).

O nó troca esses retornos por +inf (sem retorno) e republica. O collision
monitor E os costmaps do nav2 (local/global) consomem /scan_safe
(nav2_params_pi.yaml) — 2026-06-23: o costmap usava /scan cru com
obstacle_min_range 0.0, então os fantasmas <0.15 m viravam obstáculo letal e
fincavam o robô (ver comentário no yaml). SLAM e cone_detector seguem no /scan
cru — eles já têm range_min próprio e usam o scan pra geometria.

A lógica pura (sanitize_ranges) é testável sem ROS; o nó embaixo é só cola de
I/O — mesmo padrão do unstuck_supervisor.
"""
import math

import numpy as np


def sanitize_ranges(ranges, min_valid: float):
    """Retorna (ranges_filtrados float32, n_descartados).

    Descarta (vira +inf) todo retorno 0 < r < min_valid. r == 0.0 (inválido
    do driver), inf e NaN passam intocados — consumidores já os tratam.
    """
    r = np.asarray(ranges, dtype=np.float32)
    bad = (r > 0.0) & (r < min_valid)
    n = int(np.count_nonzero(bad))
    if n == 0:
        return r, 0
    out = r.copy()
    out[bad] = math.inf
    return out, n


def mask_door_jambs(ranges, angle_min: float, angle_increment: float,
                    pose, jambs, jamb_r: float):
    """(ranges com batentes mascarados, n_mascarados).

    Converte cada retorno pro frame do MAPA (pose = (x,y,yaw) do TF) e troca
    por +inf os que caem num disco de jamb_r ao redor de um batente marcado.
    Chamado SÓ quando o door_crossing está em estado 'crossing' (gate) — o
    collision monitor fica "do tamanho da porta": cego pros 2 batentes
    clicados, enxergando todo o resto (pessoa no vão continua freando).
    """
    r = np.asarray(ranges, dtype=np.float32)
    if r.size == 0 or not jambs or angle_increment == 0.0:
        return r, 0
    ok = np.isfinite(r) & (r > 0.0)
    rr = np.where(ok, r, 0.0)
    a = angle_min + np.arange(r.size) * angle_increment
    x = rr * np.cos(a)
    y = rr * np.sin(a)
    px, py, pyaw = pose
    c, s = math.cos(pyaw), math.sin(pyaw)
    mx = px + x * c - y * s
    my = py + x * s + y * c
    bad = np.zeros(r.size, dtype=bool)
    for jx, jy in jambs:
        bad |= ((mx - jx) ** 2 + (my - jy) ** 2) <= jamb_r ** 2
    bad &= ok
    n = int(np.count_nonzero(bad))
    if n == 0:
        return r, 0
    out = r.copy()
    out[bad] = math.inf
    return out, n


def main(args=None):  # pragma: no cover - cola de I/O, validar na bancada
    import json

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException

    from .utils import quat_to_yaw, spin_node

    class ScanSanitizer(Node):

        def __init__(self):
            super().__init__('scan_sanitizer')
            # Raio do corpo do robô: retorno mais perto que isso é fantasma.
            # (footprint ±0.25; 0.15 deixa folga pra não comer obstáculo real
            # encostado no para-choque.)
            self.declare_parameter('min_valid_range', 0.15)
            self.min_valid = float(self.get_parameter('min_valid_range').value)

            self._dropped_total = 0
            self.pub = self.create_publisher(LaserScan, 'scan_safe',
                                             qos_profile_sensor_data)
            self.create_subscription(LaserScan, 'scan', self._on_scan,
                                     qos_profile_sensor_data)

            # Máscara de batente: durante a travessia verificada (door_crossing
            # em 'crossing'), os 2 batentes da porta marcada viram inf no
            # /scan_safe — o collision monitor fica "do tamanho da porta".
            self.declare_parameter('jamb_radius', 0.30)
            self.jamb_r = float(self.get_parameter('jamb_radius').value)
            self._doors = {}          # id -> {'a': [x,y], 'b': [x,y]}
            self._crossing_id = None  # id da porta em travessia, ou None
            self._masked_total = 0
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            latched = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, 'doors', self._on_doors, latched)
            self.create_subscription(String, 'door_zone', self._on_zone, latched)

            self.get_logger().info(
                f'scan_sanitizer ativo: retornos <{self.min_valid:.2f} m '
                f'viram inf (/scan -> /scan_safe)')

        def _on_doors(self, msg):
            try:
                self._doors = {d['id']: d
                               for d in json.loads(msg.data).get('doors', [])}
            except (ValueError, KeyError, TypeError) as e:
                self.get_logger().warn(f'/doors inválido: {e}')

        def _on_zone(self, msg):
            try:
                z = json.loads(msg.data)
                self._crossing_id = (z.get('door_id')
                                     if z.get('state') == 'crossing' else None)
            except ValueError:
                self._crossing_id = None

        def _pose_map(self):
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time())
            except TransformException:
                return None
            t = tf.transform.translation
            q = tf.transform.rotation
            return (t.x, t.y, quat_to_yaw(q.x, q.y, q.z, q.w))

        def _on_scan(self, msg: LaserScan):
            out, n = sanitize_ranges(msg.ranges, self.min_valid)
            if n:
                self._dropped_total += n
                # Throttle: fantasma é esporádico (~2% dos scans); logar todos
                # poluiria. O total acumulado correlaciona com freezes.
                self.get_logger().info(
                    f'{n} retorno(s) fantasma <{self.min_valid:.2f} m '
                    f'descartado(s) (total {self._dropped_total})',
                    throttle_duration_sec=10.0)
                msg.ranges = out.tolist()
            door = self._doors.get(self._crossing_id)
            if door is not None:
                pose = self._pose_map()
                if pose is not None:     # sem TF -> fail-safe: sem máscara
                    base = out if n else np.asarray(msg.ranges,
                                                    dtype=np.float32)
                    masked, nm = mask_door_jambs(
                        base, msg.angle_min, msg.angle_increment, pose,
                        [tuple(door['a']), tuple(door['b'])], self.jamb_r)
                    if nm:
                        self._masked_total += nm
                        self.get_logger().info(
                            f'porta {self._crossing_id}: {nm} ponto(s) de '
                            f'batente mascarado(s) (total {self._masked_total})',
                            throttle_duration_sec=5.0)
                        msg.ranges = masked.tolist()
            # Sem fantasma nem máscara: repassa a msg como veio (zero cópia).
            self.pub.publish(msg)

    rclpy.init(args=args)
    node = ScanSanitizer()
    try:
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':  # pragma: no cover
    main()
