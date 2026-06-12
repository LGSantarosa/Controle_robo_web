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

O nó troca esses retornos por +inf (sem retorno) e republica. SÓ o collision
monitor consome /scan_safe (nav2_params_pi.yaml); SLAM, costmaps e
cone_detector seguem no /scan cru — eles já têm range_min próprio e usam o
scan pra geometria, não pra reflexo de freio.

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


def main(args=None):  # pragma: no cover - cola de I/O, validar na bancada
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    from .utils import spin_node

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
            self.get_logger().info(
                f'scan_sanitizer ativo: retornos <{self.min_valid:.2f} m '
                f'viram inf (/scan -> /scan_safe)')

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
            # Sem fantasma: repassa a msg como veio (zero cópia extra).
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
