#!/usr/bin/env python3
"""Probe A/B: manda a rota como goals do nav2 e mede comportamento.

Uso: probe.py <rota.json> <saida.json> [timeout_por_goal_s]
Mede POR GOAL: tempo, status, distancia, tempo parado, v_max/v_med,
menor distancia lida no /scan (global e no setor frontal), tempo com o
collision_monitor agindo (por tipo de acao) e tempo de unstuck.
"""
import json, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

ACT = {0: 'nada', 1: 'STOP', 2: 'SLOWDOWN', 3: 'APPROACH', 4: 'LIMIT'}


class Probe(Node):
    def __init__(self, rota, out, tmo):
        super().__init__('ab_probe')
        self.rota, self.out, self.tmo = rota, out, tmo
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, 'odom', self._odom, qos)
        self.create_subscription(LaserScan, 'scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(Twist, 'unstuck_vel', self._unstuck, qos)
        self.create_subscription(CollisionMonitorState, 'collision_monitor_state',
                                 self._cm, 10)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._reset()
        self.cm_action = 0
        self._cm_t = None
        self._unstuck_t = 0.0

    # -- acumuladores por goal --
    def _reset(self):
        self.dist = 0.0
        self.stopped = 0.0
        self.vmax = 0.0
        self.vsum = 0.0
        self.vn = 0
        self.min_scan = math.inf
        self.min_front = math.inf
        self.cm_time = {k: 0.0 for k in ACT}
        self.unstuck_time = 0.0
        self._last_xy = None
        self._last_t = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom(self, m):
        t = self.now()
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        v = abs(m.twist.twist.linear.x)
        w = abs(m.twist.twist.angular.z)
        if self._last_xy is not None and self._last_t is not None:
            dt = t - self._last_t
            if 0 < dt < 1.0:
                self.dist += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
                if v < 0.02 and w < 0.05:
                    self.stopped += dt
                # tempo em cada acao do collision monitor
                self.cm_time[self.cm_action] = self.cm_time.get(self.cm_action, 0.0) + dt
                if t - self._unstuck_t < 0.5:
                    self.unstuck_time += dt
        self._last_xy, self._last_t = (x, y), t
        self.vmax = max(self.vmax, v)
        self.vsum += v
        self.vn += 1

    def _scan(self, m):
        n = len(m.ranges)
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r < m.range_min or r > m.range_max:
                continue
            if r < self.min_scan:
                self.min_scan = r
            a = m.angle_min + i * m.angle_increment
            if abs(math.atan2(math.sin(a), math.cos(a))) < math.radians(30):
                if r < self.min_front:
                    self.min_front = r

    def _unstuck(self, m):
        if abs(m.linear.x) > 0.01 or abs(m.angular.z) > 0.01:
            self._unstuck_t = self.now()

    def _cm(self, m):
        self.cm_action = m.action_type

    def run(self):
        self.get_logger().info('esperando o action server navigate_to_pose...')
        if not self.ac.wait_for_server(timeout_sec=120.0):
            self.get_logger().error('nav2 nao subiu (sem action server)')
            return 2
        wps = json.load(open(self.rota))['waypoints']
        res = []
        t0 = self.now()
        for i, wp in enumerate(wps, 1):
            self._reset()
            g = NavigateToPose.Goal()
            g.pose.header.frame_id = 'map'
            g.pose.header.stamp = self.get_clock().now().to_msg()
            g.pose.pose.position.x = float(wp['x'])
            g.pose.pose.position.y = float(wp['y'])
            yaw = float(wp.get('yaw', 0.0))
            g.pose.pose.orientation.z = math.sin(yaw / 2)
            g.pose.pose.orientation.w = math.cos(yaw / 2)
            ini = self.now()
            fut = self.ac.send_goal_async(g)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
            gh = fut.result()
            if gh is None or not gh.accepted:
                res.append(dict(goal=i, status='REJEITADO', t=0))
                continue
            rf = gh.get_result_async()
            deadline = time.time() + self.tmo
            while rclpy.ok() and not rf.done() and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            if not rf.done():
                # 2026-08-27: NAO cancelamos mais. No trekking de competicao o
                # robo TEM que chegar em todos os goals (o tempo e a nota), entao
                # "desisti aos 150 s" nao e resultado: o numero certo e o tempo
                # ATE CHEGAR. Este teto e so a rede do harness pra uma volta nao
                # rodar pra sempre; quando bate, registramos PRESO e seguimos.
                gh.cancel_goal_async()
                for _ in range(30):
                    rclpy.spin_once(self, timeout_sec=0.1)
                st = 'PRESO'
            else:
                # 2026-08-31: o mapeamento estava INVERTIDO. action_msgs/GoalStatus:
                # 4=SUCCEEDED, 5=CANCELED, 6=ABORTED (conferido no jazzy). Como o
                # probe so' cancela no timeout — e ai' o status vira PRESO, sem
                # passar por aqui — na pratica todo 6 (Nav2 DESISTIU do goal) era
                # relatado como "CANCELADO", que se le como "o harness desistiu".
                # Trocou a leitura de uma volta inteira: latchN1 goal 2.
                st = {4: 'OK', 5: 'CANCELADO', 6: 'ABORTADO'}.get(
                    rf.result().status, f'status_{rf.result().status}')
            dur = self.now() - ini
            r = dict(goal=i, alvo=[round(wp['x'], 2), round(wp['y'], 2)], status=st,
                     t=round(dur, 1), dist=round(self.dist, 2),
                     parado=round(self.stopped, 1),
                     v_max=round(self.vmax, 3),
                     v_med=round(self.vsum / max(self.vn, 1), 3),
                     min_scan=round(self.min_scan, 3) if math.isfinite(self.min_scan) else None,
                     min_front=round(self.min_front, 3) if math.isfinite(self.min_front) else None,
                     cm={ACT[k]: round(v, 1) for k, v in self.cm_time.items() if v > 0.05},
                     unstuck=round(self.unstuck_time, 1))
            res.append(r)
            self.get_logger().info(f'goal {i}: {st} em {dur:.0f}s  dist={r["dist"]}m  '
                                   f'parado={r["parado"]}s  vmax={r["v_max"]}  '
                                   f'min_scan={r["min_scan"]}  cm={r["cm"]}')
        total = self.now() - t0
        json.dump(dict(total_s=round(total, 1), goals=res), open(self.out, 'w'), indent=2)
        self.get_logger().info(f'FIM: {total:.0f}s total -> {self.out}')
        return 0


def main():
    rclpy.init()
    n = Probe(sys.argv[1], sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 600.0)
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    try:
        rc = n.run()
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
    sys.exit(rc)


main()
