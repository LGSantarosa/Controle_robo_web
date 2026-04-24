"""
Coletor de métricas de navegação Nav2.

Grava uma linha em CSV por tentativa de navegação (do ACCEPTED ao
SUCCEEDED/ABORTED/CANCELED da action NavigateToPose). Cada linha resume:
 - tempo até o goal, status final
 - pose inicial e final (do /odom)
 - comprimento do plano inicial + quantos replans aconteceram
 - nº de comportamentos de recovery acionados (backup / spin / wait)
 - distância percorrida, velocidade média e máxima
 - tempo parado (cmd_vel ~0) e nº de inversões de direção (sinal de vx)

É a base pra tuning: sem esses números, qualquer ajuste de params é palpite.
"""
from __future__ import annotations

import csv
import logging
import math
import os
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_action_status_default

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path


log = logging.getLogger(__name__)

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN:   'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED:  'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED:  'CANCELED',
    GoalStatus.STATUS_ABORTED:   'ABORTED',
}

_TERMINAL_STATUSES = {
    GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_CANCELED,
    GoalStatus.STATUS_ABORTED,
}


@dataclass
class NavAttempt:
    nav_id:                   str
    start_ts:                 float
    start_x:                  float = 0.0
    start_y:                  float = 0.0
    end_ts:                   Optional[float] = None
    status:                   Optional[int]   = None
    end_x:                    float = 0.0
    end_y:                    float = 0.0
    end_yaw:                  float = 0.0
    initial_plan_length_m:    float = 0.0
    replans:                  int   = 0
    recoveries:               dict  = field(default_factory=lambda: defaultdict(int))
    distance_traveled_m:      float = 0.0
    max_linear_speed:         float = 0.0
    sum_linear_speed:         float = 0.0
    linear_samples:           int   = 0
    time_stopped_s:           float = 0.0
    direction_reversals:      int   = 0
    _last_cmd_sign:           int   = 0
    _last_cmd_ts:             Optional[float] = None


CSV_FIELDS = [
    'nav_id', 'start_ts', 'end_ts', 'duration_s', 'status',
    'start_x', 'start_y', 'end_x', 'end_y', 'end_yaw',
    'initial_plan_length_m', 'replans',
    'rec_backup', 'rec_spin', 'rec_wait',
    'distance_traveled_m', 'avg_linear_speed', 'max_linear_speed',
    'time_stopped_s', 'direction_reversals',
]


class NavMetricsCollector:
    """Roda em thread própria; subscreve tópicos Nav2 e acumula por tentativa."""

    def __init__(self, log_dir: str):
        self._log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        # CSV por dia — permite comparar antes/depois ao longo do tempo sem
        # juntar tudo num arquivo único gigante.
        self._csv_path = os.path.join(
            log_dir, f'nav_metrics_{time.strftime("%Y%m%d")}.csv'
        )
        self._csv_lock = threading.Lock()
        self._ensure_csv_header()

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node('nav_metrics_collector')
        if os.environ.get('ROBOT_SIM', 'false').lower() == 'true':
            from rclpy.parameter import Parameter
            self._node.set_parameters([
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
            ])

        # Estado
        self._attempt:        Optional[NavAttempt] = None
        self._nav_goal_id:    Optional[bytes]      = None
        self._last_odom_pose: Optional[tuple]      = None  # (x, y, yaw)
        self._last_odom_ts:   Optional[float]      = None
        self._recovery_ids:   dict                 = {}    # name → último goal_id contado

        # Subs — action statuses (Nav2 + recoveries)
        self._node.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._on_nav_status, qos_profile_action_status_default,
        )
        for name in ('backup', 'spin', 'wait'):
            self._node.create_subscription(
                GoalStatusArray, f'/{name}/_action/status',
                lambda msg, n=name: self._on_recovery_status(msg, n),
                qos_profile_action_status_default,
            )

        # Subs — dados do robô
        self._node.create_subscription(Path,     '/plan',    self._on_plan, 10)
        self._node.create_subscription(Odometry, '/odom',    self._on_odom, 10)
        self._node.create_subscription(Twist,    '/cmd_vel', self._on_cmd,  10)

        # Executor próprio em thread daemon
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._running  = True
        self._spin_th  = threading.Thread(
            target=self._spin_loop, daemon=True, name='nav_metrics_spin'
        )
        self._spin_th.start()

        log.info(f"[NavMetrics] coletor iniciado. CSV: {self._csv_path}")

    def _spin_loop(self):
        while self._running and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                log.debug(f"[NavMetrics] spin: {e}")

    def shutdown(self):
        self._running = False
        # Se estava no meio de uma navegação, marca como ABORTED e grava.
        if self._attempt is not None and self._attempt.end_ts is None:
            self._attempt.status = GoalStatus.STATUS_ABORTED
            self._attempt.end_ts = time.time()
            self._flush_attempt(self._attempt)
            self._attempt = None
        try:
            self._executor.shutdown()
            self._node.destroy_node()
        except Exception:
            pass

    # ---------------- Callbacks ----------------

    def _on_nav_status(self, msg: GoalStatusArray):
        """Detecta início e fim de uma navegação pela action NavigateToPose."""
        if not msg.status_list:
            return
        last      = msg.status_list[-1]
        status    = last.status
        goal_id   = bytes(last.goal_info.goal_id.uuid)

        # Novo goal em execução → abre tentativa
        if status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
            if self._attempt is None or self._nav_goal_id != goal_id:
                # Fecha tentativa anterior sem status terminal (raro; cleanup).
                if self._attempt is not None and self._attempt.end_ts is None:
                    self._attempt.status = GoalStatus.STATUS_ABORTED
                    self._attempt.end_ts = time.time()
                    self._flush_attempt(self._attempt)
                self._nav_goal_id = goal_id
                self._attempt = NavAttempt(
                    nav_id=goal_id.hex()[:8],
                    start_ts=time.time(),
                    start_x=self._last_odom_pose[0] if self._last_odom_pose else 0.0,
                    start_y=self._last_odom_pose[1] if self._last_odom_pose else 0.0,
                )
                log.info(f"[NavMetrics] início nav {self._attempt.nav_id}")
            return

        # Status terminal do goal corrente → fecha tentativa
        if status in _TERMINAL_STATUSES and self._attempt is not None \
                and self._nav_goal_id == goal_id:
            self._attempt.status = status
            self._attempt.end_ts = time.time()
            if self._last_odom_pose:
                self._attempt.end_x, self._attempt.end_y, self._attempt.end_yaw = self._last_odom_pose
            self._flush_attempt(self._attempt)
            self._attempt = None
            self._nav_goal_id = None

    def _on_recovery_status(self, msg: GoalStatusArray, name: str):
        """Conta +1 na primeira vez que vemos cada goal_id de recovery."""
        if not msg.status_list or self._attempt is None:
            return
        last    = msg.status_list[-1]
        goal_id = bytes(last.goal_info.goal_id.uuid)
        if last.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING) \
                and self._recovery_ids.get(name) != goal_id:
            self._recovery_ids[name] = goal_id
            self._attempt.recoveries[name] += 1

    def _on_plan(self, msg: Path):
        if self._attempt is None:
            return
        pts   = msg.poses
        plen  = 0.0
        for i in range(1, len(pts)):
            dx = pts[i].pose.position.x - pts[i - 1].pose.position.x
            dy = pts[i].pose.position.y - pts[i - 1].pose.position.y
            plen += math.hypot(dx, dy)
        if self._attempt.initial_plan_length_m == 0.0:
            self._attempt.initial_plan_length_m = plen
        else:
            self._attempt.replans += 1

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._last_odom_pose = (x, y, yaw)
        now = time.time()
        if self._attempt is not None and self._last_odom_ts is not None:
            vx    = msg.twist.twist.linear.x
            speed = abs(vx)
            dt    = now - self._last_odom_ts
            self._attempt.distance_traveled_m += speed * dt
            self._attempt.sum_linear_speed    += speed
            self._attempt.linear_samples      += 1
            if speed > self._attempt.max_linear_speed:
                self._attempt.max_linear_speed = speed
        self._last_odom_ts = now

    def _on_cmd(self, msg: Twist):
        a = self._attempt
        if a is None:
            return
        vx   = msg.linear.x
        now  = time.time()
        sign = 0 if abs(vx) < 0.01 else (1 if vx > 0 else -1)
        if a._last_cmd_ts is not None:
            dt = now - a._last_cmd_ts
            if sign == 0:
                a.time_stopped_s += dt
            if a._last_cmd_sign != 0 and sign != 0 and sign != a._last_cmd_sign:
                a.direction_reversals += 1
        if sign != 0:
            a._last_cmd_sign = sign
        a._last_cmd_ts = now

    # ---------------- CSV ----------------

    def _ensure_csv_header(self):
        with self._csv_lock:
            exists = os.path.isfile(self._csv_path)
            if not exists:
                with open(self._csv_path, 'w', newline='') as f:
                    csv.writer(f).writerow(CSV_FIELDS)

    def _flush_attempt(self, a: NavAttempt):
        end   = a.end_ts or a.start_ts
        dur   = end - a.start_ts
        avg   = (a.sum_linear_speed / a.linear_samples) if a.linear_samples else 0.0
        row   = [
            a.nav_id, f'{a.start_ts:.3f}', f'{end:.3f}', f'{dur:.2f}',
            STATUS_NAMES.get(a.status or 0, str(a.status)),
            f'{a.start_x:.3f}', f'{a.start_y:.3f}',
            f'{a.end_x:.3f}',   f'{a.end_y:.3f}', f'{a.end_yaw:.3f}',
            f'{a.initial_plan_length_m:.3f}', a.replans,
            a.recoveries.get('backup', 0),
            a.recoveries.get('spin',   0),
            a.recoveries.get('wait',   0),
            f'{a.distance_traveled_m:.3f}', f'{avg:.3f}', f'{a.max_linear_speed:.3f}',
            f'{a.time_stopped_s:.3f}', a.direction_reversals,
        ]
        with self._csv_lock:
            with open(self._csv_path, 'a', newline='') as f:
                csv.writer(f).writerow(row)
        log.info(
            f"[NavMetrics] nav {a.nav_id} → {STATUS_NAMES.get(a.status or 0)} "
            f"em {dur:.1f}s | replans={a.replans} | "
            f"rec(b/s/w)={a.recoveries.get('backup',0)}/"
            f"{a.recoveries.get('spin',0)}/{a.recoveries.get('wait',0)} | "
            f"dist={a.distance_traveled_m:.2f}m"
        )
