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
import json
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

# Checkpoint da tentativa EM ANDAMENTO (bateria pode morrer no meio de um
# goal — apagão seco, sem shutdown). Reescrito a cada ~2s com fsync; no boot
# seguinte vira linha POWERLOSS no CSV e é apagado. Goal que termina limpo
# apaga o checkpoint junto do flush normal.
CKPT_NAME = 'attempt_checkpoint.json'
CKPT_INTERVAL_S = 2.0


class NavMetricsCollector:
    """Roda em thread própria; subscreve tópicos Nav2 e acumula por tentativa."""

    def __init__(self, log_dir: str, on_nav_start=None, on_nav_end=None):
        self._log_dir = log_dir
        # Callbacks opcionais (ex.: câmera POV) — chamados no início e no fim
        # (status terminal) de cada tentativa. Erros deles não podem derrubar
        # o coletor, então sempre passam por _safe_cb.
        self._on_nav_start = on_nav_start
        self._on_nav_end = on_nav_end
        os.makedirs(log_dir, exist_ok=True)
        # CSV por dia — calculado a cada flush baseado em `end_ts` (clock ROS),
        # senão um servidor rodando 24h grava as tentativas do dia novo no
        # arquivo do dia anterior.
        self._csv_lock = threading.Lock()
        self._ckpt_path = os.path.join(log_dir, CKPT_NAME)
        self._ckpt_last = 0.0
        self._recover_checkpoint()

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

        log.info(f"[NavMetrics] coletor iniciado. CSV dir: {self._log_dir}")

    def _safe_cb(self, cb):
        if cb is None:
            return
        try:
            cb()
        except Exception as e:
            log.debug(f"[NavMetrics] callback falhou: {e}")

    def _now(self) -> float:
        """Tempo em segundos do clock ROS (respeita use_sim_time)."""
        try:
            return self._node.get_clock().now().nanoseconds * 1e-9
        except Exception:
            return time.time()

    def _csv_path_for(self, ts: float) -> str:
        return os.path.join(
            self._log_dir,
            f'nav_metrics_{time.strftime("%Y%m%d", time.localtime(ts))}.csv',
        )

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
            self._attempt.end_ts = self._now()
            self._flush_attempt(self._attempt)
            self._attempt = None
        try:
            self._executor.shutdown()
            self._node.destroy_node()
        except Exception:
            pass

    # ---------------- Callbacks ----------------

    def _on_nav_status(self, msg: GoalStatusArray):
        """Detecta início e fim de uma navegação pela action NavigateToPose.

        Itera a lista inteira em vez de pegar só `status_list[-1]` — quando
        waypoints sequenciais disparam goals em rápida sucessão, o último
        status pode ser do *próximo* goal e mascarar o SUCCEEDED/ABORTED do
        anterior. Processa terminal do goal corrente antes de aceitar um novo.
        """
        if not msg.status_list:
            return

        # 1) Procura status terminal do goal corrente
        if self._attempt is not None and self._nav_goal_id is not None:
            for s in msg.status_list:
                goal_id = bytes(s.goal_info.goal_id.uuid)
                if goal_id == self._nav_goal_id and s.status in _TERMINAL_STATUSES:
                    self._attempt.status = s.status
                    self._attempt.end_ts = self._now()
                    if self._last_odom_pose:
                        self._attempt.end_x, self._attempt.end_y, self._attempt.end_yaw = self._last_odom_pose
                    self._flush_attempt(self._attempt)
                    self._attempt = None
                    self._nav_goal_id = None
                    self._safe_cb(self._on_nav_end)
                    break

        # 2) Procura ACCEPTED/EXECUTING (novo goal) — sempre o último encontrado,
        # que é o goal "ativo" agora.
        new_id = None
        for s in msg.status_list:
            if s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                new_id = bytes(s.goal_info.goal_id.uuid)
        if new_id is not None and (self._attempt is None or self._nav_goal_id != new_id):
            # Fecha tentativa anterior sem status terminal (raro; cleanup).
            if self._attempt is not None and self._attempt.end_ts is None:
                self._attempt.status = GoalStatus.STATUS_ABORTED
                self._attempt.end_ts = self._now()
                self._flush_attempt(self._attempt)
            self._nav_goal_id = new_id
            # B16: limpa contagem de recoveries da tentativa anterior pra não
            # ignorar recoveries reais do goal novo só porque o UUID coincidiu.
            self._recovery_ids.clear()
            self._attempt = NavAttempt(
                nav_id=new_id.hex()[:8],
                start_ts=self._now(),
                start_x=self._last_odom_pose[0] if self._last_odom_pose else 0.0,
                start_y=self._last_odom_pose[1] if self._last_odom_pose else 0.0,
            )
            log.info(f"[NavMetrics] início nav {self._attempt.nav_id}")
            self._safe_cb(self._on_nav_start)

    def _on_recovery_status(self, msg: GoalStatusArray, name: str):
        """Conta +1 na primeira vez que vemos cada goal_id de recovery."""
        if not msg.status_list or self._attempt is None:
            return
        # Itera lista inteira: o último status pode ser de um recovery de outro
        # goal — só conta o primeiro ACCEPTED/EXECUTING que vier de cada UUID.
        for s in msg.status_list:
            if s.status not in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                continue
            goal_id = bytes(s.goal_info.goal_id.uuid)
            if self._recovery_ids.get(name) != goal_id:
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
        prev = self._last_odom_pose
        self._last_odom_pose = (x, y, yaw)
        now = self._now()
        if self._attempt is not None and self._last_odom_ts is not None and prev is not None:
            vx    = msg.twist.twist.linear.x
            speed = abs(vx)
            dt    = now - self._last_odom_ts
            # Distância via pose (hypot) — em slip, integrar velocidade
            # reportada infla o número.
            self._attempt.distance_traveled_m += math.hypot(x - prev[0], y - prev[1])
            self._attempt.sum_linear_speed    += speed
            self._attempt.linear_samples      += 1
            if speed > self._attempt.max_linear_speed:
                self._attempt.max_linear_speed = speed
        self._last_odom_ts = now
        self._maybe_checkpoint(now)

    def _on_cmd(self, msg: Twist):
        a = self._attempt
        if a is None:
            return
        vx   = msg.linear.x
        now  = self._now()
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

    def _row_for(self, a: NavAttempt, end: float, status_name: str) -> list:
        dur = end - a.start_ts
        avg = (a.sum_linear_speed / a.linear_samples) if a.linear_samples else 0.0
        return [
            a.nav_id, f'{a.start_ts:.3f}', f'{end:.3f}', f'{dur:.2f}',
            status_name,
            f'{a.start_x:.3f}', f'{a.start_y:.3f}',
            f'{a.end_x:.3f}',   f'{a.end_y:.3f}', f'{a.end_yaw:.3f}',
            f'{a.initial_plan_length_m:.3f}', a.replans,
            a.recoveries.get('backup', 0),
            a.recoveries.get('spin',   0),
            a.recoveries.get('wait',   0),
            f'{a.distance_traveled_m:.3f}', f'{avg:.3f}', f'{a.max_linear_speed:.3f}',
            f'{a.time_stopped_s:.3f}', a.direction_reversals,
        ]

    def _append_row(self, row: list, end_ts: float):
        csv_path = self._csv_path_for(end_ts)
        with self._csv_lock:
            new_file = not os.path.isfile(csv_path)
            with open(csv_path, 'a', newline='') as f:
                w = csv.writer(f)
                if new_file:
                    w.writerow(CSV_FIELDS)
                w.writerow(row)
                f.flush()
                os.fsync(f.fileno())

    def _maybe_checkpoint(self, now: float):
        """Persiste a tentativa em andamento (~2s, atômico+fsync) — se a
        bateria morrer no meio do goal, o boot seguinte recupera a linha."""
        a = self._attempt
        if a is None or now - self._ckpt_last < CKPT_INTERVAL_S:
            return
        self._ckpt_last = now
        end = self._now()
        if self._last_odom_pose:
            # pose "final" provisória = onde o robô está agora (o flush
            # terminal sobrescreve se o goal acabar limpo)
            a.end_x, a.end_y, a.end_yaw = self._last_odom_pose
        tmp = self._ckpt_path + '.tmp'
        try:
            with open(tmp, 'w') as f:
                json.dump({'row': self._row_for(a, end, 'POWERLOSS'),
                           'end_ts': end}, f)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self._ckpt_path)
        except OSError as e:
            log.debug(f"[NavMetrics] checkpoint falhou: {e}")

    def _clear_checkpoint(self):
        try:
            os.unlink(self._ckpt_path)
        except OSError:
            pass

    def _recover_checkpoint(self):
        """Checkpoint sobrou de uma sessão anterior = apagão no meio de um
        goal. Vira linha POWERLOSS no CSV do dia dele e é removido."""
        try:
            with open(self._ckpt_path) as f:
                data = json.load(f)
        except FileNotFoundError:
            return
        except Exception as e:
            log.warning(f"[NavMetrics] checkpoint ilegível ({e}) — descartado")
            self._clear_checkpoint()
            return
        row = data.get('row')
        if row:
            self._append_row(row, float(data.get('end_ts', time.time())))
            log.warning(f"[NavMetrics] tentativa {row[0]} interrompida por "
                        f"queda de energia — recuperada do checkpoint pro CSV")
        self._clear_checkpoint()

    def _flush_attempt(self, a: NavAttempt):
        end = a.end_ts or a.start_ts
        dur = end - a.start_ts
        row = self._row_for(a, end, STATUS_NAMES.get(a.status or 0, str(a.status)))
        self._append_row(row, end)
        self._clear_checkpoint()
        log.info(
            f"[NavMetrics] nav {a.nav_id} → {STATUS_NAMES.get(a.status or 0)} "
            f"em {dur:.1f}s | replans={a.replans} | "
            f"rec(b/s/w)={a.recoveries.get('backup',0)}/"
            f"{a.recoveries.get('spin',0)}/{a.recoveries.get('wait',0)} | "
            f"dist={a.distance_traveled_m:.2f}m"
        )
