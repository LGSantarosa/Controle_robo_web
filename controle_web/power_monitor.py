"""
Monitor de tensão das placas hoverboard.

Grava CSV contínuo (tensão por placa + setpoint vs velocidade medida das
rodas) e empurra telemetria ao vivo pro browser ('power_update'). Detecta
eventos elétricos na BORDA da condição: SAG gradual de tensão, STALL
(comando forte com roda parada) e corte seco — assinaturas que separam
sobrecarga de mau contato quando uma queda de energia acontecer.

Nasceu na investigação das quedas de 2026-06 (hipótese: desarme do BMS —
nunca confirmada em campo); fica como telemetria geral de energia.
Spec: docs/superpowers/specs/2026-06-11-power-monitor-design.md
Padrão de integração igual ao nav_metrics/trekking_service: nó rclpy próprio
em thread daemon, dentro do processo do Flask. Só OBSERVA — não publica nada
em tópico ROS.
"""
from __future__ import annotations

import csv
import logging
import os
import threading
import time
from collections import deque
from typing import List, Optional, Sequence, Tuple

log = logging.getLogger(__name__)

CSV_FIELDS = [
    'ts', 'v_front', 'v_rear',
    'set_left', 'set_right',
    'meas_fl', 'meas_fr', 'meas_rl', 'meas_rr',
    'stall', 'event',
]


class PowerEventDetector:
    """Detector puro (sem ROS) dos eventos elétricos. Eventos disparam na
    BORDA da condição (não repetem enquanto ela persiste):

      sag_front / sag_rear   queda > sag_drop_v dentro de sag_window_s
      trip_front / trip_rear placa viva caiu abaixo de trip_low_v (0 = stale,
                             o mega_bridge zera quando a placa não responde)
      stall                  |setpoint| >= stall_set_min com roda do mesmo
                             lado |medido| <= stall_meas_max por > stall_hold_s
    """

    SIDE_WHEELS = {'left': (0, 2), 'right': (1, 3)}  # índices FL FR RL RR

    def __init__(self,
                 sag_drop_v: float = 3.0,
                 sag_window_s: float = 1.0,
                 trip_low_v: float = 30.0,
                 stall_set_min: float = 50.0,
                 stall_meas_max: float = 5.0,
                 stall_hold_s: float = 0.5):
        self.sag_drop_v = sag_drop_v
        self.sag_window_s = sag_window_s
        self.trip_low_v = trip_low_v
        self.stall_set_min = stall_set_min
        self.stall_meas_max = stall_meas_max
        self.stall_hold_s = stall_hold_s

        self._boards = {
            name: {'alive': False, 'hist': deque(), 'sag_cond': False}
            for name in ('front', 'rear')
        }
        self._stall_since: Optional[float] = None
        self.stall_active = False

    @property
    def front_ok(self) -> bool:
        return self._boards['front']['alive']

    @property
    def rear_ok(self) -> bool:
        return self._boards['rear']['alive']

    def update(self, t: float, v_front: float, v_rear: float,
               setpoints: Sequence[float],
               measured: Sequence[float]) -> List[str]:
        events: List[str] = []
        for name, v in (('front', v_front), ('rear', v_rear)):
            ev = self._update_board(name, t, v)
            if ev:
                events.append(ev)
        ev = self._update_stall(t, setpoints, measured)
        if ev:
            events.append(ev)
        return events

    def _update_board(self, name: str, t: float, v: float) -> Optional[str]:
        b = self._boards[name]

        if not b['alive']:
            # Religa só ACIMA do nível de trip — senão uma tensão descendo
            # devagar oscilaria morta/viva e cuspiria trip a cada tick.
            if v >= self.trip_low_v:            # ligou (ou religou pós-trip)
                b['alive'] = True
                b['hist'] = deque([(t, v)])
                b['sag_cond'] = False
            return None                          # morta desde antes: sem evento

        if v < self.trip_low_v:                  # viva → morta = TRIP
            b['alive'] = False
            b['hist'].clear()
            b['sag_cond'] = False
            return f'trip_{name}'

        # SAG: queda rápida dentro da janela, ainda acima do nível de trip.
        hist = b['hist']
        hist.append((t, v))
        while hist and hist[0][0] < t - self.sag_window_s:
            hist.popleft()
        cond = (max(h[1] for h in hist) - v) > self.sag_drop_v
        fired = cond and not b['sag_cond']
        b['sag_cond'] = cond
        return f'sag_{name}' if fired else None

    def _update_stall(self, t: float, setpoints: Sequence[float],
                      measured: Sequence[float]) -> Optional[str]:
        cond = False
        for side, set_v in zip(('left', 'right'), setpoints):
            if abs(set_v) < self.stall_set_min:
                continue
            if any(abs(measured[i]) <= self.stall_meas_max
                   for i in self.SIDE_WHEELS[side]):
                cond = True
                break

        if not cond:
            self._stall_since = None
            self.stall_active = False
            return None
        if self._stall_since is None:
            self._stall_since = t
            return None
        if not self.stall_active and (t - self._stall_since) > self.stall_hold_s:
            self.stall_active = True
            return 'stall'
        return None


class PowerCsvLogger:
    """Um CSV por sessão em log_dir/power_YYYY-MM-DD_HHMMSS.csv, escrita
    bufferizada com flush periódico (pra linha do desarme estar no disco
    mesmo se o processo morrer logo depois)."""

    def __init__(self, log_dir: str, flush_interval_s: float = 1.0):
        os.makedirs(log_dir, exist_ok=True)
        self._path = os.path.join(
            log_dir,
            time.strftime('power_%Y-%m-%d_%H%M%S.csv'),
        )
        self._flush_interval_s = flush_interval_s
        self._last_flush = 0.0
        self._lock = threading.Lock()
        self._file = open(self._path, 'w', newline='')
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()

    @property
    def path(self) -> str:
        return self._path

    def log(self, ts: float, v_front: float, v_rear: float,
            setpoints: Sequence[float], measured: Sequence[float],
            stall: bool, events: Sequence[str]) -> None:
        row = {
            'ts': f'{ts:.3f}',
            'v_front': f'{v_front:.2f}',
            'v_rear': f'{v_rear:.2f}',
            'set_left': str(setpoints[0]),
            'set_right': str(setpoints[1]),
            'meas_fl': str(measured[0]),
            'meas_fr': str(measured[1]),
            'meas_rl': str(measured[2]),
            'meas_rr': str(measured[3]),
            'stall': '1' if stall else '0',
            'event': '|'.join(events),
        }
        with self._lock:
            self._writer.writerow(row)
            now = time.monotonic()
            if now - self._last_flush >= self._flush_interval_s:
                self._file.flush()
                # fsync: apagão seco de bateria não espera o commit do ext4
                # (~5s) — a curva de descarga tem que ir até o último segundo
                try:
                    os.fsync(self._file.fileno())
                except OSError:
                    pass
                self._last_flush = now

    def close(self) -> None:
        with self._lock:
            try:
                self._file.flush()
                self._file.close()
            except Exception:
                pass


class PowerMonitor:
    """Serviço: nó rclpy + amostragem 10 Hz → detector + CSV + UI (2 Hz).

    Tópicos (todos publicados pelo mega_bridge, QoS RELIABLE):
      /battery/front, /battery/rear     sensor_msgs/BatteryState (V; 0=stale)
      /wheel_vel_setpoints              wheel_msgs/WheelSpeeds (units ±1000)
      /hoverboard/wheel_velocities      Float64MultiArray [FL FR RL RR] (RPM)
    """

    STALE_AGE_S = 1.0  # sem msg de bateria há mais que isso = MEGA muda → "—"

    def __init__(self, socketio, log_dir: str,
                 sample_hz: float = 10.0, ui_hz: float = 2.0):
        # Imports de ROS aqui dentro pra lógica acima ser testável sem rclpy.
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import BatteryState
        from std_msgs.msg import Float64MultiArray
        from wheel_msgs.msg import WheelSpeeds

        self._sock = socketio
        self._detector = PowerEventDetector()
        self._csv = PowerCsvLogger(log_dir)

        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node('web_power_monitor')

        self._lock = threading.Lock()
        self._v = {'front': 0.0, 'rear': 0.0}
        self._v_ts = 0.0                       # última msg de bateria (wall)
        self._setpoints = (0.0, 0.0)           # (left, right)
        self._measured = (0.0, 0.0, 0.0, 0.0)  # FL FR RL RR

        self._node.create_subscription(
            BatteryState, '/battery/front',
            lambda m: self._on_battery('front', m), 10)
        self._node.create_subscription(
            BatteryState, '/battery/rear',
            lambda m: self._on_battery('rear', m), 10)
        self._node.create_subscription(
            WheelSpeeds, '/wheel_vel_setpoints', self._on_setpoint, 10)
        # sensor_data: casa com o pub do mega_bridge (P4 da AUDITORIA_2026-06-11;
        # QoS incompatível = silêncio total).
        self._node.create_subscription(
            Float64MultiArray, '/hoverboard/wheel_velocities', self._on_wheels,
            qos_profile_sensor_data)

        self._ui_period = 1.0 / ui_hz
        self._last_ui_emit = 0.0
        self._ui_event: str = ''               # último evento, pro flash na UI
        self._node.create_timer(1.0 / sample_hz, self._on_tick)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._running = True
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True, name='power_monitor_spin')
        self._spin_thread.start()

        log.info(f'[PowerMonitor] iniciado. CSV: {self._csv.path}')

    # ---- Callbacks ROS (só guardam o último valor; CPU mínima) ----

    def _on_battery(self, name: str, msg) -> None:
        with self._lock:
            self._v[name] = float(msg.voltage)
            self._v_ts = time.monotonic()

    def _on_setpoint(self, msg) -> None:
        with self._lock:
            self._setpoints = (float(msg.left_wheel), float(msg.right_wheel))

    def _on_wheels(self, msg) -> None:
        if len(msg.data) < 4:
            return
        with self._lock:
            self._measured = tuple(float(x) for x in msg.data[:4])

    # ---- Amostragem ----

    def _on_tick(self) -> None:
        # Detector/freshness/throttle em time.monotonic(): na Pi o relógio de
        # parede SALTA quando o NTP sincroniza pós-boot, e um salto atravessando
        # a janela de sag de 1 s cuspia sag_* falso no CSV (B5 da
        # AUDITORIA_2026-06-11). time.time() fica SÓ na coluna ts do CSV
        # (correlação com logs ROS).
        now = time.monotonic()
        with self._lock:
            v_front, v_rear = self._v['front'], self._v['rear']
            fresh = (now - self._v_ts) <= self.STALE_AGE_S
            setpoints, measured = self._setpoints, self._measured

        events: List[str] = []
        if fresh:
            events = self._detector.update(now, v_front, v_rear,
                                           setpoints, measured)
            self._csv.log(time.time(), v_front, v_rear, setpoints, measured,
                          self._detector.stall_active, events)
            for ev in events:
                log.warning(
                    f'[PowerMonitor] {ev.upper()}: vF={v_front:.1f}V '
                    f'vR={v_rear:.1f}V setL/R={setpoints[0]:.0f}/'
                    f'{setpoints[1]:.0f} meas={measured}')
                self._ui_event = ev

        if now - self._last_ui_emit >= self._ui_period:
            self._last_ui_emit = now
            payload = {
                'fresh': fresh,
                'v_front': round(v_front, 1) if fresh else None,
                'v_rear': round(v_rear, 1) if fresh else None,
                'front_ok': self._detector.front_ok,
                'rear_ok': self._detector.rear_ok,
                'stall': self._detector.stall_active,
                'event': self._ui_event,
            }
            self._ui_event = ''
            try:
                self._sock.emit('power_update', payload, namespace='/')
            except Exception as e:
                log.debug(f'[PowerMonitor] emit falhou: {e}')

    def _spin_loop(self) -> None:
        import rclpy
        while self._running and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                log.debug(f'[PowerMonitor] spin: {e}')

    def shutdown(self) -> None:
        # Mesma ordem do TrekkingBridge: para o loop → executor → nó.
        self._running = False
        try:
            if self._spin_thread.is_alive():
                self._spin_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self._executor.shutdown()
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
        self._csv.close()
