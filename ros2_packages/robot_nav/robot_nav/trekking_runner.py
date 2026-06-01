#!/usr/bin/env python3
"""
Controlador ponto-a-ponto para a competição de trekking.

Máquina de estado:
  IDLE   — parado. Publica zero em /cmd_vel ocasionalmente e não interfere.
  RECORD — usuário dirige; cada rising edge do botão (ou /trekking/cmd
           save_point) grava o waypoint atual + cone mais próximo no scan.
  PLAY   — percorre a lista de waypoints com PID heading + velocidade
           proporcional, fazendo "snap-to-cone" quando entra no raio de
           busca do cone gravado.

Entradas:
  /trekking/pose       PoseStamped       posição/yaw fundidos
  /trekking/cones      PoseArray         cones detectados em odom (com width na orientation.x)
  /start_button        Bool              botão físico da MEGA (deadman + save)
  /trekking/cmd        String (JSON)     comandos vindos da UI

Saídas:
  /cmd_vel             Twist
  /leds/color          ColorRGBA         (alpha = modo: 0 fixo, 1 pisca, 2 rotação)
  /trekking/state      String (JSON)     estado completo p/ a UI (~10 Hz)
  /trekking/waypoints  PoseArray         lista de waypoints (visualização)
  /trekking/target     PoseStamped       alvo corrente do PID (post-snap)

Filosofia:
  - Sair voado: PID heading + v = v_max * cos²(err) * clamp(dist/d_brake, 0, 1).
  - Cone como landmark: ao chegar perto da posição esperada do cone, casa
    com cone detectado no scan e RE-ÂNCORA o alvo (alvo = cone_observado +
    (waypoint - cone_gravado)). Isso compensa drift acumulado por waypoint.
  - Sem TF — só /trekking/pose. Trekking não acorda se o pose_estimator
    não estiver publicando.
"""
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Bool, ColorRGBA, String


MODE_IDLE   = 'idle'
MODE_RECORD = 'record'
MODE_PLAY   = 'play'


from .utils import quat_to_yaw as _quat_to_yaw, wrap_pi as _wrap_pi
from .cone_pose_fix import ConeFixConfirmer, cone_fix_delta


def _yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class TrekkingRunner(Node):

    def __init__(self):
        super().__init__('trekking_runner')

        # --- PID heading ---
        self.declare_parameter('kp_heading', 1.6)
        self.declare_parameter('kd_heading', 0.20)
        self.declare_parameter('omega_max', 1.2)        # rad/s

        # --- Velocidade ---
        self.declare_parameter('v_max', 0.35)           # m/s — começamos devagar
        self.declare_parameter('v_min', 0.05)
        self.declare_parameter('d_brake', 0.6)          # m — freia ao último ponto a partir daqui
        # Power do cosseno: cos^n; n alto pune mais o erro de heading.
        self.declare_parameter('heading_cos_power', 2.0)

        # --- Avanço de waypoint ---
        self.declare_parameter('arrival_tolerance', 0.25)    # m
        # Se o produto escalar do vetor pro alvo trocar de sinal: passou batido.
        self.declare_parameter('passby_detection', True)

        # --- Snap-to-cone ---
        self.declare_parameter('cone_search_radius', 1.5)    # m — começa a procurar
        self.declare_parameter('cone_match_radius',  0.6)    # m — distância máx do esperado
        self.declare_parameter('cone_bearing_tol_deg', 60.0) # ° — janela angular relativa

        # --- Correção persistente de pose por cone-âncora (aditiva ao snap) ---
        self.declare_parameter('enable_cone_pose_fix', True)
        self.declare_parameter('cone_confirm_frames', 4)     # ciclos estáveis p/ confirmar
        self.declare_parameter('cone_stable_eps', 0.10)      # m — "mesma posição" entre ciclos
        self.declare_parameter('cone_unique_radius', 0.50)   # m — se >1 candidato aqui → ambíguo

        # --- LEDs ---
        self.declare_parameter('led_arrival_ms', 600)
        self.declare_parameter('publish_state_hz', 10.0)

        # --- Loop ---
        self.declare_parameter('control_hz', 30.0)

        self.kp_h    = float(self.get_parameter('kp_heading').value)
        self.kd_h    = float(self.get_parameter('kd_heading').value)
        self.w_max   = float(self.get_parameter('omega_max').value)
        self.v_max   = float(self.get_parameter('v_max').value)
        self.v_min   = float(self.get_parameter('v_min').value)
        self.d_brake = float(self.get_parameter('d_brake').value)
        self.cos_n   = float(self.get_parameter('heading_cos_power').value)
        self.arr_tol = float(self.get_parameter('arrival_tolerance').value)
        self.passby  = bool(self.get_parameter('passby_detection').value)
        self.r_search= float(self.get_parameter('cone_search_radius').value)
        self.r_match = float(self.get_parameter('cone_match_radius').value)
        self.bear_tol= math.radians(float(self.get_parameter('cone_bearing_tol_deg').value))
        self.led_ms  = int(self.get_parameter('led_arrival_ms').value)
        self.state_dt= 1.0 / float(self.get_parameter('publish_state_hz').value)
        self.ctrl_dt = 1.0 / float(self.get_parameter('control_hz').value)
        self.enable_cone_pose_fix = bool(self.get_parameter('enable_cone_pose_fix').value)
        self.cone_confirm_frames  = int(self.get_parameter('cone_confirm_frames').value)
        self.cone_stable_eps      = float(self.get_parameter('cone_stable_eps').value)
        self.cone_unique_radius   = float(self.get_parameter('cone_unique_radius').value)

        # --- Estado do robô ---
        # _state_lock protege x/y/yaw/have_pose/cones — escritos pelos callbacks
        # _on_pose/_on_cones e lidos por _control_tick/_state_tick. Hoje o
        # SingleThreadedExecutor serializa tudo, mas migrar para MultiThreaded
        # ou ReentrantCallbackGroup quebraria silenciosamente sem o lock.
        self._state_lock = threading.Lock()
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.have_pose = False

        # --- Cones detectados (lista de tuplas (x, y, w)) ---
        self.cones = []

        # --- Botão ---
        # `button_stable` é o estado debounçado (último estado confirmado);
        # `button_pending` guarda quantos frames consecutivos vimos o novo
        # valor. Só atualiza o estável após DEBOUNCE_FRAMES iguais — evita
        # falsos rising edges no bouncing mecânico a 50 Hz.
        # `button_stable=None` marca "ainda não calibrado": o primeiro
        # callback adota o valor recebido sem disparar rising edge — caso
        # o botão esteja pressionado quando o nó sobe.
        self.button_stable = None
        self.button_pending_value = False
        self.button_pending_count = 0
        self.DEBOUNCE_FRAMES = 2

        # --- Máquina de estado ---
        self.mode = MODE_IDLE
        # waypoints: lista de dicts {x, y, yaw, cone_x, cone_y, cone_bearing, has_cone}
        # cone_bearing é relativo ao yaw do robô na gravação (rad).
        self.waypoints = []
        self.current_idx = 0
        self.locked_cone = None    # (x, y) — cone "trancado" pra esse waypoint, ou None
        # Correção de pose: confirmador + trava 1x-por-cone + telemetria read-only.
        self._confirmer = ConeFixConfirmer(self.cone_confirm_frames, self.cone_stable_eps)
        self._cone_fix_done = False
        self._anchor = None            # (x,y) detecção usada como referência, ou None
        self._anchor_status = 'idle'   # idle | confirming | ambiguous | fixed
        self._anchor_clutter = []      # [(x,y), ...] candidatos descartados perto do esperado
        self._anchor_confirm = 0       # progresso do confirmador
        self.last_to_target = None # vetor (dx, dy) último → detecção de pass-by
        self.prev_heading_err = 0.0
        self.led_until = 0.0       # walltime até quando manter LED de chegada
        self._last_led = None      # última cor publicada (dedup do _led_tick 1 Hz)
        self.last_msg = ''

        # --- Subs ---
        self.create_subscription(PoseStamped, 'trekking/pose', self._on_pose, 20)
        self.create_subscription(PoseArray,   'trekking/cones', self._on_cones, 10)
        self.create_subscription(Bool,        'start_button', self._on_button, 10)
        self.create_subscription(String,      'trekking/cmd', self._on_cmd, 10)

        # --- Pubs ---
        self.pub_cmd    = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_leds   = self.create_publisher(ColorRGBA, 'leds/color', 10)
        self.pub_state  = self.create_publisher(String, 'trekking/state', 10)
        self.pub_wps    = self.create_publisher(PoseArray, 'trekking/waypoints', 10)
        self.pub_target = self.create_publisher(PoseStamped, 'trekking/target', 10)
        self.pub_pose_fix = self.create_publisher(Vector3Stamped, 'trekking/pose_fix', 10)

        self.create_timer(self.ctrl_dt, self._control_tick)
        self.create_timer(self.state_dt, self._state_tick)
        # Pulso de LED no modo (rotação/pisca) também precisa ser reenviado
        # periodicamente — a MEGA não decai sozinha.
        self.create_timer(1.0, self._led_tick)

        self.get_logger().info(
            f'trekking_runner: v_max={self.v_max:.2f} m/s, '
            f'kp_h={self.kp_h:.2f}, arrival={self.arr_tol*100:.0f} cm'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_pose(self, msg: PoseStamped):
        yaw = _quat_to_yaw(
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        )
        with self._state_lock:
            self.x = msg.pose.position.x
            self.y = msg.pose.position.y
            self.yaw = yaw
            self.have_pose = True

    def _on_cones(self, msg: PoseArray):
        cones = [
            (p.position.x, p.position.y, p.orientation.x)  # x.orientation = width
            for p in msg.poses
        ]
        with self._state_lock:
            self.cones = cones

    def _state_snapshot(self):
        with self._state_lock:
            return (self.x, self.y, self.yaw, self.have_pose, list(self.cones))

    def _on_button(self, msg: Bool):
        v = bool(msg.data)
        if self.button_stable is None:
            # Primeira amostra — calibra o estado estável sem rising edge.
            self.button_stable = v
            self.button_pending_value = v
            self.button_pending_count = 0
            return
        if v == self.button_stable:
            # Já estamos no estado v — limpa qualquer transição pendente.
            self.button_pending_value = v
            self.button_pending_count = 0
            return
        if v != self.button_pending_value:
            self.button_pending_value = v
            self.button_pending_count = 1
        else:
            self.button_pending_count += 1
        if self.button_pending_count < self.DEBOUNCE_FRAMES:
            return
        rising = v and not self.button_stable
        self.button_stable = v
        self.button_pending_count = 0
        # Botão no modo RECORD → grava waypoint.
        if rising and self.mode == MODE_RECORD:
            self._save_point()

    def _on_cmd(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'cmd JSON inválido: {e}')
            return

        cmd = (data.get('cmd') or '').lower()
        if cmd == 'reset':
            self._reset_origin()
        elif cmd == 'record':
            self.mode = MODE_RECORD
            self.last_msg = 'modo RECORD'
        elif cmd == 'save_point':
            if self.mode != MODE_RECORD:
                self.mode = MODE_RECORD
            self._save_point()
        elif cmd == 'play':
            self._start_play()
        elif cmd == 'stop':
            self.mode = MODE_IDLE
            self.current_idx = 0
            self._stop_robot()
            self.last_msg = 'parado'
        elif cmd == 'load_waypoints':
            wps = data.get('waypoints') or []
            sane = []
            errors = 0
            for w in wps:
                if not w:
                    continue
                try:
                    sane.append(self._sanitize_wp(w))
                except (TypeError, ValueError, AttributeError) as e:
                    errors += 1
                    self.get_logger().warn(f'waypoint inválido descartado: {e}')
            self.waypoints = sane
            self.current_idx = 0
            if errors:
                self.last_msg = f'{len(sane)} waypoints carregados ({errors} ignorados)'
            else:
                self.last_msg = f'{len(sane)} waypoints carregados'
        elif cmd == 'clear':
            self.waypoints = []
            self.current_idx = 0
            self.last_msg = 'lista limpa'
        else:
            self.get_logger().warn(f'cmd desconhecido: {cmd}')

    # ------------------------------------------------------------------
    # Comandos da UI
    # ------------------------------------------------------------------
    def _reset_origin(self):
        # Não dá pra "zerar" a saída do pose_estimator daqui sem mexer nele.
        # Em vez disso, registramos a posição atual como origem lógica — todos
        # os waypoints são gravados em coordenadas absolutas do /trekking/pose,
        # então só limpamos a lista. (O usuário "voltar pro 0" é só voltar
        # pra perto da posição em que pressionou Reset.)
        self.waypoints = []
        self.current_idx = 0
        self.mode = MODE_IDLE
        self.locked_cone = None
        self._reset_cone_fix()
        self._stop_robot()
        x, y, yaw, have_pose, _ = self._state_snapshot()
        if have_pose:
            self.last_msg = (
                f'origem: ({x:.2f}, {y:.2f}) yaw={math.degrees(yaw):.0f}°'
            )
        else:
            self.last_msg = 'origem registrada (sem pose ainda)'

    def _sanitize_wp(self, w: dict) -> dict:
        return {
            'x':     float(w.get('x', 0.0)),
            'y':     float(w.get('y', 0.0)),
            'yaw':   float(w.get('yaw', 0.0)),
            'cone_x':       float(w.get('cone_x', 0.0)),
            'cone_y':       float(w.get('cone_y', 0.0)),
            'cone_bearing': float(w.get('cone_bearing', 0.0)),
            'has_cone':     bool(w.get('has_cone', False)),
        }

    def _save_point(self):
        x, y, yaw, have_pose, cones = self._state_snapshot()
        if not have_pose:
            self.last_msg = 'sem pose — pose_estimator parado?'
            return

        # Procura cone mais próximo no semicírculo frontal do robô (±90°)
        # com leve preferência pelo mais próximo.
        cone = self._nearest_front_cone(x, y, yaw, cones)
        wp = {
            'x': x,
            'y': y,
            'yaw': yaw,
            'has_cone': cone is not None,
            'cone_x': cone[0] if cone else 0.0,
            'cone_y': cone[1] if cone else 0.0,
            # bearing relativo ao yaw atual (importante na verificação no play)
            'cone_bearing': cone[2] if cone else 0.0,
        }
        self.waypoints.append(wp)
        idx = len(self.waypoints) - 1
        if cone:
            self._flash_led(0.0, 0.5, 0.0, mode=1)   # verde pisca → ok
            self.last_msg = f'wp{idx}: ({x:.2f}, {y:.2f}) + cone'
        else:
            self._flash_led(1.0, 0.7, 0.0, mode=1)   # amarelo pisca → sem cone
            self.last_msg = f'wp{idx}: ({x:.2f}, {y:.2f}) — cone não visto'

    def _nearest_front_cone(self, x: float, y: float, yaw: float, cones):
        # Retorna (cone_x, cone_y, bearing_relativo) do cone mais próximo
        # cujo bearing relativo ao yaw atual esteja em [-90°, +90°].
        best = None
        best_d = float('inf')
        for cx, cy, _w in cones:
            dx = cx - x; dy = cy - y
            d = math.hypot(dx, dy)
            if d < 0.05 or d > self.r_search * 2:
                continue
            bearing = _wrap_pi(math.atan2(dy, dx) - yaw)
            if abs(bearing) > math.pi / 2.0:
                continue
            if d < best_d:
                best_d = d
                best = (cx, cy, bearing)
        return best

    def _start_play(self):
        if not self.waypoints:
            self.last_msg = 'sem waypoints — nada pra fazer'
            return
        _, _, _, have_pose, _ = self._state_snapshot()
        if not have_pose:
            self.last_msg = 'sem pose — pose_estimator parado?'
            return
        self.mode = MODE_PLAY
        self.current_idx = 0
        self.locked_cone = None
        self._reset_cone_fix()
        self.last_to_target = None
        self.prev_heading_err = 0.0
        self.last_msg = f'PLAY {len(self.waypoints)} waypoints'

    def _reset_cone_fix(self):
        self._cone_fix_done = False
        self._confirmer.reset()
        self._anchor = None
        self._anchor_status = 'idle'
        self._anchor_clutter = []
        self._anchor_confirm = 0

    # ------------------------------------------------------------------
    # Loop de controle (30 Hz)
    # ------------------------------------------------------------------
    def _control_tick(self):
        if self.mode != MODE_PLAY:
            return
        x, y, yaw, have_pose, cones = self._state_snapshot()
        if not have_pose:
            return
        if self.current_idx >= len(self.waypoints):
            self.mode = MODE_IDLE
            self._stop_robot()
            self._flash_led(0.0, 1.0, 0.0, mode=1)
            self.last_msg = 'rota concluída'
            return

        wp = self.waypoints[self.current_idx]

        # 1) Re-âncora pelo cone se já estivermos perto da posição esperada
        target_x, target_y = wp['x'], wp['y']
        if wp['has_cone']:
            dist_to_cone_expected = math.hypot(
                wp['cone_x'] - x, wp['cone_y'] - y
            )
            if self.locked_cone is None and dist_to_cone_expected < self.r_search:
                snap = self._find_matching_cone(wp, x, y, yaw, cones)
                if snap is not None:
                    self.locked_cone = snap
            if self.locked_cone is not None:
                # alvo corrigido: cone_observado + offset gravado
                ox = wp['x'] - wp['cone_x']
                oy = wp['y'] - wp['cone_y']
                target_x = self.locked_cone[0] + ox
                target_y = self.locked_cone[1] + oy

        # 1b) Correção PERSISTENTE de pose por cone-âncora (aditiva: não mexe no
        # alvo acima). Gates conservadores no _confirmer; na dúvida não corrige.
        if self.enable_cone_pose_fix and wp['has_cone'] and not self._cone_fix_done:
            self._maybe_publish_pose_fix(wp, x, y, yaw, cones)

        dx = target_x - x
        dy = target_y - y
        dist = math.hypot(dx, dy)

        # 2) Detecção de chegada
        arrived = dist < self.arr_tol
        passed_by = False
        if self.passby and self.last_to_target is not None:
            dot = dx * self.last_to_target[0] + dy * self.last_to_target[1]
            passed_by = dot < 0.0 and dist < 2.0 * self.arr_tol
        self.last_to_target = (dx, dy)

        if arrived or passed_by:
            self._on_arrival(self.current_idx)
            self.current_idx += 1
            self.locked_cone = None
            self._reset_cone_fix()
            self.last_to_target = None
            self.prev_heading_err = 0.0
            return

        # 3) PID de heading + velocidade adaptativa
        desired_heading = math.atan2(dy, dx)
        h_err = _wrap_pi(desired_heading - yaw)
        d_err = (h_err - self.prev_heading_err) / max(self.ctrl_dt, 1e-3)
        self.prev_heading_err = h_err

        omega = self.kp_h * h_err + self.kd_h * d_err
        omega = max(-self.w_max, min(self.w_max, omega))

        # cos^n cai rápido quando errando de lado → quase só gira até alinhar
        align = max(0.0, math.cos(h_err)) ** self.cos_n
        # Freia só no último waypoint (não interessa parar nos intermediários)
        is_last = self.current_idx == len(self.waypoints) - 1
        brake = min(dist / self.d_brake, 1.0) if is_last else 1.0
        v = self.v_max * align * brake
        if 0.0 < v < self.v_min:
            v = self.v_min

        tw = Twist()
        tw.linear.x = float(v)
        tw.angular.z = float(omega)
        self.pub_cmd.publish(tw)

        # publica alvo corrente pra visualização
        ts = PoseStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'odom'
        ts.pose.position.x = target_x
        ts.pose.position.y = target_y
        _, _, qz, qw = _yaw_to_quat(desired_heading)
        ts.pose.orientation.z = qz
        ts.pose.orientation.w = qw
        self.pub_target.publish(ts)

    def _find_matching_cone(self, wp: dict, x: float, y: float, yaw: float, cones):
        expected_x = wp['cone_x']
        expected_y = wp['cone_y']
        # Bearing esperado no FRAME do robô agora (igual ao que foi gravado):
        expected_bearing_world = wp['cone_bearing']  # rad relativo ao yaw GRAVADO
        # Mais robusto: usar a direção no mundo do cone gravado → robô_atual:
        # se a pose drifteou mas o cone está no mesmo lugar, casa pela posição.
        best = None
        best_score = float('inf')
        for cx, cy, _w in cones:
            dx = cx - expected_x; dy = cy - expected_y
            d_pos = math.hypot(dx, dy)
            if d_pos > self.r_match:
                continue
            # checagem angular extra: bearing relativo ao yaw atual deve ser
            # parecido com o gravado.
            cur_bearing = _wrap_pi(math.atan2(cy - y, cx - x) - yaw)
            d_ang = abs(_wrap_pi(cur_bearing - expected_bearing_world))
            if d_ang > self.bear_tol:
                continue
            score = d_pos + 0.3 * d_ang   # pequeno peso angular
            if score < best_score:
                best_score = score
                best = (cx, cy)
        return best

    def _candidates(self, wp: dict, cones):
        # Detecções dentro do raio de unicidade ao redor da posição esperada do
        # cone gravado. Usa max(unique, match) p/ NUNCA ficar menor que a região
        # de onde o match sai (senão a trava de unicidade teria uma brecha).
        r = max(self.cone_unique_radius, self.r_match)
        out = []
        for cx, cy, _w in cones:
            if math.hypot(cx - wp['cone_x'], cy - wp['cone_y']) <= r:
                out.append((cx, cy))
        return out

    def _maybe_publish_pose_fix(self, wp: dict, x, y, yaw, cones):
        # Confirmação ANTES de corrigir a pose — independente do snap do alvo.
        match = self._find_matching_cone(wp, x, y, yaw, cones)
        cands = self._candidates(wp, cones)
        n_cand = len(cands)
        confirmed = self._confirmer.update(match, n_cand)
        # telemetria do que ele está usando de referência (read-only p/ UI)
        if match is None:
            self._anchor = None
            self._anchor_status = 'idle'
            self._anchor_clutter = []
            self._anchor_confirm = 0
        else:
            self._anchor = match
            self._anchor_status = 'ambiguous' if n_cand > 1 else 'confirming'
            self._anchor_clutter = [c for c in cands if c != match]
        self._anchor_confirm = self._confirmer.count
        if not confirmed:
            return
        # Confirmado e único: delta = cone_gravado - cone_observado.
        dx, dy = cone_fix_delta((wp['cone_x'], wp['cone_y']), match)
        v = Vector3Stamped()
        v.header.stamp = self.get_clock().now().to_msg()
        v.header.frame_id = 'odom'
        v.vector.x = float(dx)
        v.vector.y = float(dy)
        self.pub_pose_fix.publish(v)
        self._cone_fix_done = True   # só uma vez por cone travado
        self._anchor_status = 'fixed'
        self.last_msg = f'pose_fix wp{self.current_idx}: Δ=({dx:+.2f}, {dy:+.2f})'

    def _on_arrival(self, idx: int):
        self._flash_led(1.0, 0.4, 0.0, mode=1, hold_ms=self.led_ms)  # laranja pisca
        self.last_msg = f'chegou wp{idx}'
        # Pequena pausa de velocidade — publica zero por uma iteração.
        self._stop_robot()

    # ------------------------------------------------------------------
    # Estado / LEDs / utilitários
    # ------------------------------------------------------------------
    def _state_tick(self):
        x, y, yaw, have_pose, cones = self._state_snapshot()
        state = {
            'mode': self.mode,
            'x': x, 'y': y, 'yaw': yaw,
            'have_pose': have_pose,
            'waypoints': self.waypoints,
            'current_idx': self.current_idx,
            'total': len(self.waypoints),
            'locked_cone': list(self.locked_cone) if self.locked_cone else None,
            'cones': [[c[0], c[1], c[2]] for c in cones],
            'anchor': list(self._anchor) if self._anchor else None,
            'anchor_status': self._anchor_status,
            'anchor_clutter': [list(c) for c in self._anchor_clutter],
            'anchor_confirm': [self._anchor_confirm, self.cone_confirm_frames],
            'msg': self.last_msg,
            'ts': time.time(),
        }
        self.pub_state.publish(String(data=json.dumps(state)))

        # PoseArray dos waypoints (pra visualização rviz/UI alternativa)
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'odom'
        for wp in self.waypoints:
            p = Pose()
            p.position.x = wp['x']
            p.position.y = wp['y']
            _, _, qz, qw = _yaw_to_quat(wp['yaw'])
            p.orientation.z = qz
            p.orientation.w = qw
            pa.poses.append(p)
        self.pub_wps.publish(pa)

    def _led_tick(self):
        # Se a chegada laranja já expirou, volta pra cor do modo.
        if time.time() < self.led_until:
            return
        if self.mode == MODE_IDLE:
            self._set_led(0.0, 0.0, 0.3, mode=0)        # azul fixo
        elif self.mode == MODE_RECORD:
            self._set_led(0.0, 0.5, 0.0, mode=1)        # verde piscando
        elif self.mode == MODE_PLAY:
            self._set_led(0.0, 0.3, 0.5, mode=2)        # ciano rotação

    def _set_led(self, r, g, b, mode=0):
        # Dedup: o _led_tick roda a 1 Hz e quase sempre repete a mesma cor.
        # Só publica quando muda de fato (a chegada laranja, via _flash_led,
        # registra como última cor, então o tick seguinte re-publica o modo).
        key = (float(r), float(g), float(b), int(mode))
        if key == self._last_led:
            return
        self._last_led = key
        c = ColorRGBA()
        c.r = float(r); c.g = float(g); c.b = float(b)
        c.a = float(mode)
        self.pub_leds.publish(c)

    def _flash_led(self, r, g, b, mode=1, hold_ms=600):
        self._set_led(r, g, b, mode)
        self.led_until = time.time() + hold_ms / 1000.0

    def _stop_robot(self):
        self.pub_cmd.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = TrekkingRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
