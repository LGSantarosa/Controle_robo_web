#!/usr/bin/env python3
"""
Controlador ponto-a-ponto para a competição de trekking.

Máquina de estado:
  IDLE   — parado. Publica zero em /cmd_vel ocasionalmente e não interfere.
  RECORD — usuário dirige; /trekking/cmd save_point grava o waypoint atual
           + cone mais próximo no scan.
  PLAY   — percorre a lista de waypoints com PID heading + velocidade
           proporcional, fazendo "snap-to-cone" quando entra no raio de
           busca do cone gravado.

Entradas:
  /trekking/pose       PoseStamped       posição/yaw fundidos
  /trekking/cones      PoseArray         cones detectados em odom (com width na orientation.x)
  /trekking/cmd        String (JSON)     comandos vindos da UI
  /joy                 Joy               PS4: um botao grava waypoint (save_point)

Saídas:
  /cmd_vel             Twist
  /leds/color          ColorRGBA         (alpha = modo: 0 fixo, 1 pisca, 2 rotação)
  /trekking/state      String (JSON)     estado completo p/ a UI (~10 Hz)
  /trekking/waypoints  PoseArray         lista de waypoints (visualização)
  /trekking/target     PoseStamped       alvo corrente do PID (post-snap)
  controle_web/logs/trekking.csv        telemetria do PLAY (1 linha por tick)

Filosofia:
  - Sair voado: PID heading + v = v_max * cos²(err) * clamp(dist/d_brake, 0, 1).
  - Cone como landmark: ao chegar perto da posição esperada do cone, casa
    com cone detectado no scan e RE-ÂNCORA o alvo (alvo = cone_observado +
    (waypoint - cone_gravado)). Isso compensa drift acumulado por waypoint.
  - Sem TF — só /trekking/pose. Trekking não acorda se o pose_estimator
    não estiver publicando.
"""
import csv as _csv
import json
import math
import os as _os
import threading
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist, Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import ColorRGBA, String


MODE_IDLE   = 'idle'
MODE_RECORD = 'record'
MODE_PLAY   = 'play'


from .utils import quat_to_yaw as _quat_to_yaw, spin_node, wrap_pi as _wrap_pi
from .cone_pose_fix import ConeFixConfirmer, cone_fix_delta, cone_bearing


def _yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


@dataclass
class DriveConfig:
    """Autoridade de atuador do skid-steer — MEDIDA, não chutada.

    O `trekking_runner` congelou em 2026-06-12, uma semana antes das duas
    medições que mudaram todo o resto da stack:

      - `spin_calib` (06-19, fitas nas rodas): abaixo de ~1.7 rad/s comandados a
        roda simplesmente NÃO gira. Resposta real ~0.6*(cmd-1.7), satura ~1.7
        rad/s reais. Mandar wz fraco é comando MORTO (mesma lição do
        motion_guard: nunca escalar wz parcialmente).
      - `arc_calib` (06-25, no robô real): andando, o wz comandado rende 2-3%
        até 1.2 e no MÁXIMO 19% a 2.5. Veredito registrado: o robô não arqueia,
        é FÍSICO (o diferencial pequeno não vence a patinagem lateral do
        skid-steer), não é tuning.

    Consequência: existem só DOIS primitivos — RETO e GIRO NO LUGAR. O PID
    antigo tinha `omega_max=1.2`, ou seja, saturava todo giro DENTRO da
    zona-morta: no PLAY o robô nunca girou de verdade, só andava reto
    acumulando desvio. Estes números vêm do `path_follower`, validado em campo.
    """
    rot_deadzone: float = 1.7       # rad/s — abaixo disso a roda não vira (spin_calib)
    rot_k: float = 3.0              # ganho P do giro (rad/s por rad)
    rot_min: float = 2.4            # piso do giro (2.0 = rastejo ~10°/s reais)
    rot_max: float = 4.5            # teto do giro
    turn_enter: float = math.radians(20.0)  # entra em point-turn
    turn_exit: float = math.radians(6.0)    # e só solta aqui (histerese)
    v_max: float = 0.35             # m/s — cruzeiro da reta
    # path_follower 06-26, medido em campo: "0.11 trava, 0.25 anda" -> a
    # zona-morta linear está no meio; 0.22 fica bem acima. O freio do último
    # ponto NUNCA pode descer abaixo disso ou o robô congela sem finalizar.
    min_speed: float = 0.22         # m/s — avanço mínimo
    d_brake: float = 0.6            # m — começa a frear no ÚLTIMO waypoint


def pick_cone(cones, x, y, yaw, radius):
    """Cone-âncora do waypoint: o MAIS PRÓXIMO em QUALQUER direção, dentro de
    `radius`. Devolve (cx, cy, bearing_relativo_ao_yaw) ou None.

    Era só o semicírculo FRONTAL (±90°). BO medido no sim 2026-08-24: o dono
    gravou 4 pontos e só 2 saíram com cone — nos outros dois o cone real estava
    perto (1,44 m e 2,47 m) mas com bearing +170° e +134°, ou seja ATRÁS, porque
    ele já tinha passado ao lado do cone quando apertou o botão. Gravar assim é
    o natural; a regra é que estava errada.

    O `bearing` devolvido é RELATIVO ao yaw da gravação — é o que o PLAY usa
    depois pra conferir que casou com o cone certo.
    """
    best = None
    best_d = float('inf')
    for cx, cy, _w in cones:
        dx = cx - x
        dy = cy - y
        d = math.hypot(dx, dy)
        if d < 0.05 or d > radius:      # <5 cm = ruído em cima do robô
            continue
        if d < best_d:
            best_d = d
            best = (cx, cy, _wrap_pi(math.atan2(dy, dx) - yaw))
    return best


def drive_cmd(h_err, dist, is_last, turning, cfg):
    """(vx, wz, turning) — RETO ou GIRO NO LUGAR, nunca os dois.

    `turning` entra e sai por histerese pra não ficar liga-desliga na fronteira.
    Nenhum comando abaixo da zona-morta é emitido: ou gira de verdade, ou manda
    zero e anda reto.
    """
    if turning:
        turning = abs(h_err) > cfg.turn_exit
    else:
        turning = abs(h_err) >= cfg.turn_enter

    if turning:
        mag = min(cfg.rot_max, max(cfg.rot_min, abs(h_err) * cfg.rot_k))
        return 0.0, math.copysign(mag, h_err), True

    # Reto. O erro residual (< turn_enter) NÃO vira wz fraco: seria comando
    # morto e ainda cobraria autoridade do mux. A geometria do próximo
    # waypoint reabre o erro e o point-turn corrige de uma vez.
    v = cfg.v_max
    if is_last:
        v = min(v, cfg.v_max * min(dist / cfg.d_brake, 1.0))
    return max(v, cfg.min_speed), 0.0, False


class TrekkingRunner(Node):

    def __init__(self):
        super().__init__('trekking_runner')

        # --- Giro: reto OU point-turn (ver DriveConfig; o robô não arqueia) ---
        _d = DriveConfig()
        self.declare_parameter('rot_k', _d.rot_k)
        self.declare_parameter('rot_min', _d.rot_min)          # piso: fura a zona-morta 1.7
        self.declare_parameter('rot_max', _d.rot_max)
        self.declare_parameter('turn_enter_deg', math.degrees(_d.turn_enter))
        self.declare_parameter('turn_exit_deg', math.degrees(_d.turn_exit))

        # --- Velocidade ---
        self.declare_parameter('v_max', _d.v_max)              # m/s — cruzeiro da reta
        self.declare_parameter('min_speed', _d.min_speed)      # m/s — acima da zona-morta linear
        self.declare_parameter('d_brake', _d.d_brake)          # m — freia ao último ponto a partir daqui

        # --- Avanço de waypoint ---
        self.declare_parameter('arrival_tolerance', 0.25)    # m
        # Se o produto escalar do vetor pro alvo trocar de sinal: passou batido.
        self.declare_parameter('passby_detection', True)

        # --- Snap-to-cone ---
        self.declare_parameter('cone_search_radius', 1.5)    # m — começa a procurar
        self.declare_parameter('cone_match_radius',  0.6)    # m — distância máx do esperado
        self.declare_parameter('cone_bearing_tol_deg', 60.0) # ° — janela angular relativa
        # Raio de captura do cone na GRAVAÇÃO, em qualquer direção (o ±90° da
        # frente saiu — ver pick_cone). Mantém o alcance efetivo de antes.
        self.declare_parameter('cone_capture_radius', 3.0)   # m

        # --- Correção persistente de pose por cone-âncora (aditiva ao snap) ---
        self.declare_parameter('enable_cone_pose_fix', True)
        self.declare_parameter('cone_confirm_frames', 4)     # ciclos estáveis p/ confirmar
        self.declare_parameter('cone_stable_eps', 0.10)      # m — "mesma posição" entre ciclos
        self.declare_parameter('cone_unique_radius', 0.50)   # m — se >1 candidato aqui → ambíguo

        # --- LEDs ---
        self.declare_parameter('led_arrival_ms', 600)
        self.declare_parameter('publish_state_hz', 10.0)

        # --- Gravar ponto pelo CONTROLE ---
        # Gravar rota dirigindo e ter que largar o controle pra clicar "+ Ponto"
        # na web e ruim: o robo anda no meio do caminho. Botao no PS4 resolve.
        # PS4 (driver joy): 0=X, 1=O, 2=triangulo, 3=quadrado, 4=L1, 5=R1.
        # TRIANGULO (2), nao o X: no gamepad.js da web o ✕ e a TRAVA DE
        # EMERGENCIA e o quadrado e boost. Triangulo e circulo estao livres.
        self.declare_parameter('save_point_button', 2)   # triangulo
        self.declare_parameter('joy_enabled', True)

        # --- Loop ---
        self.declare_parameter('control_hz', 30.0)

        self.v_max   = float(self.get_parameter('v_max').value)
        self.d_brake = float(self.get_parameter('d_brake').value)
        self.drive_cfg = DriveConfig(
            rot_k=float(self.get_parameter('rot_k').value),
            rot_min=float(self.get_parameter('rot_min').value),
            rot_max=float(self.get_parameter('rot_max').value),
            turn_enter=math.radians(float(self.get_parameter('turn_enter_deg').value)),
            turn_exit=math.radians(float(self.get_parameter('turn_exit_deg').value)),
            v_max=self.v_max,
            min_speed=float(self.get_parameter('min_speed').value),
            d_brake=self.d_brake,
        )
        self.arr_tol = float(self.get_parameter('arrival_tolerance').value)
        self.passby  = bool(self.get_parameter('passby_detection').value)
        self.r_search= float(self.get_parameter('cone_search_radius').value)
        self.r_match = float(self.get_parameter('cone_match_radius').value)
        self.bear_tol= math.radians(float(self.get_parameter('cone_bearing_tol_deg').value))
        self.r_capture = float(self.get_parameter('cone_capture_radius').value)
        self.led_ms  = int(self.get_parameter('led_arrival_ms').value)
        self.state_dt= 1.0 / float(self.get_parameter('publish_state_hz').value)
        self.ctrl_dt = 1.0 / float(self.get_parameter('control_hz').value)
        self.save_btn = int(self.get_parameter('save_point_button').value)
        self.joy_enabled = bool(self.get_parameter('joy_enabled').value)
        self._joy_prev = 0     # estado anterior do botao (borda de subida)
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
        self._turning = False
        self.led_until = 0.0       # walltime até quando manter LED de chegada
        self._last_led = None      # última cor publicada (dedup do _led_tick 1 Hz)
        self.last_msg = ''

        # --- Subs ---
        self.create_subscription(PoseStamped, 'trekking/pose', self._on_pose, 20)
        self.create_subscription(PoseArray,   'trekking/cones', self._on_cones, 10)
        self.create_subscription(String,      'trekking/cmd', self._on_cmd, 10)

        # --- Pubs ---
        self.pub_cmd    = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_leds   = self.create_publisher(ColorRGBA, 'leds/color', 10)
        self.pub_state  = self.create_publisher(String, 'trekking/state', 10)
        self.pub_wps    = self.create_publisher(PoseArray, 'trekking/waypoints', 10)
        self.pub_target = self.create_publisher(PoseStamped, 'trekking/target', 10)
        self.pub_pose_fix = self.create_publisher(Vector3Stamped, 'trekking/pose_fix', 10)
        if self.joy_enabled:
            self.create_subscription(Joy, 'joy', self._on_joy, 10)

        # --- Telemetria do PLAY (padrão dos outros nós: motion_guard/freeze_capture) ---
        # O trekking era o ÚNICO modo autônomo sem CSV nenhum — rodava cego, e
        # por isso o "testei uma vez e não deu boa" nunca virou diagnóstico.
        # Uma linha por tick de controle (~30 Hz) só no PLAY; IDLE/RECORD não
        # escrevem nada.
        self._t0 = time.time()
        d = 'controle_web/logs'
        _os.makedirs(d, exist_ok=True)
        self._csv_f = open(_os.path.join(d, 'trekking.csv'), 'w', newline='')
        self._csv = _csv.writer(self._csv_f)
        self._csv.writerow([
            't', 'idx', 'n_wps', 'state', 'dist', 'h_err_deg', 'vx', 'wz',
            'x', 'y', 'yaw_deg', 'tx', 'ty', 'snap', 'snap_dx', 'snap_dy',
            'event'])
        # flush em timer, não por linha (8ª auditoria A5: flush a 30 Hz
        # castiga o SD da Pi). Perde <=2 s no pior caso.
        self.create_timer(2.0, self._csv_f.flush)

        self.create_timer(self.ctrl_dt, self._control_tick)
        self.create_timer(self.state_dt, self._state_tick)
        # Pulso de LED no modo (rotação/pisca) também precisa ser reenviado
        # periodicamente — a MEGA não decai sozinha.
        self.create_timer(1.0, self._led_tick)

        self.get_logger().info(
            f'trekking_runner: v_max={self.v_max:.2f} m/s, '
            f'rot_min={self.drive_cfg.rot_min:.1f} rad/s (zona-morta '
            f'{self.drive_cfg.rot_deadzone:.1f}), arrival={self.arr_tol*100:.0f} cm'
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
        elif cmd == 'set_cone':
            self._set_wp_cone(data)
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

    def _set_wp_cone(self, data: dict):
        # Corrige/limpa o cone preso a um waypoint (só faz sentido fora do PLAY;
        # a UI esconde o controle no PLAY). Mexe em self.waypoints sem lock extra,
        # igual a load_waypoints/_save_point (serializado pelo executor).
        try:
            idx = int(data.get('idx', -1))
        except (TypeError, ValueError):
            self.last_msg = 'set_cone: idx inválido'
            return
        if not (0 <= idx < len(self.waypoints)):
            self.last_msg = f'set_cone: idx {idx} fora da faixa'
            return
        wp = self.waypoints[idx]
        # Se estamos dirigindo justo este waypoint, invalida o snap travado pra
        # o cone novo (ou a ausência dele) valer já, não só no próximo waypoint.
        if self.mode == MODE_PLAY and idx == self.current_idx:
            self.locked_cone = None
        if data.get('clear'):
            wp['has_cone'] = False
            wp['cone_x'] = 0.0
            wp['cone_y'] = 0.0
            wp['cone_bearing'] = 0.0
            self.last_msg = f'wp{idx}: cone removido'
            return
        try:
            cx = float(data['cone_x'])
            cy = float(data['cone_y'])
        except (KeyError, TypeError, ValueError):
            self.last_msg = 'set_cone: cone_x/cone_y inválidos'
            return
        wp['cone_x'] = cx
        wp['cone_y'] = cy
        wp['has_cone'] = True
        # bearing relativo à pose GRAVADA do waypoint (igual à gravação) — sem
        # isso o gate angular do PLAY furaria após a troca.
        wp['cone_bearing'] = cone_bearing(wp['x'], wp['y'], wp['yaw'], cx, cy)
        self.last_msg = f'wp{idx}: cone → ({cx:.2f}, {cy:.2f})'

    def _save_point(self):
        x, y, yaw, have_pose, cones = self._state_snapshot()
        if not have_pose:
            self.last_msg = 'sem pose — pose_estimator parado?'
            return

        # Cone mais próximo em QUALQUER direção, dentro do raio de captura.
        cone = pick_cone(cones, x, y, yaw, self.r_capture)
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
        self._turning = False
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
        desired_heading = math.atan2(dy, dx)
        h_err = _wrap_pi(desired_heading - yaw)

        # 2) Detecção de chegada
        arrived = dist < self.arr_tol
        passed_by = False
        if self.passby and self.last_to_target is not None:
            dot = dx * self.last_to_target[0] + dy * self.last_to_target[1]
            passed_by = dot < 0.0 and dist < 2.0 * self.arr_tol
        self.last_to_target = (dx, dy)

        if arrived or passed_by:
            self._log_csv(x, y, yaw, target_x, target_y, dist, h_err, 0.0, 0.0,
                          'passby' if passed_by else 'arrive')
            self._on_arrival(self.current_idx)
            self.current_idx += 1
            self.locked_cone = None
            self._reset_cone_fix()
            self.last_to_target = None
            self._turning = False
            return

        # 3) RETO ou GIRO NO LUGAR — o robô não arqueia (arc_calib 06-25).
        # Freia só no último waypoint; nos intermediários passa voado.
        is_last = self.current_idx == len(self.waypoints) - 1
        v, omega, self._turning = drive_cmd(
            h_err, dist, is_last, self._turning, self.drive_cfg)

        tw = Twist()
        tw.linear.x = float(v)
        tw.angular.z = float(omega)
        self.pub_cmd.publish(tw)
        self._log_csv(x, y, yaw, target_x, target_y, dist, h_err, v, omega, '')

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

    def _on_joy(self, msg):
        """Botao do PS4 grava waypoint. So na BORDA DE SUBIDA — o joy_node
        repete a mensagem a 20 Hz (autorepeat), sem isso um toque viraria
        dezenas de waypoints. Bloqueado no PLAY, igual a web faz com o botao.
        """
        if self.save_btn >= len(msg.buttons):
            return
        agora = int(msg.buttons[self.save_btn])
        subiu = agora and not self._joy_prev
        self._joy_prev = agora
        if not subiu:
            return
        if self.mode == MODE_PLAY:
            self.last_msg = 'no PLAY nao grava ponto'
            return
        self._save_point()

    def _log_csv(self, x, y, yaw, tx, ty, dist, h_err, vx, wz, event):
        """Uma linha do PLAY. `snap_dx/dy` = o quanto o cone-âncora deslocou o
        alvo em relação ao waypoint gravado, ou seja, o DRIFT que ele corrigiu
        naquele ponto — é a métrica de precisão da rota."""
        snap_dx = snap_dy = ''
        if 0 <= self.current_idx < len(self.waypoints):
            wp = self.waypoints[self.current_idx]
            snap_dx = round(tx - wp['x'], 3)
            snap_dy = round(ty - wp['y'], 3)
        self._csv.writerow([
            round(time.time() - self._t0, 3), self.current_idx,
            len(self.waypoints), 'turn' if self._turning else 'drive',
            round(dist, 3), round(math.degrees(h_err), 1),
            round(vx, 3), round(wz, 3),
            round(x, 3), round(y, 3), round(math.degrees(yaw), 1),
            round(tx, 3), round(ty, 3),
            int(self.locked_cone is not None), snap_dx, snap_dy, event])

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

        # PoseArray dos waypoints (pra visualização rviz/UI alternativa).
        # Gate por assinante: no robô ninguém assina (a UI usa o JSON do
        # /trekking/state) — montar/publicar a 10 Hz à toa é CPU da Pi
        # (L1 da AUDITORIA_2026-06-11; mesmo padrão do pose_estimator).
        if self.pub_wps.get_subscription_count() == 0:
            return
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
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._csv_f.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
