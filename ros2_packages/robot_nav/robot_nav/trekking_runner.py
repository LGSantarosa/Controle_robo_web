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
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
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


def trail_step(last, x, y, yaw, t, ds_min, dyaw_min):
    """Decide se (x, y, yaw) vira uma migalha da trilha. `None` = nao vale.

    POR QUE POR DISTANCIA E NAO POR TEMPO: gravando a 30 Hz por tempo, o robo
    parado gera milhares de migalhas identicas e a rota vira um arquivo inutil;
    e quem dirige devagar (que e justamente onde a precisao importa) gastaria
    mais pontos que quem passa voado. Por distancia, a densidade da trilha
    acompanha o CAMINHO, nao o relogio.

    O gate de yaw existe porque point-turn anda 0 m: sem ele, um giro no lugar
    — que e exatamente onde o robo mais erra — nao deixaria migalha nenhuma.

    v e wz saem da DIFERENCA de pose, nao do cmd_vel: e o que o robo fez, nao o
    que foi mandado. Em skid-steer a diferenca entre os dois e o que interessa.
    """
    if last is None:
        return {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4),
                't': round(t, 2), 'v': 0.0, 'wz': 0.0}
    ds = math.hypot(x - last['x'], y - last['y'])
    dyaw = _wrap_pi(yaw - last['yaw'])
    if ds < ds_min and abs(dyaw) < dyaw_min:
        return None
    dt = t - last['t']
    return {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4),
            't': round(t, 2),
            'v':  round(ds / dt, 3) if dt > 1e-3 else 0.0,
            'wz': round(dyaw / dt, 3) if dt > 1e-3 else 0.0}


def atualiza_trava_do_cone(travado, candidato, raio_de_troca):
    """Qual deteccao o PLAY usa como ancora agora.

    Antes a trava era DEFINITIVA: `if self.locked_cone is None` — a primeira
    deteccao que aparecesse virava a ancora pro resto do waypoint. Isso era
    tolerável enquanto o snap so engatava a 1,5 m do cone (perto = medida boa).
    Engatando CEDO (a 3,5 m) a primeira medida e a PIOR de todas: cone longe tem
    poucos pontos no scan e a posicao balanca. Congelar essa seria trocar
    "tarde e violento" por "cedo e errado".

    Agora a trava REFINA: enquanto a nova deteccao estiver perto da travada, ela
    substitui — e o mesmo cone, medido melhor de perto. Longe demais e OUTRO
    cone e nao troca (senao um cone vizinho roubaria a ancora no meio da
    aproximacao). Sem deteccao no tick, segura a ultima: piscada do detector nao
    pode largar a ancora.
    """
    if candidato is None:
        return travado
    if travado is None:
        return candidato
    d = math.hypot(candidato[0] - travado[0], candidato[1] - travado[1])
    return candidato if d <= raio_de_troca else travado


def trail_progress(trail, p, x, y, window=25):
    """Avanca o marcador de progresso na trilha. NUNCA anda pra tras.

    Procura a migalha mais proxima do robo numa JANELA a frente de `p`, e nao
    na trilha inteira, por dois motivos: (1) rota que passa duas vezes perto do
    mesmo lugar (ou volta pelo mesmo corredor) faria o robo "pular" pro trecho
    errado e cortar caminho; (2) custo — varrer 2000 migalhas a 30 Hz e desperdicio.
    Monotonico pela mesma razao: retroceder o progresso e como esquecer o que ja
    andou, e o robo fica em looping num trecho.
    """
    if not trail:
        return p
    melhor, melhor_d = p, float('inf')
    for i in range(p, min(p + window, len(trail))):
        d = math.hypot(trail[i]['x'] - x, trail[i]['y'] - y)
        if d < melhor_d:
            melhor_d, melhor = d, i
    return melhor


def trail_lookahead(trail, p, dist):
    """Ponto de mira: caminha `dist` metros a frente de `p` SOBRE a trilha.

    Devolve (x, y, is_last). `is_last` avisa que a mira bateu no fim da trilha —
    e o que faz o robo frear no final em vez de chegar voado.

    Mirar a frente (e nao na migalha mais proxima) e o que faz o seguidor
    convergir suave pro caminho em vez de serpentear: perseguir o ponto colado
    vira correcao violenta a cada tick.
    """
    if not trail:
        return None
    i = min(p, len(trail) - 1)
    acc = 0.0
    while i + 1 < len(trail) and acc < dist:
        acc += math.hypot(trail[i + 1]['x'] - trail[i]['x'],
                          trail[i + 1]['y'] - trail[i]['y'])
        i += 1
    return trail[i]['x'], trail[i]['y'], i >= len(trail) - 1


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
        # 1.5 -> 3.5 (2026-08-25). O gatilho media do robo ao CONE, entao ele
        # dependia de quao perto do cone o waypoint tinha sido gravado:
        # gravando colado (0,41 m) o snap engatava EM CIMA do waypoint e o alvo
        # pulava 28 cm de lado a 32 cm do ponto — point-turn de 48° ao lado do
        # cone, medido no tick-a-tick, que e o "foi de cara no cone". Gravando a
        # 1,5 m (o que o dono quer, pra nao atropelar) ele NUNCA engatava antes
        # de chegar. 3,5 m cobre o raio de captura da gravacao (3,0): o que deu
        # pra gravar da pra casar. O ponto-chave: a correcao lateral vira
        # point-turn de qualquer jeito (o drive_cmd proibe reto+giro, o robo nao
        # arqueia) — engatar cedo nao remove o giro, ele MUDA DE LUGAR, pra
        # longe do cone, onde girar e seguro.
        self.declare_parameter('cone_search_radius', 3.5)    # m — começa a procurar
        self.declare_parameter('cone_match_radius',  0.6)    # m — distância máx do esperado
        self.declare_parameter('cone_bearing_tol_deg', 60.0) # ° — janela angular relativa
        # Raio de captura do cone na GRAVAÇÃO, em qualquer direção (o ±90° da
        # frente saiu — ver pick_cone). Mantém o alcance efetivo de antes.
        self.declare_parameter('cone_capture_radius', 3.0)   # m
        # Raio em que uma deteccao nova ainda e "o mesmo cone" e refina a trava.
        # Ver atualiza_trava_do_cone.
        self.declare_parameter('cone_lock_track_radius', 0.4)  # m

        # --- Correção persistente de pose por cone-âncora (aditiva ao snap) ---
        self.declare_parameter('enable_cone_pose_fix', True)
        self.declare_parameter('cone_confirm_frames', 4)     # ciclos estáveis p/ confirmar
        self.declare_parameter('cone_stable_eps', 0.10)      # m — "mesma posição" entre ciclos
        self.declare_parameter('cone_unique_radius', 0.50)   # m — se >1 candidato aqui → ambíguo

        # --- Seguir a trilha no PLAY ---
        # Com trilha gravada, o alvo deixa de ser o waypoint e passa a ser um
        # ponto de MIRA que corre sobre a trilha. Rota sem trilha (as antigas)
        # cai no caminho de sempre — nada muda pra elas.
        self.declare_parameter('follow_trail', True)
        self.declare_parameter('lookahead', 0.6)      # m — distância da mira
        self.declare_parameter('trail_window', 25)    # migalhas varridas por tick

        # --- Trilha densa (teach-and-repeat) ---
        # O RECORD so guardava os waypoints que voce apertou no botao: entre um
        # e outro, o PLAY inventa uma RETA. Se voce contornou alguma coisa, a
        # reta passa por cima. A trilha grava por onde o robo REALMENTE andou.
        # Amostrada por DISTANCIA (nao por tempo): parado nao gera migalha, e
        # andar devagar nao enche o arquivo. Curva entra pelo gate de yaw, senao
        # um giro no lugar (que anda 0 m) sumiria da trilha.
        self.declare_parameter('trail_ds', 0.10)          # m entre migalhas
        self.declare_parameter('trail_dyaw_deg', 10.0)    # ° que forcam migalha
        self.declare_parameter('trail_max', 20000)        # teto (2 km a 10 cm)

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
        self.follow_trail = bool(self.get_parameter('follow_trail').value)
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.trail_window = int(self.get_parameter('trail_window').value)
        self.trail_ds = float(self.get_parameter('trail_ds').value)
        self.trail_dyaw = math.radians(float(self.get_parameter('trail_dyaw_deg').value))
        self.trail_max = int(self.get_parameter('trail_max').value)
        self.passby  = bool(self.get_parameter('passby_detection').value)
        self.r_search= float(self.get_parameter('cone_search_radius').value)
        self.r_match = float(self.get_parameter('cone_match_radius').value)
        self.bear_tol= math.radians(float(self.get_parameter('cone_bearing_tol_deg').value))
        self.r_lock_track = float(self.get_parameter('cone_lock_track_radius').value)
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
        # trilha: [{x, y, yaw, t, v, wz}, ...] — por onde o robo passou no RECORD
        self.trail = []
        self._trail_cheia = False   # avisa 1x só quando bate o teto
        self.trail_p = 0            # progresso na trilha (monotônico) no PLAY
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
        # Trilha: LATCHED (transient_local) e publicada por EVENTO, nao no tick.
        # A trilha inteira a 10 Hz seriam centenas de KB/s pra nada; quem assina
        # (a web, ao salvar a rota) so precisa da ULTIMA versao, e o latch
        # entrega ela pra quem assinar DEPOIS — sem isso, reiniciar a web no
        # meio de uma gravacao perderia a trilha inteira.
        # JSON e nao PoseArray: PoseArray so carrega pose, e ai o v/wz/t (os
        # "dados de movimento") morreriam antes de chegar no disco.
        self.pub_trail = self.create_publisher(
            String, 'trekking/trail',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST))
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
        elif cmd == 'get_trail':
            self._publish_trail()
            self.last_msg = f'trilha: {len(self.trail)} migalhas'
        elif cmd == 'save_point':
            if self.mode != MODE_RECORD:
                self.mode = MODE_RECORD
            self._save_point()
        elif cmd == 'play':
            self._start_play()
        elif cmd == 'stop':
            was_record = self.mode == MODE_RECORD
            self.mode = MODE_IDLE
            self.current_idx = 0
            self._stop_robot()
            self.last_msg = 'parado'
            if was_record:
                self._publish_trail()   # fecha a gravação com a trilha no fio
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
            self.trail = self._sanitize_trail(data.get('trail') or [])
            self.trail_p = 0
            self.current_idx = 0
            if errors:
                self.last_msg = f'{len(sane)} waypoints carregados ({errors} ignorados)'
            else:
                self.last_msg = f'{len(sane)} waypoints carregados'
        elif cmd == 'clear':
            self.waypoints = []
            self.trail = []
            self._trail_cheia = False
            self.trail_p = 0
            self.current_idx = 0
            self._publish_trail()
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
            # rota antiga (gravada antes da trilha) nao tem o elo -> -1
            'trail_i':      int(w.get('trail_i', -1)),
        }

    def _sanitize_trail(self, pts) -> list:
        """Migalha invalida some SOZINHA — trilha meia-boca vale mais que rota
        recusada. Ao contrario do waypoint, aqui nao ha o que consertar: um
        ponto ruim no meio de mil so precisa nao existir."""
        sane = []
        for p in pts:
            try:
                sane.append({'x': float(p['x']), 'y': float(p['y']),
                             'yaw': float(p.get('yaw', 0.0)),
                             't': float(p.get('t', 0.0)),
                             'v': float(p.get('v', 0.0)),
                             'wz': float(p.get('wz', 0.0))})
            except (KeyError, TypeError, ValueError):
                continue
        descartadas = len(pts) - len(sane)
        if descartadas:
            self.get_logger().warn(f'trilha: {descartadas} migalhas inválidas descartadas')
        return sane

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

    def _sample_trail(self):
        """Uma migalha da trilha, se `trail_step` disser que vale a pena."""
        x, y, yaw, have_pose, _ = self._state_snapshot()
        if not have_pose:
            return
        if len(self.trail) >= self.trail_max:
            # Teto: para de gravar e AVISA (1x). Jogar fora as migalhas antigas
            # seria pior — perderia o comeco da rota calado.
            if not self._trail_cheia:
                self._trail_cheia = True
                self.get_logger().warn(
                    f'trilha atingiu o teto de {self.trail_max} migalhas — '
                    'parei de gravar (aumente trail_max ou trail_ds)')
            return
        p = trail_step(self.trail[-1] if self.trail else None, x, y, yaw,
                       time.time() - self._t0, self.trail_ds, self.trail_dyaw)
        if p is not None:
            self.trail.append(p)

    def _publish_trail(self):
        self.pub_trail.publish(String(data=json.dumps(self.trail)))

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
            # Onde este waypoint cai DENTRO da trilha. E o elo que deixa o PLAY
            # seguir a trilha e ainda saber em que migalha o cone/ancora entra.
            'trail_i': len(self.trail) - 1 if self.trail else -1,
        }
        self.waypoints.append(wp)
        idx = len(self.waypoints) - 1
        if cone:
            self._flash_led(0.0, 0.5, 0.0, mode=1)   # verde pisca → ok
            self.last_msg = f'wp{idx}: ({x:.2f}, {y:.2f}) + cone'
        else:
            self._flash_led(1.0, 0.7, 0.0, mode=1)   # amarelo pisca → sem cone
            self.last_msg = f'wp{idx}: ({x:.2f}, {y:.2f}) — cone não visto'
        self._publish_trail()

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
        self.trail_p = 0
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
        if self.mode == MODE_RECORD:
            self._sample_trail()
            return
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

        # 1) Re-âncora pelo cone. O snap vira um OFFSET DE DERIVA (cone visto −
        # cone gravado) em vez de um alvo novo. Matematicamente é o mesmo que
        # antes pro waypoint (cone_obs + (wp − cone_grav) == wp + (cone_obs −
        # cone_grav)), mas escrito assim ele também se aplica à trilha inteira:
        # o que o cone mede é a MENTIRA DA ODOMETRIA, não um destino diferente.
        sx = sy = 0.0
        if wp['has_cone']:
            # Gatilho pelo que estiver mais perto: o CONE esperado ou o
            # WAYPOINT. Só pelo cone, quem grava longe dele nunca engata antes
            # de chegar; só pelo waypoint, quem grava colado engata tarde. O par
            # cobre os dois jeitos de gravar.
            perto = min(math.hypot(wp['cone_x'] - x, wp['cone_y'] - y),
                        math.hypot(wp['x'] - x, wp['y'] - y))
            if perto < self.r_search:
                # Roda TODO tick, não só na aquisição: a trava refina conforme
                # chega perto (ver atualiza_trava_do_cone).
                self.locked_cone = atualiza_trava_do_cone(
                    self.locked_cone, self._find_matching_cone(wp, x, y, yaw, cones),
                    self.r_lock_track)
            if self.locked_cone is not None:
                sx = self.locked_cone[0] - wp['cone_x']
                sy = self.locked_cone[1] - wp['cone_y']

        # 1b) Correção PERSISTENTE de pose por cone-âncora (aditiva: não mexe no
        # alvo acima). Gates conservadores no _confirmer; na dúvida não corrige.
        if self.enable_cone_pose_fix and wp['has_cone'] and not self._cone_fix_done:
            self._maybe_publish_pose_fix(wp, x, y, yaw, cones)

        # 2) Alvo: mira na trilha, ou o próprio waypoint (rota sem trilha).
        seguindo_trilha = bool(self.follow_trail and self.trail)
        if seguindo_trilha:
            self.trail_p = trail_progress(self.trail, self.trail_p, x - sx,
                                          y - sy, self.trail_window)
            mira = trail_lookahead(self.trail, self.trail_p, self.lookahead)
            target_x, target_y = mira[0] + sx, mira[1] + sy
            fim_da_trilha = mira[2]
        else:
            target_x, target_y = wp['x'] + sx, wp['y'] + sy
            fim_da_trilha = False

        dx = target_x - x
        dy = target_y - y
        dist = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)
        h_err = _wrap_pi(desired_heading - yaw)

        # 3) Detecção de chegada no WAYPOINT (o alvo pode ser a mira, mas quem
        # marca progresso da rota continua sendo o waypoint — é ele que carrega
        # o cone e é nele que o LED pisca).
        d_wp = math.hypot(wp['x'] + sx - x, wp['y'] + sy - y)
        passed_by = False
        if seguindo_trilha and wp.get('trail_i', -1) >= 0:
            # Chegou quando o PROGRESSO passou da migalha do waypoint. Distância
            # não serve aqui: a mira vai 0,6 m à frente, então o robô nunca fica
            # a <25 cm do alvo e a chegada por distância jamais dispararia.
            arrived = self.trail_p >= wp['trail_i']
        else:
            arrived = d_wp < self.arr_tol
            if self.passby and self.last_to_target is not None:
                dot = dx * self.last_to_target[0] + dy * self.last_to_target[1]
                passed_by = dot < 0.0 and dist < 2.0 * self.arr_tol
            self.last_to_target = (dx, dy)

        if arrived or passed_by:
            self._log_csv(x, y, yaw, target_x, target_y, d_wp, h_err, 0.0, 0.0,
                          'passby' if passed_by else 'arrive')
            self._on_arrival(self.current_idx)
            self.current_idx += 1
            self.locked_cone = None
            self._reset_cone_fix()
            self.last_to_target = None
            self._turning = False
            return

        # 4) RETO ou GIRO NO LUGAR — o robô não arqueia (arc_calib 06-25).
        # Freia só no fim; nos pontos intermediários passa voado.
        is_last = self.current_idx == len(self.waypoints) - 1
        if seguindo_trilha:
            # Com mira, `dist` é sempre ~lookahead e nunca frearia. Quem manda
            # na freada é a distância que falta até o WAYPOINT final.
            is_last = is_last and fim_da_trilha
            dist = d_wp
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
            # so a contagem: a trilha inteira vai no /trekking/trail (latched)
            'trail_n': len(self.trail),
            'trail_p': self.trail_p,
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
