#!/usr/bin/env python3
"""path_follower — seguidor decisivo "reto + giro no lugar" p/ skid-steer que NÃO arqueia.

Por que existe (2026-06-25): o arc_calib provou que este robô não faz arco (vira
~3% do comando a 0.5 rad/s; só gira de verdade no lugar). Os controladores de
prateleira do nav2 brigam com isso (DWB arqueia e deriva pro obstáculo; o
RotationShim micro-gira em limite-ciclo = zigue-zague).

Este nó ignora o *tracking* do controller_server e segue o /plan (Theta*, retas
com cantos) com lógica DETERMINÍSTICA + 2 truques que faltavam:

  1. CARROT no caminho: mira um ponto ~`lookahead` m À FRENTE NO PLANO (não o
     goal lá no fim). Assim ele segue a FORMA do plano (o desvio pro vão), em vez
     de cortar reto pro destino e raspar o obstáculo.
  2. HISTERESE: começa a girar quando o erro de heading passa de `turn_enter`,
     mas só PARA de girar quando cai abaixo de `turn_exit` (bem menor). Sem isso
     ele girava e parava no MESMO limiar -> limite-ciclo = "pulinhos". Com
     histerese: gira decidido até alinhar, anda comprometido, re-gira só no canto.

Estados: idle | turning | driving | goal_turn | arrived.
Publica `follow_vel` (twist_mux prio 15 > nav_vel 10 = ignora controller_server;
< door 20 < unstuck 30). Enquanto há goal ativo SEMPRE publica (segura o mux).

A LÓGICA é pura (funções + DecisiveFollower) p/ testar sem ROS; o main() é a cola
de I/O (TF, /plan, status do goal, publisher, timer, CSV de diagnóstico).
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

Pt = Tuple[float, float]


def wrap(a: float) -> float:
    """normaliza ângulo p/ (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def closest_index(path: List[Pt], x: float, y: float) -> int:
    """índice do ponto do caminho mais próximo de (x,y)."""
    best_i, best_d = 0, float('inf')
    for i, (px, py) in enumerate(path):
        d = (px - x) ** 2 + (py - y) ** 2
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def carrot_point(path: List[Pt], i0: int, lookahead: float) -> Tuple[int, Pt]:
    """Ponto do caminho a ~`lookahead` m À FRENTE (arc-length) do índice i0.
    Devolve (índice, ponto). Se o caminho acaba antes, devolve o último (goal).
    É o 'carrot' do pure-pursuit — mas aqui ele só dá a DIREÇÃO pra reto/giro."""
    acc = 0.0
    n = len(path)
    for k in range(i0, n - 1):
        acc += math.hypot(path[k + 1][0] - path[k][0],
                          path[k + 1][1] - path[k][1])
        if acc >= lookahead:
            return k + 1, path[k + 1]
    return n - 1, path[-1]


def straight_deviation(path: List[Pt], i0: int, i1: int) -> float:
    """Máx desvio perpendicular de path[i0..i1] à corda path[i0]->path[i1].
    ~0 = trecho reto; grande = tem canto/curva no meio."""
    ax, ay = path[i0]
    bx, by = path[i1]
    ux, uy = bx - ax, by - ay
    norm = math.hypot(ux, uy)
    if norm < 1e-9:
        return 0.0
    worst = 0.0
    for k in range(i0 + 1, i1):
        px, py = path[k]
        d = abs((px - ax) * uy - (py - ay) * ux) / norm
        if d > worst:
            worst = d
    return worst


@dataclass
class FollowConfig:
    forward_speed: float = 0.30     # m/s no trecho reto (2026-06-27: 0.25->0.30 a
                                    # pedido — robô tava lento; teto do nav é 0.35)
    lookahead: float = 0.6          # m — distância do carrot à frente no plano.
                                    # 1.0 cortava o arco/raspava; 0.6 = VALIDADO (porta
                                    # real 4/4). 2026-06-27 tentei 0.4 (achei que sairia
                                    # mais da porta antes de virar) e a mira-no-canto
                                    # (rdp+segment_aim): AMBOS PIORARAM no maze sala_grande
                                    # (0.4 = hunting na boca da porta; mira-longe = não
                                    # segura a linha c/ a assimetria do skid -> oscila sem
                                    # avançar). Revertido p/ 0.6. O problema do maze é o
                                    # MAZE apertado demais (porta 0.93 + pinch 0.75 vs
                                    # inflação 0.45), não o follower. Ver ESTADO 06-28.
    lookahead_far: float = 1.5      # m — carrot ESTICADO quando o plano à frente é
                                    # RETO. Run hotmilk 07-08: carrot 0.6 amplifica
                                    # ruído de pose (12cm lateral = 12° = turn_enter)
                                    # -> 184 giros no lugar, 127 <10°, zigue-zague em
                                    # corredor. A 1.5m os mesmos 12cm = ~4.6° -> segue
                                    # reto. Só em trecho reto: curva mantém 0.6 (o 1.0
                                    # fixo de 06-27 cortava arco/raspava — o estico é
                                    # CONDICIONAL, não volta esse BO).
    straight_tol: float = 0.18      # m — desvio máx da corda p/ chamar de reto
                                    # (< meia-inflação 0.45; canto de verdade desvia
                                    # muito mais). <=0 desliga o carrot adaptativo.
                                    # 0.10→0.18 dono 07-09: zigue-zague residual
                                    # concentrava nos 32-36% do tempo em carrot
                                    # curto (0.6m); afrouxar estica em mais trechos
                                    # ondulados -> menos tempo curto -> menos giro
                                    # vai-e-volta (volta tinha 58% de giro
                                    # desperdiçado). 0.18 ainda < inflação, não
                                    # corta canto de verdade.
    stretch_clearance: float = 0.55 # m — só estica com ESPAÇO à frente (menor
                                    # leitura do scan no setor frontal >= isso).
                                    # A banda morta de drift lateral escala com
                                    # o carrot (la*sin(turn_enter)): 1.5m tolera
                                    # ±31cm fora do eixo; 0.6m, ±13cm. Fresta
                                    # exige ±13cm e o ruído de pose é ~12cm ->
                                    # IMPOSSÍVEL distinguir pela pose (e o plano
                                    # nasce no robô a cada replan, i0=0, então
                                    # offset ao plano é sempre ~0 — 1ª tentativa
                                    # 07-10 falhou por isso). Passagem apertada =
                                    # parede perto por definição -> gate pelo
                                    # scan: preso 262s->27s na fresta do sim
                                    # hotmilk_portas raspando a quina por chegar
                                    # de diagonal rasa. <=0 desliga o gate.
    turn_enter: float = 0.28        # rad (~16°) — acima disso COMEÇA a girar.
                                    # 12->16 (07-10): no sim hotmilk_portas 63%
                                    # dos giros eram VAI-E-VOLTA (+14/-14 que se
                                    # cancelam, herr de entrada mediano 14° = na
                                    # beirada do 12) — replan 1Hz balança a mira
                                    # ±14° e a banda de 12 não engole. Mesmo
                                    # padrão da VOLTA de campo 07-09 (58% do
                                    # giro cancelado). Alavanca mapeada no
                                    # ESTADO 07-09.
    turn_exit: float = 0.05         # rad (~3°)  — abaixo disso PARA de girar
                                    # (histerese; re-entrar pede turn_enter).
                                    # 7->3 (07-17, sim hotmilk): no driving o wz
                                    # é 0 travado, então sair 7° torto = andar
                                    # em diagonal até estourar os 16° -> dente-
                                    # de-serra no corredor RETO (64 giros, 51
                                    # <10°). Sair alinhado (~3°) é o que deixa
                                    # o reto ser reto; o vai-e-volta que motivou
                                    # os 7° hoje é coberto pela saída preditiva
                                    # (turn_stop_tau) + histerese de 16°.
    turn_stop_tau: float = 0.10     # s — saída PREDITIVA do giro (07-17): o
                                    # robô continua girando ~tau depois do
                                    # comando parar (pose lagada + inércia);
                                    # na run real ele saía do giro já ±16° do
                                    # OUTRO lado e 52% dos giros só desfaziam
                                    # o anterior. Solta o giro quando o yaw
                                    # PREVISTO em tau segundos cruza a banda.
                                    # 0.0 = comportamento antigo. A histerese
                                    # (re-entrar pede 16°) protege se tau
                                    # passar do ponto.
                                    # 0.25->0.10 (07-17, sim hotmilk): deslize
                                    # MEDIDO no CSV pós-parada = 2-4° em ~0.15s
                                    # (tau ~0.10); com 0.25 a previsão soltava
                                    # o giro ~6° antes do necessário (resíduo
                                    # 8-12°) e um spike de yaw_rate (salto de
                                    # pose AMCL) descontava até 24° -> saída com
                                    # 25.8° de resíduo e vai-e-volta 4->10. Se
                                    # o REAL (lag de pose maior sob carga)
                                    # voltar a ter overshoot, subir o param
                                    # turn_stop_tau via ROS — não voltar o
                                    # default às cegas.
    aim_tau: float = 2.0            # s — EMA na DIREÇÃO da mira, só com carrot
                                    # ESTICADO (trecho reto). Sim hotmilk 07-17:
                                    # a mira salta 13-15° entre replans (Theta*
                                    # 1Hz pivota ora numa parede inflada ora na
                                    # outra — biestável em corredor); apertar o
                                    # alinhamento (exit 3° + tau 0.10) fez o
                                    # robô PERSEGUIR o balanço: vai-e-volta
                                    # 4->15. Filtrado (tau 2s, swing 1Hz):
                                    # ±14° -> ±3.4°, nunca estoura o turn_enter
                                    # -> corredor reto fica reto. <=0 desliga
                                    # (mira crua no esticado).
    aim_tau_short: float = 0.8      # s — EMA da mira no carrot CURTO (curva).
                                    # Run 07-17 pós-filtro: zona curva perto da
                                    # casa (x 1-2.5 do hotmilk) seguia chovendo
                                    # giro (8-12/30s) — a 0.6m o mesmo ruído
                                    # lateral vira 2.5x mais graus e a mira
                                    # crua persegue replan. 0.8s engole o
                                    # balanço 1Hz (±20°->±9°) e ainda entra no
                                    # canto em <1s (~20cm a 0.25m/s; histerese
                                    # 16° já atrasava parecido). 1º avistamento
                                    # de canto NÃO tem lag (filtro semeia cru).
                                    # <=0 desliga (mira crua no curto).
    tick_dt: float = 0.05           # s — período do update() (nó seta
                                    # 1/rate_hz); usado só pra derivar o yaw.
    goal_xy_tol: float = 0.15       # m — chegou no goal (casa c/ goal_checker do nav2)
    goal_yaw_tol: float = 0.10      # rad (~6°) — encarou o yaw do goal
    rot_k: float = 3.0              # ganho P do giro (rad/s por rad)
    rot_min: float = 2.4            # rad/s — piso do giro (2.0 dava ~10°/s real =
                                    # rastejo na zona-morta 1.7; 2.4 ≈ 25°/s,
                                    # ver spec fluidez 07-02)
    rot_max: float = 4.5            # rad/s — teto do giro
    slow_radius: float = 0.4        # m — começa a frear o avanço perto do goal
    # 2026-06-26: 0.10 -> 0.22. CAMPO: perto do goal o ramp baixava p/ ~0.10-0.11
    # m/s, ABAIXO da zona-morta linear do robô real (pesado) -> mandava 0.11 e NÃO
    # ANDAVA = congelava sem finalizar nenhum ponto (precisava empurrar no controle).
    # 0.11 trava, 0.25 (cruise) anda -> a zona-morta tá no meio; 0.22 fica bem acima.
    # Overshoot a 20Hz ~1cm, irrelevante vs goal_xy_tol 0.15. Se AINDA rastejar/
    # travar, subir p/ 0.25; se passar do ponto, descer. (Zona-morta linear NUNCA
    # foi medida — só a do giro=1.7; o sim_actuator_model tb só modela o giro, por
    # isso o sim não pegava esse trava.)
    min_speed: float = 0.22         # m/s — avanço mínimo (acima da zona-morta)


@dataclass
class Cmd:
    vx: float
    wz: float
    state: str          # idle | turning | driving | goal_turn | arrived


class DecisiveFollower:
    def __init__(self, cfg: FollowConfig):
        self.cfg = cfg
        self.state = 'idle'
        self._turn_target = None  # bearing (map) congelado durante o turning
        self._prev_yaw = None     # p/ derivar a taxa de giro medida
        self._yaw_rate = 0.0      # rad/s, EMA (ruído de pose não vira taxa)
        self._aim_filt = None     # bearing (map) filtrado da mira (só esticado)
        self.dbg = {}        # diagnóstico do último update (logado pelo nó)

    def _turn_cmd(self, herr: float) -> float:
        """giro no lugar pelo MENOR ângulo: sinal = sinal do erro; magnitude P
        saturada entre rot_min e rot_max."""
        c = self.cfg
        mag = min(c.rot_max, max(c.rot_min, abs(herr) * c.rot_k))
        return math.copysign(mag, herr)

    def update(self, pose: Optional[Tuple[float, float, float]],
               path: Optional[List[Pt]], goal_active: bool,
               goal_yaw: Optional[float],
               front_clear: float = float('inf')) -> Cmd:
        c = self.cfg
        if pose is None or not goal_active or not path or len(path) < 2:
            self.state = 'idle'
            self._turn_target = None
            self._prev_yaw = None
            self._yaw_rate = 0.0
            self._aim_filt = None
            return Cmd(0.0, 0.0, 'idle')

        x, y, yaw = pose
        if self._prev_yaw is not None and c.tick_dt > 0.0:
            inst = wrap(yaw - self._prev_yaw) / c.tick_dt
            # clamp no máximo físico (~1.7 rad/s real, ver spin_calib): salto
            # de pose (correção do AMCL) não pode virar "taxa" e adiantar a
            # saída preditiva do giro.
            inst = max(-2.0, min(2.0, inst))
            self._yaw_rate += 0.3 * (inst - self._yaw_rate)
        self._prev_yaw = yaw
        gx, gy = path[-1]
        dist_goal = math.hypot(gx - x, gy - y)

        # 1) chegou no goal (xy) -> encara o yaw do goal, depois para
        if dist_goal <= c.goal_xy_tol:
            if goal_yaw is not None:
                yerr = wrap(goal_yaw - yaw)
                if abs(yerr) > c.goal_yaw_tol:
                    self.state = 'goal_turn'
                    self.dbg = {'i0': len(path) - 1, 'ci': len(path) - 1,
                                'n': len(path), 'ax': gx, 'ay': gy,
                                'herr_deg': math.degrees(yerr), 'dist_aim': 0.0,
                                'dist_goal': dist_goal}
                    return Cmd(0.0, self._turn_cmd(yerr), 'goal_turn')
            self.state = 'arrived'
            self._turn_target = None
            return Cmd(0.0, 0.0, 'arrived')

        # 2) CARROT no plano a ~lookahead à frente (segue a FORMA do caminho).
        #    ADAPTATIVO (07-08): se o plano até lookahead_far é RETO (desvio da
        #    corda <= straight_tol), mira LONGE — ruído de pose vira ângulo
        #    pequeno e não dispara giro (fim do zigue-zague em corredor). Curva
        #    à frente -> mira perto (0.6 validado), não corta canto.
        i0 = closest_index(path, x, y)
        ci, (ax, ay) = carrot_point(path, i0, c.lookahead)
        la = c.lookahead
        if (c.lookahead_far > c.lookahead and c.straight_tol > 0.0
                and (c.stretch_clearance <= 0.0
                     or front_clear >= c.stretch_clearance)):
            cf, (fx, fy) = carrot_point(path, i0, c.lookahead_far)
            if straight_deviation(path, i0, cf) <= c.straight_tol:
                ci, (ax, ay), la = cf, (fx, fy), c.lookahead_far
        bearing = math.atan2(ay - y, ax - x)
        # MIRA FILTRADA (07-17): EMA temporal na direção da mira — o replan
        # 1Hz do Theta* balança a mira ±14° (pivô biestável nas paredes
        # infladas) e o robô alinhado justo perseguia cada balançada
        # (vai-e-volta 4->15 no sim hotmilk). Tau por modo: esticado filtra
        # forte (corredor reto fica reto); curto filtra leve (engole o
        # chilique sem atrasar canto — 1º avistamento semeia cru, sem lag).
        # Filtro é CONTÍNUO na troca de modo (resetar a cada flap curto<->
        # esticado, comum na zona curva, viraria mira crua de novo).
        tau = c.aim_tau if la == c.lookahead_far else c.aim_tau_short
        if tau > 0.0:
            if self._aim_filt is None:
                self._aim_filt = bearing
            else:
                alpha = 1.0 - math.exp(-c.tick_dt / tau)
                self._aim_filt = wrap(
                    self._aim_filt + alpha * wrap(bearing - self._aim_filt))
            bearing = self._aim_filt
        else:
            self._aim_filt = None
        herr = wrap(bearing - yaw)
        dist_aim = math.hypot(ax - x, ay - y)
        self.dbg = {'i0': i0, 'ci': ci, 'n': len(path), 'ax': ax, 'ay': ay,
                    'herr_deg': math.degrees(herr), 'dist_aim': dist_aim,
                    'dist_goal': dist_goal, 'la': la}

        # 3) HISTERESE + ALVO CONGELADO: ao ENTRAR no giro trava o bearing-alvo
        #    (replans ~1Hz moviam o carrot NO MEIO do giro -> caçava alvo móvel,
        #    giros de 8-19s na run real de 07-02). Sai do giro -> re-olha o plano.
        if self.state == 'turning':
            if self._turn_target is not None:
                herr = wrap(self._turn_target - yaw)
            # saída preditiva: onde o yaw vai ESTAR quando o robô de fato
            # parar (taxa medida × tau). Projetado no sinal do erro: taxa na
            # direção errada (ex.: salto de pose do AMCL) só ATRASA a saída,
            # nunca adianta.
            herr_pred = (herr - self._yaw_rate * c.turn_stop_tau) \
                * math.copysign(1.0, herr)
            if abs(herr) <= c.turn_exit or herr_pred <= c.turn_exit:
                self.state = 'driving'
                self._turn_target = None
        else:
            if abs(herr) >= c.turn_enter:
                self.state = 'turning'
                self._turn_target = bearing
            else:
                self.state = 'driving'

        if self.state == 'turning':
            return Cmd(0.0, self._turn_cmd(herr), 'turning')

        # 4) de frente -> anda RETO (wz=0). Freia perto do goal.
        speed = c.forward_speed
        if dist_goal < c.slow_radius:
            speed = max(c.min_speed, c.forward_speed * dist_goal / c.slow_radius)
        return Cmd(speed, 0.0, 'driving')


def main(args=None):  # pragma: no cover - cola de I/O, validar no sim/bancada
    import csv as _csv
    import os as _os
    import time as _time

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Path
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException

    from .utils import quat_to_yaw, spin_node

    ACTIVE = {1, 2, 3}  # ACCEPTED, EXECUTING, CANCELING (igual door/unstuck)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    class PathFollowerNode(Node):
        def __init__(self):
            super().__init__('path_follower')
            g = {}
            for name, default in (
                ('forward_speed', 0.30), ('lookahead', 0.6),
                ('lookahead_far', 1.5), ('straight_tol', 0.18),
                # sector 40° (era 60 na 1ª run 07-10): a 60° a parede LATERAL
                # do corredor (~0.67m) entrava no gate (0.67/sin60=0.77 <0.9)
                # -> carrot curto 60% da run = zigue-zague de volta. A 40° a
                # mesma parede lê 1.04m (fora do gate) e as quinas de fresta,
                # quase À FRENTE na aproximação, continuam pegas a 0.9m.
                ('stretch_clearance', 0.55), ('clear_sector_deg', 40.0),
                ('turn_enter_deg', 16.0), ('turn_exit_deg', 3.0),
                ('goal_xy_tol', 0.15), ('goal_yaw_tol_deg', 6.0),
                ('rot_k', 3.0), ('rot_min', 2.4), ('rot_max', 4.5),
                ('slow_radius', 0.4), ('min_speed', 0.22), ('rate_hz', 20.0),
                ('turn_stop_tau', 0.10), ('aim_tau', 2.0),
                ('aim_tau_short', 0.8),
            ):
                self.declare_parameter(name, default)
                g[name] = self.get_parameter(name).value

            self.cfg = FollowConfig(
                forward_speed=g['forward_speed'], lookahead=g['lookahead'],
                lookahead_far=g['lookahead_far'],
                straight_tol=g['straight_tol'],
                stretch_clearance=g['stretch_clearance'],
                turn_enter=math.radians(g['turn_enter_deg']),
                turn_exit=math.radians(g['turn_exit_deg']),
                goal_xy_tol=g['goal_xy_tol'],
                goal_yaw_tol=math.radians(g['goal_yaw_tol_deg']),
                rot_k=g['rot_k'], rot_min=g['rot_min'], rot_max=g['rot_max'],
                slow_radius=g['slow_radius'], min_speed=g['min_speed'],
                turn_stop_tau=g['turn_stop_tau'], aim_tau=g['aim_tau'],
                aim_tau_short=g['aim_tau_short'],
                tick_dt=1.0 / g['rate_hz'])
            self.fol = DecisiveFollower(self.cfg)

            self._path: Optional[List[Pt]] = None
            self._goal_yaw: Optional[float] = None
            self._goal_active = {}

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            # publica follow_vel DIRETO no twist_mux (prio 15). 2026-06-26: tentei
            # rotear pelo collision (follow_vel_raw -> collision -> follow_vel) p/ o
            # reflexo proteger o seguidor, mas isso criou PONTO ÚNICO DE FALHA: quando
            # o bringup do Nav2 aborta antes de ativar o collision_monitor (bond
            # timeout do velocity_smoother na Pi lenta), o follow_vel não era
            # republicado e a NAV INTEIRA MORRIA (robô só andava no controle/unstuck).
            # Revertido: o seguidor não depende do collision pra dirigir.
            self.pub = self.create_publisher(Twist, 'follow_vel', 10)
            self.pub_state = self.create_publisher(String, 'follow_state', latched)

            self.create_subscription(Path, 'plan', self._on_plan,
                                     qos_profile_sensor_data)
            # clearance frontal p/ o gate do carrot esticado: lê o /scan_safe
            # (sanitizado, sem fantasma <0.15m — mesmo que unstuck/collision).
            # Falha graciosa: sem scan (ou >1s velho) -> inf = estica normal
            # (comportamento pré-gate; scan nunca derruba a nav).
            self._front_clear = float('inf')
            self._front_clear_t = 0.0
            self._clear_sector = math.radians(g['clear_sector_deg'])
            self.create_subscription(LaserScan, 'scan_safe', self._on_scan,
                                     qos_profile_sensor_data)
            for topic in ('navigate_to_pose/_action/status',
                          'navigate_through_poses/_action/status'):
                self.create_subscription(
                    GoalStatusArray, topic,
                    lambda m, t=topic: self._on_status(t, m), 10)

            self._last_state = None
            d = 'controle_web/logs'
            _os.makedirs(d, exist_ok=True)
            self._csv_f = open(_os.path.join(d, 'follow_debug.csv'), 'w', newline='')
            self._csv = _csv.writer(self._csv_f)
            self._csv.writerow(['t', 'state', 'x', 'y', 'yaw_deg', 'i0', 'ci', 'n',
                                'aim_x', 'aim_y', 'herr_deg', 'dist_aim',
                                'dist_goal', 'vx', 'wz', 'la', 'clear'])
            self._plan_path = _os.path.join(d, 'follow_plan_last.csv')
            # Snapshot do PRIMEIRO plano longo de cada goal (a FORMA do contorno —
            # o last.csv vira stub coladinho no goal). Resetado quando um novo goal
            # fica ativo (_on_status).
            self._plan_first_path = _os.path.join(d, 'follow_plan_first.csv')
            self._plan_snapped = False
            self._goal_active_any = False
            self._time = _time
            self.create_timer(1.0 / g['rate_hz'], self._tick)
            # flush do CSV em timer (8ª auditoria A5): flush por linha a 20 Hz
            # eram ~30k syncs/run castigando o SD da Pi. Mesmo padrão do
            # freeze_capture; perda máx. em power-cut = 2 s de log.
            self.create_timer(2.0, self._csv_f.flush)
            self.get_logger().info(
                'path_follower ativo: reto %.2fm/s, carrot %.1fm (reta %.1fm, '
                'tol %.2fm), gira>%.0f° até <%.0f°, giro %.1f–%.1f rad/s' % (
                    self.cfg.forward_speed, self.cfg.lookahead,
                    self.cfg.lookahead_far, self.cfg.straight_tol,
                    g['turn_enter_deg'], g['turn_exit_deg'],
                    self.cfg.rot_min, self.cfg.rot_max))

        def _on_plan(self, msg: Path):
            self._path = [(p.pose.position.x, p.pose.position.y)
                          for p in msg.poses]
            if msg.poses:
                q = msg.poses[-1].pose.orientation
                self._goal_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
                # dump do plano (sobrescreve) p/ eu inspecionar a FORMA depois
                try:
                    with open(self._plan_path, 'w', newline='') as f:
                        w = _csv.writer(f)
                        w.writerow(['x', 'y'])
                        w.writerows(self._path)
                except OSError:
                    pass
                # primeiro plano LONGO do goal (>=0.5 m) -> grava 1x a forma do
                # contorno num arquivo que NÃO é sobrescrito pelo stub final
                if not self._plan_snapped and len(self._path) >= 2:
                    plen = sum(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
                               for a, b in zip(self._path, self._path[1:]))
                    if plen >= 0.5:
                        try:
                            with open(self._plan_first_path, 'w', newline='') as f:
                                w = _csv.writer(f)
                                w.writerow(['x', 'y'])
                                w.writerows(self._path)
                            self._plan_snapped = True
                        except OSError:
                            pass

        def _on_scan(self, msg: LaserScan):
            best = float('inf')
            a = msg.angle_min
            for r in msg.ranges:
                if abs(wrap(a)) <= self._clear_sector and \
                        msg.range_min < r < msg.range_max and r > 0.05:
                    if r < best:
                        best = r
                a += msg.angle_increment
            self._front_clear = best
            self._front_clear_t = self._time.time()

        def _on_status(self, topic, msg):
            self._goal_active[topic] = any(st.status in ACTIVE
                                           for st in msg.status_list)
            active = any(self._goal_active.values())
            if active and not self._goal_active_any:
                # novo goal -> libera o snapshot do primeiro plano longo
                self._plan_snapped = False
            self._goal_active_any = active

        def _pose_map(self):
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time())
            except TransformException:
                return None
            t = tf.transform.translation
            q = tf.transform.rotation
            return (t.x, t.y, quat_to_yaw(q.x, q.y, q.z, q.w))

        def _tick(self):
            goal = any(self._goal_active.values()) if self._goal_active else False
            pose = self._pose_map()
            clear = self._front_clear
            if self._time.time() - self._front_clear_t > 1.0:
                clear = float('inf')   # scan velho/ausente -> não trava o estico
            cmd = self.fol.update(pose, self._path, goal, self._goal_yaw,
                                  front_clear=clear)

            # SEGURA O MUX: com goal ativo SEMPRE publica (prio 15 não expira ->
            # o controller_server prio 10 nunca assume e briga). Sem goal -> cala.
            if goal:
                m = Twist()
                m.linear.x = cmd.vx
                m.angular.z = cmd.wz
                self.pub.publish(m)
            if cmd.state != self._last_state:
                self._last_state = cmd.state
                self.pub_state.publish(String(data=cmd.state))

            if goal:
                d = self.fol.dbg
                x = pose[0] if pose else float('nan')
                y = pose[1] if pose else float('nan')
                yd = math.degrees(pose[2]) if pose else float('nan')
                self._csv.writerow([
                    round(self._time.time(), 3), cmd.state, round(x, 3),
                    round(y, 3), round(yd, 1), d.get('i0', ''), d.get('ci', ''),
                    d.get('n', ''), round(d.get('ax', float('nan')), 3),
                    round(d.get('ay', float('nan')), 3),
                    round(d.get('herr_deg', float('nan')), 1),
                    round(d.get('dist_aim', float('nan')), 3),
                    round(d.get('dist_goal', float('nan')), 3),
                    round(cmd.vx, 3), round(cmd.wz, 3),
                    d.get('la', ''),
                    round(clear, 2) if clear != float('inf') else ''])

    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
