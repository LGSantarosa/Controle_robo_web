#!/usr/bin/env python3
"""Travessia de porta — door_crossing.

O nav2 NÃO atravessa portas estreitas: entra torto, o batente entra na caixa
do PolygonStop e congela (5/22 freezes do bag de 2026-06-12 — os outros 17
eram fantasmas do LD06, ver scan_sanitizer). Este nó assume a travessia
quando o robô chega na zona de uma porta MARCADA pelo usuário:

  IDLE -> STAGING (vai pro ponto de preparação no eixo da porta)
       -> ROTATING (gira no lugar até encarar o eixo: |lat|<8cm E |yaw|<5°)
       -> CROSSING (reto e devagar, micro-correção no eixo, vigiando o vão;
                    publica estado 'crossing' = gate da máscara de batente
                    no scan_sanitizer)
       -> solta pro nav2 (passou do centro + exit_margin)

Collision monitor 100% ativo fora do CROSSING. Aborta e devolve pro nav2 se:
pose (TF map->base_link) sumir, goal morrer, scan envelhecer, vão fechar ou
timeout. Lógica pura (sem ROS) testável offline; cola de I/O no main() —
mesmo padrão do unstuck_supervisor. Spec:
docs/superpowers/specs/2026-06-12-zonas-de-porta-design.md
"""
import math
import time
from dataclasses import dataclass
from typing import List, NamedTuple, Optional, Tuple

import numpy as np


# ---- geometria pura --------------------------------------------------------

class DoorGeom(NamedTuple):
    cx: float
    cy: float
    half_width: float
    tx: float   # tangente unitária (ao longo da parede, a->b)
    ty: float
    nx: float   # normal unitária (eixo de travessia; sinal vem de `side`)
    ny: float


def door_geometry(a: Tuple[float, float], b: Tuple[float, float]) -> DoorGeom:
    """Centro/eixos da porta a partir dos 2 batentes clicados (frame do mapa)."""
    ax, ay = a
    bx, by = b
    w = math.hypot(bx - ax, by - ay)
    if w <= 0.0:
        raise ValueError('batentes coincidentes')
    tx, ty = (bx - ax) / w, (by - ay) / w
    return DoorGeom((ax + bx) / 2.0, (ay + by) / 2.0, w / 2.0,
                    tx, ty, -ty, tx)


def door_progress_lateral(g: DoorGeom, x: float, y: float,
                          side: int) -> Tuple[float, float]:
    """(progresso s, offset lateral d) do ponto no frame da porta.

    s < 0 = ainda do lado de aproximação (side escolhe qual lado é "antes");
    d = distância assinada ao eixo de travessia, ao longo da parede.
    """
    px, py = x - g.cx, y - g.cy
    s = (px * g.nx + py * g.ny) * side
    d = px * g.tx + py * g.ty
    return s, d


def crossing_yaw(g: DoorGeom, side: int) -> float:
    """Yaw do mapa que encara o eixo de travessia na direção `side`."""
    return math.atan2(side * g.ny, side * g.nx)


def fit_lat(g: DoorGeom, robot_half_width: float, fit_margin: float) -> float:
    """Folga lateral (m) pra passar RETO sem encostar nos batentes: meia-largura
    do vão MARCADO menos a meia-largura do robô menos uma margem. 'Dá pra ir reto
    daqui sem bater?' = |offset lateral| <= fit_lat. Auto-ajusta à porta: vão
    largo relaxa (não precisa do meio exato), vão apertado exige mais centro.
    Nunca negativo (porta mais estreita que o robô -> 0 = só dead-center)."""
    return max(0.0, g.half_width - robot_half_width - fit_margin)


def nav_engaging(linear_x: float, nav_move_lin: float) -> bool:
    """True se o nav NÃO está dando ré — i.e., avançando OU girando no lugar
    pra alinhar (linear≈0). Antes o gate exigia avançar (linear>thresh) e a
    porta NÃO armava na hora que o robô chegava torto e o RotationShim queria
    girar (linear≈0) -> door_crossing piscava pra idle -> unstuck escapava do
    standdown e sabotava. Como o DWB roda com min_vel_x:0.0 (não dá ré em
    navegação normal), nunca há ré sustentada no ramo do nav, então isto é
    seguro (não reintroduz o 'atravessar de costas')."""
    return linear_x > -nav_move_lin


def nearest_door_in_zone(pose: Optional[Tuple[float, float, float]],
                         doors: List[dict], zone_radius: float) -> Optional[dict]:
    """Porta marcada mais próxima cujo CENTRO está dentro de zone_radius do
    robô, IGNORANDO o bearing (só proximidade). None se nenhuma.

    Usado pra sinalizar 'approaching' no /door_zone (gate do standdown do
    unstuck), separado da decisão de CONDUZIR (que usa o cone, em _pick_door).
    Ignora o cone de propósito: a sabotagem do unstuck era pior justamente na
    chegada torta (porta fora do cone)."""
    if pose is None:
        return None
    x, y, _ = pose
    best, best_d = None, zone_radius
    for d in doors:
        g = door_geometry(tuple(d['a']), tuple(d['b']))
        dist = math.hypot(x - g.cx, y - g.cy)
        if dist <= best_d:
            best_d, best = dist, d
    return best


GAP_CORRIDOR_HALF_W = 0.28   # m — meia-largura do corredor vigiado (corpo+3cm)
GAP_MAX_X = 0.80             # m — até onde olhar à frente


def gap_ahead(ranges, angle_min: float, angle_increment: float,
              pose: Tuple[float, float, float],
              jambs: List[Tuple[float, float]], jamb_r: float) -> float:
    """Distância (m) do obstáculo mais próximo no corredor à FRENTE do robô,
    descontando os discos dos batentes marcados (em frame do MAPA). inf = livre.

    Usado no CROSSING: pessoa/obstáculo no vão -> aborta; os batentes que o
    usuário marcou não contam (são a parede da própria porta).
    """
    if angle_increment == 0.0:
        return math.inf
    r = np.asarray(ranges, dtype=np.float64)
    if r.size == 0:
        return math.inf
    ok = np.isfinite(r) & (r > 0.0)
    r = np.where(ok, r, 0.0)
    a = angle_min + np.arange(r.size) * angle_increment
    x = r * np.cos(a)
    y = r * np.sin(a)
    sel = ok & (x > 0.0) & (x <= GAP_MAX_X) & (np.abs(y) <= GAP_CORRIDOR_HALF_W)
    if not sel.any():
        return math.inf
    if jambs:
        px, py, pyaw = pose
        c, s = math.cos(pyaw), math.sin(pyaw)
        mx = px + x * c - y * s
        my = py + x * s + y * c
        for jx, jy in jambs:
            sel &= ((mx - jx) ** 2 + (my - jy) ** 2) > jamb_r ** 2
        if not sel.any():
            return math.inf
    return float(x[sel].min())


def _ccw(a, b, c) -> float:
    """Produto vetorial (orientação) de a->b vs a->c. >0 esq, <0 dir."""
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_cross(p1, p2, p3, p4) -> bool:
    """True se os segmentos p1-p2 e p3-p4 se cruzam de verdade (não conta só
    encostar nas pontas/colinear — quer um cruzamento claro)."""
    d1, d2 = _ccw(p3, p4, p1), _ccw(p3, p4, p2)
    d3, d4 = _ccw(p1, p2, p3), _ccw(p1, p2, p4)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def plan_crosses_door(plan, a, b) -> bool:
    """True se a rota planejada (lista de (x,y) no frame do mapa) atravessa o
    vão da porta (segmento entre os 2 batentes a e b).

    É o sinal de arming robusto (achado 2026-06-17): o gate antigo por bearing
    (porta "na frente" do robô) só fechava DEPOIS do Nav2 curvar o robô pra
    porta -> o door assumia tarde, colado/torto. O plano cruzar a porta diz
    'o goal é do outro lado DESTA porta' independente de pra onde o nariz aponta
    -> arma cedo e reto, antes da curva. E não dá falso-positivo (porta que o
    robô só passa do lado: o plano não a cruza)."""
    if not plan or len(plan) < 2:
        return False
    for i in range(len(plan) - 1):
        if _segments_cross(plan[i], plan[i + 1], a, b):
            return True
    return False


# ---- máquina de estados pura ------------------------------------------------

@dataclass
class DoorCrossConfig:
    zone_radius: float = 1.2        # m — distância do centro que arma a manobra
    approach_bearing: float = math.radians(70)  # porta tem que estar "na frente"
    # 2026-06-15: experimento "girar mais longe" (0.6 -> 1.0) REVERTIDO pra 0.6.
    # Com 1.0 o ponto de staging caía num ângulo que exigia GIRAR NO LUGAR pra
    # encarar (|err|>=60° -> vx=0). E giro no lugar fraco (~2.2 rad/s) o
    # skid-steer NÃO executa (patina; precisa ~6.0) -> robô travava "tentando
    # virar". Com 0.6 ele vai DIRIGINDO até o ponto (quebra o atrito andando) e
    # funciona — era o validado em campo 06-12 (atravessou a porta).
    stage_dist: float = 0.6         # m — ponto de preparação antes do centro
    stage_tol: float = 0.10         # m — chegou no staging
    stage_speed: float = 0.20       # m/s — aproximação (0.12->0.20 em 2026-06-16: a 0.12 patinava sem vencer o atrito estático; régua = ré do unstuck 0.25, validada em campo)
    stage_k_heading: float = 1.8    # ganho P do heading no staging
    align_lat: float = 0.08         # m — DEPRECATED: era o gate de "no eixo" (8cm,
    # apertado demais -> ping-pong staging<->rotating). O gate de "pronto pra
    # cruzar" virou o fit_lat geométrico (2026-06-17); não entra mais na decisão.
    align_yaw: float = math.radians(5.0)   # rad — |erro de yaw| máximo. APERTADO
    # de propósito: o corredor pós-porta é só um tiquinho mais largo que ela, então
    # a travessia tem que ser RETA. Quem deixa o robô reto é o point-turn do
    # rotating, NUNCA um arco dentro do vão. NÃO afrouxar.
    robot_half_width: float = 0.25  # m — meia-largura do robô (footprint 0.5) p/ fit_lat
    fit_margin: float = 0.05        # m — folga de segurança subtraída do vão no fit_lat
    cross_yaw_rate_max: float = 0.5  # rad/s — só ATRAVESSA quando o robô PAROU de
    # girar (taxa de yaw real entre ticks abaixo disto). Sem isto, no meio de um
    # point-turn rápido (rot_speed 4.0) um tick caía na banda de ±align_yaw e
    # ativava o crossing; a inércia angular levava o robô torto pra dentro do vão
    # -> batia no batente (2026-06-17). "Alinhou E parou, aí passa."
    align_stable: int = 2           # DEPRECATED (2026-06-17): a transição pro
    # crossing virou a checagem universal "passo reto daqui?" (todo tick, sem
    # esperar N ticks estáveis — era o "alinhou e voltou a caçar o meio"). Mantido
    # só pra não quebrar quem seta o param; não entra mais na decisão.
    # 2026-06-15: experimento 15 -> 600 REVERTIDO pra 15. O 600 não fazia o robô
    # "tentar mais" — transformava um STALL (ver stage_dist) num FREEZE de 10
    # min. O "não desistir do ponto" real era o timeout do MapBridge web (120 ->
    # 3600), já resolvido. Aqui 15s é a rede de segurança: se não alinhar,
    # aborta e devolve pro nav2 em vez de congelar.
    align_timeout: float = 15.0     # s — STAGING+ROTATING juntos
    rot_speed: float = 4.0          # rad/s — giro no lugar (point-turn forte; 3->4 em 2026-06-16, sobe a 6.0 ao vivo se patinar; NUNCA arco)
    # As rodas pegam pior girando pra ESQUERDA -> boost de FORÇA nesse lado
    # (mesmo 1.4 do unstuck spin, validado em campo). Com as rodas ruins na
    # DIAGONAL (RL+FR), esq/dir é simétrico, então este boost é só paridade com
    # o unstuck — live-tunable (zerar p/ simétrico se o campo pedir).
    rot_left_boost: float = 1.4
    cross_speed: float = 0.22       # m/s — travessia (0.15->0.22 em 2026-06-16: vencer o atrito estático sem patinar)
    cross_k_lat: float = 1.5        # corrige offset lateral durante a travessia
    cross_k_yaw: float = 2.0        # corrige heading durante a travessia
    cross_wz_max: float = 0.8       # rad/s — teto da micro-correção (NÃO girar)
    gap_min: float = 0.45           # m — vão mínimo à frente pra seguir
    exit_margin: float = 0.5        # m — além do centro pra soltar
    total_timeout: float = 40.0     # s — manobra inteira (revertido de 600; ver align_timeout)
    retrigger_cooldown: float = 3.0  # s — após abort, não rearmar na hora
    success_cooldown: float = 2.0   # s — após ATRAVESSAR limpo, não rearmar (cobre
    # o /plan defasado ~1Hz que ainda mostra a rota velha cruzando a porta -> sem
    # isso o robô re-armava, invertia o `side` e tentava voltar pra porta)
    # Ré de ESCAPE (2026-06-16): sem a ré do unstuck (calado na região da
    # porta), o door_crossing precisa se reajustar sozinho — senão fica
    # morto-preso de nariz na parede (e stalla o motor -> desarma o BMS, já que
    # door_vel fura o collision). Ré RETA (NUNCA arco), gated pelo vão traseiro.
    escape_front_gap: float = 0.20      # m — obstáculo a menos disso à frente -> ré (anti-stall)
    escape_substuck_time: float = 5.0   # s — alinhando sem chegar ao crossing -> ré
    escape_reverse_dist: float = 0.30   # m — quanto recua por escape (teto)
    escape_reverse_speed: float = 0.25  # m/s — ré de escape (0.12->0.25 em 2026-06-16: = ré do unstuck, validada vencendo o atrito em campo)
    escape_max_count: int = 3           # nº de escapes por travessia antes de abortar
    escape_rear_margin: float = 0.10    # m — folga: nunca chega a menos disso do obstáculo atrás (cap da distância de ré)
    escape_rear_min: float = 0.10       # m — vão traseiro útil MÍNIMO; abaixo disso nem vale a pena dar ré -> aborta
    align_progress_radius: float = 0.05  # m — moveu menos que isso desde a âncora = "parado" -> conta o substuck


class Cmd(NamedTuple):
    # estados que SAEM do update(): idle | staging | rotating | crossing.
    # (o /door_zone publica ainda 'approaching', injetado pelo nó na zona da
    # porta antes de assumir — NÃO é um estado do update().)
    state: str
    vx: float
    wz: float
    door_id: Optional[int]


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class DoorCrossing:
    """Decisão pura da travessia. O nó alimenta com pose do TF, portas,
    status do goal, gap e freshness; recebe (estado, vx, wz)."""

    def __init__(self, cfg: DoorCrossConfig):
        self.cfg = cfg
        self.state = 'idle'
        self.door = None          # dict da porta ativa
        self.geom: Optional[DoorGeom] = None
        self.side = 0             # +1/-1 — de que lado o robô aproximou
        self.t_start = 0.0
        self._last_yaw = None     # yaw do tick anterior (p/ medir a taxa de giro)
        self._last_now = 0.0
        self._cooldown_until = 0.0
        self._escape_count = 0          # rés de escape NESTA travessia
        self._rot_dir = 0               # sentido do giro do episódio atual (+1 esq/-1 dir/0 livre)
        self._align_t0 = 0.0            # início do "tentando alinhar" (sub-timeout)
        self._align_anchor = (0.0, 0.0)  # posição de referência do substuck
        self._esc_start = (0.0, 0.0)    # pose (x,y) no começo da ré atual
        self._esc_target = 0.0          # quanto recuar nesta ré

    # -- helpers ------------------------------------------------------------
    def _abort(self, now: float) -> Cmd:
        self.state = 'idle'
        self.door = None
        self.geom = None
        self._rot_dir = 0
        self._cooldown_until = now + self.cfg.retrigger_cooldown
        return Cmd('idle', 0.0, 0.0, None)

    def _maybe_escape(self, now, pos, front_gap, rear_gap, allow_substuck=True):
        """Decide se entra na ré de escape (ou aborta). Retorna um Cmd se a ré
        toma conta agora, ou None pra seguir o staging/rotating normal.

        Dispara quando: obstáculo perto na FRENTE (anti-stall/anti-BMS) OU não
        alinhou dentro de escape_substuck_time. Recua RETO (nunca arco), no
        máximo (rear_gap - escape_rear_margin), limitado a escape_reverse_dist.
        Sem vão atrás útil, ou estourado o escape_max_count -> ABORTA (larga pro
        nav2/unstuck como último recurso).

        allow_substuck=False (no ROTATING): girar parado pra alinhar NÃO é estar
        travado, então o gatilho por TEMPO não vale ali — só o obstáculo à frente.
        Era o que fazia a ré reta disparar no meio do giro com a traseira pra porta
        e parecer que o robô "ia atravessar de ré" (2026-06-16). O align_timeout
        (15 s) segue como rede de segurança pra 'girando sem nunca alinhar'."""
        cfg = self.cfg
        # progresso: se o robô se deslocou, reseta o relógio do substuck — só
        # conta "parado de verdade" (mesma ideia da âncora do unstuck). Assim
        # uma aproximação LEGÍTIMA (andando devagar) não dispara a ré de escape.
        if math.hypot(pos[0] - self._align_anchor[0],
                      pos[1] - self._align_anchor[1]) > cfg.align_progress_radius:
            self._align_anchor = pos
            self._align_t0 = now
        front_block = front_gap < cfg.escape_front_gap
        substuck = allow_substuck and (now - self._align_t0 > cfg.escape_substuck_time)
        need = front_block or substuck
        if not need:
            return None
        if self._escape_count >= cfg.escape_max_count:
            return self._abort(now)
        target = min(cfg.escape_reverse_dist, rear_gap - cfg.escape_rear_margin)
        if target < cfg.escape_rear_min:
            return self._abort(now)     # sem vão atrás -> não força contra a parede
        self._escape_count += 1
        self.state = 'reversing'
        self._esc_start = pos
        self._esc_target = target
        return Cmd('reversing', -cfg.escape_reverse_speed, 0.0, self.door['id'])

    def _pick_door(self, pose, doors, plan=None):
        """Escolhe a porta a cruzar entre as marcadas na zona. Critério primário
        = o /plan ATRAVESSA a porta (desacopla o arming do heading -> assume
        antes da curva do Nav2). Sem /plan disponível, cai no bearing antigo
        (compat). Empate -> a mais próxima."""
        x, y, yaw = pose
        best, best_dist = None, None
        for d in doors:
            g = door_geometry(tuple(d['a']), tuple(d['b']))
            dist = math.hypot(x - g.cx, y - g.cy)
            if dist > self.cfg.zone_radius:
                continue
            if plan:
                # tem rota: só arma se ela cruza ESTA porta (sem falso-positivo)
                if not plan_crosses_door(plan, tuple(d['a']), tuple(d['b'])):
                    continue
            else:
                # sem rota (/plan não chegou): "na frente" = QUALQUER parte do
                # vão dentro do cone (centro ou batente).
                bearing = min(
                    abs(_wrap(math.atan2(py - y, px - x) - yaw))
                    for px, py in ((g.cx, g.cy), tuple(d['a']), tuple(d['b'])))
                if bearing > self.cfg.approach_bearing:
                    continue
            if best_dist is None or dist < best_dist:
                best, best_dist = (d, g), dist
        return best if best is not None else (None, None)

    # -- tick -----------------------------------------------------------------
    def update(self, now, pose, doors, goal_active, nav_forward, gap,
               scan_fresh, front_gap=math.inf, rear_gap=math.inf,
               plan=None) -> Cmd:
        cfg = self.cfg

        if self.state == 'idle':
            if (pose is None or not goal_active or not nav_forward
                    or now < self._cooldown_until or not doors):
                return Cmd('idle', 0.0, 0.0, None)
            door, geom = self._pick_door(pose, doors, plan)
            if door is None:
                return Cmd('idle', 0.0, 0.0, None)
            x, y, _ = pose
            # lado de aproximação: progresso negativo = "antes" da porta
            raw_s = ((x - geom.cx) * geom.nx + (y - geom.cy) * geom.ny)
            self.side = -1 if raw_s > 0 else +1
            self.door, self.geom = door, geom
            self.state = 'staging'
            self.t_start = now
            self._escape_count = 0
            self._align_t0 = now
            self._align_anchor = (x, y)
            # cai no fluxo já neste tick; a checagem universal "passo reto daqui?"
            # (logo abaixo) atravessa NA HORA se já estiver alinhado — inclusive
            # já no tick de armar (era o "veio reto, passa").

        # guardas comuns a qualquer estado ativo
        if pose is None or not goal_active or not scan_fresh:
            return self._abort(now)
        if now - self.t_start > cfg.total_timeout:
            return self._abort(now)

        x, y, yaw = pose
        g = self.geom
        s, d = door_progress_lateral(g, x, y, self.side)
        yaw_des = crossing_yaw(g, self.side)
        yaw_err = _wrap(yaw - yaw_des)

        # taxa de giro REAL (entre ticks): saber se o robô PAROU de girar antes de
        # commitar a travessia (a inércia faz "mandar parar" != "estar parado").
        # Sem histórico válido (1º tick, ou gap grande do idle) -> inf = "girando"
        # (não cruza ainda; assenta no tick seguinte). dt limitado p/ não usar uma
        # referência velha do maneuver anterior.
        dt = now - self._last_now
        if self._last_yaw is not None and 0.0 < dt <= 0.25:
            yaw_rate = abs(_wrap(yaw - self._last_yaw)) / dt
        else:
            yaw_rate = math.inf
        self._last_yaw, self._last_now = yaw, now

        if self.state in ('staging', 'rotating'):
            if now - self.t_start > cfg.align_timeout:
                return self._abort(now)
            # SEGURANÇA (2026-06-17): obstáculo (não-batente) no vão à frente
            # DURANTE a aproximação -> larga pro nav2, que passa pelo collision
            # monitor e freia. Como door_vel fura o collision, a aproximação
            # precisa da própria checagem. Usa o `gap` COM máscara de batente (a
            # porta não dispara) e gap_min (0.45) < stage_dist (0.6+) -> a parede
            # da porta fica SEMPRE além do limiar, então só intruso de verdade
            # aborta (sem falso-positivo na parede). A travessia (crossing), que
            # fura o collision no vão estreito, já tinha essa checagem.
            if gap < cfg.gap_min:
                return self._abort(now)
            # A TODO MOMENTO: "passo reto daqui?" ATRAVESSA quando: já está RETO
            # (yaw apertado) E CABE pelo vão (fit_lat geométrico) E PAROU de girar
            # (taxa de yaw baixa). Roda em qualquer fase (staging OU rotating). É O
            # OBJETIVO — sem isto o robô alinhava e voltava a CAÇAR o meio em vez de
            # passar. A trava de taxa evita commitar no meio do giro rápido (inércia
            # -> entrava torto -> batia). O vão já foi garantido (gap>=gap_min).
            fit = fit_lat(g, cfg.robot_half_width, cfg.fit_margin)
            if (abs(yaw_err) <= cfg.align_yaw and abs(d) <= fit
                    and yaw_rate <= cfg.cross_yaw_rate_max):
                self.state = 'crossing'
                return Cmd('crossing', cfg.cross_speed, 0.0, self.door['id'])

        if self.state == 'staging':
            esc = self._maybe_escape(now, (x, y), front_gap, rear_gap)
            if esc is not None:
                return esc
            if abs(d) <= fit:
                # JÁ NO EIXO: não persegue o ponto exato de staging — vai alinhar
                # NO LUGAR (rotating). Era o "fica se enrolando indo pro eixo
                # sendo que já está no meio" (2026-06-17). Cai no rotating abaixo.
                self.state = 'rotating'
                self._rot_dir = 0          # episódio de giro novo
            else:
                # FORA do eixo: dirige PRO eixo (mira o ponto de staging, no eixo
                # a stage_dist antes do centro). Quando |d| entrar no fit (acima)
                # -> vira rotating, em qualquer distância.
                tgx = g.cx - g.nx * self.side * cfg.stage_dist
                tgy = g.cy - g.ny * self.side * cfg.stage_dist
                head = math.atan2(tgy - y, tgx - x)
                err = _wrap(head - yaw)
                wz = max(-cfg.rot_speed, min(cfg.rot_speed,
                                             cfg.stage_k_heading * err))
                vx = cfg.stage_speed if abs(err) < math.pi / 3 else 0.0
                return Cmd('staging', vx, wz, self.door['id'])

        if self.state == 'rotating':
            esc = self._maybe_escape(now, (x, y), front_gap, rear_gap,
                                     allow_substuck=False)
            if esc is not None:
                return esc
            # Aqui já sabemos que NÃO dá pra passar reto (a checagem universal
            # acima não disparou). Se está genuinamente FORA do vão (não cabe ir
            # reto daqui), volta pro staging reaproximar do eixo. Com fit_lat
            # (folga real, não os 8cm fixos) isso só dispara quando precisa, não a
            # cada drift de giro = sem ping-pong "caçando o meio".
            if abs(d) > fit:
                self.state = 'staging'
                self._rot_dir = 0
                return Cmd('staging', 0.0, 0.0, self.door['id'])
            if abs(yaw_err) <= cfg.align_yaw:
                # JÁ está reto, só não cruzou ainda (a checagem universal exige
                # também a taxa de giro baixa). Não gira mais — comanda parar e
                # ASSENTA; quando a taxa cair, a universal acima atravessa. Sem
                # isto o robô daria mais um giro e perderia o alinhamento (era
                # parte do "girou demais e bateu").
                self._rot_dir = 0
                return Cmd('rotating', 0.0, 0.0, self.door['id'])
            # GIRO LIMPO (igual ao spin do unstuck): escolhe o lado UMA vez e
            # gira forte até CRUZAR o alvo, sem inverter a cada tick. O bang-bang
            # antigo (±rot_speed recalculado todo tick) oscilava em torno do
            # alvo -> cada vai-e-volta arrastava o lateral -> estourava o gate
            # -> ping-pong com o staging. O yaw aqui vem do TF (já fundido c/
            # IMU pelo pose_estimator), então é "malha fechada na IMU" de graça.
            want = -1 if yaw_err > 0 else 1     # lado que reduz o yaw_err
            if self._rot_dir == 0 or want == self._rot_dir:
                # ainda não cruzou o alvo -> segue no MESMO sentido
                self._rot_dir = want
                speed = cfg.rot_speed
                if want > 0:
                    speed *= cfg.rot_left_boost   # esquerda escorrega: + força
                return Cmd('rotating', 0.0, want * speed, self.door['id'])
            # o lado necessário INVERTEU => cruzou o alvo (overshoot < 1 tick).
            # PARA e assenta; re-avalia no próximo tick em vez de reverter
            # girando (é isto que mata o limit cycle do bang-bang).
            self._rot_dir = 0
            return Cmd('rotating', 0.0, 0.0, self.door['id'])

        if self.state == 'reversing':
            if rear_gap <= cfg.escape_rear_margin:
                # algo entrou atrás no meio da ré -> para e re-tenta o staging
                self.state = 'staging'
                self._align_t0 = now
                self._align_anchor = (x, y)
                return Cmd('staging', 0.0, 0.0, self.door['id'])
            travelled = math.hypot(x - self._esc_start[0], y - self._esc_start[1])
            if travelled >= self._esc_target:
                # recuou o suficiente -> re-tenta o alinhamento de um ponto melhor
                self.state = 'staging'
                self._align_t0 = now
                self._align_anchor = (x, y)
                return Cmd('staging', 0.0, 0.0, self.door['id'])
            return Cmd('reversing', -cfg.escape_reverse_speed, 0.0,
                       self.door['id'])

        if self.state == 'crossing':
            if gap < cfg.gap_min:
                return self._abort(now)
            if s > cfg.exit_margin:
                # atravessou: solta com success_cooldown (2026-06-17). NÃO é falha,
                # mas o /plan (~1Hz) ainda mostra por ~1s a rota velha cruzando a
                # porta -> sem cooldown o robô re-armava, invertia o `side` e
                # tentava voltar pra porta que já passou. O cooldown segura até o
                # plano atualizar e o robô sair de vez.
                self.state = 'idle'
                self.door = None
                self.geom = None
                self._cooldown_until = now + cfg.success_cooldown
                return Cmd('idle', 0.0, 0.0, None)
            wz = -cfg.cross_k_lat * d - cfg.cross_k_yaw * yaw_err
            wz = max(-cfg.cross_wz_max, min(cfg.cross_wz_max, wz))
            return Cmd('crossing', cfg.cross_speed, wz, self.door['id'])

        return Cmd('idle', 0.0, 0.0, None)


def main(args=None):  # pragma: no cover - cola de I/O, validar na bancada
    import json

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Path
    from rcl_interfaces.msg import SetParametersResult
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException

    from .utils import quat_to_yaw, spin_node
    from .unstuck_supervisor import front_min_gap, rear_min_gap

    ACTIVE = {1, 2, 3}  # ACCEPTED, EXECUTING, CANCELING (igual unstuck)

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    class DoorCrossingNode(Node):
        def __init__(self):
            super().__init__('door_crossing')
            g = {}
            for name, default in (
                # 2026-06-15: REVERTIDO pros valores de 06-12 (validados: o robô
                # atravessou a porta). O experimento stage_dist 1.0 + timeout 600
                # travava o robô girando fraco no lugar. Ver DoorCrossConfig.
                ('zone_radius', 1.2), ('stage_dist', 0.6),
                ('align_lat', 0.08), ('align_yaw_deg', 5.0),
                ('align_timeout', 15.0), ('rot_speed', 4.0),
                ('rot_left_boost', 1.4),
                ('cross_speed', 0.22), ('stage_speed', 0.20),
                ('escape_reverse_speed', 0.25), ('gap_min', 0.45),
                ('exit_margin', 0.5), ('rate_hz', 20.0),
                # 2026-06-17 (atravessar reto): folga geométrica + cooldown +
                # trava de taxa de giro (só cruza quando parou de girar)
                ('robot_half_width', 0.25), ('fit_margin', 0.05),
                ('success_cooldown', 2.0), ('cross_yaw_rate_max', 0.5),
                ('scan_stale', 0.6), ('nav_move_lin', 0.02),
                ('rear_tail_x', -0.25), ('rear_half_width', 0.30),
                ('front_head_x', 0.25), ('lidar_x', 0.0),
            ):
                self.declare_parameter(name, default)
                g[name] = self.get_parameter(name).value

            self.cfg = DoorCrossConfig(
                zone_radius=g['zone_radius'], stage_dist=g['stage_dist'],
                align_lat=g['align_lat'],
                align_yaw=math.radians(g['align_yaw_deg']),
                align_timeout=g['align_timeout'], rot_speed=g['rot_speed'],
                rot_left_boost=g['rot_left_boost'],
                cross_speed=g['cross_speed'], stage_speed=g['stage_speed'],
                escape_reverse_speed=g['escape_reverse_speed'],
                gap_min=g['gap_min'], exit_margin=g['exit_margin'],
                robot_half_width=g['robot_half_width'],
                fit_margin=g['fit_margin'],
                success_cooldown=g['success_cooldown'],
                cross_yaw_rate_max=g['cross_yaw_rate_max'])
            self.sup = DoorCrossing(self.cfg)
            self.scan_stale = g['scan_stale']
            self.nav_move_lin = g['nav_move_lin']
            self.rear_tail_x = g['rear_tail_x']
            self.rear_half_width = g['rear_half_width']
            self.front_head_x = g['front_head_x']
            # LiDAR no CENTRO (0.0) hoje; param (igual ao unstuck) p/ não ficar
            # hardcoded se o sensor sair do centro um dia.
            self.lidar_x = g['lidar_x']

            # Live-tuning: o nó lia os params SÓ no boot -> `ros2 param set` não
            # pegava no nó rodando (achado 2026-06-17 afinando a porta em campo).
            # O DoorCrossConfig é mutável e a máquina de estados guarda a MESMA
            # referência (self.sup.cfg is self.cfg) relendo cfg todo tick, então
            # o callback só muta os campos -> pega no tick seguinte, sem restart.
            self.add_on_set_parameters_callback(self._on_set_params)

            self.doors = []
            self._goal_active = {}
            self._nav_forward = False
            self._plan = []            # rota planejada [(x,y), ...] no frame map
            self._scan = None          # (ranges, angle_min, inc)
            self._scan_t = None
            self._last_zone = None     # dedup do /door_zone

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self.pub = self.create_publisher(Twist, 'door_vel', 10)
            self.pub_zone = self.create_publisher(String, 'door_zone', latched)

            self.create_subscription(String, 'doors', self._on_doors, latched)
            be = qos_profile_sensor_data
            self.create_subscription(LaserScan, 'scan', self._on_scan, be)
            self.create_subscription(Twist, 'nav_vel_raw', self._on_nav, 10)
            self.create_subscription(Path, 'plan', self._on_plan, 10)
            for topic in ('navigate_to_pose/_action/status',
                          'navigate_through_poses/_action/status'):
                self.create_subscription(
                    GoalStatusArray, topic,
                    lambda m, t=topic: self._on_status(t, m), 10)

            self.create_timer(1.0 / g['rate_hz'], self._tick)
            self._publish_zone('idle', None)
            self.get_logger().info(
                'door_crossing ativo: zona %.1fm, reto |yaw|<%.0f° + cabe pelo '
                'vão (robô %.2fm, margem %.2fm), atravessa %.2fm/s' % (
                    self.cfg.zone_radius, math.degrees(self.cfg.align_yaw),
                    2 * self.cfg.robot_half_width, self.cfg.fit_margin,
                    self.cfg.cross_speed))

        # campos do DoorCrossConfig afináveis ao vivo (mutados na MESMA ref que
        # a máquina de estados relê todo tick); rate_hz fica de fora (o timer é
        # criado no boot).
        _CFG_PARAMS = ('zone_radius', 'stage_dist', 'align_lat',
                       'align_timeout', 'rot_speed', 'rot_left_boost',
                       'cross_speed', 'stage_speed', 'escape_reverse_speed',
                       'gap_min', 'exit_margin', 'robot_half_width',
                       'fit_margin', 'success_cooldown', 'cross_yaw_rate_max')
        _NODE_PARAMS = ('scan_stale', 'nav_move_lin', 'rear_tail_x',
                        'rear_half_width', 'front_head_x', 'lidar_x')

        def _on_set_params(self, params):
            for p in params:
                if p.name == 'align_yaw_deg':
                    self.cfg.align_yaw = math.radians(p.value)
                elif p.name in self._CFG_PARAMS:
                    setattr(self.cfg, p.name, p.value)
                elif p.name in self._NODE_PARAMS:
                    setattr(self, p.name, p.value)
                elif p.name == 'rate_hz':
                    self.get_logger().warn(
                        'rate_hz só muda com restart do nó (timer fixo no boot)')
            return SetParametersResult(successful=True)

        def _on_doors(self, msg):
            try:
                self.doors = json.loads(msg.data).get('doors', [])
                self.get_logger().info(f'{len(self.doors)} porta(s) carregada(s)')
            except (ValueError, AttributeError) as e:
                self.get_logger().warn(f'/doors inválido: {e}')

        def _on_scan(self, msg):
            self._scan = (msg.ranges, msg.angle_min, msg.angle_increment)
            self._scan_t = time.monotonic()

        def _on_nav(self, msg):
            # 2026-06-16: "indo pra frente" -> "não está dando ré". Deixa o
            # door_crossing armado quando o nav quer GIRAR pra alinhar (linear≈0).
            self._nav_forward = nav_engaging(msg.linear.x, self.nav_move_lin)

        def _on_plan(self, msg):
            # rota global do Nav2 -> sinal de arming (atravessa a porta?).
            self._plan = [(p.pose.position.x, p.pose.position.y)
                          for p in msg.poses]

        def _on_status(self, topic, msg):
            self._goal_active[topic] = any(
                st.status in ACTIVE for st in msg.status_list)

        def _pose_map(self):
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time())
            except TransformException:
                return None
            t = tf.transform.translation
            q = tf.transform.rotation
            return (t.x, t.y, quat_to_yaw(q.x, q.y, q.z, q.w))

        def _publish_zone(self, state, door_id):
            payload = json.dumps({'state': state, 'door_id': door_id})
            if payload != self._last_zone:
                self._last_zone = payload
                self.pub_zone.publish(String(data=payload))

        def _tick(self):
            now = time.monotonic()
            pose = self._pose_map()
            goal = any(self._goal_active.values()) if self._goal_active else False
            fresh = (self._scan_t is not None
                     and now - self._scan_t <= self.scan_stale)
            gap = math.inf
            # gap COM máscara de batente: no crossing (vão estreito) e também no
            # staging/rotating (aborto-de-segurança da aproximação, 2026-06-17).
            if (fresh and pose is not None and self.sup.door is not None
                    and self.sup.state in ('staging', 'rotating', 'crossing')):
                ranges, amin, ainc = self._scan
                jambs = [tuple(self.sup.door['a']), tuple(self.sup.door['b'])]
                gap = gap_ahead(ranges, amin, ainc, pose, jambs, 0.30)

            front_gap = math.inf
            rear_gap = math.inf
            if fresh and self._scan is not None:
                ranges, amin, ainc = self._scan
                arr = np.asarray(ranges, dtype=np.float64)
                # LiDAR no centro (lidar_x=0); vão medido do para-choque. Sem
                # descontar batente de propósito (anti-stall: contato com a
                # parede/batente conta), diferente do gap_ahead do crossing.
                front_gap = front_min_gap(arr, amin, ainc, self.lidar_x,
                                          self.front_head_x, self.rear_half_width)
                rear_gap = rear_min_gap(arr, amin, ainc, self.lidar_x,
                                        self.rear_tail_x, self.rear_half_width)

            prev = self.sup.state
            cmd = self.sup.update(now, pose, self.doors, goal,
                                  self._nav_forward, gap, fresh,
                                  front_gap, rear_gap, plan=self._plan)
            if cmd.state != prev:
                self.get_logger().info(f'door_crossing: {prev} -> {cmd.state}')
            # /door_zone: a manobra ativa manda; senão, se há porta marcada na
            # zona com goal ativo, publica 'approaching' (gate do standdown do
            # unstuck). 'approaching' NÃO comanda door_vel — só sinaliza a região.
            if cmd.state != 'idle':
                self._publish_zone(cmd.state, cmd.door_id)
            else:
                nd = (nearest_door_in_zone(pose, self.doors, self.cfg.zone_radius)
                      if goal else None)
                if nd is not None:
                    self._publish_zone('approaching', nd['id'])
                else:
                    self._publish_zone('idle', None)
            if cmd.state != 'idle' or prev != 'idle':
                # Twist zero explícito na transição pra idle (mesma lição do
                # unstuck: cmd_vel_to_wheels segura o último comando).
                t = Twist()
                t.linear.x = cmd.vx
                t.angular.z = cmd.wz
                self.pub.publish(t)

    rclpy.init(args=args)
    node = DoorCrossingNode()
    try:
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':  # pragma: no cover
    main()
