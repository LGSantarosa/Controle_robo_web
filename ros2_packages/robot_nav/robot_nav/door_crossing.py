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


def pre_door_waypoint(g: DoorGeom, side: int, standoff: float):
    """Waypoint pré-porta: no eixo, recuado `standoff` do centro no lado de
    aproximação `side`, orientação = heading de travessia (de frente pra porta).
    É a POSIÇÃO que o nav2 entrega; o alinhamento fino fica com o door."""
    x = g.cx - g.nx * side * standoff
    y = g.cy - g.ny * side * standoff
    return x, y, crossing_yaw(g, side)


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


GAP_CORRIDOR_HALF_W = 0.30   # m — meia-largura do corredor vigiado (corpo 0.25 +5cm)
GAP_MAX_X = 1.0              # m — até onde olhar à frente (também o gap reportado no log)


def gap_ahead(ranges, angle_min: float, angle_increment: float,
              pose: Tuple[float, float, float],
              jambs: List[Tuple[float, float]], jamb_r: float,
              half_w: float = GAP_CORRIDOR_HALF_W,
              max_x: float = GAP_MAX_X) -> float:
    """Distância (m) do obstáculo mais próximo numa ZONA DE PARADA à FRENTE do
    robô (corredor de meia-largura `half_w`, até `max_x` à frente), descontando os
    discos dos batentes marcados (em frame do MAPA). inf = livre.

    É a checagem de segurança da travessia: como o `door_vel` PASSA POR CIMA do
    collision monitor (prio 20 no twist_mux), o collision NÃO protege contra uma
    pessoa durante o crossing — então o door_crossing precisa ser a própria
    autoridade. Os batentes que o usuário marcou não contam (são a parede da porta);
    qualquer outra coisa (pessoa!) conta -> o crossing PARA (não fura cego).
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
    sel = ok & (x > 0.0) & (x <= max_x) & (np.abs(y) <= half_w)
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
    # ARMA a manobra
    zone_radius: float = 1.2        # m — distância do centro que arma a manobra
    approach_bearing: float = math.radians(70)  # cone (só usado sem /plan)
    # WAYPOINT pré-porta (2026-06-18): nav2 leva o robô a W (no eixo, recuado,
    # centrado) e o door só alinha+cruza. Substitui a aproximação reativa
    # (staging/ré/escape) inteira — o erro principal era o door tentar se
    # posicionar na unha colado na porta.
    wp_standoff: float = 1.0        # m — distância de W antes do centro da porta
    wp_retries: int = 2             # re-tentativas de mandar W antes de desistir
    wp_timeout: float = 30.0        # s — tempo do nav2 chegar em W antes de re-tentar
    # ALINHA no lugar (point-turn) — longe da porta = seguro
    align_yaw: float = math.radians(5.0)   # rad — |erro de yaw| máximo p/ cruzar
    rot_speed: float = 4.0          # rad/s — giro no lugar (forte; sobe a 6.0 ao vivo se patinar; NUNCA arco)
    rot_left_boost: float = 1.4     # esquerda escorrega -> + força nesse lado
    rot_brake_angle: float = math.radians(12.0)  # rad — dentro disto, freia o giro
    rot_brake_speed: float = 2.0    # rad/s — velocidade do giro dentro do freio
    cross_yaw_rate_max: float = 0.5  # rad/s — só cruza quando PAROU de girar
    # CRUZA reto
    robot_half_width: float = 0.25  # m — meia-largura do robô (0.50 medido roda a roda)
    fit_margin: float = 0.13        # m — folga subtraída do vão no fit_lat. Porta real
    # 0.93m mas a marcada é 0.968m (1.9cm/lado a mais); 0.13 absorve isso + deixa
    # ~11cm de folga real -> só cruza com |lat|<~10cm (campo 06-18: raspou a 18cm).
    jamb_safety: float = 0.25       # m — perto dos batentes (s>-jamb_safety) e ainda
    # descentrado (|d|>fit) -> ABORTA e re-posiciona (re-manda W) em vez de raspar.
    cross_speed: float = 0.22       # m/s — travessia (vence o atrito estático sem patinar)
    cross_k_lat: float = 1.5        # corrige offset lateral durante a travessia
    cross_k_yaw: float = 2.0        # corrige heading durante a travessia
    cross_wz_max: float = 0.8       # rad/s — teto da micro-correção (NÃO girar)
    # SEGURANÇA da travessia ("caminho B", 2026-06-17): door_vel fura o collision
    # monitor (prio 20), então o door é a autoridade contra PESSOA no vão -> PARA.
    stop_zone_half_w: float = 0.30  # m — meia-largura da zona vigiada (corpo 0.25 +5cm)
    stop_look_ahead: float = 1.0    # m — alcance da vigia (também o gap do log)
    stop_dist: float = 0.6          # m — obstáculo mais perto que isto no crossing -> PARA
    stop_hold_timeout: float = 8.0  # s — parado esperando liberar; estourou -> aborta
    # SOLTA assim que passa dos batentes (re-manda G pro nav2 continuar)
    exit_margin: float = 0.30       # m — centro além do plano da porta p/ SOLTAR
    # rede de segurança / cooldowns
    total_timeout: float = 40.0     # s — manobra inteira (rotating+crossing)
    retrigger_cooldown: float = 3.0  # s — após abort, não rearmar na hora
    success_cooldown: float = 2.0   # s — após cruzar limpo, segura o /plan defasado


class Cmd(NamedTuple):
    # estados que SAEM do update(): idle | positioning | rotating | crossing.
    # (o /door_zone publica ainda 'approaching', injetado pelo nó na zona da
    # porta antes de assumir — NÃO é um estado do update().)
    state: str
    vx: float
    wz: float
    door_id: Optional[int]
    # pedido de navegação pro nó executar (cliente de action nav2):
    # None | ('goto', (x, y, yaw)) | ('cancel',). A máquina pura só EMITE; o nó
    # manda/cancela o goal. É como o door leva o robô ao waypoint pré-porta (W)
    # e re-manda o destino do usuário (G) depois de cruzar.
    nav: object = None


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _pose_changed(a, b, tol: float = 0.05) -> bool:
    """True se o destino mudou (posição > tol OU yaw > ~6°). Usado pra detectar
    que o usuário setou outro goal no meio da manobra."""
    return (abs(a[0] - b[0]) > tol or abs(a[1] - b[1]) > tol
            or abs(_wrap(a[2] - b[2])) > 0.1)


def yaw_to_quat(yaw: float):
    """(x, y, z, w) de um yaw puro (rotação só em Z) — p/ montar o goal do nav2."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


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
        # diagnóstico (lido pelo nó p/ log de campo): erro de yaw, offset lateral
        # e taxa de giro do último tick ativo.
        self.dbg_yaw_err = 0.0
        self.dbg_d = 0.0
        self.dbg_yaw_rate = 0.0
        self.dbg_s = 0.0          # progresso ao longo do eixo (s>exit_margin solta)
        self._hold_t0 = None      # início do stop-hold (pessoa no caminho) no crossing
        self._cooldown_until = 0.0
        self._rot_dir = 0               # sentido do giro do episódio atual (+1 esq/-1 dir/0 livre)
        self._goal_g = None             # destino do usuário (x,y,yaw) capturado p/ re-mandar
        self._wp_t0 = 0.0               # quando mandou o W atual (timeout)
        self._wp_tries = 0              # re-tentativas de W nesta manobra

    # -- helpers ------------------------------------------------------------
    def _abort(self, now: float) -> Cmd:
        self.state = 'idle'
        self.door = None
        self.geom = None
        self._rot_dir = 0
        self._hold_t0 = None
        self._cooldown_until = now + self.cfg.retrigger_cooldown
        return Cmd('idle', 0.0, 0.0, None)

    def _abort_to_idle(self, now: float, nav=None) -> Cmd:
        """Volta pra idle com retrigger_cooldown (falha/cancelamento). `nav` deixa
        cancelar o goal W pendente no nó (nav=('cancel',))."""
        self.state = 'idle'
        self.door = None
        self.geom = None
        self._goal_g = None
        self._rot_dir = 0
        self._hold_t0 = None
        self._cooldown_until = now + self.cfg.retrigger_cooldown
        return Cmd('idle', 0.0, 0.0, None, nav=nav)

    def _to_idle_success(self, now: float) -> None:
        """Travessia OK: idle com success_cooldown (não re-arma no plano defasado)."""
        self.state = 'idle'
        self.door = None
        self.geom = None
        self._goal_g = None
        self._rot_dir = 0
        self._hold_t0 = None
        self._cooldown_until = now + self.cfg.success_cooldown

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
               goal_g=None, wp_status='idle', plan=None) -> Cmd:
        cfg = self.cfg

        if self.state == 'idle':
            if (pose is None or not goal_active or not nav_forward
                    or now < self._cooldown_until or not doors
                    or goal_g is None):
                return Cmd('idle', 0.0, 0.0, None)
            door, geom = self._pick_door(pose, doors, plan)
            if door is None:
                return Cmd('idle', 0.0, 0.0, None)
            x, y, _ = pose
            # lado de aproximação: progresso negativo = "antes" da porta
            raw_s = ((x - geom.cx) * geom.nx + (y - geom.cy) * geom.ny)
            self.side = -1 if raw_s > 0 else +1
            self.door, self.geom = door, geom
            self._goal_g = goal_g          # destino do usuário (re-mandado ao cruzar)
            self.t_start = now
            self._wp_tries = 0
            self._wp_t0 = now
            # POSICIONAR via nav2: manda o robô pro waypoint pré-porta W (no eixo,
            # recuado, de frente). O door fica em positioning (mãos quietas) até o
            # nav2 entregar; aí assume pro alinhar+cruzar de um ponto seguro.
            self.state = 'positioning'
            W = pre_door_waypoint(geom, self.side, cfg.wp_standoff)
            return Cmd('positioning', 0.0, 0.0, door['id'], nav=('goto', W))

        if self.state == 'positioning':
            # mãos quietas: o nav2 dirige o robô até W. O door só espera o
            # RESULTADO do goal W e decide. NÃO precisa de scan aqui.
            if pose is None or not goal_active:
                return self._abort_to_idle(now, nav=('cancel',))
            if (goal_g is not None and self._goal_g is not None
                    and _pose_changed(goal_g, self._goal_g)):
                # usuário setou outro destino -> cancela W e re-avalia do idle
                return self._abort_to_idle(now, nav=('cancel',))
            if wp_status == 'succeeded':
                # nav2 entregou o robô em W -> assume pro alinhar+cruzar
                self.state = 'rotating'
                self._rot_dir = 0
                self.t_start = now            # reinicia o relógio (timeouts do giro)
                return Cmd('rotating', 0.0, 0.0, self.door['id'])
            if wp_status == 'aborted' or (now - self._wp_t0) > cfg.wp_timeout:
                self._wp_tries += 1
                if self._wp_tries > cfg.wp_retries:
                    return self._abort_to_idle(now)   # desiste; nó publica 'failed'
                self._wp_t0 = now
                W = pre_door_waypoint(self.geom, self.side, cfg.wp_standoff)
                return Cmd('positioning', 0.0, 0.0, self.door['id'],
                           nav=('goto', W))
            return Cmd('positioning', 0.0, 0.0, self.door['id'])  # esperando nav2

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
        self.dbg_yaw_err, self.dbg_d, self.dbg_yaw_rate = yaw_err, d, yaw_rate
        self.dbg_s = s

        if self.state == 'rotating':
            # ALINHADO e PAROU de girar -> ATRAVESSA. NÃO exige fit: o robô está em
            # W, ainda pode estar lateralmente fora da tolerância do nav2; o
            # crossing corrige o lateral ANDANDO (Task 5). Exigir fit aqui prenderia
            # o robô girando, já que o point-turn (vx=0) não reduz o lateral.
            # 2026-06-18.
            if abs(yaw_err) <= cfg.align_yaw and yaw_rate <= cfg.cross_yaw_rate_max:
                self.state = 'crossing'
                self._hold_t0 = None
                return Cmd('crossing', cfg.cross_speed, 0.0, self.door['id'])
            if abs(yaw_err) <= cfg.align_yaw:
                # reto mas ainda girando (taxa alta) -> para e ASSENTA; cruza no
                # tick seguinte quando a taxa cair. Sem isto o robô daria mais um
                # giro e perderia o alinhamento (era parte do "girou demais e bateu").
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
                if abs(yaw_err) > cfg.rot_brake_angle:
                    # LONGE do alvo: velocidade cheia (quebra o atrito)
                    speed = cfg.rot_speed
                    if want > 0:
                        speed *= cfg.rot_left_boost   # esquerda escorrega: + força
                else:
                    # PERTO do alvo: freia pra ENCAIXAR na banda sem passar direto
                    # (mata a oscilação esq/dir). Já está girando -> não stalla.
                    speed = cfg.rot_brake_speed
                return Cmd('rotating', 0.0, want * speed, self.door['id'])
            # o lado necessário INVERTEU => cruzou o alvo (overshoot < 1 tick).
            # PARA e assenta; re-avalia no próximo tick em vez de reverter
            # girando (é isto que mata o limit cycle do bang-bang).
            self._rot_dir = 0
            return Cmd('rotating', 0.0, 0.0, self.door['id'])

        if self.state == 'crossing':
            # SAÍDA PRIMEIRO (06-18): passou dos batentes (s além do plano) -> o door
            # cumpriu o papel, SOLTA pro nav2, não importa o que tem à frente. Isto
            # TEM que vir antes do gap-stop: senão, com o robô JÁ do outro lado mas
            # uma parede a <stop_dist à frente (corredor de saída), o gap-stop
            # congelava o robô atravessado e só largava pelo timeout de 8s (campo
            # 06-18: "atravessou e ficou parado e travado"). O gap-stop (pessoa) só
            # faz sentido ENQUANTO ainda está no vão.
            if s > cfg.exit_margin:
                # PASSOU DOS BATENTES -> o door cumpriu o papel. SOLTA e RE-MANDA o
                # destino do usuário (G) pro nav2 continuar o trajeto. success_cooldown
                # segura o re-arme no /plan defasado (~1Hz mostra a rota velha por ~1s).
                g_dest = self._goal_g
                self._to_idle_success(now)
                return Cmd('idle', 0.0, 0.0, None, nav=('goto', g_dest))
            if gap < cfg.stop_dist:
                # SEGURANÇA (caminho B, 2026-06-17): obstáculo não-batente (PESSOA)
                # na zona de parada à frente, AINDA dentro do vão. O door_vel fura o
                # collision monitor, então o door É a autoridade aqui: PARA (vx=0) e
                # segura — NÃO fura cego pra cima dela. Resume sozinho quando liberar;
                # se persistir mais que stop_hold_timeout, larga pro nav2 (replana).
                if self._hold_t0 is None:
                    self._hold_t0 = now
                elif now - self._hold_t0 > cfg.stop_hold_timeout:
                    return self._abort(now)
                return Cmd('crossing', 0.0, 0.0, self.door['id'])
            self._hold_t0 = None        # caminho livre -> reseta o relógio do hold
            # TRAVA DE SEGURANÇA (2026-06-18): chegou perto dos batentes ainda
            # descentrado (|d|>fit) -> NÃO fura/raspa: volta pra positioning e
            # re-manda W (re-posiciona). Teve ~1m aberto de W até aqui pra convergir
            # o lateral; se não convergiu, é re-posicionar, não raspar a roda.
            fit = fit_lat(g, cfg.robot_half_width, cfg.fit_margin)
            if s > -cfg.jamb_safety and abs(d) > fit:
                self.state = 'positioning'
                self._wp_tries = 0
                self._wp_t0 = now
                W = pre_door_waypoint(g, self.side, cfg.wp_standoff)
                return Cmd('positioning', 0.0, 0.0, self.door['id'],
                           nav=('goto', W))
            wz = -cfg.cross_k_lat * d - cfg.cross_k_yaw * yaw_err
            wz = max(-cfg.cross_wz_max, min(cfg.cross_wz_max, wz))
            return Cmd('crossing', cfg.cross_speed, wz, self.door['id'])

        return Cmd('idle', 0.0, 0.0, None)


def main(args=None):  # pragma: no cover - cola de I/O, validar na bancada
    import json

    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Path
    from nav2_msgs.action import NavigateToPose
    from rcl_interfaces.msg import SetParametersResult
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException

    from .utils import quat_to_yaw, spin_node

    ACTIVE = {1, 2, 3}  # ACCEPTED, EXECUTING, CANCELING (igual unstuck)

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    class DoorCrossingNode(Node):
        def __init__(self):
            super().__init__('door_crossing')
            g = {}
            for name, default in (
                ('zone_radius', 1.2), ('align_yaw_deg', 5.0),
                ('rot_speed', 4.0), ('rot_left_boost', 1.4),
                ('rot_brake_deg', 12.0), ('rot_brake_speed', 2.0),
                ('cross_yaw_rate_max', 0.5),
                # WAYPOINT pré-porta (2026-06-18): nav2 posiciona, door cruza
                ('wp_standoff', 1.0), ('wp_retries', 2), ('wp_timeout', 30.0),
                ('cross_speed', 0.22), ('exit_margin', 0.30), ('rate_hz', 20.0),
                ('robot_half_width', 0.25), ('fit_margin', 0.13),
                ('jamb_safety', 0.25), ('success_cooldown', 2.0),
                # caminho B: zona de parada da travessia (door é a autoridade)
                ('stop_zone_half_w', 0.30), ('stop_look_ahead', 1.0),
                ('stop_dist', 0.6), ('stop_hold_timeout', 8.0),
                ('scan_stale', 0.6), ('nav_move_lin', 0.02),
                ('rear_tail_x', -0.25), ('rear_half_width', 0.30),
                ('front_head_x', 0.25), ('lidar_x', 0.0),
            ):
                self.declare_parameter(name, default)
                g[name] = self.get_parameter(name).value

            self.cfg = DoorCrossConfig(
                zone_radius=g['zone_radius'],
                align_yaw=math.radians(g['align_yaw_deg']),
                rot_speed=g['rot_speed'], rot_left_boost=g['rot_left_boost'],
                rot_brake_angle=math.radians(g['rot_brake_deg']),
                rot_brake_speed=g['rot_brake_speed'],
                cross_yaw_rate_max=g['cross_yaw_rate_max'],
                wp_standoff=g['wp_standoff'], wp_retries=g['wp_retries'],
                wp_timeout=g['wp_timeout'],
                cross_speed=g['cross_speed'], exit_margin=g['exit_margin'],
                robot_half_width=g['robot_half_width'],
                fit_margin=g['fit_margin'], jamb_safety=g['jamb_safety'],
                success_cooldown=g['success_cooldown'],
                stop_zone_half_w=g['stop_zone_half_w'],
                stop_look_ahead=g['stop_look_ahead'],
                stop_dist=g['stop_dist'],
                stop_hold_timeout=g['stop_hold_timeout'])
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
            # cliente de action nav2 (posicionar via W; re-mandar G ao cruzar)
            self._goal_g = None        # destino do usuário (x,y,yaw) de /goal_pose
            self._plan_goal = None     # destino = fim do /plan (fonte robusta de G)
            self._wp_status = 'idle'   # status do goal W: idle|active|succeeded|aborted
            self._wp_handle = None     # handle do goal W em voo (p/ cancelar)
            self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self.pub = self.create_publisher(Twist, 'door_vel', 10)
            self.pub_zone = self.create_publisher(String, 'door_zone', latched)

            self.create_subscription(String, 'doors', self._on_doors, latched)
            be = qos_profile_sensor_data
            self.create_subscription(LaserScan, 'scan', self._on_scan, be)
            self.create_subscription(Twist, 'nav_vel_raw', self._on_nav, 10)
            self.create_subscription(Path, 'plan', self._on_plan, 10)
            self.create_subscription(PoseStamped, 'goal_pose', self._on_goal_pose, 10)
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
        _CFG_PARAMS = ('zone_radius', 'rot_speed', 'rot_left_boost',
                       'rot_brake_speed', 'cross_yaw_rate_max',
                       'wp_standoff', 'wp_retries', 'wp_timeout',
                       'cross_speed', 'exit_margin', 'robot_half_width',
                       'fit_margin', 'jamb_safety', 'success_cooldown',
                       'stop_zone_half_w', 'stop_look_ahead',
                       'stop_dist', 'stop_hold_timeout')
        _NODE_PARAMS = ('scan_stale', 'nav_move_lin', 'rear_tail_x',
                        'rear_half_width', 'front_head_x', 'lidar_x')

        def _on_set_params(self, params):
            for p in params:
                if p.name == 'align_yaw_deg':
                    self.cfg.align_yaw = math.radians(p.value)
                elif p.name == 'rot_brake_deg':
                    self.cfg.rot_brake_angle = math.radians(p.value)
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
            # fim do plano = destino G. Fonte ROBUSTA do destino (o /goal_pose é
            # VOLATILE de um disparo só -> o door perde se assinar depois). Usado
            # como goal_g no arming quando o /goal_pose não chegou.
            if msg.poses:
                gp = msg.poses[-1].pose
                q = gp.orientation
                self._plan_goal = (gp.position.x, gp.position.y,
                                   quat_to_yaw(q.x, q.y, q.z, q.w))

        def _on_status(self, topic, msg):
            self._goal_active[topic] = any(
                st.status in ACTIVE for st in msg.status_list)

        # -- cliente de action nav2 (posicionar via W; re-mandar G) ----------
        def _on_goal_pose(self, msg):
            # destino do usuário (G). Capturado p/ re-mandar depois de cruzar.
            q = msg.pose.orientation
            self._goal_g = (msg.pose.position.x, msg.pose.position.y,
                            quat_to_yaw(q.x, q.y, q.z, q.w))

        def _send_nav_goal(self, pose, track=True):
            # track=True (W): rastreia o resultado em self._wp_status. track=False
            # (G, na saída): fire-and-forget — NÃO toca _wp_status (senão o
            # resultado de G poluiria o tracking do W na próxima travessia).
            if not self._nav_client.wait_for_server(timeout_sec=0.0):
                self.get_logger().warn('navigate_to_pose indisponível')
                if track:
                    self._wp_status = 'aborted'
                return
            x, y, yaw = pose
            g = NavigateToPose.Goal()
            g.pose.header.frame_id = 'map'
            g.pose.pose.position.x = x
            g.pose.pose.position.y = y
            qx, qy, qz, qw = yaw_to_quat(yaw)
            g.pose.pose.orientation.x = qx
            g.pose.pose.orientation.y = qy
            g.pose.pose.orientation.z = qz
            g.pose.pose.orientation.w = qw
            fut = self._nav_client.send_goal_async(g)
            if track:
                self._wp_status = 'active'
                fut.add_done_callback(self._on_wp_accepted)

        def _on_wp_accepted(self, fut):
            h = fut.result()
            if not h.accepted:
                self._wp_status = 'aborted'
                return
            self._wp_handle = h
            h.get_result_async().add_done_callback(self._on_wp_result)

        def _on_wp_result(self, fut):
            # status 4 = SUCCEEDED (action_msgs/GoalStatus.STATUS_SUCCEEDED)
            self._wp_status = ('succeeded'
                               if fut.result().status == 4 else 'aborted')

        def _cancel_nav_goal(self):
            if self._wp_handle is not None:
                self._wp_handle.cancel_goal_async()
                self._wp_handle = None
            self._wp_status = 'idle'

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
            # gap COM máscara de batente: a zona de parada (caminho B) da travessia.
            if (fresh and pose is not None and self.sup.door is not None
                    and self.sup.state in ('rotating', 'crossing')):
                ranges, amin, ainc = self._scan
                jambs = [tuple(self.sup.door['a']), tuple(self.sup.door['b'])]
                gap = gap_ahead(ranges, amin, ainc, pose, jambs, 0.30,
                                half_w=self.cfg.stop_zone_half_w,
                                max_x=self.cfg.stop_look_ahead)

            prev = self.sup.state
            # destino G: prefere o /goal_pose (estável); senão o fim do /plan. O
            # fallback do plano SÓ enquanto idle — em positioning+ o /plan vira a
            # rota pro W, então usar o fim dele contaminaria (pareceria "novo goal").
            if prev == 'idle' and self._goal_g is None:
                goal_g = self._plan_goal
            else:
                goal_g = self._goal_g
            cmd = self.sup.update(now, pose, self.doors, goal,
                                  self._nav_forward, gap, fresh,
                                  goal_g=goal_g, wp_status=self._wp_status,
                                  plan=self._plan)
            # executa o pedido de navegação da máquina (cliente de action nav2)
            if cmd.nav is not None:
                if cmd.nav[0] == 'goto':
                    # W (em positioning) rastreia o resultado; G (na saída) não.
                    self._send_nav_goal(cmd.nav[1],
                                        track=(cmd.state == 'positioning'))
                elif cmd.nav[0] == 'cancel':
                    self._cancel_nav_goal()
            # fora do positioning não esperamos mais o W -> status limpo (o
            # 'succeeded' que levou pro rotating já foi consumido pela transição).
            if cmd.state in ('idle', 'rotating', 'crossing'):
                self._wp_status = 'idle'
            # diagnóstico de campo: s/yaw/lat/taxa por tick (transição + ~2 Hz).
            dbg = ('s=%+.2f yaw_err=%+.1f° lat=%+.0fcm taxa=%.1f vx=%+.2f wz=%+.2f '
                   'gap=%.2f' % (
                       self.sup.dbg_s, math.degrees(self.sup.dbg_yaw_err),
                       self.sup.dbg_d * 100, self.sup.dbg_yaw_rate, cmd.vx, cmd.wz,
                       gap))
            if cmd.state != prev:
                self.get_logger().info(
                    f'door_crossing: {prev} -> {cmd.state} | {dbg}')
            elif cmd.state in ('positioning', 'rotating', 'crossing'):
                self._dbg_tick = getattr(self, '_dbg_tick', 0) + 1
                if self._dbg_tick % 10 == 0:        # ~2 Hz a 20 Hz de loop
                    self.get_logger().info(f'door_crossing[{cmd.state}] | {dbg}')
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
            # door_vel SÓ nos estados que DIRIGEM (rotating/crossing). Em
            # positioning/idle o nó NÃO publica -> o twist_mux (door_vel prio 20)
            # NÃO segura o nav2 (que é quem dirige até W). Publica um zero só ao
            # SAIR de rotating/crossing (solta o último comando, lição do unstuck).
            ACTIVE_DRV = ('rotating', 'crossing')
            if cmd.state in ACTIVE_DRV or prev in ACTIVE_DRV:
                t = Twist()
                t.linear.x = cmd.vx
                t.angular.z = cmd.wz
                self.pub.publish(t)
            # desistiu de posicionar (estourou os retries do W) -> avisa
            if prev == 'positioning' and cmd.state == 'idle' and cmd.nav is None:
                self.get_logger().warn(
                    'door_crossing: nav2 não chegou em W (retries esgotados) -> '
                    'larga pro controle manual')

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
