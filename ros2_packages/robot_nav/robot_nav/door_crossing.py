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
    fit_margin: float = 0.13        # m — folga de segurança subtraída do vão no fit_lat.
    # 2026-06-18 (0.05 -> 0.13): porta REAL 0.93m (meia 0.465) mas a MARCADA é
    # 0.968m (meia 0.484, 1.9cm/lado a mais) e robô é 0.50m (meia 0.25, MEDIDO roda
    # a roda — correto). Com margem 0.05 o fit deixava commitar a |lat|=18cm -> folga
    # real ~3cm -> raspou a roda no batente (campo 06-18). 0.13 absorve os 1.9cm da
    # porta marcada larga + deixa folga real ~11cm: só cruza com |lat|<~10cm (as
    # travessias boas commitaram a 0-9cm; a que raspou foi 18cm -> agora rejeitada,
    # recentra/larga pro nav2 em vez de raspar). É live-tunable.
    turn_standoff: float = 0.5      # m — STANDOFF mínimo do plano da porta pra GIRAR
    # no lugar (2026-06-18). O giro varre os cantos do robô num raio ~0.35m; girar
    # colado na porta enfia o canto no batente (campo: "virou perto demais e deu
    # uma porradona"). Antes a decisão de girar olhava SÓ o alinhamento lateral
    # (|d|<=fit) e ignorava a distância -> girava onde calhasse (0.82m=ok num teste,
    # 0.38m=bateu no outro: sorte do approach). Agora: só gira se s<=-turn_standoff
    # (longe). No eixo MAS colado -> ré reta pra ganhar distância, DEPOIS gira longe.
    # Se já estiver ALINHADO, atravessa direto mesmo colado (sem giro = sem varrer).
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
    # FREIO do giro perto do alvo (2026-06-17, 3ª rodada): a velocidade cheia (4.0
    # = 11.5°/tick) PASSAVA DIRETO da banda de ±align_yaw e oscilava esq/dir ->
    # bateu no batente. Longe do alvo (|yaw_err| > rot_brake_angle) gira cheia
    # (quebra o atrito); dentro do rot_brake_angle desacelera pra rot_brake_speed
    # (~5°/tick) pra ENCAIXAR sem overshoot. Não é o "proporcional" reprovado (que
    # abaixava o giro inteiro): aqui só os últimos graus, e o robô JÁ está girando
    # (atrito quebrado) -> não stalla. Sem boost no freio (não precisa).
    rot_brake_angle: float = math.radians(12.0)  # rad — a partir daqui, freia
    rot_brake_speed: float = 2.0    # rad/s — velocidade do giro dentro do freio
    # WAYPOINT pré-porta (2026-06-18): nav2 leva o robô a W (no eixo, recuado,
    # centrado) e o door só alinha+cruza. Substitui a aproximação reativa.
    wp_standoff: float = 1.0        # m — distância de W antes do centro da porta
    wp_retries: int = 2             # re-tentativas de mandar W antes de desistir
    wp_timeout: float = 30.0        # s — tempo do nav2 chegar em W antes de re-tentar
    cross_speed: float = 0.22       # m/s — travessia (0.15->0.22 em 2026-06-16: vencer o atrito estático sem patinar)
    cross_k_lat: float = 1.5        # corrige offset lateral durante a travessia
    cross_k_yaw: float = 2.0        # corrige heading durante a travessia
    cross_wz_max: float = 0.8       # rad/s — teto da micro-correção (NÃO girar)
    gap_min: float = 0.45           # m — vão mínimo na APROXIMAÇÃO (staging/rotating);
    # abaixo disso larga pro nav2 (que freia pelo collision). A travessia usa o
    # stop_dist abaixo (PARA em vez de largar).
    # SEGURANÇA da travessia (2026-06-17, "caminho B"): como o door_vel fura o
    # collision monitor (prio 20), o door_crossing É a autoridade de segurança no
    # crossing. Em vez da janela estreita antiga (0.80×±0.28 que deixou ele ir pra
    # cima de uma pessoa), olha uma ZONA DE PARADA mais larga/longa (gap_ahead) e,
    # se tiver obstáculo não-batente, PARA (vx=0) e segura — não fura cego.
    stop_zone_half_w: float = 0.30  # m — meia-largura da zona vigiada (corpo 0.25 +5cm)
    stop_look_ahead: float = 1.0    # m — alcance da vigia (também o gap reportado no log)
    stop_dist: float = 0.6          # m — obstáculo mais perto que isto no crossing -> PARA
    stop_hold_timeout: float = 8.0  # s — parado esperando liberar; estourou -> aborta pro nav2
    exit_margin: float = 0.30       # m — centro além do plano da porta p/ SOLTAR.
    # 2026-06-18 (0.5 -> 0.30): o door fez o papel dele quando o robô PASSOU DOS
    # BATENTES (traseira limpou o plano da porta ~= meio-comprimento, 0.30). Daí
    # pra frente é problema do NAV2, não do door. Com 0.5 o door continuava
    # CORRIGINDO heading/lateral por mais meio metro DEPOIS de já estar do outro
    # lado — e era nessa faixa que ele costurava, roçava o batente e enlouquecia
    # (campo 06-18: entrou reto colado no batente, o "já passei" só dispararia a
    # 0.5 mas ele travava/pivotava antes de chegar lá -> nunca soltava -> spin).
    # Soltar assim que limpa o batente mata a costura pós-porta na origem.
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
        self._escape_count = 0          # rés de escape NESTA travessia
        self._rot_dir = 0               # sentido do giro do episódio atual (+1 esq/-1 dir/0 livre)
        self._align_t0 = 0.0            # início do "tentando alinhar" (sub-timeout)
        self._align_anchor = (0.0, 0.0)  # posição de referência do substuck
        self._esc_start = (0.0, 0.0)    # pose (x,y) no começo da ré atual
        self._esc_target = 0.0          # quanto recuar nesta ré
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
                if abs(yaw_err) > cfg.align_yaw and s > -cfg.turn_standoff:
                    # PRECISA GIRAR (yaw torto) mas está COLADO na porta: girar aqui
                    # varre o canto do robô pra dentro do batente (campo 06-18).
                    # MANOBRA LONGE: ré RETA pra ganhar standoff, e o reversing volta
                    # pro staging -> aí, longe o bastante, gira. Respeita o vão atrás;
                    # sem espaço -> larga pro nav2. (Só dispara quando precisa girar:
                    # se já estiver alinhado, NÃO dá ré — vai pro rotating que só
                    # parka, sem varrer; e a universal atravessa no tick seguinte.)
                    avail = rear_gap - cfg.escape_rear_margin
                    if avail < cfg.escape_rear_min:
                        return self._abort(now)
                    self.state = 'reversing'
                    self._esc_start = (x, y)
                    self._esc_target = min(cfg.escape_reverse_dist, avail)
                    return Cmd('reversing', -cfg.escape_reverse_speed, 0.0,
                               self.door['id'])
                # NO EIXO e (LONGE o bastante pra girar OU já alinhado): alinha NO
                # LUGAR (rotating gira se preciso, parka se já reto). Era o "fica se
                # enrolando indo pro eixo sendo que já está no meio" (2026-06-17).
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
            # SAÍDA PRIMEIRO (06-18): passou dos batentes (s além do plano) -> o door
            # cumpriu o papel, SOLTA pro nav2, não importa o que tem à frente. Isto
            # TEM que vir antes do gap-stop: senão, com o robô JÁ do outro lado mas
            # uma parede a <stop_dist à frente (corredor de saída), o gap-stop
            # congelava o robô atravessado e só largava pelo timeout de 8s (campo
            # 06-18: "atravessou e ficou parado e travado"). O gap-stop (pessoa) só
            # faz sentido ENQUANTO ainda está no vão.
            if s > cfg.exit_margin:
                # Solta com success_cooldown (2026-06-17): o /plan (~1Hz) ainda
                # mostra por ~1s a rota velha cruzando a porta -> sem cooldown o robô
                # re-armava, invertia o `side` e tentava voltar pra porta que já
                # passou. O cooldown segura até o plano atualizar e sair de vez.
                self.state = 'idle'
                self.door = None
                self.geom = None
                self._cooldown_until = now + cfg.success_cooldown
                return Cmd('idle', 0.0, 0.0, None)
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
                ('exit_margin', 0.30), ('rate_hz', 20.0),
                # 2026-06-17 (atravessar reto): folga geométrica + cooldown +
                # trava de taxa de giro (só cruza quando parou de girar)
                ('robot_half_width', 0.25), ('fit_margin', 0.13),
                ('turn_standoff', 0.5),
                ('success_cooldown', 2.0), ('cross_yaw_rate_max', 0.5),
                ('rot_brake_deg', 12.0), ('rot_brake_speed', 2.0),
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
                zone_radius=g['zone_radius'], stage_dist=g['stage_dist'],
                align_lat=g['align_lat'],
                align_yaw=math.radians(g['align_yaw_deg']),
                align_timeout=g['align_timeout'], rot_speed=g['rot_speed'],
                rot_left_boost=g['rot_left_boost'],
                cross_speed=g['cross_speed'], stage_speed=g['stage_speed'],
                escape_reverse_speed=g['escape_reverse_speed'],
                gap_min=g['gap_min'], exit_margin=g['exit_margin'],
                robot_half_width=g['robot_half_width'],
                fit_margin=g['fit_margin'], turn_standoff=g['turn_standoff'],
                success_cooldown=g['success_cooldown'],
                cross_yaw_rate_max=g['cross_yaw_rate_max'],
                rot_brake_angle=math.radians(g['rot_brake_deg']),
                rot_brake_speed=g['rot_brake_speed'],
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
                       'fit_margin', 'turn_standoff', 'success_cooldown',
                       'cross_yaw_rate_max',
                       'rot_brake_speed', 'stop_zone_half_w', 'stop_look_ahead',
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
                gap = gap_ahead(ranges, amin, ainc, pose, jambs, 0.30,
                                half_w=self.cfg.stop_zone_half_w,
                                max_x=self.cfg.stop_look_ahead)

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
            # diagnóstico de campo (2026-06-17): yaw_err/lat/taxa/gaps na transição
            # E estrangulado durante a manobra (~2 Hz), p/ achar a causa do "bateu
            # no batente" sem adivinhar (overshoot do giro? lateral? cruzou torto?).
            dbg = ('s=%+.2f yaw_err=%+.1f° lat=%+.0fcm taxa=%.1f vx=%+.2f wz=%+.2f '
                   'gap=%.2f front=%.2f' % (
                       self.sup.dbg_s, math.degrees(self.sup.dbg_yaw_err),
                       self.sup.dbg_d * 100, self.sup.dbg_yaw_rate, cmd.vx, cmd.wz,
                       gap, front_gap))
            if cmd.state != prev:
                self.get_logger().info(
                    f'door_crossing: {prev} -> {cmd.state} | {dbg}')
            elif cmd.state in ('staging', 'rotating', 'crossing', 'reversing'):
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
