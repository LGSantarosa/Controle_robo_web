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


def will_clear(g: DoorGeom, s: float, d: float, yaw_err: float, side: int,
               robot_half_width: float, fit_margin: float) -> bool:
    """True se o robô PASSA reto pela porta a partir de onde está (trava
    "passo aqui?"). GEOMÉTRICO, sem LiDAR.

    Projeta a linha de heading ATUAL (offset lateral `d`, erro de ângulo
    `yaw_err` vs o eixo) até o plano dos batentes (s=0) e compara com a folga
    útil. Inclui o YAW (não só o lateral): foi o yaw que falhou em campo
    2026-06-19 (lateral OK, mas apontando pro batente).

      lat_no_batente = d + s * side * tan(yaw_err)   # s<0 na aproximação
      fit            = half_width - robot_half_width - fit_margin

    s>=0 já passou do ponto mais estreito -> sempre passa. O termo `s*side`
    dá o braço de alavanca (quanto mais longe e mais torto, mais desvia)."""
    if s >= 0.0:
        return True
    lat = d + s * side * math.tan(yaw_err)
    fit = g.half_width - robot_half_width - fit_margin
    return abs(lat) <= fit


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


# ---- máquina de estados pura ------------------------------------------------

@dataclass
class DoorCrossConfig:
    # 2026-06-19: 1.2 -> 0.9 -> 1.1. O ponto-pré-porta fica a 1.0 m do centro.
    # 1.2 armava 0.2 m ANTES do ponto -> pegava o robô torto (apontava pra parede).
    # 0.9 (< 1.0) foi LONGE demais: o robô parava NO ponto (1.0 m), FORA da zona,
    # e o door nunca pegava (só armava se o robô continuasse entrando, com atraso
    # enorme/"não ativou"). A zona TEM que ser >= standoff. 1.1 arma só 0.1 m antes
    # do ponto, com o robô já vindo centrado pelo xy_goal_tolerance=0.15 do nav2.
    zone_radius: float = 1.1        # m — distância do centro que arma a manobra (>= standoff 1.0)
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
    align_lat: float = 0.08         # m — |offset lateral| máximo pra "tô no eixo"
    align_yaw: float = math.radians(3.0)   # rad — |erro de yaw| máximo (5°->3° em 2026-06-19: 5° numa porta de ~11cm de folga desvia ~9cm no fim -> apontava pro batente; 3° alinha mais reto)
    align_stable: int = 5           # ticks consecutivos dentro da tolerância
    # 2026-06-15: experimento 15 -> 600 REVERTIDO pra 15. O 600 não fazia o robô
    # "tentar mais" — transformava um STALL (ver stage_dist) num FREEZE de 10
    # min. O "não desistir do ponto" real era o timeout do MapBridge web (120 ->
    # 3600), já resolvido. Aqui 15s é a rede de segurança: se não alinhar,
    # aborta e devolve pro nav2 em vez de congelar.
    align_timeout: float = 15.0     # s — STAGING+ROTATING juntos
    rot_speed: float = 3.0          # rad/s — TETO do giro no lugar (4->3 em 2026-06-19: giro estava forte/passando do alvo pós-fitas nas rodas; proporcional rot_k/piso rot_min seguem; NUNCA arco)
    rot_k: float = 6.0              # ganho P do giro: desacelera perto do alvo (2026-06-16; bang-bang passava da janela ±5° e ficava caçando dir/esq)
    rot_min: float = 2.5            # rad/s — PISO do giro: abaixo disso o skid-steer não vira (atrito); nunca desacelera além disso
    cross_speed: float = 0.22       # m/s — travessia (0.15->0.22 em 2026-06-16: vencer o atrito estático sem patinar)
    cross_k_lat: float = 1.5        # corrige offset lateral durante a travessia
    cross_k_yaw: float = 2.0        # corrige heading durante a travessia
    cross_wz_max: float = 0.8       # rad/s — teto da micro-correção (NÃO girar)
    # 2026-06-19: passado o centro (s >= cross_lat_off_s) PARA de corrigir lateral.
    # A correção lateral persegue o eixo dos 2 cliques (doors.json), NÃO o corredor
    # real -> no fim da travessia dava uma curvinha desnecessária que deixava o robô
    # anguladinho no corredor pós-porta (relato de campo). Antes do centro corrige
    # (pra cruzar centrado no batente); depois sai RETO, só segurando o yaw.
    cross_lat_off_s: float = 0.0    # m — progresso (s) a partir do qual zera a correção lateral (0 = centro)
    # Trava "passo aqui?" geométrica com yaw (2026-06-22, pendência A; volta do
    # backup door-redesign-0618 + projeção do yaw). will_clear projeta a
    # trajetória reta até o plano dos batentes; se não passa, re-estagia.
    robot_half_width: float = 0.25  # m — meia-largura do robô (0.50 medido roda-a-roda)
    fit_margin: float = 0.13        # m — folga subtraída do vão no will_clear (fit = half_width - robot_half_width - fit_margin). KNOB DE CAMPO nº1: re-estagia à toa -> DIMINUIR fit_margin (afrouxa a trava)
    # Ponto de não-retorno (2026-06-22, capengada de campo): nos últimos cm antes
    # do plano (s>commit_s) o robô já está com >metade do corpo no vão; o braço de
    # alavanca do will_clear some (lat≈d) e um offset lateral residual irrelevante
    # disparava a ré a meia-travessia (saía de um lugar que JÁ estava passando bem).
    # A partir daqui PARA de re-checar e COMITA pra frente — dar ré meio-atravessado
    # é mais arriscado que terminar. commit_s=-0.15: para-choque (front_head_x~0.25)
    # já ~0.10 m além do plano. A trava segue inteira longe da porta (onde importa).
    commit_s: float = -0.15         # m — progresso (s) a partir do qual não re-checa o will_clear (commit)
    gap_min: float = 0.45           # m — vão mínimo à frente pra seguir
    exit_margin: float = 0.5        # m — além do centro pra soltar
    total_timeout: float = 40.0     # s — manobra inteira (revertido de 600; ver align_timeout)
    retrigger_cooldown: float = 3.0  # s — após abort, não rearmar na hora
    crossing_cooldown: float = 8.0   # s — após CRUZAR, não re-armar (campo 06-22: a ré pós-porta trazia o robô de volta pra zona e re-armava a door — sendo que ele já estava do OUTRO lado)
    # Ré de ESCAPE (2026-06-16): sem a ré do unstuck (calado na região da
    # porta), o door_crossing precisa se reajustar sozinho — senão fica
    # morto-preso de nariz na parede (e stalla o motor -> desarma o BMS, já que
    # door_vel fura o collision). Ré RETA (NUNCA arco), gated pelo vão traseiro.
    escape_front_gap: float = 0.20      # m — obstáculo a menos disso à frente -> ré (anti-stall)
    escape_substuck_time: float = 5.0   # s — alinhando sem chegar ao crossing -> ré
    escape_reverse_dist: float = 0.30   # m — quanto recua por escape (teto)
    escape_reverse_speed: float = 0.25  # m/s — ré de escape (0.12->0.25 em 2026-06-16: = ré do unstuck, validada vencendo o atrito em campo)
    escape_max_count: int = 0           # nº de escapes por travessia antes de abortar; 0 = ILIMITADO (campo 06-22: a door ajusta bem, deixar PRESO na zona — unstuck calado no standdown — era pior que seguir re-estagiando; total_timeout segue como rede)
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
        self._stable = 0
        self._cooldown_until = 0.0
        self._escape_count = 0          # rés de escape NESTA travessia
        self._align_t0 = 0.0            # início do "tentando alinhar" (sub-timeout)
        self._align_anchor = (0.0, 0.0)  # posição de referência do substuck
        self._esc_start = (0.0, 0.0)    # pose (x,y) no começo da ré atual
        self._esc_target = 0.0          # quanto recuar nesta ré

    # -- helpers ------------------------------------------------------------
    def _abort(self, now: float) -> Cmd:
        self.state = 'idle'
        self.door = None
        self.geom = None
        self._stable = 0
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
        return self._enter_reverse(now, pos, rear_gap)

    def _enter_reverse(self, now, pos, rear_gap) -> Cmd:
        """Entra na ré RETA de re-aproximação (nunca arco), limitada pelo vão
        traseiro. Compartilhada pela ré de escape (_maybe_escape) e pela trava
        'passo aqui?' (re-estágio). Sem vão atrás útil -> ABORTA pro nav2. Com
        escape_max_count>0 também aborta ao estourar a contagem; 0 = ilimitado."""
        cfg = self.cfg
        if cfg.escape_max_count > 0 and self._escape_count >= cfg.escape_max_count:
            return self._abort(now)
        target = min(cfg.escape_reverse_dist, rear_gap - cfg.escape_rear_margin)
        if target < cfg.escape_rear_min:
            return self._abort(now)     # sem vão atrás -> não força contra a parede
        self._escape_count += 1
        self.state = 'reversing'
        self._esc_start = pos
        self._esc_target = target
        return Cmd('reversing', -cfg.escape_reverse_speed, 0.0, self.door['id'])

    def _pick_door(self, pose, doors):
        x, y, yaw = pose
        for d in doors:
            g = door_geometry(tuple(d['a']), tuple(d['b']))
            dist = math.hypot(x - g.cx, y - g.cy)
            if dist > self.cfg.zone_radius:
                continue
            # "na frente" = QUALQUER parte do vão dentro do cone (centro ou
            # batente); na zona (<=1.2 m) a aproximação pode vir torta e o
            # centro sozinho cair fora do cone com o vão ainda visível.
            bearing = min(
                abs(_wrap(math.atan2(py - y, px - x) - yaw))
                for px, py in ((g.cx, g.cy), tuple(d['a']), tuple(d['b'])))
            if bearing > self.cfg.approach_bearing:
                continue
            return d, g
        return None, None

    # -- tick -----------------------------------------------------------------
    def update(self, now, pose, doors, goal_active, nav_forward, gap,
               scan_fresh, front_gap=math.inf, rear_gap=math.inf) -> Cmd:
        cfg = self.cfg

        if self.state == 'idle':
            if (pose is None or not goal_active or not nav_forward
                    or now < self._cooldown_until or not doors):
                return Cmd('idle', 0.0, 0.0, None)
            door, geom = self._pick_door(pose, doors)
            if door is None:
                return Cmd('idle', 0.0, 0.0, None)
            x, y, _ = pose
            # lado de aproximação: progresso negativo = "antes" da porta
            raw_s = ((x - geom.cx) * geom.nx + (y - geom.cy) * geom.ny)
            self.side = -1 if raw_s > 0 else +1
            self.door, self.geom = door, geom
            # 2026-06-19: a web (nav2 via ponto-pré-porta) já entrega o robô
            # centrado na frente da porta. Arma DIRETO no rotating (só alinha o
            # ângulo) e cruza — NÃO faz staging (não persegue o centro do vão,
            # que era o "buscando eixo" que estragava a posição já boa). O
            # staging fica só como recuperação pós-escape (reversing).
            self.state = 'rotating'
            self.t_start = now
            self._stable = 0
            self._escape_count = 0
            self._align_t0 = now
            self._align_anchor = (x, y)
            # cai no fluxo de rotating já neste tick

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

        if self.state in ('staging', 'rotating'):
            if now - self.t_start > cfg.align_timeout:
                return self._abort(now)

        if self.state == 'staging':
            esc = self._maybe_escape(now, (x, y), front_gap, rear_gap)
            if esc is not None:
                return esc
            # alvo: ponto no eixo, stage_dist antes do centro
            tgx = g.cx - g.nx * self.side * cfg.stage_dist
            tgy = g.cy - g.ny * self.side * cfg.stage_dist
            dist = math.hypot(tgx - x, tgy - y)
            if dist <= cfg.stage_tol:
                self.state = 'rotating'
                self._stable = 0
            else:
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
            # Alinha SÓ o ângulo (yaw) no lugar. NÃO exige estar no eixo
            # (|lat|): o nav2 já entregou centrado e o offset lateral residual é
            # corrigido aos poucos ANDANDO no crossing (cross_k_lat) — nunca
            # girando atrás do centro do vão.
            aligned = abs(yaw_err) <= cfg.align_yaw
            if aligned:
                self._stable += 1
                if self._stable >= cfg.align_stable:
                    # trava "passo aqui?": alinhou, mas se a projeção (lateral +
                    # yaw) bate no batente, NÃO commita -> re-estagia (recua reto,
                    # re-aproxima). Não atravessa torto (pendência A, campo 06-19).
                    if not will_clear(g, s, d, yaw_err, self.side,
                                      cfg.robot_half_width, cfg.fit_margin):
                        return self._enter_reverse(now, (x, y), rear_gap)
                    self.state = 'crossing'
                    return Cmd('crossing', cfg.cross_speed, 0.0,
                               self.door['id'])
                return Cmd('rotating', 0.0, 0.0, self.door['id'])
            self._stable = 0
            # GIRO PROPORCIONAL (validado em campo, 3b40817): rápido longe,
            # DEVAGAR perto do alvo -> não passa da janela ±5° e fica caçando
            # dir/esq (era o bang-bang a ~11°/tick com 4 rad/s @20Hz que voltou
            # no revert 46ec8ab; o sentido-único do 1a0fe30 também girava a vel.
            # cheia e seguia chacoalhando -> relato de campo 2026-06-19). Piso
            # rot_min pra não parar de girar (atrito do skid-steer). O yaw vem
            # do TF (já fundido c/ IMU) -> malha fechada na IMU de graça.
            mag = max(cfg.rot_min, min(cfg.rot_speed, cfg.rot_k * abs(yaw_err)))
            wz = mag if yaw_err < 0 else -mag
            return Cmd('rotating', 0.0, wz, self.door['id'])

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
                # atravessou: solta e ARMA cooldown pra não re-armar com a ré
                # pós-porta (que traz o robô de volta pra zona, já do outro lado).
                self.state = 'idle'
                self.door = None
                self.geom = None
                self._cooldown_until = now + cfg.crossing_cooldown
                return Cmd('idle', 0.0, 0.0, None)
            # trava "passo aqui?" no meio: se a projeção (lateral + yaw) deriva e
            # passa a bater no batente antes de cruzar (s<0), re-estagia em vez de
            # raspar. Passado o estreito (s>=0) will_clear já devolve True. Mas só
            # re-checa ATÉ o ponto de não-retorno (s<=commit_s): nos últimos cm o
            # robô já está comprometido no vão e a ré custa mais que terminar.
            if s <= cfg.commit_s and not will_clear(
                    g, s, d, yaw_err, self.side,
                    cfg.robot_half_width, cfg.fit_margin):
                return self._enter_reverse(now, (x, y), rear_gap)
            # corrige lateral SÓ até o centro; depois sai reto (só segura o yaw)
            # pra não sair anguladinho perseguindo o eixo dos cliques (campo 06-19)
            k_lat = cfg.cross_k_lat if s < cfg.cross_lat_off_s else 0.0
            wz = -k_lat * d - cfg.cross_k_yaw * yaw_err
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
                ('zone_radius', 1.1), ('stage_dist', 0.6),
                ('align_lat', 0.08), ('align_yaw_deg', 3.0),
                ('align_timeout', 15.0), ('rot_speed', 3.0),
                ('rot_k', 6.0), ('rot_min', 2.5),
                ('cross_speed', 0.22), ('stage_speed', 0.20),
                ('cross_lat_off_s', 0.0),
                ('robot_half_width', 0.25), ('fit_margin', 0.13),
                ('commit_s', -0.15),
                ('escape_reverse_speed', 0.25), ('gap_min', 0.45),
                ('exit_margin', 0.5), ('rate_hz', 20.0),
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
                rot_k=g['rot_k'], rot_min=g['rot_min'],
                cross_speed=g['cross_speed'], stage_speed=g['stage_speed'],
                cross_lat_off_s=g['cross_lat_off_s'],
                robot_half_width=g['robot_half_width'],
                fit_margin=g['fit_margin'], commit_s=g['commit_s'],
                escape_reverse_speed=g['escape_reverse_speed'],
                gap_min=g['gap_min'], exit_margin=g['exit_margin'])
            self.sup = DoorCrossing(self.cfg)
            self.scan_stale = g['scan_stale']
            self.nav_move_lin = g['nav_move_lin']
            self.rear_tail_x = g['rear_tail_x']
            self.rear_half_width = g['rear_half_width']
            self.front_head_x = g['front_head_x']
            # LiDAR no CENTRO (0.0) hoje; param (igual ao unstuck) p/ não ficar
            # hardcoded se o sensor sair do centro um dia.
            self.lidar_x = g['lidar_x']

            self.doors = []
            self._goal_active = {}
            self._nav_forward = False
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
            for topic in ('navigate_to_pose/_action/status',
                          'navigate_through_poses/_action/status'):
                self.create_subscription(
                    GoalStatusArray, topic,
                    lambda m, t=topic: self._on_status(t, m), 10)

            self.create_timer(1.0 / g['rate_hz'], self._tick)
            self._publish_zone('idle', None)
            self.get_logger().info(
                'door_crossing ativo: zona %.1fm, alinhar |lat|<%.2fm '
                '|yaw|<%.0f°, atravessa %.2fm/s' % (
                    self.cfg.zone_radius, self.cfg.align_lat,
                    math.degrees(self.cfg.align_yaw), self.cfg.cross_speed))

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
            if (fresh and pose is not None and self.sup.state == 'crossing'
                    and self.sup.door is not None):
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
                                  front_gap, rear_gap)
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
