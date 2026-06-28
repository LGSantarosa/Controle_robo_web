"""Watchdog de desencalhe do Nav2 — unstuck_supervisor.

Se o robô NÃO SE DESLOCA (>stuck_radius) por mais de `stuck_timeout` segundos
enquanto o nav2 está comandando, este nó assume o controle por um canal do
twist_mux que FURA o collision monitor (`unstuck_vel`, prioridade acima do
`nav_vel`) e dá RÉ; depois solta e o nav2 replaneja e desvia.

Decisões de campo (2026-06-10, validadas em teste ao vivo):
- Gatilho por DESLOCAMENTO, não velocidade: o robô "tentando girar" sem sair
  do lugar (RotationShim, recoveries do nav2) mexe uns mm e enganava o gatilho
  por velocidade. Não se deslocou = travado, ponto.
- SEM GIRO: o giro a baixa velocidade não vence o atrito do skid-steer (parecia
  "não fez nada"). A manobra é SEMPRE ré.
- Ré com OLHO NO VÃO (batida de 2026-06-11: a checagem antiga por setor
  angular media do LiDAR e era cega pra quina — o robô recuou em cima de um
  obstáculo atrás): `rear_min_gap` mede em METROS o vão real entre o
  para-choque traseiro e o /scan num corredor retangular da largura do robô.
  Sem vão útil → espera; vão curto → ré PARCIAL (recua o que dá); vão some
  no MEIO da manobra → STOP imediato.
- Só age com goal ativo: o gate primário é o STATUS do action server do
  bt_navigator (ACCEPTED/EXECUTING/CANCELING = ativo) — autoritativo, mata a
  "ré póstuma" pós-cancel e cobre o BT em recovery com o controller mudo.
  Sem status visto ainda, cai no fallback: nav2 comandou há <nav_latch
  (latch tolera os gaps de ~1-2s do ciclo de abort do progress_checker).
- Fim da ré publica um Twist ZERO explícito (cmd_vel_to_wheels segura o último
  comando) e /scan velho >scan_stale trata a traseira como bloqueada.

O collision monitor e a curva (RotationShim/DWB) NÃO são tocados. Ver
`docs/superpowers/specs/2026-06-10-unstuck-supervisor-design.md`.

A lógica de decisão é pura (sem ROS) pra ser testável offline; o nó embaixo é
só a cola de I/O.
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, NamedTuple, Optional, Tuple

import numpy as np


# ---- lógica pura -----------------------------------------------------------

def _norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class MapGrid(NamedTuple):
    """Recorte leve do nav_msgs/OccupancyGrid pra lookup puro (sem ROS)."""
    data: List[int]      # row-major, 0-100 (ocupação) ou -1 (desconhecido)
    width: int
    height: int
    resolution: float    # m/célula
    origin_x: float      # canto (0,0) do grid no frame do mapa
    origin_y: float


def map_occupied(grid: MapGrid, x: float, y: float, neighborhood: float,
                 occ_threshold: int) -> bool:
    """True se alguma célula dentro de `neighborhood` (m) do ponto (x,y) está
    OCUPADA (valor >= occ_threshold) no mapa estático. Recovery contextual
    (2026-06-22): só dá ré rápida se o bloqueio à frente coincide com parede já
    MAPEADA. Célula desconhecida (-1) ou fora dos limites = NÃO ocupada (não
    encurta o timeout — fallback seguro)."""
    res = grid.resolution
    if res <= 0.0:
        return False
    cr = int(neighborhood / res) + 1     # raio de busca em células
    col0 = int((x - grid.origin_x) / res)
    row0 = int((y - grid.origin_y) / res)
    for dr in range(-cr, cr + 1):
        for dc in range(-cr, cr + 1):
            r, c = row0 + dr, col0 + dc
            if not (0 <= r < grid.height and 0 <= c < grid.width):
                continue
            v = grid.data[r * grid.width + c]
            if v < occ_threshold:        # -1 (desconhecido) e livres caem aqui
                continue
            # centro da célula no frame do mapa
            cx = grid.origin_x + (c + 0.5) * res
            cy = grid.origin_y + (r + 0.5) * res
            if math.hypot(cx - x, cy - y) <= neighborhood:
                return True
    return False


def rear_min_gap(ranges, angle_min: float, angle_increment: float,
                 lidar_x: float, tail_x: float, half_width: float) -> float:
    """Menor vão livre (m) entre o PARA-CHOQUE traseiro e o que o /scan vê
    no corredor que o corpo varre dando ré. inf = nada atrás.

    Substitui o setor angular de 2026-06-10, que causou a batida de ré de
    2026-06-11 por 3 vias: media a folga a partir do LIDAR e não do
    para-choque (0.35m de "folga" = 0.10m de vão real, e a ré recuava
    0.30m), o cone de ±30° era mais estreito que o robô (quina traseira a
    ~43° passava despercebida), e era um bool sem noção de quanto espaço
    existe. Aqui cada ponto vira (x,y) no frame base_link e conta se cai
    no retângulo atrás do robô: x < tail_x, |y| <= half_width.
    `lidar_x` = posição do LiDAR no frame base (0.0 = centro, confirmado
    pelo usuário 2026-06-11: TODOS os sensores ficam no centro do robô).
    """
    if angle_increment == 0.0:
        return math.inf
    # Vetorizado (P2 da AUDITORIA_2026-06-11): ~450 pts a 10 Hz com trig em
    # Python puro era CPU de verdade na Pi. None vira NaN no asarray e cai
    # no isfinite — mesmo descarte da versão escalar.
    r = np.asarray(ranges, dtype=np.float64)
    if r.size == 0:
        return math.inf
    ok = np.isfinite(r) & (r > 0.0)
    r = np.where(ok, r, 0.0)  # evita inf*cos -> NaN com warning
    a = angle_min + np.arange(r.size) * angle_increment
    x = lidar_x + r * np.cos(a)
    y = r * np.sin(a)
    sel = ok & (x < tail_x) & (np.abs(y) <= half_width)
    if not sel.any():
        return math.inf
    return float((tail_x - x[sel]).min())


def front_min_gap(ranges, angle_min: float, angle_increment: float,
                  lidar_x: float, head_x: float, half_width: float) -> float:
    """Espelho dianteiro do `rear_min_gap`: menor vão livre (m) entre o
    PARA-CHOQUE dianteiro (head_x) e o que o /scan vê no corredor retangular
    que o corpo varre AVANÇANDO. inf = nada na frente.

    Usado pelo escape pra frente (pedido 2026-06-15): quando a traseira está
    bloqueada, o robô avança em vez de travar. Como o canal `unstuck_vel` fura
    o collision monitor, esta checagem é o "respeitar o collision" do avanço —
    mede na MESMA /scan e nunca deixa avançar em cima de obstáculo. Cada ponto
    vira (x,y) em base_link e conta se cai no retângulo à frente do robô:
    x > head_x, |y| <= half_width.
    """
    if angle_increment == 0.0:
        return math.inf
    r = np.asarray(ranges, dtype=np.float64)
    if r.size == 0:
        return math.inf
    ok = np.isfinite(r) & (r > 0.0)
    r = np.where(ok, r, 0.0)
    a = angle_min + np.arange(r.size) * angle_increment
    x = lidar_x + r * np.cos(a)
    y = r * np.sin(a)
    sel = ok & (x > head_x) & (np.abs(y) <= half_width)
    if not sel.any():
        return math.inf
    return float((x[sel] - head_x).min())


def front_block_point(ranges, angle_min, angle_increment, lidar_x, head_x,
                      half_width):
    """(x,y) em base_link do retorno mais próximo à FRENTE — a "parte que
    travou". Mesmo corredor do `front_min_gap` (x>head_x, |y|<=half_width), mas
    devolve o PONTO (com o offset lateral real), não só a distância. None se o
    corredor está livre. Campo 2026-06-22: o robô encosta torto, então o contato
    NÃO está reto à frente — projetar reto errava a parede no mapa."""
    if angle_increment == 0.0:
        return None
    r = np.asarray(ranges, dtype=np.float64)
    if r.size == 0:
        return None
    ok = np.isfinite(r) & (r > 0.0)
    r = np.where(ok, r, 0.0)                          # evita inf*cos (warning/nan)
    a = angle_min + np.arange(r.size) * angle_increment
    x = lidar_x + r * np.cos(a)
    y = r * np.sin(a)
    sel = ok & (x > head_x) & (np.abs(y) <= half_width)
    if not sel.any():
        return None
    i = int(np.argmin(np.where(sel, x, np.inf)))    # o mais próximo (menor x)
    return (float(x[i]), float(y[i]))


def block_point_mapped(grid, position, yaw, bp, head_x, block_range,
                       neighborhood, occ_threshold) -> bool:
    """True se o ponto de contato `bp` (x,y em base_link, do `front_block_point`)
    coincide com parede MAPEADA. Transforma bp pro frame do MAPA com a rotação
    2D COMPLETA (preserva o offset lateral — era o bug: projetava só reto à
    frente) e consulta a vizinhança. Gated pela distância frontal ao para-choque
    (bx-head_x <= block_range). Sem grid/contato -> False."""
    if grid is None or bp is None:
        return False
    bx_r, by_r = bp
    if (bx_r - head_x) > block_range:
        return False
    c, s = math.cos(yaw), math.sin(yaw)
    mx = position[0] + bx_r * c - by_r * s
    my = position[1] + bx_r * s + by_r * c
    return map_occupied(grid, mx, my, neighborhood, occ_threshold)


def freer_side(ranges, angle_min: float, angle_increment: float) -> int:
    """+1 se o setor frontal ESQUERDO (20°..90°) tem mais espaço, -1 se o direito.

    Usado pra escolher pra que lado a ré em arco vira o nariz.
    """
    if angle_increment == 0.0:
        return 1
    r = np.asarray(ranges, dtype=np.float64)
    if r.size == 0:
        return 1
    lo, hi = math.radians(20.0), math.radians(90.0)
    a = angle_min + np.arange(r.size) * angle_increment
    a = np.arctan2(np.sin(a), np.cos(a))  # wrap (-pi, pi], = _norm_angle
    ok = np.isfinite(r) & (r > 0.0)
    left = ok & (a >= lo) & (a <= hi)
    right = ok & (a >= -hi) & (a <= -lo)
    best_left = float(r[left].min()) if left.any() else math.inf
    best_right = float(r[right].min()) if right.any() else math.inf
    return 1 if best_left >= best_right else -1


def door_zone_active(state: str) -> bool:
    """True se o door_crossing está CONDUZINDO (staging/rotating/crossing),
    dando a ré de escape ('reversing') OU apenas SE APROXIMANDO ('approaching')
    de uma porta marcada -> unstuck em standdown. 'approaching' incluído em
    2026-06-16: sem ele, o unstuck (prio 30, ré+giro 15°) sabotava a aproximação
    antes do door_crossing assumir. 'reversing' também: a ré de escape é o
    door_crossing se reajustando sozinho — o unstuck não pode atropelar a
    manobra. Garbage/'idle' -> False (não silencia a rede de segurança à toa)."""
    return state in ('approaching', 'staging', 'rotating', 'crossing',
                     'reversing')


@dataclass
class UnstuckConfig:
    stuck_timeout: float = 10.0
    # Recovery contextual (2026-06-22): se o bloqueio à frente coincide com
    # parede JÁ MAPEADA (não vai sair andando), dá ré bem antes dos 10 s. Os
    # mesmos segundos servem de mini-confirmação (mapeado contínuo por esse
    # tempo) pra não reagir a um frame solto de mapa/pose. Obstáculo NOVO (só no
    # LiDAR, livre no /map) segue o stuck_timeout cheio. Ver
    # docs/superpowers/specs/2026-06-22-unstuck-recovery-contextual-design.md
    stuck_timeout_mapped: float = 2.0
    stuck_radius: float = 0.05     # deslocou menos que isso = "parado"
    # 2026-06-27 BO: o robô point-turnando (vx=0, girando no lugar) NÃO desloca,
    # então o unstuck achava que travou e dava RÉ no meio do giro legítimo do
    # path_follower, fodendo o nav2. Rotação > stuck_yaw também conta como
    # PROGRESSO (re-ancora). Travado de verdade = comanda giro mas o yaw não muda.
    stuck_yaw: float = 0.15        # rad (~9°) — girou mais que isso = "fez progresso"
    reverse_distance: float = 0.30
    reverse_speed: float = 0.25
    reverse_time_cap: float = 6.0
    grace: float = 2.0
    nav_latch: float = 15.0        # nav2 conta como "com goal" se comandou há <=15s
    # Escalada (pedido 2026-06-10: "limite de 3 tentativas até pensar em virar"):
    # ré reta repetida no MESMO ponto não resolve -> a partir da 3ª, depois da
    # ré vem um GIRO FORTE no lugar. Forte porque giro fraco não vence o atrito
    # do skid-steer (0.5 falhou em campo; o RotationShim gira o robô a 3.67).
    # Arco durante a ré também falhou (30cm não muda heading).
    escalate_after: int = 3        # tentativas no mesmo ponto antes do giro
    same_spot_radius: float = 0.5  # raio que define "mesmo ponto"
    escalate_window: float = 120.0  # esquece travamentos mais velhos que isso
    spin_speed: float = 3.0        # rad/s do giro pós-ré (precisa vencer atrito)
    # Giro em MALHA FECHADA no yaw (campo: comanda 30° e a roda patinando
    # entrega 5°): gira até o yaw MEDIDO (IMU, confiável mesmo patinando)
    # acumular spin_angle; spin_time_cap é o teto se nem patinar resolver.
    spin_angle: float = 0.44       # alvo de virada REAL (~25°)
    spin_time_cap: float = 4.0     # teto de tempo do giro
    # As rodas pegam pior girando pra ESQUERDA -> boost de FORÇA nesse lado.
    spin_left_boost: float = 1.4   # velocidade do giro à esquerda x1.4
    # Segurança da ré (batida de 2026-06-11: ré em cima de obstáculo atrás).
    # A ré só sai se houver vão útil, recua NO MÁXIMO (vão - margem) e aborta
    # na hora se o vão cair abaixo da margem durante a manobra.
    rear_stop_margin: float = 0.10  # nunca chega a menos disso do obstáculo
    reverse_min: float = 0.10       # vão útil mínimo pra valer a pena dar ré
    # Escape PRA FRENTE (pedido 2026-06-15: "se o obstáculo é atrás, para e
    # ajusta pra frente"). Antes a manobra era SÓ ré -> com obstáculo atrás o
    # robô travava (recusava a ré e não tinha plano B). Agora: traseira sem vão
    # útil + frente livre -> avança. Conservador de propósito (a frente é onde
    # ele atropelou alguém em 06-08): mais devagar e mais curto que a ré, e
    # gated pelo front_min_gap (aborta se a frente fechar). A ré segue PREFERIDA
    # quando há vão atrás (caso comum: obstáculo na frente -> recua).
    forward_distance: float = 0.20  # avanço mais curto que a ré (0.30)
    forward_speed: float = 0.15     # mais devagar que a ré (0.25)
    forward_time_cap: float = 6.0
    front_stop_margin: float = 0.10  # nunca chega a menos disso do obstáculo à frente
    forward_min: float = 0.10        # vão frontal mínimo pra valer o avanço
    # 2026-06-28 OPÇÃO A ("vai bater de verdade?"): com a frente LIVRE a parada
    # geralmente não é obstáculo (collision freando manobra/giro, ou alinhamento) ->
    # DEFERE a recovery (dá tempo pro nav). MAS não suprime pra sempre: se ficar
    # travado além de front_clear_timeout mesmo com a frente "livre" (bloqueio
    # LATERAL/no giro que o front reto não enxerga), dispara assim mesmo (senão o
    # robô fica preso eternamente). Frente bloqueada (<front_clear) = dispara no
    # timeout normal. Ver ESTADO 06-28.
    front_clear: float = 0.40        # m — frente com >isto de vão = "livre" (defere)
    front_clear_timeout: float = 15.0  # s — travado c/ frente livre + obstáculo DESCONHECIDO -> age
    # "conheço esse obstáculo?": se há parede MAPEADA perto do robô (batente, etc.),
    # a parada não é surpresa -> age MUITO mais rápido (não espera os 15s). Pedido do
    # dono 2026-06-28 (defer tava demorando demais pra desencalhar do conhecido).
    front_clear_timeout_mapped: float = 3.0  # s — idem mas perto de parede MAPEADA
    mapped_near_radius: float = 0.35  # m — parede mapeada a <isto do robô = "conhecido"


class Command(NamedTuple):
    lin: float
    ang: float
    active: bool


_IDLE = Command(0.0, 0.0, False)

# estados
_MONITORING = "monitoring"
_REVERSING = "reversing"
_ADVANCING = "advancing"
_SPINNING = "spinning"
_GRACE = "grace"


@dataclass
class UnstuckSupervisor:
    cfg: UnstuckConfig
    state: str = _MONITORING
    anchor: Optional[Tuple[float, float]] = None  # última posição "nova"
    anchor_t: float = 0.0
    anchor_yaw: float = 0.0        # yaw quando ancorou (rotação reseta o stuck)
    mapped_since: Optional[float] = None  # desde quando o bloqueio à frente é parede mapeada
    maneuver_start_t: float = 0.0
    maneuver_start_pos: Tuple[float, float] = (0.0, 0.0)
    reverse_target: float = 0.0    # quanto recuar NESTA manobra (<= reverse_distance)
    forward_target: float = 0.0    # quanto avançar NESTA manobra (<= forward_distance)
    grace_start: float = 0.0
    last_nav_t: Optional[float] = None
    escalated: bool = False    # esta manobra termina em giro forte?
    spin_side: int = 1         # +1 esq / -1 dir
    spin_start_t: float = 0.0
    spin_start_yaw: float = 0.0
    history: List[Tuple[float, Tuple[float, float]]] = field(default_factory=list)

    def update(self, now: float, *, nav_wants_move: bool,
               position: Tuple[float, float], rear_gap: float = math.inf,
               front_gap: float = math.inf,
               goal_active: Optional[bool] = None,
               open_side: int = 1, yaw: float = 0.0,
               door_active: bool = False,
               obstacle_mapped: bool = False,
               near_mapped: bool = False) -> Command:
        if nav_wants_move:
            self.last_nav_t = now
        if door_active:
            # STANDDOWN: o door_crossing está conduzindo a travessia (door_vel,
            # prio 20 no twist_mux). O unstuck (prio 30) SOBREPÕE e sabotava a
            # manobra — revertia/girava o robô pra fora do ponto de alinhamento,
            # então o door_crossing nunca fechava |lat|<8cm/|yaw|<5° dentro do
            # align_timeout e abortava em loop (campo 2026-06-15: "5 min na
            # porta", door_crossing staging->idle de 15 em 15s). Enquanto a porta
            # está ativa o unstuck fica quieto E não acumula tempo de "travado"
            # (re-ancora). Se a travessia genuinamente travar, o PRÓPRIO
            # door_crossing aborta (vão/timeout) -> door_active cai -> o unstuck
            # volta a poder agir.
            self.state = _MONITORING
            self.anchor = None
            self.mapped_since = None
            return _IDLE
        if self.state == _MONITORING:
            return self._monitoring(now, position, rear_gap, front_gap,
                                    goal_active, open_side, obstacle_mapped, yaw,
                                    near_mapped)
        if self.state == _REVERSING:
            return self._reversing(now, position, yaw, rear_gap)
        if self.state == _ADVANCING:
            return self._advancing(now, position, front_gap)
        if self.state == _SPINNING:
            return self._spinning(now, yaw)
        if self.state == _GRACE:
            return self._grace(now)
        return _IDLE

    # -- estados --

    def _monitoring(self, now, position, rear_gap, front_gap, goal_active,
                    open_side, obstacle_mapped=False, yaw=0.0,
                    near_mapped=False) -> Command:
        if goal_active is not None:
            # status do action server do nav2 disponível: é AUTORITATIVO.
            # Mata a "ré póstuma" (goal cancelado mas flag de nav_vel_raw
            # parado em True) e cobre o BT em recovery (controller mudo
            # mas goal seguindo ativo).
            nav_gate = goal_active
        else:
            # fallback sem status: nav2 comandou há <nav_latch (tolera os
            # gaps de ~1-2s do ciclo de abort do progress_checker)
            nav_gate = (self.last_nav_t is not None
                        and now - self.last_nav_t <= self.cfg.nav_latch)
        if not nav_gate:
            # sem goal ativo: parado aqui é normal (goal atingido/cancelado)
            self.anchor = None
            self.mapped_since = None
            return _IDLE
        # continuidade do "bloqueio à frente é parede mapeada" (recovery
        # contextual): rastreado a cada tick com goal ativo. Zera assim que
        # deixa de ser mapeado (a janela de confirmação recomeça do zero).
        if obstacle_mapped:
            if self.mapped_since is None:
                self.mapped_since = now
        else:
            self.mapped_since = None
        # âncora de PROGRESSO: re-ancora quando o robô sai do raio de deslocamento
        # OU gira mais que stuck_yaw (point-turn legítimo = progresso, não trava).
        # Micro-mexidas (ruído de odom) não resetam. Travado de verdade = nem
        # desloca nem gira -> o timer acumula e a recovery dispara.
        if (self.anchor is None
                or self._dist(position, self.anchor) > self.cfg.stuck_radius
                or abs(_norm_angle(yaw - self.anchor_yaw)) > self.cfg.stuck_yaw):
            self.anchor = position
            self.anchor_yaw = yaw
            self.anchor_t = now
            return _IDLE
        # dispara a recovery: pelo timeout cheio (obstáculo novo/desconhecido)
        # OU, se o bloqueio à frente é parede MAPEADA confirmada por
        # stuck_timeout_mapped contínuos, bem antes. O guard `stuck >=
        # stuck_timeout_mapped` evita disparar logo após re-ancorar com um
        # mapped_since antigo.
        stuck = now - self.anchor_t
        mapped_fire = (self.mapped_since is not None
                       and now - self.mapped_since >= self.cfg.stuck_timeout_mapped
                       and stuck >= self.cfg.stuck_timeout_mapped)
        # near_mapped (parede MAPEADA perto do robô, QUALQUER lado — ex. batente) deixa
        # chegar à decisão CEDO (>= stuck_timeout_mapped), igual ao mapped_fire faz pro
        # bloqueio à frente. Senão o 1º timeout (10s) seguraria e o "conhecido" não
        # agilizaria nada (BO: defer demorava demais perto do batente).
        near_fire = near_mapped and stuck >= self.cfg.stuck_timeout_mapped
        if stuck < self.cfg.stuck_timeout and not mapped_fire and not near_fire:
            return _IDLE
        # OPÇÃO A (2026-06-28 "vai bater de verdade?"): caminho livre à frente = a
        # parada provavelmente não é obstáculo (collision freando uma manobra/giro, ou
        # alinhamento) -> DEFERE (dá tempo pro nav), mas NÃO zera o relógio: se passar
        # de front_clear_timeout travado mesmo assim (bloqueio lateral/no giro que o
        # front reto não vê), cai na recovery abaixo. (Giro legítimo já re-ancora via
        # stuck_yaw; isto é só pro caso de ficar REALMENTE preso.)
        # timeout do defer: CURTO se conhece o obstáculo (parede mapeada perto -> não é
        # surpresa, age rápido); LONGO se é desconhecido (pode ser pessoa/algo novo ->
        # cauteloso, dá tempo pro nav). "Conheço esse obstáculo?" (pedido 06-28).
        clear_timeout = (self.cfg.front_clear_timeout_mapped if near_mapped
                         else self.cfg.front_clear_timeout)
        if front_gap > self.cfg.front_clear and stuck < clear_timeout:
            return _IDLE
        # DIREÇÃO pela CENA (2026-06-28, "analisar se precisa ré ou ir reto"):
        # - FRENTE LIVRE e mesmo assim travou (preso de lado / no batente da porta):
        #   AVANÇA (passa o batente). Dar ré aqui desfazia o progresso e re-aproximava
        #   o batente = loop de ré (o BO que o dono viu). A frente livre É o caminho.
        # - FRENTE BLOQUEADA: ré PREFERIDA se há vão atrás (clássico: parede na frente);
        #   sem vão atrás mas frente parcial -> avança o que dá; encurralado -> segura.
        forward_target = min(self.cfg.forward_distance,
                             front_gap - self.cfg.front_stop_margin)
        if front_gap > self.cfg.front_clear:
            if forward_target >= self.cfg.forward_min:
                return self._begin_advance(now, position, forward_target)
            return _IDLE
        rear_target = min(self.cfg.reverse_distance,
                          rear_gap - self.cfg.rear_stop_margin)
        if rear_target >= self.cfg.reverse_min:
            return self._begin_reverse(now, position, open_side, rear_target)
        if forward_target >= self.cfg.forward_min:
            return self._begin_advance(now, position, forward_target)
        return _IDLE

    def _begin_reverse(self, now, position, open_side, target) -> Command:
        # escalada: conta travamentos recentes perto DESTE ponto; na 3ª
        # tentativa no mesmo lugar a ré reta não resolveu -> ré + GIRO FORTE.
        # A escalada vive só na ré (o avanço é o plano B simples).
        self.history = [(t, p) for (t, p) in self.history
                        if now - t <= self.cfg.escalate_window]
        self.history.append((now, position))
        nearby = sum(
            1 for (_, p) in self.history
            if self._dist(p, position) <= self.cfg.same_spot_radius)
        self.escalated = nearby >= self.cfg.escalate_after
        self.spin_side = 1 if open_side >= 0 else -1
        self.state = _REVERSING
        self.maneuver_start_t = now
        self.maneuver_start_pos = position
        self.reverse_target = target
        return Command(-self.cfg.reverse_speed, 0.0, True)

    def _begin_advance(self, now, position, target) -> Command:
        self.state = _ADVANCING
        self.maneuver_start_t = now
        self.maneuver_start_pos = position
        self.forward_target = target
        return Command(self.cfg.forward_speed, 0.0, True)

    def _spin_cmd(self) -> Command:
        speed = self.cfg.spin_speed
        if self.spin_side > 0:
            speed *= self.cfg.spin_left_boost  # esquerda escorrega: + força
        return Command(0.0, self.spin_side * speed, True)

    def _reversing(self, now, position, yaw, rear_gap) -> Command:
        if rear_gap <= self.cfg.rear_stop_margin:
            # Algo apareceu/entrou atrás DURANTE a ré (batida de 2026-06-11:
            # a checagem era só no disparo). STOP imediato e SEM giro — com
            # coisa colada atrás, girar varre as quinas pra cima dela.
            self.state = _GRACE
            self.grace_start = now
            return Command(0.0, 0.0, True)
        dist = self._dist(position, self.maneuver_start_pos)
        if (dist >= self.reverse_target
                or now - self.maneuver_start_t >= self.cfg.reverse_time_cap):
            if self.escalated:
                # recuou: agora GIRO FORTE no lugar pro lado mais livre —
                # muda o heading de verdade (arco em 30cm não virava)
                self.state = _SPINNING
                self.spin_start_t = now
                self.spin_start_yaw = yaw
                return self._spin_cmd()
            self.state = _GRACE
            self.grace_start = now
            # STOP explícito: o cmd_vel_to_wheels segura o último comando;
            # sem este zero o robô continuaria de ré até o nav2 publicar
            # de novo (que pode estar mudo, abortado/replanejando).
            return Command(0.0, 0.0, True)
        return Command(-self.cfg.reverse_speed, 0.0, True)

    def _advancing(self, now, position, front_gap) -> Command:
        if front_gap <= self.cfg.front_stop_margin:
            # algo entrou/apareceu na FRENTE durante o avanço -> STOP imediato.
            # É o "respeitar o collision" do escape: nunca avança em cima de
            # obstáculo (o canal unstuck_vel fura o collision monitor real).
            self.state = _GRACE
            self.grace_start = now
            return Command(0.0, 0.0, True)
        dist = self._dist(position, self.maneuver_start_pos)
        if (dist >= self.forward_target
                or now - self.maneuver_start_t >= self.cfg.forward_time_cap):
            self.state = _GRACE
            self.grace_start = now
            return Command(0.0, 0.0, True)  # STOP explícito (mesmo motivo da ré)
        return Command(self.cfg.forward_speed, 0.0, True)

    def _spinning(self, now, yaw) -> Command:
        # MALHA FECHADA: para pelo yaw MEDIDO, não por tempo — roda patinando
        # comanda 30° e entrega 5°; a IMU vê a virada real.
        turned = abs(_norm_angle(yaw - self.spin_start_yaw))
        if (turned >= self.cfg.spin_angle
                or now - self.spin_start_t >= self.cfg.spin_time_cap):
            self.state = _GRACE
            self.grace_start = now
            return Command(0.0, 0.0, True)  # STOP explícito
        return self._spin_cmd()

    def _grace(self, now) -> Command:
        if now - self.grace_start >= self.cfg.grace:
            self.state = _MONITORING
            self.anchor = None  # re-ancora na posição pós-manobra
        return _IDLE

    @staticmethod
    def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])


# ---- nó ROS (cola de I/O) --------------------------------------------------

def main(args=None):  # pragma: no cover - I/O glue, validado na bancada
    import json

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                           QoSDurabilityPolicy)
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    try:
        from nav2_msgs.msg import CollisionMonitorState
    except ImportError:
        CollisionMonitorState = None

    # STOP = 1 no enum do collision monitor — só pra LOG (não é mais gatilho)
    STOP_ACTION = 1
    # GoalStatus: ACCEPTED=1, EXECUTING=2, CANCELING=3 = goal ainda vivo
    ACTIVE_STATUSES = (1, 2, 3)

    class UnstuckSupervisorNode(Node):
        def __init__(self):
            super().__init__("unstuck_supervisor")
            p = self.declare_parameters("", [
                ("stuck_timeout", 10.0),
                # Recovery contextual (2026-06-22): obstáculo à frente que bate
                # no /map (parede mapeada) -> ré após stuck_timeout_mapped (não
                # os 10 s). block_range = front_gap acima disso não conta como
                # bloqueio à frente. Lookup no /map cru (sem inflação).
                ("stuck_timeout_mapped", 2.0),
                ("block_range", 0.5),
                ("map_occ_threshold", 65),
                # 0.22 (não 0.15): absorve o offset de registro pose↔mapa (~0.2 m
                # medido em campo 2026-06-22 — a leitura do LiDAR cai ~0.2 m antes
                # da parede do /map).
                ("map_neighborhood", 0.22),
                ("stuck_radius", 0.05),
                ("stuck_yaw", 0.15),
                ("reverse_distance", 0.30),
                ("reverse_speed", 0.25),
                ("reverse_time_cap", 6.0),
                ("grace", 2.0),
                ("nav_latch", 15.0),
                ("escalate_after", 3),
                ("same_spot_radius", 0.5),
                ("escalate_window", 120.0),
                ("spin_speed", 3.0),
                ("spin_angle", 0.44),
                ("spin_time_cap", 4.0),
                ("spin_left_boost", 1.4),
                # Geometria da ré (frame base_link): LiDAR no CENTRO do robô
                # (todos os sensores são centrais — confirmado 2026-06-11);
                # o vão é medido do PARA-CHOQUE traseiro (tail_x).
                ("rear_lidar_x", 0.0),
                ("rear_tail_x", -0.25),
                ("rear_half_width", 0.30),
                ("rear_stop_margin", 0.10),
                ("reverse_min", 0.10),
                # Escape pra frente (obstáculo atrás): para-choque dianteiro
                # em head_x=+0.25; corredor com a MESMA largura da ré.
                ("front_head_x", 0.25),
                ("forward_distance", 0.20),
                ("forward_speed", 0.15),
                ("forward_time_cap", 6.0),
                ("front_stop_margin", 0.10),
                ("forward_min", 0.10),
                ("front_clear", 0.40),
                ("front_clear_timeout", 15.0),
                ("front_clear_timeout_mapped", 3.0),
                ("mapped_near_radius", 0.35),
                ("scan_stale", 2.0),
                ("nav_move_lin", 0.01),
                ("nav_move_ang", 0.05),
                ("rate_hz", 10.0),
            ])
            g = {n.name: n.value for n in p}
            self.cfg = UnstuckConfig(
                stuck_timeout=g["stuck_timeout"],
                stuck_timeout_mapped=g["stuck_timeout_mapped"],
                stuck_radius=g["stuck_radius"],
                stuck_yaw=g["stuck_yaw"],
                reverse_distance=g["reverse_distance"],
                reverse_speed=g["reverse_speed"],
                reverse_time_cap=g["reverse_time_cap"],
                grace=g["grace"],
                nav_latch=g["nav_latch"],
                escalate_after=int(g["escalate_after"]),
                same_spot_radius=g["same_spot_radius"],
                escalate_window=g["escalate_window"],
                spin_speed=g["spin_speed"],
                spin_angle=g["spin_angle"],
                spin_time_cap=g["spin_time_cap"],
                spin_left_boost=g["spin_left_boost"],
                rear_stop_margin=g["rear_stop_margin"],
                reverse_min=g["reverse_min"],
                forward_distance=g["forward_distance"],
                forward_speed=g["forward_speed"],
                forward_time_cap=g["forward_time_cap"],
                front_stop_margin=g["front_stop_margin"],
                forward_min=g["forward_min"],
                front_clear=g["front_clear"],
                front_clear_timeout=g["front_clear_timeout"],
                front_clear_timeout_mapped=g["front_clear_timeout_mapped"],
                mapped_near_radius=g["mapped_near_radius"],
            )
            self.rear_lidar_x = g["rear_lidar_x"]
            self.rear_tail_x = g["rear_tail_x"]
            self.rear_half_width = g["rear_half_width"]
            self.front_head_x = g["front_head_x"]
            self.block_range = g["block_range"]
            self.map_occ_threshold = g["map_occ_threshold"]
            self.map_neighborhood = g["map_neighborhood"]
            self.mapped_near_radius = g["mapped_near_radius"]  # usado no _tick (near_mapped)
            self.scan_stale = g["scan_stale"]
            self.nav_move_lin = g["nav_move_lin"]
            self.nav_move_ang = g["nav_move_ang"]

            self.sup = UnstuckSupervisor(self.cfg)

            self._nav_wants_move = False
            self._position = (0.0, 0.0)
            self._yaw = 0.0
            self._rear_gap = math.inf
            self._front_gap = math.inf
            self._open_side = 1  # +1 esq / -1 dir (lado mais livre na frente)
            self._scan_t = None  # quando o último /scan chegou
            self._goal_active = {}  # por tópico de status; None até a 1ª msg
            self._stop_active = False  # só pra log
            self._door_active = False  # door_crossing conduzindo? -> standdown
            self._map = None           # MapGrid do /map estático (None até a 1ª msg)
            self._front_bp = None      # ponto de contato à frente (x,y base_link)
            self._near_r = math.inf    # DEBUG: retorno LiDAR mais próximo (m)
            self._near_deg = 0.0       # DEBUG: ângulo desse retorno (graus)
            self._dbg_t = 0.0          # DEBUG: throttle do log
            self._last_state = self.sup.state

            be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
            # /door_zone é latched (door_crossing publica TRANSIENT_LOCAL): casa
            # a QoS pra pegar o estado atual já no boot.
            latched = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST)

            self.pub = self.create_publisher(Twist, "unstuck_vel", 10)
            self.create_subscription(Odometry, "odom", self._on_odom, 10)
            # 2026-06-26 (2-mux): "nav_vel_raw" virou "nav_vel" (saída do smoother).
            # É o MESMO sinal de antes (intenção do controller, pré-collision); só o
            # nome mudou — o collision saiu de cima do smoother e foi pro mux de
            # autonomia. Continua sendo a intenção do nav p/ o gate _nav_wants_move.
            self.create_subscription(Twist, "nav_vel", self._on_nav_raw, 10)
            self.create_subscription(LaserScan, "scan", self._on_scan, be)
            # Standdown durante a travessia de porta: enquanto o door_crossing
            # está staging/rotating/crossing, o unstuck fica quieto (senão a ré
            # prio 30 sabota a manobra prio 20 e a porta nunca fecha).
            self.create_subscription(
                String, "door_zone", self._on_door_zone, latched)
            # /map estático do SLAM (latched/transient_local): recovery
            # contextual — bloqueio à frente que bate aqui = parede mapeada.
            self.create_subscription(
                OccupancyGrid, "map", self._on_map, latched)
            # Status dos goals do bt_navigator: gate AUTORITATIVO (mata a
            # "ré póstuma" após cancel e cobre o BT em recovery c/ controller
            # mudo). Sem msg ainda -> cai no fallback por nav_latch.
            for topic in ("navigate_to_pose/_action/status",
                          "navigate_through_poses/_action/status"):
                self.create_subscription(
                    GoalStatusArray, topic,
                    lambda m, t=topic: self._on_goal_status(t, m), 10)
            if CollisionMonitorState is not None:
                self.create_subscription(
                    CollisionMonitorState, "collision_monitor_state",
                    self._on_collision, 10)

            self.create_timer(1.0 / g["rate_hz"], self._tick)
            self.get_logger().info(
                "unstuck_supervisor ativo (sem-deslocamento %.0fs -> ré %.2fm "
                "se há vão atrás, senão AVANÇA %.2fm se a frente livre; "
                "%dª vez no mesmo ponto -> ré + giro %.0f° pro lado livre)" % (
                    self.cfg.stuck_timeout, self.cfg.reverse_distance,
                    self.cfg.forward_distance, self.cfg.escalate_after,
                    math.degrees(self.cfg.spin_angle)))

        def _on_odom(self, msg):
            self._position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            q = msg.pose.pose.orientation
            self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        def _on_nav_raw(self, msg):
            self._nav_wants_move = (abs(msg.linear.x) > self.nav_move_lin
                                    or abs(msg.angular.z) > self.nav_move_ang)

        def _on_scan(self, msg):
            # time.monotonic(): freshness local, sem criar rclpy.time.Time a
            # 10 Hz nem depender de NTP (P3 da AUDITORIA_2026-06-11). O update()
            # da lógica pura só usa diferenças, então a base monotônica serve.
            self._scan_t = time.monotonic()
            ranges = np.asarray(msg.ranges, dtype=np.float64)
            self._rear_gap = rear_min_gap(
                ranges, msg.angle_min, msg.angle_increment,
                self.rear_lidar_x, self.rear_tail_x, self.rear_half_width)
            self._front_gap = front_min_gap(
                ranges, msg.angle_min, msg.angle_increment,
                self.rear_lidar_x, self.front_head_x, self.rear_half_width)
            self._open_side = freer_side(
                ranges, msg.angle_min, msg.angle_increment)
            # ponto de contato à frente (base_link) pra recovery contextual:
            # a "parte que travou", com o offset lateral real (não reto à frente).
            self._front_bp = front_block_point(
                ranges, msg.angle_min, msg.angle_increment,
                self.rear_lidar_x, self.front_head_x, self.rear_half_width)
            # DEBUG temporário (2026-06-22): retorno mais próximo (dist+ângulo).
            finite = np.isfinite(ranges) & (ranges > 0.0)
            if finite.any():
                i = int(np.argmin(np.where(finite, ranges, np.inf)))
                self._near_r = float(ranges[i])
                self._near_deg = math.degrees(
                    _norm_angle(msg.angle_min + i * msg.angle_increment))

        def _on_goal_status(self, topic, msg):
            self._goal_active[topic] = any(
                s.status in ACTIVE_STATUSES for s in msg.status_list)

        def _on_collision(self, msg):
            self._stop_active = (getattr(msg, "action_type", 0) == STOP_ACTION)

        def _on_door_zone(self, msg):
            try:
                st = json.loads(msg.data).get("state", "idle")
            except (ValueError, AttributeError):
                st = "idle"
            self._door_active = door_zone_active(st)

        def _on_map(self, msg):
            self._map = MapGrid(
                data=msg.data, width=msg.info.width, height=msg.info.height,
                resolution=msg.info.resolution,
                origin_x=msg.info.origin.position.x,
                origin_y=msg.info.origin.position.y)

        def _obstacle_mapped(self):
            """True se o ponto de contato à frente coincide com parede MAPEADA.
            Usa o (x,y) REAL do contato (front_block_point) com o offset lateral,
            transformado pro frame do mapa — não a projeção reto à frente."""
            return block_point_mapped(
                self._map, self._position, self._yaw, self._front_bp,
                self.front_head_x, self.block_range, self.map_neighborhood,
                self.map_occ_threshold)

        def _tick(self):
            now = time.monotonic()
            # scan velho (LiDAR caiu?) -> trata traseira como BLOQUEADA
            # (vão zero): melhor segurar a ré do que dar ré cego.
            scan_fresh = (self._scan_t is not None
                          and now - self._scan_t <= self.scan_stale)
            gap = self._rear_gap if scan_fresh else 0.0
            # scan velho -> frente também BLOQUEADA (não avança cego, igual à ré)
            front_gap = self._front_gap if scan_fresh else 0.0
            # status visto em algum tópico? OR entre eles; nunca visto -> None
            goal_active = (any(self._goal_active.values())
                           if self._goal_active else None)
            obstacle_mapped = self._obstacle_mapped() if scan_fresh else False
            # "conheço esse obstáculo?": há parede MAPEADA perto do robô (qualquer
            # lado — ex. batente da porta). Se sim, a recovery age mais RÁPIDO (não é
            # surpresa). Pedido do dono 2026-06-28 (defer de 15s tava demorando).
            near_mapped = (self._map is not None and map_occupied(
                self._map, self._position[0], self._position[1],
                self.mapped_near_radius, self.map_occ_threshold))
            cmd = self.sup.update(
                now, nav_wants_move=self._nav_wants_move,
                position=self._position, rear_gap=gap, front_gap=front_gap,
                goal_active=goal_active, open_side=self._open_side,
                yaw=self._yaw, door_active=self._door_active,
                obstacle_mapped=obstacle_mapped, near_mapped=near_mapped)
            # DEBUG temporário (2026-06-22): diagnóstico da recovery contextual.
            if (self.sup.state == "monitoring" and self._nav_wants_move
                    and now - self._dbg_t >= 1.0):
                self._dbg_t = now
                if self._front_bp is not None:
                    bx_r, by_r = self._front_bp
                    c, s = math.cos(self._yaw), math.sin(self._yaw)
                    mx = self._position[0] + bx_r * c - by_r * s
                    my = self._position[1] + bx_r * s + by_r * c
                else:
                    mx = my = float('nan')
                self.get_logger().warn(
                    "DBG recov: front_gap=%.2f map=%s mapped=%s near=%.2fm@%.0f° "
                    "blk=(%.2f,%.2f) anchor_t=%.1f" % (
                        front_gap, self._map is not None, obstacle_mapped,
                        self._near_r, self._near_deg, mx, my,
                        now - self.sup.anchor_t if self.sup.anchor else -1))
            if self.sup.state != self._last_state:
                self.get_logger().warn(
                    "unstuck: %s -> %s (pos=%.2f,%.2f stop=%s vao_re=%.2f)" % (
                        self._last_state, self.sup.state,
                        self._position[0], self._position[1],
                        self._stop_active, gap))
                self._last_state = self.sup.state
            if cmd.active:
                t = Twist()
                t.linear.x = cmd.lin
                t.angular.z = cmd.ang
                self.pub.publish(t)

    from .utils import spin_node

    rclpy.init(args=args)
    node = UnstuckSupervisorNode()
    try:
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
