#!/usr/bin/env python3
"""motion_guard — cautela com objeto EM MOVIMENTO perto do robô.

Por que existe (pedido do dono pós-run 2026-07-02): nada na stack distingue
móvel de estático — o collision_monitor é reativo instantâneo (freia quando
algo JÁ está na frente). Este nó compara scans no frame ODOM: o que é
estático (parede, móvel parado) fica na mesma célula; célula que estava LIVRE
~0.5s atrás e agora tem retorno = borda de ataque de coisa se movendo.

Atuação (filtro de velocidade, só autonomia):
    twist_mux_auto -> auto_vel_pre -> [motion_guard] -> auto_vel_raw
        -> collision_monitor -> auto_vel -> mux final
  - móvel no raio guard_radius  -> linear.x escala pela distância (slowing);
    angular.z passa INTOCADO (continua navegando/girando perto de gente)
  - móvel no corredor à frente  -> PARADA TOTAL vx=0 E wz=0 até limpar
    clear_time (blocked). wz zerado a pedido do dono 07-02: com wz liberado o
    replan balançava o caminho e o robô GIRAVA no lugar enquanto a pessoa
    passava. NUNCA escalar wz parcialmente (zona-morta 1.7 = comando fraco
    que não gira); zerar é seguro.
  - TF/scan indisponível ou enabled=false -> PASS-THROUGH (nunca mata a nav).
  - FILTRO ANTI-VIDRO (07-08): ponto "móvel" cuja linha de visão robô->ponto
    cruza PAREDE do mapa estático = gente vista ATRAVÉS do vidro (29/52
    paradas da run hotmilk eram isso) -> descartado antes de latchar. Sem
    /map ou sem TF map<-odom o filtro fica inerte (failsafe).

SEM predição de cruzamento por enquanto (proposta B da spec): os pontos
móveis já saem clusterizados pra plugar velocidade+predição depois se a
versão A reagir tarde em campo.

Spec: docs/superpowers/specs/2026-07-02-motion-guard-design.md
A lógica (MotionGuard) é pura p/ testar sem ROS; main() é a cola de I/O.
"""
import json
import math
import os
from collections import deque
from dataclasses import dataclass
from typing import List, Tuple

Pt = Tuple[float, float]


@dataclass
class GuardConfig:
    enabled: bool = True
    guard_radius: float = 3.5       # m — só olha móvel até aqui (2.5->3.5 dono
                                    # 07-09: "aumentar o raio do medo", enxergar
                                    # quem se aproxima mais cedo e desacelerar
                                    # numa faixa maior; anti-vidro compensa o
                                    # falso positivo que a distância extra traria)
    slow_scale: float = 0.25        # PISO do fator no vx (móvel colado)
    slow_dist: float = 0.6          # m — abaixo disso o fator satura no piso
                                    # (entre slow_dist e guard_radius a escala
                                    # sobe linear até 1.0: perto=lento, longe=
                                    # quase cheio — feedback do dono 07-02:
                                    # 50% uniforme era imperceptível de lado)
    corridor_half_w: float = 0.35   # m — meia-largura do corredor à frente
    corridor_len: float = 2.5       # m — alcance do corredor (1.5→2.5 dono
                                    # 07-02: cruzava o caminho ALÉM do corredor
                                    # e o follower saía atrás do desvio)
    freeze_dist: float = 1.2        # m — BOLHA: móvel mais perto que isso em
                                    # QUALQUER direção = parada total (dono
                                    # 07-02: do lado, o giro liberado rodava
                                    # atrás do plano-contorno; "para de pensar")
    clear_time: float = 5.0         # s — limpo por isso -> retoma (1.5→3.0
                                    # dono 07-02: gap p/ ~3 replans do nav2
                                    # endireitarem o plano antes de andar;
                                    # 3.0→5.0 dono 07-09: se é gente ele deve
                                    # SEMPRE esperar mais antes de voltar a andar)
    grid_res: float = 0.15          # m — célula da grade de comparação
    lookback: float = 0.5           # s — compara com snapshot desta idade
    min_cluster_points: int = 3     # cluster menor = ruído
    persist_frames: int = 3         # scans CONSECUTIVOS c/ móvel p/ latchar.
                                    # Campo 07-03: TF atrasado + borda de
                                    # oclusão "piscam" parede mapeada como
                                    # móvel (62% dos falsos = 1 frame, 81%
                                    # <=3) -> guard vivia preso em slowing/
                                    # blocked sem ninguém perto. Custo: ~0.3s
                                    # de latência a 10Hz (pessoa real dispara
                                    # todo frame; detecção começa a 2.5m).
    cluster_gap: float = 0.3        # m — distância máx p/ mesmo cluster
    wz_gate: float = 0.3            # rad/s — girando acima disso não avalia
    ray_bin_deg: float = 1.0        # ° — bin do mapa polar do raycast (visto
                                    # do pose antigo). Campo 07-03 (2ª rodada):
                                    # "célula ausente" ≠ "estava livre" —
                                    # trecho de parede saindo da SOMBRA de um
                                    # objeto disparava móvel sustentado (71%
                                    # do tempo freado sem ninguém perto). Só é
                                    # móvel se o feixe velho ATRAVESSOU a
                                    # célula (alcance > dist + grid_res).
    scan_stale: float = 1.0         # s sem scan -> pass-through
    map_filter: bool = True         # descarta móvel visto ATRAVÉS de parede do
                                    # mapa estático (LD06 enxerga gente pelo
                                    # VIDRO que no mapa é parede virtual). Run
                                    # hotmilk 07-08: 29/52 paradas eram isso
                                    # (50.8s freado à toa vs 29.7s por gente
                                    # real). Sem /map ou sem TF map<-odom o
                                    # filtro simplesmente não atua (failsafe).
    hold_still_max: float = 20.0    # s — VIGÍLIA (dono 07-10): móvel que
                                    # BLOQUEOU (bolha/corredor) e PAROU segue
                                    # segurando o blocked enquanto o scan
                                    # mostrar ocupação no lugar dele, até este
                                    # teto; saiu -> solta pelo clear_time.
                                    # Antes: pessoa parada sumia do diff em ~1s
                                    # e o robô voltava a empurrar pra cima dela.
    hold_still_radius: float = 0.5  # m — raio da vigília em volta do centróide
                                    # do móvel parado (cobre pé trocado/balanço)
    wall_near: float = 0.15         # m — ponto do scan a <isto de parede do
                                    # MAPA não conta como presença (senão
                                    # pessoa que parou perto de parede prendia
                                    # a vigília até o teto depois de sair)
    slow_wz_cap: float = 2.4        # teto do |wz| no slowing (dono 07-10: o
                                    # robô girava a 4.0-4.5 do lado de gente).
                                    # CAP, nunca escala (zona-morta do skid:
                                    # comando fraco não gira) — 2.4 fica acima
                                    # da zona morta (~1.7-1.9) e dá ~0.4rad/s
                                    # reais (spin_calib 06-19). blocked segue
                                    # zerando tudo.
    wall_ghost_frac: float = 0.8    # fração dos pontos do cluster EM CIMA de
                                    # parede do MAPA p/ descartar como
                                    # FANTASMA DE PAREDE (campo 07-10: corredor
                                    # reto, transladando a ~0.37m/s, trecho da
                                    # parede virava "móvel" a <1m e parava SECO
                                    # — o cluster acompanhava o robô). Pessoa
                                    # encostada na parede: o corpo sobra fora
                                    # (frac < limiar) -> mantém.
    settle_enabled: bool = True     # soltar o blocked por PLANO ASSENTADO e
                                    # não só pelo relógio (bug da curva ~70° ao
                                    # retomar pós-blocked, repro no sim 07-20):
                                    # no fim do clear_time o global plan ainda
                                    # nasce CONTORNANDO a pessoa que segue no
                                    # costmap; o robô arranca comprometido com
                                    # um desvio que morre em ~2s. Enquanto o
                                    # rumo do plano balança, ESTENDE o blocked
                                    # (robô parado). False = pré-07-20 exato.
    settle_window: float = 1.0      # s — janela deslizante do rumo do plano
    settle_tol_deg: float = 8.0     # ° — AMPLITUDE (máx-mín) na janela abaixo
                                    # disso = assentado. ~metade do erro de
                                    # mira medido no release ruim (15-19°).
    settle_max: float = 4.0         # s — teto do settling desde o fim do
                                    # clear_time (curto: o settling só ESTENDE o
                                    # blocked por alguns ciclos até o plano
                                    # assentar).
    settle_min_samples: int = 3     # não declarar assentado com 1 amostra solta
    settle_plan_stale: float = 1.0  # s sem /plan fresco -> libera (fail-open)
    settle_lookahead: float = 0.6   # m — arco do início do plano até o ponto
                                    # que define o rumo (mira além do 1º passo)


class MapGhostFilter:
    """Mapa estático (OccupancyGrid) p/ caçar FANTASMA DE VIDRO: o LD06 enxerga
    gente através de vidro/vão que no mapa é PAREDE (virtual ou real). Se a
    linha de visão robô->ponto cruza parede do mapa, o retorno não pode ser
    coisa alcançável — é reflexo/atravessou vidro -> descarta ANTES de latchar.

    Só descarta com parede FRANCA no meio do caminho (>= min_wall_hits amostras
    consecutivas ocupadas, ignorando os end_margin finais): pessoa real ENCOSTADA
    numa parede mapeada não some (o raio até ela corre no livre), e raspão de
    quina por jitter do AMCL não conta como travessia."""

    def __init__(self, grid, width: int, height: int, res: float,
                 ox: float, oy: float, occ_thresh: int = 65,
                 min_wall_hits: int = 2, end_margin: float = 0.2):
        self.grid = grid            # row-major, linha 0 = y do origin (padrão ROS)
        self.w, self.h = width, height
        self.res, self.ox, self.oy = res, ox, oy
        self.occ_thresh = occ_thresh
        self.min_wall_hits = min_wall_hits
        self.end_margin = end_margin

    def _occupied(self, x: float, y: float) -> bool:
        cx = int((x - self.ox) / self.res)
        cy = int((y - self.oy) / self.res)
        if cx < 0 or cx >= self.w or cy < 0 or cy >= self.h:
            return False            # fora do mapa != parede (unknown tb não)
        return self.grid[cy * self.w + cx] >= self.occ_thresh

    def occupied_near(self, x: float, y: float, r: float) -> bool:
        """Há parede do mapa a até r do ponto? Varre as células no quadrado
        de lado 2r em passos de res (não perde parede entre amostras)."""
        n = max(1, int(math.ceil(r / self.res)))
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                if self._occupied(x + i * self.res, y + j * self.res):
                    return True
        return False

    def sees_through_wall(self, a: Pt, b: Pt) -> bool:
        """True se o segmento a->b (frame MAP) atravessa parede do mapa."""
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if d <= self.end_margin:
            return False
        step = self.res / 2.0
        n = max(int(d / step), 1)
        hits = 0
        for i in range(1, n):
            t = i / n
            if d * (1.0 - t) < self.end_margin:
                break               # ponta final: pode ser gente colada na parede
            if self._occupied(a[0] + (b[0] - a[0]) * t,
                              a[1] + (b[1] - a[1]) * t):
                hits += 1
                if hits >= self.min_wall_hits:
                    return True
            else:
                hits = 0            # exige travessia CONSECUTIVA (raspão não vale)
        return False


class MotionGuard:
    """Detector de movimento por diff temporal em grade (frame odom).

    observe() processa um scan; filter() aplica a decisão no comando.
    """

    def __init__(self, cfg: GuardConfig):
        self.cfg = cfg
        self._snaps = deque()   # (t, células, (px,py) da pose, polar bin->alcance)
        self.moving_clusters: List[List[Pt]] = []
        self.nearest_moving: float = math.inf
        self.in_corridor: bool = False
        self._last_moving_t: float = -math.inf
        self._last_nearest: float = math.inf   # dist do móvel na última vista
        self._last_corridor_t: float = -math.inf
        self._last_scan_t: float = -math.inf
        self._consec: int = 0       # scans consecutivos vendo móvel
        # filtro anti-vidro (setados pelo nó quando /map + TF map<-odom
        # existem; None = filtro inerte, comportamento pré-07-08)
        self.ghost_map: 'MapGhostFilter|None' = None
        self.map_tf = None          # (tx, ty, cos, sin) odom->map
        self.ghost_dropped: int = 0  # pts descartados no último observe (CSV)
        self.wall_dropped: int = 0   # clusters descartados como fantasma de
                                     # parede no último observe (CSV)
        # vigília do "parou-mas-está-lá" (dono 07-10)
        self._watch: List[Pt] = []          # centróides vigiados (frame odom)
        self._watch_since: float = -math.inf
        self._watch_corridor: bool = False
        # assentamento do plano (07-20): rumo do início do /plan numa janela
        # deslizante; o release pós-blocked espera a amplitude cair.
        self._plan_hdg = deque()            # (t, rumo em frame map, rad)
        self._last_plan_t: float = -math.inf
        self._was_blocked: bool = False     # esteve em blocked desde o último idle
        self._settle_since: float = -math.inf   # t em que o clear_time venceu

    def _cell(self, p: Pt) -> Tuple[int, int]:
        r = self.cfg.grid_res
        return (int(math.floor(p[0] / r)), int(math.floor(p[1] / r)))

    def _old_snapshot(self, t: float):
        """último snapshot com idade >= lookback (descarta os mais velhos)."""
        c = self.cfg
        old = None
        while self._snaps and t - self._snaps[0][0] >= c.lookback:
            old = self._snaps.popleft()
        if old is not None:
            self._snaps.appendleft(old)   # mantém p/ os próximos ticks
        return old

    def observe(self, t: float, pts: List[Pt],
                pose: Tuple[float, float, float], wz: float,
                polar=None) -> None:
        c = self.cfg
        self._last_scan_t = t
        # GATE DE GIRO (movido pra ANTES do snapshot — 8ª auditoria A4):
        # girando, pose/TF atrasam e a nuvem projetada sai BORRADA (medido
        # 06-30: tf_fallback 100%, p99 222ms). Antes o gate só pulava a
        # AVALIAÇÃO, mas o snapshot borrado entrava no deque e virava a
        # referência "old" dos primeiros ~0.5s pós-giro -> células/polar
        # erradas -> falso móvel logo depois de girar. Agora girando não
        # avalia NEM snapshotta; a referência pós-giro é o último snapshot
        # PRÉ-giro (limpo, mais velho que lookback — o _old_snapshot guarda).
        # A decisão anterior decai sozinha (clear_time no filter).
        if abs(wz) > c.wz_gate:
            return
        cells = frozenset(self._cell(p) for p in pts)
        px, py, pyaw = pose
        binw = math.radians(c.ray_bin_deg)
        # mapa polar visto DESTA pose (bin de bearing -> maior alcance): é o
        # "o que o feixe atravessou" que o raycast do futuro consulta.
        # PREFERIR o polar do nó (scan COMPLETO, dropout=0.0=desconhecido —
        # campo 07-03: feixe rasante do LD06 some e volta; sem isso a volta
        # parecia móvel). Fallback (testes/sem nó): monta dos próprios pts
        # (bin sem feixe = livre).
        if polar is None:
            polar = {}
            for p in pts:
                b = int(math.floor(math.atan2(p[1] - py, p[0] - px) / binw))
                d = math.hypot(p[0] - px, p[1] - py)
                if d > polar.get(b, 0.0):
                    polar[b] = d
        self._snaps.append((t, cells, (px, py), polar))

        old = self._old_snapshot(t)
        if old is None:
            return                      # histórico curto demais ainda
        _, old_cells, (opx, opy), old_polar = old

        # filtro anti-vidro (07-08): pose do robô no frame MAP, 1x por scan
        ghost_ready = (c.map_filter and self.ghost_map is not None
                       and self.map_tf is not None)
        if ghost_ready:
            tx, ty, tc, ts = self.map_tf
            robot_map = (tx + tc * px - ts * py, ty + ts * px + tc * py)
        self.ghost_dropped = 0

        r2 = c.guard_radius ** 2
        moving: List[Pt] = []
        for p in pts:
            if (p[0] - px) ** 2 + (p[1] - py) ** 2 > r2:
                continue
            cx, cy = self._cell(p)
            # célula (ou vizinha imediata) ocupada antes -> estático
            if any((cx + dx, cy + dy) in old_cells
                   for dx in (-1, 0, 1) for dy in (-1, 0, 1)):
                continue
            # RAYCAST (07-03): célula ausente ≠ célula LIVRE — pode só estar
            # na sombra de um objeto (oclusão) ou fora do alcance cortado.
            # Móvel de verdade = o feixe velho ATRAVESSAVA essa célula (alcance
            # no bin > distância + margem). Bin sem feixe = livre (feixe foi
            # além do corte de alcance) -> mantém a detecção.
            d_old = math.hypot(p[0] - opx, p[1] - opy)
            b = int(math.floor(math.atan2(p[1] - opy, p[0] - opx) / binw))
            if old_polar.get(b, math.inf) <= d_old + c.grid_res:
                continue                # oclusão/superfície, não movimento
            # ANTI-VIDRO (07-08): linha de visão robô->ponto cruza parede do
            # MAPA -> é gente atrás do vidro (29/52 paradas da run hotmilk),
            # não obstáculo alcançável. Descarta ANTES de clusterizar/latchar.
            if ghost_ready:
                p_map = (tx + tc * p[0] - ts * p[1],
                         ty + ts * p[0] + tc * p[1])
                if self.ghost_map.sees_through_wall(robot_map, p_map):
                    self.ghost_dropped += 1
                    continue
            moving.append(p)

        clusters = [cl for cl in self._cluster(moving)
                    if len(cl) >= c.min_cluster_points]
        # FANTASMA DE PAREDE (campo 07-10, corredor reto do hotmilk): feixe
        # rasante + erro de pose transladando rápido faz trecho da PAREDE
        # cair em bin "livre 0.5s atrás" -> "móvel" a <1m -> bolha -> parada
        # SECA repetida (o cluster ACOMPANHAVA o robô, colado na parede).
        # Cluster com >= wall_ghost_frac dos pontos em cima de parede MAPEADA
        # não é gente -> descarta ANTES de latchar. Pessoa encostada na
        # parede sobra fora da linha (frac < limiar) e o collision monitor
        # cobre o resto. Failsafe: sem mapa/TF o filtro não atua.
        self.wall_dropped = 0
        if ghost_ready and clusters:
            kept = []
            for cl in clusters:
                on_wall = sum(
                    1 for p in cl if self.ghost_map.occupied_near(
                        tx + tc * p[0] - ts * p[1],
                        ty + ts * p[0] + tc * p[1], c.wall_near))
                if on_wall >= c.wall_ghost_frac * len(cl):
                    self.wall_dropped += 1
                else:
                    kept.append(cl)
            clusters = kept
        self.moving_clusters = clusters
        self.nearest_moving = min(
            (math.hypot(p[0] - px, p[1] - py) for cl in clusters for p in cl),
            default=math.inf)

        # corredor à frente em base_link: xb à frente, yb lateral
        cos_y, sin_y = math.cos(pyaw), math.sin(pyaw)
        self.in_corridor = False
        for cl in clusters:
            for p in cl:
                dx, dy = p[0] - px, p[1] - py
                xb = dx * cos_y + dy * sin_y
                yb = -dx * sin_y + dy * cos_y
                if 0.0 < xb <= c.corridor_len and abs(yb) <= c.corridor_half_w:
                    self.in_corridor = True
                    break
            if self.in_corridor:
                break
        # PERSISTÊNCIA: só latcha com persist_frames scans consecutivos vendo
        # móvel — 1 frame isolado é flicker de TF atrasado/borda de oclusão
        # (falso positivo de campo 07-03), não pessoa. Frame limpo zera.
        self._consec = self._consec + 1 if clusters else 0
        if self._consec >= c.persist_frames:
            self._last_moving_t = t
            self._last_nearest = self.nearest_moving
            if self.in_corridor:
                self._last_corridor_t = t
            # VIGÍLIA (dono 07-10): móvel que BLOQUEOU (bolha/corredor) ganha
            # vigília no lugar — pessoa que para de se mexer não pode sumir
            # do radar em ~1s (o robô voltava a empurrar pra cima dela).
            if self.in_corridor or self.nearest_moving < c.freeze_dist:
                self._watch = [
                    (sum(p[0] for p in cl) / len(cl),
                     sum(p[1] for p in cl) / len(cl)) for cl in clusters]
                self._watch_since = t
                self._watch_corridor = self.in_corridor
        elif self._watch:
            if t - self._watch_since > c.hold_still_max:
                self._watch = []    # teto: solta a vigília, decai no clear_time
            else:
                d = self._presence(pts, (px, py))
                if d is None:
                    self._watch = []    # saiu: decai no clear_time e retoma
                else:
                    self._last_moving_t = t     # ainda LÁ: renova o latch
                    self._last_nearest = d
                    if self._watch_corridor:
                        self._last_corridor_t = t

    def _presence(self, pts: List[Pt], robot: Pt) -> 'float|None':
        """Alguém AINDA está no lugar vigiado? Menor distância robô->ponto
        dentro do raio da vigília, ignorando retornos colados em parede do
        MAPA (pessoa saiu, sobrou parede -> não é presença). None = vazio."""
        c = self.cfg
        ghost_ready = (c.map_filter and self.ghost_map is not None
                       and self.map_tf is not None)
        if ghost_ready:
            tx, ty, tc, ts = self.map_tf
        r2 = c.hold_still_radius ** 2
        best = None
        for p in pts:
            if not any((p[0] - wx) ** 2 + (p[1] - wy) ** 2 <= r2
                       for wx, wy in self._watch):
                continue
            if ghost_ready and self.ghost_map.occupied_near(
                    tx + tc * p[0] - ts * p[1],
                    ty + ts * p[0] + tc * p[1], c.wall_near):
                continue            # parede mapeada, não pessoa
            d = math.hypot(p[0] - robot[0], p[1] - robot[1])
            if best is None or d < best:
                best = d
        return best

    def filter(self, t: float, vx: float, wz: float
               ) -> Tuple[float, float, str]:
        """aplica a decisão no comando. wz nunca é ESCALADO (zona-morta do
        giro); no blocked ele é ZERADO junto (parada total). Os latches
        expiram sozinhos pelo relógio (clear_time) — cobre também o
        decaimento durante o gate de giro (gated não re-avista o móvel)."""
        c = self.cfg
        if not c.enabled or t - self._last_scan_t > c.scan_stale:
            return vx, wz, 'passthrough'
        freeze = (t - self._last_moving_t < c.clear_time
                  and self._last_nearest < c.freeze_dist)
        if freeze or t - self._last_corridor_t < c.clear_time:
            # parada TOTAL: wz TAMBÉM zera (dono 07-02: com wz liberado o
            # replan do nav2 balançava o caminho e o robô girava no lugar
            # enquanto a pessoa ainda passava). Zerar é seguro — o perigo da
            # zona-morta é ESCALAR wz (comando fraco que não gira), não zerar.
            # Ré (vx<0, afasta do móvel à frente) continua passando.
            self._was_blocked = True
            self._settle_since = -math.inf      # o relógio ainda nem venceu
            return (0.0 if vx > 0.0 else vx), 0.0, 'blocked'
        # o clear_time venceu. Antes de arrancar, o plano tem que estar
        # ASSENTADO — senão o robô sai comprometido com o contorno da pessoa
        # que ainda está no costmap (curva ~70° do bug de 07-20). Enquanto o
        # rumo balança, ESTENDE o blocked (só para; ré ainda passa), com teto
        # settle_max. fail-open: sem plano/settle_enabled=False cai no de hoje.
        if self._was_blocked and c.settle_enabled:
            if self._settle_since == -math.inf:
                self._settle_since = t
            if (t - self._settle_since < c.settle_max
                    and not self._plan_settled(t)):
                return (0.0 if vx > 0.0 else vx), 0.0, 'settling'
        self._was_blocked = False
        self._settle_since = -math.inf
        if t - self._last_moving_t < c.clear_time:
            # escala PROPORCIONAL à distância do móvel: colado (<=slow_dist)
            # freia no piso slow_scale; na borda do raio quase não freia.
            span = max(c.guard_radius - c.slow_dist, 1e-6)
            k = min(1.0, max(0.0, (self._last_nearest - c.slow_dist) / span))
            # giro CALMO perto de gente (dono 07-10): CAP no |wz| (nunca
            # escala — zona-morta), o vx proporcional continua como era.
            wz_cap = max(-c.slow_wz_cap, min(c.slow_wz_cap, wz))
            return vx * (c.slow_scale + (1.0 - c.slow_scale) * k), wz_cap, \
                'slowing'
        return vx, wz, 'idle'

    def observe_plan(self, t: float, poses: List[Pt]) -> None:
        """rumo do início do plano global (frame map) na janela deslizante.

        Rumo ABSOLUTO de propósito: relativo ao robô, o giro do próprio robô
        entraria na medida e um plano parado pareceria instável.
        """
        c = self.cfg
        if len(poses) < 2:
            return
        self._last_plan_t = t
        x0, y0 = poses[0]
        tip = poses[-1]
        acc = 0.0
        for a, b in zip(poses, poses[1:]):
            acc += math.hypot(b[0] - a[0], b[1] - a[1])
            if acc >= c.settle_lookahead:
                tip = b
                break
        dx, dy = tip[0] - x0, tip[1] - y0
        if math.hypot(dx, dy) < 1e-6:
            return          # plano degenerado (robô em cima do goal)
        self._plan_hdg.append((t, math.atan2(dy, dx)))
        while self._plan_hdg and t - self._plan_hdg[0][0] > c.settle_window:
            self._plan_hdg.popleft()

    def _plan_settled(self, t: float) -> bool:
        """AMPLITUDE (máx-mín, wrap ±π) na janela < settle_tol_deg.

        Amplitude e não delta entre replans: um plano que gira devagar e
        constante tem delta pequeno a cada ciclo e amplitude grande — é
        exatamente o caso da pessoa saindo ANDANDO, o pior do bug.
        Todo caminho de dúvida devolve True (fail-open: settling só PARA).
        """
        c = self.cfg
        if t - self._last_plan_t > c.settle_plan_stale:
            return True                     # sem plano fresco -> libera
        while self._plan_hdg and t - self._plan_hdg[0][0] > c.settle_window:
            self._plan_hdg.popleft()
        if len(self._plan_hdg) < c.settle_min_samples:
            return True
        ref = self._plan_hdg[0][1]
        rel = [(h - ref + math.pi) % (2 * math.pi) - math.pi
               for _, h in self._plan_hdg]
        return (max(rel) - min(rel)) <= math.radians(c.settle_tol_deg)

    def _cluster(self, pts: List[Pt]) -> List[List[Pt]]:
        """agrupamento single-link por distância <= cluster_gap (N pequeno)."""
        gap2 = self.cfg.cluster_gap ** 2
        clusters: List[List[Pt]] = []
        for p in pts:
            hits = [cl for cl in clusters
                    if any((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 <= gap2
                           for q in cl)]
            if not hits:
                clusters.append([p])
            else:
                hits[0].append(p)
                for other in hits[1:]:      # p uniu clusters -> merge
                    hits[0].extend(other)
                    clusters.remove(other)
        return clusters


class FaceStateFile:
    """Rumo da pessoa pro face_web (cara fase 2): JSON minúsculo em tmpfs.

    Atômico (tmp + os.replace), ≤5Hz com cluster; na transição pra
    sem-cluster grava UMA vez cbear_deg=null e silencia. I/O NUNCA propaga
    (a cara é decorativa; o guard não pode cair por ela).
    """

    def __init__(self, path: str = '/tmp/motion_guard_face.json',
                 min_period: float = 0.2):
        self.path = path
        self.min_period = min_period
        self.last_error: 'str|None' = None
        self._last_write_t = -math.inf
        self._had_person = False

    def update(self, t: float, cbear_deg: 'int|None',
               state: 'str|None' = None) -> bool:
        if cbear_deg is None:
            if not self._had_person:
                return False
            self._had_person = False
        else:
            if t - self._last_write_t < self.min_period:
                return False
            self._had_person = True
        try:
            tmp = self.path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump({'ts': round(t, 3), 'cbear_deg': cbear_deg,
                           'state': state}, f)
            os.replace(tmp, self.path)
        except OSError as e:
            self.last_error = str(e)
            return False
        self._last_write_t = t
        return True


def main(args=None):  # pragma: no cover - cola de I/O, validar no sim
    import csv as _csv
    import os as _os

    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from rcl_interfaces.msg import SetParametersResult
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException

    from .utils import quat_to_yaw, spin_node

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    class MotionGuardNode(Node):
        # afináveis ao vivo (lição 04bcf86): mutam a MESMA ref de cfg que
        # observe/filter leem -> `ros2 param set` pega no tick seguinte
        _CFG_PARAMS = ('enabled', 'guard_radius', 'slow_scale', 'slow_dist',
                       'freeze_dist', 'corridor_half_w', 'corridor_len',
                       'clear_time',
                       'grid_res', 'lookback', 'min_cluster_points',
                       'persist_frames',
                       'cluster_gap', 'wz_gate', 'ray_bin_deg', 'scan_stale',
                       'map_filter', 'hold_still_max', 'hold_still_radius',
                       'wall_near', 'slow_wz_cap', 'wall_ghost_frac',
                       'settle_enabled', 'settle_window', 'settle_tol_deg',
                       'settle_max', 'settle_min_samples',
                       'settle_plan_stale', 'settle_lookahead')

        def __init__(self):
            super().__init__('motion_guard')
            cfg = GuardConfig()
            for name in self._CFG_PARAMS:
                self.declare_parameter(name, getattr(cfg, name))
                setattr(cfg, name, self.get_parameter(name).value)
            self.guard = MotionGuard(cfg)
            self.add_on_set_parameters_callback(self._on_set_params)

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self._wz = 0.0
            self._vx = 0.0
            self._last_pose = None
            self._last_state = None
            self._last_cmd_t = -math.inf
            self._face = FaceStateFile()

            self.pub = self.create_publisher(Twist, 'auto_vel_raw', 10)
            self.pub_state = self.create_publisher(
                String, 'motion_guard/state', latched)
            self.create_subscription(LaserScan, 'scan_safe', self._on_scan,
                                     qos_profile_sensor_data)
            self.create_subscription(Odometry, 'odom', self._on_odom,
                                     qos_profile_sensor_data)
            self.create_subscription(Twist, 'auto_vel_pre', self._on_cmd, 10)
            # plano global do nav2 p/ o release assentado (07-20). Já vem em
            # frame map -> sem TF. Sem plano o guard cai no release temporal
            # de hoje (fail-open no _plan_settled).
            self.create_subscription(Path, 'plan', self._on_plan, 10)
            # mapa estático p/ o filtro anti-vidro (map_server publica latched;
            # QoS transient_local pega mesmo assinando depois). Sem mapa (ex.
            # teleop puro) o filtro fica inerte — guard igual ao pré-07-08.
            self.create_subscription(OccupancyGrid, 'map', self._on_map,
                                     QoSProfile(
                                         depth=1,
                                         reliability=ReliabilityPolicy.RELIABLE,
                                         durability=QoSDurabilityPolicy
                                         .TRANSIENT_LOCAL))

            d = 'controle_web/logs'
            _os.makedirs(d, exist_ok=True)
            self._csv_f = open(_os.path.join(d, 'motion_guard.csv'),
                               'w', newline='')
            self._csv = _csv.writer(self._csv_f)
            # px..cy: diagnóstico de campo 07-03 (ONDE nasce o falso positivo:
            # pose do robô + vel medida + centróide do cluster móvel + bearing
            # relativo ao heading). Remover as colunas quando o guard assentar.
            self._csv.writerow(['t', 'state', 'n_moving', 'nearest',
                                'in_corridor', 'vx_in', 'vx_out',
                                'px', 'py', 'pyaw', 'vx_odom', 'wz_odom',
                                'cx', 'cy', 'cbear_deg', 'n_ghost', 'n_wallghost'])
            # flush em timer (8ª auditoria A5): flush por linha a ~20 Hz
            # castigava o SD da Pi. Padrão do freeze_capture; perde ≤2 s no pior.
            self.create_timer(2.0, self._csv_f.flush)
            self.get_logger().info(
                'motion_guard ativo: raio %.1fm, corredor %.2fx%.1fm, '
                'slow %.0f%%@%.1fm..100%%@%.1fm, clear %.1fs, '
                'settle %s %.1f°/%.1fs' % (
                    cfg.guard_radius, cfg.corridor_half_w * 2,
                    cfg.corridor_len, cfg.slow_scale * 100, cfg.slow_dist,
                    cfg.guard_radius, cfg.clear_time,
                    'on' if cfg.settle_enabled else 'off',
                    cfg.settle_tol_deg, cfg.settle_max))

        def _on_set_params(self, params):
            for p in params:
                if p.name in self._CFG_PARAMS:
                    setattr(self.guard.cfg, p.name, p.value)
                    self.get_logger().info(
                        'param %s = %s (live)' % (p.name, p.value))
            return SetParametersResult(successful=True)

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def _on_plan(self, msg: Path):
            self.guard.observe_plan(
                self._now(),
                [(p.pose.position.x, p.pose.position.y) for p in msg.poses])

        def _on_odom(self, msg: Odometry):
            self._wz = msg.twist.twist.angular.z
            self._vx = msg.twist.twist.linear.x

        def _on_map(self, msg: OccupancyGrid):
            i = msg.info
            self.guard.ghost_map = MapGhostFilter(
                msg.data, i.width, i.height, i.resolution,
                i.origin.position.x, i.origin.position.y)
            self.get_logger().info(
                'filtro anti-vidro: mapa %dx%d @%.2fm carregado'
                % (i.width, i.height, i.resolution))

        def _pose_odom(self):
            try:
                tf = self.tf_buffer.lookup_transform(
                    'odom', 'base_link', rclpy.time.Time())
            except TransformException:
                return None
            t = tf.transform.translation
            q = tf.transform.rotation
            return (t.x, t.y, quat_to_yaw(q.x, q.y, q.z, q.w))

        def _on_scan(self, msg: LaserScan):
            # pontos do scan -> frame odom (TF mais recente; a 10Hz e objeto
            # lento a defasagem é < grid_res). TF faltando -> NÃO alimenta o
            # guard -> scan_stale -> pass-through (failsafe da spec).
            try:
                tf = self.tf_buffer.lookup_transform(
                    'odom', msg.header.frame_id, rclpy.time.Time())
            except TransformException:
                self.get_logger().warn('sem TF odom<-%s; pass-through'
                                       % msg.header.frame_id,
                                       throttle_duration_sec=5.0)
                return
            pose = self._pose_odom()
            if pose is None:
                return
            # TF odom->map do filtro anti-vidro (AMCL publica map<-odom).
            # Sem ele (SLAM parado, AMCL caído) map_tf=None = filtro inerte.
            try:
                mtf = self.tf_buffer.lookup_transform(
                    'map', 'odom', rclpy.time.Time())
                mt, mq = mtf.transform.translation, mtf.transform.rotation
                myaw = quat_to_yaw(mq.x, mq.y, mq.z, mq.w)
                self.guard.map_tf = (mt.x, mt.y,
                                     math.cos(myaw), math.sin(myaw))
            except TransformException:
                self.guard.map_tf = None
            r = np.asarray(msg.ranges, dtype=np.float32)
            a = msg.angle_min + np.arange(r.size) * msg.angle_increment
            tt, q = tf.transform.translation, tf.transform.rotation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            # polar do scan COMPLETO pro raycast (07-03): feixe válido ->
            # alcance real (sem corte: "atravessou até lá"); feixe
            # dropado/inválido -> 0.0 = DESCONHECIDO (o dropout rasante do
            # LD06 que some-e-volta não valida mais movimento). max por bin =
            # um feixe válido no bin vence o dropout vizinho (viés pra manter
            # a detecção). LiDAR ~centro do robô (= referência do raycast).
            binw = math.radians(self.guard.cfg.ray_bin_deg)
            bear = (a + yaw + math.pi) % (2.0 * math.pi) - math.pi
            bins = np.floor(bear / binw).astype(int)
            rr = np.where(np.isfinite(r) & (r > 0.0), r, 0.0)
            polar = {}
            for b, d in zip(bins.tolist(), rr.tolist()):
                if d > polar.get(b, -1.0):
                    polar[b] = d
            # corta em guard_radius + 1m: barato e o guard re-filtra pelo robô
            ok = np.isfinite(r) & (r > 0.0) & \
                (r <= self.guard.cfg.guard_radius + 1.0)
            if not np.any(ok):
                self.guard.observe(self._now(), [], pose, self._wz,
                                   polar=polar)
                self._last_pose = pose
                self._face_tick()
                return
            c, s = math.cos(yaw), math.sin(yaw)
            xl, yl = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
            pts = list(zip((tt.x + xl * c - yl * s).tolist(),
                           (tt.y + xl * s + yl * c).tolist()))
            self.guard.observe(self._now(), pts, pose, self._wz, polar=polar)
            self._last_pose = pose
            self._face_tick()

        def _person_centroid(self):
            """(cx, cy, cbear_deg) do cluster móvel mais PRÓXIMO (o que
            manda na decisão), ou ('', '', '') sem pessoa/pose."""
            if not self.guard.moving_clusters or self._last_pose is None:
                return '', '', ''
            px, py, pyaw = self._last_pose
            cl = min(self.guard.moving_clusters,
                     key=lambda cl: min(math.hypot(p[0] - px, p[1] - py)
                                        for p in cl))
            cx = round(sum(p[0] for p in cl) / len(cl), 2)
            cy = round(sum(p[1] for p in cl) / len(cl), 2)
            cbear = round(math.degrees(
                (math.atan2(cy - py, cx - px) - pyaw + math.pi)
                % (2 * math.pi) - math.pi))
            return cx, cy, cbear

        def _face_tick(self):
            # cara fase 2 no callback do SCAN (flui sempre que o lidar
            # roda), não no do cmd — auto_vel_pre fica MUDO sem goal ativo
            # e o olho tem que seguir gente com o robô parado sem rota.
            _, _, cbear = self._person_centroid()
            t = self._now()
            # state só com cmd FRESCO: sem goal o auto_vel_pre cala e o
            # último verdict ficaria fossilizado — 'blocked' velho faria a
            # cara pedir licença pra sempre.
            state = self._last_state if t - self._last_cmd_t < 1.0 else None
            self._face.update(t, cbear if cbear != '' else None, state)
            if self._face.last_error:
                self.get_logger().warn(
                    'face state: ' + self._face.last_error,
                    throttle_duration_sec=10.0)
                self._face.last_error = None

        def _on_cmd(self, msg: Twist):
            t = self._now()
            self._last_cmd_t = t
            vx, wz, state = self.guard.filter(t, msg.linear.x, msg.angular.z)
            out = Twist()
            out.linear.x = vx
            out.angular.z = wz
            self.pub.publish(out)
            # 'settling' é 'blocked' NO FIO: unstuck_supervisor casa a string
            # EXATA 'blocked' pro standdown (BO 07-10: ré em cima de pessoa
            # durante blocked). O estado fino fica só no CSV, pra análise.
            wire = 'blocked' if state == 'settling' else state
            if wire != self._last_state:
                self._last_state = wire
                self.pub_state.publish(String(data=wire))
                if wire == 'passthrough':
                    self.get_logger().warn(
                        'pass-through (scan/TF indisponível ou disabled)',
                        throttle_duration_sec=5.0)
            cx, cy, cbear = self._person_centroid()
            pose = self._last_pose or ('', '', '')
            self._csv.writerow([
                round(t, 3), state, len(self.guard.moving_clusters),
                round(self.guard.nearest_moving, 2)
                if math.isfinite(self.guard.nearest_moving) else '',
                int(self.guard.in_corridor),
                round(msg.linear.x, 3), round(vx, 3),
                *(round(v, 3) if v != '' else '' for v in pose),
                round(self._vx, 3), round(self._wz, 3), cx, cy, cbear,
                self.guard.ghost_dropped or '',
                self.guard.wall_dropped or ''])

    rclpy.init(args=args)
    node = MotionGuardNode()
    try:
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
