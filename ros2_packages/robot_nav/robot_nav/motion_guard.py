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
  - móvel no raio guard_radius  -> linear.x *= slow_scale   (slowing)
  - móvel no corredor à frente  -> linear.x = 0 até limpar clear_time (blocked)
  - angular.z passa INTOCADO SEMPRE (escalar wz cai na zona-morta 1.7 e
    congela o point-turn — lição do rot_min 07-02).
  - TF/scan indisponível ou enabled=false -> PASS-THROUGH (nunca mata a nav).

SEM predição de cruzamento por enquanto (proposta B da spec): os pontos
móveis já saem clusterizados pra plugar velocidade+predição depois se a
versão A reagir tarde em campo.

Spec: docs/superpowers/specs/2026-07-02-motion-guard-design.md
A lógica (MotionGuard) é pura p/ testar sem ROS; main() é a cola de I/O.
"""
import math
from collections import deque
from dataclasses import dataclass
from typing import List, Tuple

Pt = Tuple[float, float]


@dataclass
class GuardConfig:
    enabled: bool = True
    guard_radius: float = 2.5       # m — só olha móvel até aqui
    slow_scale: float = 0.5         # fator no vx com móvel no raio
    corridor_half_w: float = 0.35   # m — meia-largura do corredor à frente
    corridor_len: float = 1.5       # m — alcance do corredor
    clear_time: float = 1.5         # s — corredor limpo por isso -> retoma
    grid_res: float = 0.15          # m — célula da grade de comparação
    lookback: float = 0.5           # s — compara com snapshot desta idade
    min_cluster_points: int = 3     # cluster menor = ruído
    cluster_gap: float = 0.3        # m — distância máx p/ mesmo cluster
    wz_gate: float = 0.3            # rad/s — girando acima disso não avalia
    scan_stale: float = 1.0         # s sem scan -> pass-through


class MotionGuard:
    """Detector de movimento por diff temporal em grade (frame odom).

    observe() processa um scan; filter() aplica a decisão no comando.
    """

    def __init__(self, cfg: GuardConfig):
        self.cfg = cfg
        self._snaps = deque()            # (t, frozenset de células)
        self.moving_clusters: List[List[Pt]] = []
        self.nearest_moving: float = math.inf
        self.in_corridor: bool = False
        self._last_moving_t: float = -math.inf
        self._last_corridor_t: float = -math.inf
        self._last_scan_t: float = -math.inf

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
                pose: Tuple[float, float, float], wz: float) -> None:
        c = self.cfg
        self._last_scan_t = t
        cells = frozenset(self._cell(p) for p in pts)
        self._snaps.append((t, cells))

        # GATE DE GIRO: girando, o scan inteiro "anda" (pose/TF atrasam) ->
        # não avalia; a decisão anterior decai sozinha (clear_time no filter).
        if abs(wz) > c.wz_gate:
            return
        old = self._old_snapshot(t)
        if old is None:
            return                      # histórico curto demais ainda
        _, old_cells = old

        px, py, pyaw = pose
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
            moving.append(p)

        clusters = [cl for cl in self._cluster(moving)
                    if len(cl) >= c.min_cluster_points]
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
        if clusters:
            self._last_moving_t = t
        if self.in_corridor:
            self._last_corridor_t = t

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
