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
    zone_radius: float = 1.2        # m — distância do centro que arma a manobra
    approach_bearing: float = math.radians(70)  # porta tem que estar "na frente"
    stage_dist: float = 0.6         # m — ponto de preparação antes do centro
    stage_tol: float = 0.10         # m — chegou no staging
    stage_speed: float = 0.12       # m/s — aproximação mansa
    stage_k_heading: float = 1.8    # ganho P do heading no staging
    align_lat: float = 0.08         # m — |offset lateral| máximo pra "tô no eixo"
    align_yaw: float = math.radians(5.0)   # rad — |erro de yaw| máximo
    align_stable: int = 5           # ticks consecutivos dentro da tolerância
    align_timeout: float = 15.0     # s — STAGING+ROTATING juntos
    rot_speed: float = 3.0          # rad/s — giro no lugar (vence atrito; unstuck)
    cross_speed: float = 0.15       # m/s — travessia
    cross_k_lat: float = 1.5        # corrige offset lateral durante a travessia
    cross_k_yaw: float = 2.0        # corrige heading durante a travessia
    cross_wz_max: float = 0.8       # rad/s — teto da micro-correção (NÃO girar)
    gap_min: float = 0.45           # m — vão mínimo à frente pra seguir
    exit_margin: float = 0.5        # m — além do centro pra soltar
    total_timeout: float = 40.0     # s — manobra inteira
    retrigger_cooldown: float = 3.0  # s — após abort, não rearmar na hora


class Cmd(NamedTuple):
    state: str       # idle | staging | rotating | crossing
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

    # -- helpers ------------------------------------------------------------
    def _abort(self, now: float) -> Cmd:
        self.state = 'idle'
        self.door = None
        self.geom = None
        self._stable = 0
        self._cooldown_until = now + self.cfg.retrigger_cooldown
        return Cmd('idle', 0.0, 0.0, None)

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
               scan_fresh) -> Cmd:
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
            self.state = 'staging'
            self.t_start = now
            self._stable = 0
            # cai no fluxo de staging já neste tick

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
            aligned = abs(yaw_err) <= cfg.align_yaw and abs(d) <= cfg.align_lat
            if aligned:
                self._stable += 1
                if self._stable >= cfg.align_stable:
                    self.state = 'crossing'
                    return Cmd('crossing', cfg.cross_speed, 0.0,
                               self.door['id'])
                return Cmd('rotating', 0.0, 0.0, self.door['id'])
            self._stable = 0
            if abs(d) > cfg.align_lat:
                # saiu do eixo girando (skid-steer arrasta) -> volta pro staging
                self.state = 'staging'
                return Cmd('staging', 0.0, 0.0, self.door['id'])
            wz = cfg.rot_speed if yaw_err < 0 else -cfg.rot_speed
            return Cmd('rotating', 0.0, wz, self.door['id'])

        if self.state == 'crossing':
            if gap < cfg.gap_min:
                return self._abort(now)
            if s > cfg.exit_margin:
                # atravessou: solta SEM cooldown (não é falha)
                self.state = 'idle'
                self.door = None
                self.geom = None
                return Cmd('idle', 0.0, 0.0, None)
            wz = -cfg.cross_k_lat * d - cfg.cross_k_yaw * yaw_err
            wz = max(-cfg.cross_wz_max, min(cfg.cross_wz_max, wz))
            return Cmd('crossing', cfg.cross_speed, wz, self.door['id'])

        return Cmd('idle', 0.0, 0.0, None)
