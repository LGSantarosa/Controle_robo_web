# Travessia de Porta (door_crossing) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Robô atravessa portas marcadas pelo usuário: alinha de verdade no eixo (critério numérico), atravessa reto vigiando o vão, e o collision monitor fica cego SÓ pros 2 batentes marcados SÓ durante a travessia verificada.

**Architecture:** UI marca portas (2 cliques) → `maps/<mapa>.doors.json` → `/doors` (String JSON transient_local). Novo nó `door_crossing` (lógica pura + cola, molde do unstuck_supervisor) assume via `door_vel` (twist_mux prio 20) quando goal ativo + robô na zona + nav empurrando pra porta: STAGING → ROTATING → CROSSING → solta. Publica `/door_zone`; o `scan_sanitizer` mascara os discos dos batentes no `/scan_safe` apenas quando o estado é `crossing`. Shim reverte 0.15→0.30 / 6.0→4.2.

**Tech Stack:** Python (rclpy, numpy, tf2_ros), pytest puro (PYTHONPATH da árvore), JS canvas (map.js), Flask-SocketIO.

**Regras do repo:** sem `Co-Authored-By` nos commits; robô DESLIGADO durante implementação; testes rodam `cd ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/ -q` e `cd controle_web && python3 -m pytest test_*.py -q`; deploy = push + na Pi `git fetch && git reset --hard origin/main` + `colcon build --packages-select robot_nav --base-paths ros2_packages --symlink-install`.

**Spec:** `docs/superpowers/specs/2026-06-12-zonas-de-porta-design.md`

---

### Task 1: Infra — twist_mux `door_vel` (prio 20) + `ROBOT_MAP_FILE` no launch.sh

**Files:**
- Modify: `ros2_packages/robot_nav/config/twist_mux.yaml` (depois do bloco `unstuck:`)
- Modify: `launch.sh` (bloco de exports ~linha 606; e `KNOWN_NODE_PATTERNS` ~linha 224)

- [ ] **Step 1: twist_mux.yaml — adicionar canal door entre unstuck(30) e navigation(10)**

```yaml
      unstuck:
        topic: unstuck_vel   # publicado pelo unstuck_supervisor (desencalhe)
        timeout: 0.5
        priority: 30         # > nav_vel (fura o collision na ré/giro), < web/PS4
      door:
        topic: door_vel      # publicado pelo door_crossing (travessia de porta)
        timeout: 0.5
        priority: 20         # > nav_vel (assume a manobra), < unstuck (resgate vence)
      navigation:
        topic: nav_vel       # publicado por velocity_smoother / trekking_runner
        timeout: 0.5
        priority: 10
```

- [ ] **Step 2: launch.sh — exportar o mapa pro app.py e cobrir órfão do nó novo**

No bloco de exports (junto de `ROBOT_MAPS_DIR`):

```bash
export ROBOT_MAP_FILE="$MAP_FILE"
```

Em `KNOWN_NODE_PATTERNS`, logo após `"robot_nav/scan_sanitizer"`:

```bash
    "robot_nav/door_crossing"
```

- [ ] **Step 3: validar sintaxe e commitar**

Run: `bash -n launch.sh && python3 -c "import yaml; yaml.safe_load(open('ros2_packages/robot_nav/config/twist_mux.yaml')); print('ok')"`
Expected: `ok`

```bash
git add ros2_packages/robot_nav/config/twist_mux.yaml launch.sh
git commit -m "feat(door): canal door_vel (prio 20) no twist_mux + ROBOT_MAP_FILE + órfão door_crossing"
```

---

### Task 2: door_crossing — geometria pura (TDD)

**Files:**
- Create: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (só a parte pura nesta task)
- Create: `ros2_packages/robot_nav/test/test_door_crossing.py`

Convenções de frame (iguais ao resto do repo): mapa ROS (x→, y↑, yaw CCW).
`a`/`b` = batentes. Tangente `t` = unit(b−a) (ao longo da parede). Normal
`n` = (−ty, tx) (eixo de travessia, sinal arbitrário — o chamador escolhe a
direção via `side`). Offset LATERAL do eixo = componente ao longo de `t`;
PROGRESSO da travessia = componente ao longo de `n`.

- [ ] **Step 1: testes de geometria que falham**

```python
# test/test_door_crossing.py
import math

import pytest

from robot_nav.door_crossing import (
    DoorGeom,
    door_geometry,
    door_progress_lateral,
    crossing_yaw,
)


def test_door_geometry_axis_horizontal_wall():
    # parede ao longo de x (porta "olhando" pra cima/baixo)
    g = door_geometry((1.0, 2.0), (2.0, 2.0))
    assert (g.cx, g.cy) == pytest.approx((1.5, 2.0))
    assert g.half_width == pytest.approx(0.5)
    assert (g.tx, g.ty) == pytest.approx((1.0, 0.0))
    assert (g.nx, g.ny) == pytest.approx((0.0, 1.0))


def test_progress_lateral_and_side():
    g = door_geometry((1.0, 2.0), (2.0, 2.0))
    # robô 1 m "abaixo" da porta, 0.2 m à direita do centro
    s, d = door_progress_lateral(g, 1.7, 1.0, side=+1)
    assert s == pytest.approx(-1.0)   # ainda não cruzou (progresso negativo)
    assert d == pytest.approx(0.2)    # offset lateral ao longo da parede
    # mesmo ponto com side=-1: progresso inverte, lateral mantém o sinal de t
    s2, _ = door_progress_lateral(g, 1.7, 1.0, side=-1)
    assert s2 == pytest.approx(1.0)


def test_crossing_yaw_faces_normal():
    g = door_geometry((1.0, 2.0), (2.0, 2.0))
    assert crossing_yaw(g, side=+1) == pytest.approx(math.pi / 2)   # +n = +y
    assert crossing_yaw(g, side=-1) == pytest.approx(-math.pi / 2)


def test_door_geometry_diagonal():
    g = door_geometry((0.0, 0.0), (1.0, 1.0))
    assert g.half_width == pytest.approx(math.sqrt(2) / 2)
    # n perpendicular a t, ambos unitários
    assert g.tx * g.nx + g.ty * g.ny == pytest.approx(0.0)
    assert math.hypot(g.nx, g.ny) == pytest.approx(1.0)
```

- [ ] **Step 2: rodar e ver falhar**

Run: `cd ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_door_crossing.py -q`
Expected: FAIL (`ModuleNotFoundError: robot_nav.door_crossing`)

- [ ] **Step 3: implementação mínima**

```python
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
from dataclasses import dataclass, field
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
```

- [ ] **Step 4: rodar e ver passar**

Run: `cd ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_door_crossing.py -q`
Expected: 4 passed

- [ ] **Step 5: commit**

```bash
git add ros2_packages/robot_nav/robot_nav/door_crossing.py ros2_packages/robot_nav/test/test_door_crossing.py
git commit -m "feat(door): geometria pura da porta (eixos, progresso, lateral, yaw de travessia)"
```

---

### Task 3: door_crossing — vão à frente no scan (puro, numpy)

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (append)
- Modify: `ros2_packages/robot_nav/test/test_door_crossing.py` (append)

- [ ] **Step 1: testes que falham**

```python
# append em test/test_door_crossing.py
from robot_nav.door_crossing import gap_ahead


def _scan_one_point(x_robot, y_robot):
    # constrói um scan de 8 feixes com UM ponto em (x,y) no frame do robô
    import numpy as np
    a = math.atan2(y_robot, x_robot)
    r = math.hypot(x_robot, y_robot)
    angle_min, inc = -math.pi, math.pi / 4
    ranges = [float('inf')] * 8
    idx = int(round((a - angle_min) / inc)) % 8
    ranges[idx] = r
    return ranges, angle_min, inc


def test_gap_ahead_sees_obstacle_in_corridor():
    ranges, amin, ainc = _scan_one_point(0.5, 0.0)   # bem na frente
    g = gap_ahead(ranges, amin, ainc, pose=(0.0, 0.0, 0.0),
                  jambs=[], jamb_r=0.30)
    assert g == pytest.approx(0.5, abs=0.15)  # discretização de 8 feixes


def test_gap_ahead_ignores_lateral_and_behind():
    for px, py in [(0.0, 1.0), (-0.5, 0.0), (0.5, 0.6)]:
        ranges, amin, ainc = _scan_one_point(px, py)
        g = gap_ahead(ranges, amin, ainc, pose=(0.0, 0.0, 0.0),
                      jambs=[], jamb_r=0.30)
        assert math.isinf(g)


def test_gap_ahead_excludes_marked_jamb():
    # ponto na frente, mas que em coordenadas do MAPA cai no disco do batente
    ranges, amin, ainc = _scan_one_point(0.5, 0.0)
    pose = (3.0, 4.0, 0.0)                      # robô no mapa
    jamb = (3.5, 4.0)                            # batente exatamente ali
    g = gap_ahead(ranges, amin, ainc, pose=pose,
                  jambs=[jamb], jamb_r=0.30)
    assert math.isinf(g)                         # batente não conta como vão
```

- [ ] **Step 2: rodar e ver falhar**

Run: `cd ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_door_crossing.py -q`
Expected: FAIL (`ImportError: gap_ahead`)

- [ ] **Step 3: implementação (append no door_crossing.py, depois da geometria)**

```python
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
```

- [ ] **Step 4: rodar e ver passar**

Run: `cd ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_door_crossing.py -q`
Expected: 7 passed

- [ ] **Step 5: commit**

```bash
git add ros2_packages/robot_nav/robot_nav/door_crossing.py ros2_packages/robot_nav/test/test_door_crossing.py
git commit -m "feat(door): gap_ahead — vão à frente no scan, descontando os batentes marcados"
```

---

### Task 4: door_crossing — máquina de estados pura (TDD)

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (append)
- Modify: `ros2_packages/robot_nav/test/test_door_crossing.py` (append)

API: `DoorCrossing(cfg).update(now, pose, doors, goal_active, nav_forward,
gap, scan_fresh) -> Cmd`. `pose=(x,y,yaw)` no mapa ou `None`; `doors` =
lista de dicts `{'id', 'a', 'b'}`; `nav_forward` = nav comandando avanço;
`gap` = saída de `gap_ahead`; `Cmd = (state, vx, wz, door_id)`. O nó só
publica Twist quando `state != 'idle'`.

- [ ] **Step 1: testes que falham**

```python
# append em test/test_door_crossing.py
from robot_nav.door_crossing import DoorCrossing, DoorCrossConfig

DOOR = {'id': 1, 'a': [1.0, 2.0], 'b': [2.0, 2.0]}   # parede em x, vão 1.0 m
CFG = DoorCrossConfig()


def mk():
    return DoorCrossing(CFG)


def step(dc, t, pose, goal=True, nav=True, gap=math.inf, fresh=True):
    return dc.update(t, pose, [DOOR], goal, nav, gap, fresh)


def test_idle_sem_goal_ou_fora_da_zona():
    dc = mk()
    # na zona mas sem goal
    assert step(dc, 0.0, (1.5, 1.2, math.pi/2), goal=False).state == 'idle'
    # com goal mas longe (>zone_radius do centro)
    assert step(dc, 0.1, (1.5, -1.0, math.pi/2)).state == 'idle'
    # sem pose (TF caiu) nunca arma
    assert step(dc, 0.2, None).state == 'idle'


def test_arma_e_vai_pro_staging():
    dc = mk()
    # a 1.0 m do centro, olhando pra porta, nav empurrando
    c = step(dc, 0.0, (1.5, 1.0, math.pi/2))
    assert c.state == 'staging'
    assert c.door_id == 1
    # side foi escolhido pra aproximação: alvo de staging fica ENTRE o robô
    # e a porta (y = 2.0 - stage_dist)
    assert dc.side == +1


def test_staging_converge_e_rotaciona():
    dc = mk()
    t = 0.0
    pose = (1.7, 1.2, 0.0)   # fora do eixo, yaw errado
    c = step(dc, t, pose)
    assert c.state == 'staging'
    # teleporta pro ponto de staging (simula chegada): vira ROTATING
    stage_y = 2.0 - CFG.stage_dist
    c = step(dc, t + 1.0, (1.5, stage_y, 0.0))
    assert c.state == 'rotating'
    assert c.vx == pytest.approx(0.0)
    assert c.wz != 0.0   # girando pra encarar pi/2


def test_rotating_estavel_vira_crossing():
    dc = mk()
    stage_y = 2.0 - CFG.stage_dist
    step(dc, 0.0, (1.5, stage_y - 0.3, math.pi/2))    # arma (staging)
    step(dc, 0.1, (1.5, stage_y, math.pi/2))          # chegou -> rotating
    # já alinhado: precisa de align_stable ticks estáveis pra promover
    t = 0.2
    for _ in range(CFG.align_stable):
        c = step(dc, t, (1.5, stage_y, math.pi/2))
        t += 0.05
    assert c.state == 'crossing'


def _ate_crossing(dc):
    stage_y = 2.0 - CFG.stage_dist
    step(dc, 0.0, (1.5, stage_y - 0.3, math.pi/2))
    step(dc, 0.1, (1.5, stage_y, math.pi/2))
    t = 0.2
    for _ in range(CFG.align_stable):
        c = step(dc, t, (1.5, stage_y, math.pi/2))
        t += 0.05
    assert c.state == 'crossing'
    return t


def test_crossing_anda_reto_e_solta_depois_da_porta():
    dc = mk()
    t = _ate_crossing(dc)
    c = step(dc, t, (1.5, 1.9, math.pi/2))
    assert c.state == 'crossing' and c.vx == pytest.approx(CFG.cross_speed)
    # passou do centro + exit_margin -> solta
    c = step(dc, t + 1.0, (1.5, 2.0 + CFG.exit_margin + 0.05, math.pi/2))
    assert c.state == 'idle'


def test_crossing_aborta_se_vao_fecha_ou_goal_morre():
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 1.9, math.pi/2), gap=0.3).state == 'idle'
    dc2 = mk()
    t2 = _ate_crossing(dc2)
    assert step(dc2, t2, (1.5, 1.9, math.pi/2), goal=False).state == 'idle'


def test_align_timeout_aborta_e_respeita_cooldown():
    dc = mk()
    step(dc, 0.0, (1.5, 1.0, math.pi/2))                       # arma
    c = step(dc, CFG.align_timeout + 0.1, (1.5, 1.0, math.pi/2))
    assert c.state == 'idle'
    # cooldown: tick seguinte ainda não rearma
    assert step(dc, CFG.align_timeout + 0.2, (1.5, 1.0, math.pi/2)).state == 'idle'
    # passado o cooldown, rearma
    t = CFG.align_timeout + CFG.retrigger_cooldown + 0.3
    assert step(dc, t, (1.5, 1.0, math.pi/2)).state == 'staging'


def test_scan_velho_aborta_crossing():
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 1.9, math.pi/2), fresh=False).state == 'idle'
```

- [ ] **Step 2: rodar e ver falhar**

Run: `cd ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_door_crossing.py -q`
Expected: FAIL (`ImportError: DoorCrossing`)

- [ ] **Step 3: implementação (append no door_crossing.py)**

```python
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
            bearing = _wrap(math.atan2(g.cy - y, g.cx - x) - yaw)
            if abs(bearing) > self.cfg.approach_bearing:
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
```

- [ ] **Step 4: rodar e ver passar (sinais do controle conferidos pelos testes)**

Run: `cd ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_door_crossing.py -q`
Expected: 15 passed
Nota: se `test_staging_converge_e_rotaciona` falhar no sinal de `wz`, o bug é
na convenção de `_wrap(head - yaw)` — corrigir a implementação, NUNCA o teste.

- [ ] **Step 5: commit**

```bash
git add ros2_packages/robot_nav/robot_nav/door_crossing.py ros2_packages/robot_nav/test/test_door_crossing.py
git commit -m "feat(door): máquina de estados pura da travessia (staging/rotating/crossing + aborts)"
```

---

### Task 5: door_crossing — nó (cola de I/O) + launch + setup.py

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (append `main()`)
- Modify: `ros2_packages/robot_nav/setup.py` (entry point)
- Modify: `ros2_packages/robot_nav/launch/nav2.launch.py` (nó novo)

- [ ] **Step 1: main() no padrão do unstuck (imports dentro, classe local)**

```python
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

    ACTIVE = {1, 2, 3}  # ACCEPTED, EXECUTING, CANCELING (igual unstuck)

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    class DoorCrossingNode(Node):
        def __init__(self):
            super().__init__('door_crossing')
            g = {}
            for name, default in (
                ('zone_radius', 1.2), ('stage_dist', 0.6),
                ('align_lat', 0.08), ('align_yaw_deg', 5.0),
                ('align_timeout', 15.0), ('rot_speed', 3.0),
                ('cross_speed', 0.15), ('gap_min', 0.45),
                ('exit_margin', 0.5), ('rate_hz', 20.0),
                ('scan_stale', 0.6), ('nav_move_lin', 0.02),
            ):
                self.declare_parameter(name, default)
                g[name] = self.get_parameter(name).value

            self.cfg = DoorCrossConfig(
                zone_radius=g['zone_radius'], stage_dist=g['stage_dist'],
                align_lat=g['align_lat'],
                align_yaw=math.radians(g['align_yaw_deg']),
                align_timeout=g['align_timeout'], rot_speed=g['rot_speed'],
                cross_speed=g['cross_speed'], gap_min=g['gap_min'],
                exit_margin=g['exit_margin'])
            self.sup = DoorCrossing(self.cfg)
            self.scan_stale = g['scan_stale']
            self.nav_move_lin = g['nav_move_lin']

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
            self._nav_forward = msg.linear.x > self.nav_move_lin

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

            prev = self.sup.state
            cmd = self.sup.update(now, pose, self.doors, goal,
                                  self._nav_forward, gap, fresh)
            if cmd.state != prev:
                self.get_logger().info(f'door_crossing: {prev} -> {cmd.state}')
            self._publish_zone(cmd.state, cmd.door_id)
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
```

E no topo do arquivo (junto dos imports puros), adicionar `import time`.

- [ ] **Step 2: setup.py — entry point**

```python
            'scan_sanitizer = robot_nav.scan_sanitizer:main',
            'door_crossing = robot_nav.door_crossing:main',
```

- [ ] **Step 3: nav2.launch.py — subir o nó (depois do scan_sanitizer)**

```python
        # Travessia de porta: alinha no eixo de porta MARCADA e atravessa
        # reto vigiando o vão (door_vel, prio 20 no twist_mux). Publica
        # /door_zone = gate da máscara de batente no scan_sanitizer.
        # Spec: docs/superpowers/specs/2026-06-12-zonas-de-porta-design.md
        Node(
            package='robot_nav', executable='door_crossing',
            name='door_crossing', output=nav_output,
            parameters=[sim_time_param],
        ),
```

- [ ] **Step 4: build + smoke**

Run: `cd /home/rbe-luis/Workspace/Controle_robo_web && source /opt/ros/jazzy/setup.bash && colcon build --packages-select robot_nav --base-paths ros2_packages --symlink-install && source install/setup.bash && (timeout 6 ros2 run robot_nav door_crossing >/tmp/door_smoke.log 2>&1; true) && cat /tmp/door_smoke.log`
Expected: log `door_crossing ativo: zona 1.2m...` sem traceback

- [ ] **Step 5: pytest inteiro + commit**

Run: `cd ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/ -q`
Expected: todos passam (82: 67 existentes + 15 da task 2-4)

```bash
git add ros2_packages/robot_nav/robot_nav/door_crossing.py ros2_packages/robot_nav/setup.py ros2_packages/robot_nav/launch/nav2.launch.py
git commit -m "feat(door): nó door_crossing (TF map->base, gate por goal, door_vel + /door_zone)"
```

---

### Task 6: scan_sanitizer — máscara de batente gated por /door_zone

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/scan_sanitizer.py`
- Modify: `ros2_packages/robot_nav/test/test_scan_sanitizer.py` (append)

- [ ] **Step 1: testes que falham**

```python
# append em test/test_scan_sanitizer.py
from robot_nav.scan_sanitizer import mask_door_jambs


def test_mask_door_jambs_kills_only_jamb_points():
    # robô no mapa em (3,4) yaw=0; feixe da frente acerta o batente (3.5,4.0)
    ranges = [0.5, 2.0]
    angle_min, inc = 0.0, math.pi / 2     # feixe0 = frente, feixe1 = esquerda
    out, n = mask_door_jambs(ranges, angle_min, inc,
                             pose=(3.0, 4.0, 0.0),
                             jambs=[(3.5, 4.0)], jamb_r=0.30)
    assert n == 1
    assert math.isinf(out[0])
    assert out[1] == pytest.approx(2.0)   # fora do disco: intacto


def test_mask_door_jambs_no_jambs_noop():
    out, n = mask_door_jambs([0.5], 0.0, 0.1, pose=(0, 0, 0),
                             jambs=[], jamb_r=0.30)
    assert n == 0 and out[0] == pytest.approx(0.5)


def test_mask_composes_with_phantom_filter():
    # fantasma a 5cm E batente real: os dois filtros compõem
    ranges = [0.05, 0.5]
    angle_min, inc = 0.0, math.pi / 2
    r1, n1 = sanitize_ranges(ranges, min_valid=0.15)
    r2, n2 = mask_door_jambs(r1, angle_min, inc, pose=(3.0, 4.0, 0.0),
                             jambs=[(3.5, 4.0)], jamb_r=0.30)
    assert n1 == 1 and n2 == 1
    assert math.isinf(r2[0]) and math.isinf(r2[1])
```

- [ ] **Step 2: rodar e ver falhar**

Run: `cd ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_scan_sanitizer.py -q`
Expected: FAIL (`ImportError: mask_door_jambs`)

- [ ] **Step 3: pure function (append na seção pura do scan_sanitizer.py)**

```python
def mask_door_jambs(ranges, angle_min: float, angle_increment: float,
                    pose, jambs, jamb_r: float):
    """(ranges com batentes mascarados, n_mascarados).

    Converte cada retorno pro frame do MAPA (pose = (x,y,yaw) do TF) e troca
    por +inf os que caem num disco de jamb_r ao redor de um batente marcado.
    Chamado SÓ quando o door_crossing está em estado 'crossing' (gate) — o
    collision monitor fica "do tamanho da porta": cego pros 2 batentes
    clicados, enxergando todo o resto (pessoa no vão continua freando).
    """
    r = np.asarray(ranges, dtype=np.float32)
    if r.size == 0 or not jambs or angle_increment == 0.0:
        return r, 0
    ok = np.isfinite(r) & (r > 0.0)
    rr = np.where(ok, r, 0.0)
    a = angle_min + np.arange(r.size) * angle_increment
    x = rr * np.cos(a)
    y = rr * np.sin(a)
    px, py, pyaw = pose
    c, s = math.cos(pyaw), math.sin(pyaw)
    mx = px + x * c - y * s
    my = py + x * s + y * c
    bad = np.zeros(r.size, dtype=bool)
    for jx, jy in jambs:
        bad |= ((mx - jx) ** 2 + (my - jy) ** 2) <= jamb_r ** 2
    bad &= ok
    n = int(np.count_nonzero(bad))
    if n == 0:
        return r, 0
    out = r.copy()
    out[bad] = math.inf
    return out, n
```

- [ ] **Step 4: cola no nó (dentro do main() do scan_sanitizer)**

Adicionar imports no main: `import json`, `from std_msgs.msg import String`,
`from tf2_ros import Buffer, TransformListener, TransformException`,
`from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy`,
e `from .utils import quat_to_yaw`. Na classe `ScanSanitizer.__init__`:

```python
            self.declare_parameter('jamb_radius', 0.30)
            self.jamb_r = float(self.get_parameter('jamb_radius').value)
            self._doors = {}          # id -> {'a': [x,y], 'b': [x,y]}
            self._crossing_id = None  # id da porta em travessia, ou None
            self._masked_total = 0
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            latched = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, 'doors', self._on_doors, latched)
            self.create_subscription(String, 'door_zone', self._on_zone, latched)
```

Métodos novos na classe:

```python
        def _on_doors(self, msg):
            try:
                self._doors = {d['id']: d
                               for d in json.loads(msg.data).get('doors', [])}
            except (ValueError, KeyError, TypeError) as e:
                self.get_logger().warn(f'/doors inválido: {e}')

        def _on_zone(self, msg):
            try:
                z = json.loads(msg.data)
                self._crossing_id = (z.get('door_id')
                                     if z.get('state') == 'crossing' else None)
            except ValueError:
                self._crossing_id = None

        def _pose_map(self):
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time())
            except TransformException:
                return None
            t = tf.transform.translation
            q = tf.transform.rotation
            return (t.x, t.y, quat_to_yaw(q.x, q.y, q.z, q.w))
```

E no `_on_scan`, depois do filtro de fantasmas e antes do publish:

```python
            door = self._doors.get(self._crossing_id)
            if door is not None:
                pose = self._pose_map()
                if pose is not None:     # sem TF -> fail-safe: sem máscara
                    base = out if n else np.asarray(msg.ranges,
                                                    dtype=np.float32)
                    masked, nm = mask_door_jambs(
                        base, msg.angle_min, msg.angle_increment, pose,
                        [tuple(door['a']), tuple(door['b'])], self.jamb_r)
                    if nm:
                        self._masked_total += nm
                        self.get_logger().info(
                            f'porta {self._crossing_id}: {nm} ponto(s) de '
                            f'batente mascarado(s) (total {self._masked_total})',
                            throttle_duration_sec=5.0)
                        msg.ranges = masked.tolist()
```

(Atenção ao fluxo existente: hoje `msg.ranges` só é reescrito quando `n>0`;
com a máscara, reescrever também quando `nm>0`.)

- [ ] **Step 5: pytest + build + smoke + commit**

Run: `cd ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/ -q`
Expected: todos passam (85)
Run (na raiz): `source /opt/ros/jazzy/setup.bash && colcon build --packages-select robot_nav --base-paths ros2_packages --symlink-install && source install/setup.bash && (timeout 6 ros2 run robot_nav scan_sanitizer >/tmp/san_smoke.log 2>&1; true) && cat /tmp/san_smoke.log`
Expected: boot limpo

```bash
git add ros2_packages/robot_nav/robot_nav/scan_sanitizer.py ros2_packages/robot_nav/test/test_scan_sanitizer.py
git commit -m "feat(door): máscara de batente no scan_sanitizer, gated pelo estado crossing do /door_zone"
```

---

### Task 7: DoorStore + /doors no MapBridge (TDD na parte pura)

**Files:**
- Modify: `controle_web/map_service.py`
- Create: `controle_web/test_map_service_doors.py`

- [ ] **Step 1: testes que falham**

```python
# controle_web/test_map_service_doors.py
import json

import pytest

from map_service import DoorStore


def test_add_persists_and_assigns_id(tmp_path):
    p = tmp_path / 'sala.doors.json'
    ds = DoorStore(str(p))
    d = ds.add([1.0, 2.0], [1.9, 2.0])
    assert d['id'] == 1
    on_disk = json.loads(p.read_text())
    assert on_disk['doors'][0]['a'] == [1.0, 2.0]


def test_add_validates_width(tmp_path):
    ds = DoorStore(str(tmp_path / 'x.doors.json'))
    with pytest.raises(ValueError):
        ds.add([0.0, 0.0], [0.1, 0.0])      # 0.1 m: estreito demais
    with pytest.raises(ValueError):
        ds.add([0.0, 0.0], [3.0, 0.0])      # 3 m: não é porta


def test_remove_and_reload(tmp_path):
    p = tmp_path / 'sala.doors.json'
    ds = DoorStore(str(p))
    d1 = ds.add([0.0, 0.0], [1.0, 0.0])
    ds.add([5.0, 0.0], [6.0, 0.0])
    assert ds.remove(d1['id']) is True
    assert ds.remove(99) is False
    ds2 = DoorStore(str(p))                  # recarrega do disco
    assert len(ds2.doors) == 1
    assert ds2.doors[0]['id'] == 2


def test_payload_shape(tmp_path):
    ds = DoorStore(str(tmp_path / 'x.doors.json'))
    ds.add([0.0, 0.0], [1.0, 0.0])
    pl = json.loads(ds.payload())
    assert pl == {'doors': [{'id': 1, 'a': [0.0, 0.0], 'b': [1.0, 0.0]}]}


def test_corrupt_file_starts_empty(tmp_path):
    p = tmp_path / 'bad.doors.json'
    p.write_text('{nope')
    ds = DoorStore(str(p))
    assert ds.doors == []
```

- [ ] **Step 2: rodar e ver falhar**

Run: `cd controle_web && python3 -m pytest test_map_service_doors.py -q`
Expected: FAIL (`ImportError: DoorStore`)

- [ ] **Step 3: DoorStore puro (em map_service.py, perto dos outros helpers)**

```python
class DoorStore:
    """Portas marcadas pelo usuário (2 batentes por porta), persistidas em
    maps/<mapa>.doors.json. Consumidas pelo door_crossing/scan_sanitizer via
    /doors. Spec: docs/superpowers/specs/2026-06-12-zonas-de-porta-design.md
    """
    MIN_W, MAX_W = 0.4, 2.0

    def __init__(self, path: str):
        self.path = path
        self.doors = []
        try:
            with open(path, encoding='utf-8') as f:
                self.doors = json.load(f).get('doors', [])
        except (OSError, ValueError):
            self.doors = []

    def _save(self):
        with open(self.path, 'w', encoding='utf-8') as f:
            json.dump({'doors': self.doors}, f, indent=1)

    def add(self, a, b) -> dict:
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        w = math.hypot(bx - ax, by - ay)
        if not (self.MIN_W <= w <= self.MAX_W):
            raise ValueError(
                f'vão de {w:.2f} m fora da faixa {self.MIN_W}-{self.MAX_W} m')
        new_id = max((d['id'] for d in self.doors), default=0) + 1
        door = {'id': new_id, 'a': [ax, ay], 'b': [bx, by]}
        self.doors.append(door)
        self._save()
        return door

    def remove(self, door_id) -> bool:
        n = len(self.doors)
        self.doors = [d for d in self.doors if d['id'] != door_id]
        if len(self.doors) != n:
            self._save()
            return True
        return False

    def payload(self) -> str:
        return json.dumps({'doors': self.doors})
```

(Garantir `import json`/`import math` no topo — map_service já deve ter.)

- [ ] **Step 4: rodar e ver passar**

Run: `cd controle_web && python3 -m pytest test_map_service_doors.py -q`
Expected: 5 passed

- [ ] **Step 5: fiação no MapBridge (modo nav2)**

No `MapBridge.__init__` (junto dos outros publishers; usar o QoS
transient_local que o arquivo já usa pro /map como referência):

```python
        # Portas marcadas (travessia): arquivo ao lado do mapa carregado.
        map_file = os.environ.get('ROBOT_MAP_FILE', '')
        stem = os.path.splitext(os.path.basename(map_file))[0] or 'doors'
        self._doors = DoorStore(os.path.join(maps_dir, f'{stem}.doors.json'))
        doors_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._doors_pub = self._node.create_publisher(String, '/doors',
                                                      doors_qos)
        self._doors_pub.publish(String(data=self._doors.payload()))
        # Estado da travessia -> chip na UI
        self._node.create_subscription(String, '/door_zone',
                                       self._on_door_zone, doors_qos)
```

Métodos no MapBridge:

```python
    def door_cmd(self, data: dict) -> dict:
        try:
            if 'add' in data:
                d = self._doors.add(data['add']['a'], data['add']['b'])
            elif 'del' in data:
                if not self._doors.remove(int(data['del'])):
                    return {'ok': False, 'error': 'porta não encontrada'}
                d = None
            else:
                return {'ok': False, 'error': 'cmd desconhecido'}
        except (ValueError, KeyError, TypeError) as e:
            return {'ok': False, 'error': str(e)}
        self._doors_pub.publish(String(data=self._doors.payload()))
        self._socketio.emit('doors_update', self._doors.payload())
        return {'ok': True, 'door': d}

    def get_doors_payload(self) -> str:
        return self._doors.payload()

    def _on_door_zone(self, msg):
        self._socketio.emit('door_zone', msg.data)
```

(Conferir os nomes reais dos imports de QoS no topo do map_service.py —
seguir o padrão já usado pro /map latched; `String` de `std_msgs.msg`.)

- [ ] **Step 6: socket no app.py + replay no connect**

```python
@socketio.on('door_cmd')
def handle_door_cmd(data):
    """Marca/apaga porta no mapa (travessia door_crossing)."""
    if map_bridge is None:
        emit('door_ack', {'ok': False, 'error': 'sem mapa neste modo'})
        return
    result = map_bridge.door_cmd(data or {})
    app.logger.info(f"door_cmd from {request.remote_addr}: {data} -> {result.get('ok')}")
    emit('door_ack', result)
```

E no handler de `connect` existente, junto do replay do mapa/waypoints:

```python
    if map_bridge is not None:
        emit('doors_update', map_bridge.get_doors_payload())
```

- [ ] **Step 7: testes web + commit**

Run: `cd controle_web && python3 -m pytest test_*.py -q`
Expected: todos passam (20: 15 + 5 novos)

```bash
git add controle_web/map_service.py controle_web/test_map_service_doors.py controle_web/app.py
git commit -m "feat(door): DoorStore persistido por mapa + /doors transient_local + socket door_cmd + relay door_zone"
```

---

### Task 8: UI — marcar porta no mapa (map.js + index.html)

**Files:**
- Modify: `controle_web/templates/index.html` (botões do painel nav2, perto do botão de waypoints)
- Modify: `controle_web/static/js/map.js`

Sem testes JS no repo — validação manual no navegador (passo final).

- [ ] **Step 1: index.html — botão + chip (no bloco dos controles do mapa nav2, junto de `map-btn-wp`/afins)**

```html
<button id="map-btn-door" class="btn btn-sm">🚪 Marcar porta</button>
<span id="map-door-chip" class="chip" style="display:none"></span>
```

- [ ] **Step 2: map.js — estado/modo (junto das outras vars de modo, ~linha 30)**

```javascript
  let doorMode = false;          // modo "marcar porta"
  let doorFirst = null;          // 1º batente clicado {x, y}
  let doors = [];                // [{id, a:[x,y], b:[x,y]}]
  let doorZone = null;           // {state, door_id} vindo do robô
  const btnDoor = document.getElementById('map-btn-door');
  const doorChip = document.getElementById('map-door-chip');
```

- [ ] **Step 3: map.js — toggle do modo + eventos socket (junto dos outros handlers)**

```javascript
  btnDoor.addEventListener('click', () => {
    doorMode = !doorMode;
    doorFirst = null;
    btnDoor.classList.toggle('active', doorMode);
    statusEl.textContent = doorMode
      ? 'modo porta: clique no 1º batente (clique numa porta p/ apagar)'
      : '';
    render();
  });

  socket.on('doors_update', (payload) => {
    try { doors = JSON.parse(payload).doors || []; } catch (e) { doors = []; }
    render();
  });

  socket.on('door_ack', (r) => {
    if (!r.ok) statusEl.textContent = `porta: ${r.error}`;
  });

  socket.on('door_zone', (payload) => {
    try { doorZone = JSON.parse(payload); } catch (e) { doorZone = null; }
    const active = doorZone && doorZone.state !== 'idle';
    doorChip.style.display = active ? '' : 'none';
    if (active) {
      const nome = {staging: 'indo pro eixo', rotating: 'alinhando',
                    crossing: 'ATRAVESSANDO'}[doorZone.state] || doorZone.state;
      doorChip.textContent = `🚪 porta ${doorZone.door_id}: ${nome}`;
    }
    render();
  });
```

- [ ] **Step 4: map.js — cliques do modo porta (no handler de mouseup, ANTES do bloco "Click simples → goal único")**

```javascript
      if (doorMode && wpMouseDown) {
        const world = wpMouseDown.world;
        // clique perto de porta existente = apagar
        const NEAR = 0.35;
        const hit = doors.find(d => {
          const mx = (d.a[0] + d.b[0]) / 2, my = (d.a[1] + d.b[1]) / 2;
          return Math.hypot(world.x - mx, world.y - my) < NEAR;
        });
        if (hit) {
          socket.emit('door_cmd', { del: hit.id });
          statusEl.textContent = `porta ${hit.id} apagada`;
        } else if (!doorFirst) {
          doorFirst = world;
          statusEl.textContent = 'agora clique no 2º batente';
        } else {
          socket.emit('door_cmd', {
            add: { a: [doorFirst.x, doorFirst.y], b: [world.x, world.y] } });
          doorFirst = null;
          statusEl.textContent = 'porta marcada';
        }
        wpDrag = null; wpMouseDown = null;
        render();
        return;
      }
```

- [ ] **Step 5: map.js — desenho das portas no render() (depois do plan, antes do robô)**

```javascript
    // Portas marcadas: segmento entre batentes + discos; ativa = destacada
    doors.forEach(d => {
      const a = worldToCanvas(d.a[0], d.a[1]);
      const b = worldToCanvas(d.b[0], d.b[1]);
      if (!a || !b) return;
      const active = doorZone && doorZone.door_id === d.id
                     && doorZone.state !== 'idle';
      ctx.strokeStyle = active ? '#0f0' : '#0aa';
      ctx.lineWidth = active ? 3 : 2;
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      const rPix = (0.30 / mapInfo.resolution) * getDrawRect().scale;
      [a, b].forEach(p => {
        ctx.beginPath(); ctx.arc(p.x, p.y, rPix, 0, 2 * Math.PI); ctx.stroke();
      });
      ctx.fillStyle = ctx.strokeStyle;
      ctx.font = '12px sans-serif';
      ctx.fillText(`🚪${d.id}`, (a.x + b.x) / 2 + 6, (a.y + b.y) / 2 - 6);
    });
    if (doorMode && doorFirst) {
      const p = worldToCanvas(doorFirst.x, doorFirst.y);
      if (p) {
        ctx.fillStyle = '#0aa';
        ctx.beginPath(); ctx.arc(p.x, p.y, 4, 0, 2 * Math.PI); ctx.fill();
      }
    }
```

- [ ] **Step 6: commit**

```bash
git add controle_web/templates/index.html controle_web/static/js/map.js
git commit -m "feat(door): UI marcar porta (2 cliques nos batentes), desenho no mapa e chip de estado"
```

---

### Task 9: revert do shim (nav2_params_pi.yaml)

**Files:**
- Modify: `ros2_packages/robot_nav/config/nav2_params_pi.yaml:113-131`

- [ ] **Step 1: trocar valores e atualizar o comentário**

`angular_dist_threshold: 0.15` → `0.30` e
`rotate_to_heading_angular_vel: 6.0` → `4.2`. Substituir o parágrafo
"2026-06-11 PORTA: ..." do comentário por:

```yaml
      # 2026-06-11 PORTA: apertamos p/ 0.15 (~9°) pra caber na porta — e a
      # oscilação prevista CONFIRMOU em campo 2026-06-12 ("tenta várias e
      # várias vezes até ficar de frente"). Rollback executado: 0.30 (~17°) +
      # rotate_to_heading_angular_vel 6.0->4.2 (giro mais fino). A precisão
      # fina de porta agora é do door_crossing (alinha |lat|<8cm |yaw|<5° em
      # malha fechada no TF e atravessa reto) — o shim não precisa mais dela.
```

Atenção: manter o comentário existente que explica por que 6.0 casa com o
manual — mas anotar nele que o rotate_to_heading usa 4.2 (o giro do shim é
contínuo e fino; o piso de 6.0 vale pro arranque parado do teleop/unstuck).

- [ ] **Step 2: validar + commit**

Run: `python3 -c "import yaml; yaml.safe_load(open('ros2_packages/robot_nav/config/nav2_params_pi.yaml')); print('ok')"`
Expected: `ok`

```bash
git add ros2_packages/robot_nav/config/nav2_params_pi.yaml
git commit -m "revert(nav2/shim): 0.15->0.30 + vel 6.0->4.2 — oscilação confirmada em campo; porta agora é do door_crossing"
```

---

### Task 10: verificação final + push (robô continua desligado)

- [ ] **Step 1: suites completas**

Run: `cd ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/ -q && cd ../../controle_web && python3 -m pytest test_*.py -q`
Expected: ~85 + 20, zero falhas

- [ ] **Step 2: build limpo + smokes**

Run: `source /opt/ros/jazzy/setup.bash && colcon build --packages-select robot_nav --base-paths ros2_packages --symlink-install && source install/setup.bash && (timeout 6 ros2 run robot_nav door_crossing >/tmp/s1.log 2>&1; true) && (timeout 6 ros2 run robot_nav scan_sanitizer >/tmp/s2.log 2>&1; true) && tail -1 /tmp/s1.log /tmp/s2.log && bash -n launch.sh && echo TUDO-OK`
Expected: `TUDO-OK` e logs de boot dos 2 nós

- [ ] **Step 3: push**

```bash
git push origin main
```

---

### Task 11: deploy + validação de campo (PRECISA do robô LIGADO — anunciar e esperar o "pode")

- [ ] **Step 1: deploy na Pi**

```bash
ssh robo@robo-desktop.local 'cd ~/workspace/Controle_robo_web && git fetch origin && git reset --hard origin/main && source /opt/ros/jazzy/setup.bash && colcon build --packages-select robot_nav --base-paths ros2_packages --symlink-install'
```

(Se o mDNS falhar, IP via `getent hosts robo-desktop.local`; era 192.168.18.95.)

- [ ] **Step 2: roteiro de campo (usuário dirige/manda goal; assistente só lê)**

1. Subir nav2; conferir no log: `door_crossing ativo`, `scan_sanitizer ativo`.
2. UI: marcar a porta real (2 cliques nos batentes), conferir desenho + json.
3. Goal através da porta: (a) aproximação torta ainda freia no collision;
   (b) chip muda staging→alinhando→ATRAVESSANDO; (c) alinhamento <15 s SEM
   oscilar (revert do shim); (d) atravessa sem freeze; (e) `/door_zone` volta
   a idle e o nav2 segue o goal.
4. Segurança: pessoa parada NO VÃO durante o crossing → robô para (gap_min).
5. Conferir fantasmas: log do sanitizer (`retorno(s) fantasma ... descartado`)
   correlacionado com não-freeze.
6. Anotar números no handoff de memória.

---

## Self-review (do plano)

- Spec coberta: marcação UI (T8), doors.json+/doors (T7), door_crossing puro+nó
  (T2-T5), máscara gated (T6), revert shim (T9), twist_mux/launch (T1/T5),
  fail-safes (T4 testes + T5 cola), campo (T11). ✔
- Tipos consistentes: `Cmd(state,vx,wz,door_id)` usado em T4/T5; payloads
  `{'doors':[{id,a,b}]}` e `{'state','door_id'}` idênticos em T5/T6/T7/T8. ✔
- Sem placeholders. ✔
