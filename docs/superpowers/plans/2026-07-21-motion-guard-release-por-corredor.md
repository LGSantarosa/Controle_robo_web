# motion_guard: release por corredor livre + probe blindado — Plano

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Trocar a vigília por-ponto-velho (`hold_still_max=20s`) por um release que solta quando o corredor à frente do plano fica livre 1,2s, com micro-passo de teste só se travar 10s por um retorno que não parece pessoa.

**Architecture:** `observe()` passa a calcular a ocupação de um corredor (todos os pontos do scan, menos parede mapeada) ao longo do rumo do `/plan`; `filter()` mantém o `blocked` enquanto esse corredor estiver ocupado e solta após `release_confirm`; se travar além de `probe_after` por um cluster não-pessoa, emite um creep curto. Tudo atrás da flag `release_by_corridor` (fail-open sem plano).

**Tech Stack:** Python puro (lógica testável sem ROS) em `robot_nav/motion_guard.py`; testes pytest em `test/test_motion_guard.py`; cola ROS2 (rclpy) em `main()`.

## Global Constraints

- Componente de segurança: **valida no sim antes da Pi** (regra da memória "mudança grande = mega revisar").
- **Sem `Co-Authored-By`** nos commits deste repo.
- `wz` NUNCA é escalado; no `blocked`/`probing` o giro é zerado (`wz=0.0`) — regra do dono.
- Tudo novo atrás de `release_by_corridor` (default `True`; `False` = comportamento pré-mudança exato).
- Fail-open: sem `/plan` fresco (`settle_plan_stale`) → cai no release temporal de hoje.
- Ré (`vx<0`) sempre passa, mesmo em `blocked`.
- Rodar a suíte inteira do pacote a cada task: `colcon test` não; usar pytest direto (mais rápido) — `python -m pytest ros2_packages/robot_nav/test/test_motion_guard.py -q`.

---

### Task 1: Parâmetros e estado novos

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/motion_guard.py` (`GuardConfig` ~147; `MotionGuard.__init__` ~216-243)
- Test: `ros2_packages/robot_nav/test/test_motion_guard.py`

**Interfaces:**
- Produces: `GuardConfig` ganha `release_by_corridor: bool=True`, `release_len: float=1.5`, `release_confirm: float=1.2`, `probe_after: float=10.0`, `probe_vx: float=0.05`, `probe_dist: float=0.15`, `probe_person_min_pts: int=5`, `probe_person_min_span: float=0.12`. `MotionGuard` ganha os campos `_pose`, `_corridor_occupied`, `_corridor_clear_since`, `_corridor_person_like`, `_blocked_since`, `_probe_start`, `_probe_done`.

- [ ] **Step 1: Escrever o teste que falha**

```python
def test_release_params_default():
    c = GuardConfig()
    assert c.release_by_corridor is True
    assert c.release_len == 1.5
    assert c.release_confirm == 1.2
    assert c.probe_after == 10.0
    assert c.probe_vx == 0.05
    assert c.probe_dist == 0.15
    assert c.probe_person_min_pts == 5
    assert c.probe_person_min_span == 0.12

def test_state_fields_init():
    g = _guard()
    assert g._corridor_occupied is False
    assert g._corridor_person_like is False
    assert g._probe_start is None
    assert g._probe_done is False
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `python -m pytest ros2_packages/robot_nav/test/test_motion_guard.py -q -k "release_params_default or state_fields_init"`
Expected: FAIL (`AttributeError` — atributos não existem)

- [ ] **Step 3: Adicionar os parâmetros ao `GuardConfig`**

Logo após `settle_lookahead: float = 0.6` (linha ~147), antes do fim do dataclass:

```python
    # RELEASE POR CORREDOR (07-21): soltar o blocked quando o corredor à
    # frente do PLANO fica livre, em vez de segurar pelo ponto velho da
    # vigília (falso-positivo de ~27s na run 07-20). Fecha o gap; a vigília
    # segurava enquanto sobrasse qualquer retorno a 0.5m do centróide velho.
    release_by_corridor: bool = True   # False = vigília antiga exata (rollback)
    release_len: float = 1.5           # m — alcance do corredor de release
    release_confirm: float = 1.2       # s — corredor limpo contínuo p/ soltar
    probe_after: float = 10.0          # s — travado antes do micro-passo
    probe_vx: float = 0.05             # m/s — creep do micro-passo de teste
    probe_dist: float = 0.15           # m — deslocamento máx de um probe
    probe_person_min_pts: int = 5      # cluster >= isto pontos = parece pessoa
    probe_person_min_span: float = 0.12  # m — extensão >= isto = parece pessoa
```

- [ ] **Step 4: Adicionar os campos de estado ao `__init__`**

Logo após `self._settle_since: float = -math.inf` (linha ~243):

```python
        # release por corredor (07-21)
        self._pose: 'Pt|None' = None            # última pose (odom) do observe
        self._corridor_occupied: bool = False   # há ponto não-parede no corredor
        self._corridor_clear_since: float = -math.inf  # t em que ficou livre
        self._corridor_person_like: bool = False  # ocupante parece pessoa
        self._blocked_since: float = -math.inf  # início do blocked contínuo
        self._probe_start: 'Pt|None' = None     # pose no início do creep
        self._probe_done: bool = False          # já gastou o probe_dist deste bloqueio
```

- [ ] **Step 5: Rodar e ver passar**

Run: `python -m pytest ros2_packages/robot_nav/test/test_motion_guard.py -q`
Expected: PASS (a suíte inteira segue verde)

- [ ] **Step 6: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/motion_guard.py ros2_packages/robot_nav/test/test_motion_guard.py
git commit -m "motion_guard: parametros e estado do release por corredor"
```

---

### Task 2: Ocupação do corredor no `observe()` + gate da vigília

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/motion_guard.py` (`observe` ~276, ~376, ~386-406; helper novo `_span`)
- Test: `ros2_packages/robot_nav/test/test_motion_guard.py`

**Interfaces:**
- Consumes: campos da Task 1; `self._plan_hdg`, `self._last_plan_t`, `self.map_tf`, `self.ghost_map`, `self._cluster`, `self.cfg`.
- Produces: após cada `observe()` com histórico, `self._corridor_occupied` (bool), `self._corridor_clear_since` (float), `self._corridor_person_like` (bool), `self._pose` setados. Com `release_by_corridor=True` a vigília (`_watch`) NÃO renova mais `_last_moving_t`. Helper `_span(cl) -> float` = diagonal do bbox do cluster.

- [ ] **Step 1: Escrever os testes que falham**

```python
def test_corridor_occupied_by_point_ahead():
    g = _guard()
    g.observe_plan(0.0, _plan(0.0))            # plano reto +x, fresco
    t = _feed_static(g)                        # WALL@x=2 (fora do release_len=1.5)
    g.observe_plan(t, _plan(0.0))
    g.observe(t, WALL + [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)], POSE, 0.0)
    assert g._corridor_occupied is True

def test_corridor_clear_when_only_far_wall():
    g = _guard()
    t = _feed_static(g)                        # só WALL@x=2, além de 1.5m
    g.observe_plan(t, _plan(0.0))
    g.observe(t, WALL, POSE, 0.0)
    assert g._corridor_occupied is False
    assert g._corridor_clear_since != math.inf

def test_vigilia_watch_empty_when_release_by_corridor():
    # com a flag ligada a vigília NÃO arma (o _watch nunca popula)
    g = _guard()                                  # release_by_corridor=True default
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]    # bloqueia (na bolha/corredor)
    _feed_mover(g, t, obj)
    assert g._watch == []

def test_vigilia_watch_arms_when_flag_off():
    g = _guard(release_by_corridor=False)
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    _feed_mover(g, t, obj)                        # móvel na bolha -> arma a vigília
    assert g._watch != []
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `python -m pytest ros2_packages/robot_nav/test/test_motion_guard.py -q -k "corridor or vigilia"`
Expected: FAIL (`_corridor_occupied` não muda; `_watch` ainda arma com a flag ligada)

- [ ] **Step 3: Guardar a pose no início do `observe()`**

Logo após `px, py, pyaw = pose` (linha ~276):

```python
        self._pose = (px, py)
```

- [ ] **Step 4: Gatear a vigília atrás da flag**

Envolver o bloco da vigília. O SET (dentro do `if self._consec >= c.persist_frames:`, linhas ~386-394 — o `if self.in_corridor or self.nearest_moving < c.freeze_dist:`) e o RENEW (`elif self._watch:`, ~395-406) passam a rodar só com a flag desligada. Trocar:

```python
            if self.in_corridor or self.nearest_moving < c.freeze_dist:
                self._watch = [ ... ]
                self._watch_since = t
                self._watch_corridor = self.in_corridor
        elif self._watch:
            if t - self._watch_since > c.hold_still_max:
                self._watch = []
            else:
                d = self._presence(pts, (px, py))
                ...
```

por (adicionar `not c.release_by_corridor and` na condição do SET, e `not c.release_by_corridor and self._watch` no `elif`):

```python
            if (not c.release_by_corridor
                    and (self.in_corridor or self.nearest_moving < c.freeze_dist)):
                self._watch = [
                    (sum(p[0] for p in cl) / len(cl),
                     sum(p[1] for p in cl) / len(cl)) for cl in clusters]
                self._watch_since = t
                self._watch_corridor = self.in_corridor
        elif self._watch and not c.release_by_corridor:
            if t - self._watch_since > c.hold_still_max:
                self._watch = []
            else:
                d = self._presence(pts, (px, py))
                if d is None:
                    self._watch = []
                else:
                    self._last_moving_t = t
                    self._last_nearest = d
                    if self._watch_corridor:
                        self._last_corridor_t = t
```

- [ ] **Step 5: Calcular a ocupação do corredor**

No FIM do `observe()`, logo após o bloco do `in_corridor` (após a linha ~376, antes do bloco de PERSISTÊNCIA em ~377), inserir:

```python
        # CORREDOR DE RELEASE (07-21): ocupação à frente do RUMO DO PLANO
        # com TODOS os pontos do scan (pessoa parada some do diff de
        # movimento mas continua no scan — é isso que tem que segurar). Sem
        # plano fresco cai no rumo do robô (fail-open).
        if (self._plan_hdg and t - self._last_plan_t <= c.settle_plan_stale
                and self.map_tf is not None):
            _, _, mc, ms = self.map_tf          # cos/sin do yaw map<-odom
            dir_odom = self._plan_hdg[-1][1] - math.atan2(ms, mc)
        else:
            dir_odom = pyaw
        cos_d, sin_d = math.cos(dir_odom), math.sin(dir_odom)
        occ: List[Pt] = []
        for p in pts:
            dx, dy = p[0] - px, p[1] - py
            xb = dx * cos_d + dy * sin_d
            yb = -dx * sin_d + dy * cos_d
            if not (0.0 < xb <= c.release_len and abs(yb) <= c.corridor_half_w):
                continue
            if ghost_ready and self.ghost_map.occupied_near(
                    tx + tc * p[0] - ts * p[1],
                    ty + ts * p[0] + tc * p[1], c.wall_near):
                continue                        # parede mapeada não é presença
            occ.append(p)
        self._corridor_occupied = bool(occ)
        if self._corridor_occupied:
            self._corridor_clear_since = math.inf
            occ_clusters = [cl for cl in self._cluster(occ)
                            if len(cl) >= c.min_cluster_points]
            self._corridor_person_like = any(
                len(cl) >= c.probe_person_min_pts
                or self._span(cl) >= c.probe_person_min_span
                for cl in occ_clusters)
        else:
            if self._corridor_clear_since == math.inf:
                self._corridor_clear_since = t
            self._corridor_person_like = False
```

- [ ] **Step 6: Adicionar o helper `_span`**

Logo após `_cluster` (~537), no corpo da classe `MotionGuard`:

```python
    @staticmethod
    def _span(cl: List[Pt]) -> float:
        """diagonal do bounding-box do cluster (m) — extensão espacial."""
        xs = [p[0] for p in cl]
        ys = [p[1] for p in cl]
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys))
```

- [ ] **Step 6b: Migrar os testes da vigília para a flag desligada**

A vigília agora só roda com `release_by_corridor=False`. Os 5 testes da seção
"vigília" (comentário na linha ~564) exercitam esse caminho legado — passam a
construir o guard com a flag desligada. Editar cada `_guard(...)` dessa seção:

- `test_stopped_blocker_still_there_keeps_blocking` (~575): `_guard()` → `_guard(release_by_corridor=False)`
- `test_stopped_blocker_leaving_releases_by_clear_time` (~586): `_guard(clear_time=1.5)` → `_guard(clear_time=1.5, release_by_corridor=False)`
- `test_stopped_blocker_watch_has_ceiling` (~600): `_guard(clear_time=1.5, hold_still_max=3.0)` → `_guard(clear_time=1.5, hold_still_max=3.0, release_by_corridor=False)`
- `test_stopped_far_lateral_mover_not_watched` (~613): `_guard(clear_time=1.5)` → `_guard(clear_time=1.5, release_by_corridor=False)`
- `test_watch_ignores_mapped_wall_points` (~627): `_guard(clear_time=1.5)` → `_guard(clear_time=1.5, release_by_corridor=False)`

(Os testes de settling e de fantasma-de-parede NÃO mudam: o corredor fica limpo
ou `_was_blocked` não é setado, então o gate novo fica transparente.)

- [ ] **Step 7: Rodar e ver passar**

Run: `python -m pytest ros2_packages/robot_nav/test/test_motion_guard.py -q`
Expected: PASS (novos + suíte antiga verdes)

- [ ] **Step 8: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/motion_guard.py ros2_packages/robot_nav/test/test_motion_guard.py
git commit -m "motion_guard: ocupacao do corredor do plano + gate da vigilia"
```

---

### Task 3: Release passivo no `filter()`

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/motion_guard.py` (`filter` ~432-475)
- Test: `ros2_packages/robot_nav/test/test_motion_guard.py`

**Interfaces:**
- Consumes: `self._corridor_occupied`, `self._corridor_clear_since`, `self._plan_hdg`, `self._last_plan_t`, `self._was_blocked`, `self.cfg.release_*`.
- Produces: `filter()` mantém `'blocked'` enquanto o corredor não confirmar livre por `release_confirm`, com `/plan` fresco e `release_by_corridor=True`; caso contrário segue o caminho de hoje (settle/slowing/idle). Mantém `self._blocked_since` (início do blocked contínuo, resetado no release).

- [ ] **Step 1: Escrever os testes que falham**

```python
def test_release_when_corridor_clears():
    # clear_time curto (freeze segura até ele; o release por corredor atua
    # DEPOIS). Pessoa sai -> solta em ~clear_time+release_confirm, não 20s.
    g = _guard(clear_time=1.5)
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    tl = _feed_mover(g, t, obj)
    g.observe_plan(tl, _plan(0.0))
    assert g.filter(tl + 0.1, 0.30, 0.0)[2] == 'blocked'   # latcha _was_blocked
    # corredor limpo + scans/plano frescos cobrindo o clear_time e o confirm
    last = tl + 0.1
    for k in range(1, 26):
        last = tl + 0.1 + k * 0.1
        g.observe(last, WALL, POSE, 0.0)
        g.observe_plan(last, _plan(0.0))
    assert g.filter(tl + 0.8, 0.30, 0.0)[2] == 'blocked'   # freeze ainda segura
    assert g.filter(last, 0.30, 0.0)[2] in ('slowing', 'idle')  # ~tl+2.6: soltou

def test_stay_blocked_while_person_stands():
    g = _guard(clear_time=1.5)
    t = _feed_static(g)
    person = [(1.0, y * 0.05 - 0.15) for y in range(8)]   # 8 pts, span ~0.35
    tl = _feed_mover(g, t, person)
    g.observe_plan(tl, _plan(0.0))
    assert g.filter(tl + 0.05, 0.30, 0.0)[2] == 'blocked'  # latcha _was_blocked
    # pessoa parada NO corredor por 25s (> hold_still_max antigo de 20s)
    last = tl + 0.05
    for k in range(1, 260):
        last = tl + k * 0.1
        g.observe(last, WALL + person, POSE, 0.0)
        g.observe_plan(last, _plan(0.0))
    assert g.filter(last + 0.05, 0.30, 0.0)[2] == 'blocked'

def test_fail_open_without_plan():
    # sem /plan o release por corredor não atua: solta pelo caminho temporal
    g = _guard(clear_time=1.5, settle_enabled=False)   # isola do settling
    t = _feed_static(g)
    obj = [(1.0, 0.0), (1.0, 0.1), (1.1, 0.0)]
    tl = _feed_mover(g, t, obj)
    assert g.filter(tl + 0.1, 0.30, 0.0)[2] == 'blocked'
    g.observe(tl + 0.1, WALL, POSE, 0.0)
    g.observe(tl + 2.4, WALL, POSE, 0.0)               # > clear_time, SEM plano
    assert g.filter(tl + 2.4, 0.30, 0.0)[2] in ('slowing', 'idle')
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `python -m pytest ros2_packages/robot_nav/test/test_motion_guard.py -q -k "release_when_corridor or stay_blocked_while_person or fail_open_without_plan"`
Expected: FAIL (`test_stay_blocked_while_person` solta cedo; sem o hold novo)

- [ ] **Step 3: Inserir o hold por corredor no `filter()`**

No `filter()`, dentro do bloco do freeze (linha ~449-451), marcar o início do blocked. Trocar:

```python
            self._was_blocked = True
            self._settle_since = -math.inf      # o relógio ainda nem venceu
            return (0.0 if vx > 0.0 else vx), 0.0, 'blocked'
```

por:

```python
            self._was_blocked = True
            self._settle_since = -math.inf      # o relógio ainda nem venceu
            if self._blocked_since == -math.inf:
                self._blocked_since = t
            return (0.0 if vx > 0.0 else vx), 0.0, 'blocked'
```

Depois, logo após esse bloco (após a linha ~451, ANTES do comentário "o clear_time venceu"), inserir o gate por corredor:

```python
        # RELEASE POR CORREDOR (07-21): o clear_time pode ter vencido, mas se
        # o robô esteve em blocked e o corredor à frente do plano ainda não
        # confirmou livre por release_confirm, SEGURA (substitui a vigília).
        # fail-open: sem /plan fresco cai no caminho temporal de hoje.
        plan_fresh = bool(self._plan_hdg) and t - self._last_plan_t <= c.settle_plan_stale
        if c.release_by_corridor and self._was_blocked and plan_fresh:
            clear_ok = (self._corridor_clear_since != math.inf
                        and t - self._corridor_clear_since >= c.release_confirm)
            if not clear_ok:
                if self._blocked_since == -math.inf:
                    self._blocked_since = t
                return (0.0 if vx > 0.0 else vx), 0.0, 'blocked'
        self._blocked_since = -math.inf
```

- [ ] **Step 4: Rodar e ver passar**

Run: `python -m pytest ros2_packages/robot_nav/test/test_motion_guard.py -q`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/motion_guard.py ros2_packages/robot_nav/test/test_motion_guard.py
git commit -m "motion_guard: release passivo quando o corredor do plano fica livre"
```

---

### Task 4: Micro-passo blindado (probe ativo)

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/motion_guard.py` (`filter` — dentro do gate da Task 3)
- Test: `ros2_packages/robot_nav/test/test_motion_guard.py`

**Interfaces:**
- Consumes: `self._corridor_occupied`, `self._corridor_person_like`, `self._blocked_since`, `self._pose`, `self._probe_start`, `self._probe_done`, `self.cfg.probe_*`.
- Produces: dentro do hold, se `t - _blocked_since > probe_after` E `_corridor_occupied` E não `_corridor_person_like` E o probe ainda não gastou `probe_dist`, `filter()` retorna `(probe_vx, 0.0, 'probing')`. Reset de `_probe_start/_probe_done` no release.

- [ ] **Step 1: Escrever os testes que falham**

```python
def _hold_occupied(g, obj, secs):
    """bloqueia e mantém `obj` no corredor por `secs`, plano fresco. Latcha
    _was_blocked/_blocked_since via um filter durante o freeze. Retorna t."""
    t = _feed_static(g)
    tl = _feed_mover(g, t, obj)
    g.observe_plan(tl, _plan(0.0))
    assert g.filter(tl + 0.1, 0.30, 0.0)[2] == 'blocked'   # latcha _blocked_since=tl+0.1
    last = tl + 0.1
    for k in range(1, int(secs / 0.1)):
        last = tl + 0.1 + k * 0.1
        g.observe(last, WALL + obj, POSE, 0.0)
        g.observe_plan(last, _plan(0.0))
    return last

def test_probe_fires_on_nonperson_after_timeout():
    g = _guard()
    ghost = [(1.0, 0.0), (1.0, 0.05), (1.02, 0.0)]   # 3 pts, span ~0.05 < 0.12
    last = _hold_occupied(g, ghost, secs=11.0)       # _blocked_since+11 > probe_after=10
    vx, wz, st = g.filter(last + 0.05, 0.30, 0.0)
    assert st == 'probing'
    assert 0.0 < vx <= g.cfg.probe_vx
    assert wz == 0.0

def test_no_probe_on_personlike():
    g = _guard()
    person = [(1.0, y * 0.05 - 0.15) for y in range(8)]  # 8 pts, span ~0.35
    last = _hold_occupied(g, person, secs=11.0)
    vx, wz, st = g.filter(last + 0.05, 0.30, 0.0)
    assert st == 'blocked'
    assert vx == 0.0

def test_no_probe_before_timeout():
    g = _guard()
    ghost = [(1.0, 0.0), (1.0, 0.05), (1.02, 0.0)]
    last = _hold_occupied(g, ghost, secs=5.0)        # < probe_after
    assert g.filter(last + 0.05, 0.30, 0.0)[2] == 'blocked'
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `python -m pytest ros2_packages/robot_nav/test/test_motion_guard.py -q -k probe`
Expected: FAIL (`test_probe_fires...` dá `'blocked'`, não `'probing'`)

- [ ] **Step 3: Inserir o probe dentro do gate**

No `filter()`, dentro do `if not clear_ok:` da Task 3, ANTES do `return ...'blocked'`, inserir o probe. Trocar:

```python
            if not clear_ok:
                if self._blocked_since == -math.inf:
                    self._blocked_since = t
                return (0.0 if vx > 0.0 else vx), 0.0, 'blocked'
```

por:

```python
            if not clear_ok:
                if self._blocked_since == -math.inf:
                    self._blocked_since = t
                # MICRO-PASSO BLINDADO: travado demais por retorno que NÃO
                # parece pessoa -> creep curto p/ desprender (fantasma/quina).
                # Se parece gente, NUNCA cutuca (segue blocked). Bolha dura /
                # collision_monitor cobrem o backstop físico do creep.
                if (t - self._blocked_since > c.probe_after
                        and self._corridor_occupied
                        and not self._corridor_person_like
                        and not self._probe_done):
                    if self._probe_start is None:
                        self._probe_start = self._pose
                    moved = None
                    if self._pose is not None and self._probe_start is not None:
                        moved = math.hypot(self._pose[0] - self._probe_start[0],
                                           self._pose[1] - self._probe_start[1])
                    if moved is not None and moved >= c.probe_dist:
                        self._probe_done = True   # gastou o passo; volta a parar
                    else:
                        return c.probe_vx, 0.0, 'probing'
                return (0.0 if vx > 0.0 else vx), 0.0, 'blocked'
```

E no reset do release (o `self._blocked_since = -math.inf` logo abaixo do gate), zerar também o probe. Trocar:

```python
        self._blocked_since = -math.inf
```

por:

```python
        self._blocked_since = -math.inf
        self._probe_start = None
        self._probe_done = False
```

- [ ] **Step 4: Rodar e ver passar**

Run: `python -m pytest ros2_packages/robot_nav/test/test_motion_guard.py -q`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/motion_guard.py ros2_packages/robot_nav/test/test_motion_guard.py
git commit -m "motion_guard: micro-passo blindado (probe) so em retorno nao-pessoa"
```

---

### Task 5: Cola ROS — parâmetros ao vivo e log

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/motion_guard.py` (`_CFG_PARAMS` ~603-613; log de boot ~670-678)

**Interfaces:**
- Consumes: os novos campos de `GuardConfig`.
- Produces: os 8 params novos declarados e afináveis ao vivo (`ros2 param set`); linha de boot menciona o modo. Sem novos tópicos. `main()` é `# pragma: no cover` (validar no sim, não em teste unitário).

- [ ] **Step 1: Declarar os params novos como afináveis**

Em `_CFG_PARAMS` (~603), acrescentar ao final da tupla, após `'settle_plan_stale', 'settle_lookahead'`:

```python
                       'settle_plan_stale', 'settle_lookahead',
                       'release_by_corridor', 'release_len', 'release_confirm',
                       'probe_after', 'probe_vx', 'probe_dist',
                       'probe_person_min_pts', 'probe_person_min_span')
```

- [ ] **Step 2: Mencionar o modo no log de boot**

No `get_logger().info(...)` de boot (~670-678), acrescentar ao final do format string e dos args:

```python
            self.get_logger().info(
                'motion_guard ativo: raio %.1fm, corredor %.2fx%.1fm, '
                'slow %.0f%%@%.1fm..100%%@%.1fm, clear %.1fs, '
                'settle %s %.1f°/%.1fs, release %s %.1fm/%.1fs' % (
                    cfg.guard_radius, cfg.corridor_half_w * 2,
                    cfg.corridor_len, cfg.slow_scale * 100, cfg.slow_dist,
                    cfg.guard_radius, cfg.clear_time,
                    'on' if cfg.settle_enabled else 'off',
                    cfg.settle_tol_deg, cfg.settle_max,
                    'corridor' if cfg.release_by_corridor else 'vigilia',
                    cfg.release_len, cfg.release_confirm))
```

- [ ] **Step 3: Sanidade de import/compilação**

Run: `python -c "import ast; ast.parse(open('ros2_packages/robot_nav/robot_nav/motion_guard.py').read()); print('ok')"`
Expected: `ok`

Run: `python -m pytest ros2_packages/robot_nav/test/test_motion_guard.py -q`
Expected: PASS (suíte inteira)

- [ ] **Step 4: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/motion_guard.py
git commit -m "motion_guard: params do release por corredor afinaveis ao vivo + log"
```

---

## Validação no sim (pós-plano, antes da Pi)

Não é task de código, mas obrigatório antes de deployar (regra da memória):

1. `colcon build --packages-select robot_nav` + rodar a stack no sim.
2. Reproduzir o cenário da run 07-20: pessoa entra no corredor (bloqueia), sai andando → confirmar release em ~1,2s (não 20s) e SEM curva torta (settle segue atuando).
3. Pessoa para e fica no corredor → confirmar que segue `blocked` (não cutuca).
4. Fantasma/quina prendendo além de 10s → confirmar 1 creep de ~0,15m e re-avaliação.
5. Comparar `motion_guard.csv`: sumiço do `blocked` sustentado com `n_moving=0` por dezenas de segundos.

## Self-review (coberto)

- **Spec → tasks:** corredor do plano (T2), release passivo 1,2s (T3), probe blindado 10s/não-pessoa (T4), params/flag/fail-open (T1/T3/T5), testes (todas). ✔
- **Fora de escopo mantido fora:** #1 do arranque não é tocado. ✔
- **Tipos consistentes:** `_corridor_clear_since` sentinela `math.inf`=ocupado em todo o código; `_span` retorna float; `probe_vx`/`probe_dist` floats. ✔
