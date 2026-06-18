# Door positioning via nav2 — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Trocar a aproximação reativa do door_crossing (que raspa/ping-pongა colada na porta) por: nav2 leva o robô a um waypoint pré-porta (W, no eixo, recuado, centrado) e o door só alinha+cruza desse ponto seguro.

**Architecture:** A máquina de estados pura (`DoorCrossing`) deixa de DIRIGIR a aproximação e passa a EMITIR um pedido de navegação (`Cmd.nav`) que o nó executa via cliente de action `navigate_to_pose`. Estados: `idle → positioning → rotating → crossing`. `staging`/`reversing` e toda a máquina de escape são removidos. O nó captura o destino do usuário (G) de `/goal_pose`, manda W, espera o resultado, e re-manda G ao cruzar.

**Tech Stack:** Python, ROS 2 Jazzy, rclpy, nav2_msgs (NavigateToPose action), geometry_msgs (PoseStamped), pytest. Pacote `ros2_packages/robot_nav`.

## Global Constraints

- Skid-steer: NUNCA giro em arco pra alinhar; alinhamento = point-turn no lugar. Correção lateral suave durante o `crossing` (cross_k_lat) é OK (já existe).
- `door_vel` é prio 20 no twist_mux (acima do nav). Em estado PASSIVO (`idle`, `positioning`) o nó NÃO publica `door_vel`, senão congela o nav2.
- Parâmetros novos do `DoorCrossConfig` afináveis ao vivo (mutados na mesma ref `self.cfg`); rate_hz fica de fora.
- Sem rodapé de autoria nos commits (preferência do repo).
- Rodar testes: `cd ros2_packages/robot_nav && python3 -m pytest test/test_door_crossing.py -q`.
- Baseline do "último bom": `89d08c7`. Backup antes de mexer.

---

### Task 1: `Cmd.nav` + cálculo do waypoint W

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (dataclass `Cmd`; nova função `pre_door_waypoint`)
- Test: `ros2_packages/robot_nav/test/test_door_crossing.py`

**Interfaces:**
- Produces: `Cmd(state, vx, wz, door_id, nav=None)` — `nav` ∈ `None | ('goto', (x,y,yaw)) | ('cancel',)`.
- Produces: `pre_door_waypoint(g: DoorGeom, side: int, standoff: float) -> (x, y, yaw)` — W no eixo, recuado `standoff` do centro no lado `side`, orientação = heading de travessia.

- [ ] **Step 1: Write the failing test**

```python
from robot_nav.door_crossing import pre_door_waypoint, door_geometry, Cmd

def test_pre_door_waypoint_no_eixo_recuado_de_frente():
    g = door_geometry((1.0, 2.0), (2.0, 2.0))   # centro (1.5,2.0), normal (0,1)
    # side=+1: aproxima de baixo (y<2). W fica 1.0m ABAIXO do centro, no eixo,
    # de frente pra porta (yaw=+pi/2).
    x, y, yaw = pre_door_waypoint(g, side=+1, standoff=1.0)
    assert (x, y) == pytest.approx((1.5, 1.0))
    assert yaw == pytest.approx(math.pi / 2)
    # side=-1: aproxima de cima; W 1.0m ACIMA, de frente (yaw=-pi/2)
    x2, y2, yaw2 = pre_door_waypoint(g, side=-1, standoff=1.0)
    assert (x2, y2) == pytest.approx((1.5, 3.0))
    assert yaw2 == pytest.approx(-math.pi / 2)

def test_cmd_nav_default_none():
    assert Cmd('idle', 0.0, 0.0, None).nav is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_door_crossing.py::test_pre_door_waypoint_no_eixo_recuado_de_frente -v`
Expected: FAIL (`pre_door_waypoint` não existe / `Cmd` sem `nav`).

- [ ] **Step 3: Write minimal implementation**

No `Cmd` (dataclass), adicionar campo: `nav: object = None`.

Nova função (perto de `crossing_yaw`):

```python
def pre_door_waypoint(g: DoorGeom, side: int, standoff: float):
    """Waypoint pré-porta: no eixo, recuado `standoff` do centro no lado de
    aproximação `side`, orientação = heading de travessia (de frente pra porta).
    É a POSIÇÃO que o nav2 entrega; o alinhamento fino fica com o door."""
    x = g.cx - g.nx * side * standoff
    y = g.cy - g.ny * side * standoff
    return x, y, crossing_yaw(g, side)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_door_crossing.py -k "pre_door_waypoint or cmd_nav" -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/
git commit -m "feat(door): Cmd.nav + pre_door_waypoint (waypoint pre-porta no eixo)"
```

---

### Task 2: `idle → positioning` manda W

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (`DoorCrossing.__init__`, `DoorCrossConfig`, `DoorCrossing.update` bloco idle)
- Test: `ros2_packages/robot_nav/test/test_door_crossing.py`

**Interfaces:**
- Consumes: `pre_door_waypoint`, `Cmd.nav` (Task 1).
- Produces: `update(... goal_g=None, wp_status='idle', plan=None)` — `goal_g`=(x,y,yaw) do destino do usuário; `wp_status` ∈ `'idle'|'active'|'succeeded'|'aborted'`.
- Produces: config `wp_standoff: float = 1.0`, `wp_retries: int = 2`, `wp_timeout: float = 30.0`.
- Produces: ao armar, `Cmd('positioning', 0.0, 0.0, door_id, nav=('goto', W))`; salva `self._goal_g`, `self._wp_t0`, `self._wp_tries=0`.

- [ ] **Step 1: Write the failing test**

```python
def test_arma_manda_waypoint_e_vai_pro_positioning():
    dc = mk()
    plan = [(1.5, 1.0), (1.5, 3.0)]          # cruza a porta -> arma
    G = (1.5, 5.0, math.pi / 2)              # destino além da porta
    c = dc.update(0.0, (1.5, 1.0, math.pi/2), [DOOR], True, True,
                  math.inf, True, goal_g=G, wp_status='idle', plan=plan)
    assert c.state == 'positioning'
    assert c.vx == 0.0 and c.wz == 0.0       # mãos quietas: nav2 dirige
    assert c.nav[0] == 'goto'
    wx, wy, wyaw = c.nav[1]
    assert (wx, wy) == pytest.approx((1.5, 2.0 - 1.0))   # W = eixo, 1m antes
    assert wyaw == pytest.approx(math.pi / 2)

def test_nao_arma_sem_goal_g():
    dc = mk()
    plan = [(1.5, 1.0), (1.5, 3.0)]
    c = dc.update(0.0, (1.5, 1.0, math.pi/2), [DOOR], True, True,
                  math.inf, True, goal_g=None, wp_status='idle', plan=plan)
    assert c.state == 'idle' and c.nav is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_door_crossing.py -k "arma_manda_waypoint or nao_arma_sem_goal" -v`
Expected: FAIL (assinatura de `update` sem `goal_g`/`wp_status`; estado `positioning` não existe).

- [ ] **Step 3: Write minimal implementation**

Em `DoorCrossConfig` adicionar: `wp_standoff: float = 1.0`, `wp_retries: int = 2`, `wp_timeout: float = 30.0`.

Em `DoorCrossing.__init__`/`_to_idle` inicializar: `self._goal_g = None`, `self._wp_t0 = 0.0`, `self._wp_tries = 0`.

Mudar assinatura: `def update(self, now, pose, doors, goal_active, nav_forward, gap, scan_fresh, goal_g=None, wp_status='idle', plan=None)`.

No bloco `idle`, condição de armar passa a exigir `goal_g is not None`. Ao armar:

```python
door, geom = self._pick_door(pose, doors, plan)
if door is None or goal_g is None:
    return Cmd('idle', 0.0, 0.0, None)
x, y, _ = pose
raw_s = (x - geom.cx) * geom.nx + (y - geom.cy) * geom.ny
self.side = -1 if raw_s > 0 else +1
self.door, self.geom = door, geom
self._goal_g = goal_g
self._wp_tries = 0
self._wp_t0 = now
self.state = 'positioning'
W = pre_door_waypoint(geom, self.side, cfg.wp_standoff)
return Cmd('positioning', 0.0, 0.0, door['id'], nav=('goto', W))
```

Remover o `self.t_start`/`_align_t0`/`_align_anchor` do antigo arme (vão sair com o staging na Task 6; por ora não referenciar).

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_door_crossing.py -k "arma_manda_waypoint or nao_arma_sem_goal" -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/
git commit -m "feat(door): idle arma mandando W e entra em positioning"
```

---

### Task 3: estado `positioning` — takeover / retry / desiste

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (`update`: bloco `positioning`)
- Test: `ros2_packages/robot_nav/test/test_door_crossing.py`

**Interfaces:**
- Consumes: `wp_status`, `goal_g`, `cfg.wp_retries`, `cfg.wp_timeout`.
- Produces: `positioning` → `rotating` quando `wp_status=='succeeded'`; em `'aborted'` ou timeout re-manda W (`nav=('goto',W)`) até `wp_retries`; estourou → `idle` (door_zone 'failed' fica no nó). Novo `goal_g` ≠ salvo → cancela e re-avalia.

- [ ] **Step 1: Write the failing test**

```python
def _ate_positioning(dc, t=0.0):
    plan = [(1.5, 1.0), (1.5, 3.0)]
    G = (1.5, 5.0, math.pi / 2)
    c = dc.update(t, (1.5, 1.0, math.pi/2), [DOOR], True, True,
                  math.inf, True, goal_g=G, wp_status='idle', plan=plan)
    assert c.state == 'positioning'
    return G

def _pos(dc, t, wp_status, G, pose=(1.5, 1.0, math.pi/2)):
    return dc.update(t, pose, [DOOR], True, True, math.inf, True,
                     goal_g=G, wp_status=wp_status, plan=[(1.5,1.0),(1.5,3.0)])

def test_positioning_succeeded_vai_pro_rotating():
    dc = mk(); G = _ate_positioning(dc)
    c = _pos(dc, 0.1, 'succeeded', G)
    assert c.state == 'rotating'

def test_positioning_aborted_remanda_w_ate_o_limite():
    dc = mk(); G = _ate_positioning(dc)
    c = _pos(dc, 0.1, 'aborted', G)        # 1a falha -> retry
    assert c.state == 'positioning' and c.nav[0] == 'goto'
    c = _pos(dc, 0.2, 'aborted', G)        # 2a falha -> retry (wp_retries=2)
    assert c.state == 'positioning' and c.nav[0] == 'goto'
    c = _pos(dc, 0.3, 'aborted', G)        # estourou -> desiste
    assert c.state == 'idle'

def test_positioning_timeout_conta_como_falha():
    dc = mk(); G = _ate_positioning(dc)
    c = _pos(dc, CFG.wp_timeout + 0.1, 'active', G)  # nunca chegou
    assert c.state == 'positioning' and c.nav[0] == 'goto'   # re-mandou

def test_positioning_novo_goal_cancela():
    dc = mk(); G = _ate_positioning(dc)
    G2 = (9.0, 9.0, 0.0)
    c = _pos(dc, 0.1, 'active', G2)        # destino mudou
    assert c.state == 'idle' and c.nav == ('cancel',)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_door_crossing.py -k positioning -v`
Expected: FAIL (bloco `positioning` não existe).

- [ ] **Step 3: Write minimal implementation**

No `update`, depois das guardas comuns (pose/goal/scan) e ANTES dos blocos de movimento, tratar `positioning`:

```python
if self.state == 'positioning':
    if goal_g is not None and self._goal_g is not None and \
            _pose_changed(goal_g, self._goal_g):
        return self._abort_to_idle(now, nav=('cancel',))
    if wp_status == 'succeeded':
        self.state = 'rotating'
        self._rot_dir = 0
        return Cmd('rotating', 0.0, 0.0, self.door['id'])
    if wp_status == 'aborted' or (now - self._wp_t0) > cfg.wp_timeout:
        self._wp_tries += 1
        if self._wp_tries > cfg.wp_retries:
            return self._abort_to_idle(now)     # desiste; nó publica 'failed'
        self._wp_t0 = now
        W = pre_door_waypoint(self.geom, self.side, cfg.wp_standoff)
        return Cmd('positioning', 0.0, 0.0, self.door['id'], nav=('goto', W))
    return Cmd('positioning', 0.0, 0.0, self.door['id'])   # esperando nav2
```

Helpers (adicionar):

```python
def _abort_to_idle(self, now, nav=None):
    self.state = 'idle'
    self.door = None
    self.geom = None
    self._goal_g = None
    self._cooldown_until = now + self.cfg.retrigger_cooldown
    return Cmd('idle', 0.0, 0.0, None, nav=nav)

def _pose_changed(a, b, tol=0.05):   # módulo-level helper
    return (abs(a[0]-b[0]) > tol or abs(a[1]-b[1]) > tol
            or abs(_wrap(a[2]-b[2])) > 0.1)
```

Nota: a guarda comum "pose/goal/scan" no topo do `update` deve, em `positioning`, abortar com `nav=('cancel',)` se perder goal/pose (cancela o W pendente). Ajustar a guarda pra usar `_abort_to_idle(now, nav=('cancel',))` quando `self.state` já saiu de idle.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_door_crossing.py -k positioning -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/
git commit -m "feat(door): positioning espera nav2 (succeeded->rotating, aborta/timeout->retry, desiste)"
```

---

### Task 4: `rotating` sem escape, só alinha → `crossing`

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (bloco `rotating`)
- Test: `ros2_packages/robot_nav/test/test_door_crossing.py`

**Interfaces:**
- Consumes: estado `rotating` vindo do `positioning`.
- Produces: `rotating` faz point-turn pro heading de travessia (lógica de hoje: freio perto do alvo, sentido único); alinhado+parado → `crossing`. SEM `_maybe_escape`, SEM `d>fit→staging` (staging não existe mais).

- [ ] **Step 1: Write the failing test**

```python
def test_rotating_alinha_e_vai_pro_crossing():
    dc = mk(); G = _ate_positioning(dc)
    _pos(dc, 0.1, 'succeeded', G)            # -> rotating
    # robô em W (centrado), girando pra alinhar; depois reto e parado
    pose = (1.5, 1.0, math.pi/2 - 0.5)       # 28° torto
    dc.update(0.2, pose, [DOOR], True, True, math.inf, True, goal_g=G,
              wp_status='idle', plan=[(1.5,1.0),(1.5,3.0)])
    aligned = (1.5, 1.0, math.pi/2)
    dc.update(0.25, aligned, [DOOR], True, True, math.inf, True, goal_g=G,
              wp_status='idle', plan=[(1.5,1.0),(1.5,3.0)])   # taxa alta ainda
    c = dc.update(0.30, aligned, [DOOR], True, True, math.inf, True, goal_g=G,
                  wp_status='idle', plan=[(1.5,1.0),(1.5,3.0)])  # assentou
    assert c.state == 'crossing'
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_door_crossing.py::test_rotating_alinha_e_vai_pro_crossing -v`
Expected: FAIL (rotating ainda chama `_maybe_escape`/usa `staging`, ou a universal exige fit que W não satisfaz).

- [ ] **Step 3: Write minimal implementation**

No bloco `rotating`: remover a chamada `_maybe_escape(...)` e o ramo `if abs(d) > fit: self.state = 'staging'`. Manter o resto (alinhar com freio, sentido único, parar quando `|yaw_err|<=align_yaw`).

A transição pro `crossing` (alinhado+parado) NÃO exige mais `|d|<=fit` (o robô está em W, ainda pode estar lateralmente fora da tolerância do nav2; o `crossing` corrige isso na aproximação — Task 5). Trocar a "checagem universal passo-reto" por uma checagem de alinhamento simples DENTRO do rotating:

```python
if abs(yaw_err) <= cfg.align_yaw and yaw_rate <= cfg.cross_yaw_rate_max:
    self.state = 'crossing'
    self._hold_t0 = None
    return Cmd('crossing', cfg.cross_speed, 0.0, self.door['id'])
```

Remover a "checagem universal" antiga (que rodava em staging/rotating) — sem staging ela só existe pro rotating, e vira a linha acima.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_door_crossing.py::test_rotating_alinha_e_vai_pro_crossing -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/
git commit -m "feat(door): rotating so alinha (sem escape/staging) -> crossing"
```

---

### Task 5: `crossing` desde W — corrige na aproximação, trava de segurança, re-manda G

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (bloco `crossing`; config `jamb_safety`)
- Test: `ros2_packages/robot_nav/test/test_door_crossing.py`

**Interfaces:**
- Consumes: `crossing` vindo do `rotating`, partindo de W (~1m antes, `s≈-1.0`).
- Produces: corrige lateral/yaw andando (`cross_k_lat`/`cross_k_yaw`, cap `cross_wz_max`); TRAVA DE SEGURANÇA: ao chegar perto dos batentes (`s > -cfg.jamb_safety`) exige `|d|<=fit`, senão ABORTA (`nav=('cancel',)`, volta a `positioning` re-mandando W); zona de parada (caminho B) e release (`s>exit_margin`) como hoje; ao soltar, `nav=('goto', G)`.
- Produces: config `jamb_safety: float = 0.25`.

- [ ] **Step 1: Write the failing test**

```python
def _into_crossing_from_w(dc):
    G = _ate_positioning(dc)
    _pos(dc, 0.1, 'succeeded', G)
    dc.update(0.2, (1.5, 1.0, math.pi/2), [DOOR], True, True, math.inf, True,
              goal_g=G, wp_status='idle', plan=[(1.5,1.0),(1.5,3.0)])
    c = dc.update(0.25, (1.5, 1.0, math.pi/2), [DOOR], True, True, math.inf,
                  True, goal_g=G, wp_status='idle', plan=[(1.5,1.0),(1.5,3.0)])
    assert c.state == 'crossing'
    return G

def test_crossing_desde_w_corrige_lateral_andando():
    # em W, 15cm fora do eixo, alinhado: anda corrigindo (vx>0, wz reduz o lat)
    dc = mk(); G = _into_crossing_from_w(dc)
    c = dc.update(0.4, (1.5 + 0.15, 1.2, math.pi/2), [DOOR], True, True,
                  math.inf, True, goal_g=G, wp_status='idle',
                  plan=[(1.5,1.0),(1.5,3.0)])
    assert c.state == 'crossing' and c.vx == pytest.approx(CFG.cross_speed)
    assert c.wz < 0          # corrige o +15cm pro eixo

def test_crossing_aborta_se_descentrado_perto_dos_batentes():
    # perto dos batentes (s=-0.1 > -jamb_safety) e ainda 18cm fora -> ABORTA
    dc = mk(); G = _into_crossing_from_w(dc)
    c = dc.update(0.4, (1.5 + 0.18, 1.9, math.pi/2), [DOOR], True, True,
                  math.inf, True, goal_g=G, wp_status='idle',
                  plan=[(1.5,1.0),(1.5,3.0)])
    assert c.state == 'positioning'      # volta a re-posicionar
    assert c.nav == ('goto', pre_door_waypoint(dc.geom, dc.side, CFG.wp_standoff)) \
        or c.nav[0] == 'goto'

def test_crossing_solta_remandando_g():
    dc = mk(); G = _into_crossing_from_w(dc)
    c = dc.update(0.5, (1.5, 2.0 + CFG.exit_margin + 0.05, math.pi/2), [DOOR],
                  True, True, math.inf, True, goal_g=G, wp_status='idle',
                  plan=[(1.5,1.0),(1.5,3.0)])
    assert c.state == 'idle'
    assert c.nav == ('goto', G)          # continua pro destino do usuário
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_door_crossing.py -k "crossing_desde_w or descentrado or solta_remandando" -v`
Expected: FAIL.

- [ ] **Step 3: Write minimal implementation**

Em `DoorCrossConfig` adicionar `jamb_safety: float = 0.25`.

Reescrever o bloco `crossing` (a ordem: release primeiro já está; adicionar a trava antes da correção):

```python
if self.state == 'crossing':
    if s > cfg.exit_margin:                       # passou dos batentes -> SOLTA
        g = Cmd('idle', 0.0, 0.0, None, nav=('goto', self._goal_g))
        self._to_idle_success(now)
        return g
    if gap < cfg.stop_dist:                        # caminho B (pessoa) no vão
        if self._hold_t0 is None:
            self._hold_t0 = now
        elif now - self._hold_t0 > cfg.stop_hold_timeout:
            return self._abort_to_idle(now)
        return Cmd('crossing', 0.0, 0.0, self.door['id'])
    self._hold_t0 = None
    fit = fit_lat(g_geom, cfg.robot_half_width, cfg.fit_margin)
    if s > -cfg.jamb_safety and abs(d) > fit:      # descentrado perto dos batentes
        self._wp_tries = 0
        self._wp_t0 = now
        self.state = 'positioning'
        W = pre_door_waypoint(self.geom, self.side, cfg.wp_standoff)
        return Cmd('positioning', 0.0, 0.0, self.door['id'], nav=('goto', W))
    wz = -cfg.cross_k_lat * d - cfg.cross_k_yaw * yaw_err
    wz = max(-cfg.cross_wz_max, min(cfg.cross_wz_max, wz))
    return Cmd('crossing', cfg.cross_speed, wz, self.door['id'])
```

(`g_geom` = `self.geom`; usar o nome de var já existente no `update`. Ajustar.)

Helper de sucesso (re-mandar G é responsabilidade do retorno; o estado some):

```python
def _to_idle_success(self, now):
    self.state = 'idle'
    self.door = None
    self.geom = None
    self._goal_g = None
    self._cooldown_until = now + self.cfg.success_cooldown
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_door_crossing.py -k "crossing" -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/
git commit -m "feat(door): crossing desde W (corrige andando, trava de seguranca perto do batente, re-manda G)"
```

---

### Task 6: deletar `staging`/`reversing`/escape + podar params e testes

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (remover blocos/params/helpers órfãos)
- Modify: `ros2_packages/robot_nav/test/test_door_crossing.py` (remover/adaptar testes que usavam staging/reversing/escape/standoff)

**Interfaces:**
- Produces: máquina com só `idle/positioning/rotating/crossing`. Sem `_maybe_escape`, sem estados `staging`/`reversing`, sem params de escape/standoff/substuck.

- [ ] **Step 1: Mapear o que sai (rodar p/ achar referências)**

Run: `grep -nE "staging|reversing|_maybe_escape|escape_|substuck|_align_anchor|_align_t0|turn_standoff|stage_dist|stage_speed|stage_k_heading|_esc_start|_esc_target|_escape_count|align_progress_radius" ros2_packages/robot_nav/robot_nav/door_crossing.py`
Expected: lista das linhas a remover.

- [ ] **Step 2: Remover do código**

Remover: os blocos `if self.state == 'staging':` e `if self.state == 'reversing':`; o método `_maybe_escape`; os campos `_esc_start/_esc_target/_escape_count/_align_anchor/_align_t0`; do `DoorCrossConfig` os campos `stage_dist, stage_tol, stage_speed, stage_k_heading, align_lat, align_stable, turn_standoff, escape_front_gap, escape_substuck_time, escape_reverse_dist, escape_reverse_speed, escape_rear_margin, escape_rear_min, escape_max_count, align_progress_radius` (manter só o que `positioning/rotating/crossing` usam). No nó: tirar esses nomes do `declare`, da construção do `DoorCrossConfig`, das listas `_CFG_PARAMS`; e `front_gap`/`rear_gap` (eram só pro escape) saem da chamada do `update` e do cálculo no `_tick`.

Atenção: a guarda `total_timeout`/`align_timeout` — manter `total_timeout` como rede de segurança global (em `positioning`/`crossing`). Remover `align_timeout` (era staging+rotating).

- [ ] **Step 3: Adaptar/remover testes órfãos**

Run: `grep -nE "staging|reversing|escape|standoff|_ate_crossing|_into_rotating|gap=0\.05" ros2_packages/robot_nav/test/test_door_crossing.py`
Remover os testes especificamente de staging/reversing/escape/standoff (`test_staging_*`, `test_*reversing*`, `test_*re_de_escape*`, `test_staging_re_pra_ganhar_standoff*`, `test_staging_gira_quando_longe*`, `test_colado_mas_ja_alinhado*`, `test_arma_e_vai_pro_staging`, `test_staging_converge*`, `test_rotating_drift_pequeno*` se dependia de staging). Adaptar helpers (`_ate_crossing`/`_into_rotating`) pro novo fluxo (via `_into_crossing_from_w`).

- [ ] **Step 4: Rodar TODA a suite**

Run: `python3 -m pytest test/test_door_crossing.py -q`
Expected: PASS (sem refs órfãs; contagem menor que 60 — testes de staging/escape removidos, novos de positioning/W somados).

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/
git commit -m "refactor(door): deleta staging/reversing/escape — fluxo linear idle->positioning->rotating->crossing"
```

---

### Task 7: nó — cliente de action nav2 + captura de `/goal_pose` + executar `Cmd.nav`

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (`DoorCrossingNode`: imports, `__init__`, callbacks, `_tick`)

**Interfaces:**
- Consumes: `Cmd.nav` da máquina; `update(... goal_g=, wp_status=)`.
- Produces: cliente `ActionClient(self, NavigateToPose, 'navigate_to_pose')`; `self._goal_g` de `/goal_pose`; `self._wp_status` atualizado por callbacks de resultado; `_send_nav_goal(pose)` / `_cancel_nav_goal()`.

- [ ] **Step 1: Imports e membros**

Adicionar imports: `from nav2_msgs.action import NavigateToPose`, `from rclpy.action import ActionClient`, `from geometry_msgs.msg import PoseStamped`. (yaw→quat: usar helper inverso de `quat_to_yaw`, criar `yaw_to_quat(yaw)`.)

No `__init__`: `self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')`; `self._goal_g = None`; `self._wp_status = 'idle'`; `self._wp_handle = None`. Subscrição: `self.create_subscription(PoseStamped, '/goal_pose', self._on_goal_pose, 10)`.

- [ ] **Step 2: Callbacks de goal/resultado**

```python
def _on_goal_pose(self, msg):
    q = msg.pose.orientation
    self._goal_g = (msg.pose.position.x, msg.pose.position.y,
                    quat_to_yaw(q.x, q.y, q.z, q.w))

def _send_nav_goal(self, pose):
    if not self._nav_client.wait_for_server(timeout_sec=0.0):
        self.get_logger().warn('navigate_to_pose indisponível'); return
    x, y, yaw = pose
    g = NavigateToPose.Goal()
    g.pose.header.frame_id = 'map'
    g.pose.pose.position.x = x; g.pose.pose.position.y = y
    qx, qy, qz, qw = yaw_to_quat(yaw)
    g.pose.pose.orientation.x = qx; g.pose.pose.orientation.y = qy
    g.pose.pose.orientation.z = qz; g.pose.pose.orientation.w = qw
    self._wp_status = 'active'
    fut = self._nav_client.send_goal_async(g)
    fut.add_done_callback(self._on_wp_accepted)

def _on_wp_accepted(self, fut):
    h = fut.result()
    if not h.accepted:
        self._wp_status = 'aborted'; return
    self._wp_handle = h
    h.get_result_async().add_done_callback(self._on_wp_result)

def _on_wp_result(self, fut):
    st = fut.result().status
    self._wp_status = 'succeeded' if st == 4 else 'aborted'  # 4=SUCCEEDED

def _cancel_nav_goal(self):
    if self._wp_handle is not None:
        self._wp_handle.cancel_goal_async()
        self._wp_handle = None
    self._wp_status = 'idle'
```

- [ ] **Step 3: Executar `Cmd.nav` no `_tick`**

Depois de obter `cmd = self.sup.update(...)` (agora passando `goal_g=self._goal_g, wp_status=self._wp_status`), executar o pedido:

```python
if cmd.nav is not None:
    if cmd.nav[0] == 'goto':
        self._send_nav_goal(cmd.nav[1])
    elif cmd.nav[0] == 'cancel':
        self._cancel_nav_goal()
# ao voltar pra idle/positioning(novo W), zerar o status consumido
if cmd.state in ('idle', 'rotating', 'crossing'):
    self._wp_status = 'idle'
```

(Cuidado: só re-zere `_wp_status` DEPOIS de consumido pela transição, pra não perder o 'succeeded' do mesmo tick. Reordenar conforme necessário — o `update` lê `wp_status` ANTES; zerar depois é seguro.)

- [ ] **Step 4: yaw_to_quat + smoke de import**

Adicionar `def yaw_to_quat(yaw): return (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2))` (módulo-level).

Run: `cd ros2_packages/robot_nav && python3 -c "import robot_nav.door_crossing"`
Expected: sem ImportError (nav2_msgs/rclpy.action disponíveis no ambiente ROS; rodar com `source /opt/ros/jazzy/setup.bash`).

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/
git commit -m "feat(door): no vira cliente de action nav2 (manda W, captura G de /goal_pose, executa Cmd.nav)"
```

---

### Task 8: nó — não publicar `door_vel` em `positioning` + door_zone/dbg + params novos

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (`_tick` publish; `declare`/`_CFG_PARAMS`; dbg states; door_zone)

**Interfaces:**
- Produces: publica `door_vel` SÓ em `rotating`/`crossing` (e zero na transição PRA SAIR delas). `positioning`/`idle` passivos. door_zone publica `positioning`/`failed`. Params `wp_standoff`/`wp_retries`/`wp_timeout` no declare + live-tune.

- [ ] **Step 1: Publish só nos estados ativos**

Trocar a condição de publish (linha ~895) por:

```python
ACTIVE_DRV = ('rotating', 'crossing')
if cmd.state in ACTIVE_DRV or prev in ACTIVE_DRV:
    t = Twist(); t.linear.x = cmd.vx; t.angular.z = cmd.wz
    self.pub.publish(t)
```

Isso garante que em `positioning` o nó NÃO segura o twist_mux (nav2 dirige), e publica um zero só ao SAIR de rotating/crossing.

- [ ] **Step 2: door_zone e dbg**

No `_publish_zone`: `cmd.state in ('positioning','rotating','crossing')` publica o estado; idle com porta na zona publica 'approaching'; quando a máquina desiste (Task 3, retorno idle após estourar retries) publicar 'failed' uma vez — detectar pela transição `prev=='positioning' and cmd.state=='idle' and cmd.nav is None`. dbg: trocar a lista `('staging','rotating','crossing','reversing')` por `('rotating','crossing')` (positioning não tem vx/wz úteis; logar s/yaw no positioning é opcional).

- [ ] **Step 3: Params novos + podar os velhos**

No `declare`: adicionar `('wp_standoff', 1.0), ('wp_retries', 2), ('wp_timeout', 30.0)`; remover os de escape/staging/standoff (já tirados da config na Task 6). Na construção do `DoorCrossConfig`: passar `wp_standoff/wp_retries/wp_timeout`. Em `_CFG_PARAMS`: adicionar os 3 novos, remover os velhos.

- [ ] **Step 4: Build + smoke na bancada**

Run: `cd ros2_packages/robot_nav && python3 -m pytest test/test_door_crossing.py -q`
Expected: PASS.
Run (ambiente ROS): `colcon build --packages-select robot_nav && source install/setup.bash && ros2 run robot_nav door_crossing --ros-args -p wp_standoff:=1.0`
Expected: nó sobe, loga "door_crossing ativo", sem crash. (Ctrl-C.)

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/
git commit -m "feat(door): positioning passivo (nao segura twist_mux) + door_zone/params novos"
```

---

## Self-Review

**Spec coverage:**
- Posicionar via nav2 (W) → Tasks 1,2,7. ✅
- positioning espera/retry/desiste → Task 3. ✅
- rotating alinha longe → Task 4. ✅
- crossing desde W + trava de segurança + release + re-manda G → Task 5. ✅
- Deletar staging/reversing/escape (redução de complexidade) → Task 6. ✅
- Cliente de action + captura G + executar nav → Task 7. ✅
- Não segurar twist_mux em positioning; params novos; door_zone → Task 8. ✅
- Caso de borda "novo goal no meio" → Task 3 (positioning) + nota; durante crossing o `_on_goal_pose` atualiza G, re-mandado ao soltar. ✅
- Fallback A (retry, depois avisa) → Tasks 3 + 8 (door_zone 'failed'). ✅

**Placeholder scan:** sem TBD/TODO; código real em cada step. (Task 6 usa `grep` p/ achar refs antes de remover — é ação concreta, não placeholder.)

**Type consistency:** `Cmd.nav` = tupla `('goto', (x,y,yaw))`/`('cancel',)`/None em todas as tasks; `wp_status` strings consistentes; `pre_door_waypoint` mesma assinatura em 1/2/3/5; helpers `_abort_to_idle`/`_to_idle_success`/`_pose_changed`/`yaw_to_quat` definidos onde citados.

## Validação de campo (após implementar)

Não vai blind pra main (memória do projeto). Após a suite verde: deploy na branch → smoke (nó sobe) → campo com robô ligado (anunciar antes), observar no log: `idle→positioning` manda W, nav2 leva, `positioning→rotating→crossing`, solta re-mandando G. Comparar com o baseline `89d08c7`. Backup do último bom antes de subir pra main.
