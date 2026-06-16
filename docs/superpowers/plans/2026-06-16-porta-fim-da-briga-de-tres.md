# Porta — fim da briga de três (iteração 1) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Acabar com a "briga de três" na aproximação da porta (nav2 ↔ collision ↔ unstuck), fazendo o `door_crossing` ser o dono confiável da região da porta e o `unstuck_supervisor` se calar lá — com uma ré de escape própria pro door_crossing pra ele se reajustar sem depender do unstuck.

**Architecture:** Cinco mudanças pequenas em dois nós Python do `robot_nav`, todas com a lógica de decisão em **funções/máquina-de-estado puras** (testáveis por pytest sem ROS) e a cola de I/O fina no `main()` (`# pragma: no cover`, validada em campo). Reaproveita `rear_min_gap`/`front_min_gap` que já existem no `unstuck_supervisor`.

**Tech Stack:** Python 3, rclpy (ROS 2 Jazzy), numpy, pytest. Nós em `ros2_packages/robot_nav/robot_nav/`, testes em `ros2_packages/robot_nav/test/`.

**Spec:** `docs/superpowers/specs/2026-06-16-porta-fim-da-briga-de-tres-design.md`

**Como rodar os testes** (no PC dev, da árvore fonte — NÃO do `install/`):
```bash
cd ~/Workspace/Controle_robo_web/ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/ -q
```

**⛔ Regra inviolável deste robô:** NUNCA giro em ARCO pra realinhar — só point-turn (giro no mesmo lugar). Nada neste plano introduz arco.

---

## File Structure

- `ros2_packages/robot_nav/robot_nav/door_crossing.py` — **modificar**. Ganha: helper puro `nav_engaging`, helper puro `nearest_door_in_zone`, campos novos em `DoorCrossConfig`, estado `reversing` + método `_maybe_escape` na classe pura `DoorCrossing`, default `rot_speed` 4.0, e a cola no `main()` (computar gaps, publicar `approaching`).
- `ros2_packages/robot_nav/robot_nav/unstuck_supervisor.py` — **modificar**. Ganha helper puro `door_zone_active` e usa ele no `_on_door_zone`.
- `ros2_packages/robot_nav/test/test_door_crossing.py` — **modificar**. Testes novos dos helpers e da ré de escape.
- `ros2_packages/robot_nav/test/test_unstuck_supervisor.py` — **modificar**. Teste do `door_zone_active`.

Sem reflash da MEGA. No robô precisa `colcon build --packages-select robot_nav` + relançar nav2 (feito na Pi pelo usuário, NÃO neste plano).

---

## Task 1: Subir a força do giro no lugar (rot_speed 3.0 → 4.0)

Mudança 5 do spec. Point-turn mais forte (não arco). `rot_speed` é param ROS, tunável ao vivo até 6.0 se patinar.

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (`DoorCrossConfig.rot_speed` e o `declare_parameter` no `main()`)
- Test: `ros2_packages/robot_nav/test/test_door_crossing.py`

- [ ] **Step 1: Escrever o teste que falha**

Adicionar ao fim de `test_door_crossing.py`:

```python
def test_default_rot_speed_is_4():
    # 2026-06-16: 3.0 -> 4.0. Point-turn mais forte pra vencer o atrito do
    # skid-steer parado, sem ser agressivo a ponto de passar do |yaw|<5° (6.0
    # passava). NUNCA arco. Param ROS, sobe pra 6.0 ao vivo se patinar.
    assert DoorCrossConfig().rot_speed == 4.0
```

- [ ] **Step 2: Rodar o teste e confirmar que falha**

Run: `cd ~/Workspace/Controle_robo_web/ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_door_crossing.py::test_default_rot_speed_is_4 -q`
Expected: FAIL (`assert 3.0 == 4.0`).

- [ ] **Step 3: Implementar**

Em `door_crossing.py`, no `DoorCrossConfig`, trocar a linha do `rot_speed`:

```python
    rot_speed: float = 4.0          # rad/s — giro no lugar (point-turn forte; 3->4 em 2026-06-16, sobe a 6.0 ao vivo se patinar; NUNCA arco)
```

E no `main()`, no tuple de defaults do `declare_parameter`, trocar `('rot_speed', 3.0)` por:

```python
                ('rot_speed', 4.0),
```

- [ ] **Step 4: Rodar o teste e confirmar que passa**

Run: `cd ~/Workspace/Controle_robo_web/ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_door_crossing.py::test_default_rot_speed_is_4 -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
cd ~/Workspace/Controle_robo_web
git add ros2_packages/robot_nav/robot_nav/door_crossing.py ros2_packages/robot_nav/test/test_door_crossing.py
git commit -m "feat(door): rot_speed 3.0 -> 4.0 (point-turn mais forte; nunca arco)"
```

---

## Task 2: Afrouxar o gate de armar (nav_engaging)

Mudança 1 do spec. O door_crossing parava de armar quando o nav queria GIRAR pra alinhar (linear≈0). Novo critério: "não está dando ré".

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (novo helper de módulo + uso em `_on_nav`)
- Test: `ros2_packages/robot_nav/test/test_door_crossing.py`

- [ ] **Step 1: Escrever o teste que falha**

Adicionar ao fim de `test_door_crossing.py`:

```python
from robot_nav.door_crossing import nav_engaging


def test_nav_engaging_true_when_rotating_or_forward():
    # girando pra alinhar (linear ~0) ou indo pra frente -> engajado (arma)
    assert nav_engaging(0.0, 0.02) is True
    assert nav_engaging(0.30, 0.02) is True
    # ruído de ré minúsculo dentro da banda ainda conta como engajado
    assert nav_engaging(-0.01, 0.02) is True


def test_nav_engaging_false_only_on_real_reverse():
    # ré sustentada (abaixo de -nav_move_lin) -> NÃO arma
    assert nav_engaging(-0.05, 0.02) is False
```

- [ ] **Step 2: Rodar e confirmar que falha**

Run: `cd ~/Workspace/Controle_robo_web/ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_door_crossing.py -k nav_engaging -q`
Expected: FAIL (`ImportError: cannot import name 'nav_engaging'`).

- [ ] **Step 3: Implementar o helper**

Em `door_crossing.py`, logo após a função `crossing_yaw` (antes da seção `# ---- geometria ...` terminar, perto da linha do `GAP_CORRIDOR_HALF_W`), adicionar:

```python
def nav_engaging(linear_x: float, nav_move_lin: float) -> bool:
    """True se o nav NÃO está dando ré — i.e., avançando OU girando no lugar
    pra alinhar (linear≈0). Antes o gate exigia avançar (linear>thresh) e a
    porta NÃO armava na hora que o robô chegava torto e o RotationShim queria
    girar (linear≈0) -> door_crossing piscava pra idle -> unstuck escapava do
    standdown e sabotava. Como o DWB roda com min_vel_x:0.0 (não dá ré em
    navegação normal), nunca há ré sustentada no ramo do nav, então isto é
    seguro (não reintroduz o 'atravessar de costas')."""
    return linear_x > -nav_move_lin
```

- [ ] **Step 4: Usar no nó**

Em `door_crossing.py`, no método `_on_nav` do `DoorCrossingNode` (dentro do `main()`), trocar:

```python
        def _on_nav(self, msg):
            self._nav_forward = msg.linear.x > self.nav_move_lin
```

por:

```python
        def _on_nav(self, msg):
            # 2026-06-16: "indo pra frente" -> "não está dando ré". Deixa o
            # door_crossing armado quando o nav quer GIRAR pra alinhar (linear≈0).
            self._nav_forward = nav_engaging(msg.linear.x, self.nav_move_lin)
```

- [ ] **Step 5: Rodar e confirmar que passa**

Run: `cd ~/Workspace/Controle_robo_web/ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_door_crossing.py -k nav_engaging -q`
Expected: PASS (2 testes).

- [ ] **Step 6: Commit**

```bash
cd ~/Workspace/Controle_robo_web
git add ros2_packages/robot_nav/robot_nav/door_crossing.py ros2_packages/robot_nav/test/test_door_crossing.py
git commit -m "feat(door): afrouxa o gate de armar (nav_engaging = não-está-dando-ré)"
```

---

## Task 3: Sinalizar `approaching` por proximidade (nearest_door_in_zone)

Mudança 2 do spec. Marca a região da porta de forma contínua (proximidade, ignorando o cone) pro standdown do unstuck. NÃO comanda `door_vel` — só publica `/door_zone`.

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (novo helper de módulo + cola no `_tick`)
- Test: `ros2_packages/robot_nav/test/test_door_crossing.py`

- [ ] **Step 1: Escrever o teste que falha**

Adicionar ao fim de `test_door_crossing.py`:

```python
from robot_nav.door_crossing import nearest_door_in_zone


def test_nearest_door_in_zone_proximity_only():
    doors = [DOOR]                      # centro em (1.5, 2.0)
    # dentro da zona, mas de COSTAS pra porta (cone não importa aqui)
    d = nearest_door_in_zone((1.5, 1.0, -math.pi / 2), doors, zone_radius=1.2)
    assert d is not None and d['id'] == 1
    # fora da zona -> None
    assert nearest_door_in_zone((1.5, -1.0, 0.0), doors, zone_radius=1.2) is None
    # sem pose -> None
    assert nearest_door_in_zone(None, doors, zone_radius=1.2) is None


def test_nearest_door_in_zone_picks_closest():
    doors = [DOOR, {'id': 2, 'a': [1.0, 5.0], 'b': [2.0, 5.0]}]  # centro (1.5,5)
    d = nearest_door_in_zone((1.5, 4.5, 0.0), doors, zone_radius=1.2)
    assert d['id'] == 2
```

- [ ] **Step 2: Rodar e confirmar que falha**

Run: `cd ~/Workspace/Controle_robo_web/ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_door_crossing.py -k nearest_door_in_zone -q`
Expected: FAIL (`ImportError`).

- [ ] **Step 3: Implementar o helper**

Em `door_crossing.py`, logo após a função `nav_engaging` (Task 2), adicionar:

```python
def nearest_door_in_zone(pose, doors, zone_radius: float):
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
```

- [ ] **Step 4: Rodar e confirmar que passa**

Run: `cd ~/Workspace/Controle_robo_web/ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_door_crossing.py -k nearest_door_in_zone -q`
Expected: PASS (2 testes).

- [ ] **Step 5: Cola no nó — publicar `approaching` sem comandar door_vel**

Em `door_crossing.py`, no método `_tick` do `DoorCrossingNode`, trocar o bloco final:

```python
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
```

por:

```python
            prev = self.sup.state
            cmd = self.sup.update(now, pose, self.doors, goal,
                                  self._nav_forward, gap, fresh)
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
```

> A chamada de `update()` segue com 7 args aqui (sem `front_gap`/`rear_gap`) —
> a ré de escape só é ligada na Task 6, depois que a Task 5 dá os parâmetros
> com default a `update()`. Assim o código fica VÁLIDO ao fim desta task.

- [ ] **Step 6: Commit**

```bash
cd ~/Workspace/Controle_robo_web
git add ros2_packages/robot_nav/robot_nav/door_crossing.py ros2_packages/robot_nav/test/test_door_crossing.py
git commit -m "feat(door): sinaliza door_zone='approaching' por proximidade (sem comandar door_vel)"
```

---

## Task 4: Unstuck cala no `approaching` (door_zone_active)

Mudança 3 do spec. O unstuck já fica em standdown com `door_active=True`; agora `approaching` também conta.

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/unstuck_supervisor.py` (novo helper de módulo + uso em `_on_door_zone`)
- Test: `ros2_packages/robot_nav/test/test_unstuck_supervisor.py`

- [ ] **Step 1: Escrever o teste que falha**

Adicionar ao fim de `test_unstuck_supervisor.py`:

```python
from robot_nav.unstuck_supervisor import door_zone_active


def test_door_zone_active_includes_approaching():
    # 2026-06-16: 'approaching' entra no standdown — o unstuck sabotava a
    # APROXIMAÇÃO da porta (ré+giro) antes do door_crossing assumir.
    for st in ('approaching', 'staging', 'rotating', 'crossing'):
        assert door_zone_active(st) is True
    for st in ('idle', '', 'whatever'):
        assert door_zone_active(st) is False
```

- [ ] **Step 2: Rodar e confirmar que falha**

Run: `cd ~/Workspace/Controle_robo_web/ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_unstuck_supervisor.py -k door_zone_active -q`
Expected: FAIL (`ImportError`).

- [ ] **Step 3: Implementar o helper**

Em `unstuck_supervisor.py`, na seção de lógica pura (logo após `freer_side`, antes de `@dataclass class UnstuckConfig`), adicionar:

```python
def door_zone_active(state: str) -> bool:
    """True se o door_crossing está CONDUZINDO (staging/rotating/crossing) OU
    apenas SE APROXIMANDO ('approaching') de uma porta marcada -> unstuck em
    standdown. 'approaching' incluído em 2026-06-16: sem ele, o unstuck (prio
    30, ré+giro 15°) sabotava a aproximação antes do door_crossing assumir, e o
    robô brigava com a porta por minutos."""
    return state in ('approaching', 'staging', 'rotating', 'crossing')
```

- [ ] **Step 4: Usar no nó**

Em `unstuck_supervisor.py`, no método `_on_door_zone` (dentro do `main()`), trocar:

```python
        def _on_door_zone(self, msg):
            try:
                st = json.loads(msg.data).get("state", "idle")
            except (ValueError, AttributeError):
                st = "idle"
            self._door_active = st in ("staging", "rotating", "crossing")
```

por:

```python
        def _on_door_zone(self, msg):
            try:
                st = json.loads(msg.data).get("state", "idle")
            except (ValueError, AttributeError):
                st = "idle"
            self._door_active = door_zone_active(st)
```

- [ ] **Step 5: Rodar e confirmar que passa**

Run: `cd ~/Workspace/Controle_robo_web/ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_unstuck_supervisor.py -k door_zone_active -q`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
cd ~/Workspace/Controle_robo_web
git add ros2_packages/robot_nav/robot_nav/unstuck_supervisor.py ros2_packages/robot_nav/test/test_unstuck_supervisor.py
git commit -m "feat(unstuck): standdown também em door_zone='approaching'"
```

---

## Task 5: Ré de ESCAPE no door_crossing (lógica pura)

Mudança 4 do spec — a maior. Estado `reversing` + método `_maybe_escape`. Disparada por obstáculo perto na frente (anti-stall/anti-BMS) OU sub-timeout de alinhamento; recua reto (NUNCA arco), gated pelo vão traseiro, limitada a `escape_max_count` -> senão aborta (handoff pro unstuck).

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (`DoorCrossConfig`, `DoorCrossing.__init__`, `DoorCrossing.update`, novo `_maybe_escape`)
- Test: `ros2_packages/robot_nav/test/test_door_crossing.py`

- [ ] **Step 1: Escrever os testes que falham**

Adicionar ao fim de `test_door_crossing.py`:

```python
# ---- ré de escape (2026-06-16) -----------------------------------------------

ECFG = DoorCrossConfig(zone_radius=1.2, stage_dist=0.6, align_timeout=15.0,
                       total_timeout=40.0)
P_STAGE = (1.5, 1.0, math.pi / 2)   # na zona, encarando a porta (centro 1.5,2.0)


def estep(dc, t, pose, front_gap=math.inf, rear_gap=math.inf,
          goal=True, nav=True, gap=math.inf, fresh=True):
    return dc.update(t, pose, [DOOR], goal, nav, gap, fresh, front_gap, rear_gap)


def test_escape_reverse_on_front_block():
    dc = DoorCrossing(ECFG)
    assert estep(dc, 0.0, P_STAGE).state == 'staging'         # arma
    c = estep(dc, 0.1, P_STAGE, front_gap=0.10)               # parede perto -> ré
    assert c.state == 'reversing'
    assert c.vx < 0.0 and c.wz == pytest.approx(0.0)          # ré RETA, nunca arco


def test_escape_reverse_on_substuck_timeout():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)                                   # arma (align_t0=0)
    c = estep(dc, ECFG.escape_substuck_time + 0.1, P_STAGE)   # não progrediu -> ré
    assert c.state == 'reversing'


def test_escape_aborts_when_rear_blocked():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    # parede na frente E sem vão atrás -> não força, larga pro nav2/unstuck
    c = estep(dc, 0.1, P_STAGE, front_gap=0.10, rear_gap=0.05)
    assert c.state == 'idle'


def test_escape_target_capped_by_rear_gap():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    estep(dc, 0.1, P_STAGE, front_gap=0.10, rear_gap=0.25)
    # alvo = min(escape_reverse_dist, rear_gap - escape_rear_margin) = min(0.30,0.15)
    assert dc._esc_target == pytest.approx(0.15)


def test_reverse_returns_to_staging_after_distance():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    estep(dc, 0.1, P_STAGE, front_gap=0.10)                   # -> reversing (alvo 0.30)
    # recuou 0.4 m (afastou da porta, y caiu) -> volta pro staging
    c = estep(dc, 0.5, (1.5, 0.6, math.pi / 2))
    assert c.state == 'staging'


def test_reverse_aborts_to_staging_if_rear_closes():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    estep(dc, 0.1, P_STAGE, front_gap=0.10)                   # -> reversing
    # algo entrou atrás no meio da ré -> para e volta pro staging
    c = estep(dc, 0.2, (1.5, 0.95, math.pi / 2), rear_gap=0.05)
    assert c.state == 'staging'
    assert c.vx == pytest.approx(0.0)


def test_escape_max_count_then_abort():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    t = 0.1
    for _ in range(ECFG.escape_max_count):
        assert estep(dc, t, P_STAGE, front_gap=0.10).state == 'reversing'
        # completa a ré (recua bastante) -> staging
        assert estep(dc, t + 0.05, (1.5, 0.5, math.pi / 2)).state == 'staging'
        t += 0.2
    # estourou o nº de escapes -> próximo bloqueio aborta (larga pro unstuck)
    assert estep(dc, t, P_STAGE, front_gap=0.10).state == 'idle'
```

- [ ] **Step 2: Rodar e confirmar que falha**

Run: `cd ~/Workspace/Controle_robo_web/ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_door_crossing.py -k "escape or reverse" -q`
Expected: FAIL (`AttributeError`/`TypeError`: `update()` não aceita 2 args extras, `_esc_target`/`escape_*` não existem).

- [ ] **Step 3: Campos novos no `DoorCrossConfig`**

Em `door_crossing.py`, no `DoorCrossConfig`, adicionar ao fim (depois de `retrigger_cooldown`):

```python
    # Ré de ESCAPE (2026-06-16): sem a ré do unstuck (calado na região da
    # porta), o door_crossing precisa se reajustar sozinho — senão fica
    # morto-preso de nariz na parede (e stalla o motor -> desarma o BMS, já que
    # door_vel fura o collision). Ré RETA (NUNCA arco), gated pelo vão traseiro.
    escape_front_gap: float = 0.20      # m — obstáculo a menos disso à frente -> ré (anti-stall)
    escape_substuck_time: float = 5.0   # s — alinhando sem chegar ao crossing -> ré
    escape_reverse_dist: float = 0.30   # m — quanto recua por escape (teto)
    escape_reverse_speed: float = 0.12  # m/s — ré mansa
    escape_max_count: int = 3           # nº de escapes por travessia antes de abortar
    escape_rear_margin: float = 0.10    # m — nunca chega a menos disso do obstáculo atrás
    escape_rear_min: float = 0.10       # m — vão traseiro útil mínimo pra valer a ré
```

- [ ] **Step 4: Campos novos no `DoorCrossing.__init__`**

Em `door_crossing.py`, no `DoorCrossing.__init__`, adicionar depois de `self._cooldown_until = 0.0`:

```python
        self._escape_count = 0          # rés de escape NESTA travessia
        self._align_t0 = 0.0            # início do "tentando alinhar" (sub-timeout)
        self._esc_start = (0.0, 0.0)    # pose (x,y) no começo da ré atual
        self._esc_target = 0.0          # quanto recuar nesta ré
```

- [ ] **Step 5: Adicionar o método `_maybe_escape`**

Em `door_crossing.py`, dentro da classe `DoorCrossing`, logo após o método `_abort`, adicionar:

```python
    def _maybe_escape(self, now, pos, front_gap, rear_gap):
        """Decide se entra na ré de escape (ou aborta). Retorna um Cmd se a ré
        toma conta agora, ou None pra seguir o staging/rotating normal.

        Dispara quando: obstáculo perto na FRENTE (anti-stall/anti-BMS) OU não
        alinhou dentro de escape_substuck_time. Recua RETO (nunca arco), no
        máximo (rear_gap - escape_rear_margin), limitado a escape_reverse_dist.
        Sem vão atrás útil, ou estourado o escape_max_count -> ABORTA (larga pro
        nav2/unstuck como último recurso)."""
        cfg = self.cfg
        front_block = front_gap < cfg.escape_front_gap
        need = front_block or (now - self._align_t0 > cfg.escape_substuck_time)
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
```

- [ ] **Step 6: Trocar a assinatura do `update` e setar os timers no arm**

Em `door_crossing.py`, trocar a assinatura do `update`:

```python
    def update(self, now, pose, doors, goal_active, nav_forward, gap,
               scan_fresh) -> Cmd:
```

por:

```python
    def update(self, now, pose, doors, goal_active, nav_forward, gap,
               scan_fresh, front_gap=math.inf, rear_gap=math.inf) -> Cmd:
```

E no bloco de arm (dentro de `if self.state == 'idle':`), depois de `self._stable = 0`, adicionar:

```python
            self._escape_count = 0
            self._align_t0 = now
```

(de modo que o trecho fique:)

```python
            self.door, self.geom = door, geom
            self.state = 'staging'
            self.t_start = now
            self._stable = 0
            self._escape_count = 0
            self._align_t0 = now
            # cai no fluxo de staging já neste tick
```

- [ ] **Step 7: Chamar `_maybe_escape` no topo de `staging` e `rotating`**

Em `door_crossing.py`, no `update`, no início do bloco `if self.state == 'staging':` (antes de calcular `tgx`), adicionar:

```python
        if self.state == 'staging':
            esc = self._maybe_escape(now, (x, y), front_gap, rear_gap)
            if esc is not None:
                return esc
            # alvo: ponto no eixo, stage_dist antes do centro
            tgx = g.cx - g.nx * self.side * cfg.stage_dist
```

E no início do bloco `if self.state == 'rotating':` (antes de `aligned = ...`), adicionar:

```python
        if self.state == 'rotating':
            esc = self._maybe_escape(now, (x, y), front_gap, rear_gap)
            if esc is not None:
                return esc
            aligned = abs(yaw_err) <= cfg.align_yaw and abs(d) <= cfg.align_lat
```

- [ ] **Step 8: Adicionar o estado `reversing`**

Em `door_crossing.py`, no `update`, logo ANTES do bloco `if self.state == 'crossing':`, adicionar:

```python
        if self.state == 'reversing':
            if rear_gap <= cfg.escape_rear_margin:
                # algo entrou atrás no meio da ré -> para e re-tenta o staging
                self.state = 'staging'
                self._align_t0 = now
                return Cmd('staging', 0.0, 0.0, self.door['id'])
            travelled = math.hypot(x - self._esc_start[0], y - self._esc_start[1])
            if travelled >= self._esc_target:
                # recuou o suficiente -> re-tenta o alinhamento de um ponto melhor
                self.state = 'staging'
                self._align_t0 = now
                return Cmd('staging', 0.0, 0.0, self.door['id'])
            return Cmd('reversing', -cfg.escape_reverse_speed, 0.0,
                       self.door['id'])
```

- [ ] **Step 9: Rodar os testes da ré de escape e confirmar que passam**

Run: `cd ~/Workspace/Controle_robo_web/ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_door_crossing.py -k "escape or reverse" -q`
Expected: PASS (7 testes).

- [ ] **Step 10: Rodar a suíte INTEIRA do door_crossing (sem regressão)**

Run: `cd ~/Workspace/Controle_robo_web/ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/test_door_crossing.py -q`
Expected: PASS (todos — os testes antigos chamam `update` sem `front_gap`/`rear_gap`, que caem no default `inf` e não disparam escape; o `test_align_timeout_*` pula o intervalo do sub-timeout num único tick e bate primeiro no guard do `align_timeout`).

- [ ] **Step 11: Commit**

```bash
cd ~/Workspace/Controle_robo_web
git add ros2_packages/robot_nav/robot_nav/door_crossing.py ros2_packages/robot_nav/test/test_door_crossing.py
git commit -m "feat(door): ré de ESCAPE (estado reversing, gated por rear_gap, anti-stall por front_gap; nunca arco)"
```

---

## Task 6: Cola de I/O — alimentar front_gap/rear_gap no nó

Mudança 4 (parte de I/O). O `_tick` mede o vão dianteiro e traseiro reais do /scan (reusando `front_min_gap`/`rear_min_gap` do unstuck) e passa pro `update`. Glue (`# pragma: no cover`) — sem teste unitário; validado em campo.

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/door_crossing.py` (`main()`: imports, params de geometria, `_tick`)

- [ ] **Step 1: Importar as funções de vão e numpy no `main()`**

Em `door_crossing.py`, no `main()`, no bloco de imports locais (perto de `from .utils import quat_to_yaw, spin_node`), adicionar:

```python
    from .unstuck_supervisor import front_min_gap, rear_min_gap
```

(`numpy` já está importado no topo do módulo como `np`; `math` também.)

- [ ] **Step 2: Declarar os params de geometria da ré**

Em `door_crossing.py`, no `DoorCrossingNode.__init__`, no tuple do `for name, default in (...)`, adicionar (junto dos outros, ex. após `('nav_move_lin', 0.02),`):

```python
                ('rear_tail_x', -0.25), ('rear_half_width', 0.30),
                ('front_head_x', 0.25),
```

E depois do bloco que lê os params (após `self.nav_move_lin = g['nav_move_lin']`), guardar:

```python
            self.rear_tail_x = g['rear_tail_x']
            self.rear_half_width = g['rear_half_width']
            self.front_head_x = g['front_head_x']
```

- [ ] **Step 3: Computar front_gap/rear_gap no `_tick`**

Em `door_crossing.py`, no `_tick`, logo depois do bloco que computa `gap` (o `gap_ahead` do crossing) e ANTES de `prev = self.sup.state`, inserir:

```python
            front_gap = math.inf
            rear_gap = math.inf
            if fresh and self._scan is not None:
                ranges, amin, ainc = self._scan
                arr = np.asarray(ranges, dtype=np.float64)
                # LiDAR no centro (lidar_x=0); vão medido do para-choque. Sem
                # descontar batente de propósito (anti-stall: contato com a
                # parede/batente conta), diferente do gap_ahead do crossing.
                front_gap = front_min_gap(arr, amin, ainc, 0.0,
                                          self.front_head_x, self.rear_half_width)
                rear_gap = rear_min_gap(arr, amin, ainc, 0.0,
                                        self.rear_tail_x, self.rear_half_width)
```

- [ ] **Step 4: Passar os gaps pro `update` (liga a ré de escape)**

Em `door_crossing.py`, no `_tick`, trocar a chamada de `update` (posta na Task 3):

```python
            cmd = self.sup.update(now, pose, self.doors, goal,
                                  self._nav_forward, gap, fresh)
```

por:

```python
            cmd = self.sup.update(now, pose, self.doors, goal,
                                  self._nav_forward, gap, fresh,
                                  front_gap, rear_gap)
```

- [ ] **Step 5: Sanidade de import (a árvore carrega sem ROS)**

Run: `cd ~/Workspace/Controle_robo_web/ros2_packages/robot_nav && PYTHONPATH=. python3 -c "import robot_nav.door_crossing as d; print(d.nav_engaging(0.0,0.02), d.nearest_door_in_zone((1.5,1.0,0.0),[{'id':1,'a':[1,2],'b':[2,2]}],1.2)['id'])"`
Expected: imprime `True 1` sem erro de import (confirma que o `from .unstuck_supervisor import ...` no `main()` não quebra o carregamento do módulo, e os helpers puros funcionam).

- [ ] **Step 6: Rodar a suíte INTEIRA do robot_nav (sem regressão)**

Run: `cd ~/Workspace/Controle_robo_web/ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/ -q`
Expected: PASS em tudo (door_crossing, unstuck_supervisor, scan_sanitizer, fused_odom, cone_pose_fix).

- [ ] **Step 7: Commit**

```bash
cd ~/Workspace/Controle_robo_web
git add ros2_packages/robot_nav/robot_nav/door_crossing.py
git commit -m "feat(door): mede front_gap/rear_gap do /scan e alimenta a ré de escape (reusa unstuck)"
```

---

## Task 7: Verificação final + nota de deploy

**Files:** nenhum (verificação).

- [ ] **Step 1: Suíte inteira verde**

Run: `cd ~/Workspace/Controle_robo_web/ros2_packages/robot_nav && PYTHONPATH=. python3 -m pytest test/ -q`
Expected: todos PASS, zero falhas.

- [ ] **Step 2: Conferir o git log das 6 mudanças**

Run: `cd ~/Workspace/Controle_robo_web && git log --oneline -8`
Expected: ver os commits do plano (rot_speed, nav_engaging, approaching, unstuck standdown, ré de escape, gaps no nó) + os 4 commits de spec/docs anteriores.

- [ ] **Step 3: Nota de deploy (NÃO executar — é o usuário que sobe a stack)**

O deploy na Pi é manual (memória do projeto: o usuário sobe a stack sozinho). Quando ele pedir:
```bash
# na Pi (ssh robo@robo-desktop.local):
cd ~/workspace/Controle_robo_web && git fetch && git reset --hard origin/main \
  && colcon build --packages-select robot_nav
# depois relançar a nav2
```
Validação de campo (robô LIGADO, anunciar antes) está no spec, seção "Validação":
o robô deve atravessar SEM o unstuck dar ré+giro 15°; a ré que aparecer é a
curta de escape do door_crossing; nenhum desarme de BMS; tempo cai de ~5 min
pra dezenas de segundos. Watch nos logs: `unstuck:` sem `reversing/spinning`
durante `door_zone ∈ {approaching,staging,rotating,crossing}`; `door_crossing:`
sem voltar repetidamente pra `idle`.

---

## Notas de fechamento

- **Sem reflash da MEGA.** Tudo é Python no `robot_nav`.
- **Iteração 2 (futura, NÃO neste plano):** malha fechada no yaw da IMU pro
  `rotating` (gira até o yaw MEDIDO chegar no alvo, igual ao spin do
  `unstuck_supervisor`) — precisão fina sem oscilar. ⛔ Sempre point-turn, NUNCA
  arco.
- **Rollback:** cada mudança é um commit isolado; `git revert <sha>` desfaz
  qualquer uma sem tocar nas outras. `rot_speed` e os `escape_*` são params ROS
  (tunáveis ao vivo sem rebuild).
