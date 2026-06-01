# Correção de pose por cone-âncora no trekking — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Corrigir a POSE (x/y) do `pose_estimator` de forma persistente quando um cone-âncora gravado é confirmado (gates conservadores que nunca teleportam o robô), expor na UI o que ele usa de referência (read-only), e permitir corrigir/limpar o cone de um waypoint na gravação.

**Architecture:** Toda a lógica de decisão (delta, gate de magnitude, confirmação temporal, unicidade, recálculo de bearing) vive num módulo puro sem ROS (`cone_pose_fix.py`), testável com pytest. O `trekking_runner` publica `/trekking/pose_fix` (`Vector3Stamped`, `odom`) só quando um cone é confirmado e único, expõe `anchor`/`anchor_status`/`anchor_clutter`/`anchor_confirm` no `/trekking/state` (JSON existente), e aceita o comando `set_cone` para re-vincular/limpar o cone de um waypoint. O `pose_estimator` assina `pose_fix` e aplica sob `_lock` com ganho parcial + rejeição por magnitude. A correção é **aditiva** — snap-to-cone do alvo inalterado; sem cone confirmado, comportamento idêntico ao de hoje.

**Tech Stack:** Python 3, ROS 2 (rclpy), `geometry_msgs/Vector3Stamped`, pytest; Flask + socket.io + canvas 2D (UI web).

**Spec de origem:** `docs/superpowers/specs/2026-06-01-correcao-pose-cone-trekking-design.md`

---

## File Structure

- **Create:** `ros2_packages/robot_nav/robot_nav/cone_pose_fix.py` — puro (sem ROS): `cone_fix_delta`, `apply_pose_fix`, `cone_bearing`, `ConeFixConfirmer`.
- **Create:** `ros2_packages/robot_nav/test/test_cone_pose_fix.py` — unit tests (cobre §6 do spec).
- **Modify:** `ros2_packages/robot_nav/robot_nav/pose_estimator.py` — assina `/trekking/pose_fix`; 2 params.
- **Modify:** `ros2_packages/robot_nav/robot_nav/trekking_runner.py` — publica `pose_fix` + telemetria de âncora + comando `set_cone`; 4 params.
- **Modify:** `ros2_packages/robot_nav/launch/trekking.launch.py` — arg `enable_cone_pose_fix`.
- **Modify:** `controle_web/app.py` — whitelist `set_cone` + kwargs.
- **Modify:** `controle_web/templates/index.html` — botão "limpar cone".
- **Modify:** `controle_web/static/js/trekking.js` — render âncora/clutter/status + clique-pra-corrigir.

`controle_web/trekking_service.py` **não muda** (passthrough genérico).

---

## Task 1: Módulo puro `cone_pose_fix`

**Files:**
- Create: `ros2_packages/robot_nav/robot_nav/cone_pose_fix.py`
- Test: `ros2_packages/robot_nav/test/test_cone_pose_fix.py`

- [ ] **Step 1: Escrever os testes que falham**

Create `ros2_packages/robot_nav/test/test_cone_pose_fix.py`:

```python
import math

import pytest

from robot_nav.cone_pose_fix import (
    ConeFixConfirmer,
    apply_pose_fix,
    cone_bearing,
    cone_fix_delta,
)


def test_cone_fix_delta():
    dx, dy = cone_fix_delta((2.0, 3.0), (1.7, 3.4))
    assert dx == pytest.approx(0.3)
    assert dy == pytest.approx(-0.4)


def test_apply_pose_fix_accepts_small():
    nx, ny, ok = apply_pose_fix(10.0, 5.0, 0.4, -0.2, gain=0.5, max_mag=0.6)
    assert ok is True
    assert nx == pytest.approx(10.2)
    assert ny == pytest.approx(4.9)


def test_apply_pose_fix_rejects_large():
    nx, ny, ok = apply_pose_fix(10.0, 5.0, 0.7, 0.0, gain=0.5, max_mag=0.6)
    assert ok is False
    assert (nx, ny) == (10.0, 5.0)


def test_cone_bearing_relative_to_recorded_yaw():
    # robô em (0,0) olhando +x (yaw=0); cone em (1,1) → bearing +45°
    assert cone_bearing(0.0, 0.0, 0.0, 1.0, 1.0) == pytest.approx(math.pi / 4)
    # mesmo cone, robô girado +90° (olhando +y) → cone agora a -45° relativo
    assert cone_bearing(0.0, 0.0, math.pi / 2, 1.0, 1.0) == pytest.approx(-math.pi / 4)


def test_confirmer_stable_sequence_confirms():
    c = ConeFixConfirmer(confirm_frames=4, stable_eps=0.10)
    pos = (1.0, 2.0)
    results = [c.update(pos, n_candidates=1) for _ in range(4)]
    assert results == [False, False, False, True]


def test_confirmer_moving_never_confirms():
    c = ConeFixConfirmer(confirm_frames=4, stable_eps=0.10)
    confirmed = False
    for i in range(10):
        confirmed = confirmed or c.update((1.0 + 0.2 * i, 2.0), n_candidates=1)
    assert confirmed is False


def test_confirmer_ambiguous_skips():
    c = ConeFixConfirmer(confirm_frames=2, stable_eps=0.10)
    pos = (1.0, 2.0)
    assert c.update(pos, n_candidates=2) is False
    assert c.update(pos, n_candidates=2) is False
    assert c.update(pos, n_candidates=2) is False


def test_confirmer_no_match_resets():
    c = ConeFixConfirmer(confirm_frames=2, stable_eps=0.10)
    pos = (1.0, 2.0)
    assert c.update(pos, n_candidates=1) is False
    assert c.update(None, n_candidates=1) is False
    assert c.update(pos, n_candidates=1) is False


def test_confirmer_count_exposes_progress():
    c = ConeFixConfirmer(confirm_frames=4, stable_eps=0.10)
    assert c.count == 0
    c.update((1.0, 2.0), n_candidates=1)
    assert c.count == 1
    c.update((1.0, 2.0), n_candidates=1)
    assert c.count == 2
```

- [ ] **Step 2: Rodar os testes pra confirmar que falham**

Run: `cd ros2_packages/robot_nav && python -m pytest test/test_cone_pose_fix.py -v`
Expected: FAIL com `ModuleNotFoundError: No module named 'robot_nav.cone_pose_fix'`.
(Fallback da raiz: `PYTHONPATH=ros2_packages/robot_nav python -m pytest ros2_packages/robot_nav/test/test_cone_pose_fix.py -v`.)

- [ ] **Step 3: Implementar o módulo puro**

Create `ros2_packages/robot_nav/robot_nav/cone_pose_fix.py`:

```python
#!/usr/bin/env python3
"""
Regras puras (sem ROS) da correção de pose por cone-âncora no trekking.

Isolar isto do nó ROS permite testar com pytest direto e mantém o
trekking_runner/pose_estimator enxutos. Design:
docs/superpowers/specs/2026-06-01-correcao-pose-cone-trekking-design.md.
"""
import math


def cone_fix_delta(recorded, observed):
    """Deriva medida = cone gravado - cone observado, ambos (x, y) em odom.

    Numa sessão de trekking o frame odom é o mesmo da gravação ao percurso;
    a diferença entre onde o cone foi gravado e onde é visto agora é
    exatamente o quanto a pose derivou.
    """
    return (recorded[0] - observed[0], recorded[1] - observed[1])


def apply_pose_fix(x, y, dx, dy, gain, max_mag):
    """Aplica o delta à pose com ganho parcial; rejeita teleportes.

    Se |(dx, dy)| > max_mag a associação é suspeita (cone errado) e nada muda.
    Retorna (novo_x, novo_y, aceito: bool).
    """
    if math.hypot(dx, dy) > max_mag:
        return x, y, False
    return x + gain * dx, y + gain * dy, True


def cone_bearing(wp_x, wp_y, wp_yaw, cone_x, cone_y):
    """Bearing do cone relativo à pose GRAVADA do waypoint (rad, wrap ±π).

    Reproduz o que _save_point grava, pra que o gate angular do PLAY continue
    coerente após uma troca de cone via set_cone.
    """
    b = math.atan2(cone_y - wp_y, cone_x - wp_x) - wp_yaw
    return math.atan2(math.sin(b), math.cos(b))  # wrap_pi


class ConeFixConfirmer:
    """Gate temporal + unicidade antes de corrigir a pose.

    `update` retorna True na PRIMEIRA chamada em que o mesmo candidato
    (posição estável dentro de `stable_eps`) e ÚNICO (n_candidates <= 1) se
    manteve por `confirm_frames` chamadas seguidas. Cone parado confirma;
    objeto se movendo reseta; ambiguidade (n>1) nunca confirma. O chamador
    deve parar de chamar após o primeiro True e chamar reset() ao trocar de
    waypoint. `count` expõe o progresso pra telemetria.
    """

    def __init__(self, confirm_frames, stable_eps):
        self.confirm_frames = int(confirm_frames)
        self.stable_eps = float(stable_eps)
        self._pos = None
        self._count = 0

    @property
    def count(self):
        return self._count

    def reset(self):
        self._pos = None
        self._count = 0

    def update(self, match_pos, n_candidates):
        if match_pos is None or n_candidates > 1:
            self.reset()
            return False
        if (
            self._pos is not None
            and math.hypot(match_pos[0] - self._pos[0],
                           match_pos[1] - self._pos[1]) < self.stable_eps
        ):
            self._count += 1
        else:
            self._count = 1
        self._pos = match_pos
        return self._count >= self.confirm_frames
```

- [ ] **Step 4: Rodar os testes pra confirmar que passam**

Run: `cd ros2_packages/robot_nav && python -m pytest test/test_cone_pose_fix.py -v`
Expected: PASS (9 passed).

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/cone_pose_fix.py ros2_packages/robot_nav/test/test_cone_pose_fix.py
git commit -m "trekking: regras puras da correcao de pose por cone-ancora (+ testes)"
```

---

## Task 2: `pose_estimator` assina e aplica `/trekking/pose_fix`

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/pose_estimator.py`

Nota: `Vector3Stamped` (linha 32), `math` (linha 28) e o `_lock` que protege `self.x/y` (linha 111) já existem.

- [ ] **Step 1: Importar o helper puro**

Modify, após a linha 39:

```python
from .utils import quat_to_yaw as _quat_to_yaw  # noqa: F401
from .cone_pose_fix import apply_pose_fix
```

- [ ] **Step 2: Declarar e ler os 2 params**

Modify, após a linha 80 (`self.declare_parameter('slip_threshold', 0.15)`):

```python
        # --- Detecção de slip ---
        self.declare_parameter('slip_threshold', 0.15)  # m/s

        # --- Correção de pose por cone-âncora (trekking_runner publica pose_fix) ---
        self.declare_parameter('pose_fix_gain', 0.5)   # fração do delta aplicada
        self.declare_parameter('pose_fix_max', 0.6)    # m — acima disso, rejeita
```

E após a linha 105 (`self.slip_threshold = float(...)`):

```python
        self.slip_threshold = float(self.get_parameter('slip_threshold').value)
        self.pose_fix_gain  = float(self.get_parameter('pose_fix_gain').value)
        self.pose_fix_max   = float(self.get_parameter('pose_fix_max').value)
```

- [ ] **Step 3: Assinar o tópico**

Modify, após a linha 145 (subscription do `optical_flow`):

```python
        self.create_subscription(Vector3Stamped, 'optical_flow', self._on_flow, 20)
        self.create_subscription(Vector3Stamped, 'trekking/pose_fix', self._on_pose_fix, 10)
```

- [ ] **Step 4: Implementar o callback**

Modify, adicionar o método logo após `_on_flow` (após a linha 209, antes de `_set_wheel`):

```python
    def _on_pose_fix(self, msg: Vector3Stamped):
        # Empurra x/y pela deriva medida no cone-âncora. Rejeita teleportes
        # (associação suspeita) e aplica suave. Yaw nunca é tocado (só IMU).
        dx = float(msg.vector.x)
        dy = float(msg.vector.y)
        with self._lock:
            nx, ny, ok = apply_pose_fix(
                self.x, self.y, dx, dy, self.pose_fix_gain, self.pose_fix_max,
            )
            if ok:
                self.x = nx
                self.y = ny
        if ok:
            self.get_logger().info(
                f'pose_fix aplicado: Δ=({dx:+.2f}, {dy:+.2f}) m '
                f'(ganho {self.pose_fix_gain:.2f})'
            )
        else:
            self.get_logger().warn(
                f'pose_fix REJEITADO: |Δ|={math.hypot(dx, dy):.2f} m '
                f'> {self.pose_fix_max:.2f} m — associação de cone suspeita'
            )
```

- [ ] **Step 5: Smoke test de sintaxe**

Run: `cd ros2_packages/robot_nav && python -c "import ast; ast.parse(open('robot_nav/pose_estimator.py').read()); print('ok')"`
Expected: `ok`

- [ ] **Step 6: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/pose_estimator.py
git commit -m "pose_estimator: assina /trekking/pose_fix e aplica a x/y com ganho+gate"
```

---

## Task 3: `trekking_runner` confirma o cone-âncora, publica `pose_fix` e expõe a telemetria

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/trekking_runner.py`

- [ ] **Step 1: Importar `Vector3Stamped` e os helpers puros**

Modify a linha 40:

```python
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist, Vector3Stamped
```

E após a linha 50:

```python
from .utils import quat_to_yaw as _quat_to_yaw, wrap_pi as _wrap_pi
from .cone_pose_fix import ConeFixConfirmer, cone_fix_delta
```

- [ ] **Step 2: Declarar e ler os 4 params**

Modify, após a linha 82 (`cone_bearing_tol_deg`):

```python
        self.declare_parameter('cone_bearing_tol_deg', 60.0) # ° — janela angular relativa

        # --- Correção persistente de pose por cone-âncora (aditiva ao snap) ---
        self.declare_parameter('enable_cone_pose_fix', True)
        self.declare_parameter('cone_confirm_frames', 4)     # ciclos estáveis p/ confirmar
        self.declare_parameter('cone_stable_eps', 0.10)      # m — "mesma posição" entre ciclos
        self.declare_parameter('cone_unique_radius', 0.50)   # m — se >1 candidato aqui → ambíguo
```

E após a linha 105 (`self.ctrl_dt = ...`):

```python
        self.ctrl_dt = 1.0 / float(self.get_parameter('control_hz').value)
        self.enable_cone_pose_fix = bool(self.get_parameter('enable_cone_pose_fix').value)
        self.cone_confirm_frames  = int(self.get_parameter('cone_confirm_frames').value)
        self.cone_stable_eps      = float(self.get_parameter('cone_stable_eps').value)
        self.cone_unique_radius   = float(self.get_parameter('cone_unique_radius').value)
```

- [ ] **Step 3: Estado do confirmador + telemetria de âncora**

Modify, logo após a linha 138 (`self.locked_cone = None ...`):

```python
        self.locked_cone = None    # (x, y) — cone "trancado" pra esse waypoint, ou None
        # Correção de pose: confirmador + trava 1x-por-cone + telemetria read-only.
        self._confirmer = ConeFixConfirmer(self.cone_confirm_frames, self.cone_stable_eps)
        self._cone_fix_done = False
        self._anchor = None            # (x,y) detecção usada como referência, ou None
        self._anchor_status = 'idle'   # idle | confirming | ambiguous | fixed
        self._anchor_clutter = []      # [(x,y), ...] candidatos descartados perto do esperado
        self._anchor_confirm = 0       # progresso do confirmador
```

- [ ] **Step 4: Criar o publisher**

Modify, após a linha 156 (`pub_target`):

```python
        self.pub_target = self.create_publisher(PoseStamped, 'trekking/target', 10)
        self.pub_pose_fix = self.create_publisher(Vector3Stamped, 'trekking/pose_fix', 10)
```

- [ ] **Step 5: Helper de candidatos + publicação do fix com telemetria**

Modify, adicionar dois métodos logo após `_find_matching_cone` (após a linha 478, antes de `_on_arrival`):

```python
    def _candidates(self, wp: dict, cones):
        # Detecções dentro do raio de unicidade ao redor da posição esperada do
        # cone gravado. Usa max(unique, match) p/ NUNCA ficar menor que a região
        # de onde o match sai (senão a trava de unicidade teria uma brecha).
        r = max(self.cone_unique_radius, self.r_match)
        out = []
        for cx, cy, _w in cones:
            if math.hypot(cx - wp['cone_x'], cy - wp['cone_y']) <= r:
                out.append((cx, cy))
        return out

    def _maybe_publish_pose_fix(self, wp: dict, x, y, yaw, cones):
        # Confirmação ANTES de corrigir a pose — independente do snap do alvo.
        match = self._find_matching_cone(wp, x, y, yaw, cones)
        cands = self._candidates(wp, cones)
        n_cand = len(cands)
        confirmed = self._confirmer.update(match, n_cand)
        # telemetria do que ele está usando de referência (read-only p/ UI)
        if match is None:
            self._anchor = None
            self._anchor_status = 'idle'
            self._anchor_clutter = []
        else:
            self._anchor = match
            self._anchor_status = 'ambiguous' if n_cand > 1 else 'confirming'
            self._anchor_clutter = [c for c in cands if c != match]
        self._anchor_confirm = self._confirmer.count
        if not confirmed:
            return
        # Confirmado e único: delta = cone_gravado - cone_observado.
        dx, dy = cone_fix_delta((wp['cone_x'], wp['cone_y']), match)
        v = Vector3Stamped()
        v.header.stamp = self.get_clock().now().to_msg()
        v.header.frame_id = 'odom'
        v.vector.x = float(dx)
        v.vector.y = float(dy)
        self.pub_pose_fix.publish(v)
        self._cone_fix_done = True   # só uma vez por cone travado
        self._anchor_status = 'fixed'
        self.last_msg = f'pose_fix wp{self.current_idx}: Δ=({dx:+.2f}, {dy:+.2f})'
```

- [ ] **Step 6: Fiar no `_control_tick` (após o bloco de snap, antes do cálculo de `dx`)**

Modify `_control_tick`. O bloco de snap termina na linha 398 (`target_y = self.locked_cone[1] + oy`). Inserir logo depois, antes de `dx = target_x - x`:

```python
            if self.locked_cone is not None:
                # alvo corrigido: cone_observado + offset gravado
                ox = wp['x'] - wp['cone_x']
                oy = wp['y'] - wp['cone_y']
                target_x = self.locked_cone[0] + ox
                target_y = self.locked_cone[1] + oy

        # 1b) Correção PERSISTENTE de pose por cone-âncora (aditiva: não mexe no
        # alvo acima). Gates conservadores no _confirmer; na dúvida não corrige.
        if self.enable_cone_pose_fix and wp['has_cone'] and not self._cone_fix_done:
            self._maybe_publish_pose_fix(wp, x, y, yaw, cones)

        dx = target_x - x
        dy = target_y - y
        dist = math.hypot(dx, dy)
```

- [ ] **Step 7: Resetar a trava + telemetria ao trocar de waypoint**

Modify o bloco de chegada do `_control_tick` (linhas 412-418):

```python
        if arrived or passed_by:
            self._on_arrival(self.current_idx)
            self.current_idx += 1
            self.locked_cone = None
            self._reset_cone_fix()
            self.last_to_target = None
            self.prev_heading_err = 0.0
            return
```

- [ ] **Step 8: Helper de reset (DRY entre chegada e start_play)**

Modify, adicionar logo após `_start_play` (após a linha 363):

```python
    def _reset_cone_fix(self):
        self._cone_fix_done = False
        self._confirmer.reset()
        self._anchor = None
        self._anchor_status = 'idle'
        self._anchor_clutter = []
        self._anchor_confirm = 0
```

E chamar em `_start_play`, junto dos demais resets (substituir `self.locked_cone = None` no bloco das linhas 358-363):

```python
        self.mode = MODE_PLAY
        self.current_idx = 0
        self.locked_cone = None
        self._reset_cone_fix()
        self.last_to_target = None
        self.prev_heading_err = 0.0
        self.last_msg = f'PLAY {len(self.waypoints)} waypoints'
```

- [ ] **Step 9: Expor a telemetria no `_state_tick`**

Modify o dict `state` em `_state_tick` (linhas 491-502), adicionar os 4 campos:

```python
        state = {
            'mode': self.mode,
            'x': x, 'y': y, 'yaw': yaw,
            'have_pose': have_pose,
            'waypoints': self.waypoints,
            'current_idx': self.current_idx,
            'total': len(self.waypoints),
            'locked_cone': list(self.locked_cone) if self.locked_cone else None,
            'cones': [[c[0], c[1], c[2]] for c in cones],
            'anchor': list(self._anchor) if self._anchor else None,
            'anchor_status': self._anchor_status,
            'anchor_clutter': [list(c) for c in self._anchor_clutter],
            'anchor_confirm': [self._anchor_confirm, self.cone_confirm_frames],
            'msg': self.last_msg,
            'ts': time.time(),
        }
```

- [ ] **Step 10: Smoke test de sintaxe**

Run: `cd ros2_packages/robot_nav && python -c "import ast; ast.parse(open('robot_nav/trekking_runner.py').read()); print('ok')"`
Expected: `ok`

- [ ] **Step 11: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/trekking_runner.py
git commit -m "trekking_runner: confirma cone-ancora, publica /trekking/pose_fix e expoe telemetria da ancora"
```

---

## Task 4: `trekking_runner` aceita `set_cone` (corrigir cone na gravação)

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/trekking_runner.py`

- [ ] **Step 1: Importar `cone_bearing`**

Modify a linha de import dos helpers (a mesma da Task 3 Step 1):

```python
from .cone_pose_fix import ConeFixConfirmer, cone_fix_delta, cone_bearing
```

- [ ] **Step 2: Adicionar o branch no `_on_cmd`**

Modify `_on_cmd`, após o branch `elif cmd == 'clear':` (linhas 264-267), antes do `else:`:

```python
        elif cmd == 'clear':
            self.waypoints = []
            self.current_idx = 0
            self.last_msg = 'lista limpa'
        elif cmd == 'set_cone':
            self._set_wp_cone(data)
        else:
            self.get_logger().warn(f'cmd desconhecido: {cmd}')
```

- [ ] **Step 3: Implementar `_set_wp_cone`**

Modify, adicionar o método logo após `_sanitize_wp` (após a linha 302):

```python
    def _set_wp_cone(self, data: dict):
        # Corrige/limpa o cone preso a um waypoint (só faz sentido fora do PLAY;
        # a UI esconde o controle no PLAY). Mexe em self.waypoints sem lock extra,
        # igual a load_waypoints/_save_point (serializado pelo executor).
        try:
            idx = int(data.get('idx', -1))
        except (TypeError, ValueError):
            self.last_msg = 'set_cone: idx inválido'
            return
        if not (0 <= idx < len(self.waypoints)):
            self.last_msg = f'set_cone: idx {idx} fora da faixa'
            return
        wp = self.waypoints[idx]
        if data.get('clear'):
            wp['has_cone'] = False
            wp['cone_x'] = 0.0
            wp['cone_y'] = 0.0
            wp['cone_bearing'] = 0.0
            self.last_msg = f'wp{idx}: cone removido'
            return
        try:
            cx = float(data['cone_x'])
            cy = float(data['cone_y'])
        except (KeyError, TypeError, ValueError):
            self.last_msg = 'set_cone: cone_x/cone_y inválidos'
            return
        wp['cone_x'] = cx
        wp['cone_y'] = cy
        wp['has_cone'] = True
        # bearing relativo à pose GRAVADA do waypoint (igual à gravação) — sem
        # isso o gate angular do PLAY furaria após a troca.
        wp['cone_bearing'] = cone_bearing(wp['x'], wp['y'], wp['yaw'], cx, cy)
        self.last_msg = f'wp{idx}: cone → ({cx:.2f}, {cy:.2f})'
```

- [ ] **Step 4: Smoke test de sintaxe**

Run: `cd ros2_packages/robot_nav && python -c "import ast; ast.parse(open('robot_nav/trekking_runner.py').read()); print('ok')"`
Expected: `ok`

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/trekking_runner.py
git commit -m "trekking_runner: comando set_cone (corrigir/limpar cone do waypoint na gravacao)"
```

---

## Task 5: `app.py` libera `set_cone` na whitelist

**Files:**
- Modify: `controle_web/app.py`

- [ ] **Step 1: Adicionar cmd + kwargs**

Modify as linhas 576-582:

```python
_TREKKING_CMDS = {
    'reset', 'record', 'save_point', 'play', 'stop',
    'load_waypoints', 'clear', 'set_cone',
}
# Apenas estes kwargs passam para o runner — rejeita o resto pra não acabar
# como vetor de injeção (`os.system` numa lib futura, etc.).
_TREKKING_KWARGS = {'waypoints', 'v_max', 'kp_heading', 'kd_heading',
                    'idx', 'cone_x', 'cone_y', 'clear'}
```

- [ ] **Step 2: Smoke test de sintaxe**

Run: `python -c "import ast; ast.parse(open('controle_web/app.py').read()); print('ok')"`
Expected: `ok`

- [ ] **Step 3: Commit**

```bash
git add controle_web/app.py
git commit -m "controle_web: libera set_cone (idx/cone_x/cone_y/clear) na whitelist do trekking"
```

---

## Task 6: Botão "limpar cone" no `index.html`

**Files:**
- Modify: `controle_web/templates/index.html`

- [ ] **Step 1: Adicionar o botão na toolbar**

Modify, após a linha 169 (`trek-btn-stop`):

```html
          <button id="trek-btn-stop"   class="trek-btn trek-stop">■ Parar</button>
          <button id="trek-btn-clear-cone" class="trek-btn">⊘ Limpar cone</button>
          <span class="trek-sep">|</span>
```

- [ ] **Step 2: Conferir no browser (sem JS ainda)**

Run: `python -c "import ast; ast.parse(open('controle_web/app.py').read()); print('ok')"` (sanidade do servidor)
Verificação manual: abrir a UI no modo trekking e confirmar que o botão "⊘ Limpar cone" aparece na toolbar (ainda sem efeito até a Task 7).

- [ ] **Step 3: Commit**

```bash
git add controle_web/templates/index.html
git commit -m "controle_web: botao 'limpar cone' na toolbar do trekking"
```

---

## Task 7: `trekking.js` — render âncora/clutter/status + clique-pra-corrigir

**Files:**
- Modify: `controle_web/static/js/trekking.js`

- [ ] **Step 1: Estado de seleção + referência ao botão**

Modify, após a linha 27 (`routeSelect = ...`):

```python
  const routeSelect = document.getElementById('trek-route-select');
  const btnClearCone = document.getElementById('trek-btn-clear-cone');

  let selectedWp = null;             // índice do waypoint selecionado p/ editar cone
```

(O bloco acima é JS — o cabeçalho ```python é só p/ destacar; cole como JS.)

- [ ] **Step 2: Render do clutter + âncora em `drawCones`**

Modify `drawCones`, após o bloco do `locked_cone` (linha 127), antes do fecho da função (linha 128):

```javascript
    // Cone trancado (snap atual) — anel amarelo
    if (state.locked_cone) {
      const [cx, cy] = state.locked_cone;
      ctx.strokeStyle = '#fbbf24';
      ctx.lineWidth = 2.5;
      ctx.beginPath(); ctx.arc(view.tx(cx), view.ty(cy), 9, 0, 2*Math.PI); ctx.stroke();
    }
    // Clutter descartado pela unicidade — X magenta
    ctx.strokeStyle = '#d946ef';
    ctx.lineWidth = 1.5;
    (state.anchor_clutter || []).forEach(c => {
      const px = view.tx(c[0]), py = view.ty(c[1]);
      ctx.beginPath();
      ctx.moveTo(px-5, py-5); ctx.lineTo(px+5, py+5);
      ctx.moveTo(px+5, py-5); ctx.lineTo(px-5, py+5);
      ctx.stroke();
    });
    // Âncora de pose (o que ele usa de referência) — anel verde-limão grosso
    if (state.anchor) {
      ctx.strokeStyle = '#a3e635';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.arc(view.tx(state.anchor[0]), view.ty(state.anchor[1]), 11, 0, 2*Math.PI);
      ctx.stroke();
    }
```

- [ ] **Step 3: Destacar o waypoint selecionado em `drawWaypoints`**

Modify `drawWaypoints`, dentro do `wps.forEach`, após desenhar o número do waypoint (após a linha 155, `ctx.fillText(String(idx), px, py);`):

```javascript
      ctx.fillText(String(idx), px, py);
      // Waypoint selecionado p/ edição de cone — anel ciano
      if (idx === selectedWp) {
        ctx.strokeStyle = '#22d3ee';
        ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(px, py, 12, 0, 2*Math.PI); ctx.stroke();
      }
```

- [ ] **Step 4: Badge de status da âncora em `refreshLabels`**

Modify `refreshLabels`, substituir a linha 208 (`snapEl.textContent = ...`):

```javascript
    let snap = state.locked_cone ? '✓ cone trancado' : '';
    if (state.anchor_status && state.anchor_status !== 'idle') {
      const cf = state.anchor_confirm || [0, 0];
      snap += (snap ? ' | ' : '') + `âncora: ${state.anchor_status} ${cf[0]}/${cf[1]}`;
    }
    snapEl.textContent = snap;
```

E, junto dos enables por modo (após a linha 213, `btnSavePt.disabled = ...`):

```javascript
    btnSavePt.disabled  = state.mode === 'play';
    btnClearCone.disabled = state.mode === 'play';
```

- [ ] **Step 5: Clique no canvas — selecionar wp / vincular cone**

Modify, adicionar após o handler do `btnStop` (após a linha 224):

```javascript
  btnStop  .addEventListener('click', () => cmd('stop'));

  // ----- edição de cone na gravação (clica wp, depois clica cone) -----
  function canvasCoords(e) {
    const rect = canvas.getBoundingClientRect();
    return {
      mx: (e.clientX - rect.left) * (canvas.width / rect.width),
      my: (e.clientY - rect.top) * (canvas.height / rect.height),
    };
  }
  canvas.addEventListener('click', (e) => {
    if (!state || state.mode === 'play') return;   // só fora do PLAY
    const {mx, my} = canvasCoords(e);
    const view = computeView();
    const HIT = 12;
    // 1) clicou perto de um waypoint? seleciona
    const wps = state.waypoints || [];
    let bestWp = -1, bestWpD = HIT;
    wps.forEach((wp, idx) => {
      const d = Math.hypot(view.tx(wp.x) - mx, view.ty(wp.y) - my);
      if (d < bestWpD) { bestWpD = d; bestWp = idx; }
    });
    if (bestWp >= 0) {
      selectedWp = bestWp;
      statusEl.textContent = `wp${selectedWp} selecionado — clique num cone p/ vincular`;
      render();
      return;
    }
    // 2) wp selecionado + clicou perto de uma detecção de cone? vincula
    if (selectedWp !== null) {
      const cones = state.cones || [];
      let bestC = -1, bestCD = HIT;
      cones.forEach((c, i) => {
        const d = Math.hypot(view.tx(c[0]) - mx, view.ty(c[1]) - my);
        if (d < bestCD) { bestCD = d; bestC = i; }
      });
      if (bestC >= 0) {
        const c = cones[bestC];
        cmd('set_cone', {idx: selectedWp, cone_x: c[0], cone_y: c[1]});
        statusEl.textContent = `wp${selectedWp}: cone vinculado`;
      }
    }
  });
  btnClearCone.addEventListener('click', () => {
    if (selectedWp === null) {
      statusEl.textContent = 'selecione um waypoint primeiro';
      return;
    }
    cmd('set_cone', {idx: selectedWp, clear: true});
    statusEl.textContent = `wp${selectedWp}: cone removido`;
  });
```

- [ ] **Step 6: Verificação manual no browser**

Subir a UI em modo trekking. Confirmar:
- com detecções de cone na tela, a âncora (quando confirmando) aparece com anel verde-limão e o badge mostra `âncora: confirming N/4`;
- em RECORD, clicar num waypoint o destaca (anel ciano); clicar numa detecção de cone envia `set_cone` (status "cone vinculado") e o cone gravado do waypoint muda;
- "⊘ Limpar cone" com um waypoint selecionado remove o cone (a setinha do bearing some);
- no PLAY o clique não edita e os botões de edição ficam desabilitados.

- [ ] **Step 7: Commit**

```bash
git add controle_web/static/js/trekking.js
git commit -m "trekking.js: render ancora/clutter/status + clique-pra-corrigir cone na gravacao"
```

---

## Task 8: Arg de launch `enable_cone_pose_fix`

**Files:**
- Modify: `ros2_packages/robot_nav/launch/trekking.launch.py`

- [ ] **Step 1: Declarar o arg**

Modify, após `lidar_offset_x_arg` (linha 37):

```python
    lidar_offset_x_arg = DeclareLaunchArgument(
        'lidar_offset_x', default_value='0.10',
        description='Deslocamento x do base_laser em relação a base_link (m)'
    )
    enable_cone_pose_fix_arg = DeclareLaunchArgument(
        'enable_cone_pose_fix', default_value='true',
        description='Liga a correção persistente de pose por cone-âncora (A/B em campo)'
    )
```

- [ ] **Step 2: Repassar ao runner**

Modify o bloco `parameters` do `trekking_runner` (linhas 72-74):

```python
        parameters=[{
            'v_max': LaunchConfiguration('v_max'),
            'enable_cone_pose_fix': LaunchConfiguration('enable_cone_pose_fix'),
        }],
```

- [ ] **Step 3: Adicionar à LaunchDescription**

Modify o `return LaunchDescription([...])` (linhas 80-87):

```python
    return LaunchDescription([
        v_max_arg,
        flow_height_arg,
        lidar_offset_x_arg,
        enable_cone_pose_fix_arg,
        pose_estimator,
        cone_detector,
        trekking_runner,
    ])
```

- [ ] **Step 4: Smoke test de sintaxe**

Run: `cd ros2_packages/robot_nav && python -c "import ast; ast.parse(open('launch/trekking.launch.py').read()); print('ok')"`
Expected: `ok`

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/launch/trekking.launch.py
git commit -m "trekking.launch: arg enable_cone_pose_fix (A/B em campo sem recompilar)"
```

---

## Task 9: Build colcon + verificação de bancada

**Files:** nenhum (verificação).

- [ ] **Step 1: Build do pacote**

Run: `cd /home/rbe-luis/Workspace/Controle_robo_web && colcon build --packages-select robot_nav`
Expected: `Finished <<< robot_nav` sem erro.

- [ ] **Step 2: Unit tests pelo colcon**

Run: `colcon test --packages-select robot_nav --pytest-args -k cone_pose_fix && colcon test-result --verbose`
Expected: 9 passed.
(Fallback: `python -m pytest ros2_packages/robot_nav/test/test_cone_pose_fix.py -v`.)

- [ ] **Step 3: Verificação de bancada (rodas no ar / LiDAR na mesa) — §6 do spec**

> ⚠️ Teste hands-on: ANUNCIAR e ESPERAR o "pode" do usuário antes de abrir a janela de captura (memória `feedback_announce_before_test`).

Após `source install/setup.bash`, subir `robot.launch.py` + `trekking.launch.py` + LiDAR + UI web:

- **Correção de pose:**
  - sem cone visível → `ros2 topic echo /trekking/pose_fix` mudo; pose intacta; `anchor_status=idle`.
  - cone fixo a offset conhecido (gravar 1 wp, PLAY) → um único `/trekking/pose_fix`; log `pose_fix aplicado`; UI mostra âncora verde-limão e status `confirming`→`fixed`.
  - objeto movido na frente → correção rejeitada; UI mostra `confirming` resetando (ou `ambiguous` com clutter magenta se 2 objetos).
  - forçar Δ > `pose_fix_max` → log `pose_fix REJEITADO` + pose intacta.
  - relançar com `enable_cone_pose_fix:=false` → comportamento idêntico ao snap-só-do-alvo.
- **Correção de cone na gravação (UI, RECORD):** gravar um waypoint; clicar nele (anel ciano); clicar outra detecção → cone do waypoint troca; "⊘ Limpar cone" → waypoint sem âncora; conferir que no PLAY a edição fica bloqueada.
- **Campo:** gravar percurso, repetir autônomo com e sem `enable_cone_pose_fix`, medir erro de chegada por waypoint.

---

## Self-Review

**1. Spec coverage:**
- §4.1 interface (pose_fix, observabilidade no state, set_cone) → Tasks 2/3 (pose_fix+telemetria), 4/5 (set_cone). ✅
- §4.2 fluxo (estabilidade, unicidade, delta, 1x, lock+gate) → `ConeFixConfirmer`/`cone_fix_delta` (T1), `_maybe_publish_pose_fix` (T3), `apply_pose_fix`/`_on_pose_fix` (T1/T2). ✅
- §4.4 params (6) → T2 (2) + T3 (4). ✅
- §4.5 camadas → largura/posição/bearing (já existem) + estabilidade/unicidade (T1/T3) + magnitude (T1/T2) + ganho+warn (T2). ✅
- §4.6 observabilidade (anchor/status/clutter/confirm + count property + limitação "fixed sem ack") → T1 (count), T3 (vars+state_tick), T7 (render). ✅
- §4.7 set_cone (clica-wp-cone, recompute bearing, app.py whitelist, service intacto) → T4/T5/T6/T7. ✅
- §5 fallback → T2/T3/T4/T8. ✅
- §6 testes → T1 (unit, incl. `cone_bearing`), T9 (bancada/campo). ✅
- §7 rollout (arg launch) → T8. ✅
- Não-objetivos (sem desvio, sem escolha no PLAY, yaw só IMU) → runner não toca navegação; clique bloqueado no PLAY; `_on_pose_fix` só mexe x/y. ✅

**2. Placeholder scan:** sem TBD/TODO/"handle edge cases"; todo passo de código tem o código. ✅

**3. Type consistency:** `cone_fix_delta(recorded, observed)`, `apply_pose_fix(x,y,dx,dy,gain,max_mag)->(x,y,bool)`, `cone_bearing(wp_x,wp_y,wp_yaw,cone_x,cone_y)->float`, `ConeFixConfirmer(confirm_frames,stable_eps).update(match_pos,n_candidates)->bool`/`.reset()`/`.count` — idênticos entre módulo (T1), testes (T1) e chamadas (T2 `apply_pose_fix`; T3 `ConeFixConfirmer`/`cone_fix_delta`; T4 `cone_bearing`). Tópico `trekking/pose_fix`/frame `odom` casam entre pub (T3) e sub (T2). Campos do state JSON (`anchor`/`anchor_status`/`anchor_clutter`/`anchor_confirm`) idênticos entre T3 (escrita) e T7 (leitura). Comando `set_cone` + kwargs `idx/cone_x/cone_y/clear` casam entre T4 (runner), T5 (whitelist) e T7 (UI). ✅

**Desvio consciente do spec (sinalizado):** o spec define `cone_unique_radius=0.50` < `cone_match_radius=0.60`, o que abriria brecha na trava de unicidade. O plano usa `max(cone_unique_radius, cone_match_radius)` em `_candidates` (Task 3 Step 5) p/ a contagem cobrir sempre a região do match. Não muda a intenção do spec (">1 candidato perto do esperado → ambíguo"); só fecha o buraco. Vale alinhar o spec a esse detalhe.

---

## Execution Handoff

Plano atualizado em `docs/superpowers/plans/2026-06-01-correcao-pose-cone-trekking.md`. Duas opções de execução:

**1. Subagent-Driven (recomendado)** — um subagente fresco por task, revisão entre tasks, iteração rápida.

**2. Inline Execution** — executar as tasks nesta sessão (executing-plans), em lotes com checkpoints.

Qual abordagem?
