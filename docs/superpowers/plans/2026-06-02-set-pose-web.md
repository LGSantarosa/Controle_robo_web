# "Definir pose do robô" na web — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Botão "Definir pose" na web que publica `/initialpose` (click+arrasta no mapa) pra relocalizar o slam_toolbox (slam) e o AMCL (nav2), corrigindo o toque no celular de quebra.

**Architecture:** Tudo no lado web. JS (`map.js`) ganha um helper de coordenada robusto a touch/escala (conserta o "torto" do goal/waypoint também) + um modo armado click-arrasta que emite `set_pose`. `MapBridge` (`map_service.py`) publica `PoseWithCovarianceStamped` em `/initialpose`; `app.py` registra o handler socketio. Zero toque na odometria/launches.

**Tech Stack:** Flask-SocketIO, rclpy, geometry_msgs, JS canvas, ROS2 Jazzy.

Spec: `docs/superpowers/specs/2026-06-02-set-pose-web-design.md`

---

### Task 1: Helper puro `build_initialpose` + teste

**Files:**
- Modify: `controle_web/map_service.py` (import + função module-level, perto de `_yaw_to_quat`)
- Test: `controle_web/test_map_service_initialpose.py` (novo)

- [ ] **Step 1: Escrever o teste que falha**

```python
# controle_web/test_map_service_initialpose.py
import math
from map_service import build_initialpose
from builtin_interfaces.msg import Time


def test_build_initialpose_frame_position_quat_cov():
    msg = build_initialpose(1.0, 2.0, math.pi / 2, Time())
    assert msg.header.frame_id == 'map'
    assert abs(msg.pose.pose.position.x - 1.0) < 1e-9
    assert abs(msg.pose.pose.position.y - 2.0) < 1e-9
    # yaw=pi/2 -> qz=qw=0.7071
    assert abs(msg.pose.pose.orientation.z - 0.70710678) < 1e-3
    assert abs(msg.pose.pose.orientation.w - 0.70710678) < 1e-3
    assert msg.pose.covariance[0] == 0.25     # var x
    assert msg.pose.covariance[7] == 0.25     # var y
    assert msg.pose.covariance[35] > 0.0      # var yaw
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `cd controle_web && source /opt/ros/jazzy/setup.bash && python3 -m pytest test_map_service_initialpose.py -q`
Expected: FAIL (`ImportError: cannot import name 'build_initialpose'`).

- [ ] **Step 3: Implementar o helper**

Em `controle_web/map_service.py`, adicionar ao import de geometry_msgs `PoseWithCovarianceStamped` e, perto de `_yaw_to_quat`:
```python
def build_initialpose(x, y, yaw, stamp):
    """Monta PoseWithCovarianceStamped (frame 'map') pra /initialpose.

    Covariância diagonal moderada: confiante mas não absoluta — no AMCL é a
    dispersão inicial das partículas; no slam_toolbox é quase ignorada (seta
    a pose direto).
    """
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = stamp
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    qx, qy, qz, qw = _yaw_to_quat(float(yaw))
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    cov = [0.0] * 36
    cov[0] = 0.25      # var(x)  m²
    cov[7] = 0.25      # var(y)  m²
    cov[35] = 0.0685   # var(yaw) rad² (~15° 1σ)
    msg.pose.covariance = cov
    return msg
```

- [ ] **Step 4: Rodar e ver passar**

Run: `cd controle_web && source /opt/ros/jazzy/setup.bash && python3 -m pytest test_map_service_initialpose.py -q`
Expected: PASS (1 passed).

- [ ] **Step 5: Commit**

```bash
git add controle_web/map_service.py controle_web/test_map_service_initialpose.py
git commit -m "map_service: build_initialpose (PoseWithCovarianceStamped p/ /initialpose)"
```

---

### Task 2: Publisher `/initialpose` + `MapBridge.set_pose`

**Files:**
- Modify: `controle_web/map_service.py` (publisher no `__init__` perto do `_goal_pub` ~linha 120; método `set_pose` perto de `send_goal` ~linha 541)

- [ ] **Step 1: Adicionar o publisher no `__init__`**

Depois do bloco `self._goal_pub = ...` (linha ~120-122):
```python
        self._initialpose_pub = self._node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
```

- [ ] **Step 2: Adicionar o método `set_pose`**

Perto de `send_goal` (~linha 541):
```python
    def set_pose(self, x: float, y: float, yaw: float = 0.0) -> dict:
        """Relocaliza: publica PoseWithCovarianceStamped em /initialpose.

        slam_toolbox (slam) re-ancora map→odom; AMCL (nav2) re-semeia as
        partículas. Frame 'map'.
        """
        stamp = self._node.get_clock().now().to_msg()
        msg = build_initialpose(x, y, yaw, stamp)
        self._initialpose_pub.publish(msg)
        log.info(f"[MapBridge] /initialpose → ({x:.2f}, {y:.2f}, yaw={yaw:.2f})")
        return {'ok': True, 'x': x, 'y': y, 'yaw': yaw}
```

- [ ] **Step 3: Sanity de import (módulo carrega)**

Run: `cd controle_web && source /opt/ros/jazzy/setup.bash && python3 -c "import map_service; print('ok')"`
Expected: `ok`.

- [ ] **Step 4: Commit**

```bash
git add controle_web/map_service.py
git commit -m "map_service: MapBridge.set_pose publica /initialpose (relocalizacao manual)"
```

---

### Task 3: Handler socketio `set_pose` no app.py

**Files:**
- Modify: `controle_web/app.py` (novo `@socketio.on('set_pose')`, espelha `handle_nav_goal` ~linha 276)

- [ ] **Step 1: Adicionar o handler**

Depois do `handle_nav_goal` (~linha 293):
```python
@socketio.on('set_pose')
def handle_set_pose(data):
    """Cliente definiu a pose real no mapa: publica /initialpose (relocaliza)."""
    if map_bridge is None:
        emit('set_pose_ack', {'ok': False, 'error': 'mapa indisponível neste modo'})
        return
    if ROBOT_MODE not in ('slam', 'nav2'):
        emit('set_pose_ack', {'ok': False, 'error': 'definir pose só vale em SLAM ou NAV2'})
        return
    try:
        x, y = _validate_xy(data.get('x'), data.get('y'))
        yaw = _validate_yaw(data.get('yaw', 0.0))
        result = map_bridge.set_pose(x, y, yaw)
        app.logger.info(f"set_pose from {request.remote_addr}: ({x:.2f}, {y:.2f}, {yaw:.2f})")
        emit('set_pose_ack', result)
    except Exception as e:
        emit('set_pose_ack', {'ok': False, 'error': str(e)})
```

- [ ] **Step 2: Sanity (app importa sem erro de sintaxe)**

Run: `cd controle_web && source /opt/ros/jazzy/setup.bash && python3 -c "import ast; ast.parse(open('app.py').read()); print('sintaxe ok')"`
Expected: `sintaxe ok`.

- [ ] **Step 3: Commit**

```bash
git add controle_web/app.py
git commit -m "app: socketio set_pose -> MapBridge.set_pose (gate slam/nav2)"
```

---

### Task 4: `eventToCanvasPx` (fix touch/escala) + refactor dos handlers

**Files:**
- Modify: `controle_web/static/js/map.js` (helper novo + mousedown/move/up ~linha 271-337)

- [ ] **Step 1: Adicionar o helper compartilhado**

Perto dos helpers de transformação (~linha 340):
```javascript
  // Coordenada do evento → pixel INTERNO do canvas (corrige o "torto" no mobile:
  // o canvas é exibido por CSS em tamanho != canvas.width/height). Funciona pra
  // mouse E touch.
  function eventToCanvasPx(ev) {
    const rect = canvas.getBoundingClientRect();
    const src = (ev.touches && ev.touches[0]) ? ev.touches[0]
              : (ev.changedTouches && ev.changedTouches[0]) ? ev.changedTouches[0]
              : ev;
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    return {
      cx: (src.clientX - rect.left) * scaleX,
      cy: (src.clientY - rect.top) * scaleY,
    };
  }
```

- [ ] **Step 2: Trocar `ev.clientX - rect.left` pelo helper nos 3 handlers**

Em `mousedown` (271): trocar
```javascript
      const rect = canvas.getBoundingClientRect();
      const cx = ev.clientX - rect.left;
      const cy = ev.clientY - rect.top;
```
por
```javascript
      const { cx, cy } = eventToCanvasPx(ev);
```
Repetir o mesmo em `mousemove` (linhas 286-288 → `const { cx, cy } = eventToCanvasPx(ev); wpDrag.curX = cx; wpDrag.curY = cy;`) e em `mouseup` (294-296 → `const { cx, cy } = eventToCanvasPx(ev);`).

- [ ] **Step 3: Verificar no navegador (desktop) que goal/waypoint ainda caem certo**

Run: subir o app em modo nav2 no dev/sim, clicar no mapa, confirmar que o alvo cai sob o cursor. (Sem harness JS — verificação visual.)
Expected: clique cai no ponto certo (e agora robusto a resize).

- [ ] **Step 4: Commit**

```bash
git add controle_web/static/js/map.js
git commit -m "map.js: eventToCanvasPx (escala + touch) — corrige clique torto no mobile"
```

---

### Task 5: Modo armado "Definir pose" no map.js

**Files:**
- Modify: `controle_web/static/js/map.js` (estado + toggle + emit + ack + touch listeners + render da seta)

- [ ] **Step 1: Estado + função de armar**

Perto do `setWpMode` (~linha 52):
```javascript
  let setPoseMode = false;   // armado: próximo click-arrasta define a pose real
  let setPoseDrag = null;    // { canvasX, canvasY, curX, curY, world }
  function setSetPoseMode(on) {
    setPoseMode = on;
    if (on) setWpMode(false);            // exclusivos
    canvas.style.cursor = on ? 'crosshair' : 'default';
    const btn = document.getElementById('btn-set-pose');
    if (btn) btn.classList.toggle('active', on);
    if (clickHint) clickHint.textContent = on
      ? 'Definir pose: clique onde o robô está e arraste pra direção'
      : '';
  }
```

- [ ] **Step 2: Tratar o set-pose no mousedown/up (antes dos ramos de wp/goal)**

No `mousedown`, logo após obter `cx,cy` e `world`:
```javascript
      if (setPoseMode) {
        setPoseDrag = { canvasX: cx, canvasY: cy, curX: cx, curY: cy, world };
        return;
      }
```
No `mousemove`, no topo:
```javascript
      if (setPoseMode && setPoseDrag) {
        const { cx, cy } = eventToCanvasPx(ev);
        setPoseDrag.curX = cx; setPoseDrag.curY = cy; render();
        return;
      }
```
No `mouseup`, antes do ramo waypoint:
```javascript
      if (setPoseMode && setPoseDrag) {
        const ddx = cx - setPoseDrag.canvasX;
        const ddy = cy - setPoseDrag.canvasY;
        const dragged = Math.sqrt(ddx*ddx + ddy*ddy) > DRAG_THRESHOLD;
        const yaw = dragged ? Math.atan2(-ddy, ddx) : 0.0;   // canvas y p/ baixo
        const w = setPoseDrag.world;
        socket.emit('set_pose', { x: w.x, y: w.y, yaw });
        statusEl.textContent = `pose definida: (${w.x.toFixed(2)}, ${w.y.toFixed(2)})`;
        setPoseDrag = null;
        setSetPoseMode(false);   // one-shot
        render();
        return;
      }
```

- [ ] **Step 3: Touch listeners (reusam os mesmos handlers de mouse)**

Após os listeners de mouse (~linha 337), registrar touch encaminhando pros mesmos callbacks e prevenindo scroll no drag:
```javascript
    canvas.addEventListener('touchstart', (ev) => {
      ev.preventDefault();
      canvas.dispatchEvent(new MouseEvent('mousedown', {
        clientX: ev.touches[0].clientX, clientY: ev.touches[0].clientY }));
    }, { passive: false });
    canvas.addEventListener('touchmove', (ev) => {
      ev.preventDefault();
      canvas.dispatchEvent(new MouseEvent('mousemove', {
        clientX: ev.touches[0].clientX, clientY: ev.touches[0].clientY }));
    }, { passive: false });
    canvas.addEventListener('touchend', (ev) => {
      ev.preventDefault();
      const t = ev.changedTouches[0];
      canvas.dispatchEvent(new MouseEvent('mouseup', {
        clientX: t.clientX, clientY: t.clientY }));
    }, { passive: false });
```
(O `eventToCanvasPx` já trata `MouseEvent` sintético — usa `clientX/clientY`.)

- [ ] **Step 4: Render da seta de set-pose + ack + visibilidade por modo**

No `render()`, junto do desenho do `wpDrag`, desenhar a seta de `setPoseDrag` (de `canvasX,canvasY` até `curX,curY`). No bloco de listeners, ligar o botão e o ack:
```javascript
    const btnSetPose = document.getElementById('btn-set-pose');
    if (btnSetPose) btnSetPose.addEventListener('click', () => setSetPoseMode(!setPoseMode));
    socket.on('set_pose_ack', (data) => {
      statusEl.textContent = data.ok
        ? `pose aplicada: (${data.x.toFixed(2)}, ${data.y.toFixed(2)})`
        : `falha ao definir pose: ${data.error}`;
    });
```
No handler `mode_info` (~linha 73), mostrar o botão só em slam/nav2:
```javascript
      if (btnSetPose) btnSetPose.style.display =
        (currentMode === 'slam' || currentMode === 'nav2') ? '' : 'none';
```

- [ ] **Step 5: Verificação visual (desktop)**

Subir em modo slam (sim), clicar "Definir pose", click+arrasta no mapa, confirmar emit `set_pose` no console e a seta desenhada.

- [ ] **Step 6: Commit**

```bash
git add controle_web/static/js/map.js
git commit -m "map.js: modo armado 'Definir pose' (click+arrasta) -> set_pose; touch"
```

---

### Task 6: Botão "Definir pose" no index.html

**Files:**
- Modify: `controle_web/templates/index.html` (perto do `#map-click-hint` ~linha 127 / controles do mapa)

- [ ] **Step 1: Adicionar o botão**

Junto dos controles do mapa (perto da linha 127, onde estão os botões de waypoint):
```html
          <button id="btn-set-pose" type="button" class="map-btn">Definir pose</button>
```
(usar a mesma classe CSS dos outros botões de mapa do template — conferir o nome real da classe ao editar.)

- [ ] **Step 2: Verificar render**

Run: subir o app, abrir a página em modo slam, confirmar que o botão "Definir pose" aparece (e some em teleop/trekking).
Expected: botão visível em slam/nav2.

- [ ] **Step 3: Commit**

```bash
git add controle_web/templates/index.html
git commit -m "index.html: botao 'Definir pose' (visivel em slam/nav2)"
```

---

### Task 7: Push + deploy + validação de bancada (precisa da Pi + celular)

**Files:** nenhum (deploy + teste hands-on)

- [ ] **Step 1: Push**

```bash
git push origin feat/odometria-fundida
```

- [ ] **Step 2: Deploy na Pi**

```bash
# na Pi: git fetch && git reset --hard origin/feat/odometria-fundida
#        colcon build --packages-select robot_nav   (web não precisa de build; app.py/JS são lidos do source)
```
(Conferir se o servidor web roda do source ou do install — se do install, rebuildar o pacote que instala controle_web; senão, só reiniciar o app.)

- [ ] **Step 3: Validação (hands-on — anunciar e esperar "pode", memória `feedback_announce_before_test`)**

`--slam`: deixar a pose ficar errada, "Definir pose" → click+arrasta → confirmar que `map→base_link` salta pro lugar e o slam segue coerente. `--nav2`: idem, AMCL converge. **No celular:** confirmar que o ponto cai sob o dedo (set-pose, goal, waypoint).

- [ ] **Step 4: Marcar validado no spec/memória.**

---

## Self-Review

- **Spec coverage:** UI toggle armado one-shot (T5) ✓; click+arrasta heading (T5) ✓; `eventToCanvasPx` touch/escala compartilhado, conserta goal/waypoint (T4) ✓; `/initialpose` publisher + `build_initialpose` (T1,T2) ✓; handler socketio gate slam/nav2 (T3) ✓; botão só em slam/nav2 (T5 mode_info + T6) ✓; validação bancada + touch no celular (T7) ✓.
- **Placeholders:** "conferir o nome real da classe CSS" (T6) e "do source vs install" (T7) são verificações pontuais no ato, não lacunas de design.
- **Consistência:** evento `set_pose`/`set_pose_ack`, método `set_pose`, helper `build_initialpose`, id `btn-set-pose` usados igual em todas as tasks; `eventToCanvasPx` retorna `{cx,cy}` consumido igual nos 3 handlers.
