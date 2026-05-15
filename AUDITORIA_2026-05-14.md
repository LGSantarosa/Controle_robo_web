# Auditoria do projeto Controle_robo_web — 2026-05-14

Documento gerado por auditoria completa (firmware MEGA + ROS2 + Flask/web + shell scripts) cruzando o código com o `README.md`. Use como roteiro para aplicar correções numa próxima sessão.

> **Pré-requisito para usar este documento:** abrir uma nova sessão na raiz `/home/rbe-luis/Workspace/Controle_robo_web/`. Cada item traz `arquivo:linha`, descrição e correção sugerida. Severidade: 🔴 crítica (mexer antes de rodar no robô real), 🟠 alta (bug real), 🟡 média (qualidade/robustez), 🟢 baixa (cosmético).

---

## Decisões já tomadas (2026-05-14)

### Decisão D1 — Remover o `collision_monitor` inteiramente

O usuário decidiu **descartar** o `nav2_collision_monitor`. Não tentar consertar o fluxo `/cmd_vel_filtered`. **Apagar dos seguintes pontos** (lista completa e exata):

**Arquivos para deletar:**
- `ros2_packages/robot_nav/launch/nav2_collision.launch.py`
- `ros2_packages/robot_nav/config/collision_monitor.yaml`
- `controle_web/logs/nav2_collision.log` (se existir; é gerado em runtime)
- `worlds/small_box.sdf` (README:276,811 declara que existe só para "ensaiar collision_monitor")

**Edições:**
- `launch.sh:378-394` (bloco `teleop)` inteiro): remover o `elif` que sobe `nav2_collision.launch.py`. Deixar apenas: `echo "[3/4] Modo TELEOP — sem collision monitor."`. Também remover `--no-nav2` se ele só servia para esse caso (verificar se ainda faz sentido em outros modos — só é usado nesse trecho, então pode tirar).
  - `launch.sh:28` — `NO_NAV2=false` (linha do default)
  - `launch.sh:51` — `--no-nav2) NO_NAV2=true ;;` (linha do parser)
  - `launch.sh:381` — `[ "$NO_NAV2" = false ] && ros2 pkg list...`
- `ros2_packages/robot_nav/package.xml:24` — remover `<exec_depend>nav2_collision_monitor</exec_depend>`
- `ros2_packages/robot_nav/robot_nav/cmd_vel_to_wheels.py:3` — atualizar docstring: trocar "Twist (/cmd_vel_filtered from Nav2 Collision Monitor) into wheel speeds" por "Twist (/cmd_vel) into wheel speeds"
- `ros2_packages/robot_nav/launch/robot.launch.py:9` — atualizar comentário: `cmd_vel_to_wheels (/cmd_vel → /wheel_vel_setpoints)`
- `ros2_packages/robot_nav/launch/robot.launch.py:94` — confirmar que `cmd_vel_topic` continua `'cmd_vel'` (não precisa mudar — já está correto)
- `ros2_packages/robot_nav/config/nav2_params.yaml:5` — remover comentário "interceptado pelo collision_monitor → /cmd_vel_filtered". Substituir por "Saída direta: /cmd_vel."
- `setup.sh:26` — remover `ros-jazzy-nav2-collision-monitor` da lista apt
- `setup_pi.sh:95` — remover `"ros-${ROS_DISTRO}-nav2-collision-monitor"`
- `install_nav2.sh:10` — remover `ros-jazzy-nav2-collision-monitor` (e ver decisão D2 abaixo sobre esse script)

**README.md — edições:**
- `README.md:64` — apt list: remover `nav2-collision-monitor`
- `README.md:175` — remover `+ nav2_collision_mon.` do diagrama
- `README.md:204` — remover linha "TELEOP | (padrão) | Dirigir manualmente | `nav2_collision_monitor`..." e substituir por "Dirigir manualmente — só web + LiDAR + nodes do robô"
- `README.md:211-217` — apagar a seção inteira "Espera, por que aparece 'nav2' em dois lugares?" (era explicação do collision_monitor)
- `README.md:276` — remover menção a `small_box.sdf` "mundo mínimo pra ensaiar collision_monitor"
- `README.md:300` — remover `ros-$ROS_DISTRO-nav2-collision-monitor \` do bloco apt
- `README.md:452` — trocar "Sobe o `nav2_collision_monitor` como camada de segurança" por "Dirige manualmente; nenhuma camada extra de segurança"
- `README.md:463` — remover linha da tabela: `| 3 | nav2_collision_monitor (só segurança) | logs/nav2_collision.log |` e renumerar tabela (passa a ter 3 linhas)
- `README.md:472` — trocar "Em vez do collision_monitor sobe o slam_toolbox" por "Sobe o slam_toolbox (mapping online async)"
- `README.md:573` — remover linha `./launch.sh --no-nav2 # Teleop sem collision_monitor`
- `README.md:723` — na tabela de tópicos, em `/scan`: remover `/ nav2_collision_monitor` da coluna "Consumidor"
- `README.md:811` — remover linha `small_box.sdf  # Mundo mínimo pra ensaio de collision_monitor` da árvore de arquivos
- `README.md:818` — trocar `launch/  # robot, lidar, slam, nav2, nav2_collision, sim, trekking` por `launch/  # robot, lidar, slam, nav2, sim, trekking`
- `README.md:827` — remover linha `collision_monitor.yaml  # Zonas de freada (modo teleop)`
- `README.md:913` — remover linha da tabela de logs: `| nav2_collision.log | Nav2 Collision Monitor (modo teleop) |`

**Confirmar:** depois das remoções, `grep -rn -i "collision_monitor\|nav2_collision\|cmd_vel_filtered" .` deve retornar zero hits.

---

## 🔴 CRÍTICOS

### C1 — `robot_radius: 0.18` subdimensiona o robô (50×50 cm)

Raio inscrito real do chassi = 0.25 m; circunscrito = 0.354 m. Nav2 planeja trajetórias que raspam/colidem.

- `ros2_packages/robot_nav/config/nav2_params.yaml:155` (local_costmap)
- `ros2_packages/robot_nav/config/nav2_params.yaml:195` (global_costmap)
- `ros2_packages/robot_nav/config/nav2_params_pi.yaml:154`
- `ros2_packages/robot_nav/config/nav2_params_pi.yaml:189`

**Fix:** substituir `robot_radius: 0.18` por footprint poligonal:
```yaml
footprint: "[[0.25, 0.25], [0.25, -0.25], [-0.25, -0.25], [-0.25, 0.25]]"
```
Alternativa rápida: `robot_radius: 0.30`.

### C2 — Inversão L/R hardcoded no comando + sinal paramétrico no feedback

Dois caminhos diferentes para corrigir polaridade — odom e comando podem divergir, AMCL/EKF ficam loucos.

- `ros2_packages/robot_nav/robot_nav/cmd_vel_to_wheels.py:67-68` — troca os campos `wheels.right_wheel = float(left)` e `wheels.left_wheel = float(right)` (com comentário "fios invertidos")
- `ros2_packages/robot_nav/robot_nav/odom_publisher.py:91` — aplica `self.left_sign if which in ('fl','rl') else self.right_sign`

**Fix:**
1. Em `cmd_vel_to_wheels.py:67-68`, voltar para atribuição direta (`wheels.left_wheel = float(left)`, `wheels.right_wheel = float(right)`).
2. Adicionar parâmetros `left_wheel_sign` / `right_wheel_sign` em `cmd_vel_to_wheels.py` e multiplicar antes de publicar.
3. Garantir que o launch passa o mesmo valor para os dois nós (`odom_publisher` e `cmd_vel_to_wheels`).

### C3 — `pose_covariance` / `twist_covariance` zeradas em `/odom`

AMCL/EKF ou ignora a odom ou trata como infinitamente confiável. Sintoma: localização ruidosa ou rígida demais.

- `ros2_packages/robot_nav/robot_nav/odom_publisher.py:120-141`

**Fix:** preencher a diagonal:
```python
odom.pose.covariance[0]  = 0.05   # x
odom.pose.covariance[7]  = 0.05   # y
odom.pose.covariance[35] = 0.10   # yaw
odom.twist.covariance[0]  = 0.01  # vx
odom.twist.covariance[35] = 0.05  # vyaw
```

### C4 — `flow_link` posicionado **abaixo do chão** + 3 valores diferentes para a mesma altura

Três fontes divergentes para "altura do PMW3901 ao chão":
- `ros2_packages/robot_nav/urdf/robot.urdf.xacro:30-32` posiciona `flow_link` em z ≈ -0.075 m relativo ao `base_footprint` (abaixo do chão — fisicamente impossível)
- `ros2_packages/robot_nav/launch/trekking.launch.py:39` usa default `flow_height: 0.12` (12 cm)
- `README.md:628` diz "~3 cm pela posição no URDF"

**Fix:**
1. Medir a altura física real do PMW3901 ao chão.
2. Corrigir o URDF (linha do `flow_joint`) para que `flow_link` fique em z positivo correspondente.
3. Alinhar default do `pose_estimator.flow_height` ao mesmo valor.
4. Atualizar `README.md:628` com o valor real.

### C5 — PMW3901 envia `quality = 0` constante

O `pose_estimator` faz `alpha = sigmoid((quality - q_mid)/q_slope)` para decidir quando confiar no flow. Com quality sempre 0, **o flow nunca é fundido** — o trekking volta a depender só das rodas (e patina em grama / piso liso).

- `firmware/mega_bridge/src/sensors_flow.cpp:15` — `quality_ = 0;` literal

**Fix:** o lib Bitcraze PMW3901 expõe `readMotionCount(dx, dy)` mas não a quality diretamente. Ler o registrador `0x07` (SQUAL) manualmente via `Pmw3901.h` privado ou usar fork com getter. Se não houver tempo, **avisar no README** que a fusão está desligada e marcar como TODO.

### C6 — Feedback stale das placas de hoverboard

Se uma placa cai (cabo, alimentação), `fb_front.last()` continua retornando o último frame bom **para sempre**. MEGA republica RPMs antigos como atuais → `odom_publisher` integra valores fantasmas.

- `firmware/mega_bridge/include/hoverboard.h:32-43` — `FeedbackParser` não expõe `last_recv_ms`
- `firmware/mega_bridge/src/main.cpp:130-138` — `txState()` usa `fb_front.last()` / `fb_rear.last()` sem checar staleness

**Fix:**
1. Em `hoverboard.h`: adicionar `uint32_t last_recv_ms_ = 0;` e setá-lo em `parseByte` quando checksum bate.
2. Adicionar método `bool stale(uint32_t now_ms, uint32_t timeout=200)`.
3. Em `main.cpp:130-138`: se `stale()`, publicar zeros em vez do último frame.

### C7 — Web sem autenticação + CORS aberto + SECRET_KEY hardcoded

Qualquer dispositivo na LAN comanda o robô.

- `controle_web/app.py:79` — `SECRET_KEY = 'change-me'`
- `controle_web/app.py:83` — `cors_allowed_origins="*"`
- `controle_web/app.py:487` — `host='0.0.0.0'`

**Fix mínimo (não bloqueia uso interno):**
```python
import secrets, os
app.config['SECRET_KEY'] = os.environ.get('FLASK_SECRET_KEY') or secrets.token_hex(32)
socketio = SocketIO(app, async_mode='threading',
                   cors_allowed_origins=os.environ.get('CORS_ORIGIN', '*').split(','))
```
**Fix completo:** adicionar token simples por query string (`?token=`) validado no `handle_connect`.

### C8 — `cmd_vel_to_wheels`: scale linear/angular sem amarração à cinemática

`Klin=400`, `Kang=150` magic numbers sem relação com `wheel_base=0.50`. O `odom_publisher` faz a fórmula correta `ω = (vR-vL)/wheel_base`, mas o `cmd_vel_to_wheels` não usa `wheel_base` — então comandar `angular=1 rad/s` nunca produz `1 rad/s` medido.

- `ros2_packages/robot_nav/robot_nav/cmd_vel_to_wheels.py:54-55`

**Fix:** derivar `Kang` de `Klin` e geometria:
```python
# left  = (linear - angular * wheel_base/2) * scale_per_m_s
# right = (linear + angular * wheel_base/2) * scale_per_m_s
```
Ou pelo menos: documentar a relação no comentário acima dos params e fazer benchmark `cmd_vel.angular=1 ↔ /odom.twist.twist.angular` para calibrar.

### C9 — Gamepad: documentação vs código completamente trocados

| | README:603-606 | `static/js/gamepad.js:185-211` |
|---|--------|----------------------|
| □ (square / X) | reduz para 0.8× | **boost 2.0×** |
| ○ (circle / B) | aumenta até 4× | **fine 0.75×** |

E ainda inconsistência de limites:
- `controle_web/templates/index.html:27` — slider `min="0.5"`
- `controle_web/templates/index.html:35` — preset `data-mult="0.75"`
- `controle_web/controllers/robot_controller.py:132-133` — `SPEED_MULT_MIN=0.8` (server clipa silenciosamente)

**Fix:** decidir qual é a verdade (provavelmente código está certo, README ficou pra trás). Alinhar os 4 pontos:
1. README:603-606
2. `gamepad.js:185-211`
3. `index.html:27` e `:35`
4. `robot_controller.py:132-133`

---

## 🟠 ALTOS — bugs reais

### A1 — Race conditions em `MapBridge` (waypoints)
- `controle_web/map_service.py:144-154` e `:386` — estado de waypoints (`_wp_list`, `_wp_loop`, `_wp_goal_handle`, `_wp_goal_status`) acessado de múltiplas threads sem `threading.Lock`.
- `controle_web/map_service.py:267-289` — `_wp_stop` pode ficar zumbi se `_wp_runner` está bloqueado em `wait_for_server(2.0)`.

**Fix:** envolver mutações de estado de waypoint em `with self._wp_lock:`. Capturar o `_wp_stop` Event no construtor e reutilizar em vez de recriar.

### A2 — Shutdown global do `rclpy` derruba bridges
- `controle_web/app.py:23,55-58` — `ROS2Controller()` instanciado em import-time; `_shutdown_all` chama `controller.shutdown()` que faz `rclpy.shutdown()` sobre o contexto compartilhado com `MapBridge`/`TrekkingBridge`/`NavMetricsCollector`. Bridges podem morrer no meio de callback.
- `controle_web/controllers/robot_controller.py:172-180`

**Fix:** separar `node.destroy_node()` do `rclpy.shutdown()`. Só o último `_shutdown_all` deve chamar `rclpy.shutdown()`, e por último.

### A3 — Atomicidade quebrada (saves não-atômicos)
Crash deixa JSON/CSV parcial.
- `controle_web/map_service.py:307` — `save_route`
- `controle_web/trekking_service.py:122-129` — `save_route` trekking
- `controle_web/nav_metrics.py:280-298` — CSV de métricas

**Fix:** `tempfile.NamedTemporaryFile(dir=...)` + `os.replace(tmp, final)`. Para o CSV, abrir arquivo uma vez no `__init__` e fazer `f.flush(); os.fsync(f.fileno())` após cada `writerow`.

### A4 — `nav_metrics`: CSV path travado no `__init__`
- `controle_web/nav_metrics.py:97-99` — `csv_path` fixado com data daquele instante. Servidor rodando 24h grava registros do dia novo no CSV do dia anterior.

**Fix:** recomputar `csv_path` a cada `_flush_attempt` baseado em `time.strftime('%Y%m%d', time.localtime(self._attempt.end_ts))`.

### A5 — `nav_metrics`: `time.time()` em vez do clock ROS
- `controle_web/nav_metrics.py:233,259` — wall clock; quebra em modo `--sim` com `use_sim_time:=true`.

**Fix:** `self._node.get_clock().now().nanoseconds * 1e-9`.

### A6 — `nav_metrics`: distância integra velocidade reportada
- `controle_web/nav_metrics.py:247` — `distance_traveled_m += abs(speed)*dt`. Em slip, distância é inflada.

**Fix:** usar `math.hypot(x - last_x, y - last_y)` da `/odom.pose.pose.position`.

### A7 — `client.js`: keydown dispara em campos de input
- `controle_web/static/js/client.js:233-244` — digitar nome de rota no prompt = mover robô.

**Fix:** no topo do handler:
```js
if (e.target.matches('input,select,textarea')) return;
```

### A8 — Disconnect/reconnect não força stop
- `controle_web/static/js/client.js:119-140` — se cliente cai com tecla pressionada, servidor continua publicando último `/cmd_vel` até reconnect.

**Fix:** no handler `disconnect` do servidor (`controle_web/app.py`), zerar `pressed` e publicar `Twist()` zero.

### A9 — Resync do FSM falha em `0xAA 0xAA 0x55`
- `firmware/mega_bridge/src/protocol.cpp:24-27` — em `S1`, se receber `0xAA` (e não `0x55`), volta a `S0`. Mesmo bug copiado em `controle_web/.../mega_bridge.py:80-86` (do lado Python).

**Fix C++:**
```cpp
case S1:
    if (b == START1)      st_ = TYPE;
    else if (b == START0) st_ = S1;   // novo header, mantém em S1
    else                  st_ = S0;
    break;
```
Replicar mesma correção no Python.

### A10 — `int(round(...))` cria deadband no comando
- `ros2_packages/robot_nav/robot_nav/mega_bridge.py:174-191` — `_wheelspeeds_to_steer_speed` arredonda; valores fracionários pequenos viram 0.

**Fix:** ou aumentar a escala antes do `int()` ou aceitar deadband mas tornar parâmetro.

### A11 — `mega_bridge.py`: QoS reliable depth=10 para sensores de alta freq
- `ros2_packages/robot_nav/robot_nav/mega_bridge.py:141` — `/imu/data` (50 Hz) e `/optical_flow` (100 Hz) com `RELIABLE` força reenvio sob jitter.

**Fix:** QoS separado:
```python
from rclpy.qos import qos_profile_sensor_data
self._pub_imu = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
self._pub_flow = self.create_publisher(Vector3Stamped, 'optical_flow', qos_profile_sensor_data)
```
`/wheel_vel_setpoints` (subscribed) continua com `RELIABLE`.

### A12 — `firmware/mega_bridge`: BNO055 sem fallback automático para 0x29
- `firmware/mega_bridge/include/sensors_imu.h:18` — `Adafruit_BNO055 bno_{55, BNO055_ADDRESS_A, &Wire};` hardcoded 0x28.
- `README.md:1000-1002` promete fallback automático, mas precisa de edição manual.

**Fix:** em `sensors_imu.cpp:6-12`, se `bno_.begin()` falhar com 0x28, tentar reconstruir com 0x29 e retentar.

### A13 — `firmware/mega_bridge`: IMU/flow `ok_` não re-testado runtime
- `firmware/mega_bridge/src/main.cpp:172-173` — `imu_dev.read()` só falha se `ok_` foi setado false em `begin()`. Cabo solto runtime = lixo silencioso.

**Fix:** em `Imu::read()`, verificar timeout/ack do I²C; se falhar, setar `ok_ = false` e tentar `begin()` de novo a cada N segundos.

### A14 — `firmware/leds.cpp`: brilho 255 em 24 LEDs brancos → brown-out
- `firmware/mega_bridge/src/leds.cpp:7` — `setBrightness(255)`
- `firmware/mega_bridge/src/leds.cpp:116` — modo RUN preenche todos os 24 LEDs em branco

24×60 mA ≈ 1.44 A no 5 V. USB do MEGA fornece 500 mA — risco de brown-out.

**Fix:** `setBrightness(80)` ou alimentar o anel WS2812 por linha separada.

### A15 — `urdf/robot.urdf.xacro`: inércia `izz` subdimensionada
- `ros2_packages/robot_nav/urdf/robot.urdf.xacro:57` — `ixx=iyy=izz=0.5`

Cálculo físico para caixa 0.50×0.50×0.26 m de 15.46 kg:
```
ixx = m(y²+z²)/12 ≈ 0.41
iyy = m(x²+z²)/12 ≈ 0.41
izz = m(x²+y²)/12 ≈ 0.64
```

**Fix:** atualizar:
```xml
<inertia ixx="0.41" iyy="0.41" izz="0.64" ixy="0" ixz="0" iyz="0"/>
```
(Cosmético no URDF real, mas afeta dinâmica se algum dia migrar para sim com este URDF.)

### A16 — `test_serial.py` totalmente obsoleto
- `test_serial.py` inteiro — usa protocolo `0xABCD` direto + porta `/dev/ttyUSB0`. Protocolo PC↔MEGA atual é `0xAA 0x55` via `/dev/mega`.

**Fix:** deletar o arquivo. Se quiser manter para teste de mock, mover para `firmware/mega_bridge/tools/` e reescrever para o novo protocolo.

### A17 — Cache de build cobre só `robot_nav/`
- `launch.sh:111-114` e `start.sh:21-24` — hash invalidador faz `find ros2_packages/robot_nav -name '*.py' -o -name '*.yaml' -o ...`. Mudanças em `wheel_msgs/msg/WheelSpeeds.msg`, `costmap_converter`, `teb_local_planner` **não disparam rebuild**, apesar do `colcon build` recompilar `wheel_msgs` por causa do `--packages-select`.

**Fix:** estender o find:
```bash
HASH=$(find "$SCRIPT_DIR/ros2_packages/robot_nav" "$SCRIPT_DIR/ros2_packages/wheel_msgs" \
    \( -name '*.py' -o -name '*.yaml' -o -name '*.xacro' -o -name '*.msg' \) \
    -print0 | sort -z | xargs -0 sha1sum 2>/dev/null | sha1sum | awk '{print $1}')
```

### A18 — `setup_udev.sh`: pode gerar regra udev inválida
- `setup_udev.sh:72-73, 106-107` — se `get_devpath()` retornar string vazia (regex falha em formato USB novo), regra fica `KERNELS==""`. Casa com nada (ou com tudo, dependendo do kernel).

**Fix:** após capturar `MEGA_PATH` e `LIDAR_PATH`:
```bash
[ -z "$MEGA_PATH"  ] && { echo "ERRO: não extraiu USB path da MEGA";  exit 1; }
[ -z "$LIDAR_PATH" ] && { echo "ERRO: não extraiu USB path do LiDAR"; exit 1; }
```

### A19 — `install_nav2.sh` divergente do README
- `install_nav2.sh:9-17` — falta `ros-jazzy-nav2-bringup`, `ros-jazzy-nav2-map-server`, `ros-jazzy-nav2-amcl`, `ros-jazzy-slam-toolbox`.

**Decisão D2 sugerida:** ou (a) completar a lista para bater com `setup.sh`, ou (b) **deletar** o `install_nav2.sh` inteiro (`setup.sh` já cobre). Recomendado (b) — é dead code com risco de drift.

Se for (b), remover também menção em `README.md:792` e na tabela de arquivos `README.md:791`.

---

## 🟡 MÉDIOS

### M1 — `nav_metrics`: race no goal-tracking
- `controle_web/nav_metrics.py:178-179` — `status_list[-1]` pega o último, mas em waypoints sequenciais pode pegar status do anterior.
**Fix:** iterar `status_list` e filtrar por `_nav_goal_id` corrente.

### M2 — `package.xml`: faltam exec_depends
- `ros2_packages/robot_nav/package.xml:24-26` — faltam `slam_toolbox`, `nav2_map_server`, `nav2_amcl`, `nav2_controller`, `nav2_planner`, `nav2_behaviors`, `nav2_bt_navigator`, `nav2_waypoint_follower`, `nav2_velocity_smoother`, `ros_gz_sim`, `ros_gz_bridge`. `rosdep install` não baixa tudo.
**Fix:** adicionar `<exec_depend>` para cada um. Depois da remoção do collision_monitor (D1), confirmar que o linha 24 não tem mais `nav2_collision_monitor`.

### M3 — `lidar.launch.py` default errado
- `ros2_packages/robot_nav/launch/lidar.launch.py:23` — `lidar_port='/dev/ttyUSB1'` default. README e `setup_udev.sh` criam `/dev/lidar`.
**Fix:** trocar default para `'/dev/lidar'`.

### M4 — `sim.launch.py`: TFs duplicados
- `ros2_packages/robot_nav/launch/sim.launch.py:91` — `PosePublisher` do `husky.sdf:232` publica TFs estáticos que conflitam com `robot_state_publisher`.
**Fix:** desabilitar `<static_publisher>true</static_publisher>` no SDF ou tirar bridge `/tf` no launch (manter só `/tf_static`).

### M5 — DWB `transform_tolerance` < AMCL `transform_tolerance`
- `ros2_packages/robot_nav/config/nav2_params.yaml:113` — DWB tem 0.2 enquanto AMCL tem 1.0. Erros transientes de TF quando AMCL atualiza `map→odom` lentamente.
**Fix:** alinhar em 0.5 nos dois lados.

### M6 — `cmd_vel_to_wheels`: saturação não-proporcional
- `ros2_packages/robot_nav/robot_nav/cmd_vel_to_wheels.py:60-61` — quando satura, perde proporção linear/angular.
**Fix:** se `max(|left|, |right|) > max_output`, dividir ambos pelo mesmo fator.

### M7 — `cone_detector`: loop Python custoso
- `ros2_packages/robot_nav/robot_nav/cone_detector.py:101-120` — for loop sobre 360+ pontos com cos/sin.
**Fix:** vetorizar com `numpy.cos/sin`. ~10× mais rápido na Pi.

### M8 — `trekking_runner`: `_find_matching_cone` mistura referenciais
- `ros2_packages/robot_nav/robot_nav/trekking_runner.py:402-415` — compara bearing relativo ao yaw **atual** com bearing relativo ao yaw **gravado** sem conversão.
**Fix:** salvar bearing absoluto no `_save_point` ou converter no compare: `expected_world = wp['cone_bearing'] + wp['yaw']`, `cur_world = cur_bearing + self.yaw`, comparar `_wrap_pi(cur_world - expected_world)`.

### M9 — `map_service`: conversão OccupancyGrid → PNG perde nuances
- `controle_web/map_service.py:47-61` — valores `1..49` ficam indistinguíveis de "desconhecido" (cinza 205).
**Fix:** mapear linearmente com prioridade:
```python
img = np.full_like(arr, 205, dtype=np.uint8)  # default unknown
img[arr >= 0] = (255 - (arr.clip(0, 100) * 255 // 100)).astype(np.uint8)
```

### M10 — `requirements.txt`: eventlet morto + sem pins
- `controle_web/requirements.txt:3` — `eventlet==0.36.1` pinned mas `app.py:86` usa `async_mode="threading"`. Dependência inútil.
- `:4-6` — `pyyaml`, `numpy`, `Pillow` sem pin.
**Fix:** remover eventlet; pinar com `>=x,<y`.

### M11 — `app.py`: paths de log relativos a CWD
- `controle_web/app.py:132,136,142` — `os.makedirs('logs', ...)` relativo. Rodar de outro diretório gera logs em lugar errado.
**Fix:** prefixar com `os.path.dirname(os.path.abspath(__file__))`.

### M12 — `app.py`: input validation
- `controle_web/app.py:204-205,222-223,445-454` — `x`, `y`, `waypoints`, `kwargs` de trekking aceitos sem validar tipo/range/NaN.
**Fix:** validar `math.isfinite(x) and abs(x) < 1000`, `isinstance(waypoints, list)`, filtrar `kwargs` permitidos.

### M13 — `_safe_name` inconsistente
- `controle_web/map_service.py:301` — `save_route` tem `or 'rota'`
- `controle_web/map_service.py:315` — `load_route` não tem
- `controle_web/trekking_service.py:113` — também aceita string vazia
**Fix:** padronizar — rejeitar nome vazio com `ValueError` ou aplicar `or 'rota'` em todos.

### M14 — Sleeps fixos no launch.sh
- `launch.sh:298,313,320,349,368` — `sleep 6` (Gazebo), `sleep 5` (Nav2) etc. Frágil em Pi 4 com SD.
**Fix:** trocar por health-check com timeout:
```bash
for i in {1..30}; do
    ros2 topic list 2>/dev/null | grep -q "/map" && break
    sleep 1
done
```

### M15 — Falta `--help` em quase todos os scripts
- `launch.sh`, `setup.sh`, `start.sh`, `setup_udev.sh`, `install_nav2.sh` — sem `--help`. Só `setup_pi.sh` tem.
**Fix:** adicionar caso `--help|-h)` que imprime o cabeçalho.

### M16 — Duplicação `setup.sh` vs `setup_pi.sh`
**Fix:** extrair `_setup_common.sh` com lógica compartilhada (apt list comum, bashrc append, source ROS), e os dois scripts apenas configuram variáveis e fazem `source _setup_common.sh`.

### M17 — Duplicação `launch.sh` vs `start.sh`
Blocos de build incremental + venv quase idênticos.
**Fix:** extrair `_bootstrap.sh` para os dois.

### M18 — `trekking_service`: shutdown com race
- `controle_web/trekking_service.py:72-81` — não chama `_executor.shutdown()`; thread daemon spina node já destruído por alguns ms.
**Fix:** replicar ordem do `MapBridge`: `_running=False` → `_executor.shutdown()` → `destroy_node()`.

### M19 — `setup_udev.sh`: detecção de LiDAR frágil
- `setup_udev.sh:88-93` — "qualquer porta diferente da MEGA". Se houver 3 portas (modem 3G/4G), pega a errada.
**Fix:** perguntar interativamente OU filtrar por VID:PID conhecido do FHL-LD20.

### M20 — `setup_udev.sh`: `MODE="0666"` permissivo demais
- `setup_udev.sh:122-136`
**Fix:** `MODE="0660", GROUP="dialout"`.

### M21 — Validação de flags no launch.sh
- `launch.sh:39-55` — `case` sem ramo `*)`. Typos como `--slamm` passam silenciosamente.
**Fix:**
```bash
*) echo "Flag desconhecida: $arg"; exit 1 ;;
```

### M22 — `$DRIVER_LOG` undefined
- `launch.sh:434` — mensagem `tail -f $DRIVER_LOG` sai vazia.
**Fix:** trocar por `$LOG_DIR/robot_nodes.log`.

### M23 — Buffer da Serial PC do firmware
- `firmware/mega_bridge/src/main.cpp:99-104` — a 230400 baud + FastLED.show + I²C BNO055, buffer hw (64 B) pode encher.
**Fix:** instrumentar com contador de bytes perdidos OU adicionar mais um `pumpPcSerial()` no meio do loop.

### M24 — Frames do protocolo: structs `__attribute__((packed))`
- `firmware/mega_bridge/src/main.cpp:148-200` — montagem byte-a-byte via `memcpy`. Funciona mas é verboso.
**Fix:** definir structs packed em `protocol.h` e fazer `memcpy(buf, &frame, sizeof(frame))` uma vez. Mesmo trabalho do lado Python via `struct.pack/unpack` com formato consistente.

### M25 — Clamp Q14 errado por fator 2
- `firmware/mega_bridge/src/main.cpp:156-160` — `f_to_q14` faz clamp em `±2.0`; quaternion unitário tem |q|≤1. Benigno (não estoura int16) mas perde precisão.
**Fix:** clamp em `±1.0`.

### M26 — `imu_ok` global redundante
- `firmware/mega_bridge/src/main.cpp:47,253` — duplica `imu_dev.ok()`. Se `ok_` mudar runtime (após implementar A13), o anel não acompanha.
**Fix:** usar `imu_dev.ok()` direto na linha 253.

### M27 — `mega_bridge.py`: RX thread loga/publica fora do executor
- `ros2_packages/robot_nav/robot_nav/mega_bridge.py:222-243`
**Fix:** usar `Queue` interna + timer ROS para drenar OU `ReentrantCallbackGroup`.

### M28 — `odom_publisher`: callbacks sem lock (futuro-proof)
- `ros2_packages/robot_nav/robot_nav/odom_publisher.py:66-73`
**Fix:** `threading.Lock` ao redor de `self.v_fl/fr/rl/rr` se algum dia migrar para `MultiThreadedExecutor`.

### M29 — `controle_web/README.md` obsoleto
- `controle_web/README.md` inteiro — não menciona `ROBOT_MODE`, modos, MapBridge, NavMetrics, gamepad. Lê como se servidor só fizesse echo de teclas.
**Fix:** deletar o arquivo (README raiz cobre tudo) ou reduzir a um link para o raiz.

### M30 — `sim_robot.sdf` divergente do real
- `ros2_packages/robot_nav/urdf/sim_robot.sdf:166` — `<wheel_separation>0.45</wheel_separation>`; URDF real usa 0.50.
**Fix:** alinhar para 0.50. Se este SDF ainda não está no `sim.launch.py`, marcar como WIP no comentário do arquivo.

### M31 — `husky.urdf.xacro:44`: typo
- `ros2_packages/robot_nav/urdf/husky.urdf.xacro:44` — expressão `wheel_radius - wheel_radius` zera (copy-paste).
**Fix:** corrigir para `${-body_height/2}` direto.

### M32 — Câmera RGB ainda referenciada no front-end
- `controle_web/templates/index.html:144-149` — painel "Câmera RGB"
- `controle_web/static/js/client.js:163-175` — handler `camera_frame`
- `controle_web/static/css/styles.css:82-109` — bloco de CSS

README declara que câmera foi removida.
**Fix:** remover os três.

### M33 — `controle_web/static/js/map.js:316`: yaw=0 hardcoded
- `map.js:316` — `socket.emit('nav_goal', { x, y, yaw: 0.0 })` sempre.
**Fix:** implementar click+drag igual aos waypoints (já existe lógica `dragged`), aproveitar.

---

## 🟢 BAIXOS

### B1 — `_quat_to_yaw` duplicado em 3 arquivos
- `ros2_packages/robot_nav/robot_nav/pose_estimator.py:38-41`
- `ros2_packages/robot_nav/robot_nav/cone_detector.py:29-32`
- `ros2_packages/robot_nav/robot_nav/trekking_runner.py:53-56`
**Fix:** criar `ros2_packages/robot_nav/robot_nav/utils.py` com a função e importar.

### B2 — `setup.py:17`: glob vazio
- `ros2_packages/robot_nav/setup.py:17` — `glob('maps/*')` em diretório vazio.
**Fix:** remover essa linha do `data_files`.

### B3 — `package.xml:18`: `tf2` sem uso
- `ros2_packages/robot_nav/package.xml:18` — código só usa `tf2_ros`.
**Fix:** remover `<depend>tf2</depend>`.

### B4 — `.gitignore` faltando entradas
- `firmware/mega_bridge/.pio/` aparece em `git status` (não está ignorado).
- Faltam: `*.swp`, `.vscode/`, `.idea/`, `.DS_Store`.
**Fix:** adicionar:
```
firmware/**/.pio/
*.swp
.DS_Store
.vscode/
.idea/
```

### B5 — Magic numbers que mereciam ser `constexpr` / parâmetros
- `firmware/mega_bridge/src/leds.cpp` — `16` (tick), `800` (boot), `1300` (beat)
- `ros2_packages/robot_nav/robot_nav/cmd_vel_to_wheels.py` — `Klin=400`, `Kang=150`, `max_output=1000`
- `controle_web/map_service.py` — `WP_SETTLE_TIME=0.5`, `TIMEOUT=120`, `MAX_RETRIES=2` (na verdade já existem como const, só verificar)
- `controle_web/nav_metrics.py:260` — threshold de "parado" `0.01`

### B6 — `trekking_service.py:98-100`: `get_last_state` nunca usado
**Fix:** deletar.

### B7 — `client.js:32-33`: `BASE_LINEAR=100`, `BASE_ANGULAR=65` sem unidade
- README diz "Base: 0.3 m/s linear, 0.5 rad/s angular".
**Fix:** documentar a unidade no comentário ou converter para SI.

### B8 — `gamepad.js:251`: magic `100 * 4`
**Fix:** constante nomeada `MAX_LINEAR_SCALED = BASE_LINEAR_SPEED * SPEED_MULT_MAX`.

### B9 — `app.py:148-163`: log inunda com polling do Socket.IO
**Fix:** skip de `/socket.io/` no logger:
```python
if request.path.startswith('/socket.io/'): return
```

### B10 — `app.py:284-346, 349-415`: handlers duplicados
**Fix:** extrair `_emit_movement(entry)` para reduzir 30 linhas duplicadas.

### B11 — `app.py:432`: log de exceção no `set_speed` sem broadcast correto
- `controle_web/app.py:432` — `emit('speed_update', ..., broadcast=True)` em exceção; cliente original perde feedback.
**Fix:** emitir para o `request.sid` do cliente erroneamente.

### B12 — `controle_web/static/css/styles.css:82-109`: dead CSS
**Fix:** remover bloco "Painel de obstáculos (LiDAR)".

### B13 — `controle_web/templates/index.html:200`: socket.io-client 3.1.3
- `flask-socketio==5.4.1` é compat com 4.x. Atualizar para `4.7.5`.

### B14 — `firmware/mega_bridge/platformio.ini`: faltam warnings
- `platformio.ini:19` — adicionar `-Wall -Wextra -Werror=return-type` em `build_flags`.

### B15 — `cmd_vel_to_wheels.py`/`mega_bridge.py`: QoS sem declaração explícita
- Vários publishers/subscribers usam `10` (depth) sem `QoSProfile`. ROS2 default é `RELIABLE+VOLATILE`. Declarar explícito reduz surpresas com smoother do Nav2 vs teleop.

### B16 — `nav_metrics.py:209-217`: `_recovery_ids` nunca limpo
- Mesmo `goal_id` aparecer em duas tentativas (improvável, UUIDs) ignora na segunda.
**Fix:** reset ao abrir nova `NavAttempt`.

### B17 — `trekking_runner.py:255-276`: salvar bearing absoluto resolve M8
**Já coberto em M8 acima.**

### B18 — `map.js:97-103`: alloca Image() a cada `map_update`
**Fix:** reusar uma única `Image` global, só setar `src`.

### B19 — `map.js:528`: render a 15 Hz mesmo sem mudanças
**Fix:** disparar render só ao receber `map_update`/`robot_pose`/`plan_update`.

### B20 — `trekking.js:251-262`: trail nunca resetado
**Fix:** limpar quando recebe `state.mode === 'idle'` && `state.total === 0`.

### B21 — `client.js:227-229`: log DOM limitado a 50
**Fix:** subir para 200 (custa pouco).

---

## Checklist sugerido (ordem de execução)

Quando for aplicar amanhã, sugiro esta sequência:

### Etapa 1 — Remoção do collision_monitor (D1)
1. Deletar os 2 arquivos listados em D1
2. Editar `launch.sh:378-394` (remover `elif` do collision_monitor)
3. Editar `cmd_vel_to_wheels.py:3` e `robot.launch.py:9` (docstrings/comentários)
4. Editar `nav2_params.yaml:5` (comentário)
5. Editar `setup.sh`, `setup_pi.sh`, `install_nav2.sh` (apt lists)
6. Editar `package.xml:24` (exec_depend)
7. Editar `README.md` em todos os pontos listados em D1
8. Verificar com `grep -rn -i "collision_monitor\|nav2_collision\|cmd_vel_filtered" .`
9. `colcon build` para garantir que nada quebrou

### Etapa 2 — Críticos restantes
1. C1 — robot_radius / footprint nos 4 arquivos YAML
2. C2 — inversão L/R (mover do `cmd_vel_to_wheels` para params)
3. C3 — covariâncias na `/odom`
4. C5 — quality do PMW3901 (ou marcar como TODO no README se for trabalhoso)
5. C6 — feedback stale das placas
6. C4 — `flow_link` no URDF (precisa medida física)
7. C7 — auth/CORS/SECRET_KEY no app.py
8. C8 — Kang derivado de wheel_base
9. C9 — alinhar gamepad README↔código

### Etapa 3 — Altos (A1–A19)
A ordem entre eles é flexível; A16 (deletar `test_serial.py`) e A19 (deletar `install_nav2.sh`) são one-liners.

### Etapa 4 — Médios/baixos
Conforme tempo. Os mais "give-back" rápido: M3, M22, M31, M32, B1, B2, B3, B4, B6, B7.

---

## Notas adicionais para a próxima sessão

- O `git status` no início da auditoria mostrava só `?? firmware/mega_bridge/.pio/` (untracked, do build PlatformIO). Branch `main`, último commit `743848d "ros2_ws deleted, change all to Controle_web"`.
- Memórias do projeto relevantes (em `~/.claude/projects/-home-rbe-luis-Workspace-Controle-robo-web/memory/`):
  - `project_hardware_4wheel.md` — dimensões 50×50×26 cm, roda r=0.085, bitola 0.50, entre-eixos 0.37, massa 15.46 kg
  - `project_mega_pinout.md` — Serial1/2 hoverboards, I²C BNO055, SPI PMW3901, pinos 6/7/8/9 LED/relé/LED/botão
- Após mexer no firmware, lembrar de re-flashear via `pio run -t upload` em `firmware/mega_bridge/`.
- Após mexer em `wheel_msgs/` ou em qualquer `.msg`, fazer `colcon build --packages-select wheel_msgs` (e ver A17 para fix do hash de cache).
