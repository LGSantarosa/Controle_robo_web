# Auditoria do projeto Controle_robo_web — 2026-05-18

Segunda passada de auditoria depois da limpeza da [AUDITORIA_2026-05-14.md](./AUDITORIA_2026-05-14.md). Cobre firmware MEGA + nodes ROS2 + Flask/web + JS + shell + configs YAML cruzados com o `README.md`.

> Severidades: 🔴 crítica (bloqueia uso real ou pode quebrar dados/segurança) — 🟠 alta (bug funcional, comportamento incorreto) — 🟡 média (qualidade/robustez/UX) — 🟢 baixa (cosmético / refatoração).

> **Estado da auditoria anterior:** ~95% aplicada (ver final do doc anterior). Os achados abaixo são **novos** ou **não foram pegos antes**, exceto onde marcado *(reaparece)*.

---

## 🔴 CRÍTICOS

### C1 — URDF: rodas enterradas 4.5 cm no chão

Geometria do `robot.urdf.xacro` é inconsistente entre `base_footprint → base_link` e a posição das rodas:

- `robot.urdf.xacro:40` — `base_footprint → base_link` em `xyz="0 0 ${wheel_radius}"` (= 0.085 m)
- `robot.urdf.xacro:74` — wheel macro: rodas em `xyz="${x} ${y} ${-body_height/2}"` (= −0.13 m relativo a base_link)
- Resultado: centro da roda em **z = 0.085 − 0.13 = −0.045 m** (abaixo do chão)

Para rodas tocarem o chão (centro em z = 0 mundo), a roda precisa ficar a `-wheel_radius` relativa a base_link.

**Fix (uma das duas):**
- (a) trocar `${-body_height/2}` por `${-wheel_radius}` no macro de roda (rodas no chão, corpo "flutua" baixo).
- (b) subir o `base_footprint_joint` para `xyz="0 0 ${wheel_radius + body_height/2}"` e manter `-body_height/2` na roda (mais coerente com convenção REP-103: `base_link` no centro de massa do corpo).

A (b) é mais limpa, mas exige conferir se algum tuning de Nav2/SLAM/teleop assume `base_link` a 0.085 m. Recomendado checar antes de mudar.

### C2 — `trekking_runner`: callbacks sem lock

`_on_pose` escreve `self.x/y/yaw` (linha 156-162) e `_on_cones` substitui a lista `self.cones` (164-168) sem `threading.Lock`. `_control_tick` (30 Hz) lê esses valores em outra timer callback.

- `ros2_packages/robot_nav/robot_nav/trekking_runner.py:155-168`
- `ros2_packages/robot_nav/robot_nav/trekking_runner.py:276,400` (iteração de `self.cones`)

Hoje funciona porque o `SingleThreadedExecutor` serializa callbacks, mas é frágil — `MultiThreadedExecutor` ou um `MutuallyExclusive`/`Reentrant CallbackGroup` mudaria isso silenciosamente.

**Fix:** adicionar `self._state_lock = threading.Lock()` no construtor e envolver leituras/escritas. Iterar `self.cones` num snapshot (`cones = list(self.cones)`).

### C3 — `mega_bridge.py`: RX thread publica direto fora do executor (reaparece — era M27)

A thread `_rx_loop` chama `self._pub_*.publish(...)` direto (linha 248-261 — `_handle_state/_handle_imu/_handle_flow`). rclpy é tolerante a publishers fora de callback, mas isso quebra com `MultiThreadedExecutor` e dificulta tracing de logs.

**Fix sugerido na auditoria anterior:** `Queue` interna + timer ROS para drenar, OU `ReentrantCallbackGroup`. **Não foi aplicado.**

### C4 — Trekking cmds: app e runner não falam a mesma língua

`controle_web/app.py:528-534` aceita o set `{start, stop, reset, record, play, save_point, remove_last, load_waypoints}` e bloqueia o resto. Mas o `trekking_runner._on_cmd` (linhas 184-210) aceita `{reset, record, save_point, play, stop, load_waypoints, clear}`.

| Cmd            | app.py | runner | resultado |
|----------------|--------|--------|-----------|
| `start`        | ✅    | ❌    | passa pelo socket, runner loga "cmd desconhecido" e ignora |
| `remove_last`  | ✅    | ❌    | idem |
| `clear`        | ❌    | ✅    | app rejeita com "cmd desconhecido" — função fica inalcançável |

**Fix:** decidir qual é a verdade. Sugestão: alinhar o set `_TREKKING_CMDS` no app com o que o runner aceita; remover `start`/`remove_last` (não implementados) e adicionar `clear`. Se `start`/`remove_last` deveriam existir, implementar no runner.

---

## 🟠 ALTOS — bugs reais

### A1 — `mega_bridge.py:226` — LED de marco nunca controlável via ROS

`_on_light` recebe `Bool` em `/light/cmd` e manda `bytes([1 if msg.data else 0, 0])` para o frame `FT_RELAY`. O firmware (`main.cpp:86-91`) lê `p[0]` para o relé E `p[1]` para o LED de marco — mas o bridge sempre manda `0` no segundo byte. **Não existe tópico ROS pra controlar o LED de marco** que o firmware suporta.

**Fix:** criar `/light/marker` (Bool) ou aceitar dois bools num único `ColorRGBA`/`UInt8MultiArray`. Atualizar README para refletir capacidade real.

### A2 — `mega_bridge.py:263-268` — `_handle_state` ignora `faultF`/`faultR` apesar do comentário

O docstring na linha 264 promete "rpm_*×4, batF, batR, faultF, faultR, btn, _pad". O código lê apenas `p[:12]` (RPMs + baterias) e `p[14]` (btn). Os bytes `p[12]/p[13]` (faults) **nunca são processados**. Firmware também não preenche (sempre zero — `main.cpp:153-154`), o que torna o comentário enganoso por ambos os lados.

**Fix:** implementar no firmware (campos de status: `front_stale`, `rear_stale`, `imu_ok`, `flow_ok`) e publicar como `/system/health` (`DiagnosticArray` ou JSON em String). Atualizar bridge.

### A3 — `pose_estimator`: silêncio quando flow morre runtime

`pose_estimator.py:228-230` — quando `flow_age > flow_timeout`, força `alpha=0` e segue. Nenhum log/warning. Combinado com C5 da auditoria anterior (PMW3901 envia `quality=0` sempre, `alpha≈0`), o nó nunca diz "estou rodando degradado".

**Fix:** log `warn` throttled (uma vez por minuto) quando alpha < 0.05 por >2 s, e quando `flow_age > flow_timeout`. Sinalizar via `/trekking/health` (JSON) pra UI mostrar.

### A4 — `map_service.py`: `load_route` mexe estado sem lock

`map_service.py:357-360` muda `self._wp_list`, `_wp_loop`, `_wp_active`, `_wp_current_idx` direto, sem `with self._wp_lock`. As outras mutações (em `start_waypoints`, `stop_waypoints`, `_wp_runner`) usam o lock. Race observável: cliente carrega rota enquanto outro inicia/para → estado inconsistente.

- `controle_web/map_service.py:350-366`
- `controle_web/map_service.py:528` — `self._wp_active = False` no final do runner também sem lock (inconsistente com `stop_waypoints:308-310`)

**Fix:** envolver os dois pontos com `with self._wp_lock:`.

### A5 — `trekking_runner._sanitize_wp` crasheia callback com waypoint malformado

```python
'x':     float(w.get('x', 0.0)),
```
Se `w['x']` for `None`, `list`, dict, ou string não-numérica vinda do UI, `float()` levanta `TypeError`/`ValueError` dentro do callback `_on_cmd`. rclpy não recupera — pode derrubar o spin ou deixar o nó num estado inconsistente.

- `ros2_packages/robot_nav/robot_nav/trekking_runner.py:202,233-242`

**Fix:** try/except em volta da list comprehension (linha 202) e logar warning + retornar sem alterar `self.waypoints`.

### A6 — `firmware/main.cpp:144` — botão sem debounce

`io_signals::readButton()` é leitura crua do `digitalRead(PIN_BTN)`. Sem debounce. O firmware envia `btn` a 50 Hz, e o `trekking_runner._on_button` (linha 170-175) detecta rising edge **no lado Python**. Mas se o botão bouncing produzir falsos rising edges em 50 Hz, vai gravar múltiplos waypoints num único press.

**Fix (firmware ou Python):**
- Firmware: filtro debounce simples (estado estável por >30 ms).
- Python: contador "pressed por >2 frames consecutivos" antes de declarar rising.

### A7 — `sensors_flow.cpp`: sem recovery automático

`Imu::read()` (`sensors_imu.cpp:23-32`) tem retry a cada 2 s se `ok_` cair. `Flow::read()` (`sensors_flow.cpp:10-17`) **só retorna early sem tentar re-init**:

```cpp
void Flow::read() {
    if (!ok_) return;       // morto pra sempre até reboot
    pmw_.readMotionCount(&dx_, &dy_);
    quality_ = 0;
}
```

Se o cabo SPI soltar ou o sensor pifar transitoriamente no boot, fica desabilitado até alguém resetar a MEGA.

**Fix:** replicar padrão do `Imu` — `last_recover_ms_` + `begin()` a cada 2 s se `!ok_`.

### A8 — `nav2_params.yaml:178` — `min_obstacle_height: -0.05` é sobra da câmera removida

```yaml
voxel_layer:
  scan:
    min_obstacle_height: -0.05
    max_obstacle_height: 2.0
```

`-0.05` faz sentido com PointCloud2 da câmera (filtrar chão); com **só LiDAR 2D** todos os pontos vêm na altura do laser (~0.21 m), então o threshold é inócuo — só polui o YAML.

- `ros2_packages/robot_nav/config/nav2_params.yaml:178`
- Mesmo arquivo `nav2_params_pi.yaml` — verificar (provavelmente tem só /scan).

**Fix:** voltar para `min_obstacle_height: 0.0` e atualizar comentário pra refletir LiDAR-only.

### A9 — Comentários obsoletos sobre `~/ros2_ws`

Auditoria anterior (D1) mudou o workspace pra `ros2_packages/` direto, mas dois comentários no Python ainda dizem o caminho velho:

- `controle_web/app.py:24` — `# Pré-requisito: source ~/ros2_ws/install/setup.bash`
- `controle_web/controllers/robot_controller.py:119-120` — `Execute antes... source ~/ros2_ws/install/setup.bash`

**Fix:** atualizar para `source install/setup.bash` (sem o `~/ros2_ws`).

### A10 — `app.py`: emit no exception path sem `room=request.sid`

`app.py:418-424` (handler `key_event`) faz `emit('ack', ..., broadcast=False)` no exception. `broadcast=False` ≠ `room=request.sid` — sob contexto async (raro mas possível com Werkzeug), pode emitir pra room errada. `handle_set_speed:515` faz certo com `room=request.sid`. Inconsistente.

**Fix:** trocar `broadcast=False` por `room=request.sid` em todos os emits dentro de handlers (key_event:413, key_event:418, gamepad_event:481, gamepad_event:490).

---

## 🟡 MÉDIOS — qualidade e robustez

### M1 — `odom_publisher.py:122-124` — integração Euler simples

```python
self.x += linear * math.cos(self.theta) * dt
self.y += linear * math.sin(self.theta) * dt
self.theta += angular * dt
```

Em curvas rápidas, acumula erro. Padrão melhor: usar yaw na média do passo (`self.theta + 0.5*angular*dt`). Desprezível a 20 Hz mas vale a nota.

### M2 — `sensors_imu.cpp:33-40` — quaternion não normalizado

Sanity check é "quaternion todo zerado = sensor morto". Mas BNO055 pode reportar quaternion não-unitário em transitórios (cal=0). O Python depende de unitariedade implicitamente.

**Fix:** normalizar no firmware:
```cpp
double n = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
if (n > 1e-6) { qw/=n; qx/=n; qy/=n; qz/=n; }
```

### M3 — Frames inválidos descartados sem instrumentação

Tanto firmware (`protocol.cpp:41`) quanto Python (`mega_bridge.py:99-101`) descartam silenciosamente frames com `len > MAX_PAYLOAD`. Também `main.cpp:53-94` ignora frames com `len` errado (`return` direto). Sem contador → difícil debug.

**Fix:** contador `dropped_frames` (uint16) acumulado, exposto no `STATE` ou via log throttled.

### M4 — `main.cpp`: bytes 12,13,15 do STATE reservados sem documentação

```cpp
buf[12] = 0;
buf[13] = 0;
buf[14] = btn;
buf[15] = 0;
```

Pelo comentário em `mega_bridge.py:264` deveriam ser `faultF`, `faultR`, e há um `_pad`. Mas nem firmware preenche nem Python lê. Confunde quem for estender.

**Fix:** ou (a) implementar (ver A2), ou (b) reduzir `STATE` para 15 bytes (drop dos faults), ou (c) `// reservado: faultF/faultR (futuro)` no firmware.

### M5 — `txFlow` não checa `flow_dev.read()` (assimétrico com `txImu`)

`txImu` (`main.cpp:179-181`) faz `if (!imu_dev.read()) return;`. `txFlow` (`main.cpp:217`) chama `flow_dev.read()` como void — se o SPI silenciosamente falhar, publica os mesmos `dx_/dy_` do frame anterior.

**Fix:** tornar `Flow::read()` `bool` (retorna false em erro) e symmetric.

### M6 — Bateria 0V em frame stale é ambíguo

`main.cpp:142-143` — se placa stale, `batF = 0` (0V). Lado Python (`mega_bridge.py:282`) reporta `b.present = raw > 0`, então 0 V → present=False. Funciona, mas perde a distinção "placa de bateria comunicando mas medindo 0V" (curto-circuito real) vs "placa não responde".

**Fix:** usar `INT16_MIN` ou bit dedicado no `faultF`/`faultR` (ver A2/M4).

### M7 — `nav2_params.yaml:71` — `min_y_velocity_threshold: 0.5` muito alto

Default Nav2 é `0.001`. Como diff-drive tem `vy=0` sempre, threshold é inócuo, mas o valor estranho confunde leitor.

**Fix:** voltar para `0.001` ou remover (deixa default).

### M8 — `nav2_params.yaml:84,116` — `xy_goal_tolerance` em dois lugares com valores muito diferentes

- `goal_checker.xy_goal_tolerance: 0.40` — usado pelo plugin de check de chegada
- `FollowPath.xy_goal_tolerance: 0.07` — usado pelo DWB internamente

Funciona (são parâmetros distintos), mas a diferença de 6× confunde. Documentar.

### M9 — `velocity_smoother` vs `DWB`: limites de ré divergentes

- `nav2_params.yaml:97` — DWB `min_vel_x: -0.1`
- `nav2_params.yaml:275` — smoother `min_velocity[0]: -0.15`

Smoother permite mais ré do que DWB sabe planejar. Inócuo na prática, mas combina mal — ou alinha em -0.1 ou -0.15.

### M10 — `sim_robot.sdf:196` — `static_publisher=true` (vs `husky.sdf=false`)

Se algum dia `sim_robot.sdf` for usado no lugar de `husky.sdf` em `sim.launch.py`, o `PosePublisher` do GZ duplica os TFs estáticos que o `robot_state_publisher` já publica.

**Fix:** alinhar para `false` (com comentário explicando).

### M11 — `app.py:430,363` — log INFO de gamepad/key inunda terminal

`gamepad_event` a ~50 Hz e cada keydown/keyup logam linha INFO. Em uma partida ativa, são milhares de linhas/minuto.

**Fix:** baixar para `DEBUG`. Manter INFO apenas para eventos relevantes (button press, emergency).

### M12 — `app.py:509-510` — acessa atributos privados do controller

```python
'linear_speed': controller._linear_speed,
'angular_speed': controller._angular_speed,
```

Quebra encapsulation. `_linear_speed`/`_angular_speed` são properties — usar nome público (`controller.linear_speed`) e renomear a property.

### M13 — `map_service.py`: `_pose_loop` faz polling 10 Hz em vez de callback TF

`tf2_ros.TransformListener` aceita callback. Polling com `sleep(0.1)` funciona mas é menos eficiente — cada lookup falha (`TransformException`) antes do AMCL convergir.

**Fix opcional:** usar `Buffer.set_transforms_changed_callback` ou um listener custom.

### M14 — `husky.sdf:159-167` — camera spawna mas bridge não exporta

A SDF cria o sensor `rgbd_camera` (linhas 184-202) que publica `/camera/*` no Gazebo. Mas o `parameter_bridge` em `sim.launch.py:83-94` não inclui topics de câmera. Resultado: o sensor roda (consome CPU/GPU) mas dados não chegam no ROS.

**Fix:** ou (a) remover o sensor da SDF (auditoria anterior diz que câmera foi removida do robô), ou (b) adicionar bridge das `/camera/*` se quiser usar.

### M15 — `trekking_runner._on_cmd('stop')` não reseta `current_idx`

`stop` muda mode para IDLE e publica zero. Próximo `play` reseta `current_idx=0` (`_start_play:297`), então o efeito real é OK. Mas há comportamento intermediário inconsistente — `state['current_idx']` retorna o último valor.

**Fix:** ou (a) zerar no stop, ou (b) documentar que `current_idx` no estado IDLE é "último idx do play anterior".

### M16 — `_led_tick` publica 1 Hz mesmo sem mudança

`trekking_runner.py:455-464` — timer 1 Hz publica `ColorRGBA` sempre. Pequeno tráfego ROS desnecessário; firmware vai re-renderizar a cada frame.

**Fix:** comparar com último publicado e só publicar se mudou (ou publicar a cada 5 s como heartbeat se quiser garantir).

### M17 — `cmd_vel_to_wheels.py:64` — sem validação de NaN/Inf

Se `cmd_vel.angular.z = NaN` (improvável mas possível em bug upstream), propaga até as rodas via `int()` que pode dar exception ou comportamento indefinido.

**Fix:** `import math; if not (math.isfinite(linear) and math.isfinite(angular)): return`

### M18 — `husky.urdf.xacro:107-126` — `camera_link` mantida apesar da câmera "removida"

URDF do `husky` ainda define `camera_link` (joint + link), pra dar TF estática. Mas se a câmera não existe (auditoria 2026-05-14 diz que foi removida da UI/back-end), URDF deveria refletir.

**Fix:** remover link/joint, OU manter e atualizar README mencionando que continua disponível pra futuro upgrade.

### M19 — `trekking_runner.py:114` — `button_prev=False` no construtor

Se o botão estiver pressionado quando o nó sobe, primeiro callback dispara rising edge espúrio (em RECORD, grava waypoint sem o usuário pedir).

**Fix:** inicializar como `None`, e tratar primeiro callback como "calibração":
```python
if self.button_prev is None:
    self.button_prev = msg.data
    return
```

### M20 — `nav2_params.yaml:69` — `controller_frequency: 10.0`

Default Nav2 é 20 Hz. 10 Hz pode dar atraso de reação em obstáculos dinâmicos. Comentário não explica.

**Fix:** documentar (ex: "limitado a 10 Hz pra caber na Pi sem perder tempo") ou subir para 20.

### M21 — `setup_pi.sh` divergente de `setup.sh` (M16 anterior — não refatorado)

Lógica de apt comum, bashrc append, source ROS poderia ser `_setup_common.sh` compartilhado. Auditoria anterior já cobriu, ainda WIP.

---

## 🟢 BAIXOS

### B1 — `start.sh` ainda duplica colcon build + venv de `launch.sh`

M17 da auditoria anterior. Sem refatoração.

### B2 — `launch.sh:79-81` — re-parse de args só pra `--no-pi` é confuso

```bash
for arg in "$@"; do
    [ "$arg" = "--no-pi" ] && PI_PROFILE=false
done
```

Logo após o auto-detect arm64 que pode ter setado `PI_PROFILE=true`. Refatorar pra usar uma flag tristate em vez de revisar.

### B3 — `mega_bridge.py:138` — `wheel_scale` sem validação

`float(self.get_parameter('wheel_scale').value)` aceita qualquer valor (negativo, zero, NaN). Robô passar `-1.0` inverte tudo silenciosamente.

**Fix:** `assert wheel_scale > 0 and math.isfinite(wheel_scale)` no init.

### B4 — `protocol.cpp:6-9` — checksum XOR é fraco contra erros adjacentes

`ABCABC` tem mesmo XOR que `CBABCA`. Em USB a 230400 baud com headers fixos é suficiente, mas se um dia subir para 1 Mbaud, considerar CRC8.

### B5 — `client.js:33-37` — magic numbers `BASE_LINEAR=100, BASE_ANGULAR=65`

Comentário explica que são "unidades internas" pra UI mostrar antes do ack do servidor. Mas o "100" e "65" são arbitrários — bate de onde? Mover para constantes no início do arquivo OU acompanhar das constantes reais (BASE_LINEAR_SPEED=0.3) com fator de escala explícito.

### B6 — `cone_detector.py:70-74` — `_have_pose=False` sem timeout

Se `pose_estimator` morrer, `cone_detector` para de publicar mas não loga. Adicionar warning throttled.

### B7 — `setup.sh:62` — `export PATH="$HOME/.local/bin:$PATH"` só vale no script

`pipx ensurepath` já adiciona no `.bashrc`, mas o usuário tem que reabrir terminal. Cosmético — README/setup já avisa.

### B8 — `trekking.js:217-225` — `cmd()` não trata ack

Se servidor responder `{ok: false}`, o `statusEl.textContent = 'ERRO: ...'` é setado pelo handler genérico `trekking_ack` (linha 270-272). Funciona, mas o `cmd()` mesmo não sabe o resultado da própria chamada.

### B9 — `nav2_params.yaml:26-31` — `set_initial_pose: true` + initial_pose (0,0,0,0)

AMCL sempre acha que começa em (0,0). Se o robô spawnou em outro lugar, AMCL leva tempo pra convergir. Pequena UX issue — usuário pode setar com a UI/CLI antes de mandar goal.

### B10 — `app.py:202-221` — log de cada HTTP poderia ser DEBUG

Skip de `/socket.io/` ajuda, mas página principal + assets ainda logam INFO. Em produção (mesmo LAN), DEBUG é mais limpo.

### B11 — `nav2_params.yaml:71` — `min_y_velocity_threshold: 0.5` cosmético (ver M7)

### B12 — `app.py:533-534` — `_TREKKING_KWARGS` tem `index` que nunca é usado

Whitelist permite `index`, mas nenhum cmd do runner consome. Limpar.

### B13 — `mega_bridge.py:306-308` — covariâncias IMU hardcoded sem fonte

```python
msg.orientation_covariance = [0.01, 0, 0, ..., 0.01]
```

Valores razoáveis pro BNO055 calibrado, mas o comentário não diz "datasheet página X" ou "medido empiricamente Y". Sem contexto futuro-você não saberá ajustar.

### B14 — `husky.sdf:215` vs realidade

`<odom_publish_frequency>30</odom_publish_frequency>` no sim, mas o real é 50 Hz. Inconsistência intencional? Documentar.

### B15 — `controle_web/static/js/map.js:209-214` — interpolação direta no innerHTML

`'<option value="${r}">${r}</option>'` confia no `_safe_name` do backend (só alfanumérico + `-_`). Frágil — qualquer mudança no `_safe_name` pode introduzir XSS. Trocar por `createElement('option')` + `textContent`.

### B16 — `firmware/main.cpp:170-173` — `f_to_milli` clamp em `±32.0`

Magic number. Quaternion já tem helper, milli não. Mover para `constexpr float MAX_GYRO_RAD_S = 32.0f;` (ou aproveitar do BNO055 range).

### B17 — `firmware/main.cpp:38-40` — globais não-POD inicializados em ordem de declaração

`Imu imu_dev; Flow flow_dev{PMW_CS}; Ring ring;` — construtores rodam antes de `Wire/SPI.begin()` no `setup()`. Funciona porque libs Adafruit/Bitcraze só salvam pinos no construtor, mas é assumption frágil.

### B18 — `nav_metrics.py:118-127` — subscriptions de recovery com lambda

```python
self._node.create_subscription(
    GoalStatusArray, f'/{name}/_action/status',
    lambda msg, n=name: self._on_recovery_status(msg, n),
    ...
)
```

`n=name` no default arg captura corretamente. ✅ Mas poderia ser `functools.partial(self._on_recovery_status, name=name)` — mais idiomático.

### B19 — `trekking_runner.py:436` — `state['ts'] = time.time()` em vez de ROS clock

OK pra UI mostrar "atualizado há X s", mas em modo sim com `use_sim_time=true` daria valores inconsistentes em logs.

### B20 — README:747 menciona `velocity_smoother (nav2)` publicando direto em `/cmd_vel`

Tecnicamente correto pelo remap `cmd_vel_smoothed → cmd_vel` em `nav2.launch.py:93`. Mas a frase confunde quem lê: "tanto teleop quanto velocity_smoother publicam em /cmd_vel — última mensagem vence". O fluxo real é mais sutil (velocity_smoother é cadeia consumer). Limitação documentada na seção "Limitações conhecidas:965" — OK, deixa.

---

## Checklist sugerido (ordem de execução)

### Etapa 1 — Críticos que afetam comportamento
1. **C1** — Geometria URDF (rodas enterradas). Decidir entre (a) ou (b) e medir robô real antes de mudar.
2. **C4** — Alinhar `_TREKKING_CMDS` no app com o `_on_cmd` do runner.
3. **C2** — Adicionar lock no `trekking_runner`.
4. **C3** — Queue na RX thread do `mega_bridge.py` (replicar padrão do executor).

### Etapa 2 — Bugs funcionais (A1–A10)
- **A1** — Tópico `/light/marker` no bridge.
- **A2/M4/M6** — Implementar `faultF/faultR` ou removê-los do protocolo.
- **A3** — Logs de saúde no `pose_estimator`.
- **A4** — Lock em `load_route` e `_wp_active=False` final.
- **A5** — try/except em `_sanitize_wp`.
- **A6** — Debounce do botão (firmware ou Python).
- **A7** — Recovery do `Flow::read()`.
- **A8** — `min_obstacle_height: 0.0`.
- **A9** — Atualizar comentários `~/ros2_ws`.
- **A10** — `room=request.sid` nos emits de erro.

### Etapa 3 — Médios/cosméticos
Conforme tempo. Os "give-back rápido": M11 (log DEBUG), M12 (rename properties), M17 (NaN guard), M19 (button_prev=None), M14 (camera bridge ou remover).

### Etapa 4 — Refatorações
- B1/M21 — extrair `_setup_common.sh` / `_bootstrap.sh`.
- B2 — combinar parse de `--no-pi`.
- M21 — auditoria anterior já listou.

---

## Notas adicionais

- `git status` no início desta auditoria: branch `main`, último commit `68463c8 "Merge branch 'main'..."` + 1 commit `2026-05-14` (auditoria anterior, FIXME no `sensors_flow.cpp` e edição de logs).
- A SDF `husky.sdf` ainda tem a câmera RGB-D (linha 159-203). O bridge `sim.launch.py` **não** exporta as topics — efetivamente câmera é dead weight no sim. Ver M14/M18 (decidir manter ou remover de vez).
- `_TREKKING_KWARGS` tem `'index'` mas nenhum runner cmd consome — ver B12.
- O comentário em `nav2_params.yaml:158-159` diz "VoxelLayer mantido... a câmera RGB-D foi removida do robô" — confirma a remoção mas o `husky.urdf.xacro:107-126` e `husky.sdf:168-203` ainda têm o link/sensor. Discrepância em três arquivos.
