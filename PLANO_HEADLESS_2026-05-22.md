# Plano — PC do robô headless + controle PS4/WASD + web só visualização

> Data: 2026-05-22 · Branch: `main`
> Objetivo: tornar o PC do robô **100% headless** (operável de outro PC, "a tela
> sem cabo"), tirar o **controle de movimento do navegador** (que tem latência
> ruim via WiFi/celular) e passá-lo para **PS4 (Bluetooth, local)** ou **WASD
> (teclado via SSH)**. O web continua sendo a interface de **visualização e
> planejamento** (mapa SLAM ao vivo, pose, click-to-go do Nav2, waypoints,
> salvar mapa, infos), mas **não dirige mais**.

---

## 1. Arquitetura nova

```
MANUAL:   PS4 (Bluetooth, local) ─► joy_node ─► teleop_twist_joy ─► joy_vel ┐
          WASD (SSH) ─► teleop_twist_keyboard ─────────────────────► key_vel ┤
AUTÔNOMO: Nav2 / trekking ──────────────────────────────────────────► nav_vel ┤
                                                                              │
                                              twist_mux (joy>key>nav) ◄───────┘
                                                     │
                                                  /cmd_vel ─► cmd_vel_to_wheels ─► MEGA ─► rodas

WEB (outro PC): mapa SLAM ao vivo · pose · click-to-go · /plan · waypoints · salvar mapa · infos
                (SEM dirigir)
ACESSO:  ssh robot.local + tmux + VS Code Remote-SSH + RViz (opcional)
```

### Prioridades do twist_mux
| Fonte | Tópico | Prioridade | Por quê |
|-------|--------|-----------|---------|
| PS4   | `joy_vel` | **100** | controle humano sempre vence — pode assumir/abortar a navegação |
| WASD  | `key_vel` | **90**  | fallback de teclado, abaixo do PS4 |
| Nav2/trekking | `nav_vel` | **10** | autonomia só dirige quando ninguém está no manual |

Com isso, encostar no analógico do PS4 durante um Nav2 **interrompe** a navegação
(o mux passa a publicar o `joy_vel`); soltar o controle e esperar o timeout volta
o `nav_vel`. Resolve também o achado **B20** da AUDITORIA_2026-05-18 (vários
publishers competindo direto no `/cmd_vel`).

### Estado atual confirmado (pontos que o plano toca)
- `app.py:25` — `ROS2Controller()` é criado **sempre**, e republica `/cmd_vel` a
  50 Hz (`robot_controller.py:200`). É ele que brigaria com o PS4 hoje.
- `app.py:18` — já existe `ROBOT_MODE` (teleop/slam/nav2/trekking) gating
  componentes; o `launch.sh:513` exporta essa env var.
- `app.py:147` — `map_bridge` (MapBridge) só sobe em slam/nav2. Ele **inicializa
  o próprio `rclpy`** (`map_service.py:93-94`: `if not rclpy.ok(): rclpy.init()`),
  ou seja **não depende** do `ROS2Controller`. Mesmo assim mantemos o controller
  vivo (ele é criado incondicionalmente em `app.py:25` e o caminho mais simples é
  só **desligar a publicação de teleop dele**, não removê-lo).
- `app.py:358/426/495` — handlers `key_event`/`gamepad_event`/`set_speed`
  registrados **sempre**. São a superfície de controle do navegador a remover.
- `nav2.launch.py:93` — `velocity_smoother` faz `('cmd_vel_smoothed','cmd_vel')`;
  o Nav2 interno já usa `cmd_vel_nav` (`nav2.launch.py:66,77`).
- `trekking_runner.py:152` — publica `Twist` em `cmd_vel` (tópico relativo).
- `cmd_vel_to_wheels.py:40` — assina `cmd_vel` (parametrizável via
  `cmd_vel_topic`). É o consumidor final, **não muda**.
- `robot.launch.py` — sobe sempre: `robot_state_publisher`, `mega_bridge`,
  `odom_publisher`, `cmd_vel_to_wheels`. É onde o `twist_mux` + `joy` entram.
- Rede: **não** há `ROS_DOMAIN_ID` setado (default 0). Flask já sobe em
  `0.0.0.0:5000` (`app.py:584`) → web já acessível na LAN.

---

## 2. Fase 1 — Controle nativo ROS + arbitragem

### 2.1 `ros2_packages/robot_nav/config/twist_mux.yaml` (novo)
```yaml
twist_mux:
  ros__parameters:
    use_sim_time: false
    topics:
      joystick:
        topic: joy_vel
        timeout: 0.5
        priority: 100
      keyboard:
        topic: key_vel
        timeout: 0.5
        priority: 90
      navigation:
        topic: nav_vel
        timeout: 0.5
        priority: 10
    # Sem locks por enquanto (sem botão de e-stop dedicado no protocolo ainda).
```
> `timeout`: se a fonte parar de publicar por >0.5 s, o mux a ignora e passa pra
> próxima prioridade. **Atenção (corrige o texto da §1):** com
> `require_enable_button: true` (§2.2), o `teleop_twist_joy` **NÃO** publica
> continuamente — ele só publica `joy_vel` **enquanto o L1 está segurado**.
> Soltou o L1 → para de publicar → timeout 0.5 s → o mux cai pro `nav_vel`. Ou
> seja, o PS4 só "segura" o mux durante o input com dead-man; esse é exatamente o
> comportamento que deixa o Nav2 dirigir quando ninguém está no controle e
> permite o PS4 assumir/abortar por cima.

### 2.2 `ros2_packages/robot_nav/config/teleop_ps4.yaml` (novo)
Mapa do DualShock 4 para `teleop_twist_joy`. Eixos do DS4 no Linux (`joy`):
- eixo 0 = analógico esq. X (giro), eixo 1 = analógico esq. Y (frente/ré)
- L1 = botão 4 (dead-man), R1 = botão 5 (turbo)

```yaml
teleop_twist_joy_node:
  ros__parameters:
    axis_linear:
      x: 1
    scale_linear:
      x: 0.30          # m/s — alinhado com BASE_LINEAR_SPEED do projeto
    scale_linear_turbo:
      x: 0.50
    axis_angular:
      yaw: 0
    scale_angular:
      yaw: 6.0         # rad/s — bate com BASE_ANGULAR_SPEED do projeto
    scale_angular_turbo:
      yaw: 6.0         # mantém o giro no turbo (não cai pro default 1.0)
    enable_button: 4        # L1 = dead-man (precisa segurar pra andar)
    enable_turbo_button: 5  # R1 = turbo
    require_enable_button: true
    publish_stamped_twist: false
```
> **Angular = 6.0 rad/s (não 1.2)**: o chassi de 4 rodas não apoia uniformemente,
> as rodas patinam e **comando angular baixo não gira o robô** — ver o comentário
> de `robot_controller.py:128-135` (`BASE_ANGULAR_SPEED = 6.0`). Com 1.2 rad/s o
> PS4 provavelmente não conseguiria girar. O `scale_angular_turbo` é setado junto
> porque, sem ele, o turbo cairia pro default 1.0 rad/s do `teleop_twist_joy`.
> **Dead-man (L1)**: só anda enquanto o L1 está pressionado. Segurança básica —
> se largar o controle, o robô para. (Configurável: trocar o botão se preferir.)

### 2.3 `ros2_packages/robot_nav/launch/robot.launch.py` (editar)
> ⚠️ **Nova dependência apt — `colcon build` NÃO resolve.** `joy`,
> `teleop_twist_joy` e `twist_mux` passam a subir **sempre** aqui. Como sobem em
> todos os modos, se os pacotes não estiverem instalados a `robot.launch.py`
> **falha pra todo mundo** (não só quando o PS4 é usado). Os 3 (+
> `teleop_twist_keyboard` da §2.6) **não estão** no `package.xml` hoje, então:
> 1. adicionar ao `package.xml` os `exec_depend` (pra `rosdep` pegar):
>    ```xml
>    <exec_depend>joy</exec_depend>
>    <exec_depend>teleop_twist_joy</exec_depend>
>    <exec_depend>teleop_twist_keyboard</exec_depend>
>    <exec_depend>twist_mux</exec_depend>
>    ```
> 2. **re-rodar `setup_pi.sh` (§4.1) antes de testar a Fase 1** em qualquer
>    máquina (robô atual e máquina de dev) — senão o stack inteiro não sobe.

Adicionar, subindo **sempre** (todos os modos têm controle manual):
```python
# joy_node — lê o DualShock 4 em /dev/input/js0
Node(package='joy', executable='joy_node', name='joy_node',
     parameters=[{'device_id': 0, 'deadzone': 0.05, 'autorepeat_rate': 20.0}])

# teleop_twist_joy — joy → joy_vel (config teleop_ps4.yaml)
Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
     parameters=[<config/teleop_ps4.yaml>],
     remappings=[('cmd_vel', 'joy_vel')])

# twist_mux — arbitra joy_vel/key_vel/nav_vel → cmd_vel
Node(package='twist_mux', executable='twist_mux', name='twist_mux',
     parameters=[<config/twist_mux.yaml>],
     remappings=[('cmd_vel_out', 'cmd_vel')])
```
> `cmd_vel_to_wheels` continua assinando `cmd_vel` → nada muda nele.

### 2.4 `ros2_packages/robot_nav/launch/nav2.launch.py` (editar)
Linha 93 — o smoother passa a publicar em `nav_vel` (entrada do mux), não direto
no `cmd_vel`:
```python
remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'nav_vel')]
```
> ⚠️ **O modo SIM fica SEM DIREÇÃO NENHUMA (não só o nav).** No sim o
> `launch.sh:377` sobe `sim.launch.py` — que **não** tem `twist_mux`/`joy` (eles só
> entram no `robot.launch.py`, §2.3) e faz a ponte direta `/cmd_vel`→Gazebo
> (`sim.launch.py:88`). O **único** publisher de `/cmd_vel` no sim sempre foi o
> `ROS2Controller` do web. Juntando as duas fases:
> - este remap (§2.4) → o smoother do Nav2 publica em `nav_vel`, ninguém arbitra
>   pro `/cmd_vel` → **Nav2 no sim fica parado**;
> - `WEB_TELEOP=off` por padrão (§3.1/§3.3) → o web não publica → **teleop no sim
>   também fica parado**.
>
> Ou seja, `./launch.sh --sim`, `--sim --slam` (o jeito de mapear/testar SLAM
> **sem hardware**) e `--sim --nav2` ficam **todos sem direção**. Isso quebra o
> principal fluxo de teste sem robô. Opções (decidir, mas não bloqueia o robô
> real):
> 1. **Subir `twist_mux` (e opcionalmente `joy`) também no `sim.launch.py`** e
>    tornar o remap do smoother condicional (`nav_vel` no hardware/sim-com-mux,
>    `cmd_vel` direto se não houver mux) — solução completa; **ou**
> 2. no curto prazo, subir o sim sempre com **`--web-teleop`** (Fase 2) pra
>    reativar a direção pelo web — registrar isso no README/§4.5.

### 2.5 `ros2_packages/robot_nav/launch/trekking.launch.py` (editar)
Remapear a saída do `trekking_runner` pra entrada do mux:
```python
remappings=[('cmd_vel', 'nav_vel')]
```
> Assim, no modo trekking, o PS4 também pode assumir por cima do autônomo.

### 2.6 WASD (`robot-key`, ver Fase 3)
`teleop_twist_keyboard` rodado num terminal SSH/tmux, remapeado pra `key_vel`.
Não precisa de arquivo de launch — é um comando (encapsulado no helper `robot-key`).

---

## 3. Fase 2 — Web em "modo monitor" (tira o movimento)

### 3.1 `controle_web/app.py` (editar)
- Nova env var `WEB_TELEOP` (default **`off`**):
  ```python
  WEB_TELEOP = os.environ.get('WEB_TELEOP', 'off').lower() == 'on'
  ```
- Handlers `key_event` (358), `gamepad_event` (426), `set_speed` (495): quando
  `WEB_TELEOP` for off, retornam erro curto ("controle desabilitado — use PS4/WASD")
  e **não** chamam o controller.
- `ROS2Controller`: quando `WEB_TELEOP` off, **não** inicia a thread do
  republicador e **não publica nada** em `/cmd_vel` (mantém `rclpy.init` + nó
  vivos só por simplicidade; o `map_bridge` não precisa dele — ver §1).
  → Implementar com um parâmetro no construtor ou `enable_publish(False)`, e o
  gating tem que cobrir **`_publish` inteiro**, não só a thread. Caso contrário o
  `force_stop()` chamado no `disconnect` (`app.py:353` → `robot_controller.py:207`,
  `self._publish(0,0)`) volta a publicar direto em `/cmd_vel` — que agora é a
  **saída do `twist_mux`** — reintroduzindo o publisher concorrente do achado B20.
  Com `WEB_TELEOP` off, `force_stop`/`_publish` devem ser no-ops.
- `mode_info` (emitido no `connect`, `app.py:234` — **não** é um payload do evento
  `connect`): incluir `'web_teleop': WEB_TELEOP` nesse dict pro front saber.

### 3.2 Frontend (`controle_web/static/js/client.js` + template)
- Quando `web_teleop` for false (recebido no evento `mode_info`):
  - **esconde** o painel de direção (botões/joystick virtual/slider de velocidade);
  - **para de capturar** `keydown`/`keyup` (WASD) e o gamepad do browser;
  - mostra um aviso discreto: "Controle manual: PS4/WASD no robô".
- **Não tocar** no canvas do mapa, click-to-go, lista de waypoints, salvar mapa,
  painel de infos — tudo continua.

### 3.3 `launch.sh` (editar)
- Exporta `WEB_TELEOP=off` por padrão (ao lado de `ROBOT_MODE`, ~linha 513).
- Flag opcional `--web-teleop` que seta `WEB_TELEOP=on` (reativa o controle web
  caso um dia precise — ex.: testar sem o PS4).

### 3.4 O que continua funcionando no web
| Recurso | Caminho ROS | Status |
|---------|-------------|--------|
| Mapa SLAM ao vivo | sub `/map` → `map_update` | mantém |
| Pose do robô | TF `map→base_link` → `robot_pose` | mantém |
| Click-to-go | `nav_goal` → `/goal_pose` | mantém |
| Rota do Nav2 | sub `/plan` → `plan_update` | mantém |
| Waypoints/rotas | start/stop/save/load | mantém |
| Salvar mapa | `save_map` (map_saver_cli) | mantém |
| **Dirigir (WASD/gamepad/slider)** | `key_event`/`gamepad_event`/`set_speed` | **REMOVIDO** |

---

## 4. Fase 3 — Headless OS + atalhos

### 4.1 `setup_pi.sh` (editar)
Adicionar instalação/habilitação:
```bash
sudo apt install -y \
  ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-twist-mux \
  bluez tmux avahi-daemon openssh-server
sudo systemctl enable --now ssh avahi-daemon
```
> `avahi-daemon` → o robô vira acessível como **`robot.local`** (mDNS), sem caçar IP.
> Os 4 pacotes ROS aqui (`joy`, `teleop-twist-joy`, `teleop-twist-keyboard`,
> `twist-mux`) são os mesmos `exec_depend` adicionados ao `package.xml` na §2.3 —
> **esta instalação tem que rodar antes de testar a Fase 1** (a `robot.launch.py`
> nova não sobe sem eles). Em quem usa `rosdep`, `rosdep install --from-paths …`
> também resolve depois do package.xml atualizado.

### 4.2 `pair-ps4.sh` (novo)
Fluxo guiado de pareamento por `bluetoothctl` (sem tela):
```bash
# Coloque o DS4 em pareamento: segure SHARE + PS até a barra piscar rápido.
bluetoothctl <<'EOF'
power on
agent on
default-agent
scan on
EOF
# (script mostra os MACs encontrados, você confirma o do controle, e ele faz:)
#   pair  <MAC>
#   trust <MAC>   <- ESSENCIAL: reconecta sozinho no boot
#   connect <MAC>
# Confere com: ls /dev/input/js0
```
> O `trust` é o pulo do gato: depois disso, ligar o controle reconecta automático.

### 4.3 `robot-up` (novo, instalado no PATH — ex. `/usr/local/bin/robot-up`)
Sobe a stack dentro do tmux (sobrevive à queda do SSH):
```bash
#!/usr/bin/env bash
# Uso: robot-up [slam|nav2|trekking|teleop] [args extras do launch.sh]
SESSION=robo
MODE="${1:-slam}"; shift || true
cd ~/Workspace/Controle_robo_web
if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux attach -t "$SESSION"          # já rodando → só reanexa
else
  tmux new -s "$SESSION" "./launch.sh --$MODE $*; bash"
fi
```
Fluxo do dia a dia:
```
(liga o robô — PS4 reconecta sozinho)
ssh robot.local
robot-up slam            # ou nav2 --map=maps/sala.yaml
# Ctrl+B D pra destacar; robot-up de novo pra reanexar
```

### 4.4 `robot-key` (novo, no PATH)
WASD via teclado, publicando na entrada do mux:
```bash
#!/usr/bin/env bash
cd ~/Workspace/Controle_robo_web
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=key_vel
```

### 4.5 README (editar)
Nova seção **"Operação headless"**:
- `ssh robot.local` + `robot-up <modo>` + tmux (detach/attach).
- **VS Code Remote-SSH** pra editar configs (`nav2_params.yaml`, launches) com GUI.
- **RViz no outro PC**: mesma sub-rede + mesmo `ROS_DOMAIN_ID`; nota de que
  alguns APs WiFi bloqueiam **multicast** (se o RViz não achar tópicos, é isso →
  AP que passe multicast ou Fast-DDS discovery server).
- Pareamento do PS4 (`pair-ps4.sh`) e WASD (`robot-key`).

---

## 5. Validação

| O que | Onde |
|-------|------|
| Sintaxe Python (`py_compile`) | aqui |
| Deps apt instaladas (`joy`, `teleop_twist_joy`, `teleop_twist_keyboard`, `twist_mux`) — **pré-requisito** do dry-run abaixo | aqui (e re-rodar `setup_pi.sh` no robô) |
| `colcon build` p/ instalar launch+config novos em `install/` (não há código compilado alterado) | aqui |
| YAML do twist_mux / teleop_ps4 carregam + `robot.launch.py` sobe os nós novos sem erro | aqui (lint + dry-run do launch — só passa com as deps acima instaladas) |
| Lógica de gating do web (handlers off) | aqui (revisão + teste local sem hardware) |
| Pareamento real do PS4 + reconexão no boot | **só no robô (você)** |
| Dirigir de fato + latência PS4 vs web | **só no robô (você)** |
| RViz/web no outro PC (multicast LAN) | **só na rede real (você)** |

---

## 6. Ordem de execução (3 commits isolados)

1. **Fase 1** — controle nativo + twist_mux (`twist_mux.yaml`, `teleop_ps4.yaml`,
   `robot.launch.py`, `nav2.launch.py`, `trekking.launch.py`, **`package.xml`**
   com os 4 `exec_depend` novos). Commit.
2. **Fase 2** — web modo monitor (`app.py`, `client.js`+template, `launch.sh`). Commit.
3. **Fase 3** — headless setup + atalhos (`setup_pi.sh`, `pair-ps4.sh`, `robot-up`,
   `robot-key`, README). Commit.

Cada commit isolado pra testar incremental no robô.

---

## 7. Pontos de decisão em aberto (ajustáveis)
- **Dead-man L1**: pode trocar pelo botão que preferir, ou desabilitar
  (`require_enable_button: false`) se achar chato segurar.
- **Escalas** (0.30 m/s linear, 6.0 rad/s angular — mesmos do `robot_controller.py`):
  bater com o robô real depois de medir.
- **twist_mux locks**: dá pra adicionar um botão de e-stop (lock de prioridade
  máxima) numa segunda passada, se quiser parada de emergência pelo controle.
