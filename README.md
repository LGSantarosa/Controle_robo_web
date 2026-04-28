# Controle Web do Robô Hoverboard

Interface web para controlar um robô hoverboard com ROS2, LiDAR FHL-LD20, detecção de obstáculos, **mapeamento SLAM** e **navegação autônoma Nav2 com click-to-go** no mapa web.

## Sumário

- [Guia rápido — do zero ao click-to-go](#guia-rápido--do-zero-ao-click-to-go)
- [Visão geral](#visão-geral)
- [Os três modos de operação](#os-três-modos-de-operação)
- [Modo SIM — testar tudo no Gazebo sem hardware](#modo-sim--testar-tudo-no-gazebo-sem-hardware)
- [Pré-requisitos](#pré-requisitos)
- [Configuração inicial (uma vez)](#configuração-inicial-uma-vez)
  - [1. Workspace ROS2](#1-workspace-ros2)
  - [2. Portas USB fixas](#2-portas-usb-fixas-obrigatório)
  - [3. Dependências Python](#3-dependências-python)
- [Como rodar](#como-rodar)
  - [Modo TELEOP (padrão)](#modo-teleop-padrão)
  - [Modo SLAM — mapear a sala](#modo-slam--mapear-a-sala)
  - [Modo NAV2 — navegação autônoma](#modo-nav2--navegação-autônoma)
- [Controles](#controles)
- [Câmera RGB-D](#câmera-rgb-d)
- [Navegação por waypoints](#navegação-por-waypoints)
- [Métricas Nav2 (CSV)](#métricas-nav2-csv)
- [Arquitetura](#arquitetura)
  - [Ponte ROS2 ↔ Web para mapa e navegação](#ponte-ros2--web-para-mapa-e-navegação)
- [Tuning do Nav2](#tuning-do-nav2)
- [Logs](#logs)
- [Limitações conhecidas](#limitações-conhecidas)
- [Solução de problemas](#solução-de-problemas)

---

## Guia rápido — do zero ao click-to-go

Passo a passo condensado para quem está pegando uma máquina nova e quer ver o robô andando sozinho no Gazebo, clicando num ponto do mapa. Todas as seções abaixo têm mais detalhes, isto aqui é o caminho feliz.

### 1. Instalar o ROS2 Jazzy

Siga o guia oficial (~10 min): https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Confirme:

```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
ros2 --help   # deve listar os comandos
```

### 2. Clonar este repositório

```bash
git clone <url-do-repo> ~/Controle_robo_web
```

### 3. Rodar o setup automatizado

O script `setup.sh` na raiz do repo faz tudo de uma vez: instala dependências apt, monta o `~/ros2_ws`, faz symlink do `robot_nav`, clona o `wheel_msgs` e roda `colcon build`.

```bash
cd ~/Controle_robo_web
./setup.sh
```

O script é idempotente — se já tiver rodado antes, pode rodar de novo sem quebrar nada. Ele cobre:

- **apt install**: `xacro`, `robot-state-publisher`, `slam-toolbox`, `nav2-bringup`, `nav2-collision-monitor`, `nav2-map-server`, `nav2-amcl`, `ros-gz`, `ros-gz-sim`, `ros-gz-bridge`, `ros-gz-interfaces` (tudo para Jazzy), além de `git`, `python3-venv`, `python3-pip`.
- **Workspace**: cria `~/ros2_ws/src`, faz o symlink do `robot_nav` deste repo, clona `wheel_msgs` ([Richard-Haes-Ellis/wheel_msgs](https://github.com/Richard-Haes-Ellis/wheel_msgs)), compila com `colcon build` e adiciona o `source` ao `~/.bashrc`.

Se for usar hardware real, descomente no `setup.sh` as duas linhas de clone de `ros2-hoverboard-driver` e `ldlidar_stl_ros2` antes de rodar, ou clone/compile manualmente depois.

| Pacote | Obrigatório? | Para quê |
|--------|--------------|----------|
| `wheel_msgs` | **sempre** (até no sim, senão `colcon build` falha) | Tipo de mensagem `WheelSpeeds` |
| `ros2-hoverboard-driver` | só no modo real | Driver C++ do hoverboard |
| `ldlidar_stl_ros2` | só no modo real | Driver do LiDAR FHL-LD20 |

### 4. (Só hardware real) Fixar portas USB

Se for rodar no robô físico, o hoverboard e o LiDAR precisam de symlinks estáveis em `/dev/hoverboard` e `/dev/lidar`:

```bash
sudo ~/Controle_robo_web/setup_udev.sh
```

Depois recompile o driver:
```bash
cd ~/ros2_ws && colcon build --packages-select ros2-hoverboard-driver
```

Pule este passo inteiro se só vai usar `--sim`.

### 5. Primeira execução — teste rápido no sim

Não precisa configurar mais nada. O `launch.sh` cria o `venv` Python e instala Flask/Socket.IO/Pillow automaticamente na primeira vez.

```bash
cd ~/Controle_robo_web
./launch.sh --sim
```

O que deve acontecer:
1. Uma janela do Gazebo Harmonic abre mostrando uma sala 6×6 m vazia (do `worlds/empty.sdf`) com o robô simulado no centro.
2. No terminal aparece `Iniciando servidor web em http://0.0.0.0:5000 (modo: teleop [SIM/Gazebo])`.
3. Abra `http://localhost:5000` no navegador — você vê a UI com o badge `TELEOP`.
4. Clique na área da página e use `WASD` ou setas: o robô se move no Gazebo.

`Ctrl+C` no terminal fecha tudo (Gazebo, bridges, web).

### 6. Mapear a sala simulada (SLAM)

```bash
./launch.sh --sim --slam
```

Na UI o badge vira `SLAM` e um painel **Mapa** aparece. Dirija o robô **devagar** pela sala com WASD/setas (ver [dicas de mapeamento](#modo-slam--mapear-a-sala)). O mapa cresce em tempo real no painel web.

Quando o mapa estiver bom, clique em **Salvar mapa** → nome padrão `sala` → gera `maps/sala.yaml` + `maps/sala.pgm`. Depois `Ctrl+C`.

### 7. Navegação autônoma (NAV2 click-to-go)

```bash
./launch.sh --sim --nav2
```

O badge vira `NAV2`, o painel **Mapa** mostra o mapa estático que você salvou, o robô aparece como seta laranja. **Clique em qualquer ponto livre do mapa** — o Nav2 calcula a rota (linha azul), o bt_navigator dispara o controlador e o robô do Gazebo vai até lá.

### 8. (Opcional) Use a sala que você projetou

Coloque o arquivo `.sdf` da sua sala em `Controle_robo_web/worlds/` e passe por flag:

```bash
./launch.sh --sim --slam  --world=worlds/minha_sala.sdf
./launch.sh --sim --nav2  --world=worlds/minha_sala.sdf
```

Veja [Onde colocar o arquivo da sala](#onde-colocar-o-arquivo-da-sala-mundo-gazebo) para o checklist do que o `.sdf` precisa ter (physics, luz, ground_plane, collisions).

### 9. Migrar para o hardware real

Quando o fluxo estiver funcionando no sim, basta tirar o `--sim` dos comandos. A mesma UI, o mesmo `/goal_pose`, o mesmo mapa (se for a mesma sala) — e agora com `launch.sh --slam` / `launch.sh --nav2` o robô físico responde. O único diferencial é que você precisa ter rodado o passo **4** antes.

---

## Visão geral

```
Navegador (WASD / Gamepad / Clique / Waypoints)
        │  Socket.IO
        ▼
  Flask + Socket.IO (porta 5000)
        │  /cmd_vel  (geometry_msgs/Twist)
        │  Action navigate_to_pose  (waypoints e click-to-go em NAV2)
        ▼
  cmd_vel_to_wheels
        │  /wheel_vel_setpoints  (wheel_msgs/WheelSpeeds)
        ▼
  ros2-hoverboard-driver  ──────►  /dev/hoverboard (USB serial)

  Sensores:
    LiDAR FHL-LD20  ─────────────►  /scan        (LaserScan, 360°)
    Câmera RGB-D    ─────────────►  /camera/image, /camera/depth_image
                                    /camera/points (PointCloud2 — VoxelLayer)

  ┌─────────────────────────┬─────────────────────────────────────────┐
  │  TELEOP                 │  SLAM                   │  NAV2          │
  │  obstacle_detector      │  slam_toolbox           │  map_server +  │
  │  → /tmp/obstacle_*.json │  → /map (ao vivo)       │  amcl + planner│
  │  + nav2_collision_mon.  │  → TF map→odom          │  + controller +│
  │                         │                         │  bt_navigator +│
  │                         │                         │  behaviors +   │
  │                         │                         │  velocity_smth │
  │                         │                         │  + waypoint_fl │
  │                         │                         │  costmaps com  │
  │                         │                         │  VoxelLayer    │
  │                         │                         │  (LiDAR+camera)│
  └─────────────────────────┴─────────────────────────────────────────┘
        │
        ▼  Pontes ROS2 → Socket.IO (no app Flask)
  map_service.py:    /map → PNG, TF map→base_link, /plan,
                     NavigateToPose action client (click + waypoints)
  camera_bridge.py:  /camera/image → JPEG @ 5 Hz
  nav_metrics.py:    grava CSV por navegação (status, replans, recoveries)
        │
        ▼
  Navegador
    Canvas do mapa: mapa + robô + plano + waypoints + último alvo
    Painel câmera:  stream do que o robô está vendo
    Toolbar wp:     adicionar/limpar/iniciar/parar/loop, salvar/carregar rotas
```

---

## Os três modos de operação

O `launch.sh` tem um conceito central: **o modo**. Cada modo sobe uma combinação diferente de nós ROS2 para um propósito distinto:

| Modo | Flag | Pra quê serve | O que sobe a mais |
|------|------|---------------|-------------------|
| **TELEOP** | *(padrão)* | Dirigir manualmente pela sala | `nav2_collision_monitor` — só segurança (freia se tiver obstáculo perto) |
| **SLAM** | `--slam` | Construir o mapa da sala pela primeira vez | `slam_toolbox` em modo *mapping online* (gera `/map` ao vivo) |
| **NAV2** | `--nav2` | Navegação autônoma + click-to-go + waypoints + métricas | `map_server` + `amcl` (com beam_skip) + `planner_server` + `controller_server` (DWB) + `bt_navigator` + `behavior_server` + `velocity_smoother` + `waypoint_follower` + `NavMetricsCollector` (CSV) |

Nos três modos o web control, o hoverboard e o LiDAR rodam normalmente — você sempre pode dirigir manualmente, mesmo durante SLAM ou NAV2.

### Espera, por que aparece "nav2" em dois lugares? (collision_monitor vs Nav2 completo)

Dá pra confundir: no modo TELEOP o log mostra `nav2_collision.log` e no modo `--nav2` aparece `nav2.log`. **Não são dois jeitos de rodar o Nav2** — são dois pedaços distintos do mesmo projeto upstream Nav2:

- **`nav2_collision_monitor`** (modo TELEOP) — um único nó pequeno de segurança. Só intercepta `/cmd_vel`, olha o LiDAR, e freia o robô se detectar obstáculo muito perto. **Não** planeja rota, **não** precisa de mapa, **não** sabe onde o robô está no mundo. É uma camada de proteção pra dirigir manualmente.
- **Stack Nav2 completa** (modo `--nav2`) — uma dúzia de nós que fazem navegação autônoma de verdade: carregam um mapa salvo (`map_server`), localizam o robô nele por correlação de scans (`amcl`), planejam rota até um destino (`planner_server`), executam a trajetória desviando de obstáculos dinâmicos (`controller_server`), orquestram tudo com uma árvore de comportamento (`bt_navigator`).

Por vir do mesmo projeto Nav2, os dois compartilham o prefixo `nav2_` no nome dos pacotes — mas têm papéis completamente diferentes.

---

## Modo SIM — testar tudo no Gazebo sem hardware

Antes de arriscar o hoverboard na sala real, você pode rodar o pipeline inteiro (teleop + SLAM + Nav2 click-to-go) dentro do **Gazebo Harmonic**, com um robô diferencial simulado em um mundo customizado por você.

A flag `--sim` troca tudo que é hardware por simulação:

| Stage | Modo real | Modo `--sim` |
|-------|-----------|--------------|
| Driver do hoverboard | `ros2-hoverboard-driver` | — (não usa) |
| Odometria | `odom_publisher` (feedback das rodas) | plugin `DiffDrive` do Gazebo |
| `/cmd_vel → rodas` | `cmd_vel_to_wheels` | plugin `DiffDrive` do Gazebo |
| LiDAR | `ldlidar_stl_ros2` em `/dev/lidar` | sensor `gpu_lidar` na SDF do robô |
| Câmera RGB-D | (futuro) driver da câmera real | sensor `rgbd_camera` na SDF (`/camera/*`) |
| Corpo do robô | URDF (`husky.urdf.xacro`) | URDF + SDF (`husky.sdf`) |
| `/scan`, `/odom`, `/tf`, `/camera/*` | tópicos reais | via `ros_gz_bridge` (GZ → ROS) |

O servidor web, o `map_service.py` e a UI são exatamente os mesmos — o sim é transparente do ponto de vista do navegador.

### Instalando o Gazebo e o bridge ROS↔GZ

Em Jazzy o Gazebo moderno é o **Harmonic**, separado do ROS:

```bash
sudo apt install \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-interfaces
```

Isso traz o `gz sim` (binário do Gazebo Harmonic) + o `parameter_bridge` que traduz mensagens `gz.msgs.*` ↔ `*_msgs/msg/*`.

### Onde colocar o arquivo da sala (mundo Gazebo)

**Os mundos do Gazebo ficam em `Controle_robo_web/worlds/`** (mesmo nível de `maps/`). O repositório já vem com um arquivo `worlds/empty.sdf` que cria uma sala 6×6 m com quatro paredes, um chão e uma luz — suficiente pra você testar se tudo sobe antes de trocar pelo seu mundo.

Para usar seu próprio mundo, tem dois caminhos:

1. **Substituir o padrão** — jogue seu arquivo como `worlds/sala.sdf` (ou salve por cima do `worlds/empty.sdf`):
   ```bash
   cp ~/minha_sala_projetada.sdf Controle_robo_web/worlds/empty.sdf
   ./launch.sh --sim
   ```

2. **Passar por flag** — aceita caminho absoluto ou relativo à raiz do projeto:
   ```bash
   ./launch.sh --sim --world=worlds/sala_projetada.sdf
   ./launch.sh --sim --world=/home/ubuntu/mundos/hangar.sdf
   ```

**Checklist do arquivo `.sdf` do mundo** (coisas que, se faltarem, fazem o robô cair ou o LiDAR atravessar paredes):

- `<physics>` definido (ex: `dart` ou `ode`)
- Plugins obrigatórios: `Physics`, `UserCommands`, `SceneBroadcaster`, `Sensors` com `render_engine=ogre2`
- Pelo menos uma `<light>` (sol) — senão a cena fica preta e o GPU LiDAR não vê nada
- Um `<model name="ground_plane">` estático — senão o robô despenca
- Todos os objetos com `<collision>` (paredes, móveis) — senão o LiDAR trespassa

O `worlds/empty.sdf` serve como template pronto de todos esses campos, olhe lá se estiver em dúvida.

### Rodando no modo SIM

```bash
# 1. Sim + teleop (dirige no Gazebo pelo teclado/UI web)
./launch.sh --sim

# 2. Sim + SLAM (mapeia a sala simulada com o slam_toolbox)
./launch.sh --sim --slam
#    Dirija o robô pelo Gazebo até o mapa no painel web ficar bom,
#    clique em "Salvar mapa" → fica em maps/sala.yaml

# 3. Sim + NAV2 (navegação autônoma por click-to-go dentro do Gazebo)
./launch.sh --sim --nav2
#    Clique num ponto do mapa web → o robô simulado vai até lá
```

Todas as flags combinam. `--sim --slam --world=worlds/minha_sala.sdf` também funciona.

### O robô simulado

O modelo fica em `~/ros2_ws/src/robot_nav/urdf/husky.sdf` — um diff drive customizado (corpo 31×24×14 cm em formato Husky reduzido), rodas traseiras com tração + caster esférico frontal, GPU LiDAR de 360° no topo e **câmera RGB-D** frontal. A SDF inclui:

- Sensor `gpu_lidar` (publica `/scan`, 360° @ 10 Hz)
- Sensor `rgbd_camera` frontal (publica `/camera/image`, `/camera/depth_image`, `/camera/camera_info`, `/camera/points` — FOV 60°, 320×240 @ 15 Hz)
- Plugin `DiffDrive` — consome `/cmd_vel`, publica `/odom` e TF `odom → base_link`
- Plugin `JointStatePublisher` — animação das rodas
- Plugin `PosePublisher` — snapshot da pose dos links/sensores

A URDF (`husky.urdf.xacro`) é mantida em paralelo com os mesmos `joints` e `links` (`base_link`, `base_laser`, `camera_link`, rodas) pra que o `robot_state_publisher` publique TFs estáticos consistentes — necessário pro `slam_toolbox`, AMCL e o pipeline da câmera funcionarem.

A câmera RGB-D entra no Nav2 via `VoxelLayer` no costmap local (alimenta com point cloud), permitindo detectar obstáculos baixos (mochila no chão) e altos (mesa) que o LiDAR plano não vê. Detalhes em [Câmera RGB-D](#câmera-rgb-d).

---

## Pré-requisitos

- Ubuntu 22.04 (testado) ou 24.04
- ROS2 Humble ou Jazzy instalado e no PATH (testado em **Jazzy**)
- `xacro`: `sudo apt install ros-$ROS_DISTRO-xacro`
- `robot_state_publisher`: `sudo apt install ros-$ROS_DISTRO-robot-state-publisher`
- **SLAM**: `sudo apt install ros-$ROS_DISTRO-slam-toolbox`
- **Nav2** (qualquer modo, inclusive o collision_monitor do teleop):
  ```bash
  sudo apt install \
      ros-$ROS_DISTRO-nav2-bringup \
      ros-$ROS_DISTRO-nav2-collision-monitor \
      ros-$ROS_DISTRO-nav2-map-server \
      ros-$ROS_DISTRO-nav2-amcl
  ```
- **Modo SIM (Gazebo Harmonic)** — opcional, só se você for rodar `--sim`:
  ```bash
  sudo apt install \
      ros-$ROS_DISTRO-ros-gz \
      ros-$ROS_DISTRO-ros-gz-sim \
      ros-$ROS_DISTRO-ros-gz-bridge \
      ros-$ROS_DISTRO-ros-gz-interfaces
  ```
- Python 3.10+

---

## Configuração inicial (uma vez)

### 1. Workspace ROS2

**Nada disso está neste repositório.** O `~/ros2_ws/` é um workspace ROS2 **que você cria na máquina** e precisa popular manualmente. Só o `robot_nav` mora neste repo (em `ros2_packages/robot_nav/`), via symlink. Os outros pacotes são externos e precisam ser clonados antes do `colcon build`, senão a compilação falha:

| Pacote | Origem | Obrigatório? |
|--------|--------|--------------|
| `robot_nav` | este repo (symlink) | **sempre** |
| `wheel_msgs` | repo externo | **sempre** — o `robot_nav` declara `<depend>wheel_msgs</depend>`, então mesmo no sim o `colcon build` quebra sem ele |
| `ros2-hoverboard-driver` | repo externo | só no modo real (hardware) |
| `ldlidar_stl_ros2` | repo externo | só no modo real (hardware) |

```bash
# Cria a pasta do workspace e entra nela
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 1) robot_nav — symlink do pacote deste repo
ln -s ~/Controle_robo_web/ros2_packages/robot_nav robot_nav

# 2) wheel_msgs — sempre obrigatório
git clone https://github.com/Richard-Haes-Ellis/wheel_msgs.git wheel_msgs

# 3) Só se for rodar no hardware real — pule estes dois se for só --sim
git clone https://github.com/victorfdezc/ros2-hoverboard-driver.git ros2-hoverboard-driver
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git  ldlidar_stl_ros2

# Compila tudo de uma vez
cd ~/ros2_ws
colcon build
source install/setup.bash

# Adicione ao ~/.bashrc para não precisar fazer source toda vez:
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

> **Se o `colcon build` falhar com `Package 'wheel_msgs' not found`**, é porque você pulou o passo 2. Clone o `wheel_msgs` em `~/ros2_ws/src/` e rode de novo.

Depois de editar qualquer arquivo em `ros2_packages/robot_nav/`, rode `colcon build --packages-select robot_nav` para reinstalar os launches/URDFs no `install/`.

### 2. Portas USB fixas (obrigatório)

O hoverboard e o LiDAR usam o mesmo chip USB-serial (CH340 ou similar), sem número de série. Por isso o Linux pode atribuir `/dev/ttyUSB0` e `/dev/ttyUSB1` em qualquer ordem a cada boot — causando o bug: **ao subir o LiDAR o robô para de andar**, ou vice-versa.

A solução é fixar cada dispositivo a um nome permanente usando a porta USB física:

```bash
# Com o hoverboard E o LiDAR plugados:
sudo ~/Controle_robo_web/setup_udev.sh
```

O script vai:
1. Pedir que você desplugue o LiDAR para identificar a porta do hoverboard
2. Pedir que você replugue o LiDAR para identificar a porta dele
3. Criar `/etc/udev/rules.d/99-robot-usb.rules` com os symlinks permanentes

Depois recompile o driver (necessário porque o `PORT` foi atualizado para `/dev/hoverboard`):

```bash
cd ~/ros2_ws
colcon build --packages-select ros2-hoverboard-driver
source install/setup.bash
```

Verifique se os symlinks estão corretos (devem apontar para portas **diferentes**):

```bash
ls -la /dev/hoverboard /dev/lidar
# Esperado:
# /dev/hoverboard -> ttyUSB0
# /dev/lidar      -> ttyUSB1
```

> **Atenção:** Se trocar o cabo de porta USB física (ex: plugar o hoverboard em outra entrada do notebook), rode `setup_udev.sh` novamente. Os symlinks são baseados na porta física, não no dispositivo.

### 3. Dependências Python

```bash
cd ~/Controle_robo_web/controle_web
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

---

## Como rodar

Todos os modos usam o mesmo `launch.sh` e a mesma interface web em `http://<IP>:5000`. O modo é passado como flag e propagado ao servidor web via a variável de ambiente `ROBOT_MODE` — a UI mostra um badge colorido (TELEOP / SLAM / NAV2) no topo e exibe ou esconde o painel de mapa conforme o modo.

Para descobrir o IP:

```bash
hostname -I
```

### Modo TELEOP (padrão)

Dirigir manualmente. Sobe o Nav2 Collision Monitor como camada de segurança.

```bash
cd ~/Controle_robo_web
./launch.sh
```

O script inicia, nesta ordem:

| # | Processo | Log |
|---|----------|-----|
| 1 | `ros2-hoverboard-driver` (porta `/dev/hoverboard`) | `logs/hoverboard_driver.log` |
| 2 | Nós do robô: `robot_state_publisher`, `odom_publisher`, `cmd_vel_to_wheels` | `logs/robot_nodes.log` |
| 3 | LiDAR FHL-LD20 (`ldlidar_stl_ros2`) + `obstacle_detector` | `logs/lidar.log`, `logs/obstacle_detector.log` |
| 4 | `nav2_collision_monitor` *(só segurança, não é a stack Nav2 completa)* | `logs/nav2_collision.log` |
| 5 | Servidor web Flask + Socket.IO em `http://0.0.0.0:5000` | terminal |

### Modo SLAM — mapear a sala

Primeira etapa do fluxo de navegação: você dirige o robô pela sala (com WASD, gamepad ou pad touch na UI), o `slam_toolbox` constrói o mapa em tempo real, e você salva com um clique quando terminar.

```bash
./launch.sh --slam
```

Troca o passo `[4/5]`: em vez do collision_monitor sobe o `slam_toolbox` em modo *mapping online async*. O painel **Mapa** da UI aparece automaticamente e vai mostrando o mapa crescendo à medida que você dirige.

**Como mapear bem:**
1. Comece com o robô parado no centro de onde você quer mapear.
2. Dirija **devagar** — o SLAM precisa de tempo para casar scans consecutivos. Velocidade alta quebra o matching.
3. Faça movimentos suaves, priorize retas longas e evite girar no mesmo lugar.
4. **Feche loops**: volte por onde já passou para o SLAM fechar laços e corrigir drift acumulado.
5. Evite ambientes muito simétricos (corredores longos com paredes lisas) — se o scan não tem features, o matching falha.

**Salvando o mapa:** Quando o mapa estiver bom, clique em **Salvar mapa** no canto do painel. Um prompt pede o nome (padrão: `sala`). O backend chama o `nav2_map_server/map_saver_cli`, que grava dois arquivos em `maps/`:

- `maps/sala.yaml` — metadados (resolução, origem, thresholds)
- `maps/sala.pgm` — imagem grayscale do occupancy grid

Esses arquivos ficam fora do git (`.gitignore`). Depois de salvar, você pode encerrar o SLAM (`Ctrl+C`) e rodar o modo NAV2.

### Modo NAV2 — navegação autônoma

Segunda etapa: usa um mapa já salvo pelo SLAM e ativa a stack Nav2 completa. Você clica num ponto do mapa na UI e o robô se localiza (AMCL), planeja uma rota (planner) e executa (controller) até chegar lá.

```bash
./launch.sh --nav2                           # usa maps/sala.yaml (padrão)
./launch.sh --nav2 --map=/caminho/outro.yaml # mapa customizado
```

No painel da UI:
- **Mapa** aparece com o mapa estático carregado.
- **Robô** aparece como seta laranja apontando para o yaw, atualizada a 10 Hz via TF `map→base_link`.
- **Click** no mapa envia o robô pra esse ponto (via action `navigate_to_pose`). Click+drag define o yaw final.
- **Trajetória planejada** pelo Nav2 aparece como linha azul (escutando `/plan`).
- **Último alvo** aparece como bolinha vermelha.
- **Toolbar de waypoints** permite definir uma rota multi-ponto, salvar/carregar, executar em loop. Veja [Navegação por waypoints](#navegação-por-waypoints).
- **Painel câmera** mostra o stream RGB-D do robô (~5 Hz). Veja [Câmera RGB-D](#câmera-rgb-d).

Cada navegação executada (click ou waypoint) é registrada em CSV pelo `NavMetricsCollector` em `controle_web/logs/nav_metrics/nav_metrics_YYYYMMDD.csv` — útil pra tunar o Nav2 com base em dados reais. Veja [Métricas Nav2 (CSV)](#métricas-nav2-csv).

Se o arquivo de mapa não existir, o `launch.sh` aborta com uma mensagem clara e sugere rodar `--slam` antes.

### Modo SIM — Gazebo sem hardware

Adicione `--sim` em qualquer um dos modos acima para rodar no Gazebo Harmonic em vez do hardware real. Veja a seção [Modo SIM](#modo-sim--testar-tudo-no-gazebo-sem-hardware) para detalhes completos, mas o resumo é:

```bash
./launch.sh --sim                              # sim + teleop
./launch.sh --sim --slam                       # sim + mapeamento
./launch.sh --sim --nav2                       # sim + navegação autônoma
./launch.sh --sim --world=worlds/sala.sdf      # sim com mundo customizado
```

**Seu arquivo de mundo** vai em `Controle_robo_web/worlds/` (padrão: `worlds/empty.sdf`, que já vem com uma sala 6×6m para teste inicial).

### Outras flags

```bash
./launch.sh --no-lidar              # Sobe sem LiDAR (só teleop, modos slam/nav2 exigem lidar)
./launch.sh --no-nav2               # Teleop sem collision_monitor
./launch.sh --lidar-port=/dev/lidar # Porta do LiDAR (padrão: /dev/lidar)
```

### Encerrar

`Ctrl+C` encerra todos os processos limpos. O `cleanup()` do script mata a árvore inteira de filhos (inclusive os nós spawnados pelo `ros2 launch`, que ficariam órfãos se só matasse o pai).

> **Por que tem um handler de SIGINT custom no `app.py`?** O `rclpy.init()` instala seus próprios handlers de SIGINT/SIGTERM que engolem o Ctrl+C — o processo Python fica preso esperando o executor do ROS2 que nunca acorda, e o bash em foreground nunca roda o `trap cleanup`. Por isso o `app.py` instala handlers Python que sobrescrevem os do rclpy: primeiro Ctrl+C faz shutdown limpo, segundo Ctrl+C força `os._exit(1)` imediato.

---

## Controles

### Teclado

| Tecla | Ação |
|-------|------|
| `W` / `↑` | Avançar |
| `S` / `↓` | Recuar |
| `A` / `←` | Girar esquerda |
| `D` / `→` | Girar direita |
| `Espaço` | Parar |

Combinações são suportadas (ex: `W + D` = frente + direita).

### Gamepad (PS4 / Xbox)

| Controle | Ação |
|----------|------|
| Analógico esquerdo | Movimento (linear + angular) |
| `X` (PS4) / `A` (Xbox) — segurado | Trava de emergência |
| `□` (PS4) / `X` (Xbox) | Reduz velocidade (0.8×) |
| `○` (PS4) / `B` (Xbox) | Aumenta velocidade (até 4×) |

### Velocidades

- Base: `0.3 m/s` linear, `0.5 rad/s` angular
- Multiplicador: `0.8×` a `4.0×` (controlado pelo gamepad ou interface web)

---

## Câmera RGB-D

O robô (sim e potencialmente real) tem uma câmera RGB-D frontal. Ela serve a dois propósitos distintos no sistema:

**1. Detecção de obstáculos no Nav2 (via `VoxelLayer`):**

O point cloud (`/camera/points`) entra como segunda observation source do `local_costmap`, junto com o LiDAR (`/scan`). Como a câmera vê em **3D** (até ~1.5 m de altura), ela detecta:

- Obstáculos **baixos** que o LiDAR plano (mounted a 9 cm do chão no Husky sim) perde — mochilas, livros no chão, base de cadeira.
- Obstáculos **altos** que o LiDAR não cobre — beira de mesa, peitoril.
- Obstáculos **dinâmicos** entrando no FOV frontal do robô.

A configuração filtra altura no plugin (`min_obstacle_height: 0.05`, `max_obstacle_height: 1.5`) pra ignorar o chão e o teto. O `VoxelLayer` projeta as marcas 3D no costmap 2D, fazendo a fusão LiDAR + câmera transparente pro DWB.

**Não é usada pra localização** — AMCL fica 100% no LiDAR. Adicionar a câmera no AMCL exigiria re-mapear com os dois sensores juntos e teria pouco ganho (LiDAR já cobre 360°). Pra ganhos reais de localização visual seria necessário migrar pra um SLAM visual tipo RTAB-Map.

**2. Stream pro web (via `camera_bridge.py`):**

O módulo `controle_web/camera_bridge.py` subscreve `/camera/image`, comprime cada frame em JPEG (qualidade 60), throttle de 5 Hz, emite no evento Socket.IO `camera_frame`. A UI exibe num `<img>` abaixo do mapa. Útil pra:

- Ver o que o robô está enxergando enquanto navega.
- Debug — confirmar visualmente se o robô está orientado certo, se os obstáculos detectados existem mesmo.
- Futura camada de detecção semântica (objetos/pessoas/zonas).

**Posição da câmera (`husky.sdf` + `husky.urdf.xacro`):**

```
camera_link
  pose: x=0.16, y=0, z=0.02 (frente do robô, altura média do corpo)
  FOV horizontal: 60° (1.0472 rad)
  resolução: 320×240
  taxa: 15 Hz
  alcance: 0.2 – 8.0 m
```

**Tópicos publicados (modo SIM via `ros_gz_bridge`):**

| Tópico | Tipo | Finalidade |
|--------|------|------------|
| `/camera/image` | `sensor_msgs/Image` (RGB) | Stream pro web |
| `/camera/depth_image` | `sensor_msgs/Image` (float32) | Disponível, não usado direto pelo Nav2 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Calibração intrínseca |
| `/camera/points` | `sensor_msgs/PointCloud2` | Alimenta o `VoxelLayer` |

Pra migrar pra hardware real, basta substituir o sensor `rgbd_camera` da SDF pelo driver da câmera real (RealSense, Orbbec, etc.) garantindo que ele publique nos mesmos tópicos. Nada do app ou do Nav2 muda.

---

## Navegação por waypoints

Em modo NAV2, além do click-to-go simples, a UI tem uma **toolbar de waypoints** que permite definir e executar rotas com múltiplos pontos.

**Como definir uma rota:**

1. Clica em **+ Waypoint** pra entrar em modo de adição.
2. Cada click no mapa adiciona um ponto. Click+drag define o yaw final naquele ponto (a seta do marker mostra a direção desejada).
3. Marca **Loop** se quiser que a rota repita indefinidamente.
4. Clica em **▶ Iniciar** — o `MapBridge._wp_runner` envia os goals em sequência via action `navigate_to_pose`, esperando cada um terminar antes do próximo.

**Salvar e recarregar rotas:**

- **💾 Salvar rota** grava em `maps/routes/<nome>.json` com `[{x, y, yaw}, ...]`.
- **📂 Carregar** lista as rotas salvas e permite restaurar uma.
- Em refresh da página (F5), se houver waypoints definidos eles são restaurados automaticamente via `waypoints_restored`.

**Como o `_wp_runner` decide avançar:**

Usa o status terminal da action `navigate_to_pose` do Nav2 — não estima chegada por distância/yaw. Comportamento:

- `STATUS_SUCCEEDED` → avança pro próximo waypoint imediatamente.
- `STATUS_ABORTED` → re-tenta até 2 vezes com 2 s de pausa entre tentativas. Após 3 falhas, pula o waypoint (emite `skipped: true` pra UI).
- `STATUS_CANCELED` → sai limpo (acontece quando você clica em **■ Parar**).
- Timeout de segurança de 120 s por waypoint, caso o action server não responda.

Entre cada waypoint, o runner limpa o `local_costmap` (`/local_costmap/clear_entirely_local_costmap`) pra evitar que células de custo alto da última parada atrapalhem o próximo goal.

**Por que não publicar direto em `/goal_pose`:**

Versões anteriores publicavam `/goal_pose` e adivinhavam chegada por TF. Era frágil — se o Nav2 abortava (obstáculo, timeout interno), o runner só descobria após 60 s de timeout. Usando a action, o runner reage a SUCCEEDED/ABORTED em tempo real.

---

## Métricas Nav2 (CSV)

Em modo NAV2, o `NavMetricsCollector` (em `controle_web/nav_metrics.py`) registra cada navegação em CSV. Roda em thread daemon, subscreve tópicos do Nav2 e gera uma linha por tentativa de navegação (do ACCEPTED até SUCCEEDED/ABORTED/CANCELED).

**O que é gravado em `controle_web/logs/nav_metrics/nav_metrics_YYYYMMDD.csv`:**

```
nav_id, start_ts, end_ts, duration_s, status,
start_x, start_y, end_x, end_y, end_yaw,
initial_plan_length_m, replans,
rec_backup, rec_spin, rec_wait,
distance_traveled_m, avg_linear_speed, max_linear_speed,
time_stopped_s, direction_reversals
```

**Uso típico:**

- **Tuning de DWB:** alta `time_stopped_s` ou alta contagem de `replans` em rotas curtas indica que o controller está oscilando — sintoma de pesos de critic mal calibrados.
- **Tuning de recoveries:** `rec_backup`/`rec_spin`/`rec_wait` muito altos indicam que o Nav2 está caindo em recovery muito — geralmente costmap saturado ou inflação alta demais.
- **Detecção de regressão:** depois de mexer em parâmetros, comparar CSV antes/depois numa mesma rota mostra objetivamente se o tuning ajudou ou piorou.

Tópicos consumidos:
- `/navigate_to_pose/_action/status` — detecta início/fim de cada navegação.
- `/backup/_action/status`, `/spin/_action/status`, `/wait/_action/status` — conta cada vez que recovery é acionada.
- `/plan` — comprimento do caminho + replans.
- `/odom` — distância percorrida + velocidades.
- `/cmd_vel` — tempo parado + inversões de direção.

CSV diário (não por execução) — todas as navegações do dia ficam num arquivo só, facilitando comparação ao longo do tempo.

---

## Arquitetura

### Tópicos ROS2

| Tópico / Action | Tipo | Produtor | Consumidor | Quando |
|--------|------|----------|------------|--------|
| `/cmd_vel` | `geometry_msgs/Twist` | servidor web (teleop) / `velocity_smoother` (nav2) | `cmd_vel_to_wheels` | sempre |
| `/wheel_vel_setpoints` | `wheel_msgs/WheelSpeeds` | `cmd_vel_to_wheels` | hoverboard driver | sempre |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR driver | `obstacle_detector` / `slam_toolbox` / `amcl` / `voxel_layer` | sempre |
| `/odom` | `nav_msgs/Odometry` | `odom_publisher` | `slam_toolbox` / `amcl` / `nav_metrics` | sempre |
| `/obstacle_info` | `std_msgs/String` (JSON) | `obstacle_detector` | (monitoramento) | teleop |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` / `map_server` | `map_service.py` (ponte web) | slam, nav2 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | `map_service.py` (legacy) | `bt_navigator` | nav2 |
| `/plan` | `nav_msgs/Path` | `planner_server` | `map_service.py` (ponte web) / `nav_metrics` | nav2 |
| `/camera/image` | `sensor_msgs/Image` (RGB) | Gazebo `rgbd_camera` (sim) | `camera_bridge.py` (stream web) | sim, futuro real |
| `/camera/depth_image` | `sensor_msgs/Image` (float32) | Gazebo `rgbd_camera` | (disponível, não consumido) | sim, futuro real |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo `rgbd_camera` | (calibração) | sim, futuro real |
| `/camera/points` | `sensor_msgs/PointCloud2` | Gazebo `rgbd_camera` | `voxel_layer` (`local_costmap`) | sim, futuro real |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `bt_navigator` (server) | `MapBridge` (client em waypoints e click) / `nav_metrics` | nav2 |
| `/backup/_action/status` | `action_msgs/GoalStatusArray` | `behavior_server` | `nav_metrics` (contagem de recoveries) | nav2 |
| `/spin/_action/status` | `action_msgs/GoalStatusArray` | `behavior_server` | `nav_metrics` | nav2 |
| `/wait/_action/status` | `action_msgs/GoalStatusArray` | `behavior_server` | `nav_metrics` | nav2 |

**TFs publicadas:**
- `base_link → base_laser`, `base_link → wheels` — static (URDF via `robot_state_publisher`)
- `odom → base_link` — dinâmica (`odom_publisher` a partir do feedback das rodas)
- `map → odom` — dinâmica, em SLAM pelo `slam_toolbox`, em NAV2 pelo `amcl`

### Ponte ROS2 ↔ Web para mapa e navegação

O arquivo `controle_web/map_service.py` contém uma classe `MapBridge` que roda dentro do servidor Flask como uma thread daemon com seu próprio executor ROS2. Isso é o que permite o mapa aparecer no navegador e os clicks virarem comandos Nav2.

**O que o MapBridge faz:**

| Responsabilidade | Como |
|------------------|------|
| Receber o mapa | Subscribe `/map` com QoS `TRANSIENT_LOCAL` (a mensagem é *latched*, sem essa durability o subscriber nunca recebe). Converte o `OccupancyGrid` em PNG grayscale com `numpy` + `Pillow` (−1 cinza, 0 branco, ≥50 preto), flipa verticalmente (ROS y sobe, PNG y desce), base64-encoda e emite `map_update` via Socket.IO com `{info, png_b64}` |
| Rastrear o robô | `tf2_ros.TransformListener` em polling a 10 Hz. Olha `map→base_link`, extrai x/y/yaw (yaw do quaternion via `atan2`), emite `robot_pose` via Socket.IO |
| Receber trajetória | Subscribe `/plan`, converte cada pose em `{x, y}`, emite `plan_update` via Socket.IO |
| Enviar goal (click-to-go simples) | Publisher em `/goal_pose` (`PoseStamped`, frame `map`). Handler `nav_goal` recebe `{x, y, yaw}` do click no canvas |
| Executar waypoints | `ActionClient` de `NavigateToPose`. O `_wp_runner` (thread separada) envia goals em sequência e reage a SUCCEEDED/ABORTED/CANCELED em tempo real. Re-tenta abortados até 2 vezes, pula após 3 falhas. Limpa o `local_costmap` entre waypoints |
| Persistir rotas | Handlers `save_route`/`load_route`/`list_routes` gravam JSON em `maps/routes/<nome>.json` |
| Salvar mapa | Handler `save_map` chama `ros2 run nav2_map_server map_saver_cli -f maps/<nome> --ros-args -p map_subscribe_transient_local:=true` via subprocess. O `map_subscribe_transient_local:=true` é obrigatório porque o `/map` é latched |

**Rodar só em modo slam/nav2:** o `app.py` só instancia o `MapBridge` se `ROBOT_MODE in ('slam', 'nav2')` — no teleop não há `/map` pra subscriber, então o módulo nem sobe. Falha na inicialização do MapBridge não derruba o servidor (só loga um warning e desabilita o painel de mapa).

### Outras pontes

| Módulo | Quando sobe | Função |
|--------|-------------|--------|
| `camera_bridge.py` | qualquer modo | Subscribe `/camera/image`, comprime JPEG, emite `camera_frame` via Socket.IO @ 5 Hz |
| `nav_metrics.py` | só `nav2` | Subscribe action statuses + `/plan` + `/odom` + `/cmd_vel`, grava CSV por navegação |

**Cliente (navegador):** o `static/js/map.js` escuta todos esses eventos, mantém estado local (`mapInfo`, `mapImage`, `robotPose`, `plan`, `lastGoal`) e redesenha o canvas a ~15 Hz. A conversão click→mundo usa `origin + resolution`:

```js
world_x = origin_x + px_in_img * resolution
world_y = origin_y + (height-1 - py_in_img) * resolution
```

Precisa inverter o eixo y porque o PNG foi flipado verticalmente antes de ser mandado.

### Detecção de obstáculos (modo TELEOP)

O `obstacle_detector` divide o campo de visão em 6 setores e classifica por distância:

| Cor | Distância |
|-----|-----------|
| Verde | > 1,5 m |
| Amarelo | 0,5 – 1,5 m |
| Vermelho | < 0,5 m |

Os dados são escritos em `/tmp/obstacle_current.json` e lidos pelo Flask a 5 Hz via thread separada (sem ROS2 dentro do Flask).

### Arquivos principais

```
Controle_robo_web/
├── launch.sh                          # Launcher principal (flags --slam / --nav2 / --sim / --map=)
├── setup.sh                           # Bootstrap inicial (apt, ros2_ws, colcon build)
├── setup_udev.sh                      # Configura portas USB fixas
├── maps/                              # Mapas e rotas salvos (ignorado pelo git)
│   ├── sala.yaml                      # Metadados: resolução, origem, thresholds
│   ├── sala.pgm                       # Grayscale do occupancy grid
│   └── routes/                        # Rotas de waypoints (JSON)
│       └── <nome>.json
├── worlds/                            # Mundos do Gazebo usados pelo --sim
│   ├── empty.sdf                      # Mundo padrão (sala 6×6 m vazia)
│   ├── educacao_criativa.sdf          # Mundo customizado
│   └── small_box.sdf                  # Obstáculo pra spawnar em testes
├── ros2_packages/
│   └── robot_nav/                     # Pacote ROS2 (linkado em ~/ros2_ws/src/robot_nav)
│       ├── launch/                    # robot, lidar, slam, nav2, sim, nav2_collision
│       ├── urdf/
│       │   ├── robot.urdf.xacro       # URDF do hoverboard real (referência)
│       │   ├── husky.urdf.xacro       # URDF principal (com camera_link)
│       │   └── husky.sdf              # SDF do robô simulado (com sensor RGB-D)
│       ├── config/
│       │   ├── nav2_params.yaml       # Tuning AMCL + DWB + costmaps + voxel_layer
│       │   └── collision_monitor.yaml # Zonas de freada (modo teleop)
│       └── robot_nav/                 # Nodes Python (odom, cmd_vel_to_wheels, ...)
└── controle_web/
    ├── app.py                         # Servidor Flask + Socket.IO (lê ROBOT_MODE)
    ├── map_service.py                 # Ponte mapa/pose/plan + ActionClient + waypoints
    ├── camera_bridge.py               # Subscribe /camera/image → JPEG via Socket.IO
    ├── nav_metrics.py                 # Coleta métricas do Nav2 em CSV
    ├── controllers/
    │   └── robot_controller.py        # ROS2Controller (publica /cmd_vel)
    ├── templates/index.html           # Interface web (badge + mapa + waypoints + câmera)
    ├── static/
    │   ├── css/styles.css
    │   └── js/
    │       ├── client.js              # Teclado/gamepad → Socket.IO + handler câmera
    │       ├── gamepad.js             # Leitura do gamepad e visualização
    │       └── map.js                 # Canvas do mapa, render, click → goal, waypoints
    └── logs/                          # Logs rotativos
        └── nav_metrics/               # CSVs do NavMetricsCollector (por dia)

~/ros2_ws/src/
├── robot_nav -> ~/Controle_robo_web/ros2_packages/robot_nav  # symlink
├── ros2-hoverboard-driver/                 # Driver C++ do hoverboard (repo separado)
│   └── include/.../config.hpp              # PORT = /dev/hoverboard
├── ldlidar_stl_ros2/                       # Driver do LiDAR FHL-LD20 (repo separado)
└── wheel_msgs/                             # Mensagens custom das rodas (repo separado)
```

---

## Tuning do Nav2

A configuração em `ros2_packages/robot_nav/config/nav2_params.yaml` foi calibrada iterativamente pra ambiente dinâmico (sala de aula com pessoas e móveis se mexendo). Os valores não-default importantes:

**AMCL (localização):**

| Parâmetro | Valor | Por quê |
|-----------|-------|---------|
| `do_beamskip` | `true` | Quando feixes batem em obstáculos não-mapeados (cadeira nova, pessoa), AMCL ignora esses raios em vez de penalizar partículas que estariam corretas. Sem isso, ~30% de feixes "fora do mapa" derruba a localização |
| `beam_skip_distance` | `0.5` | Distância máx pra considerar feixe como "match" |
| `beam_skip_threshold` | `0.3` | Fração de partículas que precisa concordar pra fazer skip |

**Planner (`nav2_navfn_planner`):**

| Parâmetro | Valor | Por quê |
|-----------|-------|---------|
| `tolerance` | `0.30` | Se o ponto exato estiver bloqueado (cadeira encostada, pessoa parada), planner aceita rota até 30 cm do alvo em vez de falhar |

**Goal checker:**

| Parâmetro | Valor | Por quê |
|-----------|-------|---------|
| `xy_goal_tolerance` | `0.40` | Folga pra ambiente dinâmico — se obstáculo atrapalha o ponto exato, robô para a até 40 cm |
| `yaw_goal_tolerance` | `0.35` | ~20°, mesma lógica |

**DWB Local Planner (controller):**

| Parâmetro | Valor | Por quê |
|-----------|-------|---------|
| `min_vel_x` | `-0.1` | Permite ré pequena em manobras apertadas. Antes `0.0` (só frente) fazia robô travar 15 s antes de chamar BackUp recovery |
| `BaseObstacle.scale` | `0.15` | Era `0.02` — peso 1600× menor que `PathDist` (32). Com 0.15 o controller ainda segue rota mas evita raspar |
| `Oscillation.scale` | `0.1` | Default `1.0` punia manobras legítimas em passagem apertada como "oscilação" e robô ficava parado pensando 2 min |
| `PathAlign.scale` / `PathDist.scale` / `GoalAlign.scale` | `32` cada | Manter a rota planejada como prioridade média |
| `GoalDist.scale` | `24` | Atratividade do destino |
| `RotateToGoal.scale` | `32` | Rotação final pra atingir yaw alvo |

**Costmap (`local_costmap` com `VoxelLayer`):**

| Parâmetro | Valor | Por quê |
|-----------|-------|---------|
| `plugins` | `[voxel_layer, inflation_layer]` | `VoxelLayer` em vez de `ObstacleLayer` pra aceitar tanto `LaserScan` (LiDAR) quanto `PointCloud2` (câmera) e fundir os dois |
| `observation_sources` | `scan pointcloud` | LiDAR + câmera RGB-D |
| `pointcloud.min_obstacle_height` | `0.05` | Ignora chão (senão o robô bate em si mesmo como obstáculo) |
| `pointcloud.max_obstacle_height` | `1.5` | Ignora teto |
| `inflation_radius` | `0.25` | Raio de inflação pra `cost_scaling_factor: 3.5` (gradiente moderado) |

**Recoveries (`behavior_server`):**

`BackUp` + `Spin` + `Wait` configurados — chamados pelo BT quando o controller não consegue avançar em ~15 s (`progress_checker.movement_time_allowance`). Garante que o robô tenta sair de enrascadas dando ré ou girando antes de declarar falha.

**Observação:** esses valores foram calibrados em sim Gazebo e podem precisar de ajustes finos no hardware real (atrito do hoverboard, latência do LiDAR físico, etc.). Use o CSV do `NavMetricsCollector` pra medir antes/depois de cada mudança.

---

## Logs

Todos os logs ficam em `controle_web/logs/`:

| Arquivo | Conteúdo |
|---------|----------|
| `hoverboard_driver.log` | Saída do driver C++ (serial, erros) |
| `robot_nodes.log` | robot_state_publisher, odom, cmd_vel_to_wheels |
| `lidar.log` | Driver LiDAR |
| `obstacle_detector.log` | Detecção de obstáculos |
| `nav2_collision.log` | Nav2 Collision Monitor (modo teleop) |
| `nav2.log` | Stack Nav2 completa (modo nav2) — planner, controller, BT, recoveries |
| `slam.log` | slam_toolbox (modo slam) |
| `sim.log` | Gazebo + bridges (modo sim) |
| `movements.log` | Histórico de comandos em JSON Lines |
| `movements.txt` | Histórico legível em português |
| `nav_metrics/nav_metrics_YYYYMMDD.csv` | Uma linha por navegação: status, replans, recoveries, distância, velocidade |

Para acompanhar em tempo real:

```bash
tail -f controle_web/logs/hoverboard_driver.log
tail -f controle_web/logs/lidar.log
```

---

## Limitações conhecidas

Coisas que ainda não funcionam perfeitamente ou que exigem atenção ao usar SLAM/Nav2:

- **Contenção do `/cmd_vel` em modo NAV2.** Tanto o teleop (Socket.IO → `/cmd_vel`) quanto o `velocity_smoother` do Nav2 publicam no mesmo tópico. Se você mover o joystick durante uma navegação autônoma, os comandos se atropelam — a última mensagem vence. Na prática funciona como "override manual por cima do Nav2", mas não é um protocolo robusto. *Mitigação futura:* roteamento explícito via `twist_mux`.
- **Drift de odometria.** O `odom_publisher` integra o feedback das rodas do hoverboard. Em mapeamentos longos ou salas com piso escorregadio, o drift acumula e o SLAM fecha loops mal. Dirija devagar e volte por onde já passou para ajudar o `slam_toolbox` a corrigir.
- **Ambientes muito simétricos.** Corredor longo com paredes lisas, salas quadradas vazias: o scan-matching do SLAM não encontra features suficientes e o mapa pode dobrar sobre si mesmo. AMCL tem o mesmo problema: pose pode "deslizar" ao longo do corredor. Prefira mapear ambientes com móveis, quinas e variação. *Mitigação futura:* RTAB-Map (visual SLAM) usa features visuais da câmera pra desambiguar.
- **Câmera não contribui pra localização.** A câmera RGB-D entra no costmap (desvio de obstáculos baixos/altos) mas o AMCL fica 100% no LiDAR. Adicionar a câmera no AMCL exigiria re-mapear com os dois sensores e ainda assim teria ganho modesto. Pra ganho real de localização visual, o caminho é trocar AMCL por RTAB-Map.
- **Bateria do hoverboard.** Sem bateria o driver até sobe, mas falha ao escrever na porta serial (`Error writing to hoverboard serial port`) — não é bug do código. Conecte a bateria antes de abrir um bug.
- **Pipeline não validado end-to-end em hardware.** O fluxo todo (TELEOP, SLAM, NAV2, waypoints, câmera) foi exercitado extensivamente no sim Gazebo. No hardware real ainda faltam ajustes finos de tuning conforme a dinâmica do hoverboard físico — use o CSV do `NavMetricsCollector` pra calibrar.
- **Câmera no real precisa de driver específico.** No sim a câmera vem do plugin Gazebo. Pra hardware real, escolher o modelo (RealSense D435, Orbbec, etc.) e substituir o sensor da SDF pelo driver ROS2 correspondente publicando nos mesmos tópicos `/camera/*`.

---

## Solução de problemas

### Robô não anda quando o LiDAR está ligado (ou vice-versa)

Causa: os dois dispositivos caíram no mesmo `/dev/ttyUSBX`. Veja [Portas USB fixas](#2-portas-usb-fixas-obrigatório).

```bash
# Diagnóstico rápido:
ls -la /dev/hoverboard /dev/lidar
# Se apontarem para a mesma porta → rode setup_udev.sh novamente
```

### Porta /dev/hoverboard não encontrada

```bash
ls /dev/ttyUSB*
# Se nenhuma aparecer: verifique cabo USB e permissões
sudo usermod -aG dialout $USER  # adiciona usuário ao grupo serial
# Depois faça logout e login
```

### Driver do hoverboard falha ao abrir porta

```bash
# Verifique permissões:
ls -la /dev/hoverboard
# Deve ter MODE=0666 ou pertencer ao grupo dialout

# Force permissão temporária:
sudo chmod 666 /dev/hoverboard
```

### LiDAR não publica /scan

```bash
# Verifique se o nó está rodando:
ros2 node list | grep lidar

# Verifique se há dados no tópico:
ros2 topic hz /scan

# Veja o log:
tail -f controle_web/logs/lidar.log
```

### Nav2 não instalado (aviso no launch.sh)

```bash
sudo ./install_nav2.sh
```

Sem Nav2, o robô funciona normalmente — apenas sem parada automática por obstáculos.

### Painel de mapa não aparece na UI (modo SLAM ou NAV2)

Checklist:

1. Confirme que subiu no modo certo: o badge no topo da página deve mostrar `SLAM` ou `NAV2` (não `TELEOP`). Se estiver `TELEOP`, o `MapBridge` nem é instanciado.
2. Confirme que o `/map` está sendo publicado:
   ```bash
   ros2 topic echo /map --once
   ```
   Em SLAM pode demorar alguns segundos até o `slam_toolbox` publicar o primeiro mapa (ele espera acumular scans).
3. Olhe o log do servidor web no terminal: o `MapBridge` loga `[map] recebido /map (WxH)` quando o subscriber dispara. Se não aparecer, quase certo que o QoS está errado (`TRANSIENT_LOCAL` é obrigatório).
4. Em NAV2, se o `map_server` não sobe, o `/map` nunca aparece — veja `logs/nav2.log`.

### Salvar mapa falha com "no messages received"

Causa quase sempre é o `map_server`/`slam_toolbox` publicando o `/map` como *latched* (`TRANSIENT_LOCAL`), e o `map_saver_cli` tentando se inscrever com QoS default. O `MapBridge` já passa `-p map_subscribe_transient_local:=true`, mas se você rodar manualmente:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/sala \
    --ros-args -p map_subscribe_transient_local:=true
```

### Nav2 rejeita o goal (TF timeout ou frame error)

```bash
# Confirme que a cadeia de TFs está completa:
ros2 run tf2_tools view_frames
# Esperado: map → odom → base_link → base_laser

# Se faltar map → odom: o AMCL não conseguiu localizar o robô.
#   Certifique-se de que o mapa carregado é o mesmo onde o robô está
#   e dê um "pose inicial" empurrando o robô um pouco com o teclado.

# Se faltar odom → base_link: odom_publisher não está rodando
#   (veja logs/robot_nodes.log).
```

### `rclpy` reclama de `Could not find a valid TF` no MapBridge

O `MapBridge` faz `lookup_transform('map', 'base_link', ...)` a 10 Hz. No começo, antes do AMCL/SLAM publicar `map → odom`, essas buscas falham e logam warnings — é esperado nos primeiros segundos. Se persistir, veja o item acima.
