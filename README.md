# Controle Web do Robô 4 Rodas

Interface web para controlar um robô **skid-steer de 4 rodas** com duas placas de hoverboard agregadas por um **Arduino MEGA 2560**, LiDAR FHL-LD20, detecção de obstáculos, **mapeamento SLAM** e **navegação autônoma Nav2 com click-to-go** no mapa web.

## Sumário

- [Guia rápido — do zero ao click-to-go](#guia-rápido--do-zero-ao-click-to-go)
- [Visão geral](#visão-geral)
- [Os três modos de operação](#os-três-modos-de-operação)
- [Modo SIM — testar tudo no Gazebo sem hardware](#modo-sim--testar-tudo-no-gazebo-sem-hardware)
- [Pré-requisitos](#pré-requisitos)
- [Configuração inicial (uma vez)](#configuração-inicial-uma-vez)
  - [1. Workspace ROS2](#1-workspace-ros2)
  - [2. Portas USB fixas](#2-portas-usb-fixas-obrigatório)
  - [3. Firmware da Arduino MEGA](#3-firmware-da-arduino-mega)
  - [4. Dependências Python](#4-dependências-python)
- [Como rodar](#como-rodar)
  - [Modo TELEOP (padrão)](#modo-teleop-padrão)
  - [Modo SLAM — mapear a sala](#modo-slam--mapear-a-sala)
  - [Modo NAV2 — navegação autônoma](#modo-nav2--navegação-autônoma)
- [Controles](#controles)
- [Sensores embarcados (BNO055 + PMW3901)](#sensores-embarcados-bno055--pmw3901)
- [Sinalização do robô (LEDs, relé, botão)](#sinalização-do-robô-leds-relé-botão)
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

Passo a passo condensado para quem está pegando uma máquina nova e quer ver o robô andando, primeiro no Gazebo e depois no hardware real. Todas as seções abaixo têm mais detalhes, isto aqui é o caminho feliz.

### 1. Instalar o ROS2 Jazzy

Siga o guia oficial (~10 min): https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

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

```bash
cd ~/Controle_robo_web
./setup.sh
```

Cobre:
- **apt install**: `xacro`, `robot-state-publisher`, `slam-toolbox`, `nav2-bringup`, `nav2-collision-monitor`, `nav2-map-server`, `nav2-amcl`, `ros-gz*` (para Jazzy), `git`, `python3-venv`, `python3-pip`.
- **Workspace**: cria `~/ros2_ws/src`, faz symlink do `robot_nav` deste repo, clona `wheel_msgs` ([Richard-Haes-Ellis/wheel_msgs](https://github.com/Richard-Haes-Ellis/wheel_msgs)), compila com `colcon build` e adiciona o `source` ao `~/.bashrc`.

| Pacote | Origem | Obrigatório? |
|--------|--------|--------------|
| `robot_nav` | este repo (symlink) | **sempre** |
| `wheel_msgs` | repo externo | **sempre** (até no sim, senão `colcon build` falha) |
| `ldlidar_stl_ros2` | repo externo | só no modo real (hardware) |

> Não existe mais driver C++ separado do hoverboard. A ponte para as duas placas é nativa: o nó `mega_bridge` (Python, em `robot_nav`) conversa com a Arduino MEGA via USB, e a MEGA repassa para as placas de hoverboard pelos UARTs hardware.

### 4. (Só hardware real) Compilar e flashear o firmware da MEGA

O firmware C++ da MEGA fica em `firmware/mega_bridge/` (projeto PlatformIO).

```bash
# Instale o PlatformIO (uma vez):
pip install --user platformio

# Compile e flasheia com a MEGA plugada via USB:
cd ~/Controle_robo_web/firmware/mega_bridge
pio run -t upload
```

Pule este passo se só vai usar `--sim`.

### 5. (Só hardware real) Fixar portas USB

```bash
sudo ~/Controle_robo_web/setup_udev.sh
```

Cria `/dev/mega` (Arduino MEGA) e `/dev/lidar` (FHL-LD20), baseados na porta USB física.

### 6. Primeira execução — teste rápido no sim

```bash
cd ~/Controle_robo_web
./launch.sh --sim
```

O `launch.sh` (e o `start.sh`) faz incrementalmente: cria/atualiza o symlink no workspace ROS2, roda `colcon build` se algum arquivo do `robot_nav` mudou, instala `python3-serial` se faltar e cria o venv Python do servidor. Tudo cacheado por hash — execuções seguintes pulam direto.

O que deve acontecer:
1. Uma janela do Gazebo Harmonic abre com o mundo padrão (`worlds/empty.sdf`, sala 6×6 m).
2. No terminal: `Iniciando servidor web em http://0.0.0.0:5000 (modo: teleop [SIM/Gazebo])`.
3. Abra `http://localhost:5000` no navegador.
4. Clique na página, use `WASD` ou setas — o robô se move no Gazebo.

`Ctrl+C` encerra tudo.

### 7. Mapear a sala simulada (SLAM)

```bash
./launch.sh --sim --slam
```

Dirija devagar pela sala. Quando o mapa estiver bom, clique em **Salvar mapa** → gera `maps/sala.yaml` + `maps/sala.pgm`.

### 8. Navegação autônoma (NAV2 click-to-go)

```bash
./launch.sh --sim --nav2
```

Clique num ponto livre do mapa → Nav2 calcula a rota e o robô vai até lá.

### 9. Migrar para o hardware real

Quando o fluxo estiver redondo no sim, tire o `--sim` dos comandos. A mesma UI, o mesmo `/goal_pose`, o mesmo mapa (se for a mesma sala). Pré-requisitos: passos **4** (firmware flasheado) e **5** (udev) feitos.

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
  mega_bridge (Python, robot_nav)
        │  USB serial @ 230400 baud, frames 0xAA 0x55
        ▼
  Arduino MEGA 2560 (firmware C++)
        │  Serial1 ───► placa hoverboard FRENTE  (FL + FR)
        │  Serial2 ───► placa hoverboard TRÁS    (RL + RR)
        │  I²C    ───► BNO055   (IMU 9-DOF)
        │  SPI    ───► PMW3901  (optical flow)
        │  pinos  ───► WS2812 / relé / LED / botão

  Sensores publicados pela MEGA via mega_bridge:
    /hoverboard/{front,rear}/{left,right}/velocity  (RPM por roda)
    /imu/data           (sensor_msgs/Imu — orientação, gyro, accel)
    /optical_flow       (Vector3Stamped — dx, dy, qualidade)
    /battery/{front,rear}
    /start_button

  LiDAR FHL-LD20  ───────► /scan  (direto no USB do PC, fora da MEGA)

  ┌─────────────────────────┬─────────────────────────────────────────┐
  │  TELEOP                 │  SLAM                   │  NAV2          │
  │  + nav2_collision_mon.  │  slam_toolbox           │  map_server +  │
  │                         │  → /map (ao vivo)       │  amcl + planner│
  │                         │  → TF map→odom          │  + controller +│
  │                         │                         │  bt_navigator +│
  │                         │                         │  behaviors +   │
  │                         │                         │  velocity_smth │
  │                         │                         │  + waypoint_fl │
  │                         │                         │  costmaps com  │
  │                         │                         │  VoxelLayer    │
  │                         │                         │  (só LiDAR)    │
  └─────────────────────────┴─────────────────────────────────────────┘
        │
        ▼  Pontes ROS2 → Socket.IO (no app Flask)
  map_service.py:    /map → PNG, TF map→base_link, /plan,
                     NavigateToPose action client (click + waypoints)
  nav_metrics.py:    grava CSV por navegação (status, replans, recoveries)
        │
        ▼
  Navegador
    Canvas do mapa: mapa + robô + plano + waypoints + último alvo
    Toolbar wp:     adicionar/limpar/iniciar/parar/loop, salvar/carregar rotas
```

---

## Os três modos de operação

| Modo | Flag | Pra quê serve | O que sobe a mais |
|------|------|---------------|-------------------|
| **TELEOP** | *(padrão)* | Dirigir manualmente | `nav2_collision_monitor` — só segurança (freia se tiver obstáculo perto) |
| **SLAM** | `--slam` | Construir o mapa da sala | `slam_toolbox` em modo *mapping online* (gera `/map` ao vivo) |
| **NAV2** | `--nav2` | Navegação autônoma + click-to-go + waypoints + métricas | `map_server` + `amcl` (com beam_skip) + `planner_server` + `controller_server` (DWB) + `bt_navigator` + `behavior_server` + `velocity_smoother` + `waypoint_follower` + `NavMetricsCollector` (CSV) |

Nos três modos o servidor web, a ponte MEGA (`mega_bridge`) e o LiDAR rodam normalmente — você sempre pode dirigir manualmente, mesmo durante SLAM ou NAV2.

### Espera, por que aparece "nav2" em dois lugares? (collision_monitor vs Nav2 completo)

Dá pra confundir: no modo TELEOP o log mostra `nav2_collision.log` e no modo `--nav2` aparece `nav2.log`. **Não são dois jeitos de rodar o Nav2** — são dois pedaços distintos do mesmo projeto upstream Nav2:

- **`nav2_collision_monitor`** (modo TELEOP) — nó pequeno de segurança. Só intercepta `/cmd_vel`, olha o LiDAR, e freia o robô se detectar obstáculo perto. **Não** planeja rota, **não** precisa de mapa, **não** sabe onde o robô está no mundo.
- **Stack Nav2 completa** (modo `--nav2`) — uma dúzia de nós que fazem navegação autônoma: carregam um mapa salvo (`map_server`), localizam por correlação de scans (`amcl`), planejam rota (`planner_server`), executam (`controller_server`), orquestram com BT (`bt_navigator`).

---

## Modo SIM — testar tudo no Gazebo sem hardware

Antes de arriscar o robô real, você pode rodar o pipeline inteiro (teleop + SLAM + Nav2 click-to-go) dentro do **Gazebo Harmonic**.

A flag `--sim` troca tudo que é hardware por simulação:

| Stage | Modo real | Modo `--sim` |
|-------|-----------|--------------|
| Ponte para os motores | `mega_bridge` ↔ Arduino MEGA ↔ 2 placas hoverboard | plugin `DiffDrive` do Gazebo |
| Odometria | `odom_publisher` (média dos 4 feedbacks de roda) | plugin `DiffDrive` do Gazebo |
| `/cmd_vel → rodas` | `cmd_vel_to_wheels` + MEGA | plugin `DiffDrive` do Gazebo |
| LiDAR | `ldlidar_stl_ros2` em `/dev/lidar` | sensor `gpu_lidar` na SDF do robô |
| IMU | BNO055 (via MEGA) | (não simulado) |
| Optical flow | PMW3901 (via MEGA) | (não simulado) |
| Corpo do robô | URDF (`robot.urdf.xacro` — 4 rodas) | URDF + SDF (`husky.sdf` — 2 rodas, diff drive simplificado) |
| `/scan`, `/odom`, `/tf` | tópicos reais | via `ros_gz_bridge` (GZ → ROS) |

O servidor web, o `map_service.py` e a UI são exatamente os mesmos.

> **Sobre o modelo simulado:** ainda é uma URDF/SDF estilo "husky" com **2 rodas + caster**, herdada da versão anterior do robô. Funciona perfeitamente para validar a stack Nav2 e SLAM, mas é cinematicamente diferente do robô real de 4 rodas. Um SDF 4-wheel skid-steer pode entrar numa próxima iteração — por enquanto a divergência é intencional para manter o sim leve.

### Instalando o Gazebo e o bridge ROS↔GZ

```bash
sudo apt install \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-interfaces
```

### Onde colocar o arquivo da sala (mundo Gazebo)

**Os mundos do Gazebo ficam em `Controle_robo_web/worlds/`** (mesmo nível de `maps/`). O repositório já vem com `worlds/empty.sdf` (sala 6×6 m com paredes, chão e luz) — suficiente para testar antes de trocar pelo seu mundo.

Dois caminhos:

1. **Substituir o padrão** — jogue seu `.sdf` como `worlds/sala.sdf` (ou sobrescreva `empty.sdf`):
   ```bash
   cp ~/minha_sala_projetada.sdf Controle_robo_web/worlds/empty.sdf
   ./launch.sh --sim
   ```

2. **Passar por flag** — caminho absoluto ou relativo:
   ```bash
   ./launch.sh --sim --world=worlds/sala_projetada.sdf
   ./launch.sh --sim --world=/home/ubuntu/mundos/hangar.sdf
   ```

**Checklist do `.sdf`:**
- `<physics>` definido (ex: `dart`)
- Plugins: `Physics`, `UserCommands`, `SceneBroadcaster`, `Sensors` com `render_engine=ogre2`
- Pelo menos uma `<light>` (sol) — senão a cena fica preta e o GPU LiDAR não vê nada
- `<model name="ground_plane">` estático — senão o robô despenca
- Todos os objetos com `<collision>` — senão o LiDAR trespassa

O `worlds/empty.sdf` serve como template pronto.

### Rodando no modo SIM

```bash
./launch.sh --sim                              # sim + teleop
./launch.sh --sim --slam                       # sim + mapeamento
./launch.sh --sim --nav2                       # sim + navegação autônoma
./launch.sh --sim --world=worlds/sala.sdf      # sim com mundo customizado
```

---

## Pré-requisitos

- Ubuntu 22.04 ou 24.04
- ROS2 Jazzy (testado) instalado e no PATH
- `xacro`: `sudo apt install ros-$ROS_DISTRO-xacro`
- `robot_state_publisher`: `sudo apt install ros-$ROS_DISTRO-robot-state-publisher`
- **SLAM**: `sudo apt install ros-$ROS_DISTRO-slam-toolbox`
- **Nav2**:
  ```bash
  sudo apt install \
      ros-$ROS_DISTRO-nav2-bringup \
      ros-$ROS_DISTRO-nav2-collision-monitor \
      ros-$ROS_DISTRO-nav2-map-server \
      ros-$ROS_DISTRO-nav2-amcl
  ```
- **`python3-serial`** (instalado automaticamente pelo `start.sh`/`launch.sh` quando faltar): dependência do `mega_bridge`.
- **Modo SIM (opcional)**:
  ```bash
  sudo apt install \
      ros-$ROS_DISTRO-ros-gz \
      ros-$ROS_DISTRO-ros-gz-sim \
      ros-$ROS_DISTRO-ros-gz-bridge \
      ros-$ROS_DISTRO-ros-gz-interfaces
  ```
- **Firmware da MEGA (só hardware real)**:
  - [PlatformIO Core](https://platformio.org/install/cli): `pip install --user platformio`
  - Bibliotecas (instaladas pelo `pio` automaticamente na primeira build): `Adafruit BNO055`, `Adafruit Unified Sensor`, `Bitcraze PMW3901`, `FastLED`
- Python 3.10+

---

## Configuração inicial (uma vez)

### 1. Workspace ROS2

**Nada disso está neste repositório.** O `~/ros2_ws/` é um workspace ROS2 que você cria na máquina. Só o `robot_nav` mora aqui (em `ros2_packages/robot_nav/`), via symlink. Os outros pacotes externos precisam ser clonados antes do `colcon build`:

| Pacote | Origem | Obrigatório? |
|--------|--------|--------------|
| `robot_nav` | este repo (symlink) | **sempre** |
| `wheel_msgs` | repo externo | **sempre** — o `robot_nav` declara `<depend>wheel_msgs</depend>` |
| `ldlidar_stl_ros2` | repo externo | só no modo real (hardware) |

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 1) robot_nav — symlink (o start.sh/launch.sh também faz isso se faltar)
ln -s ~/Controle_robo_web/ros2_packages/robot_nav robot_nav

# 2) wheel_msgs — sempre obrigatório
git clone https://github.com/Richard-Haes-Ellis/wheel_msgs.git wheel_msgs

# 3) Só se for rodar no hardware real
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git ldlidar_stl_ros2

# Compila
cd ~/ros2_ws
colcon build
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

> Depois de editar qualquer arquivo em `ros2_packages/robot_nav/`, o `start.sh`/`launch.sh` detecta a mudança por hash e recompila automaticamente. Só preciso rodar `colcon build` manual se quiser controlar.

### 2. Portas USB fixas (obrigatório)

A Arduino MEGA e o LiDAR podem cair em ordem variável no boot (`/dev/ttyUSB0` ↔ `/dev/ttyACM0` etc). Os symlinks fixam por porta física:

```bash
sudo ~/Controle_robo_web/setup_udev.sh
```

O script:
1. Pede que você desplugue o LiDAR para identificar a porta da MEGA.
2. Pede que você replugue o LiDAR para identificar a porta dele.
3. Cria `/etc/udev/rules.d/99-robot-usb.rules` com os symlinks `/dev/mega` e `/dev/lidar`.

```bash
ls -la /dev/mega /dev/lidar
# Esperado:
# /dev/mega  -> ttyACM0   (Arduino oficial) ou ttyUSB0 (clone CH340)
# /dev/lidar -> ttyUSB1
```

> Se trocar o cabo de porta USB física, rode `setup_udev.sh` de novo — os symlinks dependem da porta física.

### 3. Firmware da Arduino MEGA

O firmware C++ fica em `firmware/mega_bridge/` (projeto PlatformIO). Ele:
- recebe comandos do PC pela USB (frames `0xAA 0x55 [tipo] [len] [payload] [xor]`, 230400 baud);
- envia `SerialCommand` (0xABCD) para as duas placas pelos `Serial1` (frente) e `Serial2` (trás) a 50 Hz, com **watchdog de 500 ms** (zera os motores se o PC parar de falar);
- agrega os `SerialFeedback` das duas placas e os dados de BNO055 + PMW3901 num único stream para o PC.

**Pinagem fixa:**

| Função | Pino da MEGA | Conecta em |
|--------|--------------|------------|
| Serial0 (USB) | 0/1 (reservados pelo cabo) | Notebook |
| Serial1 | 18 (TX), 19 (RX) | Placa hoverboard **FRENTE** |
| Serial2 | 16 (TX), 17 (RX) | Placa hoverboard **TRÁS** |
| Serial3 | 14 (TX), 15 (RX) | Reserva / debug |
| I²C | 20 (SDA), 21 (SCL) | BNO055 |
| SPI | 50 (MISO), 51 (MOSI), 52 (SCK) | PMW3901 (via conversor 5↔3.3 V) |
| CS do PMW3901 | 10 | PMW3901 |
| DIN WS2812 | 6 (com resistor 470 Ω) | Anel RGB |
| Relé da luz | 7 | Módulo relé |
| LED de sinalização do marco | 8 | LED externo |
| Botão de partida | 9 (pull-up interno) | Botão até GND |
| Vin / GND | jack DC ou pino | 12 V da bateria principal |

**Build e flash:**

```bash
pip install --user platformio    # uma vez por máquina

cd ~/Controle_robo_web/firmware/mega_bridge
pio run                          # compila
pio run -t upload                # compila e flasheia (com MEGA conectada)
pio device monitor -b 230400     # monitor serial pra debug
```

Cada arquivo `.cpp/.h` é comentado no diretório. O `platformio.ini` lista as bibliotecas externas — o PlatformIO baixa na primeira build.

### 4. Dependências Python

```bash
cd ~/Controle_robo_web/controle_web
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

O `start.sh` e o `launch.sh` também fazem isso automaticamente se o venv não existir, com cache por hash do `requirements.txt`.

---

## Como rodar

Todos os modos usam o mesmo `launch.sh`. O modo é passado como flag e propagado ao servidor web via `ROBOT_MODE` — a UI mostra um badge (TELEOP / SLAM / NAV2) no topo.

```bash
hostname -I    # para descobrir o IP da máquina e acessar pela rede
```

### Modo TELEOP (padrão)

Dirigir manualmente. Sobe o `nav2_collision_monitor` como camada de segurança.

```bash
cd ~/Controle_robo_web
./launch.sh
```

| # | Processo | Log |
|---|----------|-----|
| 1 | Nós do robô: `robot_state_publisher`, `mega_bridge`, `odom_publisher`, `cmd_vel_to_wheels` | `logs/robot_nodes.log` |
| 2 | LiDAR FHL-LD20 (`ldlidar_stl_ros2`) | `logs/lidar.log` |
| 3 | `nav2_collision_monitor` *(só segurança)* | `logs/nav2_collision.log` |
| 4 | Servidor web Flask + Socket.IO em `http://0.0.0.0:5000` | terminal |

### Modo SLAM — mapear a sala

```bash
./launch.sh --slam
```

Em vez do collision_monitor sobe o `slam_toolbox` (mapping online async). O painel **Mapa** da UI aparece automaticamente e cresce conforme você dirige.

**Como mapear bem:**
1. Comece parado no centro de onde quer mapear.
2. Dirija **devagar** — o SLAM precisa de tempo para casar scans consecutivos.
3. Faça retas longas, evite girar no mesmo lugar.
4. **Feche loops**: volte por onde já passou para corrigir drift.
5. Evite corredores longos com paredes lisas — sem features, o matching falha.

**Salvar:** clique em **Salvar mapa** no canto do painel. Backend chama `map_saver_cli` e grava em `maps/`:
- `maps/<nome>.yaml` — metadados
- `maps/<nome>.pgm` — grayscale do occupancy grid

### Modo NAV2 — navegação autônoma

```bash
./launch.sh --nav2                           # usa maps/sala.yaml
./launch.sh --nav2 --map=/caminho/outro.yaml # mapa customizado
```

No painel:
- **Mapa** estático com seta laranja do robô (TF `map→base_link`).
- **Clique** envia o robô pra esse ponto (action `navigate_to_pose`). Click+drag define o yaw final.
- **Trajetória planejada** em linha azul (`/plan`).
- **Toolbar de waypoints** para rotas multi-ponto. Veja [Navegação por waypoints](#navegação-por-waypoints).

Cada navegação é registrada em CSV pelo `NavMetricsCollector` — útil para tuning do Nav2. Veja [Métricas Nav2 (CSV)](#métricas-nav2-csv).

### Outras flags

```bash
./launch.sh --no-lidar              # Sobe sem LiDAR (só teleop)
./launch.sh --no-nav2               # Teleop sem collision_monitor
./launch.sh --lidar-port=/dev/lidar # Porta do LiDAR (padrão: /dev/lidar)
```

### Encerrar

`Ctrl+C` encerra tudo. O `cleanup()` do script mata a árvore inteira de filhos.

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

Combinações: `W + D` = frente + direita.

### Gamepad (PS4 / Xbox)

| Controle | Ação |
|----------|------|
| Analógico esquerdo | Movimento (linear + angular) |
| `X` (PS4) / `A` (Xbox) — segurado | Trava de emergência |
| `□` (PS4) / `X` (Xbox) | Reduz velocidade (0.8×) |
| `○` (PS4) / `B` (Xbox) | Aumenta velocidade (até 4×) |

### Velocidades

- Base: `0.3 m/s` linear, `0.5 rad/s` angular
- Multiplicador: `0.8×` a `4.0×`

---

## Sensores embarcados (BNO055 + PMW3901)

A Arduino MEGA agrega dois sensores que o robô antigo não tinha. Eles entram no ROS via `mega_bridge`:

**BNO055 — IMU 9-DOF com fusão de orientação** (I²C, endereço `0x28`):
- Tópico: `/imu/data` (`sensor_msgs/Imu`)
- O firmware lê `getQuat()`, `getVector(VECTOR_GYROSCOPE)` (convertido para rad/s) e `getVector(VECTOR_LINEARACCEL)` (m/s², sem gravidade) a 50 Hz, empacota em Q14 (quaternion) + milli (gyro/accel) e envia frame `IMU` (20 bytes).
- O `mega_bridge` decodifica e publica como `Imu` padrão com covariâncias razoáveis.
- **Uso atual**: disponível para fusão com odometria das rodas (ex.: EKF do `robot_localization`). Ainda não fundido automaticamente — `odom_publisher` segue 100% nas rodas.

**PMW3901 — sensor de fluxo óptico** (SPI, CS pino 10):
- Tópico: `/optical_flow` (`geometry_msgs/Vector3Stamped`: `x=dx`, `y=dy`, `z=quality`)
- O firmware lê o motion count acumulado a 100 Hz.
- **Uso atual**: publicado para visualização/depuração. Para virar odometria visual precisa multiplicar por altura conhecida da câmera ao chão (~3 cm pela posição no URDF) e calibrar fator pixel→metro.

Quem orquestra ambos no firmware: `firmware/mega_bridge/src/main.cpp` (loop principal), `firmware/mega_bridge/include/sensors_imu.h` e `sensors_flow.h`.

---

## Sinalização do robô (LEDs, relé, botão)

A MEGA também controla periféricos de interface humana:

| Periférico | Pino | Tópico ROS | Como usar |
|-----------|------|------------|-----------|
| Anel WS2812 (16–24 LEDs) | 6 (DIN com resistor 470 Ω) | `/leds/color` (`std_msgs/ColorRGBA`) | Publica `r,g,b` ∈ [0,1] e `a` como modo (0=fixo, 1=pisca, 2=rotação). Útil pra sinalizar chegada num waypoint ou estado do robô. |
| Relé da luz | 7 | `/light/cmd` (`std_msgs/Bool`) | `true` liga, `false` desliga. *Nota: pode ser removido no futuro — o anel WS2812 já cobre o caso de mudar de cor ao chegar num ponto.* |
| LED do marco | 8 | (controlado junto com o relé, byte 2 do frame `RELAY`) | Indicador externo de status. |
| Botão de partida | 9 (pull-up interno) | `/start_button` (`std_msgs/Bool`) | `true` enquanto pressionado. Pode ser usado pra habilitar movimentação manualmente no robô (deadman) ou iniciar uma rota de waypoints sem precisar do browser. |

Tudo é configurado por frames do protocolo (`FT_LEDS = 0x02`, `FT_RELAY = 0x03`) — ver `firmware/mega_bridge/include/protocol.h`.

---

## Navegação por waypoints

Em modo NAV2, além do click-to-go simples, a UI tem uma **toolbar de waypoints**.

**Como definir uma rota:**

1. Clique em **+ Waypoint** pra entrar em modo de adição.
2. Cada click adiciona um ponto. Click+drag define o yaw final.
3. Marque **Loop** se quiser que a rota repita.
4. Clique em **▶ Iniciar** — o `MapBridge._wp_runner` envia os goals em sequência via `navigate_to_pose`.

**Salvar e recarregar:**
- **💾 Salvar rota** grava em `maps/routes/<nome>.json`.
- **📂 Carregar** lista e restaura rotas salvas.
- Em refresh da página, waypoints definidos são restaurados automaticamente.

**Como o `_wp_runner` decide avançar:**

Usa o status terminal da action `navigate_to_pose` — não estima chegada por distância:
- `STATUS_SUCCEEDED` → avança imediatamente.
- `STATUS_ABORTED` → retenta até 2 vezes com 2 s de pausa. Após 3 falhas, pula (emite `skipped: true`).
- `STATUS_CANCELED` → sai limpo.
- Timeout de segurança de 120 s por waypoint.

Entre cada waypoint, limpa o `local_costmap` (`/local_costmap/clear_entirely_local_costmap`).

---

## Métricas Nav2 (CSV)

Em NAV2, o `NavMetricsCollector` (em `controle_web/nav_metrics.py`) registra cada navegação em CSV diário em `controle_web/logs/nav_metrics/nav_metrics_YYYYMMDD.csv`:

```
nav_id, start_ts, end_ts, duration_s, status,
start_x, start_y, end_x, end_y, end_yaw,
initial_plan_length_m, replans,
rec_backup, rec_spin, rec_wait,
distance_traveled_m, avg_linear_speed, max_linear_speed,
time_stopped_s, direction_reversals
```

**Uso típico:**
- **Tuning de DWB**: alta `time_stopped_s` ou alta contagem de `replans` em rotas curtas = controller oscilando.
- **Tuning de recoveries**: `rec_backup`/`rec_spin`/`rec_wait` muito altos = Nav2 caindo em recovery (costmap saturado, inflação alta demais).
- **Detecção de regressão**: comparar CSV antes/depois numa mesma rota mostra objetivamente se o tuning ajudou.

Tópicos consumidos:
- `/navigate_to_pose/_action/status` — início/fim de cada navegação.
- `/backup/_action/status`, `/spin/_action/status`, `/wait/_action/status` — contagem de recoveries.
- `/plan` — comprimento + replans.
- `/odom` — distância percorrida + velocidades.
- `/cmd_vel` — tempo parado + inversões.

---

## Arquitetura

### Tópicos ROS2

| Tópico / Action | Tipo | Produtor | Consumidor | Quando |
|--------|------|----------|------------|--------|
| `/cmd_vel` | `geometry_msgs/Twist` | servidor web (teleop) / `velocity_smoother` (nav2) | `cmd_vel_to_wheels` | sempre |
| `/wheel_vel_setpoints` | `wheel_msgs/WheelSpeeds` | `cmd_vel_to_wheels` | `mega_bridge` (envia pras 2 placas) | sempre |
| `/hoverboard/front/left/velocity` | `std_msgs/Float64` (RPM) | `mega_bridge` | `odom_publisher` | sempre |
| `/hoverboard/front/right/velocity` | `std_msgs/Float64` (RPM) | `mega_bridge` | `odom_publisher` | sempre |
| `/hoverboard/rear/left/velocity` | `std_msgs/Float64` (RPM) | `mega_bridge` | `odom_publisher` | sempre |
| `/hoverboard/rear/right/velocity` | `std_msgs/Float64` (RPM) | `mega_bridge` | `odom_publisher` | sempre |
| `/imu/data` | `sensor_msgs/Imu` | `mega_bridge` | (disponível, ainda não fundido em odom) | sempre |
| `/optical_flow` | `geometry_msgs/Vector3Stamped` | `mega_bridge` | (disponível, debug/futuro) | sempre |
| `/battery/front` | `sensor_msgs/BatteryState` | `mega_bridge` | (monitoramento) | sempre |
| `/battery/rear` | `sensor_msgs/BatteryState` | `mega_bridge` | (monitoramento) | sempre |
| `/start_button` | `std_msgs/Bool` | `mega_bridge` | (futuro: deadman/start de rota) | sempre |
| `/leds/color` | `std_msgs/ColorRGBA` | (cliente, futuro) | `mega_bridge` → MEGA | sempre |
| `/light/cmd` | `std_msgs/Bool` | (cliente, futuro) | `mega_bridge` → MEGA | sempre |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR driver | `slam_toolbox` / `amcl` / `voxel_layer` / `nav2_collision_monitor` | sempre |
| `/odom` | `nav_msgs/Odometry` | `odom_publisher` | `slam_toolbox` / `amcl` / `nav_metrics` | sempre |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` / `map_server` | `map_service.py` (ponte web) | slam, nav2 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | `map_service.py` (legacy) | `bt_navigator` | nav2 |
| `/plan` | `nav_msgs/Path` | `planner_server` | `map_service.py` / `nav_metrics` | nav2 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `bt_navigator` (server) | `MapBridge` / `nav_metrics` | nav2 |
| `/backup/_action/status` | `action_msgs/GoalStatusArray` | `behavior_server` | `nav_metrics` | nav2 |
| `/spin/_action/status` | `action_msgs/GoalStatusArray` | `behavior_server` | `nav_metrics` | nav2 |
| `/wait/_action/status` | `action_msgs/GoalStatusArray` | `behavior_server` | `nav_metrics` | nav2 |

**TFs publicadas:**
- `base_link → base_laser, imu_link, flow_link, 4 rodas` — static (URDF via `robot_state_publisher`)
- `odom → base_link` — dinâmica (`odom_publisher` a partir da média dos 4 RPMs)
- `map → odom` — dinâmica, em SLAM pelo `slam_toolbox`, em NAV2 pelo `amcl`

### Cinemática

Skid-steer com 4 rodas motoras: as duas do lado esquerdo (FL+RL) rodam juntas, as duas do direito (FR+RR) rodam juntas. O `cmd_vel_to_wheels` produz um único par `(left, right)` (idêntico ao caso 2-rodas) e o `mega_bridge` espelha esse par para as duas placas. O `odom_publisher` usa a média `(FL+RL)/2` e `(FR+RR)/2` antes da fórmula diff-drive — mais robusto a derrapagem isolada de uma roda.

Conversão `(left, right) → (steer, speed)` no `mega_bridge` segue a convenção do firmware NiklasFauth/hoverboard-firmware-hack:

```
speedL_meas = speed + steer
speedR_meas = speed - steer
⇒
speed = (left + right) / 2
steer = (left - right) / 2
```

### Ponte ROS2 ↔ Web para mapa e navegação

`controle_web/map_service.py` contém a classe `MapBridge`, que roda dentro do servidor Flask como thread daemon com seu próprio executor ROS2.

| Responsabilidade | Como |
|------------------|------|
| Receber o mapa | Subscribe `/map` com QoS `TRANSIENT_LOCAL` (a mensagem é *latched*). Converte `OccupancyGrid` em PNG grayscale com `numpy` + `Pillow`, flipa verticalmente (ROS y sobe, PNG y desce), base64-encoda e emite `map_update` via Socket.IO |
| Rastrear o robô | `tf2_ros.TransformListener` em polling a 10 Hz. Olha `map→base_link`, extrai x/y/yaw, emite `robot_pose` |
| Receber trajetória | Subscribe `/plan`, converte cada pose em `{x, y}`, emite `plan_update` |
| Enviar goal | Publisher em `/goal_pose` (frame `map`). Handler `nav_goal` recebe `{x, y, yaw}` do click |
| Executar waypoints | `ActionClient` de `NavigateToPose`. O `_wp_runner` envia goals em sequência e reage a SUCCEEDED/ABORTED em tempo real |
| Persistir rotas | `save_route`/`load_route`/`list_routes` em `maps/routes/<nome>.json` |
| Salvar mapa | `save_map` chama `ros2 run nav2_map_server map_saver_cli -f maps/<nome> --ros-args -p map_subscribe_transient_local:=true` |

**Rodar só em modo slam/nav2:** o `app.py` só instancia o `MapBridge` se `ROBOT_MODE in ('slam', 'nav2')`. Falha não derruba o servidor — só desabilita o painel de mapa.

### Outras pontes

| Módulo | Quando sobe | Função |
|--------|-------------|--------|
| `nav_metrics.py` | só `nav2` | Subscribe action statuses + `/plan` + `/odom` + `/cmd_vel`, grava CSV por navegação |

**Cliente (navegador):** `static/js/map.js` escuta esses eventos, mantém estado local (`mapInfo`, `mapImage`, `robotPose`, `plan`, `lastGoal`) e redesenha o canvas a ~15 Hz. Conversão click→mundo:

```js
world_x = origin_x + px_in_img * resolution
world_y = origin_y + (height-1 - py_in_img) * resolution
```

Inverte y porque o PNG foi flipado antes do envio.

### Arquivos principais

```
Controle_robo_web/
├── launch.sh                          # Launcher (--slam / --nav2 / --sim / --map=)
├── start.sh                           # Só web server (modo dev)
├── setup.sh                           # Bootstrap inicial (apt, ros2_ws, colcon build)
├── setup_udev.sh                      # Configura /dev/mega e /dev/lidar
├── firmware/
│   └── mega_bridge/                   # Firmware C++ da Arduino MEGA (PlatformIO)
│       ├── platformio.ini             # board=megaatmega2560, libs externas
│       ├── include/
│       │   ├── protocol.h             # Frames 0xAA 0x55 [tipo] [len] [payload] [xor]
│       │   ├── hoverboard.h           # SerialCommand 0xABCD + parser de SerialFeedback
│       │   ├── sensors_imu.h          # Wrapper do BNO055
│       │   ├── sensors_flow.h         # Wrapper do PMW3901
│       │   ├── leds.h                 # Anel WS2812 (FastLED)
│       │   └── io_signals.h           # Relé, LED, botão
│       └── src/                       # Implementações + main.cpp
├── maps/                              # Mapas e rotas (gitignored)
│   ├── sala.yaml / sala.pgm
│   └── routes/<nome>.json
├── worlds/                            # Mundos do Gazebo
│   ├── empty.sdf                      # Sala 6×6 m vazia (padrão)
│   └── ...
├── ros2_packages/
│   └── robot_nav/                     # Pacote ROS2 (linkado em ~/ros2_ws/src/robot_nav)
│       ├── launch/                    # robot, lidar, slam, nav2, sim, nav2_collision
│       ├── urdf/
│       │   ├── robot.urdf.xacro       # URDF do robô 4 rodas (real)
│       │   ├── husky.urdf.xacro       # URDF do robô simulado (2 rodas, sim legacy)
│       │   └── husky.sdf              # SDF do robô simulado
│       ├── config/
│       │   ├── nav2_params.yaml       # Tuning AMCL + DWB + costmaps
│       │   └── collision_monitor.yaml # Zonas de freada (modo teleop)
│       └── robot_nav/                 # Nós Python
│           ├── mega_bridge.py         # Ponte USB ↔ MEGA ↔ 2 hoverboards + sensores
│           ├── cmd_vel_to_wheels.py   # /cmd_vel → /wheel_vel_setpoints
│           └── odom_publisher.py      # 4 RPMs → /odom + TF odom→base_link
└── controle_web/
    ├── app.py                         # Servidor Flask + Socket.IO (lê ROBOT_MODE)
    ├── map_service.py                 # Ponte mapa/pose/plan + ActionClient + waypoints
    ├── nav_metrics.py                 # Coleta métricas do Nav2 em CSV
    ├── controllers/
    │   └── robot_controller.py        # ROS2Controller (publica /cmd_vel)
    ├── templates/index.html
    ├── static/
    │   ├── css/styles.css
    │   └── js/{client,gamepad,map}.js
    └── logs/                          # Logs rotativos
        └── nav_metrics/               # CSVs do NavMetricsCollector (por dia)

~/ros2_ws/src/
├── robot_nav -> ~/Controle_robo_web/ros2_packages/robot_nav  # symlink
├── ldlidar_stl_ros2/                  # Driver do LiDAR FHL-LD20 (repo separado)
└── wheel_msgs/                        # Mensagens custom das rodas (repo separado)
```

---

## Tuning do Nav2

A configuração em `ros2_packages/robot_nav/config/nav2_params.yaml` foi calibrada iterativamente.

**AMCL (localização):**

| Parâmetro | Valor | Por quê |
|-----------|-------|---------|
| `do_beamskip` | `true` | Quando feixes batem em obstáculos não-mapeados (cadeira, pessoa), AMCL ignora esses raios em vez de penalizar partículas corretas |
| `beam_skip_distance` | `0.5` | Distância máx para considerar feixe como "match" |
| `beam_skip_threshold` | `0.3` | Fração de partículas que precisa concordar para fazer skip |

**Planner (`nav2_navfn_planner`):**

| Parâmetro | Valor | Por quê |
|-----------|-------|---------|
| `tolerance` | `0.30` | Se o ponto exato estiver bloqueado, aceita rota até 30 cm do alvo |

**Goal checker:**

| Parâmetro | Valor | Por quê |
|-----------|-------|---------|
| `xy_goal_tolerance` | `0.40` | Folga para ambiente dinâmico |
| `yaw_goal_tolerance` | `0.35` | ~20°, mesma lógica |

**DWB Local Planner (controller):**

| Parâmetro | Valor | Por quê |
|-----------|-------|---------|
| `min_vel_x` | `-0.1` | Permite ré pequena em manobras apertadas |
| `BaseObstacle.scale` | `0.15` | Era 0.02 — peso 1600× menor que `PathDist`. Com 0.15 segue rota mas evita raspar |
| `Oscillation.scale` | `0.1` | Default 1.0 punia manobras legítimas em passagem apertada |
| `PathAlign.scale` / `PathDist.scale` / `GoalAlign.scale` | `32` cada | Mantém a rota planejada como prioridade média |
| `GoalDist.scale` | `24` | Atratividade do destino |
| `RotateToGoal.scale` | `32` | Rotação final para atingir yaw alvo |

**Costmap (`local_costmap` com `VoxelLayer`, só LiDAR):**

| Parâmetro | Valor | Por quê |
|-----------|-------|---------|
| `plugins` | `[voxel_layer, inflation_layer]` | `VoxelLayer` é mais conservador que `ObstacleLayer` (z explícito) |
| `observation_sources` | `scan` | Só LaserScan (não há mais câmera RGB-D) |
| `inflation_radius` | `0.25` | Para `cost_scaling_factor: 3.5` |

**Recoveries (`behavior_server`):** `BackUp` + `Spin` + `Wait` — chamados pelo BT quando o controller não consegue avançar em ~15 s.

**Observação:** valores calibrados em sim Gazebo. No hardware real pode precisar de ajustes (atrito do hoverboard, latência do LiDAR físico). Use o CSV do `NavMetricsCollector` para medir antes/depois.

---

## Logs

Em `controle_web/logs/`:

| Arquivo | Conteúdo |
|---------|----------|
| `robot_nodes.log` | `robot_state_publisher`, `mega_bridge`, `odom_publisher`, `cmd_vel_to_wheels` |
| `lidar.log` | Driver LiDAR |
| `nav2_collision.log` | Nav2 Collision Monitor (modo teleop) |
| `nav2.log` | Stack Nav2 completa (modo nav2) |
| `slam.log` | slam_toolbox (modo slam) |
| `sim.log` | Gazebo + bridges (modo sim) |
| `movements.log` | Histórico de comandos em JSON Lines |
| `movements.txt` | Histórico legível em português |
| `nav_metrics/nav_metrics_YYYYMMDD.csv` | Uma linha por navegação |

Em tempo real:

```bash
tail -f controle_web/logs/robot_nodes.log
tail -f controle_web/logs/lidar.log
```

---

## Limitações conhecidas

- **Contenção do `/cmd_vel` em NAV2.** Tanto o teleop (Socket.IO → `/cmd_vel`) quanto o `velocity_smoother` do Nav2 publicam no mesmo tópico — última mensagem vence. Funciona como "override manual" mas não é protocolo robusto. *Futuro:* `twist_mux`.
- **Drift de odometria.** O `odom_publisher` integra a média dos 4 RPMs. Mesmo com a média, drift acumula em mapeamentos longos. O `/imu/data` do BNO055 está disponível e a próxima evolução natural é rodar o `robot_localization` (EKF) fundindo wheel odom + IMU + (opcionalmente) optical flow.
- **Ambientes muito simétricos.** Corredor longo com paredes lisas: scan-matching do SLAM não encontra features suficientes. AMCL tem o mesmo problema. *Mitigação:* mapear ambientes com móveis e variação.
- **Sem câmera.** A versão atual do robô não tem câmera RGB-D — foi removida do hardware. O sistema funciona 100% com LiDAR. Voltar com câmera no futuro implica reintroduzir um `camera_bridge.py` e o pointcloud no `VoxelLayer`.
- **Bateria das placas.** Sem bateria a MEGA até liga, mas as placas de hoverboard não respondem aos `SerialCommand` — a UI segue funcionando, só o robô não anda. O `mega_bridge` continua publicando `0` em todas as velocidades.
- **Pipeline não validado end-to-end em hardware real.** O fluxo (TELEOP, SLAM, NAV2, waypoints) foi exercitado em Gazebo. Na transição para o robô 4-rodas físico ainda faltam ajustes finos: sinais de cada lado (`left_wheel_sign`/`right_wheel_sign` do `odom_publisher`), escala `linear_scale`/`angular_scale` do `cmd_vel_to_wheels`, calibração do BNO055.
- **Modelo simulado divergente.** O `husky.sdf`/`husky.urdf.xacro` usados em `--sim` ainda descrevem um diff-drive 2-rodas com caster, não o robô real 4-rodas skid-steer. Funciona para validar Nav2/SLAM, mas a dinâmica é diferente.

---

## Solução de problemas

### `/dev/mega` não existe

```bash
ls -la /dev/mega
# Se faltar:
# 1) A MEGA está plugada?
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null

# 2) udev rule criada?
cat /etc/udev/rules.d/99-robot-usb.rules
sudo ~/Controle_robo_web/setup_udev.sh

# 3) Permissões:
sudo usermod -aG dialout $USER  # logout/login depois
```

### Robô não responde aos comandos

```bash
# 1) mega_bridge está rodando?
ros2 node list | grep mega_bridge

# 2) Setpoints chegando?
ros2 topic echo /wheel_vel_setpoints --once

# 3) Feedback chegando da MEGA?
ros2 topic echo /hoverboard/front/left/velocity --once

# 4) Watchdog: se o /wheel_vel_setpoints parar por 500 ms, a MEGA zera as
#    placas automaticamente. Verifique se cmd_vel_to_wheels está publicando.
ros2 topic hz /wheel_vel_setpoints
```

Se o `mega_bridge` está publicando RPM 0 nas 4 rodas mesmo com o robô comandado, suspeite de:
- Cabos UART entre MEGA e placas trocados (TX/RX invertido)
- Placas de hoverboard com firmware diferente do esperado (NiklasFauth fork)
- Bateria descarregada

### Rodas rodam para o lado errado

O sinal varia conforme fiação das placas. Ajuste no `odom_publisher`:

```bash
ros2 param set /odom_publisher left_wheel_sign -1.0
ros2 param set /odom_publisher right_wheel_sign 1.0
```

Ou para inverter o **comando** (não só o feedback), edite `cmd_vel_to_wheels.py` ou `mega_bridge.py` (`_wheelspeeds_to_steer_speed`).

### IMU não publica `/imu/data`

```bash
# 1) BNO055 detectado?
#    Conecte ao monitor serial da MEGA (pio device monitor) — no setup() o
#    anel WS2812 fica vermelho se o BNO055 não responder no I²C.

# 2) Endereço I²C correto (BNO055_ADDRESS_A = 0x28)?
#    Se o pino ADR estiver puxado para HIGH no módulo, é 0x29 — ajuste em
#    firmware/mega_bridge/include/sensors_imu.h.

# 3) Fios SDA/SCL bons (pull-up de 10 kΩ recomendado)?
```

### LiDAR não publica `/scan`

```bash
ros2 node list | grep lidar
ros2 topic hz /scan
tail -f controle_web/logs/lidar.log
```

### Painel de mapa não aparece (modo SLAM ou NAV2)

1. Confirme o badge no topo da página: `SLAM` ou `NAV2` (não `TELEOP`).
2. Confirme que `/map` está sendo publicado: `ros2 topic echo /map --once`.
3. Olhe o log do servidor: `MapBridge` loga `[map] recebido /map (WxH)` quando o subscriber dispara.
4. Em NAV2, se o `map_server` não sobe, veja `logs/nav2.log`.

### Salvar mapa falha com "no messages received"

O `/map` é *latched* (`TRANSIENT_LOCAL`). O `MapBridge` já passa o flag certo. Se rodar manualmente:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/sala \
    --ros-args -p map_subscribe_transient_local:=true
```

### Nav2 rejeita o goal (TF timeout ou frame error)

```bash
ros2 run tf2_tools view_frames
# Esperado: map → odom → base_link → base_laser, imu_link, flow_link, 4 rodas

# Falta map → odom: o AMCL não conseguiu localizar — empurre o robô um pouco
# para dar uma pose inicial.
# Falta odom → base_link: odom_publisher não está rodando (veja logs/robot_nodes.log).
```

### Firmware da MEGA não compila

```bash
# 1) PlatformIO instalado?
pio --version

# 2) Libs externas baixaram? (deve ter feito automaticamente)
cd ~/Controle_robo_web/firmware/mega_bridge
pio pkg install

# 3) MEGA conectada para upload?
pio device list
pio run -t upload --upload-port /dev/ttyACM0
```
