# Controle Web do Robô Hoverboard

Interface web para controlar um robô hoverboard com ROS2, LiDAR FHL-LD20 e detecção de obstáculos em tempo real.

## Sumário

- [Visão geral](#visão-geral)
- [Pré-requisitos](#pré-requisitos)
- [Configuração inicial (uma vez)](#configuração-inicial-uma-vez)
  - [1. Workspace ROS2](#1-workspace-ros2)
  - [2. Portas USB fixas](#2-portas-usb-fixas-obrigatório)
  - [3. Dependências Python](#3-dependências-python)
- [Como rodar](#como-rodar)
- [Controles](#controles)
- [Arquitetura](#arquitetura)
- [Logs](#logs)
- [Solução de problemas](#solução-de-problemas)

---

## Visão geral

```
Navegador (WASD / Gamepad)
        │  Socket.IO
        ▼
  Flask + Socket.IO (porta 5000)
        │  /cmd_vel  (geometry_msgs/Twist)
        ▼
  cmd_vel_to_wheels
        │  /wheel_vel_setpoints  (wheel_msgs/WheelSpeeds)
        ▼
  ros2-hoverboard-driver  ──────►  /dev/hoverboard (USB serial)
        
  LiDAR FHL-LD20  ──────────────►  /dev/lidar (USB serial)
        │  /scan  (sensor_msgs/LaserScan)
        ▼
  obstacle_detector
        │  /tmp/obstacle_current.json  (lido pelo Flask a 5 Hz)
        ▼
  Interface web  (mapa de obstáculos em tempo real)
```

---

## Pré-requisitos

- Ubuntu 22.04 (testado) ou 24.04
- ROS2 Humble ou Jazzy instalado e no PATH
- `xacro`: `sudo apt install ros-$ROS_DISTRO-xacro`
- `robot_state_publisher`: `sudo apt install ros-$ROS_DISTRO-robot-state-publisher`
- Python 3.10+

---

## Configuração inicial (uma vez)

### 1. Workspace ROS2

```bash
# Clone e compile o workspace (se ainda não tiver feito)
cd ~/ros2_ws
colcon build
source install/setup.bash

# Adicione ao ~/.bashrc para não precisar fazer source toda vez:
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

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
| 4 | Nav2 Collision Monitor (se instalado) | `logs/nav2_collision.log` |
| 5 | Servidor web Flask + Socket.IO em `http://0.0.0.0:5000` | terminal |

Acesse de outro computador na mesma rede:

```
http://<IP_DO_ROBO>:5000
```

Para descobrir o IP:

```bash
hostname -I
```

### Opções do launch.sh

```bash
./launch.sh --no-lidar              # Sobe sem LiDAR (robô apenas)
./launch.sh --no-nav2               # Sobe sem Nav2 Collision Monitor
./launch.sh --lidar-port=/dev/lidar # Porta do LiDAR (padrão: /dev/lidar)
```

### Encerrar

`Ctrl+C` encerra todos os processos limpos.

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

## Arquitetura

### Tópicos ROS2

| Tópico | Tipo | Produtor | Consumidor |
|--------|------|----------|------------|
| `/cmd_vel` | `geometry_msgs/Twist` | servidor web | `cmd_vel_to_wheels` |
| `/wheel_vel_setpoints` | `wheel_msgs/WheelSpeeds` | `cmd_vel_to_wheels` | hoverboard driver |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR driver | `obstacle_detector` |
| `/obstacle_info` | `std_msgs/String` (JSON) | `obstacle_detector` | (monitoramento) |
| `/odom` | `nav_msgs/Odometry` | `odom_publisher` | Nav2 |

### Detecção de obstáculos

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
├── launch.sh                          # Launcher principal
├── setup_udev.sh                      # Configura portas USB fixas
└── controle_web/
    ├── app.py                         # Servidor Flask + Socket.IO
    ├── controllers/
    │   └── robot_controller.py        # ROS2Controller (publica /cmd_vel)
    ├── templates/index.html           # Interface web
    ├── static/
    │   ├── css/styles.css
    │   └── js/client.js               # Captura teclado/gamepad, envia via Socket.IO
    └── logs/                          # Logs rotativos

~/ros2_ws/src/
├── robot_nav/
│   ├── launch/robot.launch.py         # robot_state_publisher + odom + cmd_vel_to_wheels
│   ├── launch/lidar.launch.py         # LiDAR FHL-LD20
│   ├── launch/nav2_collision.launch.py
│   ├── robot_nav/cmd_vel_to_wheels.py # /cmd_vel → /wheel_vel_setpoints
│   ├── robot_nav/odom_publisher.py    # odometria pelo feedback das rodas
│   └── robot_nav/obstacle_detector.py # /scan → /tmp/obstacle_current.json
├── ros2-hoverboard-driver/            # Driver C++ do hoverboard
│   └── include/.../config.hpp        # PORT = /dev/hoverboard
└── ldlidar_stl_ros2/                  # Driver do LiDAR FHL-LD20
```

---

## Logs

Todos os logs ficam em `controle_web/logs/`:

| Arquivo | Conteúdo |
|---------|----------|
| `hoverboard_driver.log` | Saída do driver C++ (serial, erros) |
| `robot_nodes.log` | robot_state_publisher, odom, cmd_vel_to_wheels |
| `lidar.log` | Driver LiDAR |
| `obstacle_detector.log` | Detecção de obstáculos |
| `nav2_collision.log` | Nav2 Collision Monitor (se ativo) |
| `movements.log` | Histórico de comandos em JSON Lines |
| `movements.txt` | Histórico legível em português |

Para acompanhar em tempo real:

```bash
tail -f controle_web/logs/hoverboard_driver.log
tail -f controle_web/logs/lidar.log
```

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
