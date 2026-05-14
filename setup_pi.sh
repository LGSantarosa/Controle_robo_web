#!/bin/bash
# Setup enxuto para Raspberry Pi (4/5) — só o necessário pra rodar o robô real.
#
# Diferenças vs setup.sh (notebook):
#   * NÃO instala Gazebo (ros-gz*) — Pi não roda simulação.
#   * NÃO instala Nav2 completo nem slam_toolbox por padrão. Adiciona com:
#       ./setup_pi.sh --with-nav2     (instala nav2 + slam_toolbox)
#   * Clona o driver do LiDAR FHL-LD20 (obrigatório no hardware real).
#
# Pré-requisitos: Ubuntu 24.04 arm64 com ROS2 Jazzy instalado.
#   https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
#
# Uso:
#   cd ~/Controle_robo_web
#   ./setup_pi.sh
#   sudo ./setup_udev.sh           # depois — fixa /dev/mega e /dev/lidar
#   ./launch.sh --trekking         # ou --slam / --nav2 se instalou

set -e

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$REPO_DIR"

WITH_NAV2=false
for arg in "$@"; do
    case $arg in
        --with-nav2) WITH_NAV2=true ;;
        -h|--help)
            sed -n '2,20p' "$0"
            exit 0
            ;;
    esac
done

# --- Checagens não-fatais (avisos, não bloqueia) ---

echo "=== [0/4] Checagens da máquina ==="

ARCH="$(uname -m)"
if [[ "$ARCH" != "aarch64" && "$ARCH" != "armv7l" ]]; then
    echo "  AVISO: arquitetura $ARCH (este script foi pensado pra Pi arm64)."
    echo "         Pra notebook x86_64 use ./setup.sh em vez disso."
fi

if [ -z "$ROS_DISTRO" ]; then
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    else
        echo "  ERRO: ROS2 Jazzy não encontrado em /opt/ros/jazzy/."
        echo "        Instale antes: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"
        exit 1
    fi
fi
echo "  ROS_DISTRO=$ROS_DISTRO"

# Avisa se rodando de microSD (Pi entra em throttle de I/O e colcon trava)
ROOT_DEV="$(findmnt -no SOURCE / 2>/dev/null || true)"
if echo "$ROOT_DEV" | grep -qE 'mmcblk'; then
    echo "  AVISO: / está em microSD ($ROOT_DEV). Recomendado bootar de SSD USB3 —"
    echo "         o colcon build em SD pode levar 20+ min e travar."
fi

# RAM disponível (1.5 GB livre é o mínimo recomendado pro colcon build)
FREE_MB="$(awk '/^MemAvailable:/{print int($2/1024)}' /proc/meminfo)"
if [ "$FREE_MB" -lt 1500 ]; then
    echo "  AVISO: só ${FREE_MB} MB de RAM livre. colcon pode falhar OOM."
    echo "         Considere fechar tudo antes ou habilitar zram/swap."
fi

# Cooler — não dá pra detectar fan, mas avisa sobre throttle
if [ -f /sys/devices/virtual/thermal/thermal_zone0/temp ]; then
    T_MC="$(cat /sys/devices/virtual/thermal/thermal_zone0/temp)"
    T_C=$(( T_MC / 1000 ))
    echo "  CPU temp: ${T_C}°C  (mantenha < 70°C com cooler ativo)"
fi

echo

# --- 1/5 — apt: só o essencial ---
echo "=== [1/4] Instalando dependências apt (enxuto pro modo real) ==="
APT_BASE=(
    git python3-venv python3-pip python3-serial
    "ros-${ROS_DISTRO}-xacro"
    "ros-${ROS_DISTRO}-robot-state-publisher"
    "ros-${ROS_DISTRO}-tf2-ros"
    "ros-${ROS_DISTRO}-tf2-tools"
    python3-colcon-common-extensions
)

if [ "$WITH_NAV2" = true ]; then
    echo "  --with-nav2: incluindo Nav2 + slam_toolbox"
    APT_BASE+=(
        "ros-${ROS_DISTRO}-slam-toolbox"
        "ros-${ROS_DISTRO}-nav2-bringup"
        "ros-${ROS_DISTRO}-nav2-collision-monitor"
        "ros-${ROS_DISTRO}-nav2-map-server"
        "ros-${ROS_DISTRO}-nav2-amcl"
    )
fi

sudo apt update
sudo apt install -y "${APT_BASE[@]}"

# --- 2/4 — Driver do LiDAR (única dep externa que ainda precisa ser clonada) ---
echo
echo "=== [2/4] Clonando driver do LiDAR FHL-LD20 em ros2_packages/ ==="
# Os pacotes do projeto (robot_nav, wheel_msgs, costmap_converter, teb_local_planner)
# já vivem em ros2_packages/. Só o driver do LiDAR não está versionado aqui.
LIDAR_DIR="$REPO_DIR/ros2_packages/ldlidar_stl_ros2"
if [ -d "$LIDAR_DIR" ]; then
    echo "  ldlidar_stl_ros2 já clonado"
else
    git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git "$LIDAR_DIR"
fi

# --- 3/4 — colcon build (paralelismo limitado pra não estourar RAM) ---
echo
echo "=== [3/4] colcon build (use 2 workers — Pi 4 4GB não aguenta 4 paralelos) ==="
cd "$WS_DIR"
# MAKEFLAGS pra Pi: limita também o paralelismo interno dos pacotes C++ (LiDAR driver).
export MAKEFLAGS="-j2"
colcon build --base-paths ros2_packages --symlink-install --executor sequential --parallel-workers 2

# --- 4/4 — bashrc ---
BASHRC_LINE="source $WS_DIR/install/setup.bash"
if ! grep -qxF "$BASHRC_LINE" "$HOME/.bashrc"; then
    echo "$BASHRC_LINE" >> "$HOME/.bashrc"
    echo
    echo "=== [4/4] ~/.bashrc atualizado ==="
    echo "  adicionado: $BASHRC_LINE"
else
    echo
    echo "=== [4/4] ~/.bashrc já tinha o source — ok ==="
fi

# --- Permissão na dialout (USB serial sem sudo) ---
if ! groups "$USER" | grep -q dialout; then
    echo
    echo "Adicionando $USER ao grupo dialout (USB serial sem sudo)..."
    sudo usermod -aG dialout "$USER"
    echo "  IMPORTANTE: faça logout/login pra o grupo entrar em vigor."
fi

cat <<EOF

=== Pronto! ===
Próximos passos:

  1) Conecte a Arduino MEGA e o LiDAR FHL-LD20 nas USBs da Pi.
  2) Fixe as portas (uma vez, mesmo após reboot):
        sudo $REPO_DIR/setup_udev.sh

  3) Abra um terminal NOVO (pra carregar o source do ~/.bashrc) e rode:
        cd $REPO_DIR
        ./launch.sh --trekking

  Outras opções (precisam de --with-nav2 no setup):
        ./launch.sh --slam
        ./launch.sh --nav2

Dicas pra Pi:
  * Mantenha cooler ativo (heatsink + fan). Sem isso entra em throttle.
  * Se rodar pela primeira vez no robô, comece com v_max baixa:
        ros2 launch robot_nav trekking.launch.py v_max:=0.2
  * Pra ver os logs em runtime:
        tail -f $REPO_DIR/controle_web/logs/robot_nodes.log
        tail -f $REPO_DIR/controle_web/logs/trekking.log
EOF
