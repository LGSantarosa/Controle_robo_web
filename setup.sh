#!/bin/bash
# Setup automatizado para rodar o projeto no modo --sim (Gazebo).
# Segue os passos 2 e 4 do README ("Guia rápido — do zero ao click-to-go").
#
# Pré-requisitos: Ubuntu 24.04 com ROS2 Jazzy já instalado
# (https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
#
# Uso:
#   cd ~/Controle_robo_web
#   ./setup.sh
#
# Para hardware real, rode depois: sudo ./setup_udev.sh

set -e

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$REPO_DIR"

echo "=== [1/4] Instalando dependências apt ==="
sudo apt update
sudo apt install -y \
    git python3-venv python3-pip \
    python3-colcon-common-extensions \
    python3-serial \
    ros-jazzy-xacro ros-jazzy-robot-state-publisher \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-map-server ros-jazzy-nav2-amcl \
    ros-jazzy-ros-gz ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces

echo
echo "=== [2/4] Instalando PlatformIO (firmware MEGA) ==="
# PlatformIO é a toolchain pra compilar/flashear o firmware C++ da MEGA.
# Instala no --user pra não exigir sudo nem poluir o venv do servidor web.
# Idempotente: pula se 'pio' já está disponível.
if command -v pio >/dev/null 2>&1; then
    echo "  PlatformIO já instalado: $(pio --version 2>&1 | head -1)"
else
    pip install --user --upgrade platformio
    if ! command -v pio >/dev/null 2>&1; then
        case ":$PATH:" in
            *":$HOME/.local/bin:"*) ;;
            *)
                echo "  AVISO: ~/.local/bin não está no PATH. Adicione ao seu ~/.bashrc:"
                echo "         export PATH=\"\$HOME/.local/bin:\$PATH\""
                ;;
        esac
    fi
fi

echo
echo "=== [3/4] Compilando workspace em $WS_DIR ==="
# Os pacotes ROS2 vivem em ros2_packages/ (robot_nav, wheel_msgs,
# costmap_converter, teb_local_planner). colcon descobre via --base-paths.
cd "$WS_DIR"
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2_packages --symlink-install

BASHRC_LINE="source $WS_DIR/install/setup.bash"
if ! grep -qxF "$BASHRC_LINE" "$HOME/.bashrc"; then
    echo "$BASHRC_LINE" >> "$HOME/.bashrc"
    echo "  adicionado ao ~/.bashrc: $BASHRC_LINE"
fi

echo
echo "=== [4/4] Pronto! ==="
echo "Abra um terminal novo (ou rode: source $WS_DIR/install/setup.bash)"
echo "e teste com:"
echo "  cd $REPO_DIR && ./launch.sh --sim"
echo
echo "Para hardware real:"
echo "  1) Flashear MEGA:  cd $REPO_DIR/firmware/mega_bridge && pio run -t upload"
echo "  2) Fixar USBs:     sudo $REPO_DIR/setup_udev.sh"
echo "  3) Testar transmissão MEGA↔placa:"
echo "                     python3 $REPO_DIR/firmware/mega_bridge/tools/test_mega.py --front-only"
