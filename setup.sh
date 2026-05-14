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

echo "=== [1/3] Instalando dependências apt ==="
sudo apt update
sudo apt install -y \
    git python3-venv python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-xacro ros-jazzy-robot-state-publisher \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-bringup ros-jazzy-nav2-collision-monitor \
    ros-jazzy-nav2-map-server ros-jazzy-nav2-amcl \
    ros-jazzy-ros-gz ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces

echo
echo "=== [2/3] Compilando workspace em $WS_DIR ==="
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
echo "=== [3/3] Pronto! ==="
echo "Abra um terminal novo (ou rode: source $WS_DIR/install/setup.bash)"
echo "e teste com:"
echo "  cd $REPO_DIR && ./launch.sh --sim"
