#!/bin/bash
# Instala o Nav2 Collision Monitor e dependências para integração com LiDAR.
# Execute uma vez: sudo ./install_nav2.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Instalando Nav2 e dependências..."

sudo apt install -y \
    ros-jazzy-nav2-collision-monitor \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-msgs \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-tf2-tools \
    ros-jazzy-geometry-msgs

echo ""
echo "Instalação concluída!"
echo "Agora compile o workspace:"
echo "  cd $SCRIPT_DIR && ./setup.sh"
