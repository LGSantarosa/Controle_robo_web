#!/bin/bash
# Setup automatizado para rodar o projeto no modo --sim (Gazebo).
# Segue os passos 2 e 4 do README ("Guia rápido — do zero ao click-to-go").
#
# Pré-requisitos: Ubuntu 24.04 com ROS2 Jazzy já instalado
# (https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
#
# Uso:
#   cd ~/Workspace/Controle_robo_web
#   ./setup.sh
#
# Para hardware real, rode depois: sudo ./setup_udev.sh

set -e

case "${1:-}" in
    --help|-h)
        sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
esac

# NÃO rode com sudo: o pipx instala o PlatformIO no $HOME do usuário,
# se rodar como root o 'pio' acaba em /root/.local/bin e fica invisível
# pro seu shell normal. O script pede sudo internamente só pro 'apt'.
if [ "$EUID" -eq 0 ]; then
    echo "ERRO: não rode este script com sudo." >&2
    echo "       Rode assim:  ./setup.sh" >&2
    echo "       (o script vai pedir senha do sudo quando precisar)" >&2
    exit 1
fi

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$REPO_DIR"

echo "=== [1/4] Instalando dependências apt ==="
sudo apt update
sudo apt install -y \
    git python3-venv python3-pip pipx \
    python3-colcon-common-extensions \
    python3-serial \
    ros-jazzy-xacro ros-jazzy-robot-state-publisher \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-map-server ros-jazzy-nav2-amcl \
    ros-jazzy-ros-gz ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces \
    ros-jazzy-joy ros-jazzy-teleop-twist-joy \
    ros-jazzy-teleop-twist-keyboard ros-jazzy-twist-mux

# Se houver platformio velho do apt (4.3.4 é incompatível com Click do 24.04,
# quebra com 'resultcallback' AttributeError), remove antes de instalar via pipx.
if dpkg -l platformio >/dev/null 2>&1; then
    echo "  Removendo platformio antigo do apt (incompatível)..."
    sudo apt remove --purge -y platformio
    hash -r
fi

echo
echo "=== [2/4] Instalando PlatformIO (firmware MEGA) ==="
# Usa pipx (recomendação oficial) pra isolar o PlatformIO do Python do sistema.
# Pacote 'pipx' já foi instalado no passo [1/4].
pipx ensurepath >/dev/null 2>&1 || true
export PATH="$HOME/.local/bin:$PATH"

# Detecta pio válido (não o apt quebrado). Reinstala se 'pio --version' falhar.
if command -v pio >/dev/null 2>&1 && pio --version >/dev/null 2>&1; then
    echo "  PlatformIO já instalado: $(pio --version 2>&1 | head -1)"
else
    pipx install --force platformio
fi

if ! pio --version >/dev/null 2>&1; then
    echo "  AVISO: 'pio' ainda não funciona. Abra um terminal novo e rode 'pio --version'."
    echo "         Se faltar PATH, adicione ao ~/.bashrc:"
    echo "         export PATH=\"\$HOME/.local/bin:\$PATH\""
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
echo "=== [4/4] Operação headless (acesso/operação de outro PC) ==="
# Deixa esta máquina pronta pra ser robô (anuncia .local, SSH, tmux) E/OU
# cliente (resolve robo-desktop.local, comando robot-connect). PLANO_HEADLESS §4.
REPO_DIR="$REPO_DIR" bash "$REPO_DIR/scripts/setup_headless.sh"

echo
echo "=== Pronto! ==="
echo "Abra um terminal novo (ou rode: source $WS_DIR/install/setup.bash)"
echo "e teste com:"
echo "  cd $REPO_DIR && ./launch.sh --sim"
echo
echo "Operar o robô a partir DESTE PC (robô já ligado e configurado):"
echo "  robot-connect slam            # conecta por SSH + sobe a stack no tmux"
echo "  robot-connect nav2 --map=maps/sala.yaml"
echo "  (se robo-desktop.local não resolver:  ROBOT_HOST=<ip> robot-connect slam)"
echo
echo "Para hardware real:"
echo "  1) Flashear MEGA:  cd $REPO_DIR/firmware/mega_bridge && pio run -t upload"
echo "  2) Fixar USBs:     sudo $REPO_DIR/setup_udev.sh"
echo "  3) Testar transmissão MEGA↔placa:"
echo "                     python3 $REPO_DIR/firmware/mega_bridge/tools/test_mega.py --front-only"
