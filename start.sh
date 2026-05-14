#!/bin/bash
# Inicia o servidor web de controle do robô com ambiente ROS2 configurado.
# Uso: ./start.sh
#
# Faz incrementalmente (cache por hash):
#   - symlink do robot_nav no workspace ROS2 (se faltar)
#   - colcon build de robot_nav + wheel_msgs (se algum arquivo mudou)
#   - instala python3-serial (necessário pelo mega_bridge) se faltar
#   - avisa se /dev/mega não está disponível
#   - venv Python do servidor web

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR"
ROS2_SETUP="$WS_DIR/install/setup.bash"

# --- [1/4] colcon build incremental (hash dos fontes do robot_nav) ---
# Pacotes vivem em ros2_packages/ — colcon descobre via --base-paths.
PKG_STAMP="$WS_DIR/install/.robot_nav.sha1"
PKG_HASH=$(find "$SCRIPT_DIR/ros2_packages/robot_nav" -type f \
    \( -name "*.py" -o -name "*.xml" -o -name "*.xacro" -o -name "*.yaml" \) \
    -not -path "*/build/*" -not -path "*/install/*" \
    2>/dev/null | sort | xargs sha1sum 2>/dev/null | sha1sum | awk '{print $1}')

if [ ! -f "$ROS2_SETUP" ] \
   || [ ! -f "$PKG_STAMP" ] \
   || [ "$(cat "$PKG_STAMP" 2>/dev/null)" != "$PKG_HASH" ]; then
    if [ -z "$ROS_DISTRO" ]; then
        for d in /opt/ros/*/setup.bash; do
            [ -f "$d" ] && source "$d" && break
        done
    fi
    if ! command -v colcon >/dev/null 2>&1; then
        echo "ERRO: colcon não encontrado. Instale: sudo apt install python3-colcon-common-extensions"
        exit 1
    fi
    if [ ! -f "$ROS2_SETUP" ]; then
        echo "[1/4] Compilando workspace ROS2 (primeira build — todos os pacotes)..."
        (cd "$WS_DIR" && colcon build --base-paths ros2_packages --symlink-install)
    else
        echo "[1/4] Compilando workspace ROS2 (mudanças detectadas)..."
        (cd "$WS_DIR" && colcon build --base-paths ros2_packages --symlink-install --packages-select robot_nav wheel_msgs)
    fi
    echo "$PKG_HASH" > "$PKG_STAMP"
fi

source "$ROS2_SETUP"

# --- [2/4] python3-serial (dependência do mega_bridge) ---
if ! python3 -c "import serial" 2>/dev/null; then
    echo "[2/4] Instalando python3-serial (sudo)..."
    sudo apt install -y python3-serial
fi

# --- [3/4] Aviso de /dev/mega ---
if [ ! -e /dev/mega ]; then
    echo "[3/4] AVISO: /dev/mega não encontrado."
    echo "      Plugue a Arduino MEGA e rode: sudo $SCRIPT_DIR/setup_udev.sh"
fi

# --- [4/4] venv Python + servidor web ---
cd "$SCRIPT_DIR/controle_web"

VENV_DIR=".venv"
REQ_FILE="requirements.txt"
REQ_STAMP="$VENV_DIR/.requirements.sha1"

if [ ! -f "$VENV_DIR/bin/activate" ]; then
    echo "[4/4] Criando venv em $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
fi

REQ_HASH=$(sha1sum "$REQ_FILE" | awk '{print $1}')
if [ ! -f "$REQ_STAMP" ] || [ "$(cat "$REQ_STAMP" 2>/dev/null)" != "$REQ_HASH" ]; then
    echo "Instalando dependências Python..."
    "$VENV_DIR/bin/pip" install --upgrade pip >/dev/null
    "$VENV_DIR/bin/pip" install -r "$REQ_FILE"
    echo "$REQ_HASH" > "$REQ_STAMP"
fi

source "$VENV_DIR/bin/activate"

echo ""
echo "Iniciando servidor de controle do robô em http://0.0.0.0:5000"
python3 app.py
