#!/bin/bash
# Configura nomes USB estáveis via localização física da porta USB.
# Funciona mesmo quando dois dispositivos têm o mesmo VID:PID (ex: CH340).
# Uso: sudo ./setup_udev.sh
#
# Identifica:
#   /dev/mega   — Arduino MEGA 2560 (ponte para as 2 placas de hoverboard
#                 e sensores BNO055 + PMW3901)
#   /dev/lidar  — FHL-LD20

set -e

if [ "$EUID" -ne 0 ]; then
    echo "ERRO: Execute com sudo: sudo ./setup_udev.sh"
    exit 1
fi

RULES_FILE="/etc/udev/rules.d/99-robot-usb.rules"

get_devpath() {
    local dev="$1"
    udevadm info "$dev" 2>/dev/null \
        | awk -F= '/DEVPATH/{print $2}' \
        | grep -oP '[0-9]+-[0-9]+(\.[0-9]+)*(?=:[0-9]+\.[0-9]+/ttyUSB|:[0-9]+\.[0-9]+/ttyACM)' \
        | head -1
}

get_vidpid() {
    local dev="$1"
    local vid pid
    vid=$(udevadm info "$dev" 2>/dev/null | awk -F= '/ID_VENDOR_ID/{print $2}')
    pid=$(udevadm info "$dev" 2>/dev/null | awk -F= '/ID_MODEL_ID/{print $2}')
    echo "${vid}:${pid}"
}

list_tty_ports() {
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
}

echo "========================================================"
echo "  Configuração de portas USB fixas — MEGA + LiDAR"
echo "========================================================"
echo ""

# ---- Passo 1: identificar porta da MEGA ----
echo "PASSO 1: Arduino MEGA 2560"
echo "  1. Desplugue o LiDAR (deixe só a MEGA plugada)"
read -p "  2. Pressione ENTER quando estiver pronto..."

sleep 1
udevadm settle

PORTS_MEGA=$(list_tty_ports)
if [ -z "$PORTS_MEGA" ]; then
    echo "ERRO: Nenhum /dev/ttyUSB* ou /dev/ttyACM* encontrado. Verifique a conexão da MEGA."
    exit 1
fi

echo "  Dispositivos detectados:"
for p in $PORTS_MEGA; do
    vidpid=$(get_vidpid "$p")
    path=$(get_devpath "$p")
    echo "    $p  [VID:PID=$vidpid  USB path=$path]"
done

if [ "$(echo "$PORTS_MEGA" | wc -l)" -eq 1 ]; then
    MEGA_PORT="$PORTS_MEGA"
else
    read -p "  Qual é a porta da MEGA? (ex: /dev/ttyACM0): " MEGA_PORT
fi

MEGA_PATH=$(get_devpath "$MEGA_PORT")
MEGA_VIDPID=$(get_vidpid "$MEGA_PORT")

echo "  MEGA identificada: $MEGA_PORT → path=$MEGA_PATH  VID:PID=$MEGA_VIDPID"
echo ""

# ---- Passo 2: identificar porta do LIDAR ----
echo "PASSO 2: LiDAR"
echo "  1. Plugue o LiDAR (pode deixar a MEGA plugada também)"
read -p "  2. Pressione ENTER quando estiver pronto..."

sleep 1
udevadm settle

PORTS_ALL=$(list_tty_ports)
LIDAR_PORT=""
for p in $PORTS_ALL; do
    if [ "$p" != "$MEGA_PORT" ]; then
        LIDAR_PORT="$p"
        break
    fi
done

if [ -z "$LIDAR_PORT" ]; then
    echo "AVISO: Não detectou porta nova. O LiDAR está na mesma porta que a MEGA?"
    echo "  Portas disponíveis:"
    for p in $PORTS_ALL; do
        vidpid=$(get_vidpid "$p")
        path=$(get_devpath "$p")
        echo "    $p  [VID:PID=$vidpid  USB path=$path]"
    done
    read -p "  Informe manualmente a porta do LiDAR (ex: /dev/ttyUSB1): " LIDAR_PORT
fi

LIDAR_PATH=$(get_devpath "$LIDAR_PORT")
LIDAR_VIDPID=$(get_vidpid "$LIDAR_PORT")

echo "  LiDAR identificado: $LIDAR_PORT → path=$LIDAR_PATH  VID:PID=$LIDAR_VIDPID"
echo ""

# ---- Validação ----
if [ "$MEGA_PATH" = "$LIDAR_PATH" ]; then
    echo "ERRO: MEGA e LiDAR têm o mesmo caminho USB ($MEGA_PATH)."
    echo "  Isso não deveria acontecer. Verifique as conexões e tente novamente."
    exit 1
fi

# ---- Cria as regras udev ----
echo "Criando $RULES_FILE ..."

cat > "$RULES_FILE" << EOF
# Regras udev para nomes estáveis — Arduino MEGA + LiDAR.
# Usa localização física da porta USB (KERNELS) para diferenciar
# dispositivos com o mesmo VID:PID.
# Gerado por setup_udev.sh em $(date).
#
# Para regenerar: sudo ~/Controle_robo_web/setup_udev.sh

# Arduino MEGA 2560 — ponte 2 placas hoverboard + sensores
# porta USB física: $MEGA_PATH
SUBSYSTEM=="tty", KERNELS=="$MEGA_PATH", SYMLINK+="mega", MODE="0666", GROUP="dialout"

# LiDAR FHL-LD20 — porta USB física: $LIDAR_PATH
SUBSYSTEM=="tty", KERNELS=="$LIDAR_PATH", SYMLINK+="lidar", MODE="0666", GROUP="dialout"
EOF

echo ""
echo "Arquivo criado:"
cat "$RULES_FILE"

echo ""
echo "Recarregando regras udev..."
udevadm control --reload-rules
udevadm trigger
sleep 1

echo ""
echo "=== Verificando symlinks ==="
ls -la /dev/mega /dev/lidar 2>/dev/null || echo "AVISO: Symlinks não apareceram ainda — desplugue e replugue os dispositivos."

echo ""
echo "=== Pronto! ==="
echo ""
echo "IMPORTANTE: Esses symlinks dependem da porta USB FÍSICA."
echo "Se trocar o cabo de entrada USB, rode este script novamente."
echo ""
echo "Próximo passo — recompile o workspace ROS2:"
echo "  cd ~/ros2_ws"
echo "  colcon build --packages-select robot_nav wheel_msgs"
echo "  source install/setup.bash"
