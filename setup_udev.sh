#!/bin/bash
# Configura nomes USB estáveis via localização física da porta USB.
# Funciona mesmo quando dois dispositivos têm o mesmo VID:PID (ex: CH340).
# Uso: sudo ./setup_udev.sh
#
# O script pede que você plugue cada dispositivo separadamente para
# identificar em qual porta física cada um está.

set -e

if [ "$EUID" -ne 0 ]; then
    echo "ERRO: Execute com sudo: sudo ./setup_udev.sh"
    exit 1
fi

RULES_FILE="/etc/udev/rules.d/99-robot-usb.rules"

get_devpath() {
    # Retorna o fragmento de path físico (ex: "1-2.1") de um ttyUSB
    local dev="$1"
    udevadm info "$dev" 2>/dev/null \
        | awk -F= '/DEVPATH/{print $2}' \
        | grep -oP '[0-9]+-[0-9]+(\.[0-9]+)*(?=:[0-9]+\.[0-9]+/ttyUSB)' \
        | head -1
}

get_vidpid() {
    local dev="$1"
    local vid pid
    vid=$(udevadm info "$dev" 2>/dev/null | awk -F= '/ID_VENDOR_ID/{print $2}')
    pid=$(udevadm info "$dev" 2>/dev/null | awk -F= '/ID_MODEL_ID/{print $2}')
    echo "${vid}:${pid}"
}

echo "========================================================"
echo "  Configuração de portas USB fixas — Hoverboard + LiDAR"
echo "========================================================"
echo ""
echo "Este script vai pedir que você plugue cada dispositivo"
echo "separadamente para identificar a porta física de cada um."
echo ""

# ---- Passo 1: identificar porta do HOVERBOARD ----
echo "PASSO 1: Hoverboard"
echo "  1. Desplugue o LiDAR (deixe só o hoverboard plugado)"
read -p "  2. Pressione ENTER quando estiver pronto..."

sleep 1
udevadm settle

PORTS_HOVER=$(ls /dev/ttyUSB* 2>/dev/null)
if [ -z "$PORTS_HOVER" ]; then
    echo "ERRO: Nenhum /dev/ttyUSB* encontrado. Verifique a conexão do hoverboard."
    exit 1
fi

echo "  Dispositivos detectados:"
for p in $PORTS_HOVER; do
    vidpid=$(get_vidpid "$p")
    path=$(get_devpath "$p")
    echo "    $p  [VID:PID=$vidpid  USB path=$path]"
done

if [ "$(echo "$PORTS_HOVER" | wc -l)" -eq 1 ]; then
    HOVER_PORT="$PORTS_HOVER"
else
    read -p "  Qual é a porta do hoverboard? (ex: /dev/ttyUSB0): " HOVER_PORT
fi

HOVER_PATH=$(get_devpath "$HOVER_PORT")
HOVER_VIDPID=$(get_vidpid "$HOVER_PORT")
HOVER_VID=$(echo "$HOVER_VIDPID" | cut -d: -f1)
HOVER_PID=$(echo "$HOVER_VIDPID" | cut -d: -f2)

echo "  Hoverboard identificado: $HOVER_PORT → path=$HOVER_PATH  VID:PID=$HOVER_VIDPID"
echo ""

# ---- Passo 2: identificar porta do LIDAR ----
echo "PASSO 2: LiDAR"
echo "  1. Plugue o LiDAR (pode deixar o hoverboard plugado também)"
read -p "  2. Pressione ENTER quando estiver pronto..."

sleep 1
udevadm settle

PORTS_ALL=$(ls /dev/ttyUSB* 2>/dev/null)
# Descobre a porta nova (que não existia antes)
LIDAR_PORT=""
for p in $PORTS_ALL; do
    if [ "$p" != "$HOVER_PORT" ]; then
        LIDAR_PORT="$p"
        break
    fi
done

if [ -z "$LIDAR_PORT" ]; then
    echo "AVISO: Não detectou porta nova. O LiDAR está na mesma porta que o hoverboard?"
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
LIDAR_VID=$(echo "$LIDAR_VIDPID" | cut -d: -f1)
LIDAR_PID=$(echo "$LIDAR_VIDPID" | cut -d: -f2)

echo "  LiDAR identificado: $LIDAR_PORT → path=$LIDAR_PATH  VID:PID=$LIDAR_VIDPID"
echo ""

# ---- Validação ----
if [ "$HOVER_PATH" = "$LIDAR_PATH" ]; then
    echo "ERRO: Hoverboard e LiDAR têm o mesmo caminho USB ($HOVER_PATH)."
    echo "  Isso não deveria acontecer. Verifique as conexões e tente novamente."
    exit 1
fi

# ---- Cria as regras udev ----
echo "Criando $RULES_FILE ..."

cat > "$RULES_FILE" << EOF
# Regras udev para nomes estáveis — hoverboard + LiDAR.
# Usa localização física da porta USB (KERNELS) para diferenciar
# dispositivos com o mesmo VID:PID (ex: chip CH340 genérico).
# Gerado por setup_udev.sh em $(date).
#
# Para regenerar: sudo ~/Controle_robo_web/setup_udev.sh

# Hoverboard driver — porta USB física: $HOVER_PATH
SUBSYSTEM=="tty", KERNELS=="$HOVER_PATH", SYMLINK+="hoverboard", MODE="0666", GROUP="dialout"

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
ls -la /dev/hoverboard /dev/lidar 2>/dev/null || echo "AVISO: Symlinks não apareceram ainda — desplugue e replugue os dispositivos."

echo ""
echo "=== Pronto! ==="
echo ""
echo "IMPORTANTE: Esses symlinks dependem da porta USB FÍSICA."
echo "Se trocar o cabo de entrada USB, rode este script novamente."
echo ""
echo "Próximo passo — recompile o driver do hoverboard:"
echo "  cd ~/ros2_ws"
echo "  colcon build --packages-select ros2-hoverboard-driver"
echo "  source install/setup.bash"
