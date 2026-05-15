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

case "${1:-}" in
    --help|-h)
        sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
esac

if [ "$EUID" -ne 0 ]; then
    echo "ERRO: Execute com sudo: sudo ./setup_udev.sh"
    exit 1
fi

RULES_FILE="/etc/udev/rules.d/99-robot-usb.rules"

get_devpath() {
    # Resolve a porta USB FÍSICA (ex: "1-7" ou "1-2.3") andando pela árvore
    # de pais em /sys. Funciona pra ttyUSB* (FTDI/CH340) e ttyACM*
    # (CDC-ACM, Arduino genuíno) sem depender de awk avançado.
    local dev="$1"
    local devname
    devname=$(basename "$dev")
    local syspath
    syspath=$(readlink -f "/sys/class/tty/$devname/device" 2>/dev/null)
    while [ -n "$syspath" ] && [ "$syspath" != "/" ]; do
        local base
        base=$(basename "$syspath")
        if [[ "$base" =~ ^[0-9]+-[0-9]+(\.[0-9]+)*$ ]]; then
            echo "$base"
            return 0
        fi
        syspath=$(dirname "$syspath")
    done
}

get_vidpid() {
    local dev="$1"
    local vid pid
    vid=$(udevadm info "$dev" 2>/dev/null | awk -F= '/ID_VENDOR_ID/{print $2}')
    pid=$(udevadm info "$dev" 2>/dev/null | awk -F= '/ID_MODEL_ID/{print $2}')
    echo "${vid}:${pid}"
}

list_tty_ports() {
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
}

settle_udev() {
    udevadm settle --timeout=5 >/dev/null 2>&1 || true
}

echo "========================================================"
echo "  Configuração de portas USB fixas — MEGA + LiDAR"
echo "========================================================"
echo ""

# Daqui pra baixo o script faz I/O com hardware imprevisível (udev, USB).
# Desligo o 'set -e' pra não morrer silencioso em substituição de comando.
set +e

# ---- Passo 1: identificar porta do LIDAR (opcional) ----
# A MEGA é obrigatória, o LiDAR é opcional — por isso o LiDAR vem primeiro:
# se o usuário não tiver LiDAR, ele passa direto e a MEGA é detectada no passo 2.
echo "PASSO 1: LiDAR FHL-LD20 (opcional)"
echo "  Se TEM LiDAR: plugue SÓ o LiDAR (a MEGA fica desplugada por enquanto)."
echo "  Se NÃO TEM LiDAR: deixe tudo desplugado e siga adiante."
read -r -p "  Pressione ENTER quando estiver pronto... " _DUMMY

echo "  Aguardando 1s e estabilizando eventos udev..."
sleep 1
settle_udev

PORTS_LIDAR=$(list_tty_ports)
LIDAR_PORT=""
LIDAR_PATH=""

if [ -z "$PORTS_LIDAR" ]; then
    echo "  Nenhum dispositivo detectado — assumindo que não há LiDAR."
else
    echo "  Dispositivos detectados:"
    for p in $PORTS_LIDAR; do
        vidpid=$(get_vidpid "$p")
        path=$(get_devpath "$p")
        echo "    $p  [VID:PID=$vidpid  USB path=$path]"
    done
    if [ "$(echo "$PORTS_LIDAR" | wc -l)" -eq 1 ]; then
        LIDAR_PORT="$PORTS_LIDAR"
    else
        read -r -p "  Qual é a porta do LiDAR? (ex: /dev/ttyUSB0): " LIDAR_PORT
    fi
    LIDAR_PATH=$(get_devpath "$LIDAR_PORT")
    LIDAR_VIDPID=$(get_vidpid "$LIDAR_PORT")
    echo "  LiDAR identificado: $LIDAR_PORT → path=$LIDAR_PATH  VID:PID=$LIDAR_VIDPID"
fi
echo ""

# ---- Passo 2: identificar porta da MEGA (obrigatória) ----
echo "PASSO 2: Arduino MEGA 2560 (obrigatória)"
echo "  Plugue a MEGA agora (o LiDAR pode permanecer plugado)."
read -r -p "  Pressione ENTER quando estiver pronto... " _DUMMY

echo "  Aguardando 1s e estabilizando eventos udev..."
sleep 1
settle_udev

PORTS_ALL=$(list_tty_ports)
if [ -z "$PORTS_ALL" ]; then
    echo "ERRO: Nenhum /dev/ttyUSB* ou /dev/ttyACM* encontrado. Verifique a conexão da MEGA."
    exit 1
fi

# Lista portas candidatas (todas exceto LiDAR). Se houver mais de uma
# (modem 3G/4G, debugger, etc.) pergunta interativamente em vez de
# pegar a primeira "qualquer porta diferente do LiDAR".
CAND_MEGA=""
for p in $PORTS_ALL; do
    if [ "$p" != "$LIDAR_PORT" ]; then
        CAND_MEGA="$CAND_MEGA $p"
    fi
done
CAND_MEGA=$(echo "$CAND_MEGA" | xargs)  # trim
CAND_COUNT=$(echo "$CAND_MEGA" | wc -w)

if [ "$CAND_COUNT" -eq 0 ]; then
    echo "ERRO: Não detectei nenhuma porta nova além do LiDAR. A MEGA está plugada?"
    exit 1
elif [ "$CAND_COUNT" -eq 1 ]; then
    MEGA_PORT="$CAND_MEGA"
else
    echo "  Múltiplas portas candidatas detectadas:"
    for p in $CAND_MEGA; do
        vidpid=$(get_vidpid "$p")
        path=$(get_devpath "$p")
        echo "    $p  [VID:PID=$vidpid  USB path=$path]"
    done
    read -r -p "  Qual é a porta da MEGA? (ex: /dev/ttyACM0): " MEGA_PORT
fi

MEGA_PATH=$(get_devpath "$MEGA_PORT")
MEGA_VIDPID=$(get_vidpid "$MEGA_PORT")

echo "  MEGA identificada: $MEGA_PORT → path=$MEGA_PATH  VID:PID=$MEGA_VIDPID"
echo ""

# Sem o USB path a regra udev vira `KERNELS==""` (casa com nada, ou pior,
# casa com tudo dependendo do kernel). Aborta antes de gravar lixo em /etc.
if [ -z "$MEGA_PATH" ]; then
    echo "ERRO: não consegui extrair o USB path da MEGA ($MEGA_PORT). Aborto."
    exit 1
fi
if [ -n "$LIDAR_PORT" ] && [ -z "$LIDAR_PATH" ]; then
    echo "ERRO: não consegui extrair o USB path do LiDAR ($LIDAR_PORT). Aborto."
    exit 1
fi

# Reativa set -e pra escrita do arquivo / udevadm reload (essas devem dar certo).
set -e

# ---- Validação ----
if [ -n "$LIDAR_PATH" ] && [ "$MEGA_PATH" = "$LIDAR_PATH" ]; then
    echo "ERRO: MEGA e LiDAR têm o mesmo caminho USB ($MEGA_PATH)."
    echo "  Isso não deveria acontecer. Verifique as conexões e tente novamente."
    exit 1
fi

# ---- Cria as regras udev ----
echo "Criando $RULES_FILE ..."

{
    cat << EOF
# Regras udev para nomes estáveis — Arduino MEGA + LiDAR.
# Usa localização física da porta USB (KERNELS) para diferenciar
# dispositivos com o mesmo VID:PID.
# Gerado por setup_udev.sh em $(date).
#
# Para regenerar: sudo ~/Controle_robo_web/setup_udev.sh

# Arduino MEGA 2560 — ponte 2 placas hoverboard + sensores
# porta USB física: $MEGA_PATH
SUBSYSTEM=="tty", KERNELS=="$MEGA_PATH", SYMLINK+="mega", MODE="0660", GROUP="dialout"
EOF

    if [ -n "$LIDAR_PATH" ]; then
        cat << EOF

# LiDAR FHL-LD20 — porta USB física: $LIDAR_PATH
SUBSYSTEM=="tty", KERNELS=="$LIDAR_PATH", SYMLINK+="lidar", MODE="0660", GROUP="dialout"
EOF
    fi
} > "$RULES_FILE"

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
EXPECTED_LINKS="/dev/mega"
[ -n "$LIDAR_PATH" ] && EXPECTED_LINKS="$EXPECTED_LINKS /dev/lidar"
if ! ls -la $EXPECTED_LINKS 2>/dev/null; then
    echo "AVISO: Symlinks não apareceram ainda — desplugue e replugue os dispositivos."
fi

echo ""
echo "=== Pronto! ==="
echo ""
echo "IMPORTANTE: Esses symlinks dependem da porta USB FÍSICA."
echo "Se trocar o cabo de entrada USB, rode este script novamente."
echo ""
echo "Próximo passo — recompile o workspace ROS2:"
echo "  cd ~/Workspace/Controle_robo_web"
echo "  colcon build --base-paths ros2_packages --symlink-install --packages-select robot_nav wheel_msgs"
echo "  source install/setup.bash"
