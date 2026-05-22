#!/bin/bash
# Pareamento guiado do DualShock 4 (PS4) sem tela, via bluetoothctl.
# PLANO_HEADLESS_2026-05-22 §4.2.
#
# Depois de pareado+confiado, ligar o controle reconecta sozinho — o joy_node
# (robot.launch.py) lê /dev/input/js0 e o teleop_twist_joy publica em joy_vel.
#
# Uso:
#   ./pair-ps4.sh
#   (coloque o DS4 em modo pareamento: segure SHARE + PS até a barra piscar rápido)

set -e

if ! command -v bluetoothctl >/dev/null 2>&1; then
    echo "ERRO: bluetoothctl não encontrado. Instale com: sudo apt install -y bluez"
    exit 1
fi

echo "=== Pareamento do DualShock 4 ==="
echo "1) Segure SHARE + PS no controle até a barra de luz piscar RÁPIDO (modo pareamento)."
read -r -p "Pronto? Enter pra começar a busca... " _

# Liga adaptador, agente e scan por ~8 s pra listar os dispositivos.
bluetoothctl --timeout 8 <<'EOF'
power on
agent on
default-agent
scan on
EOF

echo
echo "2) Dispositivos encontrados (procure 'Wireless Controller'):"
bluetoothctl devices | grep -iE "controller|wireless|dualshock|sony" || bluetoothctl devices

echo
read -r -p "3) Cole o MAC do controle (ex.: AA:BB:CC:DD:EE:FF): " MAC
if ! [[ "$MAC" =~ ^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$ ]]; then
    echo "ERRO: '$MAC' não parece um MAC válido."
    exit 1
fi

echo
echo "Pareando $MAC ..."
# trust = reconecta automático no boot (o pulo do gato).
bluetoothctl <<EOF
pair $MAC
trust $MAC
connect $MAC
EOF

echo
echo "Conferindo /dev/input/js0 ..."
if [ -e /dev/input/js0 ]; then
    echo "  OK — /dev/input/js0 presente. Controle pronto."
else
    echo "  AVISO: /dev/input/js0 ainda não apareceu. Ligue o controle (botão PS) e"
    echo "         confira de novo: ls /dev/input/js0"
fi

echo
echo "Pronto. A partir de agora, ligar o controle (botão PS) reconecta sozinho."
echo "Teste os eixos com:  ros2 run joy joy_enumerate_devices   (ou: jstest /dev/input/js0)"
