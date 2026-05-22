#!/bin/bash
# Tooling de operação headless (acessar/operar o robô de outro PC).
# Chamado por setup.sh e setup_pi.sh. PLANO_HEADLESS_2026-05-22 §4.
# Idempotente — pode rodar de novo sem problema.
#
# Deixa a máquina pronta pra ser TANTO o robô (anuncia robot.local, aceita SSH,
# roda a stack no tmux) QUANTO o "outro PC" (resolve robot.local, robot-connect).
#
# Espera REPO_DIR no ambiente; senão deduz do caminho deste script.
set -e

REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/.." && pwd)}"

echo "=== Headless: SSH + mDNS (robot.local) + tmux + bluez ==="
# openssh-server : aceita SSH de outro PC (lado robô).
# avahi-daemon   : anuncia este host como <hostname>.local na LAN (lado robô).
# libnss-mdns    : resolve *.local sem caçar IP (lado cliente; o postinst já
#                  insere o mdns no /etc/nsswitch.conf).
# tmux           : a stack sobrevive à queda do SSH (detach/attach).
# bluez          : pareamento do DualShock 4 (pair-ps4.sh).
sudo apt install -y openssh-server avahi-daemon libnss-mdns tmux bluez
sudo systemctl enable --now ssh avahi-daemon

# Atalhos no PATH. Symlinks pro repo: editar bin/ reflete na hora, sem reinstalar.
for tool in robot-up robot-key robot-connect; do
    if [ -f "$REPO_DIR/bin/$tool" ]; then
        chmod +x "$REPO_DIR/bin/$tool"
        sudo ln -sf "$REPO_DIR/bin/$tool" "/usr/local/bin/$tool"
        echo "  /usr/local/bin/$tool -> $REPO_DIR/bin/$tool"
    fi
done

HOSTNAME_SHORT="$(hostname)"
echo
echo "  Pronto. Este host se anuncia como '${HOSTNAME_SHORT}.local'."
echo "  Do OUTRO PC (com este mesmo setup):  robot-connect <modo>"
echo "  Como robô: pareie o PS4 com  ./pair-ps4.sh"
