#!/bin/bash
# Pareamento automatizado do controle PS4 no Linux, sem tela.
# PLANO_HEADLESS_2026-05-22 §4.2.
#
# Por que não é trivial: o PS4 + BlueZ tem 6 pegadinhas conhecidas que afundam
# o pareamento se não forem tratadas ANTES de chamar `bluetoothctl pair`:
#
#   1) ERTM ligado  → PS4 não fala ERTM, conexão HID é derrubada (sintoma:
#                     "Connected: yes" seguido de "Connected: no" em ~1 s).
#   2) ControllerMode dual (BR/EDR + LE) → BlueZ tenta LE no PS4 (que é só
#                     clássico), falha e mata a conexão. Força BR/EDR-only.
#   3) ClassicBondedOnly=true (default) → bluetoothd rejeita HID de devices
#                     não-bonded ("Rejected !bonded"); pair conecta em BT mas
#                     /dev/input/jsN nunca aparece. Força false.
#   4) HID drivers ausentes → BT conecta mas sem hid_playstation/hid_sony +
#                     joydev, /dev/input/jsN nunca materializa. Modprobe antes
#                     do pair garante que o nó apareça quando HID profile subir.
#   5) Agent NoInputNoOutput → pair "just works" sem link key persistente;
#                     Bonded=no, pareamento é descartado no primeiro disconnect.
#                     Usa KeyboardDisplay (força SSP com bonding completo).
#   6) bluetoothctl assíncrono → heredoc fecha stdin antes do pair completar.
#                     Usa FIFO pra manter agent vivo + one-shot pair + polling.
#
# Fixes 1, 2, 3 vivem em scripts/_bluez_fixes.sh (compartilhados com
# setup_headless.sh). Os outros 3 são lógicos do pareamento e ficam aqui.
# Este script resolve os 6, limpa pareamento velho e espera /dev/input/js0
# materializar de fato antes de declarar sucesso.
#
# Uso:
#   ./pair-ps4.sh                # interativo (pede Enters)
#   ./pair-ps4.sh --no-prompt    # non-interactive (uso via SSH); sudo deve estar cacheado
#
# Modo pareamento do PS4:
#   - Segure PS por ~10 s até a barra APAGAR completamente.
#   - Segure SHARE + PS por 5 s até a barra piscar RÁPIDO (2 flashes/s).
#   - Solte; a barra continua piscando rápido sozinha.

set -e

# ------------------------------------------------------------------
# Args
# ------------------------------------------------------------------
NO_PROMPT=0
for arg in "$@"; do
    case "$arg" in
        --no-prompt) NO_PROMPT=1 ;;
        -h|--help)
            sed -n '2,25p' "$0" | sed 's/^# \?//'
            exit 0 ;;
    esac
done

_read_or_skip() {
    # $1 = mensagem
    if [ "$NO_PROMPT" = 1 ]; then
        return
    fi
    read -r -p "$1" _
}

# Em --no-prompt o sudo precisa estar pré-cacheado (sem TTY pra prompt de senha).
if [ "$NO_PROMPT" = 1 ] && ! sudo -n true 2>/dev/null; then
    echo "ERRO: --no-prompt exige sudo cacheado. Rode 'sudo -v' antes (ou via SSH:"
    echo "      ssh <user>@<host> sudo -v)."
    exit 1
fi

# ------------------------------------------------------------------
# 0) Pré-checagens & fixes de sistema (idempotente)
# ------------------------------------------------------------------

if ! command -v bluetoothctl >/dev/null 2>&1; then
    echo "ERRO: bluetoothctl não encontrado. Instale com: sudo apt install -y bluez"
    exit 1
fi

# --- Fixes persistentes do BlueZ (compartilhados com scripts/setup_headless.sh) ---
SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/_bluez_fixes.sh
source "$SELF_DIR/scripts/_bluez_fixes.sh"
bluez_apply_persistent_fixes

# Runtime do ERTM: o config persistente só vale após reload do módulo. Tenta
# trocar via sysfs; se kernel ignorar, exige reboot.
if ! bluez_apply_ertm_runtime; then
    echo "ERRO: ERTM continua ligado e o módulo não aceita troca em runtime."
    echo "      Reinicie pra carregar /etc/modprobe.d/bluetooth-disable-ertm.conf:"
    echo "        sudo reboot"
    echo "      Depois rode ./pair-ps4.sh de novo."
    exit 1
fi

if [ "$BLUEZ_NEED_RESTART" = 1 ]; then
    echo "→ Reiniciando bluetoothd pra aplicar configs..."
    sudo systemctl restart bluetooth
    sleep 2
fi

# Destrava rfkill e garante adaptador ligado.
bluez_unblock_rfkill
sleep 1

if ! bluetoothctl show 2>/dev/null | grep -q "Powered: yes"; then
    echo "→ Ligando adaptador BT..."
    bluetoothctl power on >/dev/null 2>&1 || true
    sleep 1
fi

if ! bluetoothctl show 2>/dev/null | grep -q "Powered: yes"; then
    echo "ERRO: adaptador BT continua desligado. Diagnóstico:"
    rfkill list bluetooth
    bluetoothctl show | head -10
    exit 1
fi

# --- Fix 4: drivers HID do PS4 carregados ---
# hid_playstation é o driver moderno (kernel 5.12+); hid_sony é o fallback.
# Sem nenhum dos dois, BT aceita o controle mas /dev/input/js0 nunca aparece.
if ! lsmod | grep -qE '^(hid_playstation|hid_sony)'; then
    echo "→ Carregando driver hid_playstation..."
    sudo modprobe hid_playstation 2>/dev/null || sudo modprobe hid_sony 2>/dev/null || true
fi
# joydev cria o nó /dev/input/jsN a partir do HID.
if ! lsmod | grep -q '^joydev'; then
    sudo modprobe joydev 2>/dev/null || true
fi

echo "✓ Sistema pronto (ERTM=Y, BR/EDR-only, rfkill OK, adapter ON, HID drivers OK)"
echo

# ------------------------------------------------------------------
# 1) Coloca o usuário em modo pareamento
# ------------------------------------------------------------------

cat <<'EOF'
=== Pareamento do controle PS4 ===

Coloque o controle em modo pareamento AGORA:
  1) Segure PS por ~10s até a barra de luz APAGAR completamente.
  2) Segure SHARE + PS por 5s até a barra piscar RÁPIDO (2 flashes/s).
  3) Solte os dois botões — deve continuar piscando rápido sozinho.

EOF
_read_or_skip "Pronto? Enter pra começar... "

# ------------------------------------------------------------------
# 2) Limpa pareamento velho do MESMO controle (se houver)
# ------------------------------------------------------------------

# Filtra por Modalias=usb:v054C* (vendor Sony) pra não apagar Xbox/PS5/genéricos
# que também chamam "Wireless Controller". Se nenhum bater, deixa quieto.
OLD_MAC=""
while read -r mac; do
    [ -z "$mac" ] && continue
    if bluetoothctl info "$mac" 2>/dev/null | grep -qE "Modalias: usb:v054C|Manufacturer: Sony"; then
        OLD_MAC="$mac"
        break
    fi
done < <(bluetoothctl devices 2>/dev/null | awk '/Wireless Controller/{print $2}')

if [ -n "$OLD_MAC" ]; then
    echo "→ Removendo pareamento antigo do PS4 ($OLD_MAC)..."
    bluetoothctl remove "$OLD_MAC" >/dev/null 2>&1 || true
    sleep 1
fi

# ------------------------------------------------------------------
# 3) Mantém um bluetoothctl em background com agent KeyboardDisplay
# ------------------------------------------------------------------
# Por que KeyboardDisplay e não NoInputNoOutput: o pair "just works" do DS4 com
# NoInputNoOutput não negocia link key persistente nem completa Bonded=yes; o
# pareamento é descartado no primeiro disconnect. KeyboardDisplay força SSP com
# bonding completo, link key é salva em /var/lib/bluetooth/.../info.
#
# Por que FIFO em vez de heredoc: o heredoc fecha o stdin assim que o EOF é lido,
# matando o bluetoothctl (e o agent junto) antes do pair acontecer. O FIFO mantém
# o stdin aberto pela duração do script.

AGENT_LOG=$(mktemp)
AGENT_FIFO=$(mktemp -u)
mkfifo "$AGENT_FIFO"
exec 3<>"$AGENT_FIFO"
bluetoothctl <&3 >"$AGENT_LOG" 2>&1 &
AGENT_PID=$!
trap 'echo quit >&3 2>/dev/null; sleep 1; kill $AGENT_PID 2>/dev/null; exec 3>&- 2>/dev/null; rm -f "$AGENT_FIFO" "$AGENT_LOG"' EXIT
sleep 1
echo "agent KeyboardDisplay" >&3
sleep 1
echo "default-agent" >&3
sleep 1

if ! grep -qE "Agent registered|Agent is already registered" "$AGENT_LOG"; then
    echo "AVISO: agent KeyboardDisplay falhou em registrar; tentando NoInputNoOutput..."
    echo "agent NoInputNoOutput" >&3
    sleep 1
    echo "default-agent" >&3
    sleep 1
fi

# ------------------------------------------------------------------
# 4) Scan até achar o PS4
# ------------------------------------------------------------------

echo "→ Escaneando por até 20s (procurando 'Wireless Controller')..."
# Scan em background; mata quando achar.
bluetoothctl --timeout 20 scan on >/dev/null 2>&1 &
SCAN_PID=$!

PS4_MAC=""
for i in $(seq 1 20); do
    sleep 1
    PS4_MAC=$(bluetoothctl devices 2>/dev/null | awk '/Wireless Controller/{print $2; exit}')
    if [ -n "$PS4_MAC" ]; then
        echo "✓ PS4 encontrado: $PS4_MAC"
        break
    fi
done

# Para o scan independente de ter achado ou não.
kill $SCAN_PID 2>/dev/null || true
wait $SCAN_PID 2>/dev/null || true
bluetoothctl scan off >/dev/null 2>&1 || true

if [ -z "$PS4_MAC" ]; then
    cat <<EOF
ERRO: PS4 não apareceu no scan em 20s.

Diagnóstico:
  - A barra de luz do controle está piscando RÁPIDO agora? (2 flashes/s)
    Se NÃO, ele saiu do modo pareamento. Refaz SHARE+PS por 5s e roda de novo.
  - Outro host (PS4, celular) que já pareou com esse controle está por perto?
    O PS4 prefere o último host. Desliga/afasta antes de tentar.
EOF
    exit 1
fi

# ------------------------------------------------------------------
# 5) Pair (com polling de estado, não confia no retorno)
# ------------------------------------------------------------------

echo "→ Pairing... (até 30s, espera Bonded=yes — não só Paired)"
timeout 25 bluetoothctl pair "$PS4_MAC" 2>&1 | tail -5 || true

BONDED=0
for i in $(seq 1 30); do
    if bluetoothctl info "$PS4_MAC" 2>/dev/null | grep -q "Bonded: yes"; then
        BONDED=1
        break
    fi
    sleep 1
done

if [ "$BONDED" != 1 ]; then
    echo "ERRO: bonding não completou em 30s. Última info:"
    bluetoothctl info "$PS4_MAC" 2>/dev/null | grep -E "Paired|Bonded|Trusted|Connected" || true
    echo
    echo "Diagnóstico:"
    echo "  - Se Paired=yes mas Bonded=no, o agent foi NoInputNoOutput (fallback);"
    echo "    o link key não é persistente. Tenta desligar o controle (PS 10s),"
    echo "    refazer SHARE+PS, e rodar de novo."
    echo "  - Agent log:"
    cat "$AGENT_LOG" | tail -10 | sed 's/^/    /'
    exit 1
fi
echo "✓ Paired + Bonded"

# ------------------------------------------------------------------
# 6) Trust (instantâneo)
# ------------------------------------------------------------------

bluetoothctl trust "$PS4_MAC" >/dev/null 2>&1
echo "✓ Trusted"

# ------------------------------------------------------------------
# 7) Connect — geralmente já está conectado após pair com bonding
# ------------------------------------------------------------------
#
# Diferença vs. NoInputNoOutput: o pair com KeyboardDisplay frequentemente já
# deixa Connected=yes no fim do pair. Só precisa de connect explícito se a
# stack derrubou a HID por outro motivo (ou em --no-prompt sem auto-up).
if ! bluetoothctl info "$PS4_MAC" 2>/dev/null | grep -q "Connected: yes"; then
    if [ "$NO_PROMPT" = 1 ]; then
        echo "→ Tentando connect direto (modo --no-prompt)..."
        timeout 15 bluetoothctl connect "$PS4_MAC" >/dev/null 2>&1 || true
    else
        cat <<'EOF'

→ Pair OK! A barra de luz do controle DEVE estar apagada agora.

   Pressione PS rapidamente (toque, não segura) pra LIGAR o controle.
   A barra vai piscar e depois fixar uma cor sólida ao conectar.

EOF
        read -r -p "Apertou PS? Enter pra esperar a conexão... " _
    fi
fi

echo "→ Aguardando conexão (até 20s)..."
CONNECTED=0
for i in $(seq 1 20); do
    if bluetoothctl info "$PS4_MAC" 2>/dev/null | grep -q "Connected: yes"; then
        CONNECTED=1
        break
    fi
    # Se não auto-conectou em 5s, força um connect explícito (uma vez).
    if [ "$i" = 6 ]; then
        echo "  ... auto-reconnect não disparou, forçando connect..."
        timeout 10 bluetoothctl connect "$PS4_MAC" >/dev/null 2>&1 || true
    fi
    sleep 1
done

if [ "$CONNECTED" != 1 ]; then
    echo "ERRO: controle não conectou em 20s. Diagnóstico:"
    bluetoothctl info "$PS4_MAC" 2>/dev/null | grep -E "Paired|Trusted|Connected|Blocked" || true
    echo
    echo "Últimas linhas do bluetoothd:"
    sudo journalctl -u bluetooth --since "30 seconds ago" --no-pager | tail -15
    exit 1
fi
echo "✓ Connected"

# ------------------------------------------------------------------
# 8) Espera /dev/input/js0 materializar (HID → joydev)
# ------------------------------------------------------------------

echo "→ Aguardando /dev/input/js0 (até 30s; tenta disconnect+reconnect aos 15s)..."
JS_RETRY_DONE=0
for i in $(seq 1 30); do
    if [ -e /dev/input/js0 ]; then
        echo "  apareceu em ${i}s"
        break
    fi
    # Aos 15s: se ainda não materializou, força disconnect+reconnect (geralmente
    # subiu HID profile na 2ª connect quando 1ª ficou em "Connected sem HID").
    if [ "$i" = 15 ] && [ "$JS_RETRY_DONE" = 0 ]; then
        echo "  ... 15s sem js0, tentando disconnect+reconnect pra forçar HID profile..."
        bluetoothctl disconnect "$PS4_MAC" >/dev/null 2>&1 || true
        sleep 2
        timeout 10 bluetoothctl connect "$PS4_MAC" >/dev/null 2>&1 || true
        JS_RETRY_DONE=1
    fi
    sleep 1
done

if [ ! -e /dev/input/js0 ]; then
    cat <<EOF
AVISO: BT conectou (Connected=yes, Bonded=yes), mas /dev/input/js0 não materializou.

Estado do controle no BlueZ:
$(bluetoothctl info "$PS4_MAC" 2>/dev/null | grep -E "Paired|Bonded|Trusted|Connected|UUIDs" | sed 's/^/  /')

Modulos:
$(lsmod | grep -E "^hid_playstation|^hid_sony|^joydev|^hidp" | sed 's/^/  /')

Logs do bluetoothd nos ultimos 30s:
$(sudo journalctl -u bluetooth --since "30 seconds ago" --no-pager 2>&1 | grep -iE "input|hid|rejected|playstation" | tail -8 | sed 's/^/  /')

Tenta apertar PS curto agora (com BT já estabelecido, HID pode subir manualmente).
EOF
    exit 1
fi

# ------------------------------------------------------------------
# 9) Sucesso
# ------------------------------------------------------------------

cat <<EOF

==========================================================
✓ Tudo certo! Controle pareado, conectado e visível como JS.
==========================================================

Status:
$(bluetoothctl info "$PS4_MAC" | grep -E "Name|Paired|Trusted|Connected" | sed 's/^/  /')

Dispositivo:
  /dev/input/js0

Auto-reconnect via PS curto: NÃO confiável neste hardware. Pra próxima conexão,
entra em modo pareamento (PS 10s + SHARE+PS 5s) e roda este script de novo —
como o link key é persistente, o pair sai rápido.

Testes:
  jstest /dev/input/js0                    # aperta botões → mostra mudanças
  ros2 run joy joy_enumerate_devices       # vê o joystick no ROS

Subir a stack (vai começar a publicar /joy ao mexer no analógico com L1):
  cd ~/Workspace/Controle_robo_web && ./launch.sh
EOF
