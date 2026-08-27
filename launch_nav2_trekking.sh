#!/usr/bin/env bash
# Launcher exclusivo da stack nav2_trekking.
#
# Ele reutiliza toda a infraestrutura madura do launch.sh (build incremental,
# MEGA, LD06 com retry/watchdog, web e cleanup), mas fixa o pacote ROS no clone
# de competição. Assim não existe o risco de rodar `robot_nav` por engano.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE=""
MAP_FILE=""
ALLOW_NARROW_MAP=false
PI_PROFILE_SET=false
FLASH_MODE_SET=false
FORWARD_ARGS=()

usage() {
    cat <<EOF
Uso:
  $0 --slam [opções]
  $0 --nav2 --map=maps/<mapa>.yaml [opções]

Fluxo da prova real:
  1. $0 --slam
  2. Salve o mapa; volte ao ponto inicial (ou anote a pose) e encerre com Ctrl+C.
  3. $0 --nav2 --map=maps/<mapa>.yaml
  4. Confira o robô no mapa; se não voltou ao início, use "📍 Definir pose".

Opções próprias:
  --allow-narrow-map   continua mesmo se o raio 0,32 fechar parte do mapa;
                       use somente depois de ler o relatório de passagens.
  --flash-mega        permite explicitamente atualizar o firmware da MEGA;
                      por padrão este launcher nunca faz flash durante a prova.

As demais opções (--web-teleop, --no-flash-mega, --lidar-port=...) são
repassadas ao launch.sh. O perfil validado nav2_params_pi.yaml é obrigatório.
Este launcher não aceita teleop nem o trekking antigo.
EOF
}

for arg in "$@"; do
    case "$arg" in
        --slam)
            [ -z "$MODE" ] || { echo "ERRO: escolha somente --slam ou --nav2." >&2; exit 1; }
            MODE="slam"
            FORWARD_ARGS+=("$arg")
            ;;
        --nav2)
            [ -z "$MODE" ] || { echo "ERRO: escolha somente --slam ou --nav2." >&2; exit 1; }
            MODE="nav2"
            FORWARD_ARGS+=("$arg")
            ;;
        --map=*)
            MAP_FILE="${arg#*=}"
            FORWARD_ARGS+=("$arg")
            ;;
        --allow-narrow-map)
            ALLOW_NARROW_MAP=true
            ;;
        --pi)
            PI_PROFILE_SET=true
            FORWARD_ARGS+=("$arg")
            ;;
        --no-pi)
            echo "ERRO: --no-pi troca a configuração validada do nav2_trekking." >&2
            echo "      Use o launcher genérico se a intenção for testar outro perfil." >&2
            exit 1
            ;;
        --flash-mega|--no-flash-mega)
            [ "$FLASH_MODE_SET" = false ] || {
                echo "ERRO: escolha somente uma opção de flash da MEGA." >&2
                exit 1
            }
            FLASH_MODE_SET=true
            FORWARD_ARGS+=("$arg")
            ;;
        --teleop|--trekking)
            echo "ERRO: '$arg' não pertence ao nav2_trekking. Use --slam ou --nav2." >&2
            exit 1
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            FORWARD_ARGS+=("$arg")
            ;;
    esac
done

if [ -z "$MODE" ]; then
    usage >&2
    exit 1
fi

if [ "$MODE" = "nav2" ]; then
    if [ -z "$MAP_FILE" ]; then
        echo "ERRO: o nav2_trekking exige --map=maps/<mapa>.yaml explícito." >&2
        echo "      Isso evita navegar sem querer com o mapa antigo de outro lugar." >&2
        exit 1
    fi
    if [[ "$MAP_FILE" != /* ]]; then
        MAP_FILE="$SCRIPT_DIR/$MAP_FILE"
    fi
    if [ ! -f "$MAP_FILE" ]; then
        echo "ERRO: mapa não encontrado: $MAP_FILE" >&2
        exit 1
    fi
    if ! python3 -c 'import numpy, scipy' >/dev/null 2>&1; then
        echo "ERRO: o conferidor obrigatório do mapa precisa de NumPy e SciPy." >&2
        echo "      Na Pi: sudo apt install python3-numpy python3-scipy" >&2
        exit 1
    fi

    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  PRÉ-VOO DO MAPA — robot_radius=0,32 m"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    CHECK_ARGS=("$MAP_FILE" --raio 0.32)
    [ "$ALLOW_NARROW_MAP" = false ] && CHECK_ARGS+=(--strict)
    if ! python3 "$SCRIPT_DIR/tools/mapa_passagens.py" "${CHECK_ARGS[@]}"; then
        echo ""
        echo "ERRO: o mapa não passou no pré-voo obrigatório." >&2
        echo "      Revise os gargalos. Para prosseguir após decisão consciente:" >&2
        echo "      $0 --nav2 --map=\"$MAP_FILE\" --allow-narrow-map" >&2
        exit 1
    fi

    # A configuração medida no sim e congelada para a prova é a *_pi.yaml.
    # Força o mesmo perfil inclusive se o hardware real não for arm64.
    [ "$PI_PROFILE_SET" = true ] || FORWARD_ARGS+=(--pi)
fi

# A prova é da navegação, não do firmware. Impede o modo "auto" do launcher
# genérico de flashear a MEGA só porque o checkout/arquivo de stamp mudou.
[ "$FLASH_MODE_SET" = true ] || FORWARD_ARGS+=(--no-flash-mega)

export ROBOT_NAV_PACKAGE=nav2_trekking
exec "$SCRIPT_DIR/launch.sh" "${FORWARD_ARGS[@]}"
