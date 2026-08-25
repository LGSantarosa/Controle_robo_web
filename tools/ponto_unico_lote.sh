#!/bin/bash
# ponto_unico_lote — roda N trials do banco MINIMO, cada um numa stack LIMPA.
#
# POR QUE STACK LIMPA POR TRIAL: a /odom do sim vem do DiffDrive (integra as
# rodas) e NAO zera quando a gente teleporta o robo de volta. Reaproveitar a
# stack faria o trial 2 comecar com a deriva do trial 1 embutida — os numeros
# nao seriam independentes e a pergunta "ele para sempre no mesmo ponto?"
# ficaria sem resposta. Entao: sobe, roda 1 trial, derruba, repete.
#
# USO (com o robo REAL desligado; isto e 100% sim):
#   tools/ponto_unico_lote.sh -n 5 --tag sem_cone --sem-cone
#   tools/ponto_unico_lote.sh -n 5 --tag com_cone
#
# Sai com o CSV em log/ponto_unico/<tag>.csv e o resumo (media/desvio) no
# terminal. Qualquer Ctrl+C derruba a stack junto (trap).
set -u
SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$SCRIPT_DIR"

N=5
TAG="lote"
STANDOFF="1.2"
TIMEOUT="60"
EXTRA=()
while [ $# -gt 0 ]; do
    case "$1" in
        -n)          N="$2"; shift 2 ;;
        --tag)       TAG="$2"; shift 2 ;;
        --standoff)  STANDOFF="$2"; shift 2 ;;
        --timeout)   TIMEOUT="$2"; shift 2 ;;
        --sem-cone)  EXTRA+=(--sem-cone); TAG="${TAG}"; shift ;;
        -h|--help)   sed -n '2,16p' "$0"; exit 0 ;;
        *)           EXTRA+=("$1"); shift ;;
    esac
done

OUT_DIR="$SCRIPT_DIR/log/ponto_unico"
mkdir -p "$OUT_DIR"
STAMP="$(date +%Y-%m-%d_%H%M%S)"
CSV="$OUT_DIR/${TAG}_${STAMP}.csv"
LOGS="$OUT_DIR/${TAG}_${STAMP}_launch"
mkdir -p "$LOGS"
echo "trial,fim,dur_s,x,y,yaw_deg,odom_x,odom_y,wp_odom_x,wp_odom_y" > "$CSV"

LAUNCH_PID=""
derruba() {
    [ -n "$LAUNCH_PID" ] && kill -INT "-$LAUNCH_PID" 2>/dev/null
    # espera o cleanup() do launch.sh terminar sozinho (ate 20 s)
    for _ in $(seq 40); do
        kill -0 "-$LAUNCH_PID" 2>/dev/null || break
        sleep 0.5
    done
    # rede de seguranca: os que o cleanup() as vezes deixa (memoria 07-20:
    # orfao de parameter_bridge duplica o /clock e trava o proximo run)
    pkill -9 -f "gz sim"           2>/dev/null
    pkill -9 -f "parameter_bridge" 2>/dev/null
    pkill -9 -f "controle_web/app.py" 2>/dev/null
    pkill -9 -f "trekking_runner"  2>/dev/null
    sleep 1
    LAUNCH_PID=""
}
trap 'echo; echo "[lote] interrompido — derrubando a stack"; derruba; exit 130' INT TERM
trap 'derruba' EXIT

# `set -u` + setup.bash do ROS nao se dao: os scripts do ament referenciam
# variaveis nao setadas ($1, COLCON_TRACE) e o shell morre calado. Solto o -u so aqui.
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null
set -u

espera_pronto() {   # $1 = segundos maximos
    local fim=$(( $(date +%s) + $1 ))
    while [ "$(date +%s)" -lt "$fim" ]; do
        if ros2 topic list 2>/dev/null | grep -q '^/trekking/state$' \
           && gz model -m sim_robot -p >/dev/null 2>&1; then
            sleep 3   # folga pro cone_detector ver o primeiro scan
            return 0
        fi
        sleep 2
    done
    return 1
}

for i in $(seq 1 "$N"); do
    echo "=== trial $i/$N ($TAG) ==="
    setsid ./launch.sh --sim --trekking --world=worlds/trekking_min.sdf \
        > "$LOGS/trial_$i.log" 2>&1 &
    LAUNCH_PID=$!
    if ! espera_pronto 90; then
        echo "  FALHOU: stack nao subiu em 90 s (ver $LOGS/trial_$i.log)"
        echo "$i,SEM_STACK,,,,,,,," >> "$CSV"
        derruba
        continue
    fi
    LINHA="$(python3 tools/_ponto_unico_run.py --standoff "$STANDOFF" \
             --timeout "$TIMEOUT" ${EXTRA[@]+"${EXTRA[@]}"} 2>/dev/null | tail -1)"
    echo "  $LINHA"
    python3 - "$i" "$CSV" "$LINHA" <<'PY'
import json, sys
i, csv, linha = sys.argv[1], sys.argv[2], sys.argv[3]
try:
    d = json.loads(linha)
except Exception:
    d = {'fim': 'SEM_JSON'}
w = d.get('wp_odom') or [None, None]
cols = [i, d.get('fim'), d.get('dur'), d.get('x'), d.get('y'), d.get('yaw'),
        d.get('odom_x'), d.get('odom_y'), w[0], w[1]]
open(csv, 'a').write(','.join('' if c is None else str(c) for c in cols) + '\n')
PY
    derruba
done

echo
echo "CSV: $CSV"
python3 tools/ponto_unico_resumo.py "$CSV" --standoff "$STANDOFF"
