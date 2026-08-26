#!/bin/bash
# diag_assoc — lote do diagnostico de ASSOCIACAO do cone-ancora (100% sim).
#
# PERGUNTA QUE RESPONDE: quando a correcao repetida explode, o que muda —
# a IDENTIDADE do cone que casou (coluna `cone_id` do CSV) ou o tamanho do
# Delta (`fix_dx`/`fix_dy`)? Uma coisa e' re-associacao; a outra e' a pose
# virando escrava do ruido da ancora. So o CSV separa as duas.
#
# Stack LIMPA por trial (mesma razao do ponto_unico_lote: a /odom do DiffDrive
# nao zera com teleporte, e a rota foi gravada a partir do spawn).
#
# USO (robo real desligado):
#   tools/diag_assoc.sh -n 1 --tag base
#   tools/diag_assoc.sh -n 3 --tag rep --repeat
set -u
DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$DIR"

N=1; TAG="run"; REPEAT=""; TIMEOUT=120; VMAX=""; GT_HZ=10
ROTA="maps/routes/trekking/rota2.json"
while [ $# -gt 0 ]; do
    case "$1" in
        -n)         N="$2"; shift 2 ;;
        --tag)      TAG="$2"; shift 2 ;;
        --repeat)   REPEAT="cone_fix_repeat:=true"; shift ;;
        --vmax)     VMAX="v_max:=$2"; shift 2 ;;
        --rota)     ROTA="$2"; shift 2 ;;
        --timeout)  TIMEOUT="$2"; shift 2 ;;
        -h|--help)  sed -n '2,16p' "$0"; exit 0 ;;
        *)          echo "arg desconhecido: $1"; exit 2 ;;
    esac
done

OUT="$DIR/log/cone_assoc"; mkdir -p "$OUT"
STAMP="$(date +%Y-%m-%d_%H%M%S)"
RESUMO="$OUT/${TAG}_${STAMP}.jsonl"

LAUNCH_PID=""
derruba() {
    [ -n "$LAUNCH_PID" ] && kill -INT "-$LAUNCH_PID" 2>/dev/null
    for _ in $(seq 40); do kill -0 "-$LAUNCH_PID" 2>/dev/null || break; sleep 0.5; done
    # rede de seguranca: o orfao de parameter_bridge duplica o /clock e trava a
    # proxima corrida (memoria 2026-07-20)
    pkill -9 -f "gz sim"              2>/dev/null
    pkill -9 -f "parameter_bridge"    2>/dev/null
    pkill -9 -f "controle_web/app.py" 2>/dev/null
    pkill -9 -f "trekking_runner"     2>/dev/null
    sleep 1; LAUNCH_PID=""
}
trap 'echo; echo "[diag] interrompido"; derruba; exit 130' INT TERM
trap 'derruba' EXIT

set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$DIR/install/setup.bash" 2>/dev/null
set -u

echo "=== diag_assoc  tag=$TAG  n=$N  ${REPEAT:-(correcao 1x por cone)} ==="
for i in $(seq 1 "$N"); do
    echo "--- trial $i/$N ---"
    # YAWARG: gancho do A/B roda-vs-IMU, ex.: YAWARG="use_imu_yaw:=false"
    setsid env TREK_EXTRA_ARGS="$REPEAT $VMAX ${YAWARG:-}" \
        ./launch.sh --sim --trekking --world=worlds/trekking.sdf \
        > "$OUT/${TAG}_${STAMP}_t${i}_launch.log" 2>&1 &
    LAUNCH_PID=$!

    FIM=$(( $(date +%s) + 90 )); PRONTO=0
    while [ "$(date +%s)" -lt "$FIM" ]; do
        if ros2 topic list 2>/dev/null | grep -q '^/trekking/state$' \
           && gz model -m sim_robot -p >/dev/null 2>&1; then
            sleep 3   # folga pro cone_detector ver o primeiro scan
            PRONTO=1; break
        fi
        sleep 2
    done
    if [ "$PRONTO" != 1 ]; then
        echo "  FALHOU: stack nao subiu (ver ${TAG}_${STAMP}_t${i}_launch.log)"
        echo "{\"trial\":$i,\"fim\":\"SEM_STACK\"}" >> "$RESUMO"
        derruba; continue
    fi

    # VERDADE-TERRENO em paralelo. Sem ela eu leio de novo so' a odom — que
    # hoje ja' disse "concluiu" enquanto o robo estava capotado (2026-08-26).
    GTCSV="$OUT/${TAG}_${STAMP}_t${i}_gt.csv"
    MUNDO="$(sed -n 's/.*<world name="\([^"]*\)".*/\1/p' worlds/trekking.sdf | head -1)"
    python3 tools/gt_trekking.py --dur "$TIMEOUT" --hz "$GT_HZ" --out "$GTCSV" \
        --mundo "$MUNDO" \
        > /dev/null 2>&1 &
    GT_PID=$!

    LINHA="$(python3 tools/_diag_assoc_run.py --rota "$ROTA" \
             --timeout "$TIMEOUT" 2>/dev/null | tail -1)"
    kill "$GT_PID" 2>/dev/null; wait "$GT_PID" 2>/dev/null
    echo "  $LINHA"
    echo "$LINHA" | python3 -c "import sys,json;d=json.loads(sys.stdin.read() or '{}');d['trial']=$i;print(json.dumps(d))" \
        >> "$RESUMO" 2>/dev/null || echo "{\"trial\":$i,\"fim\":\"SEM_JSON\"}" >> "$RESUMO"

    # o trekking_runner abre o CSV com 'w' — o proximo launch APAGA este.
    cp -f "$DIR/controle_web/logs/trekking.csv" \
          "$OUT/${TAG}_${STAMP}_t${i}.csv" 2>/dev/null
    derruba
done

echo; echo "resumo: $RESUMO"; echo "CSVs:   $OUT/${TAG}_${STAMP}_t*.csv"
