#!/bin/bash
# Roda N voltas em SEQUÊNCIA, desanexado. Falha de setup não conta como volta:
# ela é refeita (até 2x por slot) e registrada à parte.
# Uso: run_n.sh <pacote> <prefixo_tag> <n>
SP=${SIM_AB_DIR:-/home/rbe-luis/Workspace/Controle_robo_web/log/sim_ab}
PKG="$1"; PREFIXO="$2"; N="$3"

# ---- LOCK: NUNCA dois runs ao mesmo tempo ----------------------------------
# 2026-08-27: relancei o run_n sem matar o anterior (o kill_all não mata os
# orquestradores, senão se mataria). Resultado: DOIS run_n vivos, DOIS gazebos,
# dois run_one escrevendo na MESMA pasta e matando os nós um do outro. O dono
# viu 2 Gazebos na tela. Agora é impossível: quem chega depois mata o anterior
# e só então assume o lock.
LOCK="$SP/.run_n.lock"
if [ -f "$LOCK" ]; then
    ANTIGO=$(cat "$LOCK" 2>/dev/null)
    if [ -n "$ANTIGO" ] && kill -0 "$ANTIGO" 2>/dev/null; then
        echo "[lock] matando run_n anterior (PID $ANTIGO) e a árvore dele"
        pkill -9 -P "$ANTIGO" 2>/dev/null
        kill -9 "$ANTIGO" 2>/dev/null
        sleep 2
    fi
fi
echo $$ > "$LOCK"
trap 'rm -f "$LOCK"' EXIT
bash "$SP/kill_all.sh"   # ambiente zerado ANTES da 1a volta
# ---------------------------------------------------------------------------
echo "=== $N voltas de $PKG, prefixo $PREFIXO — início $(date +%H:%M:%S) ==="
for i in $(seq 1 "$N"); do
    TAG="${PREFIXO}${i}"
    for tentativa in 1 2; do
        echo "--- volta $i/$N (tag $TAG), tentativa $tentativa — $(date +%H:%M:%S) ---"
        bash "$SP/run_one.sh" "$PKG" "$TAG"
        rc=$?
        if [ $rc -eq 0 ]; then
            echo "--- volta $i OK ---"
            break
        fi
        echo "--- volta $i: FALHA DE SETUP (rc=$rc), refazendo ---"
        sleep 5
    done
    sleep 5
done
bash "$SP/kill_all.sh"
echo "=== fim $(date +%H:%M:%S) ==="
touch "$SP/${PREFIXO}_TERMINOU"
