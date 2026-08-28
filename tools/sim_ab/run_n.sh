#!/bin/bash
# Roda N voltas em SEQUÊNCIA, desanexado. Falha de setup não conta como volta:
# ela é refeita (até 2x por slot) e registrada à parte.
# Uso: run_n.sh <pacote> <prefixo_tag> <n>
SP=${SIM_AB_DIR:-/home/rbe-luis/Workspace/Controle_robo_web/log/sim_ab}
# 2026-08-28: os SCRIPTS agora saem do repo (TOOLS = a pasta deste arquivo), nao
# de $SIM_AB_DIR. Antes o run_n chamava "$TOOLS/run_one.sh" e log/sim_ab/ guardava
# COPIAS do harness inteiro — que e' gitignore'd (.gitignore:20). Resultado: o
# codigo que rodava nao era o do git, correcao no repo nao chegava na execucao, e
# o `rm -rf "$OUT"` podia apagar as proprias ferramentas. $SP agora e' SO' saida.
TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$1"; PREFIXO="$2"; N="$3"

if [ $# -ne 3 ] || [ -z "$PKG" ] || [ -z "$PREFIXO" ]; then
    echo "USO: run_n.sh <pacote> <prefixo_tag> <n>" >&2; exit 2
fi
case "$N" in ''|*[!0-9]*) echo "USO: <n> tem que ser inteiro positivo (veio: '$N')" >&2; exit 2 ;; esac
[ "$N" -ge 1 ] || { echo "USO: <n> >= 1" >&2; exit 2; }

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
bash "$TOOLS/kill_all.sh"   # ambiente zerado ANTES da 1a volta
# ---------------------------------------------------------------------------
echo "=== $N voltas de $PKG, prefixo $PREFIXO — início $(date +%H:%M:%S) ==="
for i in $(seq 1 "$N"); do
    TAG="${PREFIXO}${i}"
    for tentativa in 1 2; do
        echo "--- volta $i/$N (tag $TAG), tentativa $tentativa — $(date +%H:%M:%S) ---"
        bash "$TOOLS/run_one.sh" "$PKG" "$TAG"
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
bash "$TOOLS/kill_all.sh"
echo "=== fim $(date +%H:%M:%S) ==="
touch "$SP/${PREFIXO}_TERMINOU"
