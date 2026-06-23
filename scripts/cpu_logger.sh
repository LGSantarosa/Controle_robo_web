#!/bin/bash
# cpu_logger.sh — registra a carga da CPU da Pi durante o trajeto, SEM ferramentas pesadas.
#
# POR QUE existe: a medida do dia 06-22 ("load 30") saiu poluida pela propria
# sondagem (ros2 / tf2_echo / top via SSH pesam na Pi). Este logger le SO o /proc
# (loadavg + /proc/stat por core) em shell puro, 1 amostra/s. Custo despresivel.
# A ideia: rodar ISTO na Pi, o operador fica FORA do SSH, faz o trajeto normal, e
# depois a carga real e lida UMA vez do arquivo.
#
# USO na Pi:
#   ./scripts/cpu_logger.sh                 # grava /tmp/cpu_run_<data>.log, 1/s
#   ./scripts/cpu_logger.sh /tmp/x.log      # arquivo custom
#   ./scripts/cpu_logger.sh /tmp/x.log 0.5  # arquivo + intervalo (s)
#
# Marcar eventos enquanto roda (de QUALQUER terminal, opcional):
#   echo "porta" > /tmp/cpu_logger.mark     # o proximo registro carimba "porta"
#
# Parar: Ctrl-C (se em foreground) ou: kill $(cat /tmp/cpu_logger.pid)
#
# Colunas do log (espaco-separadas, com cabecalho):
#   ts            horario ISO local
#   load1         loadavg 1min (/proc/loadavg)
#   cpu%          uso TOTAL da CPU no intervalo (0-100, todos os cores agregados)
#   c0%..cN%      uso por core no intervalo
#   nproc         qtd de processos rodaveis (R) / total (do loadavg)
#   mark          rotulo do evento, se houver (senao "-")

set -u

OUT="${1:-/tmp/cpu_run_$(date +%Y%m%d_%H%M%S).log}"
INTERVAL="${2:-1}"
MARKFILE="/tmp/cpu_logger.mark"
PIDFILE="/tmp/cpu_logger.pid"

echo $$ > "$PIDFILE"
rm -f "$MARKFILE"

NCPU=$(grep -c '^processor' /proc/cpuinfo)

# Snapshot de /proc/stat -> arrays globais PREV_IDLE[i], PREV_TOTAL[i]
# indice 0 = agregado "cpu", 1..N = "cpu0".."cpu(N-1)"
declare -a PREV_IDLE PREV_TOTAL
read_stat() {
  local -n _idle=$1
  local -n _total=$2
  local i=0 line fields idle total
  while read -r -a fields; do
    case "${fields[0]}" in
      cpu|cpu[0-9]*)
        # campos: user nice system idle iowait irq softirq steal ...
        idle=$(( ${fields[4]} + ${fields[5]:-0} ))           # idle + iowait
        total=0
        for v in "${fields[@]:1}"; do total=$(( total + v )); done
        _idle[$i]=$idle
        _total[$i]=$total
        i=$(( i + 1 ))
        ;;
    esac
  done < /proc/stat
}

# cabecalho
{
  printf '# cpu_logger.sh  inicio=%s  intervalo=%ss  cores=%s  pid=%s\n' \
         "$(date -Is)" "$INTERVAL" "$NCPU" "$$"
  printf 'ts load1 cpu%%'
  for ((c=0; c<NCPU; c++)); do printf ' c%d%%' "$c"; done
  printf ' runq mark\n'
} >> "$OUT"

cleanup() { rm -f "$PIDFILE" "$MARKFILE"; echo "[cpu_logger] fim -> $OUT"; }
trap cleanup EXIT INT TERM

echo "[cpu_logger] gravando em $OUT (1 amostra/${INTERVAL}s). Pare com Ctrl-C."

read_stat PREV_IDLE PREV_TOTAL
sleep "$INTERVAL"

while :; do
  declare -a IDLE TOTAL
  read_stat IDLE TOTAL

  ts=$(date '+%H:%M:%S')
  read -r load1 _ _ runq _ < /proc/loadavg   # ex.: "1.23 0.98 0.55 2/345 6789"

  line="$ts $load1"
  for ((i=0; i<=NCPU; i++)); do
    di=$(( IDLE[i] - PREV_IDLE[i] ))
    dt=$(( TOTAL[i] - PREV_TOTAL[i] ))
    if (( dt > 0 )); then
      use=$(( (100 * (dt - di)) / dt ))
    else
      use=0
    fi
    line="$line $use"
  done

  mark="-"
  if [[ -f "$MARKFILE" ]]; then
    mark=$(tr -s ' \n' '_' < "$MARKFILE" | sed 's/_$//')
    rm -f "$MARKFILE"
  fi
  line="$line $runq $mark"

  echo "$line" >> "$OUT"

  PREV_IDLE=("${IDLE[@]}")
  PREV_TOTAL=("${TOTAL[@]}")
  sleep "$INTERVAL"
done
