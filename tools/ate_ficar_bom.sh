#!/bin/bash
# ate_ficar_bom — roda a rota2 no sim ate sair uma corrida LIMPA, e PARA.
#
# POR QUE EXISTE (pedido do dono, 2026-08-26): "quando ele fizer o caminho
# perfeito vc para tudo pra eu n ter que te parar". O 5o giro e' intermitente —
# a mesma stack, a mesma rota, as vezes sai em 4 giros e as vezes em 5. Ficar
# rodando lote fixo obriga o dono a interromper na mao quando ve a corrida boa.
#
# LIMPA = as 4 condicoes:
#   - concluiu (nao travou)
#   - erro final < 60 cm (nao bateu)
#   - 4 giros de verdade (>=2 graus) — o minimo geometrico da rota2
#   - passou nos DOIS waypoints, cada um a menos de 50 cm
set -u
DIR="$(cd "$(dirname "$0")/.." && pwd)"; cd "$DIR"
MAX="${1:-8}"
for i in $(seq 1 "$MAX"); do
    echo "=== tentativa $i/$MAX ==="
    tools/diag_assoc.sh -n 1 --tag boa 2>&1 | grep -E '^  \{|FALHOU'
    if python3 tools/_avalia_corrida.py; then
        echo
        echo "################################################################"
        echo "#  CORRIDA LIMPA na tentativa $i — parando aqui, stack derrubada"
        echo "################################################################"
        exit 0
    fi
    echo "  (ainda nao — proxima)"
done
echo "Nenhuma corrida limpa em $MAX tentativas."
exit 1
