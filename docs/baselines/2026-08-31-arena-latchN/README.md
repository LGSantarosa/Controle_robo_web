# Repetição — 3 voltas com o latch da chegada, 2026-08-31

Evidência dos números da §2B.5 do `DIARIO_ARENA.md`. Rodadas com
`tools/sim_ab/run_n.sh robot_nav latchN 3`, mesmas env vars da §4.5, **mesmo
código** da volta `arena_latch1` (commit `c85a8d8`) — nenhuma mudança entre elas.

| arquivo | o que sustenta |
|---|---|
| `resumo_4_voltas.csv` | a tabela da §2B.5: baseline + as 4 voltas com latch, lado a lado |
| `result_latchN1.json`, `result_latchN2.json`, `result_latchN3.json` | tempo e status por goal de cada volta |

## O que estas 3 voltas mudaram na conclusão

**Confirmam** (agora com taxa, não amostra): samba **0 em 4/4** (baseline 7) e
contato **0 em 4/4** (baseline 2 colisões + 28 raspões), folga mínima sempre
positiva (6,2 a 10,1 cm).

**Derrubam** duas afirmações minhas da §2B.4:

1. o **"−5,7% de tempo"** era ruído — com n=4 a média é **237,1 s** contra
   **236,4 s** do baseline. O latch não acelerou nada; parou de bater;
2. **"o goal 4 travou"** — trava **um goal por volta, e muda de goal** (g4, g4,
   g5, g5). O defeito é sistemático (4/4), não do goal 4.

## A perda de goal em `latchN1` não é do latch

O probe imprimiu `CANCELADO`, mas **o rótulo estava invertido** no harness
(`GoalStatus` é 5=CANCELED, **6=ABORTED**; corrigido em `probe.py` nesta mesma
data). Era **ABORT do Nav2**, 0,4 s depois de
`"Either of the start or goal pose are an obstacle!"` partindo de dentro da
**fresta A** — o bug já aberto nos itens **2d/8** da §6, no mesmo ponto onde a
§2.8 o viu. Taxa de goals com o latch: **19/20**.

⚠️ Voltas até os **STANDOFFS**. Não provam A1, A2 nem A3.
