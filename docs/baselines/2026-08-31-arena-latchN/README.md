# Repetição — 3 voltas com o latch da chegada, 2026-08-31

Evidência dos números da §2B.5 do `DIARIO_ARENA.md`. Rodadas com
`tools/sim_ab/run_n.sh robot_nav latchN 3`, mesmas env vars da §4.5, **mesmo
código** da volta `arena_latch1` (commit `c85a8d8`) — nenhuma mudança entre elas.

| arquivo | o que sustenta |
|---|---|
| `resumo_4_voltas.csv` | a tabela da §2B.5: baseline + as 4 voltas com latch, lado a lado |
| `result_latchN1.json` | tempo e status por goal — ⚠️ o `status` do goal 2 diz `CANCELADO`, rótulo do bug corrigido em 08-31: **leia ABORTADO** (ver abaixo) |
| `result_latchN2.json` | tempo e status por goal |
| `result_latchN3.json` | tempo e status por goal |
| `colisao_3voltas.csv` | **contato**: a menor folga por objeto e a contagem de eventos, por volta. É o que sustenta "zero contato em 4/4" |
| `transicoes_goal_turn_3voltas.csv` | **samba**: toda transição de/para `goal_turn` nas 3 voltas. É o que sustenta "samba 0" |
| `unstuck_disparos_3voltas.csv` | **unstuck**: cada troca de estado, com `reason`, `stuck_s` e `nav_wants` |

> Estes três últimos entraram **depois**, no review do dono: a primeira versão
> desta pasta arquivava só o **resumo derivado**, então as conclusões centrais
> (zero contato, samba zero) não eram auditáveis a partir do repo — os CSVs
> brutos vivem em `log/sim_ab/`, que é `gitignore`d.
>
> **Como foram derivados** (segunda rodada do review: extrato sem script é
> número que ninguém refaz):
>
> ```
> python3 tools/sim_ab/extrai_evidencia.py \
>     docs/baselines/2026-08-31-arena-latchN latchN1 latchN2 latchN3
> ```
>
> E o `resumo_4_voltas.csv` (a coluna `samba` era conta **manual** até o segundo
> review):
>
> ```
> python3 tools/sim_ab/extrai_evidencia.py --resumo \
>     docs/baselines/2026-08-31-arena-latchN/resumo_4_voltas.csv \
>     arena_baseline1 arena_latch1 latchN1 latchN2 latchN3
> ```
>
> O script é versionado, tem autoteste, e **não escreve nada** se algum CSV bruto
> faltar: confere todos antes, gera em temporários e só troca os destinos quando
> todos derem certo. Rodá-lo por cima desta pasta reproduz os **quatro** arquivos
> byte a byte.

## O que estas 3 voltas mudaram na conclusão

**Confirmam** (agora com taxa, não amostra): samba **0 em 4/4** (baseline 8) e
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

## ⚠️ O `CANCELADO` dentro do `result_latchN1.json` é rótulo errado

O harness mapeava `GoalStatus` invertido (5/6 trocados) — corrigido em
`tools/sim_ab/probe.py` nesta mesma data. **O JSON aqui é a saída bruta daquela
volta e fica como está**: reescrever registro gravado pra ficar bonito é pior que
o rótulo errado. Quem consumir este arquivo por script tem que saber: neste
arquivo (e em qualquer saída deste harness anterior a 2026-08-31),
**`CANCELADO` = status 6 = ABORTED**, o Nav2 desistiu do goal.

## Correção de método (review do dono, 08-31)

A média de tempo das voltas com latch é **242,9 s** — só as **completas** (5/5):
222,8 / 251,2 / 254,7. A `latchN1` (4/5) **não entra**: ela perdeu o goal 2 e não
percorreu o mesmo caminho. Contra o baseline de 236,4 s (n=1), o defensável é
**"não há evidência de que o latch acelerou"**, não "o ganho era ruído".

⚠️ Voltas até os **STANDOFFS**. Não provam A1, A2 nem A3.
