# Aproximação final — 3 voltas, 2026-08-31

Evidência da §2B.6 do `DIARIO_ARENA.md`. Mesmo comando da §4.5, tags
`aprox1..3`, com o commit `4da6eb4` (aproximação final) sobre o `c85a8d8` (latch).

| arquivo | o que sustenta |
|---|---|
| `resumo_7_voltas.csv` | as 7 voltas desde o baseline, lado a lado |
| `dist_final_por_goal.csv` | **a medida direta do defeito 2e**: a que distância cada goal completou, e se cabe nos 0,15 do checker do Nav2 |
| `colisao_3voltas.csv` | contato por objeto e por volta — inclui os **4 raspões da `aprox2`** |
| `transicoes_goal_turn_3voltas.csv` | toda troca de/para as fases de chegada; é onde se lê o churn e a ausência de samba |
| `unstuck_disparos_3voltas.csv` | os disparos do unstuck (caíram: 0,0 / 3,0 / 1,4 s) |

Gerados por (o script é versionado, tem autoteste e não escreve nada se faltar
bruto):

```
python3 tools/sim_ab/extrai_evidencia.py \
    docs/baselines/2026-08-31-arena-aproximacao aprox1 aprox2 aprox3
python3 tools/sim_ab/extrai_evidencia.py --resumo \
    docs/baselines/2026-08-31-arena-aproximacao/resumo_7_voltas.csv \
    arena_baseline1 arena_latch1 latchN1 latchN2 latchN3 aprox1 aprox2 aprox3
```

## O que estas 3 voltas dizem

**A correção faz o que foi desenhada pra fazer:** os goals completam a **3–9 cm**
(era 11–15). Interferência do `unstuck` caiu.

**Mas o item 2e não fechou.** A `aprox2` levou 328,6 s com ~54 s de giro perto dos
goals — *churn* de mira (aproxima, fecha o yaw, a deriva tira, re-aproxima),
bounded em 3 re-entradas, nunca laço infinito. E o `parado` do probe **conta
point-turn como parado**, então ele mede duas coisas juntas: a métrica boa é o
`dist_final_por_goal.csv`.

**Os 4 raspões da `aprox2` não são da aproximação.** Estão em `state=turning` a
**6,15 m do goal**: é o canto varrendo o `cone_3` no point-turn de rota — o item 1
dos abertos. Em 7 voltas com latch, **6 limpas e 1 com contato**: a causa nunca
foi corrigida, só não tinha se alinhado.

**Teimoso, não investigado:** o goal 2 termina a 0,156–0,160 nas **três** voltas.

⚠️ Voltas até os **STANDOFFS**. Não provam A1, A2 nem A3.
