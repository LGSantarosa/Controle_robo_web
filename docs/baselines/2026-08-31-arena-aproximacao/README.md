# Aproximação final — 3 voltas, 2026-08-31

Evidência da §2B.6 do `DIARIO_ARENA.md`. Mesmo comando da §4.5, tags
`aprox1..3`, com o commit `4da6eb4` (aproximação final) sobre o `c85a8d8` (latch).

⚠️ **Estas voltas validam `4da6eb4`, NÃO o código atual.** A histerese angular da
mira (`6f707a3`) entrou **depois** delas — então o churn medido aqui é o do
código sem histerese. Voltas novas necessárias pra dizer se ele melhorou.

| arquivo | o que sustenta |
|---|---|
| `resumo_7_voltas.csv` | as 7 voltas desde o baseline, lado a lado |
| `dist_final_por_goal.csv` | o indicador do defeito 2e: onde o robô estava na **última amostra** antes de o plano trocar de goal. ⚠️ **não** é a pose no instante da conclusão nem o julgamento do checker — ver ressalvas abaixo |
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

**A correção faz o que foi desenhada pra fazer:** a última amostra antes da troca
de goal cai pra **3–9 cm** (era 11–15). Interferência do `unstuck` caiu.

⚠️ **Duas ressalvas do review (08-31), que valem pra toda leitura deste número:**
a 20 Hz e 0,22 m/s há **~1,1 cm entre amostras**, e o `goal_checker` do Nav2 é
**`stateful: true`** — satisfeito o XY uma vez, ele só reconfere yaw. Então esta
coluna **não prova** onde o checker aceitou, só onde o robô estava no último tick.

**Mas o item 2e não fechou.** A `aprox2` levou 328,6 s com ~54 s de giro perto dos
goals — *churn* de mira (aproxima, fecha o yaw, a deriva tira, re-aproxima),
bounded em 3 re-entradas, nunca laço infinito. E o `parado` do probe **conta
point-turn como parado**, então ele mede duas coisas juntas: a métrica boa é o
`dist_final_por_goal.csv`.

**Os 4 raspões da `aprox2`: contato durante point-turn de ROTA.** Estão em
`state=turning` a **6,15 m do goal** — o canto varrendo o `cone_3`, modo de falha
**já observado antes da aproximação existir** (item 1 dos abertos). ⚠️ **Não está
isolado** se a aproximação do goal anterior mudou pose/rumo de um jeito que
tornasse aquele giro mais provável. Em 7 voltas com latch, **6 limpas e 1 com
contato**: a causa nunca foi corrigida, só não tinha se alinhado.

**Teimoso, não investigado:** o goal 2 fica em 0,156–0,160 na última amostra das
**três** voltas — o que **não prova** que termina fora da tolerância (ressalvas
acima). Item 2h.

⚠️ Voltas até os **STANDOFFS**. Não provam A1, A2 nem A3.
