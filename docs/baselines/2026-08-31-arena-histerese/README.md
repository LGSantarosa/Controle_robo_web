# Histerese angular da mira — 3 voltas, 2026-08-31

Evidência da §2B.7 do `DIARIO_ARENA.md`. Mesmo comando da §4.5, tags `hist1..3`,
com o commit `6f707a3` (histerese na mira da aproximação) sobre o `4da6eb4`.

Estas voltas foram rodadas **porque as `aprox1..3` não valem para o código
atual** — elas mediram o churn de mira de antes da histerese existir.

| arquivo | o que sustenta |
|---|---|
| `resumo_10_voltas.csv` | as 10 voltas nossas + o baseline, lado a lado |
| `dist_final_por_goal.csv` | onde o robô estava na **última amostra** antes de o plano trocar de goal. ⚠️ **não** é a pose no instante da conclusão nem o julgamento do checker (ver as ressalvas do README da `aproximacao`) |
| `colisao_3voltas.csv` | contato por objeto e por volta — **0 COLISÃO e 0 raspão nas 3** |
| `transicoes_goal_turn_3voltas.csv` | toda troca de/para as fases de chegada |
| `unstuck_disparos_3voltas.csv` | os disparos do unstuck |
| `churn_mira.csv` | 🆕 as alternâncias mira↔avanço — **7 / 17 / 11**, contra **28 / 90 / 37** das `aprox` (o mesmo arquivo existe lá) |
| `guard_bloqueio.csv` | 🆕 as paradas longas por **duas fontes independentes**: o estado do `motion_guard` e a pose do ground truth |
| `guard_bloqueio_11voltas.csv` | 🆕 o mesmo, para **todas as 11 voltas** — é o que mostra que só as voltas com aproximação bloqueiam |

Gerados por (o script é versionado, tem autoteste e não escreve nada se faltar
bruto):

```
python3 tools/sim_ab/extrai_evidencia.py \
    docs/baselines/2026-08-31-arena-histerese hist1 hist2 hist3
python3 tools/sim_ab/extrai_evidencia.py --resumo \
    docs/baselines/2026-08-31-arena-histerese/resumo_10_voltas.csv \
    arena_baseline1 arena_latch1 latchN1 latchN2 latchN3 \
    aprox1 aprox2 aprox3 hist1 hist2 hist3
python3 tools/sim_ab/extrai_evidencia.py --guard \
    docs/baselines/2026-08-31-arena-histerese/guard_bloqueio_11voltas.csv \
    arena_baseline1 arena_latch1 latchN1 latchN2 latchN3 \
    aprox1 aprox2 aprox3 hist1 hist2 hist3
```

## O que estas 3 voltas dizem

**✅ A histerese fez o que foi desenhada pra fazer, na métrica dela.** As
alternâncias mira↔avanço dentro de `goal_approach` caíram de **28/90/37** para
**7/17/11** — corte de 3 a 5×. Contato **0 em 3/3**, samba **0 em 3/3**.

**❌ E o tempo parado NÃO caiu** (1,0 / 26,7 / 30,7 s). Então o churn de mira
**não era** a causa das paradas longas. Eu tinha atribuído a ele — é o BO 65.

**🔴 A causa dos piores casos é o `motion_guard`.** Ele fica `blocked` e **zera o
comando** entre `auto_vel_pre` e `auto_vel_raw`: na `hist3`, 505 comandos
entraram e **1** saiu, com a pose do ground truth **idêntica por 26,9 s**. Os 3
episódios das 11 voltas duram 25,7–26,9 s = `hold_still_max` 20 s + `clear_time`
5 s + settle, e nos 3 o único objeto ao alcance era um **cone**. O mundo não tem
`<actor>` nenhum: o vigia de pessoa está segurando cone. Detalhe do §2B.7.

⚠️ **O que este extrato NÃO prova:** o centróide da vigília (`_watch`) não é
publicado por nenhum tópico, então não há como exibir a coordenada que o guard
estava vigiando. O que está medido é a duração, o comando zerado, a pose
congelada e o vizinho.

⚠️ **As duas bases de tempo são diferentes.** `guard_blocked` conta a partir da
primeira linha do `freeze_capture.csv`; `pose_congelada`, a partir do início do
`colisao.csv` (que começa antes). Não some as duas colunas: o que liga os pares é
a **duração**.

⚠️ Voltas até os **STANDOFFS**. Não provam A1, A2 nem A3.
