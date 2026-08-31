# `motion_guard` DESLIGADO — 3 voltas, 2026-08-31

Evidência da §2B.9 do `DIARIO_ARENA.md`. Comando da §4.5 **com
`motion_guard:=false`** no `AB_EXTRA_LAUNCH`, tags `noguard1..3`.

Conferência de que o guard realmente não subiu (dá **0** nas 3):

```
grep -c motion_guard log/sim_ab/noguard1/nav2.log
```

| arquivo | o que sustenta |
|---|---|
| `resumo_13_voltas.csv` | as 13 voltas nossas + o baseline, lado a lado |
| `colisao_3voltas.csv` | contato por objeto e por volta — **é onde aparece o BO da `noguard3`** |
| `dist_final_por_goal.csv` | última amostra antes da troca de goal (mesmas ressalvas dos outros baselines) |
| `transicoes_goal_turn_3voltas.csv` | toda troca de/para as fases de chegada |
| `unstuck_disparos_3voltas.csv` | os disparos do unstuck — inclui o `near` aos 50,8 s da `noguard3` |
| `churn_mira.csv` | alternâncias mira↔avanço (5 / 5 / 11) |
| `guard_bloqueio_14voltas.csv` | as paradas longas nas **14** voltas: nenhuma linha `guard_blocked` nas `noguard*`, porque não há guard |

## O que estas 3 voltas dizem

**✅ O ganho é real e grande.** `parado` = **0,0 s em 14 dos 15 goals**. A
`noguard2` (221,0 s) é a **volta mais rápida das 14** e a `noguard1` a segunda.
A assinatura de parada de ~27 s da §2B.7 sumiu.

**🔴 E a `noguard3` BATEU.** 9 COLISÃO + 48 raspões, folga **0,0000**
(penetração), 58 eventos entre t=60,7 e 63,6 — todos na `A_fresta90_2`. É o pior
contato desde o `arena_baseline1` e o **segundo** contato em 14 voltas.

**Foi o guard-off que causou?** O medido vai contra, mas **não fecha**:

- a fresta A **sempre** foi passagem no fio: folga mínima 0,045–0,212 m nas 14
  voltas, abaixo de 8 cm em 4 delas;
- nas 11 voltas com guard, o estado dele na travessia da fresta foi `idle` em
  **todas** — os únicos `slowing` do histórico somam ~6 s e foram longe dali
  (muro oeste 1,29 m, `C_fresta60_1` 0,95 m, `cone_4` 0,48 m);
- a `noguard3` cruzou a fresta **atrasada e torta**: t=60,9 e yaw **−5,4°**,
  contra t=35–45 s e yaw −13° a −26° nas outras 13. Antes disso o `unstuck`
  disparou (`near`) aos 50,8 s, ainda no goal 1.

⚠️ **n=3.** Não dá pra tirar taxa de contato de três voltas, e "o guard não
atuava ali" não é "tirar o guard não muda nada em lugar nenhum". **Guard-off NÃO
está validado** — está medido. Ver item 2k dos abertos.

⚠️ Voltas até os **STANDOFFS**. Não provam A1, A2 nem A3.
