# 2026-09-03 16:16 — PROVA de que o conserto do guard foi EXERCITADO

Volta completa, **7/7 goals**, 222,3 s. Primeira corrida em que o mecanismo do BO
aconteceu **e o robô sobreviveu**.

## O trecho que fecha o caso (waypoint 6, cone_4)

O plano global colapsou para **1 pose** no meio do `goal_turn`, com **44,1° de erro
de yaw ainda por fazer**. Antes do conserto isso era `idle` instantâneo e travamento
permanente. Aqui o giro continuou:

```
t=192,93  goal_turn  n=1  yaw=133,50  herr=44,10  wz=2,4
t=193,50  goal_turn  n=1  yaw=144,40  herr=33,20  wz=2,4
t=194,03  goal_turn  n=1  yaw=156,50  herr=21,10  wz=2,4
t=194,09  goal_turn  n=1  yaw=157,60  herr=20,00  wz=2,4
t=194,10  ---------- Goal succeeded ----------
t=195,63  Begin navigating -> (1.50, 2.50)
```

**22 ticks (1,16 s) girando com plano de 1 pose**, fechando 24° de erro, e o goal
concluiu no tick seguinte.

Comparar com as 3 corridas travadas, que morreram exatamente neste ponto:
`yaw` 32,2° / 23,7° / 38,0° de erro restante, `idle`, e a volta parada para sempre.

## Números da volta

| | |
|---|---|
| goals | **7/7 SUCCEEDED** |
| duração | 222,3 s |
| `FOLLOW_IDLE motivo=path_short` | **nenhum** (a chegada absorveu o colapso) |
| `FOLLOW_IDLE motivo=goal_inactive` | 7, todos benignos (entre goals) |
| `goal_turn` com `n=1` | **22 amostras** |

Um dos `goal_inactive` saiu com `n_plan=1`: o goal fechou com o plano já em 1 pose —
a mesma situação que travava antes.

## `exit_straight` (pacote da saída reta)

| # | andou | `wz` máx | terminou por |
|---|---|---|---|
| 1 | 0,79 m | **0,00** | distância |
| 2 | 0,34 m | **0,00** | guarda de folga (`clear` 0,51) |
| 3 | 0,80 m | **0,00** | distância |

Zero pivô perto do vão nos três. O #2 pela guarda de folga é a **terceira** corrida
seguida — item `2p` (janela armando após *abort*) confirmado como consistente.

## Portas

| porta | total | preparação | travessia | tentativas |
|---|---|---|---|---|
| 1 | 8,1 s | 1,4 s | **6,8 s** | 1 |
| 2 | 31,2 s | **24,8 s** | **6,4 s** | 3 |

Travessia estável em 6-7 s. O custo segue na preparação da porta 2 (item `2q`).
