# PROVA versionada: o plano colapsou para **1 pose**

O `follow_plan_last.csv` é sobrescrito a cada `/plan` (volátil). Este é o conteúdo
**íntegro** no instante do travamento, arquivado a pedido do review do Codex:

```
x,y
6.353516069996635,2.2401137971071705
```

**Uma linha de dados. Um ponto.**

Confere com a pose congelada do robô no `follow_debug.csv`: `(6,354 / 2,240)`.

Isso fecha a cadeia da §2H.40:

`len(path) == 1` -> o guard `len(path) < 2` do `path_follower.update()` dispara ->
`idle` -> o `goal_turn` morre no meio (yaw 39,6° e girando a −2,4 rad/s no tick
anterior) -> o robô fica fora do `yaw_goal_tolerance` (20,05°) -> o Nav2 nunca fecha
o goal -> o app nunca manda o próximo -> travado.

Para comparação, `follow_plan_first.csv` (primeiro plano longo do mesmo goal) tem
**30 poses**.
