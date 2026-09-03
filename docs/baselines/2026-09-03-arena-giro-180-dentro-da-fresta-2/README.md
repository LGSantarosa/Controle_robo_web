# 2026-09-03 — giro de 180° DENTRO da fresta 2 (sim/Gazebo, máquina de dev)

Relato do dono: o robô ia passar do obstáculo 2, já tinha passado da metade, e
"do nada decide girar dentro do obstáculo, bate dos dois lados e arrasta tudo".

Análise completa: `DIARIO_ARENA.md` §2H.23. Item aberto: §6, `2n`.

## Resumo de uma linha
O `door_crossing` **atravessou certo** e soltou o robô em `exit_margin=0,5 m`
(traseira ainda no vão); o `path_follower` então executou um point-turn de ~180°
que já estava engatilhado, porque o **plano global do Nav2 nascia apontando pra
trás** (contorno pela própria fresta).

## Horas-chave (epoch)
- `874,825` — `door_crossing: rotating -> idle` (`_abort`, `align_timeout`) após
  11 ciclos `staging↔rotating` em 2,9 s com o robô parado
- `877,84` — re-arma (`idle -> rotating`), desgira e alinha
- `880,15` — `rotating -> crossing`; atravessa 1,14 m reto, yaw travado ~88,6°
- `885,8` — `path_follower` já manda `wz=4,22` com `herr=170,4°` (mux é do door)
- `886,273` — `crossing -> idle` (saída legítima, `s > 0,5`) + cooldown 8 s
- `886,5 → 891` — point-turn `vx=0`, `wz` 4,22→2,4; yaw 88,5° → −143°; `clear` 0,36 m
- `895,75` — `bt_navigator` cancela o goal

## Arquivos
- `follow_debug.csv` — estado/pose/`herr`/`wz` do `path_follower` a 20 Hz
- `follow_plan_last.csv` — **a prova**: 135 pontos, `idx 0..12` descendo pro SUL
- `nav2.log` — `No valid trajectories out of 35!`, `start or goal pose are an obstacle`
- `unstuck.csv`, `nav_metrics_19691231.csv`
