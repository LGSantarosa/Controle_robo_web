Corrida salva em 2026-09-03 para embasar a regressão aberta na frente do obstáculo 2.

Contexto:
- base restaurada para o estado anterior às mudanças locais de yaw do goal
- commit-base preservado: `3c2b051`
- baseline boa para comparar: `docs/baselines/2026-09-02-arena-door-crossing-2obstaculos-boa-corrida`

O que foi revertido hoje:
- `controle_web/map_service.py`: removido o override por waypoint de `goal_yaw_tol` e o `'_goal_yaw_tol'` do ponto pré-porta
- `ros2_packages/robot_nav/robot_nav/path_follower.py`: `goal_yaw_tol` voltou para `0.10` rad (~6°), sem subscriber dinâmico em `goal_yaw_tol`
- `ros2_packages/robot_nav/config/nav2_params_arena.yaml`: `yaw_goal_tolerance` voltou para `0.35`
- `controle_web/test_map_service_waypoints.py`: removido o teste do repasse de `goal_yaw_tol` por waypoint

Sinais desta corrida:
- `follow_debug.csv`: 5152 amostras em 285.577 s
- estados no `follow_debug.csv`: `turning=4141`, `driving=923`, `goal_approach=54`, `goal_turn=34`
- tempo em `turning`: 275.135 s (80.4% da corrida)
- maior bloco contínuo em `turning`: 94.438 s, de `1788442862.683` até `1788442957.121`
- nesse bloco, `yaw_deg` variou de `-37.9` a `167.6`, `herr_deg` de `-41.0` a `128.9`, `dist_goal` de `0.150` a `4.519`
- último segmento do `nav_metrics_tail_ultima_corrida.csv`: `nav_id=597a7919`, `ABORTED`, `200.17 s`, `177` replans, `2` backups, `2` spins, `2` waits, `17.096 m` percorridos, `115.262 s` parado

Eventos de porta no fim da corrida:
- entre `1788442924.408` e `1788442951.726`, a `porta 2` repetiu `staging -> rotating -> reversing -> staging -> idle`
- durante esse trecho o `door_crossing` logou `goal_succ=False` e `nav_fwd=True`, ou seja, a travessia não assumiu conclusão do goal e o Nav2 continuou reaplanando
- em `1788442941.055` o `controller_server` abortou com `Failed to make progress`

Arquivos salvos:
- `follow_debug.csv`
- `nav2.log`
- `nav_metrics_19691231.csv`
- `nav_metrics_tail_ultima_corrida.csv`
- `light_events.log`

Hipótese mínima sustentada por este snapshot:
- afrouxar o yaw do goal e ainda tentar alinhamento fino perto da porta piorou a estabilidade antes do obstáculo 2; o sistema ficou grande parte do tempo em giro/replan sem converter isso em progresso
