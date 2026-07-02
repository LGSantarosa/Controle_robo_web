# motion_guard — cautela com objeto EM MOVIMENTO perto do robô (2026-07-02)

## Problema

Pedido do dono pós-run 07-02: nada na stack distingue móvel de estático. O
collision_monitor é reativo instantâneo (freia quando algo JÁ está na frente);
com pessoa/animal se aproximando o robô segue em velocidade cheia até o último
momento. Desejo: desacelerar perto de coisa em movimento e PARAR se ela estiver
cruzando a frente, retomando sozinho quando passar.

## Decisões de escopo (fechadas com o dono)

- Atua SÓ na autonomia (nav2 + path_follower). Manual e unstuck ficam fora.
- Para e RETOMA SOZINHO (corredor limpo por `clear_time` → volta a andar).
- Abordagem C: detector de movimento SEM predição agora, mas já clusterizando
  os pontos móveis — se em campo reagir tarde, pluga-se estimativa de
  velocidade + predição de cruzamento em cima (vira a proposta B) sem reescrever.

## Arquitetura

Nó novo `robot_nav/motion_guard.py`, padrão do repo: classe de lógica **pura**
(`MotionGuard`, testável sem ROS) + `main()` de cola (TF, tópicos, CSV).

Filtro de velocidade na cadeia da autonomia (mesmo padrão do collision):

```
nav_vel/follow_vel/door_vel → twist_mux_auto → auto_vel_pre
    → motion_guard → auto_vel_raw → collision_monitor → auto_vel → mux final
```

Só muda o remap do `twist_mux_auto` no launch (`cmd_vel_out`: `auto_vel_raw` →
`auto_vel_pre`) + o nó novo. YAML do collision intocado. É nó rclpy simples sem
lifecycle/bond — não repete o ponto-único-de-falha do bringup nav2 de 06-26.

## Detecção (frame odom, sem velocidade por cluster)

A cada `/scan_safe` (~10 Hz):

1. Transforma os retornos pra `odom` via TF (timestamp do scan).
2. Snapshot em grade grossa (`grid_res` 0,15 m) num ring buffer; compara com o
   snapshot de ~`lookback` 0,5 s atrás.
3. Ponto atual é **móvel** se a célula dele (e vizinhas imediatas) estava LIVRE
   no snapshot antigo — borda de ataque do objeto. Parede e móvel parado ficam
   na mesma célula → não disparam.
4. Clusteriza móveis por vizinhança (gap 0,3 m); cluster < `min_cluster_points`
   (3) = ruído, descarta. Só considera raio ≤ `guard_radius` (2,5 m).
5. **Gate de giro**: |wz| medido do robô > `wz_gate` (0,3 rad/s) → não avalia
   (o scan inteiro "anda" quando o robô gira); segura a última decisão e decai
   pra livre após `hold_timeout` (1,0 s) sem avaliação.

## Atuação (só no vx — NUNCA no wz)

- Cluster móvel no raio → `linear.x *= slow_scale` (0,5) → estado `slowing`.
- Cluster móvel dentro do **corredor à frente** (retângulo `±corridor_half_w`
  0,35 m × `corridor_len` 1,5 m em base_link, avaliado só quando o comando tem
  vx > 0) → `linear.x = 0` → estado `blocked`. Retoma quando o corredor fica
  sem móvel por `clear_time` (1,5 s).
- `angular.z` passa INTOCADO sempre: escalar wz jogaria o giro pra baixo da
  zona-morta 1,7 rad/s e congelaria os point-turns (lição do rot_min 07-02).
- Sem comando entrando, nada sai (filtro puro; não "segura o mux").

## Failsafe e observabilidade

- TF indisponível, scan ausente/velho > 1 s, ou `enabled=false` → PASS-THROUGH
  (repassa o comando intocado, WARN throttled). O guard NUNCA mata a autonomia.
- Estado latched em `/motion_guard/state` (`idle|slowing|blocked|passthrough`).
- CSV `controle_web/logs/motion_guard.csv`: t, estado, n_clusters_móveis,
  dist_do_mais_perto, no_corredor, vx_in, vx_out — EU leio pós-run.
- Params com callback de reconfiguração (live-tuning real, lição do `04bcf86`).

## Parâmetros (defaults)

`enabled=true, guard_radius=2.5, slow_scale=0.5, corridor_half_w=0.35,
corridor_len=1.5, clear_time=1.5, grid_res=0.15, lookback=0.5,
min_cluster_points=3, wz_gate=0.3, hold_timeout=1.0, scan_stale=1.0`

## Testes e validação

- pytest da lógica pura: objeto móvel sintético detectado; parede parada NÃO
  dispara; móvel no corredor → vx=0; retomada após clear_time; gate de giro
  segura decisão; pass-through sem TF/scan; wz nunca alterado; enabled=false.
- Sim: caixa móvel cruzando a rota (plugin velocity-control do gz — ator
  animado não aparece no gpu_lidar). Robô desacelera → para → retoma.
- Real: fora deste escopo (deploy/validação é decisão do dono depois).

## Fora de escopo (YAGNI)

Estimativa de velocidade por cluster e predição de cruzamento (proposta B) —
só se a versão A reagir tarde em campo. Chip na UI web. Atuação no manual.
