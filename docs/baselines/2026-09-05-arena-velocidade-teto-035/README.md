# BASELINE 2026-09-05 17:20-17:30 — velocidade com teto `max_vel_x = 0.35`

Ponto de comparação para a **fase VELOCIDADE reaberta**. Rota `teste7.json`
(3 waypoints), mapa `maps/oficial.yaml`, perfil `--arena` no robô real
(`./launch.sh --nav2 --arena --map=maps/oficial.yaml`), 3 voltas seguidas.

Motivo de existir, nas palavras do dono:

> "estamos precisos mais lentos, agora precisamos achar essa limiar entre
> precisão e velocidade"

A fase VELOCIDADE tinha saído de escopo em 2026-08-28 (o perfil arena baixou
`0.60 -> 0.35` com a meta "sem medo de movimento e MUITO preciso, sem limite de
tempo"). A precisão foi atingida. Agora se compra velocidade de volta, e **este
documento é o zero da régua**.

## Estado do código nesta corrida

Branch `arena-galpao`, HEAD `f747127`. Nada modificado — é o perfil arena como
commitado.

| parâmetro | arquivo | linha | valor |
|---|---|---|---|
| `FollowPath.max_vel_x` | `nav2_params_arena.yaml` | 223 | **0.35** |
| `FollowPath.max_speed_xy` | idem | 236 | **0.35** |
| `velocity_smoother.max_velocity[0]` | idem | 589 | **0.35** |
| `acc_lim_x` / `decel_lim_x` | idem | 238/240 | 1.0 / -1.0 |
| `sim_time` | idem | 256 | 1.2 |
| `vtheta_samples` | idem | 252 | 1 (DWB só anda reto) |
| `angular_dist_threshold` (shim) | idem | 176 | 0.30 rad (~17°) |
| `rotate_to_heading_angular_vel` | idem | 193 | 4.0 |
| `PolygonFront` | idem | 683 | `x 0.25..0.50`, `y ±0.22`, limit, `linear_limit 0.0` |
| `motion_guard` | — | — | **DESLIGADO** (`--arena` no robô real) |

## Os números (fonte: `pernas.csv`, extraído do log de `[NavMetrics]`)

| volta | perna | dist | tempo | média | replans | recoveries |
|---|---|---|---|---|---|---|
| 1 | 1 | 8.15 m | 27.5 s | 0.296 | 26 | 0/0/0 |
| 1 | 2 | 13.83 m | 52.0 s | 0.266 | 49 | 0/0/0 |
| 1 | 3 | 20.26 m | 62.6 s | 0.324 | 57 | 0/0/0 |
| 2 | 1 | 9.41 m | 44.9 s | 0.210 | 43 | 0/0/0 |
| 2 | 2 | 13.29 m | 43.8 s | 0.303 | 41 | 0/0/0 |
| 2 | 3 | 20.40 m | 65.0 s | 0.314 | 60 | 0/0/0 |
| 3 | 1 | 8.02 m | 24.8 s | 0.323 | 23 | 0/0/0 |
| 3 | 2 | 12.90 m | 42.3 s | 0.305 | 40 | 0/0/0 |
| 3 | 3 | 20.72 m | 68.5 s | 0.302 | 63 | 0/0/0 |

**Agregados — são estes que a próxima corrida tem que bater:**

| métrica | valor |
|---|---|
| volta 1 (soma nav) | 142.1 s / 42.24 m |
| volta 2 (soma nav) | 153.7 s / 43.10 m |
| volta 3 (soma nav) | 135.6 s / 41.64 m |
| volta 1 (relógio, 1º `WP_SEND` → último `WP_RESULT`) | 145.6 s |
| volta 2 (relógio) | 157.3 s |
| volta 3 (relógio) | 139.1 s |
| **volta mediana (relógio)** | **145.6 s ≈ 2'26"** |
| **velocidade média global** | **0.294 m/s** (126.98 m / 431.4 s) |
| melhor perna | 0.324 m/s (93 % do teto) |
| pior perna | 0.210 m/s (a do STALL) |
| recoveries em 9 pernas | **0** |
| status em 9 pernas | **9 × SUCCEEDED (4)** |

## A conclusão que este baseline estabelece

**O gargalo é o teto de 0.35, não a arena.**

Média global de **0.294 m/s num teto de 0.35 = 84 % de saturação** — e isso
*incluindo* os point-turns parados. A melhor perna fica a 93 % do teto. O robô
passa a rota praticamente inteira grudado no limite.

Não é costmap, não é planner, não é o reflexo:

- `replans=26` em 27.5 s ≈ **1 Hz** — é a taxa nominal do planner, não sintoma.
- `rec(b/s/w) = 0/0/0` nas **nove** pernas — nenhum backup, spin ou wait.
- 9/9 `SUCCEEDED`. Zero contato, zero travamento.

A pista está sobrando. Subir o teto converte diretamente em tempo.

## O degrau proposto: 0.35 → 0.50 m/s (NÃO 0.60)

Três números que **têm que andar juntos** — subir um só faz o outro cortar e o
ganho não aparece:

| onde | linha | de → para |
|---|---|---|
| `FollowPath.max_vel_x` | 223 | 0.35 → **0.50** |
| `FollowPath.max_speed_xy` | 236 | 0.35 → **0.50** |
| `velocity_smoother.max_velocity[0]` | 589 | 0.35 → **0.50** |

**Por que 0.50 é seguro sem tocar em mais nada:** o `PolygonFront` foi
redimensionado em 2026-08-27 (frente `0.40 -> 0.50`) **para 0.60 m/s**, e em
2026-08-28 a velocidade caiu pra 0.35 sem a caixa encolher de volta. Sobra
margem de frenagem. Conta a 0.50 m/s:

```
parada  = v²/2a = 0.25 / 2.0        = 0.125 m   (decel_lim_x -1.0)
reação  = scan 10 Hz + ciclo        ≈ 0.05  m
total                               ≈ 0.18  m
aviso da caixa (x 0.25..0.50)       = 0.25  m   -> ~7 cm de folga
```

A 0.60 a mesma conta dá ~0.25 m = **folga zero**. Por isso o degrau é 0.50.

**Não mexer** (um parâmetro por vez): `acc_lim_x` (1.0 já rampa até 0.50 em
0.5 s), `PolygonFront` (já dimensionado), `sim_time` (1.2 s × 0.50 = 0.60 m de
horizonte, sobra), nada de angular.

## Critério de aceite da próxima corrida

- **Tempo esperado:** volta mediana **145.6 s → ~110 s (≈1'50")**. Não é o
  escalonamento linear 0.35/0.50 (que daria 102 s): ~15 % do tempo é point-turn
  e rampa, que **não** escalam com o teto linear. Se o tempo *não* cair pra essa
  faixa, alguma coisa está capando — é diagnóstico, não motivo pra subir mais.
- **Precisão — o que reprova:** qualquer `rec(b/s/w)` diferente de `0/0/0`, ou
  qualquer `status` != 4. O baseline é 0 e 9/9; a régua é essa.
- **Sinal de perda de precisão a olho:** o DWB está com `vtheta_samples: 1` —
  ele **só anda reto**, todo giro é point-turn do RotationShim, disparado quando
  o erro de heading passa de 0.30 rad (~17°). Mais rápido = mais metros
  percorridos antes da correção disparar = desvio maior da linha. Se aparecer
  zig-zag ou corte de quina, o próximo botão é `angular_dist_threshold`
  0.30 → 0.22 — **não** voltar a velocidade.

## Achado colateral: o custo real está no giro, não na reta

A perna mais lenta das nove (volta 2 perna 1, **0.210 m/s**, 44.9 s pra 9.41 m
contra 24.8 s pra 8.02 m na volta 3) é exatamente a que traz:

```
17:25:14 [PowerMonitor] STALL: vF=40.6V vR=40.0V setL/R=360/-360
         meas=(74.0, -40.0, 0.0, -76.0)
```

`±360` é um point-turn comandando **3.6 rad/s**. O chassi precisa de ~6.0
(±600 unid/roda) pra vencer o atrito de repouso — está documentado no próprio
`nav2_params_arena.yaml` (linha ~186) e em `[[project_hover_enable_gira_na_mao]]`.
O `rotate_to_heading_angular_vel` está em **4.0** (foi baixado 4.2 → 4.0 em
2026-06-19 por causa do grip das fitas). Resultado: patina, não gira, perde
~20 s na perna.

**Onde se perde tempo e precisão hoje é o giro parado, não a reta.** Próximo
alvo depois de fechar a velocidade linear. Não mexer agora — um parâmetro por vez.

## Arquivos

- `pernas.csv` — as 9 pernas, machine-readable, pra diff com a próxima corrida.
- `log_launch.txt` — o log completo do `launch.sh` desta sessão (fonte de tudo).

O `nav_metrics_*.csv` desta corrida ficou **na Pi** — copiar de
`~/workspace/Controle_robo_web/controle_web/logs/nav_metrics/` pra cá se quiser
os campos que o log não imprime (`max_linear_speed`, `time_stopped_s`,
`direction_reversals`, pose final real). Esses três últimos são justamente os
que vão medir o custo do point-turn no próximo degrau.
