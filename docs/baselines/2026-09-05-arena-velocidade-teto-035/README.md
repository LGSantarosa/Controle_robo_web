# BASELINE 2026-09-05 17:20-17:30 — velocidade com teto `max_vel_x = 0.35`

> ## ⛔ CORREÇÃO 2026-09-05 (revisão do dono) — LEIA ANTES DE USAR ESTE BASELINE
>
> **Os NÚMEROS medidos abaixo valem. A CONCLUSÃO original estava ERRADA e foi
> reescrita.** O que caiu, verificado no código:
>
> 1. **Quem dirige não é o DWB.** `nav2.launch.py:225` sobe o `path_follower`
>    sem `condition`, e o `twist_mux_auto.yaml` dá a ele prioridade **15**
>    contra **10** do `nav_vel`. O teto real da corrida é
>    `path_follower.py:88 forward_speed = 0.30` — **não** `max_vel_x = 0.35`.
>    Os três parâmetros propostos na versão original governam a cadeia que
>    PERDE o mux; mexer neles não teria acelerado o robô.
> 2. **A conta de frenagem era inválida.** Ela usava `decel_lim_x = -1.0` do
>    `velocity_smoother`, e o `follow_vel` **não passa** pelo smoother (vai
>    direto pro `twist_mux_auto`). A "folga de 7 cm" não descreve o robô que
>    andou.
> 3. **`SUCCEEDED` + `rec=0/0/0` NÃO provam ausência de colisão.**
>    `HANDOFF_NAV2_TREKKING.md:66` documenta uma corrida **8/8 goals com 11
>    colisões**. A frase "zero contato" foi removida. E o `colisao.py` é
>    ground truth do **Gazebo** — no robô real ele NÃO roda; ver a tabela de
>    medição possível no critério de aceite.
> 4. **A métrica de velocidade não prova saturação.** `nav_metrics.py:301`
>    integra pose do `/odom` (slip de skid-steer). A perna de **0.324 m/s**
>    excede o comando máximo de 0.30 do follower — ou é erro de escala da
>    odom, ou o follower parou de publicar (timeout 0,5 s) e o DWB assumiu.
>    **Este log não distingue os dois.**
>
> Ver §"Quem realmente dirige" e §"O degrau correto" abaixo.

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
| `path_follower.forward_speed` ⬅ **TETO REAL** | `path_follower.py` | 88 | **0.30** |
| `path_follower.min_speed` | `path_follower.py` | 247 | 0.22 |
| `clear_full` / `clear_min` (LIGADO por `--arena`) | `launch.sh` | 634 | 1.2 / 0.35 |
| `FollowPath.max_vel_x` (cadeia `nav_vel`, perde o mux) | `nav2_params_arena.yaml` | 223 | 0.35 |
| `FollowPath.max_speed_xy` (idem) | idem | 236 | 0.35 |
| `velocity_smoother.max_velocity[0]` (idem) | idem | 589 | 0.35 |
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
| melhor perna | 0.324 m/s ⚠️ **acima do comando máx 0.30 do follower** — ver correção |
| pior perna | 0.210 m/s (a do STALL) |
| recoveries em 9 pernas | **0** (≠ ausência de colisão — ver correção) |
| status em 9 pernas | **9 × SUCCEEDED (4)** |

## Quem realmente dirige (verificado 2026-09-05)

```
path_follower(follow_vel, prio 15)  ─┐
door_crossing(door_vel,   prio 20)  ─┤─> [twist_mux_auto] -> auto_vel_raw
velocity_smoother(nav_vel, prio 10) ─┘        -> [collision_monitor] -> auto_vel
        ^ DWB/RotationShim                    -> [twist_mux FINAL] -> cmd_vel
          PERDE pro follower
```

- `nav2.launch.py:225` — o `path_follower` sobe **sem `condition`** (os nós
  condicionais deste arquivo usam `IfCondition`; ver linhas 254 e 284). Ele está
  vivo em toda corrida `--nav2`.
- `twist_mux_auto.yaml` — `follower` **15** > `navigation` **10**. Enquanto o
  follower publicar, o `controller_server` é ignorado. O próprio comentário do
  launch diz isso: *"Publica follow_vel (prio 15 no twist_mux, > nav_vel:
  ignora o controller_server)"*.
- `path_follower.py:88` — `forward_speed = 0.30`. **Este é o teto da corrida.**
- `launch.sh:634` — com `--arena`, `follow_clear_full:=1.2`. Isso LIGA o
  `speed_for_clearance` (`path_follower.py:284`), que interpola o cruzeiro entre
  `min_speed 0.22` e `forward_speed 0.30` sempre que a folga frontal < 1.2 m.
  Logo o teto efetivo **não era 0.30 achatado — era 0.30 modulado pra baixo**
  em boa parte da rota.
- `follow_vel` **não passa** pelo `velocity_smoother`. `acc_lim_x`/`decel_lim_x`
  do YAML não descrevem a aceleração física desta corrida.

## O que este baseline PODE e NÃO PODE afirmar

**Pode:** 9/9 `SUCCEEDED`, 0 recoveries, os tempos e as distâncias de odometria
por perna, e a volta mediana de 145.6 s. É um registro reprodutível do estado
`forward_speed = 0.30`.

**Não pode:**

- *"Zero contato."* `rec=0/0/0` e `status=4` não medem colisão.
  `HANDOFF_NAV2_TREKKING.md:66` traz uma corrida **8/8 goals + 11 colisões**.
  Contato aqui não foi medido por instrumento nenhum: o `colisao.py` é
  ground truth do Gazebo e não roda no real.
- *"84 % de saturação."* A distância vem de integração de pose do `/odom`
  (`nav_metrics.py:301`), sujeita a slip. E a perna de **0.324 m/s excede o
  comando máximo de 0.30** — ou a odom infla, ou houve trecho em que o follower
  parou de publicar (timeout 0.5 s) e o DWB assumiu. O log não separa os casos.

O que sobra, honestamente: o robô parece rodar **perto do teto do follower**,
mas o número exato não está provado e o teto de comparação é **0.30**, não 0.35.

## O degrau correto (revisão do dono, 2026-09-05)

**`path_follower.forward_speed`: 0.30 → 0.35.** É o parâmetro que efetivamente
manda no robô, e 0.35 apenas o alinha aos tetos que a cadeia `nav_vel` já tem —
nada mais precisa se mover.

**NÃO** subir `max_vel_x` / `max_speed_xy` / `velocity_smoother.max_velocity[0]`:
governam a cadeia que perde o mux. Deixá-los em 0.35 mantém o fallback coerente.

Implementação: `forward_speed` é param ROS declarado (`path_follower.py:620`)
mas **não tem launch arg** — só `clear_full`/`clear_min` têm. Preferir adicionar
um arg espelhando `follow_clear_full`, para o `--arena` setar sem alterar o
comportamento do `--nav2` normal.

Só depois de três voltas limpas a 0.35 se discute 0.40. **0.50 e 0.60 estão fora
de pauta** até existir medição de frenagem real.

## Critério de aceite da próxima corrida (`forward_speed = 0.35`)

A versão original pedia "~110 s por volta". **Descartado** — vinha da premissa
errada de que o teto era 0.35 no DWB.

### ⚠️ `colisao.py` NÃO serve aqui (achado do review 2026-09-05)

`tools/sim_ab/colisao.py` lê os `<collision>` de um **`<mundo.sdf>`** e cruza com
a pose **ground truth do Gazebo** (SAT contra OBB). No robô real não existe nem
SDF nem pose verdadeira — **ele não roda**. A versão anterior deste critério
pedia uma medição impossível.

### O que dá pra medir no robô real

Não há ground truth automático. O contato se mede **fisicamente**, e os dados
já logados corroboram:

| o quê | como | fonte |
|---|---|---|
| **Contato (ground truth)** | fita no chão marcando a base de cada cone ANTES da 1ª volta. Cone fora da marca no fim = tocou. É o único juiz de verdade. | olho + fita |
| **Instante do contato** | pico de corrente / `STALL` no `PowerMonitor` casado com o `t` da volta | `logs/power/power_*.csv` |
| **Quase-contato (raspão)** | menor `clear` da volta — coluna já gravada pelo follower | `follow_debug.csv` |
| **Distância de parada REAL** | achar o tick em que `vx` cai de 0.35 → 0 e integrar `hypot(dx,dy)` das colunas `x`,`y` até a pose congelar. Mede a frenagem FÍSICA — que é o número que falta pra sequer discutir 0.40. | `follow_debug.csv` |
| **Odometria confiável?** | `max_linear_speed` do NavMetrics acima do comando de 0.35 = a odom infla e a régua de velocidade precisa de outra fonte | `nav_metrics_*.csv` |

O `follow_debug.csv` tem `t,state,x,y,yaw_deg,…,dist_goal,vx,wz,la,clear` — as
duas medições que faltavam (parada real e folga real) saem **dele**, sem
instrumentação nova.

### Reprova

Qualquer `rec` != `0/0/0`, qualquer `status` != 4, ou **qualquer cone fora da
marca de fita**.

## Achado colateral: o STALL é do `path_follower`, e ele PROVA quem dirigia

A perna mais lenta das nove (volta 2 perna 1, **0.210 m/s**, 44.9 s pra 9.41 m
contra 24.8 s pra 8.02 m na volta 3) traz:

```
17:25:14 [PowerMonitor] STALL: vF=40.6V vR=40.0V setL/R=360/-360
```

`±360` unid/roda = **ω = 3.6 rad/s**. Isso é exatamente
`path_follower.py:633 rot_min = 3.6` — o piso do `_turn_cmd`
(`mag = min(rot_max, max(rot_min, |herr|·rot_k))`, linha 343), subido de
2.4 → 3.6 hoje mesmo no commit `a8b35fd`.

**Não é** o `rotate_to_heading_angular_vel = 4.0` do RotationShim (que daria
±400 e pertence à cadeia que perde o mux). Ou seja: o próprio log confirma, pelo
número no barramento, que **quem estava dirigindo era o `path_follower`** — é a
evidência independente da revisão no topo deste documento.

O chassi precisa de ~6.0 rad/s (±600) pra vencer o atrito de repouso; o pivô
arranca em 3.6, patina e não gira. Custou ~20 s nessa perna.

**Onde se perde tempo e precisão hoje é o pivô parado, não a reta.** O botão é
`rot_min` (piso de arranque), não `rotate_to_heading_angular_vel`. Alvo seguinte
depois de fechar o `forward_speed` — um parâmetro por vez.

## Arquivos

- `pernas.csv` — as 9 pernas, machine-readable, pra diff com a próxima corrida.
- `log_launch.txt` — o log completo do `launch.sh` desta sessão (fonte de tudo).

O `nav_metrics_*.csv` desta corrida ficou **na Pi** — copiar de
`~/workspace/Controle_robo_web/controle_web/logs/nav_metrics/` pra cá se quiser
os campos que o log não imprime (`max_linear_speed`, `time_stopped_s`,
`direction_reversals`, pose final real). Esses três últimos são justamente os
que vão medir o custo do point-turn no próximo degrau.
