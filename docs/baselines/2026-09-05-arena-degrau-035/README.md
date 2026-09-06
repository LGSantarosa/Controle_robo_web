# DEGRAU 2026-09-05 19:35-19:42 — `forward_speed = 0.35` no robô real

Primeira corrida do degrau `0.30 → 0.35`. Mesma rota, mesmo mapa e mesmo perfil
do baseline de 17:20 (`../2026-09-05-arena-velocidade-teto-035/`), que é o zero
da régua.

Subido **pelo dono**: `./launch.sh --nav2 --arena --map=maps/oficial.yaml`
(sem flag = default 0.35 do perfil arena). Código: `a9de745`.

## VEREDITO CURTO

| pergunta | resposta |
|---|---|
| O degrau chegou nos motores? | **Sim.** `cmd_vel` máx = **0.350**, exato. |
| Ficou mais rápido? | **Sim.** Volta 145.6 s → **125.75 s (−13,6 %)**. |
| Foi seguro? | **NÃO SE SABE.** A frenagem não foi medida — ver §3 (retratação). |
| Bateu? | Sem `rec`, sem STALL. **Contato só o dono sabe** (fita nos cones). |
| Aconteceu algo? | **3 ABORTs no início**, por stall de TF/scan de ~20 s. |

## 1. O degrau valeu ponta a ponta (medido no barramento)

Do `freeze_capture.csv`, comando por elo da cadeia — **não** odometria:

| tópico | n | `vx` máx | `vx` médio andando | `wz` máx |
|---|---|---|---|---|
| `follow_vel` (path_follower) | 2462 | **0.350** | **0.345** | 6.750 |
| `nav_vel` (DWB/shim/smoother) | 2001 | 0.350 | 0.234 | 6.000 |
| `auto_vel_raw` | 2505 | 0.350 | 0.345 | 6.750 |
| `auto_vel` (pós collision) | 2463 | 0.350 | 0.345 | 6.750 |
| **`cmd_vel` (motores)** | 2645 | **0.350** | **0.341** | 6.750 |

Três leituras:

1. **O teto novo é respeitado e nada o corta.** `cmd_vel` nunca passou de 0.350.
2. **Confirmação independente de quem dirige:** `wz` máx do `follow_vel` é
   **6.750** = `rot_max` do `path_follower`; o `nav_vel` satura em **6.000** =
   `max_vel_theta` do DWB. O valor que chega no `cmd_vel` é 6.750 — o follower
   ganha o mux, como o review estabeleceu.
3. **O follower roda saturado:** 0.345 andando contra teto 0.350 = **98,6 %**.
   Desta vez medido no COMANDO, não na odometria — a afirmação de saturação que
   o baseline não podia fazer, agora está feita com o instrumento certo.

## 2. A odometria infla — agora com número

`odom` reportou `vx` máx de **0.593 m/s**, com o comando máximo jamais emitido
sendo 0.350. **+69 %.** O `max_linear_speed` do NavMetrics por perna (0.593 /
0.586 / 0.577) diz o mesmo.

Isto fecha o critério que o baseline deixou aberto: *"se `max_linear_speed`
seguir acima do comando, a odom infla e a régua precisa de outra fonte"*. **Ela
infla.** Toda `dist`/`avg_linear_speed` deste projeto — inclusive as do baseline
de 17:20 — são otimistas. Servem para COMPARAR corridas entre si (o viés é o
mesmo), nunca como valor absoluto.

## 3. ⛔ FRENAGEM: A MEDIÇÃO NÃO EXISTE (retratação, 2026-09-06)

**A versão original desta seção alegava "31 paradas, máx 0.185 m, margem de
6,5 cm". Isso está ERRADO e foi retirado.** Auditoria do Codex, confirmada aqui:

| das 31 transições `cmd_vel` ≥0.30 → 0 | n | o que era de fato |
|---|---|---|
| viraram **pivô** (`wz` até 3.60 = `rot_min`) | **29** | o deslocamento medido é a **deriva lateral do pivô**, não frenagem |
| **paradas lineares** (`wz` ≈ 0) | **2** | e as duas são **chegada de goal**, com a rampa do `slow_radius` — não freada de emergência |

As duas lineares, medidas até a odom realmente zerar:

| | desloc | `v0` (odom) | parou em |
|---|---|---|---|
| parada 1 | 0.184 m | 0.457 | 0.79 s |
| parada 2 | 0.115 m | 0.285 | 0.66 s |

**Tentativa 2 — os cortes do `collision_monitor`** (`follow_vel` ≥0.30 →
`auto_vel` = 0), que seriam as freadas de emergência de verdade: **32
episódios**, mas a velocidade da odom **no instante do corte** foi
0.00–0.14 m/s em 30 deles (máx 0.457 num só). Ou seja, o reflexo cortou quando o
robô **já estava quase parado** — nenhum corte pegou o robô a 0.35 de cruzeiro.
Os deslocamentos grandes (até 1.29 m) são o robô **retomando** a marcha dentro
da janela, não coasting.

**Conclusão honesta: não sabemos a distância de frenagem a 0.35.** Uma volta
normal não produz o evento necessário — o robô nunca foi cortado em velocidade
de cruzeiro.

**Como obter o número de verdade** (teste deliberado, não colheita de volta
normal):

1. Pista reta e livre, fita marcando o ponto de partida.
2. Robô a cruzeiro pleno (`clear` > `clear_full` = 1.2 m, senão o
   `speed_for_clearance` já reduziu antes).
3. Cortar: obstáculo entrando no `PolygonFront`, ou cancelar o goal pela UI.
4. Medir com **régua/fita** do ponto do corte até onde parou — 3 repetições.
   Fita, não odometria: a odom infla 69 % (§2), e `decel = v²/2d` herda esse
   viés, então nenhuma aritmética em cima dela fecha.

## 4. Os 3 ABORTs — stall de TF e scan, não lógica de navegação

Às 19:38:04 o dono mandou a rota e o waypoint 1 abortou **3 vezes** (`status=6`)
antes de ele parar pela UI. 1m22s depois reenviou e a volta saiu limpa.

Causa, do `nav2.log`:

```
19:38:08.8 [controller_server] Extrapolation Error ... map -> odom
           "latest data is at time 1788647888.671567"     <- TF CONGELOU aqui
19:38:09.1 [collision_monitor] [scan]: ... timestamps differ on 1.016161 seconds.
                               Ignoring the source.
19:38:09.1 [collision_monitor] Robot to stop due to invalid source.
19:38:08.8 [controller_server] Control loop missed its desired rate of 10 Hz.
                               Current loop rate is 8.4289 Hz.
```

A janela vai de **1788647888.78 → 1788647909.34 = ~20,5 s**. Depois disso:
**zero** ocorrências — a volta boa (19:39:44 em diante) não tem nenhuma.

- 22 × `Extrapolation Error` (TF `map→odom` parado)
- 41 × `Ignoring the source` (scan >1 s velho)
- 12 × `Control loop missed its desired rate`

Ou seja: o `map→odom` (AMCL) e o `/scan` pararam de atualizar ao mesmo tempo por
~20 s; o `collision_monitor` fez o certo (fonte inválida → para), o controller
não conseguiu transformar, e o `bt_navigator` abortou. **Não é bug de
navegação — é stall de sistema.** Mesma família do LiDAR que falha na tentativa
1 (aconteceu nas corridas de 17:20 e nas minhas de 19:23; nesta pegou de
primeira).

⚠️ **Risco aberto para a prova**, e não tem nada a ver com velocidade: se essa
janela de 20 s cair no meio da rota em vez de no arranque, o robô para e aborta.
Vale investigar CPU/carga na Pi antes do dia.

**Hipótese sobre o "voltar pro ponto 1":** o runner, por desenho (`d2e9eec`,
*"o runner NÃO desiste mais de ponto nenhum"*), **reenvia o mesmo waypoint** a
cada abort — aqui, 3 vezes seguidas para o ponto 1, com o robô já tendo andado
1.42 m. Visto da pista, isso é o robô insistindo/voltando no ponto 1. É
compatível com o sintoma relatado, mas **não é prova**: não há registro de que a
observação do dono tenha sido nesta corrida. Precisa da corrida em que ele viu.

## 5. Comparação com o baseline (0.30)

Volta boa vs volta 3 do baseline (mesmos 3 goals, mesma ordem):

| perna | 0.30 (baseline) | 0.35 (aqui) | Δ tempo |
|---|---|---|---|
| 1 | 24.8 s / 8.02 m | **23.88 s** / 8.02 m | −3,7 % |
| 2 | 42.3 s / 12.90 m | **38.30 s** / 13.19 m | −9,5 % |
| 3 | 68.5 s / 20.72 m | **59.95 s** / 21.01 m | −12,5 % |
| **soma nav** | 135.6 s | **122.13 s** | **−9,9 %** |
| **relógio da volta** | 145.6 s (mediana) | **125.75 s** | **−13,6 %** |

**Por que o ganho é 13,6 % e não os 16,7 % do teto:** **23,2 % do tempo o robô
está PARADO** (`time_stopped_s` = 6.24 + 8.24 + 13.87 = 28.35 s de 122.13 s).
Point-turn não escala com o teto linear. O baseline dava 15,8 % parado na perna
equivalente — a fração parada CRESCEU com a velocidade, o que faz sentido: a
parte andando encolhe e a parada não.

**Confirma o achado do baseline:** o custo está no pivô, não na reta. Com 23 %
do tempo parado, o próximo ganho grande é o giro — não subir mais o teto linear.

## 6. Saúde

- `rec(b/s/w) = 0/0/0` nas 3 pernas boas. 3/3 `SUCCEEDED`.
- `direction_reversals = 0` em todas.
- **Zero STALL** no `power_2026-09-05_193538.csv` (a corrida das 17:20 teve 1).
- `trip_front|trip_rear` no fim = o dono cortou a energia. Fim de teste normal.
- `scan_sanitizer` descartou fantasmas <0.23 m o tempo todo (3757→3933 no
  intervalo visto), comportamento esperado.

⚠️ **Contato não foi medido.** Não há instrumento no robô real; só a fita nos
cones e o olho do dono. `SUCCEEDED` + `rec=0` NÃO provam ausência de toque
(`HANDOFF_NAV2_TREKKING.md:66`: 8/8 goals com 11 colisões).

## Arquivos

| arquivo | o que tem |
|---|---|
| `freeze_capture.csv` | 1.86 MB — a cadeia inteira (`follow_vel`→`cmd_vel`) + `odom` + `follow_state`. É a fonte das §1, §2 e §3. |
| `follow_debug.csv` | estado do seguidor por tick (`vx,wz,clear,herr,dist_goal`) |
| `follow_plan_last.csv` | último plano recebido |
| `nav_metrics_20260905.csv` | as 6 tentativas + as do baseline de 17:20, mesma data |
| `power_2026-09-05_193538.csv` | tensões/correntes; 0 stalls, 1 trip final |
| `nav2.log` | 74 KB — a fonte da §4 (stall de TF/scan) |
| `robot_nodes.log` | MEGA bridge, odometria, cmd_vel→wheels |
| `pernas.csv` | as 6 tentativas resumidas, para diff com a próxima |

## ⏭️ DECIDIDO 2026-09-06: sobe pra 0.40

O dono decidiu subir depois deste resultado. Aplicado no `a…` seguinte
(`launch.sh`: default do `--arena` 0.35 → 0.40, faixa do `--follow-speed`
[0.22, 0.40]).

**A conta que o dono conhece e aceitou**, escalando o pior caso medido aqui
(frenagem ∝ v², reação ∝ v):

⛔ **A conta que eu apresentei ao dono para essa decisão estava furada** — ela
partia dos "6,5 cm de margem a 0.35" da §3, que a auditoria derrubou. **Não há
margem conhecida nem a 0.35 nem a 0.40**, porque a frenagem nunca foi medida.

O que se sabe de fato:

- o robô rodou uma volta inteira a 0.35 sem recovery e sem contato observado;
- o `collision_monitor` cortou 32 vezes, **sempre com o robô já lento** — o
  reflexo nunca foi exercitado em velocidade de cruzeiro, nem a 0.35;
- portanto o `PolygonFront` está **não testado** no regime que importa.

Alargar o `PolygonFront` (frente `0.50 → ~0.58`) continua sendo a mudança que
compraria folga, mas **decidir isso sem o número da frenagem é chutar** — pode
ser desnecessário, pode ser insuficiente. O teste de frenagem da §3 vem antes.

**Não subir de 0.40 sem alargar a caixa E medir de novo.**

## O que fazer a seguir

1. **Medir a frenagem de verdade** (§3), com fita, antes de qualquer decisão
   sobre caixa ou sobre subir de 0.40. É o dado que falta desde o começo.
2. **Atacar o pivô** — 23,2 % do tempo parado é o maior custo restante.
3. **Investigar o stall de 20 s** (CPU/carga na Pi). É o único risco desta
   corrida que pode matar uma volta na prova.
4. **Repetir 2-3 voltas a 0.35** antes de tratar estes números como estáveis:
   isto é UMA volta.
