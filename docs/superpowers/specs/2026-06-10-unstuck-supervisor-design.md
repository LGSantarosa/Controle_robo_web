# Design: `unstuck_supervisor` — watchdog de desencalhe do Nav2

**Data:** 2026-06-10
**Status:** aprovado, pronto pra implementar

## Problema

O robô empaca de frente num obstáculo e fica "navegando pra sempre" sem nunca
dar ré. Causa raiz confirmada por log ao vivo
(ver `memory/project_nav2_recovery_nao_dispara.md`):

1. O robô chega de frente no obstáculo.
2. O planner re-rota contornando; o caminho curva forte.
3. O RotationShimController para de andar e tenta **girar no lugar** pra alinhar
   (`cmd_vel_nav` = linear 0 / angular ~3.67).
4. O obstáculo está na zona de STOP do collision monitor → o collision **zera
   tudo, inclusive a rotação** (`/odom` = 0/0, congelado).
5. O robô nunca se alinha → comanda rotação pra sempre.
6. Pro Nav2 o controller está "tendo sucesso" (produz comando válido) → o
   `FollowPath` **nunca falha** → o recovery (a ré) **nunca dispara**.

Por isso o BackUp do BT (commit `4e8c5d8`) é código morto neste cenário: ele só
roda se o `FollowPath` falhar, e aqui ele nunca falha.

## Objetivo

"Quando o robô fica parado pelo collision monitor por mais de 10 segundos, ele
ignora o collision monitor e ativa a ré; aí reativa o collision e desvia de onde
parou. Nunca parar sem razão."

**Não-objetivos:** mexer na curva (RotationShim/DWB — está boa), enfraquecer o
collision monitor, ou consertar o caminho de recovery do BT.

## Ideia central

Um nó novo e pequeno (`unstuck_supervisor`) vigia o robô. Quando detecta o
travamento por >10 s, **assume o controle por um canal do twist_mux que fura o
collision monitor**, executa a manobra de desencalhe (ré ou giro) e devolve o
controle pro nav2, que replaneja e desvia. O collision e a curva não são tocados;
o nó só age **depois** que o robô já travou.

## Como fura o collision (o mecanismo)

Pipeline atual:

```
nav2 → smoother → nav_vel_raw → [collision_monitor] → nav_vel → twist_mux → wheels
```

O collision só filtra o trecho `nav_vel_raw → nav_vel`. Adicionamos uma **nova
entrada no twist_mux** (`unstuck_vel`) com prioridade **30**:

- acima de `nav_vel` (10) — a saída já filtrada do nav2;
- abaixo de web/key/joy (50/90/100) — o humano sempre pode assumir no manual.

Quando o supervisor publica em `unstuck_vel`, o mux escolhe esse canal e o comando
vai **direto pros motores, sem passar pelo collision**. Quando o supervisor para
de publicar, o mux volta sozinho pro `nav_vel` (collision reativado) após o
`timeout` (0,5 s) do mux.

Isso também resolve o segundo bloqueio do diagnóstico ("o collision congelaria a
ré também"): a manobra inteira (ré E giro) sai pelo canal que fura o collision.

## Detecção do "travado" (3 condições juntas, por >10 s)

1. **collision STOP ativo** — assinar `collision_monitor_state`
   (`nav2_msgs/CollisionMonitorState`); STOP ativo.
2. **robô congelado** — `/odom` com `|linear|` e `|angular|` ≈ 0.
3. **nav2 querendo andar** — `nav_vel_raw` ≠ 0 (senão é só goal atingido, não
   travamento).

As três simultâneas por mais de `stuck_timeout` (10 s) = travado. Bate com o log
capturado (STOP ativo, odom 0/0, RotationShim mandando angular).

## Máquina de estados

```
MONITORANDO ──(3 condições por >10s)──► DECIDE
                                          │
        traseira livre (/scan setor 180°±rear_sector > rear_clearance)?
                                          ├─ sim ──► RÉ
                                          └─ não ──► GIRO (Spin no lugar, via bypass)
                                          │
        já travou escalate_after× no mesmo ponto (<same_spot_radius)? ──► força GIRO
                                          │
RÉ/GIRO termina ──► SOLTA (para de publicar) ──► nav2 replaneja e desvia
                                          │
        grace (~2s), rearma contador de 10s ──► MONITORANDO
```

- **RÉ:** publica Twist com `linear.x = -reverse_speed`. Mede o deslocamento real
  por integração do `/odom` e para ao atingir `reverse_distance`, com **cap de
  tempo** de segurança (se o odom não acusar progresso, para mesmo assim).
- **GIRO:** publica Twist com `angular.z = spin_speed` por um ângulo/tempo
  limitado (também via canal de bypass, então o collision não congela o giro).
- **Setor traseiro:** checa o `/scan` cru no setor 180° ± `rear_sector_deg`. Se
  houver retorno < `rear_clearance`, **não dá ré** → vai pro GIRO.
- **Escalada:** guarda posição + hora de cada manobra. `escalate_after` (3)
  travamentos dentro de `same_spot_radius` (0,5 m) → força GIRO. Quando o robô
  consegue se afastar (> `same_spot_radius`), zera o contador.
- **"Nunca para sem razão":** se a ré não resolveu, re-detecta em 10 s e age de
  novo (ré → ré → ré → giro).
- **Grace:** após a manobra, segura `grace` s antes de rearmar o contador de 10 s,
  pra dar tempo do nav2 retomar.

## Arquivos

- **Novo:** `ros2_packages/robot_nav/robot_nav/unstuck_supervisor.py`
  + entry point `unstuck_supervisor = robot_nav.unstuck_supervisor:main` no
  `setup.py`.
- **Editar:** `config/twist_mux.yaml` — entrada `unstuck` (topic `unstuck_vel`,
  prio 30, timeout 0.5).
- **Editar:** `launch/nav2.launch.py` — sobe o nó com os parâmetros.
- **Não toca:** `collision_monitor` (YAML), RotationShim/DWB (curva), nem o BT
  `4e8c5d8` (fica como defesa redundante se o `FollowPath` falhar por outro motivo).

## Parâmetros (no launch, afináveis ao vivo)

| Parâmetro          | Default | Significado |
|--------------------|---------|-------------|
| `stuck_timeout`    | 10.0 s  | tempo travado antes de agir |
| `reverse_distance` | 0.30 m  | quanto recua |
| `reverse_speed`    | 0.15 m/s| velocidade da ré |
| `reverse_time_cap` | 3.0 s   | teto de tempo da ré (segurança) |
| `rear_clearance`   | 0.35 m  | distância mínima livre atrás pra permitir ré |
| `rear_sector_deg`  | 30°     | meia-largura do setor traseiro do scan |
| `spin_speed`       | 0.5 rad/s | velocidade do giro |
| `spin_angle`       | ~1.0 rad | quanto gira por manobra |
| `escalate_after`   | 3       | travamentos no mesmo ponto antes de forçar giro |
| `same_spot_radius` | 0.5 m   | raio que define "mesmo ponto" |
| `grace`            | 2.0 s   | espera após manobra antes de rearmar |
| `odom_zero_lin`    | 0.02 m/s| limiar de "parado" (linear) |
| `odom_zero_ang`    | 0.05 rad/s | limiar de "parado" (angular) |

## Tópicos

**Assina:**
- `collision_monitor_state` (`nav2_msgs/CollisionMonitorState`)
- `odom` (`nav_msgs/Odometry`)
- `nav_vel_raw` (`geometry_msgs/Twist`)
- `scan` (`sensor_msgs/LaserScan`)

**Publica:**
- `unstuck_vel` (`geometry_msgs/Twist`) → entrada prio 30 do twist_mux

## Testes

- **Unit (offline, sem ROS):** a lógica de decisão como funções puras —
  detecção das 3 condições, checagem do setor traseiro, contador de escalada,
  integração do deslocamento da ré. Roda sem ligar o robô.
- **Bancada (hands-on, robô ligado — anunciar antes):** mesmo obstáculo estático
  de frente. Verificar:
  - trava 10 s → dá ré → nav2 contorna;
  - parede atrás → gira em vez de dar ré;
  - travar 3× no mesmo ponto → escala pro giro;
  - humano no PS4 sempre sobrepõe o supervisor.

## Riscos / pontos a observar ao vivo

- A ré fura o collision: confiamos no setor traseiro do `/scan` pra não bater
  atrás. LiDAR não vê objeto muito baixo nem vidro (risco residual conhecido).
- Limiares de "parado" (`odom_zero_*`): se muito apertados, ruído de odom pode
  mascarar o congelamento; afinar na bancada.
- Confirmar o nome/campos exatos de `CollisionMonitorState` no Jazzy na hora de
  implementar (campo de ação/polígono ativo).
