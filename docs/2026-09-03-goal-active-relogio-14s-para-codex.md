# Para revisão do Codex — o `goal_active` do `path_follower` cai por um relógio de ~14 s e trava a volta

**Data:** 2026-09-03 · **Branch:** `arena-galpao` · **ROS:** jazzy · **Ambiente:** sim (Gazebo), na máquina de dev
**Prioridade:** máxima (ordem do dono). Bloqueia a arena inteira, prazo 05/09.

> **ATUALIZADO 15:20 — LEIA A §6 ANTES DE TUDO.** A hipótese original (expiração de
> goal terminal por `result_timeout`) foi **REFUTADA** por instrumentação. O
> documento foi corrigido; as perguntas da §8 mudaram.

---

## 1. TL;DR

O `path_follower` deixa de ver o goal do Nav2 como ativo **13,8 a 14,9 s depois do
`Goal succeeded` ANTERIOR**, sempre no meio do `goal_turn` (o point-turn final).
Quando isso acontece antes do yaw entrar na tolerância, **a volta trava para
sempre**: o goal nunca conclui, e o `door_crossing` — que só arma com
`goal_succeeded` dentro da zona da porta — nunca assume.

Medido em **3 de 3 corridas de hoje**, inclusive na única que deu certo (lá o goal
concluiu 1,7 s antes do relógio bater).

**Não sei a causa.** Tenho a assinatura temporal e uma hipótese. Quero a opinião do
Codex antes de consertar.

---

## 2. O sintoma

Robô chega no waypoint pré-porta 1, em `(6,40 / 2,25)`:

- `dist_goal` = **0,014 m** contra `xy_goal_tolerance = 0,15` → XY perfeito
- o waypoint **exige `yaw = 0°`** (`ponto_pre_fresta()`, `tools/gera_arena_galpao.py:161`,
  *"encarando o vão"* — de propósito: o `approach_bearing = 70°` do `door_crossing`
  é medido a partir do yaw do robô)
- `yaw_goal_tolerance = 0,35 rad = **20,05°**` (`config/nav2_params_arena.yaml:152`,
  `stateful: true`)

O robô congela a **22,0°**. **2 graus fora.**

Depois disso, por 355 s: `vx = 0`, `wz = 0`, seguidor em `idle`. O Nav2 assume
(o seguidor cala quando não há goal — ver §4) e gasta **233 replans, 6 backups,
5 spins, 6 waits**, `avg_linear_speed = 0,006 m/s`, **sem girar um único grau**.

O `door_crossing` fica vendo a porta e proibido de agir, repetindo para sempre:

```
door DIAG idle: porta 1 distC=0.80 zone=1.10 cleared=False goal_succ=False nav_fwd=True
```

Ele está **dentro** da zona (0,80 < 1,10) o tempo todo.

---

## 3. A evidência que importa: é um RELÓGIO

Tempo entre o `Goal succeeded` **anterior** e a queda do seguidor para `idle`:

| corrida | queda | veio de | resultado |
|---|---|---|---|
| 14:47 (**a boa**) | **+14,87 s** | `goal_turn` | goal concluiu **1,7 s depois** → volta seguiu até o fim |
| 14:57 | **+14,82 s** | `goal_turn` | travou 355 s |
| 15:05 | **+13,80 s** | `goal_turn` | travou |

**3/3.** Dispara sempre, inclusive na corrida boa — que portanto não era robusta,
foi ganha por 1,7 s.

### Tick a tick (15:05) — o giro estava funcionando e foi cortado

```
720,65  goal_turn  yaw=83,0   wz=-4,344
721,45  goal_turn  yaw=44,2   wz=-2,4
722,02  goal_turn  yaw=32,7   wz=-2,4
722,33  goal_turn  yaw=25,6   wz=-2,4    <- 5,5° da tolerância, convergindo a 2,4 rad/s
722,44  idle       yaw=22,9   wz= 0,0    <- perdeu o goal. Faltava UM TICK.
```

No `nav2.log` desse instante: **nada**. Nenhum `Goal succeeded`, `Goal failed`,
`canceled`. Só `Passing new path to controller` a cada segundo.

E o `nav_metrics/attempt_checkpoint.json` mostra o goal **ainda em curso** de 26,3 s
a 381,3 s — ou seja, **o seguidor e o resto do sistema discordam por 347 s**.

---

## 4. O código

`ros2_packages/robot_nav/robot_nav/path_follower.py:735`:

```python
ACTIVE = {1, 2, 3}   # ACCEPTED, EXECUTING, CANCELING

def _on_status(self, topic, msg):
    self._goal_active[topic] = any(st.status in ACTIVE for st in msg.status_list)
    active = any(self._goal_active.values())
    ...
```

Assinaturas (`path_follower.py:598`):

```python
for topic in ('navigate_to_pose/_action/status',
              'navigate_through_poses/_action/status'):
    self.create_subscription(GoalStatusArray, topic,
                             lambda m, t=topic: self._on_status(t, m), 10)
```

E no tick (`path_follower.py:755`):

```python
goal = any(self._goal_active.values()) if self._goal_active else False
```

Consumo (`path_follower.py:766`) — **quando não há goal, o seguidor CALA**, ele não
segura o mux:

```python
# SEGURA O MUX: com goal ativo SEMPRE publica (prio 15 não expira ->
# o controller_server prio 10 nunca assume e briga). Sem goal -> cala.
if goal:
    self.pub.publish(m)
```

E no `DecisiveFollower.update()`, `goal_active` falso cai no guard do topo e
devolve `Cmd(0, 0, 'idle')` **zerando** `_turn_target`, `_aim_filt`,
`_arrival_latched` e `_latch_goal`.

### O ponto frágil que eu vejo

O sinal é **por evento**. O dict só muda quando chega `GoalStatusArray`. Se **uma
única** mensagem vier sem nenhum status ativo, o seguidor cala — e **não tem como
voltar**, porque com o goal seguindo em execução não há novo evento de status.
**Um status ruim mata a volta inteira.**

---

## 5. Contexto do sistema (para não partir de premissa errada)

- Os goals são enviados por **`NavigateToPose`** (`controle_web/map_service.py:1004`).
  O cliente de `NavigateThroughPoses` existe (`:392`) mas **não é usado** para a rota.
- `result_timeout` **não está setado** em `nav2_params_arena.yaml` → default do
  `rcl_action`/`rclcpp_action` do jazzy.
- O `path_follower` **ignora o `controller_server`**: ele lê o `/plan` do planner e
  dirige com dois primitivos (reto / point-turn), publicando em `follow_vel`
  (twist_mux prio **15**); `nav_vel` do Nav2 é prio **10**; `door_vel` é **20**.
- O robô é skid-steer com zona-morta: **não esterça andando** (`arc_calib` 06-25,
  ≤19% de fidelidade) e giros comandados abaixo de ~1,7 rad/s **não movem** as rodas.

---

## 6. 🔻 A hipótese original foi REFUTADA pelos dados

**O que eu havia proposto:** os ~14-15 s seriam a expiração do goal terminal
anterior saindo da `status_list` (`result_timeout` do `rcl_action`), o que
provocaria uma publicação sem nenhum status ativo.

**Está errado.** A instrumentação (§7) rodou na corrida das 15:17 e mostra que a
lista **nunca é podada** — ela só cresce, e os goals terminados permanecem nela:

```
n=1  [2]                    -> True     (EXECUTING)
n=1  [4]                    -> False    (SUCCEEDED)
n=2  [4,2]                  -> True
n=2  [4,4]                  -> False
n=3  [4,4,2]                -> True
n=4  [4,4,4,2]              -> True
n=5  [4,4,4,4,2]            -> True
n=6  [4,4,4,4,4,2]          -> True
n=7  [4,4,4,4,4,4,2]        -> True
n=7  [4,4,4,4,4,4,4]        -> False
```

Um item por goal, de 1 a 7, **nada removido em 222 s de corrida**. Sem poda, não há
expiração para culpar.

**E nessa corrida o `goal_active` funcionou perfeitamente:** cada `True -> False`
caiu exatamente quando o goal corrente virou **4 (SUCCEEDED)**, e cada
`False -> True` ~1,5 s depois, quando o próximo foi aceito. **O bug não reproduziu**
(a volta passou inteira, 7/7 goals).

### As duas leituras que sobram

Sabendo que a lista nunca poda, nas corridas travadas o `any()` só pode ter dado
falso se o **goal corrente** apareceu como terminal enquanto o `bt_navigator` ainda
o executava. Duas explicações concorrentes, e eu **não sei** qual é:

**(A) O sinal está errado.** O status do goal corrente virou terminal (4, 5 ou 6)
indevidamente. Alvo: `path_follower` / a action do `bt_navigator`.

**(B) O goal realmente concluiu, e o BO é outro.** O goal checker usa a pose dele,
que pode diferir do TF `map->base_link` que eu logo. Os **22,0° medidos contra
`yaw_goal_tolerance = 20,05°`** são perto o bastante para essa diferença decidir.
Se o goal concluiu de verdade, o seguidor fez **certo** em calar — e o BO real é que
**o `map_service` não enviou o goal seguinte**. Evidência a favor: nas corridas
travadas, o `False -> True` que aqui vem 1,5 s depois **nunca veio**, e nenhum novo
`Begin navigating` aparece no `nav2.log`.

**(B) mudaria o alvo de `path_follower.py` para `controle_web/map_service.py`.**

O que decide entre as duas é uma corrida **ruim** com a instrumentação ligada: ela
imprime a lista crua e **qual status** o goal corrente tinha no instante da queda.
Ainda não temos essa captura.

## 7. Já instrumentado (no ar, não muda comportamento)

`_on_status` agora loga `WARN` em **toda troca** de `goal_active`, com o tópico que
mudou, o **tamanho e o conteúdo cru** da `status_list` e o dict resultante. Basta
rodar uma volta e procurar `GOAL_ACTIVE` no `nav2.log`.

---

## 8. Perguntas específicas para o Codex

1. **A `status_list` que cresce sem parar (n=7 e subindo, terminados como `4`
   para sempre) é o comportamento esperado do `rclcpp_action` no jazzy?** Se sim,
   por quanto tempo/quantos goals? Existe algum ponto em que ela é limpa e que
   poderia produzir uma publicação sem nenhum ativo?
2. **`any(st.status in {1,2,3} for st in msg.status_list)` é a forma certa de
   derivar "tem goal ativo"?** Com a lista acumulando, isso vira "existe QUALQUER
   goal não-terminal", o que funciona por acidente. O correto não seria rastrear o
   `goal_id` aceito e olhar só o status **dele**?
3. **Entre (A) e (B) da §6, qual você acha mais provável, e que evidência
   distinguiria sem precisar esperar uma corrida ruim?** Há como inspecionar
   retroativamente nos logs das corridas travadas (§10)?
4. **Se for (B):** o que no `map_service.py` poderia aceitar um `SUCCEEDED` e não
   enviar o waypoint seguinte, sem logar nada? O envio é por `NavigateToPose`
   (`map_service.py:1004`), com callbacks de estado do goal a partir de `:430`.
5. **Robustez, independente da causa:** o `path_follower` hoje, uma vez com
   `goal_active` falso, fica falso para sempre (o sinal é por evento e nenhum novo
   evento chega). Um latch — "só solto o goal ao ver status terminal do `goal_id`
   que eu estava seguindo" — é a proteção certa, ou esconderia um problema real?
6. **Segundo problema, independente:** com o seguidor calado, o Nav2 ficou com o
   robô e não conseguiu girá-lo **nem um grau** em 355 s, gastando 233 replans e 5
   spins. É a zona-morta do skid-steer comendo os comandos de rotação do DWB/spin?
7. **Design:** faz sentido o waypoint pré-porta exigir yaw dentro de 20° a 0,8 m do
   vão, sendo que o robô rotineiramente chega ~30° torto e depende do point-turn
   final? Afrouxar `yaw_goal_tolerance` esconderia o bug ou é legítimo por si?

## 9. ⚠️ O que NÃO mexer

O trabalho de hoje no `path_follower` e no `door_crossing` **não participa deste
BO** — está provado: nesta corrida os estados `preempted` e `exit_straight` **não
aparecem** no `follow_debug.csv`, o `door_crossing` nem chegou a armar.

Não reverter:
- `door_crossing`: `exit_margin` 0,50 → 0,60 (§2H.27)
- `path_follower`: `preempted` (§2H.30) e `exit_straight` (§2H.32)
- portas 3 e 4 removidas por ordem do dono (§2H.29)

Esse pacote está sendo medido em paralelo, com baselines próprios, e resolveu um BO
diferente (giro de 180° dentro/na saída do vão, arrastando cones).

---

## 10. Onde estão os dados

| corrida | pasta |
|---|---|
| 14:47 — boa, volta completa | `docs/baselines/2026-09-03-arena-saida-reta-OK-volta-completa/` |
| 14:57 — travou | `docs/baselines/2026-09-03-arena-travado-no-waypoint-pre-porta-1/` |
| 15:05 — travou (repetiu) | `docs/baselines/2026-09-03-arena-travado-repetiu-14s/` |
| 15:17 — passou direto, **com instrumentação** | `docs/baselines/2026-09-03-arena-passou-direto-com-instrumentacao/` |

Cada uma tem `follow_debug.csv` (20 Hz: estado, pose, yaw, herr, dist_goal, vx, wz,
clear) e `nav2.log`. A 14:57 tem também `attempt_checkpoint.json`.

Narrativa completa no `DIARIO_ARENA.md`, §2H.34 e §2H.35. Item `2r` do §6.

**Provável duplicata:** o item `2g` do §6 ("16 s em `idle` entre dois goals",
aberto desde 08-31) é quase certamente este mesmo relógio, catalogado na época como
"buraco entre goals".

---

## 11. Resultados do outro pacote (contexto, não é o problema desta revisão)

Em paralelo, hoje foi consertado um BO diferente: o robô girava 180° dentro/na saída
do vão e arrastava os cones. Conserto = `exit_margin` 0,50 -> 0,60 + o seguidor não
acumular estado enquanto o `door_crossing` dirige + uma janela de **saída reta** de
0,8 m após a devolução. Resultados nas duas voltas que completaram:

| | 14:47 | 15:17 |
|---|---|---|
| `exit_straight`, episódio 1 | 0,80 m | 0,79 m |
| `exit_straight`, episódio 2 | 0,26 m (guarda de folga) | 0,19 m (guarda de folga) |
| `exit_straight`, episódio 3 | 0,80 m | 0,80 m |
| `wz` durante a saída reta | **0 nos três** | **0 nos três** |
| porta 1: preparação + travessia | — | 1,7 s + **8,0 s**, 1 tentativa |
| porta 2: preparação + travessia | 28,5 s total | 20,9 s + **7,0 s**, 2 tentativas |

**A travessia é estável em 7-8 s e limpa.** O custo está na *preparação* da porta 2
(limite-ciclo `staging <-> rotating`), que é anterior a este trabalho.

⚠️ Achado consistente nas duas voltas: o episódio 2 termina pela **guarda de folga**,
não pela distância — sinal de que a janela está armando depois de um **abort** do
`door_crossing`, não só depois de travessia concluída. O `/door_zone` publica `idle`
nos dois casos e o seguidor não distingue. Anotado como item `2p` do §6 do diário.
Opinião do Codex bem-vinda aqui também.

---

## 12. Achado novo (15:26) — a janela de saída reta envenena a mira

Sintoma relatado pelo dono: *"ele sai do 2, mesmo longe, ele decide girar"*.

Na janela `exit_straight` eu deixei a EMA da mira (`_aim_filt`, tau 2 s) continuar
atualizando, com a justificativa de que assim ela estaria quente no fim da janela.
Errado: durante a janela o plano é o **ruim** (o que a janela existe para ignorar),
então a EMA integra ~180° por 4 s. Medido:

```
891,0  exit_straight  n=137 ci=12  aim=(11,35/3,93)  herr= 179,3
894,0  exit_straight  n=137 ci=136 aim=(11,57/6,88)  herr=-167,0   <- carrot pula pro goal
895,0  turning        n=24  ci=23  aim=(11,57/6,88)  herr= -73,2   <- janela acaba, gira
                      ... 6,5 s de giro (yaw 88 -> -33 -> 70,9) ...
901,5  driving        yaw=70,9
```

Aos 895,0 **já havia plano bom** (`n=24`, carrot no goal). Com a mira crua o `herr`
seria ≈ **−6,6°**, abaixo do `turn_enter` de 16° → teria seguido reto.

Conserto proposto: zerar `_aim_filt` a cada tick **dentro** do ramo da janela,
simétrico ao que o `preempted` já faz. Opinião do Codex bem-vinda: há razão para
manter a EMA viva durante uma janela em que o plano é deliberadamente ignorado?

---

## 13. Resposta ao review do Codex (verificado ponto a ponto)

**Aceito e aplicado:**
- `_aim_filt` zerado por tick dentro da `exit_straight`, com teste de regressão que
  falha sem o conserto.
- Instrumentação refeita: última `status_list` de **cada** tópico, com `goal_id`
  abreviado + status por goal, os dois impressos na troca.
- `map_service` instrumentado: `WP_RESULT idx status` em `_on_goal_result` e
  `WP_SEND idx -> (x,y,yaw)` no `_send` do `_wp_runner`.
- "A lista nunca poda" suavizado para "não podou em 222 s".
- Confirmado o descompasso **6° (seguidor) × 20,05° (Nav2)**. ⚠️ O valor "11,2°" que
  eu publiquei estava **ERRADO** (amostra pega depois da troca de goal). Medição
  refeita na §14.

**Onde eu discordo, com dado:**

Promover o descompasso 6°×20° a **hipótese principal do travamento** não fecha. Esse
mecanismo exige que o Nav2 tenha **fechado** o goal — e nas duas corridas travadas
não há **nenhum** evento terminal para o goal pré-porta:

| corrida | `Begin navigating` | `Goal succeeded` | terminal do goal 2 |
|---|---|---|---|
| 14:57 | 2 | 1 (só o goal 1) | nenhum |
| 15:05 | 2 | 1 (só o goal 1) | nenhum |

O `bt_navigator` loga isso de forma confiável (corrida boa: 7 goals, 7 `Goal
succeeded`). E o robô congelou a **22,0°**, **fora** dos 20,05° — o Nav2 ainda estava
esperando.

**Minha leitura:** são **dois fenômenos**, e eu vinha tratando como um.
1. O descompasso 6°×20° é real e explica os drops **benignos** de `goal_active`
   (toda volta) — e, mais útil, é a **causa provável do custo da porta 2**: o
   waypoint pré-porta entrega o robô 11° torto, alimentando o limite-ciclo
   `staging <-> rotating` (20,9 s de preparação). Promovi a hipótese principal **ali**.
2. O travamento continua sem causa: `goal_active` caiu sem evento terminal.

**Pergunta nova pro Codex:** dado que o Nav2 não fechou o goal, o que mais poderia
zerar `any(status in {1,2,3})` na `status_list` de `navigate_to_pose`? A
instrumentação nova (com `goal_id`) responde na próxima corrida ruim.

**Também não fiz** (concordando com ele): afrouxar `yaw_goal_tolerance` como
curativo, antes de decidir **quem é o dono do yaw final perto da porta**. Hoje são
dois donos com números diferentes — essa é a decisão de projeto que falta, e é
provavelmente o conserto certo do item `2q`.

**Nota:** o README que ele criticou
(`docs/baselines/2026-09-03-arena-obstaculo-2-yaw-goal-regressao/README.md`) não é
desta sessão — está não-commitado no working tree, de antes. A crítica de causalidade
procede.

---

## 14. Correções do 2º review + a medição refeita do descompasso de yaw

**Os 3 defeitos apontados foram corrigidos:**
1. Comentário da `exit_straight` reescrito (ainda descrevia o comportamento antigo).
2. `WP_SEND` agora diz "despachando" e o texto avisa que é **tentativa**; adicionado
   **`WP_ACCEPT`** no `_on_goal_response` (aceito / REJEITADO).
3. O `yaw = 11,2°` estava errado — meu script pegou a amostra **posterior** ao
   `Goal succeeded` (`dist_goal = 4,51 m` denunciava). Refeito abaixo.

### Erro de yaw no instante em que o Nav2 fechou cada goal (corrida 15:17)

Método: última amostra do `follow_debug.csv` com `t <= t(Goal succeeded)`, comparada
com o yaw **exigido pelo waypoint** (`maps/routes/arena_galpao.json` + `ponto_pre_fresta()`
para os pré-porta).

| alvo | yaw do robô | yaw exigido | **erro** | vs. seguidor (6°) |
|---|---|---|---|---|
| cone_1 `(5,10 / 0,90)` | 23,3° | 8,1° | **15,2°** | FORA |
| **pré-porta 1** `(6,40 / 2,25)` | 19,7° | 0,0° | **19,7°** | FORA |
| cone_2 `(10,90 / 2,40)` | −17,4° | 2,5° | **19,9°** | FORA |
| **pré-porta 2** `(11,40 / 3,50)` | 104,3° | 90,0° | **14,3°** | FORA |
| cone_3 `(11,60 / 6,90)` | 78,9° | 83,0° | 4,1° | ok |
| cone_4 `(5,60 / 7,80)` | 173,0° | 177,6° | 4,6° | ok |
| chegada `(1,50 / 2,50)` | −141,1° | −123,4° | **17,7°** | FORA |

**5 de 7 goals fecharam com erro entre 14,3° e 19,9°** — colados no teto de 20,05°
do Nav2 e todos fora dos 6° do seguidor.

Ou seja: o descompasso é **sistemático**, e o número real é **pior** que o que eu
tinha publicado. O pré-porta 1 fechou a 19,7°, praticamente no teto do Nav2.

**Estado das frentes:**
- **Custo da porta 2 (`2q`):** causa provável agora bem medida. Falta a decisão de
  projeto: **quem é o dono do yaw final perto da porta?** Nenhum de nós quer
  afrouxar `yaw_goal_tolerance` como curativo antes disso.
- **Travamento (`2r`):** sem causa. Instrumentação completa em 3 pontos
  (`GOAL_ACTIVE` com `goal_id` por tópico; `WP_SEND`; `WP_ACCEPT`; `WP_RESULT`).
  Falta a corrida ruim.

Suíte rodada aqui (o Codex não tinha `pytest`): **163 passam**, com 2 falhas
pré-existentes da §2H.22 que já falhavam em `3c2b051`.
