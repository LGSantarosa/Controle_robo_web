# Para revisão do Codex — o `goal_active` do `path_follower` cai por um relógio de ~14 s e trava a volta

**Data:** 2026-09-03 · **Branch:** `arena-galpao` · **ROS:** jazzy · **Ambiente:** sim (Gazebo), na máquina de dev
**Prioridade:** máxima (ordem do dono). Bloqueia a arena inteira, prazo 05/09.

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

## 6. Minha hipótese (NÃO confirmada)

Os ~14-15 s batem com a **expiração do goal terminal anterior** na lista de status
do action server (`result_timeout`), que provoca uma nova publicação de
`GoalStatusArray`. Se essa publicação chegar sem nenhum status em `{1,2,3}`, o
`any()` dá falso e o seguidor cala permanentemente.

**O que apoia:** a regularidade (13,8-14,9 s, 3/3), o gatilho ser sempre relativo ao
`Goal succeeded` anterior, e a ausência total de evento no `bt_navigator`.

**O que eu não consigo explicar sozinho:** se o goal corrente está `EXECUTING`, ele
deveria continuar na `status_list` mesmo após o anterior expirar — e o `any()`
continuaria verdadeiro. Então ou a lista publicada nesse instante **não contém** o
goal corrente, ou o goal corrente não está no estado que eu presumo.

---

## 7. Já instrumentado (no ar, não muda comportamento)

`_on_status` agora loga `WARN` em **toda troca** de `goal_active`, com o tópico que
mudou, o **tamanho e o conteúdo cru** da `status_list` e o dict resultante. Basta
rodar uma volta e procurar `GOAL_ACTIVE` no `nav2.log`.

---

## 8. Perguntas específicas para o Codex

1. **A hipótese do `result_timeout` procede no jazzy?** Qual é o default, e o que
   exatamente é publicado no `GoalStatusArray` quando um goal terminal expira? A
   lista pode sair **vazia** mesmo havendo um goal `EXECUTING`?
2. **`any(st.status in ACTIVE for st in msg.status_list)` é a forma certa de
   derivar "tem goal ativo"?** Se não, qual é — rastrear o `goal_id` aceito e só
   soltar em status terminal daquele id? Assinar o feedback em vez do status?
   Usar o resultado do action client do próprio `map_service` e republicar?
3. **O `path_follower` deveria poder se recuperar disto sozinho?** Hoje, uma vez
   falso, fica falso para sempre. Um latch ("só solto o goal ao ver status terminal
   do id que eu estava seguindo") resolve sem esconder um problema real?
4. **Segundo problema, independente:** com o seguidor calado, o Nav2 ficou com o
   robô e não conseguiu girá-lo **nem um grau** em 355 s, gastando 233 replans e 5
   spins. É a zona-morta do skid-steer comendo os comandos de rotação do DWB/spin?
   Vale um `min_rotational_vel` / `RotationShim` diferente, ou é sintoma de outra
   coisa?
5. **Design:** faz sentido o waypoint pré-porta exigir yaw dentro de 20° a 0,8 m do
   vão, sendo que o robô rotineiramente chega ~30° torto e depende do point-turn
   final? Afrouxar `yaw_goal_tolerance` esconderia este bug ou é legítimo por si?

---

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

Cada uma tem `follow_debug.csv` (20 Hz: estado, pose, yaw, herr, dist_goal, vx, wz,
clear) e `nav2.log`. A 14:57 tem também `attempt_checkpoint.json`.

Narrativa completa no `DIARIO_ARENA.md`, §2H.34 e §2H.35. Item `2r` do §6.

**Provável duplicata:** o item `2g` do §6 ("16 s em `idle` entre dois goals",
aberto desde 08-31) é quase certamente este mesmo relógio, catalogado na época como
"buraco entre goals".
