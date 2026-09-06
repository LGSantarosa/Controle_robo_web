# DEADLOCK 2026-09-06 12:45:35-12:46:15 — a ré da porta 2 saiu ZERADA

Corrida do dono: `./launch.sh --nav2 --arena --map=maps/oficial.yaml`, rota
`teste11.json` (7 pontos após o pré-porta), código `7b509f4`
(`follow_forward_speed = 0.30`). Relato do dono: *"ele foi atravessar a porta,
aí não deu ré, ficou preso lá um tempão, aí ele foi pro ponto de trás e passou
pelo obstáculo — passou, mas não do jeito que deveria"*.

Esta é a **medição que a spec de 01/09 §4.6 pediu e nunca tinha sido feita**:

> *"A ré de escape foi desenhada sob a premissa de que ninguém a filtrava — hoje
> o `collision_monitor` a filtra. Consequência prática a **medir**, não supor: a
> ré de escape pode ser atenuada ou zerada justamente quando o robô está de
> nariz na parede, que é quando ela existe."*

Resposta: **não foi atenuada, foi zerada. 35 s.**

O **`nav2.log` desta corrida está aqui do lado**, salvo antes de qualquer novo
launch — `launch.sh:801` abre o log com `>` (trunca), então a próxima subida
teria destruído a evidência. Todos os números abaixo saem dele; os horários do
log são epoch (`1788709070` = 12:37:50).

## VEREDITO CURTO

| pergunta | resposta |
|---|---|
| A ré foi comandada? | **Sim**, `-0.25 m/s` por 35 s seguidos. |
| A ré aconteceu? | **Não.** Deslocamento < 0.30 m em 35 s = zero. |
| Quem zerou? | `collision_monitor`, `PolygonFront`, `linear_limit: 0.0`. |
| O que destravou? | **O relógio** (`total_timeout = 40 s`). Nenhuma manobra. |
| A porta foi atravessada pela travessia fina? | **Não.** O `crossing` abortou em 70 ms e 590 ms; quem passou foi o nav2. |

## 1. Linha do tempo (waypoint idx=5, alvo `(-5.88, -5.89)`)

| hora | evento |
|---|---|
| 12:45:35.6 | `door_crossing: idle -> rotating` (arma na porta 2) |
| 12:45:37.3 | `rotating -> reversing` |
| 12:45:39.0 | `reversing -> staging` |
| 12:45:40.99 | `staging -> reversing` — comanda `vx = -0.25`, `wz = 0.0` |
| 12:45:41.05 | `collision_monitor: Robot to limit speed due to PolygonFront` |
| 12:45:49 / 12:45:57 / 12:46:05 / 12:46:13 | `controller_server: Failed to make progress` (4×) |
| 12:46:13.6 | `behavior_server: Running backup` — **perdendo o mux pro `door_vel` (20 > 10)** |
| 12:46:15.70 | `reversing -> idle` — **`total_timeout` de 40 s** (12:45:35.6 + 40.1 s) |
| 12:46:15.80 | `collision_monitor: Robot to continue normal operation` |
| 12:46:16.29 / 12:46:16.68 | limita e libera de novo — obstáculo NA BORDA da caixa (`min_points: 2`) |
| 12:46:17.6 | `backup failed` — `Collision Ahead - Exiting DriveOnHeading` |
| 12:46:17.7 → 12:46:22.7 | `Running wait` → `wait completed successfully` |
| 12:46:37.9 | `NavMetrics: nav b99734b9 → SUCCEEDED em 62.3 s, dist 5.52 m, rec(b/s/w)=1/0/1` |

O `limit` do collision cobre o estado `reversing` **inteiro**: 12:45:41.05 →
12:46:15.80, **34,76 s contínuos**, sem um único "continue normal operation" no
meio. O waypoint custou 62,3 s e 5,52 m de trajeto pra um alvo a **1,4 m**.

## 2. Mecanismo — por que a ré não saiu

`nav2_params_arena.yaml`, `PolygonFront`: `action_type: limit`,
**`linear_limit: 0.0`**, `angular_limit: 4.0`.

O Nav2 aplica o `limit` sobre o **módulo** da velocidade linear
(`hypot(vx, vy)`), não sobre o sinal: o fator de escala vira 0 e **qualquer**
comando linear morre, inclusive `vx` negativo. Só o angular passa.

O comentário no YAML dizia *"como o giro segue livre, ele se realinha e sai"* —
e essa válvula de escape **funciona**, pra quem pivota. O `path_follower` e o
DWB pivotam. A ré da porta comanda `wz = 0.0` **por desenho** (*"recua RETO,
NUNCA arco"*). Então:

| eixo | comandado | saiu |
|---|---|---|
| linear | −0.25 m/s | **0** (`linear_limit: 0.0`) |
| angular | **0.0** (por desenho) | 0 |

Robô **completamente imóvel**. E robô imóvel não limpa a própria caixa frontal:
o scan não muda, os pontos continuam dentro, o `limit` não sai. **O deadlock se
auto-alimenta** — é a única manobra da stack que não alcança a válvula de escape.

Segunda camada: enquanto isso o `door_vel` segurava **prio 20** no
`twist_mux_auto`, então as recoveries do nav2 também não comandavam nada
(`backup` às 12:46:13.6, com o door ainda no ar). A ré bloqueada **sequestrava**
o resgate de quem podia resolver.

O que quebrou o impasse não foi manobra: foi o `total_timeout`. **Ele não
escapou, ele desistiu.**

## 3. Segundo defeito, independente: o `crossing` abortando em 70 ms

Na volta (waypoint idx=6), o `door_crossing` rearmou e chegou a entrar em
`crossing` duas vezes — e saiu na hora:

- 12:46:55.19 `rotating -> crossing` → 12:46:55.26 `crossing -> idle` (**70 ms**)
- 12:47:03.6 `rotating -> crossing` → 12:47:04.2 `crossing -> idle` (**590 ms**)

Foi **abort**, não travessia concluída. Prova: o caminho de sucesso faz
`_cleared.discard(id)`, e o `door DIAG` 10 ms depois ainda mostra
`cleared=True`; além disso ele rearmou 4,6 s depois, compatível com
`retrigger_cooldown = 3.0 s` e não com o `crossing_cooldown = 8.0 s` do sucesso.
O único abort dentro do `crossing` é **`gap < gap_min` (0.45 m)**.

Ou seja: a porta foi atravessada pelo nav2/path_follower no braço, com a manobra
fina abortada — que é exatamente o *"passou, mas não do jeito que deveria"*.

**Este defeito NÃO foi corrigido.** Precisa do `/scan` cru no instante do
`crossing` pra separar cone real dentro do corredor, batente marcado fora do
lugar, e fantasma do LD06 sobrevivendo ao sanitizer. Baixar `gap_min` sem isso é
desligar o abort que existe pra não atropelar gente.

## 4. O que mudou por causa desta corrida

1. **Canal `door_escape_vel`** (`twist_mux.yaml`, prio 25): só o estado
   `reversing` sai por ali, no mux FINAL, a jusante do collision — mesmo lugar
   de onde o unstuck (30) e o humano já furam o reflexo. `staging`/`rotating`/
   `crossing` continuam no `door_vel`, atrás do collision: são movimento pra
   FRENTE, e é ali que uma pessoa no vão seria atropelada.
   Falha **fechada**: se o `door_crossing` morre, ele só para de publicar e o
   reflexo segue armado — ao contrário de desligar o polígono em runtime, que
   ficaria desligado pra todo mundo, calado.
   Durante a ré o `door_vel` publica **zero** (não fica mudo), mantendo a prio 20
   pra que o nav2 não cole um avanço no meio da manobra.
2. **`escape_stall_time = 2.0 s`** (`door_crossing.py`): o `reversing` só saía
   por deslocamento, então ré bloqueada era indistinguível de ré recém-começada
   e queimava os 40 s inteiros. Agora usa a mesma âncora de progresso do
   substuck: andou → reinicia o relógio; não andou → aborta pro nav2, **que sabe
   pivotar** (e o angular nunca é capado).

Os dois são a mesma falha vista de dois lados: um deixa a ré sair, o outro
percebe rápido quando ela ainda assim não sai.

## 5. Critério de aceite da próxima corrida na porta 2

- `reversing` que **se desloca** (comparar pose no início e no fim do estado);
- nenhum `reversing` durando mais que ~2,5 s sem deslocamento;
- nenhum waypoint de porta gastando os 40 s do `total_timeout`;
- se o `crossing` continuar abortando em <1 s, é o defeito §3 — **capturar o
  `/scan` cru** ali, não mexer em `gap_min`.
