# Porta — fim da briga de três (iteração 1 do approach)

**Data:** 2026-06-16
**Status:** design aprovado, aguardando plano de implementação
**Contexto anterior:** [2026-06-12-zonas-de-porta-design.md](2026-06-12-zonas-de-porta-design.md),
[2026-06-10-unstuck-supervisor-design.md](2026-06-10-unstuck-supervisor-design.md)

## Problema

O robô leva ~5 minutos pra atravessar uma porta. A causa **não** é o
ping-pong interno do `door_crossing`, e sim uma **briga de três** na
aproximação da porta:

1. O robô chega perto da porta desalinhado.
2. O nav2 (prio 10) quer **girar pra alinhar** → comando quase só angular,
   `linear.x ≈ 0`.
3. O collision_monitor freia o nav2 (batente perto da zona).
4. O `unstuck_supervisor` (prio 30, fura o collision) interpreta "parado com
   goal ativo" como encalhe → dá **ré + giro de 15°** → joga o robô pra trás,
   torto, longe da porta.
5. O robô volta, alinha errado de novo, e o ciclo se repete por minutos
   ("às vezes o collision manda ele umas 3 vezes pra trás e ele sai bem longe
   da porta e volta").

### Por que o standdown atual não cobre isso

O `unstuck_supervisor` **já tem** standdown (`door_active`), mas só desliga
quando `door_zone ∈ {staging, rotating, crossing}` — ou seja, **só quando o
door_crossing já assumiu a condução**. O door_crossing, porém, fica **piscando
pra `idle`** durante a aproximação, e a cada piscada o unstuck escapa do
standdown e sabota.

### Por que o door_crossing pisca pra idle

Condições de armar no estado `idle` (door_crossing.py, ~linha 204):

- **`nav_forward`** (`nav_vel_raw.linear.x > 0.02`): quando o robô chega torto,
  o nav quer **girar** (linear≈0) → `nav_forward = False` → cai pra `idle`
  justamente quando deveria assumir.
- **`cooldown` de 3s pós-abort**: aborta no timeout → 3s em `idle` → unstuck
  livre → sabota → re-arma → aborta → ...
- **`bearing < 70°`**: chegada muito torta deixa a porta fora do cone → nem
  seleciona a porta.

## Princípio da solução

**O `door_crossing` é o dono da região da porta. Perto de uma porta marcada
com goal ativo, o `unstuck_supervisor` se cala e deixa o door_crossing
conduzir** — sempre, não só durante a manobra. Fechamos a fresta pelos dois
lados: o door_crossing **para de piscar** (fica armado durante o giro de
alinhamento) e o unstuck **se cala na região inteira** (inclusive nas frestas
de cooldown).

## Mudanças (iteração 1 — pequenas e isoladas)

### 1. door_crossing — afrouxar o gate de armar

Trocar o critério `nav_forward` de "indo pra frente" (`linear.x > 0.02`) para
**"não está dando ré"** (`linear.x > -nav_move_lin`).

- **Efeito:** quando o robô chega torto e o nav quer girar pra alinhar
  (linear≈0), o door_crossing **continua armado** em vez de cair pra `idle`.
- **Por que é seguro:** o DWB roda com `min_vel_x: 0.0` (não dá ré em
  navegação normal — fix `b194dc7`), então `nav_vel_raw.linear.x` nunca fica
  sustentado-negativo no ramo do nav. O guard `> -nav_move_lin` só barra um
  improvável transiente de ré; não reintroduz o "atravessar de costas" (aquilo
  dependia da ré do DWB, que não existe mais).

### 2. door_crossing — publicar estado `approaching`

Novo estado no `/door_zone`. Quando o robô está **dentro de `zone_radius`
(1.2m) de uma porta marcada, com goal ativo** — critério **puramente
geométrico de proximidade, ignorando o cone de bearing** — e o door_crossing
ainda não está conduzindo (staging/rotating/crossing), publica
`door_zone = 'approaching'` em vez de `'idle'`.

- Marca a região da porta de forma **contínua**, sem frestas entre
  idle/cooldown/manobra.
- Ignora o cone de propósito: a sabotagem do unstuck era pior justamente na
  chegada torta (porta fora do cone). Proximidade pura cala o unstuck **até no
  pior ângulo**.
- A decisão de **conduzir** (entrar em staging) continua usando o cone +
  o gate afrouxado da mudança 1. `approaching` é só o sinal "estou na região
  da porta", separado de "assumi a direção".

Precedência da publicação do `/door_zone`:
1. manobrando (`staging`/`rotating`/`crossing`) → publica o estado da manobra;
2. senão, em zona de porta marcada + goal ativo → `approaching`;
3. senão → `idle`.

### 3. unstuck — standdown também no `approaching`

Em `unstuck_supervisor._on_door_zone`, adicionar `'approaching'` ao conjunto
de standdown (hoje `{staging, rotating, crossing}`).

- **Efeito:** perto de qualquer porta marcada com goal ativo, o unstuck **não
  dá ré nem gira** — fim da sabotagem, inclusive nos 3s de cooldown pós-abort.
- **Risco aceito:** se o robô travar de verdade só *passando perto* de uma
  porta (sem querer atravessar), o unstuck não ajuda naquela bolha de 1.2m.
  Aceitável: perto de uma porta marcada, o door_crossing é o dono pretendido,
  e o próprio door_crossing aborta por timeout se a travessia genuinamente
  emperrar (aí `door_active` cai e o unstuck volta).

## Escopo — o que esta iteração NÃO faz (de propósito)

- **NÃO** mexe no ping-pong interno staging↔rotating nem no giro-no-lugar
  fraco (`rot_speed = 3.0 rad/s`, que o skid-steer parado mal vence). Isso é a
  **iteração 2** (redesenho do approach em arco controlado por pose, pra chegar
  alinhado sem nunca parar pra girar no lugar).
- **NÃO** alarga `zone_radius` (fica 1.2m) → a janela em que o `door_vel`
  passa por cima do collision monitor (staging/rotating a 0.12 m/s) é a **mesma
  de hoje**. Não pioramos a segurança; fica anotado como limitação a revisitar.
- **NÃO** adiciona checagem de obstáculo no staging (status quo).

A aposta: sem a sabotagem do unstuck — **e com as rodas refitadas (menos
patinagem no giro)** — o alinhamento atual já fecha muito mais rápido. **Medir
em campo antes de decidir a iteração 2.**

## Validação

Teste em campo (robô **ligado**, anunciar antes), atravessando uma porta
marcada em modo nav2:

- **Sucesso:** o robô atravessa **sem** dar ré nem girar 15° na aproximação;
  tempo de travessia cai de ~5 min pra dezenas de segundos.
- **O que observar nos logs** (`controle_web/logs/nav2.log`):
  - `unstuck:` NÃO deve logar transições pra `reversing`/`spinning` enquanto
    `door_zone` está em `approaching`/`staging`/`rotating`/`crossing`.
  - `door_crossing:` deve mostrar `idle/approaching -> staging -> rotating ->
    crossing` sem voltar repetidamente pra `idle` durante a aproximação.
  - CSV do `NavMetricsCollector`: `rec_backup`/`direction_reversals` perto da
    porta devem cair drasticamente; `duration_s` da navegação que cruza a porta
    deve despencar.
- **Se ainda demorar** (alinhamento lento, mesmo sem sabotagem) → confirma que
  o gargalo restante é o giro-no-lugar → dispara a iteração 2.

## Arquivos afetados

- `ros2_packages/robot_nav/robot_nav/door_crossing.py` — gate afrouxado
  (lógica pura `DoorCrossing.update`) + publicação do `approaching` (decisão
  pura + cola no `_tick`/`_publish_zone`).
- `ros2_packages/robot_nav/robot_nav/unstuck_supervisor.py` — `'approaching'`
  no standdown (`_on_door_zone`).
- `ros2_packages/robot_nav/test/` — testes da lógica pura: gate afrouxado arma
  com comando rotacional; `approaching` publicado por proximidade ignorando
  bearing; unstuck em standdown com `door_zone='approaching'`.

Sem reflash da MEGA. Precisa `colcon build --packages-select robot_nav` +
relançar nav2 na Pi.
