# Unstuck — recovery contextual (obstáculo mapeado → ré rápida)

**Data:** 2026-06-22
**Branch:** `feat/door-para-pra-pessoa` (mesma sessão do door; isolável)
**Arquivo afetado:** só `ros2_packages/robot_nav/robot_nav/unstuck_supervisor.py`

## Problema / ideia

Hoje o `unstuck_supervisor` trata todo encalhe igual: parado <`stuck_radius` (5 cm) por
`stuck_timeout`=**10 s** com goal ativo → dá ré. Mas:

- Encalhou contra uma **parede JÁ MAPEADA** → ela não vai sair andando; esperar 10 s é
  desperdício. **Ré rápida.**
- Encalhou contra algo **NOVO** (só no LiDAR ao vivo, livre no /map) → pode ser pessoa que
  sai. **Esperar** (como hoje).

A distinção é barata: `/map` (estático) vs LiDAR ao vivo. Lookup O(1) na célula do
bloqueio. Ver [[project_recovery_contextual_mapeado_vs_novo]].

## Decisões (brainstorm 2026-06-22)

- **Timing mapeado:** ~2 s **com mini-confirmação** (não instantâneo — evita reagir a pico
  de pose/scan). Obstáculo novo segue 10 s.
- **O que testar:** o **bloqueio à FRENTE** (ponto a `front_gap` na direção do heading),
  NÃO um disco em volta do robô (em corredor apertado o robô está sempre colado nas paredes
  laterais → classificaria 'mapeado' sempre → ré rápida até com obstáculo NOVO à frente).
- **Fonte:** `/map` (mapa estático cru do SLAM), **não** o global_costmap (tem inflação →
  marcaria tudo perto de parede como ocupado).
- **Escopo:** só encurta o RELÓGIO; a escolha de direção (ré preferida / avanço / giro de
  escalada) **não muda**.

## Design

### 1. Subscription do `/map`
Nó assina `OccupancyGrid` em `/map` (QoS latched/transient-local). Guarda o último grid
(data + width + height + resolution + origin). Sem mapa recebido → `obstacle_mapped` sempre
False → comportamento IDÊNTICO ao de hoje (fallback seguro).

### 2. Helper puro `map_occupied(grid, x, y, neighborhood) -> bool`
True se ALGUMA célula dentro de `neighborhood` (m) de (x,y) está ocupada
(valor >= `map_occ_threshold`, ex. 65) no mapa estático. Célula desconhecida (-1) ou fora
dos limites do grid = **não** ocupada. Pura, testável offline. `grid` = struct leve
(data, width, height, resolution, origin_x, origin_y).

### 3. Nó calcula `obstacle_mapped` (cola de I/O)
```
bx, by = position + front_gap * [cos(yaw), sin(yaw)]          # ponto do bloqueio à frente
obstacle_mapped = (front_gap <= block_range) and map_occupied(grid, bx, by, map_neighborhood)
```
`block_range` (~0,5 m): só conta como bloqueio à frente se o obstáculo está perto; frente
livre (front_gap grande/inf) → False. Passa o bool pro `update()`.

### 4. `update()` escolhe o timeout (com mini-confirmação)
Rastreia `mapped_since` (instante em que o bloqueio à frente virou mapeado; zera quando
deixa de ser mapeado). A recovery dispara quando:
```
(now - anchor_t) >= stuck_timeout_mapped  AND  (mapped_since is not None and
                                                 now - mapped_since >= stuck_timeout_mapped)
```
senão cai no caminho normal `(now - anchor_t) >= stuck_timeout` (10 s). Os 2 s contínuos de
mapeado SÃO a mini-confirmação (um frame solto de mapa não flipa e dispara na hora). A
direção (ré/avanço) e a escalada (giro na 3ª) ficam exatamente como estão.

### 5. Config nova (param ROS)
- `stuck_timeout_mapped: 2.0` (s)
- `block_range: 0.5` (m — front_gap acima disso não conta como bloqueio à frente)
- `map_occ_threshold: 65` (0-100; >= é ocupado)
- `map_neighborhood: 0.15` (m — raio da vizinhança no lookup)

### 6. Testes (TDD)
- Unit `map_occupied`: ocupada True; livre False; desconhecida (-1) False; fora dos limites
  False; ocupada só dentro da vizinhança (ponto a <neighborhood de uma célula ocupada) True.
- Máquina (`update`): bloqueio mapeado dispara aos `stuck_timeout_mapped` (não espera 10 s);
  bloqueio NOVO (não mapeado) ainda espera `stuck_timeout`; mapa flipou tarde (parado 9 s,
  vira mapeado agora) NÃO dispara na hora — respeita os 2 s de confirmação; `obstacle_mapped`
  False (sem /map) reproduz o comportamento de hoje.
- Manter verdes os testes atuais do `unstuck_supervisor`.

## Risco residual (tunar em campo)

Depende da pose no instante: parede mapeada só é confiável com a localização boa. Mitigado
pela confirmação de 2 s + `/map` cru (sem inflação → não marca falso perto de parede). Pior
caso (pose derivada): dá ré ~8 s mais cedo contra uma parede real — e contra parede real, ré
é a saída certa de qualquer jeito.

## Fora de escopo

- "Parado vs se movendo" (rastrear o obstáculo) — alternativa registrada na memória, fica
  pra depois se a versão por mapa não bastar.
- Mudar direção/escalada da recovery — intocadas.
