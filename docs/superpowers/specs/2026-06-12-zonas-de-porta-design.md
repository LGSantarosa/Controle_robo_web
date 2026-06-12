# Travessia de porta — manobra dedicada (door_crossing) + máscara de batente gated + revert do shim

**Data:** 2026-06-12 · **Status:** v2 aprovada pelo usuário (v1 da "máscara por
proximidade" foi REJEITADA por ele: o collision salva o robô na fase torta da
aproximação; relaxar antes de alinhar = trocar freeze por batida).

## Problema

O robô NÃO atravessa a porta no nav2. Evidência medida (bag `/tmp/porta_bag`,
22 freezes do PolygonStop analisados em 2026-06-12):

- 17/22 = retornos fantasmas do LD06 <15 cm (dentro do chassi) → filtro
  `scan_sanitizer` (`71189f3`) já deployado; eficácia em campo a confirmar.
  Eram esses os freezes "indo perfeito no meio da porta".
- 5/22 = batente REAL na quina da caixa (±0,30) com entrada diagonal
  (yaw 46–65°) — o collision AGIU CERTO nesses (estava indo pro batente).
- O aperto do RotationShim pra 9° (`7b0f3c3`) confirmou em campo a oscilação
  prevista no yaml: "tenta várias e várias vezes até ficar de frente".
- Mapa golden pinça as portas (0,60–0,85 vs 0,93 real). Remapear segue
  planejado; ortogonal a este spec.

## Decisão (3 peças)

1. O usuário MARCA as portas (2 cliques, um por batente, no mapa da UI) —
   isso dá o eixo da travessia (centro + perpendicular) de graça.
2. **Manobra dedicada de travessia** (novo nó `door_crossing`, molde do
   unstuck_supervisor): alinhar DE VERDADE no eixo (critério numérico), só
   então atravessar reto e devagar vigiando o vão. Collision monitor 100%
   ativo na aproximação.
3. Máscara dos batentes marcados no `/scan_safe` **somente durante o estado
   "atravessando verificado"** — o collision fica "do tamanho da porta": cego
   pros 2 batentes clicados, enxergando todo o resto (pessoa no vão freia).

## Arquitetura

```
UI (map.js)       app.py (map_service)      door_crossing (novo nó)         twist_mux
marcar porta ──► doors.json + /doors ──►  zona+goal+nav empurrando ──►  door_vel (prio 40,
2 cliques         (String JSON,            ALINHA (TF map→base vs eixo)   entre nav=10/20 e
apagar            transient_local)         ATRAVESSA reto vigiando vão     unstuck... ver plano)
                        │                        │ estado "crossing"
                        ▼                        ▼ /door_zone (JSON)
                  scan_sanitizer: fantasmas SEMPRE; batentes da porta ativa
                  SÓ quando door_crossing publica estado "crossing"
                        │
                        ▼
                  collision_monitor (/scan_safe) — intacto em geometria/min_points
```

### 1. Dados — `maps/<mapa>.doors.json`

```json
{"doors": [{"id": 1, "a": [7.05, 4.45], "b": [7.55, 4.95]}]}
```

`a`/`b` = batentes clicados, frame do mapa. Centro/eixo/largura derivados.
Validação no app.py: largura 0,4–2,0 m. Arquivo ao lado do `.yaml` do mapa.

### 2. UI (mapa do nav2)

- Botão "Marcar porta" → 2 cliques = batentes; segmento + discos desenhados.
  Clique em porta existente no modo marcar = apagar.
- Socket `door_cmd {add:{a,b}} | {del:id}` → app.py persiste + (re)publica
  `/doors` (transient_local depth 1 — nó que reinicia recebe o estado).
- Chip "porta N: alinhando / atravessando" vindo de `/door_zone`.

### 3. door_crossing — nó novo (lógica pura + cola, padrão unstuck)

Estados: `IDLE → ALIGNING → CROSSING → RELEASE` (+ `ABORT` a qualquer hora).

- **Gatilho (IDLE→ALIGNING):** goal nav2 ativo (status do action server,
  mesmo gate do unstuck) E robô a <`zone_radius` (1,2 m) do centro de porta
  E nav comandando movimento na direção da porta (dot(nav_vel, eixo) > 0).
- **ALIGNING:** assume via `door_vel` (twist_mux, prioridade acima do nav,
  abaixo do unstuck/joy). Malha fechada na pose do TF `map→base_link` contra
  o eixo: gira/avança lento até **|offset lateral| < 0,08 m E |erro de yaw| <
  5°** estáveis por N ciclos. Timeout (param, ~15 s) → ABORT (devolve pro
  nav2, collision intacto — vira tentativa normal).
- **CROSSING:** publica estado `crossing` (gate da máscara no sanitizer).
  Avança 0,15 m/s corrigindo micro-desvio lateral no eixo (ganho pequeno,
  saturado). Vigia o VÃO à frente no scan cru (corredor da largura do robô
  até 0,6 m adiante, descontando os discos dos batentes): obstáculo → para e
  ABORT. Vão termina: passou do centro + `exit_margin` (0,5 m) → RELEASE.
- **RELEASE/ABORT:** Twist zero explícito + para de publicar (igual unstuck);
  estado `idle` no `/door_zone` (máscara desliga). Nav2 segue o goal.
- **Fail-safes:** sem TF map→base (SLAM/AMCL perdido) → nunca arma; goal
  cancelado/abortado no meio → ABORT; scan velho → ABORT; unstuck assumir
  (prioridade maior) → este nó solta sozinho (vê o robô saindo do eixo).

### 4. scan_sanitizer — máscara gated

- Assina `/doors` e `/door_zone`. Máscara dos discos (`jamb_radius` 0,30 m)
  de `a`/`b` APENAS para a porta cujo estado é `crossing`. Fora disso, o
  sanitizer só filtra fantasmas (<0,15 m), como hoje.
- Pure function: `mask_door_jambs(ranges, angle_min, inc, pose_map, door,
  jamb_r)` — composição com `sanitize_ranges` coberta por teste.

### 5. Revert do shim (`nav2_params_pi.yaml`)

- `angular_dist_threshold: 0.15 → 0.30` e
  `rotate_to_heading_angular_vel: 6.0 → 4.2` (rollback documentado no yaml;
  oscilação confirmada em campo 2026-06-12). A precisão fina de porta agora
  é da manobra, não do shim. Atualizar o comentário com o desfecho.

## O que NÃO muda

PolygonStop/Slow (geometria, min_points), costmaps, SLAM, cone_detector,
unstuck_supervisor, mapa golden. `/scan` cru intacto pra todo mundo.

## Testes

- Pure (door_crossing): gatilho exige goal+zona+direção; critério de
  alinhamento (offset/yaw/estabilidade); abort por timeout/vão/scan velho/
  goal morto; release após exit_margin; máquina nunca arma sem TF.
- Pure (sanitizer): máscara só com estado crossing e só nos discos da porta
  certa; composição fantasma+batente; sem estado → só fantasma.
- app.py: add/del persiste/republica; validação de largura.
- Campo (roteiro): porta marcada, goal através — (1) aproximação torta ainda
  é freada pelo collision; (2) alinhamento converge <15 s sem oscilar;
  (3) atravessa sem freeze; (4) pessoa no vão DURANTE crossing freia;
  (5) chip da UI acompanha os estados.

## Rollback

- Apagar porta na UI / `.doors.json` → manobra+máscara nunca armam.
- Shim: voltar 0.15/6.0.
- Nó: desabilitar door_crossing no launch; sanitizer volta a só filtrar
  fantasma (gate nunca abre).
