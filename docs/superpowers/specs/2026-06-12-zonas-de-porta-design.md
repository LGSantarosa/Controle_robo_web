# Zonas de porta — máscara de batente no scan_sanitizer + revert do shim

**Data:** 2026-06-12 · **Status:** aprovado pelo usuário (opção 1)

## Problema

O robô NÃO atravessa a porta no nav2. Evidência medida (bag `/tmp/porta_bag`,
22 freezes do PolygonStop analisados em 2026-06-12):

- 17/22 = retornos fantasmas do LD06 <15 cm → **resolvido** pelo
  `scan_sanitizer` (`71189f3`, deployado; eficácia em campo a confirmar).
- 5/22 = **batente REAL na quina dianteira da caixa** (x≈0,44, y≈0,18–0,30)
  com o robô entrando em diagonal (yaw 46–65° vs a porta). Com a caixa de
  ±0,30 m, qualquer entrada com >~17° de yaw põe o batente dentro dela.
- Agravante: o aperto do RotationShim pra 9° (`7b0f3c3`) produziu a oscilação
  prevista no yaml — o robô "tenta várias e várias vezes até ficar
  completamente de frente" (skid-steer estoura o alvo a 6.0 rad/s e re-gira).
- O mapa golden pinça as portas (0,60–0,85 m no mapa vs 0,93 m real);
  remapear segue planejado, mas é ortogonal: o freeze é no scan ao vivo.

## Decisão

O usuário MARCA as portas (2 cliques, um em cada batente, no mapa da UI).
Quando o robô está atravessando uma porta marcada, o collision monitor fica
cego SÓ para os batentes daquela porta. Junto, reverte o shim pro
comportamento de 17° com giro mais manso (rollback documentado no yaml).

## Arquitetura (opção 1 aprovada)

```
UI (map.js)          app.py (map_service)        scan_sanitizer            collision_monitor
"marcar porta" ──►  doors.json (persiste)  ──►  /doors (String JSON,  ──►  /scan_safe sem
2 cliques            por mapa + publica          transient_local)          fantasmas NEM batentes
apagar porta                                     máscara por TF map→base    de porta ativa
```

### 1. Dados — `maps/<mapa>.doors.json`

```json
{"doors": [{"id": 1, "a": [7.05, 4.45], "b": [7.55, 4.95]}]}
```

- `a`/`b` = batentes clicados, **frame do mapa**. Centro, largura e orientação
  derivam dos dois pontos. Arquivo ao lado do `.yaml` do mapa (marca 1x).
- O mapa pinça ~5–15 cm por lado; o raio do disco de máscara (0,30 m) cobre o
  erro entre batente-do-mapa e batente-real.

### 2. UI (mapa do nav2, `map.js` + `app.py`)

- Botão "Marcar porta": próximo 2 cliques no mapa = batente A e B; desenha
  segmento + discos. Clique em porta existente (modo marcar) = apagar.
- Socket: `door_cmd {add: {a, b}}` / `{del: id}` → app.py valida (largura
  0,4–2,0 m), persiste no json e (re)publica `/doors`.
- `/doors`: `std_msgs/String` JSON, QoS `transient_local` depth 1 — sanitizer
  que (re)inicia recebe o estado atual sem ordem de boot.

### 3. scan_sanitizer — máscara de batente (lógica pura + cola)

- Novo filtro APÓS o de fantasmas, só quando há porta "ativa":
  porta ativa = `dist(robô, centro_da_porta) < zone_radius` (param, 1,2 m).
- Pose do robô no mapa via TF `map → base_link` (laser = base_link, sensores
  no centro). **Fail-safe:** sem TF (modo SLAM, AMCL perdido, lookup falhou)
  → máscara INATIVA, comportamento atual.
- Máscara: ponto do scan (convertido pro frame do mapa) que cair num disco de
  `jamb_radius` (param, 0,30 m) ao redor de `a` ou `b` de porta ativa → `inf`.
- Pessoa/obstáculo NO VÃO não é mascarado (vão ≠ discos dos batentes) →
  PolygonStop continua protegendo a travessia.
- Pure function testável:
  `mask_door_jambs(ranges, angle_min, inc, pose_map, doors, zone_r, jamb_r)`.
- Log de transição (`porta 1 ativa/inativa`) + publica `/door_zone` (String
  JSON c/ id ativo ou vazio) pra UI acender o chip "porta ativa".

### 4. Revert do shim (`nav2_params_pi.yaml`)

- `angular_dist_threshold: 0.15 → 0.30` (~17°; com batentes mascarados,
  entrar torto deixa de travar).
- `rotate_to_heading_angular_vel: 6.0 → 4.2` (knob 2 do rollback documentado
  — giro mais fino, menos overshoot/oscilação).
- Atualizar o comentário do yaml contando o desfecho (oscilação confirmada
  em campo 2026-06-12).

## O que NÃO muda

- PolygonStop/PolygonSlow: geometria e min_points intactos (o usuário confia
  no collision monitor; a cegueira é cirúrgica e localizada).
- SLAM, costmaps, cone_detector, unstuck: seguem no `/scan` cru.
- Mapa golden e plano de remapear: intactos.

## Testes

- Unit (pure): ponto no disco do batente → inf; fora → intacto; porta longe
  (zona inativa) → nada; sem pose → nada; 2 portas, só a ativa mascara;
  fantasma + batente compõem (os dois filtros).
- app.py: add/del persiste e republica; validação de largura.
- Campo: porta marcada, goal através dela — critérios: PolygonStop não dispara
  por batente na zona; atravessa sem freeze; chip "porta ativa" acende/apaga;
  pessoa parada NO VÃO ainda freia o robô (teste com a perna, devagar).

## Rollback

- Apagar a porta na UI (ou o `.doors.json`) → máscara some.
- Shim: voltar 0.15/6.0 no yaml.
- Nó: fonte do collision monitor de volta pra `"scan"` desliga tudo.
