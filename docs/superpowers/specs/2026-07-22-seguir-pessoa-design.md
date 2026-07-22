# Seguir pessoa (tap-to-track por lidar) — Design

> Spec. Data: **2026-07-22**. Autor: dono + assistente (brainstorming).
> Origem: ideia do dono — "o rosto, após clicado, dá a opção de seguir, aí ele
> segue a movimentação da pessoa". Estende o MODO INTERAÇÃO anotado em 07-14 e a
> cara fase 2 (olhos seguem pessoa, já em campo).

---

## 1. Objetivo

Dar ao robô um **modo "seguir pessoa"**: a pessoa toca na cara do iPad, escolhe
**"Seguir"**, e o robô passa a **andar atrás dela mantendo ~1,5 m**, usando só o
lidar pra saber onde ela está. Todas as seguranças de hoje continuam valendo —
a única coisa que muda é o **alvo final**: em vez de um ponto da rota, o alvo é
a pessoa travada.

### Princípio-mestre (pedido explícito do dono)
> "Ele tem que ter o mesmo medo como se estivesse andando para os pontos, com
> todas as seguranças que já existem. O que muda é só o alvo final — seguir a
> pessoa. Se alguém chega perto ele para, se for bater para, etc., assim como
> funciona hoje."

Traduzindo pra arquitetura: o `person_follower` calcula **apenas a velocidade
desejada** em direção à pessoa e a injeta na **mesma entrada** que o
`path_follower` usa hoje (`twist_mux_auto`). Toda a cadeia de segurança a jusante
(`motion_guard` blocked/slowing → `collision_monitor` → `unstuck` → E-stop)
processa essa velocidade **exatamente igual** à da navegação por pontos. Não se
duplica nem se contorna nenhuma proteção.

---

## 2. Escopo (decidido no brainstorming)

- **Disparo:** toque na cara (:7000) mostra botão **"Seguir"**; toque nele inicia.
- **Locomoção:** o robô **anda atrás** da pessoa (não só gira encarando).
- **Distância mantida:** **~1,5 m** (piso rígido; acima da bolha do guard).
- **Fala ao iniciar (antes de andar):** *"Irei te seguir, tente ficar próximo e
  ir devagar."*
- **Perdeu o alvo:** **para** e fala *"Não estou mais te vendo, poderia se
  aproximar?"*; espera ~10–15 s.
- **Ninguém volta:** **retoma a rota** que estava fazendo (de onde parou).
- **Parar de propósito:** botão **"Parar"** na cara **e** na GUI (:5000).
- **Segurança:** pipeline de hoje intacto; validação **toda no sim primeiro**.

### Fora de escopo (YAGNI / fases futuras)
- **Câmera / identidade do alvo** (re-ID quando some e volta): a C922 ainda não
  subiu no tripé. v1 é **lidar puro** — associação por proximidade. É por isso
  que "perdi você" pede pra pessoa se aproximar em vez de sair caçando.
- **Girar e procurar** o alvo perdido (opção descartada no brainstorming).
- **Escolher entre várias pessoas por "quem interagiu"** — v1 trava a pessoa da
  frente no momento do toque.
- **Modo interação rico** (o robô decidir o que falar/fazer parado) — feature à
  parte.

---

## 3. Fluxo / máquina de estados

Estados do `person_follower` (publicados em `follow_person_state`, string
latched, consumidos pela GUI e pela cara):

```
IDLE  ──(START c/ alvo travável)──►  FOLLOWING
IDLE  ──(START sem ninguém à frente)─►  IDLE + fala "não vejo ninguém pra seguir" (opcional v1: só não entra)

FOLLOWING ──(alvo somiu > lost_grace)──►  LOST
FOLLOWING ──(STOP: cara/GUI)──────────►  ENDING
FOLLOWING ──(alvo colou < 1,5 m)──────►  FOLLOWING (v=0, só encara; guard é o chão)

LOST ──(alvo reapareceu perto)────────►  FOLLOWING
LOST ──(timeout lost_timeout ~12 s)───►  ENDING
LOST ──(STOP: cara/GUI)───────────────►  ENDING

ENDING ──(rota pausada existia)───────►  retoma rota → IDLE
ENDING ──(sem rota)───────────────────►  para → IDLE
```

**Transições que falam (TTS na cara):**
- `IDLE → FOLLOWING`: *"Irei te seguir, tente ficar próximo e ir devagar."*
  (fala **antes** de publicar a primeira velocidade — 1 tick de atraso proposital.)
- `FOLLOWING → LOST`: *"Não estou mais te vendo, poderia se aproximar?"*
- (reuso do TTS já existente na `face.js`: `new Audio('/static/*.mp3')`.)

---

## 4. Arquitetura — 4 peças

### 4.1 `person_follower` (NÓ NOVO, `ros2_packages/robot_nav/robot_nav/person_follower.py`)
Controlador dedicado, no mesmo padrão de `path_follower`/`motion_guard`/`unstuck`.

- **Assina:**
  - `follow_person_targets` (novo, do `motion_guard`) — lista de centróides de
    clusters candidatos a pessoa (cx, cy, dist, bearing) que o guard já calcula.
  - `follow_cmd` (novo, do `app.py`) — String `START` / `STOP`.
  - `odom` — pra taxa de giro / referência do controle (mesmas fontes do guard).
- **Publica:**
  - `follow_person_vel` (Twist) — **entra no `twist_mux_auto`** como par do
    `follow_vel`. É a velocidade DESEJADA; a segurança é aplicada a jusante.
  - `follow_person_state` (String, latched) — `idle|following|lost|ending`
    (+ campos úteis pra cara/GUI: dist do alvo, bearing).
- **Faz:**
  1. **Trava do alvo** (no START): pega o cluster candidato **mais próximo e à
     frente** (dentro de `acquire_cone` p.ex. ±60°, `acquire_range` p.ex. ≤3 m).
     Se não há candidato → não entra em FOLLOWING (fica IDLE).
  2. **Associação quadro-a-quadro** (só lidar): a cada scan, casa o alvo com o
     cluster candidato mais próximo do último centróide do alvo, dentro de
     `assoc_gate` (p.ex. 0,6 m de salto máximo). Sem match por > `lost_grace`
     (p.ex. 1,0 s) → LOST.
  3. **Malha de controle** (ver §5).
  4. **Estados + falas** (§3).

### 4.2 `motion_guard` (só ganha, não muda comportamento)
- Publica `follow_person_targets`: os clusters-pessoa que **já** calcula
  (`_cluster`, `_person_centroid`) — evita clusterizar o `/scan_safe` duas vezes.
- Aditivo: 0 impacto no caminho de guard/blocked existente. Se `follow_person`
  desligado por param, nem publica.

### 4.3 `app.py` (:5000 — orquestrador de rota)
- Endpoint novo `POST /follow` com `{action: "start"|"stop"}`:
  - **start:** **pausa o waypoint runner** (`_wp_runner`) e cancela/guarda o goal
    nav2 em andamento + o índice do waypoint atual; publica `follow_cmd=START`.
  - **stop:** publica `follow_cmd=STOP` (o `person_follower` faz o ENDING; o
    `app.py` **retoma** a rota ao ver `follow_person_state=ending`→resume, ou por
    ack simples).
- **Retomada da rota:** ao receber `follow_person_state` transicionar pra
  `ending`/`idle` (fim do seguir), se havia rota pausada → `_wp_runner` continua
  do waypoint guardado; senão → nada (robô fica parado).
- Botão "Parar de seguir" na GUI bate em `POST /follow stop`.
- Já é nó rclpy (usa action client do nav2) → só ganha 1 publisher (`follow_cmd`)
  e 1 subscriber (`follow_person_state`).

### 4.4 Cara (`face_web/face_app.py` :7000 + `static/face.js` ES5)
- **Botão "Seguir":** aparece ao **tocar na cara** enquanto NÃO está seguindo.
  Toque → `face.js` faz `POST` (via `face_app`) pro `app.py` `/follow start`.
- **Botão "Parar":** aparece enquanto `follow_person_state=following|lost`.
  Toque → `/follow stop`.
- **Falas:** dois mp3 novos gerados por gTTS pt-BR (como `ola.mp3`/`licenca.mp3`):
  `seguir_inicio.mp3` e `nao_te_vejo.mp3`. Disparados pela `face.js` na
  transição do estado (mesma mecânica do `licenca` hoje, throttle p/ não repetir).
- **Estado:** `face_app` já expõe `GET /state`; ganha o campo de follow (lê do
  `follow_person_state` — via arquivo JSON escrito pelo follower, no molde do
  `FaceStateFile`, OU via o `app.py` intermediando). **Decisão v1:** o
  `person_follower` escreve um `FaceStateFile` próprio
  (`/tmp/person_follow_face.json`) e o `face_app` o lê — simétrico com o guard,
  não acopla o face_app ao ROS.

### Injeção na tubulação (o ponto central)
```
nav_vel (smoother) ─┐
follow_vel (path_follower) ─┤
follow_person_vel (person_follower) ─┤─► twist_mux_auto ─► motion_guard ─►
door_vel (door_crossing) ─┘        auto_vel_raw ─► collision_monitor ─► auto_vel
                                    ─► twist_mux FINAL (unstuck/humano furam) ─► /cmd_vel
```
`follow_person_vel` é adicionado ao `config/twist_mux_auto.yaml` com **priority
par ou logo acima do `follow_vel`** (durante o seguir, o `path_follower` está
mudo porque a rota está pausada, então não há disputa real; a priority só
formaliza). **Nada** a jusante muda → segurança idêntica.

---

## 5. Malha de controle do `person_follower`

Entrada por tick: alvo `(bearing θ, dist d)` relativo ao robô.

- **Girar pra encarar:** point-turn na IMU (autoridade 6.0, `closed_loop=false`),
  igual ao giro decisivo de hoje. `wz` proporcional a θ, **cap `wz` = 2.4** perto
  de gente (≈0,4 rad/s reais) — reusa o cap do slowing do guard. Zona-morta de
  θ (`face_deadband`, p.ex. 8°) pra não tremer.
- **Andar pra frente:** só quando |θ| < `drive_align` (p.ex. 20°) **e**
  `d > stop_dist` (1,5 m). `vx` proporcional a `(d − stop_dist)`, saturado em
  `vx_max` (p.ex. 0,25 m/s — igual à média de campo). **Nunca** anda com o alvo
  fora do cone frontal.
- **Parar/recuar:** `d ≤ stop_dist` → `vx = 0` (só encara). **Não recua** por
  padrão (recuar em cima de gente atrás é o caso ruim do unstuck — deixar o guard
  cuidar). Se `d < stop_dist − hyst` a pessoa colou: o `motion_guard` já entra em
  blocked e zera tudo — é o chão de segurança agindo, correto.
- **Histerese** (`hyst`, p.ex. 0,2 m) entre andar/parar pra não pulsar em 1,5 m.

> A saída é **desejo**; `motion_guard` (blocked/slowing) e `collision_monitor`
> podem zerar/frear a qualquer momento. O follower **não** tenta furar isso —
> se o guard bloquear (alguém entrou na frente, ou o próprio alvo colou), o robô
> para, e o follower simplesmente segue mirando (wz respeitando o guard).

---

## 6. Segurança

- **Pipeline intacto:** `follow_person_vel` passa por `twist_mux_auto` →
  `motion_guard` → `collision_monitor` → mux final. Blocked, slowing, reflexo de
  colisão, unstuck e E-stop **valem igual** à navegação por pontos.
- **Piso de 1,5 m** no próprio follower + **bolha do guard abaixo disso**: se a
  pessoa chega a <1,5 m, o guard para o robô. Redundância proposital.
- **Cap de `wz`/`vx`** perto de gente (2.4 / 0,25) — reuso dos limites já
  calibrados.
- **Só avança com alvo à frente** no cone; nunca lateralmente/às cegas.
- **Não recua** ativamente atrás de gente (lição do tênis 07-10).
- **E-stop e "Parar" (cara+GUI)** sempre cortam.
- **MUDANÇA GRANDE:** validação **no sim primeiro** (pessoa 2-pernas teleop
  `bin/teleop-pernas` no `sala_grande`), real só valida (regra do projeto).

---

## 7. Parâmetros ROS (knobs, todos afináveis ao vivo onde o guard é)

| Param | Default | O que faz |
|---|---|---|
| `follow_enabled` | `false` | liga o modo (default OFF; só liga quando for usar) |
| `stop_dist` | `1.5` | distância mantida da pessoa (m) |
| `stop_hyst` | `0.2` | histerese andar/parar (m) |
| `vx_max` | `0.25` | velocidade máx. de avanço (m/s) |
| `wz_cap` | `2.4` | teto do giro perto de gente (comando skid) |
| `face_deadband_deg` | `8` | zona-morta do encarar |
| `drive_align_deg` | `20` | só anda com \|θ\| abaixo disso |
| `acquire_cone_deg` | `60` | cone frontal pra travar alvo no START |
| `acquire_range` | `3.0` | alcance pra travar alvo no START (m) |
| `assoc_gate` | `0.6` | salto máx. do centróide entre scans (m) |
| `lost_grace` | `1.0` | s sem match antes de declarar LOST |
| `lost_timeout` | `12.0` | s em LOST antes de desistir e retomar rota |

---

## 8. Testes

### Unit (pytest, `ros2_packages/robot_nav/test/test_person_follower.py`)
- **Malha:** dado (θ, d) → sinais de `vx`/`wz` corretos: para em 1,5 m; anda só
  alinhado; cap de wz; zona-morta; histerese não pulsa.
- **Associação:** alvo salta < gate → mantém; salto > gate ou sem cluster >
  `lost_grace` → LOST; reaparece perto em LOST → volta FOLLOWING.
- **Máquina de estados:** START sem candidato → fica IDLE; STOP em qualquer
  estado → ENDING; LOST → timeout → ENDING.
- **Relógio travado na fonte** (mesma disciplina dos testes de guard/unstuck —
  não misturar ROS clock com monotonic).

### face_web (pytest, `face_web/test_*.py`)
- `/state` reflete o follow state; falas disparam 1x por transição (throttle).

### Sim (validação, `sala_grande` + `bin/teleop-pernas`)
1. Pessoa teleop anda reto → robô segue a 1,5 m, para quando ela para.
2. Pessoa some atrás de parede → robô para, fala, e ao reaparecer retoma; se não,
   retoma a rota pausada.
3. Segundo corpo cruza colado → registrar se troca de alvo (limitação lidar).
4. Alguém entra na frente do robô durante o seguir → guard **blocked** para tudo
   (prova do pipeline intacto).
5. Botão "Parar" (cara+GUI) encerra e retoma a rota.

---

## 9. Faseamento (sim → real)

- **Fase A (código, Pi desligada):** `person_follower` + `motion_guard` publica
  targets + `twist_mux_auto` ganha a entrada + testes unit/face verdes.
- **Fase B (sim):** o ciclo de §8 no `sala_grande` com teleop-pernas. Iterar
  knobs. Nenhum deploy no real antes disso fechar.
- **Fase C (real, dono presente):** run curta controlada, mão no E-stop, cone
  livre. Validar os 5 casos. **A retomada de rota (§4.3) é a peça mais nova** —
  se der trabalho, pode ir por último (v1 sem resume = para no lugar; o dono
  pediu resume, então é meta, mas é o candidato natural a fase separada se
  atrasar).

---

## 10. Arquivos afetados

- **Novo:** `ros2_packages/robot_nav/robot_nav/person_follower.py`
- **Novo:** `ros2_packages/robot_nav/test/test_person_follower.py`
- **Novo:** `face_web/static/seguir_inicio.mp3`, `face_web/static/nao_te_vejo.mp3`
  (+ script gTTS que gera, no molde do que gerou `ola`/`licenca`)
- Edita: `motion_guard.py` (publisher `follow_person_targets`, atrás de param)
- Edita: `config/twist_mux_auto.yaml` (entrada `follow_person_vel`)
- Edita: `launch/robot.launch.py` + `launch/sim.launch.py` (sobe o nó novo)
- Edita: `controle_web/app.py` (endpoint `/follow`, pause/resume `_wp_runner`,
  pub `follow_cmd`, sub `follow_person_state`)
- Edita: `controle_web/.../templates` + GUI JS (botão "Parar de seguir")
- Edita: `face_web/static/face.js` + `face_web/face_app.py` (botões Seguir/Parar,
  falas, leitura do follow state)

---

## 11. Riscos / decisões em aberto pra fase de plano

- **Troca de alvo sem câmera** — aceito como limitação v1; mitigado pelo
  "aproxime-se". Câmera/NUC é o upgrade futuro (identidade), já mapeado em 07-14.
- **Retomada de rota** — parte mais acoplada (`app.py` ↔ `_wp_runner` ↔ estado do
  follower). Candidata a sub-fase própria se complicar.
- **Priority de `follow_person_vel` vs `follow_vel`** — como a rota fica pausada
  no seguir, não deve haver disputa; confirmar no plano que o `path_follower`
  realmente cala sem goal (não publica `follow_vel` a 0 competindo).
