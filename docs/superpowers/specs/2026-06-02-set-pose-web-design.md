# "Definir pose do robô" na web (relocalização manual)

Data: 2026-06-02
Status: aprovado (design)

## Problema

Robô sem IMU → o yaw vem da roda e mente conforme a dinâmica (sub-reporta giro
lento, super-reporta spin). No SLAM (estado B, commit 35424e6) o mapa sai bom, mas
às vezes o robô **perde a noção de onde está** — em especial "acha que girou pra
caralho mas mal girou". O slam corrige sozinho ao revisitar feature conhecida, mas
quando a deriva passa do raio do loop closure ele não re-localiza. Falta um jeito de
o operador dizer **"o robô está AQUI, com essa orientação"** pela web.

## Realidade técnica que delimita o escopo

- O slam_toolbox (mapping) e o AMCL (nav2) **aceitam relocalização manual pelo
  tópico `/initialpose`** (`geometry_msgs/PoseWithCovarianceStamped`, frame `map`) —
  é o mesmo que o "2D Pose Estimate" do RViz. Sem config extra: ambos já assinam
  esse tópico. slam_toolbox re-ancora o `map→odom`; AMCL semeia as partículas.
- A UI web **já tem** o que precisa: o `map.js` renderiza o mapa + a pose do robô
  (`map→base_link`), converte pixel↔coordenada de mapa, e tem padrões de clique
  (goal do nav2) e mousedown/drag (modo waypoint). O `MapBridge` (`map_service.py`)
  já é um nó ROS com publishers/handlers socketio.
- Trekking não tem mapa/slam (usa o cone pose_fix), então **fica fora**.

## Objetivo

Um modo **"Definir pose"** na web: o operador clica a posição do robô no mapa e
arrasta pra definir o heading; isso publica `/initialpose` e o slam_toolbox/AMCL
re-ancora — a pose `map→base_link` salta pro lugar indicado e o sistema segue dali.
Vale nos modos **slam e nav2**. Tudo no lado web; **zero toque na odometria/launches**
do estado B.

## Solução

### UI (`controle_web/static/js/map.js` + botão no template do mapa)

- Botão toggle **"Definir pose"** (espelha o toggle de modo waypoint que já existe).
- Quando **armado**: o próximo `mousedown` no canvas marca a posição; o **arraste**
  define o heading (desenha uma seta da origem até o cursor); `mouseup` confirma e
  **desarma** (one-shot). `Esc`/clique no botão de novo cancela.
- O toggle resolve a ambiguidade com o clique-goal (nav2) e o modo waypoint: só
  quando armado o click-drag vira "set pose".
- Reusa a conversão pixel→coordenada de mapa já usada pra goal/waypoints. Heading =
  `atan2(dy, dx)` do ponto inicial ao final, em coordenadas de mapa.
- **Precisão do toque (mobile):** hoje o clique no celular cai "torto" — sintoma
  clássico de descasamento entre a resolução interna do `<canvas>`
  (`canvas.width/height`) e o tamanho exibido por CSS (`rect.width/height`). O
  set-pose exige precisão (posição + heading via drag), então parte do escopo é um
  helper **único** `eventToMapCoords(evt)` que: pega `clientX/clientY` (de
  `evt` ou `evt.touches[0]` — funciona pra mouse E touch), subtrai
  `canvas.getBoundingClientRect()`, **escala por `canvas.width/rect.width` e
  `canvas.height/rect.height`**, e só então aplica o transform px→mapa. Suporta
  `touchstart/touchmove/touchend` (com `preventDefault` pra não rolar a página no
  drag). O goal e os waypoints passam a usar o MESMO helper → o "torto" some neles
  também (melhoria pontual no código que estamos tocando).
- No `mouseup` arma e emite `socket.emit('set_pose', {x, y, yaw})` (x,y em metros no
  frame `map`; yaw em rad). Mostra o ack.
- Botão só visível/ativo quando `currentMode ∈ {slam, nav2}` (o `mode_info` já
  controla a UI).

### Backend (`MapBridge` em `controle_web/map_service.py`)

- Novo publisher: `/initialpose` (`PoseWithCovarianceStamped`).
- Função pura `build_initialpose(x, y, yaw, stamp) -> PoseWithCovarianceStamped`:
  frame_id `map`, position (x, y, 0), orientation = quaternion de yaw (z=sin(yaw/2),
  w=cos(yaw/2)), covariância diagonal moderada (var x/y ≈ 0.25 m², var yaw ≈ 0.07
  rad² — confiante mas não absoluta; pro AMCL é a dispersão inicial das partículas).
- Handler socketio `set_pose`: só publica se `self._mode in ('slam', 'nav2')`;
  monta via `build_initialpose` e publica; emite `set_pose_ack` {ok, x, y, yaw} (ou
  {ok: false, reason} se modo incompatível).

### Fluxo

```
click+arrasta (armado) ─► socketio 'set_pose' {x, y, yaw}
  ─► MapBridge.build_initialpose ─► /initialpose (PoseWithCovarianceStamped, map)
  ─► slam_toolbox (mapping) re-ancora map→odom  /  amcl (nav2) re-semeia partículas
  ─► map→base_link corrige ─► 'robot_pose' na web salta pro lugar indicado
```

## Verificação

- **Unitário:** `build_initialpose` é pura → testar quaternion (ex.: yaw=π/2 →
  z≈0.707, w≈0.707), frame_id `map`, x/y corretos, covariância setada. Em
  `controle_web/` (pytest já roda lá).
- **Bancada (hands-on — anunciar e esperar "pode", memória
  `feedback_announce_before_test`):** subir `--slam`, deixar o robô derivar/girar até
  a pose ficar errada, usar "Definir pose" pra recolocá-lo; confirmar que a pose
  `map→base_link` salta pro lugar e o slam **segue mapeando coerente** dali (sem
  rasgar o mapa já feito). Repetir o sanity no `--nav2` (AMCL converge pra pose dada).
- **Toque no celular:** validar **no celular** que o ponto cai onde o dedo encosta
  (set-pose, goal e waypoint) — antes e depois do `eventToMapCoords`, pra confirmar
  que o "torto" sumiu.

## Fora de escopo

- Trekking (sem mapa/slam — usa cone pose_fix).
- Teleop (sem localização).
- O conserto da causa-raiz do yaw (rf2o abortado; estado B segue como está).
- Edição/serialização do grafo do slam_toolbox (loop closure manual, etc.).
