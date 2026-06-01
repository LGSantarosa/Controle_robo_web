# Design — Correção de pose por cone-âncora no trekking (teach-and-repeat)

**Data:** 2026-06-01
**Status:** design revisado em conversa (06-01): threat model atualizado + camada de
observabilidade + correção de cone na gravação. Aguardando revisão do spec antes do plano.

## 1. Contexto

No modo trekking a pose vem do `pose_estimator`, que funde **IMU (yaw absoluto)** +
**PMW3901 (flow)** + **rodas** em `/trekking/pose`. Essa estimativa **deriva** com o
tempo (flow é ruidoso por EMI; rodas patinam sem suspensão).

O `trekking_runner` já faz **snap-to-cone**: na gravação anota o cone mais próximo de
cada waypoint; no percurso autônomo, ao chegar perto da posição esperada do cone, casa
com o cone observado (`/trekking/cones`, publicado pelo `cone_detector` em `odom`) e
**re-ancora o ALVO** (`alvo = cone_observado + offset_gravado`). Isso cancela a deriva
**só pra mirar aquele waypoint** — a crença de pose do robô continua derivada, e a
correção é descartada no ciclo seguinte.

**Arquivos envolvidos:**
- `ros2_packages/robot_nav/robot_nav/trekking_runner.py` — `_control_tick`, `_find_matching_cone`, `locked_cone`, `_save_point`, `_on_cmd`.
- `ros2_packages/robot_nav/robot_nav/pose_estimator.py` — dono de `x/y/yaw`, `_lock`, publica `/trekking/pose`.
- `ros2_packages/robot_nav/robot_nav/cone_detector.py` — `/scan` → `/trekking/cones` (PoseArray em `odom`, largura no `orientation.x`).
- `controle_web/app.py` — whitelist `_TREKKING_CMDS`/`_TREKKING_KWARGS` do socket `trekking_cmd`.
- `controle_web/static/js/trekking.js` — render do canvas (cones/waypoints/locked) + envio de comandos.
- `controle_web/trekking_service.py` — bridge socket.io ↔ `/trekking/cmd`/`/trekking/state` (passthrough genérico; **não muda**).

## 2. Objetivo

Estender o snap-to-cone para também **corrigir a POSE (x/y) de forma persistente**:
quando o runner confirma um cone gravado, empurrar `x/y` do `pose_estimator` pela deriva
medida (`cone_gravado − cone_observado`), de modo conservador. A correção passa a valer
para **o resto do trajeto**, não só para o waypoint atual.

Dois recursos de apoio (decididos em 06-01):
- **Observabilidade (read-only):** a UI mostra qual detecção o robô está usando como
  âncora, o que descartou (clutter) e o status da decisão — pra auditar, não decidir.
- **Correção de cone na gravação:** na fase RECORD/IDLE o operador pode re-vincular ou
  limpar o cone preso a um waypoint, caso o robô tenha pego o cone errado (ou lixo).

### Não-objetivos (YAGNI)
- **Yaw:** continua só do IMU (BNO055 dá yaw absoluto bom; um cone só dá posição x/y).
- **SLAM / scan-matching** de scan inteiro (frágil em campo aberto — descartado).
- Usar feições que não sejam cones como landmark.
- **Desvio / reação a obstáculo no caminho:** o `trekking_runner` continua **pura
  perseguição PID**; o percurso gravado é assumido **livre**. (06-01: operador escolheu
  observabilidade, não navegação reativa. Se virar necessidade, é spec separado.)
- **Escolher a âncora no PLAY:** a decisão no percurso é 100% da máquina (robô passa
  rápido e longe — humano não reage a tempo). A intervenção humana é só na gravação.
- Mexer em firmware, MEGA, controle de baixo nível, ou no caminho do alvo (snap-to-cone
  atual permanece **inalterado**; a correção de pose é **aditiva**).

## 3. Requisito de segurança (threat model atualizado 06-01)

Premissa nova: **juízes e operador ficam LONGE do robô** no percurso. Então a "perna de
pessoa do tamanho de cone que **se move**" deixou de ser a ameaça dominante. **Mas o
campo não é só cone:** pode haver outros obstáculos e outros cones, inclusive **perto da
posição onde um cone-âncora foi gravado**. Associar a âncora ao objeto errado empurraria
a pose pro lugar errado.

**Princípio de risco (inalterado):** um pulo de pose por associação errada é **pior**
que a deriva suave que ele corrige. Viés: **na dúvida, NÃO corrige** — uma correção
perdida só mantém a (pequena) deriva; uma errada teleporta o robô.

Por isso as travas continuam **todas** — só muda a justificativa de cada uma:
- **Unicidade** vira a trava central: num campo com clutter, o perigo é um segundo
  objeto colado no cone gravado. >1 candidato no raio → não corrige.
- **Magnitude** limita qualquer associação errada a um erro pequeno.
- **Estabilidade temporal** perde o motivo "perna se mexendo", mas de graça (~0,13 s)
  ainda mata flicker do LiDAR, oclusão parcial e qualquer coisa/bicho que cruze o campo.

Reforço estrutural: o pose-fix **só olha detecções perto da posição gravada do cone**.
Obstáculo em qualquer outro lugar do campo é **irrelevante** pra correção — ele não está
perto de nenhum cone gravado.

## 4. Design

### 4.1 Interface
- **Tópico novo:** `/trekking/pose_fix` (`geometry_msgs/Vector3Stamped`) — delta de
  correção em `odom` (`vector.x`, `vector.y`; `z` não usado).
- `trekking_runner` **publica** `pose_fix` ao confirmar um cone-âncora.
- `pose_estimator` **assina** `pose_fix` e aplica à pose, com ganho + gates.
- **Observabilidade (read-only):** o `/trekking/state` (JSON já existente) ganha
  `anchor` (`[x,y]|null`), `anchor_status` (`idle|confirming|ambiguous|fixed`),
  `anchor_clutter` (`[[x,y],…]` — detecções perto do esperado que NÃO são a âncora) e
  `anchor_confirm` (`[count, frames]`). **Sem tópico novo.**
- **Correção de cone na gravação:** comando novo `set_cone` em `/trekking/cmd`:
  `{cmd:'set_cone', idx, cone_x, cone_y}` (re-vincula) ou `{cmd:'set_cone', idx, clear:true}`
  (remove). Só vale **fora do PLAY**.

### 4.2 Fluxo de dados (correção de pose)
1. `cone_detector` → `/trekking/cones` (já existe).
2. `runner._control_tick`: ao entrar no raio do cone esperado, `_find_matching_cone`
   (gate de posição `< cone_match_radius` + bearing `< cone_bearing_tol` — já existe).
3. **Confirmação antes de corrigir a pose** (não afeta o snap do alvo):
   - **Estabilidade temporal:** o candidato precisa casar na **mesma posição**
     (dentro de `cone_stable_eps`) por `cone_confirm_frames` ciclos seguidos. Cone
     parado confirma; objeto se movendo reseta o contador → nunca confirma.
   - **Unicidade:** se houver **mais de um** candidato dentro do raio de unicidade
     (`max(cone_unique_radius, cone_match_radius)`, ver §4.4) da posição esperada →
     ambíguo (cone + obstáculo, ou dois cones) → **não** corrige.
4. Confirmado e único: `delta = (wp.cone_x − cone_obs.x, wp.cone_y − cone_obs.y)` →
   publica `pose_fix`. Só **uma vez por cone travado** (não fica empurrando repetido).
5. `pose_estimator._on_pose_fix`:
   - rejeita se `hypot(dx,dy) > pose_fix_max` (teleporte → associação suspeita; loga warn);
   - senão, sob `_lock`: `x += pose_fix_gain·dx`, `y += pose_fix_gain·dy`.

### 4.3 Por que `cone_gravado − cone_observado` = a deriva
Numa sessão de trekking o frame `odom` é o mesmo do início (gravação) ao fim (percurso).
O cone gravado está numa posição `odom`; o **mesmo** cone visto agora está em outra. A
diferença é exatamente o quanto a pose derivou entre gravar e repetir. Empurrar a pose
por esse delta **re-alinha o robô ao frame do percurso gravado** — que é o referencial
que importa no teach-and-repeat (a verdade absoluta é irrelevante; o que importa é
chegar onde os waypoints foram gravados).

### 4.4 Parâmetros novos
| nó | parâmetro | default | função |
|----|-----------|---------|--------|
| trekking_runner | `cone_confirm_frames` | 4 (~0,13 s @30 Hz) | ciclos estáveis p/ confirmar |
| trekking_runner | `cone_stable_eps` | 0,10 m | tolerância de "mesma posição" entre ciclos |
| trekking_runner | `cone_unique_radius` | 0,50 m | se >1 candidato aqui → ambíguo (ver nota) |
| trekking_runner | `enable_cone_pose_fix` | true | liga/desliga a correção de pose |
| pose_estimator | `pose_fix_gain` | 0,5 | fração do delta aplicada (suaviza) |
| pose_estimator | `pose_fix_max` | 0,6 m | acima disso, rejeita (cone errado) |

**Nota (unicidade):** a contagem de candidatos usa
`max(cone_unique_radius, cone_match_radius)`, nunca um raio menor que a região de onde o
match sai. Se `cone_unique_radius < cone_match_radius`, um cone casado entre os dois raios
ficaria de fora da contagem e a trava de unicidade teria uma brecha. Com `max(...)` o cone
casado está sempre incluído. Com os defaults (0,50 < 0,60) o raio efetivo é 0,60 m.

A observabilidade e o `set_cone` **não** adicionam parâmetros.

### 4.5 Resumo das travas de segurança (camadas)
1. **Largura** — `cone_detector` já filtra largura (torso de pessoa é largo demais).
2. **Posição + bearing** — `_find_matching_cone` (já existe).
3. **Estabilidade temporal** (NOVO) — mata movimento / flicker / oclusão.
4. **Unicidade** (NOVO) — mata ambiguidade cone+obstáculo / dois cones (trava central).
5. **Magnitude** (`pose_fix_max`) — mata teleporte por associação errada.
6. **Ganho parcial + warn** — mesmo aceita, aplica suave e deixa rastro no log.

### 4.6 Observabilidade (read-only)
O confirmador (em `_control_tick`/`_maybe_publish_pose_fix`) atualiza estado no runner,
exposto pelo `_state_tick` no JSON `/trekking/state`:
- `anchor`: a detecção usada como referência agora (ou `null`).
- `anchor_status`: `idle` (sem candidato) | `confirming` (estável, contando) |
  `ambiguous` (>1 candidato → não corrige) | `fixed` (fix enviado p/ este cone).
- `anchor_clutter`: detecções dentro de `cone_unique_radius` do esperado que **não** são
  a âncora — é o "clutter" que dispara a unicidade. A UI pinta numa cor distinta.
- `anchor_confirm`: `[count, cone_confirm_frames]` — barrinha de progresso.

Resets junto de `locked_cone` ao trocar de waypoint e ao iniciar PLAY.

`ConeFixConfirmer` expõe uma propriedade `count` (lê o contador interno) só pra essa
telemetria.

**Limitação honesta:** o runner marca `fixed` quando **envia** o `pose_fix`. Se o
`pose_estimator` rejeitar por magnitude (raro — a unicidade já barra antes), o runner não
fica sabendo (fire-and-forget) e a UI mostra `fixed` mesmo assim; o estimator loga
`pose_fix REJEITADO` alto. **Não** haverá ack de volta (round-trip a mais por um caso
raro — YAGNI). Cobrir isso depois seria um campo lido de outro tópico.

### 4.7 Correção de cone na gravação (`set_cone`)
**Interação na UI (RECORD/IDLE; escondida no PLAY):** o operador clica num waypoint do
canvas pra selecioná-lo, depois clica numa detecção de cone pra prendê-la (envia
`set_cone` com `idx`+`cone_x`+`cone_y`), ou aperta **"limpar cone"** (envia `set_cone`
com `idx`+`clear:true`). Hit-test em pixels: transforma cada wp/cone com `view.tx/ty` e
pega o mais próximo do clique dentro de ~12 px.

**Runner `_set_wp_cone(data)`:**
- valida `idx` na faixa de `self.waypoints` (senão `last_msg` de erro, ignora);
- `clear:true` → `has_cone=False`, zera `cone_x/cone_y/cone_bearing`;
- senão → `cone_x/cone_y` = valores recebidos, `has_cone=True`, e
  `cone_bearing = wrap_pi(atan2(cone_y−wp.y, cone_x−wp.x) − wp.yaw)`.

**Por que recomputar o bearing:** o gate angular do `_find_matching_cone` no PLAY compara
o bearing observado com `wp['cone_bearing']`, que é relativo à pose **gravada** do
waypoint. Trocar o cone sem recomputar furaria esse gate. Calcular relativo a
`wp['x'/'y'/'yaw']` reproduz exatamente o que `_save_point` faz na gravação.

**Concorrência:** mexe em `self.waypoints` sem lock extra, igual ao código existente
(`load_waypoints`, `_save_point`) — serializado pelo SingleThreadedExecutor. O runner
aceita `set_cone` em qualquer modo; quem esconde no PLAY é a UI (mantém o runner simples).

**`app.py`:** adicionar `set_cone` em `_TREKKING_CMDS` e `idx, cone_x, cone_y, clear` em
`_TREKKING_KWARGS`. `trekking_service.send_cmd` é passthrough — **não muda**.

## 5. Tratamento de erro / fallback
- Sem cone, ou não confirmado, ou rejeitado → **nenhum** `pose_fix` publicado →
  comportamento **idêntico** ao de hoje. Zero risco de regressão fora do caminho novo.
- `enable_cone_pose_fix=false` desliga a correção de pose inteira (volta ao snap-só-do-alvo).
  Observabilidade e `set_cone` independem desse flag.
- Escrita concorrente de `x/y` protegida pelo `_lock` do `pose_estimator`.
- `set_cone` com `idx` inválido / `cone_x/cone_y` ausentes → ignora com `last_msg` de erro.

## 6. Testes
- **Unit (sem ROS):** cálculo do delta; gate de magnitude (rejeita > max); confirmação
  temporal (sequência estável confirma, sequência móvel não); unicidade (2 candidatos → skip).
  Toda a lógica de decisão fica num módulo puro (`cone_pose_fix.py`) p/ ser testável
  com pytest direto.
- **Unit do `_set_wp_cone`:** dado um waypoint com pose conhecida, set_cone recomputa
  `cone_bearing` pro valor esperado; `clear:true` zera e baixa `has_cone`; `idx` fora da
  faixa não altera nada. (A parte pura do recálculo de bearing pode virar helper testável.)
- **Bancada (rodas no ar / LiDAR na mesa):**
  - sem cone visível → pose intacta (não publica `pose_fix`); `anchor_status=idle`;
  - objeto fixo (cone) a offset conhecido → pose corrige pro valor esperado (± `gain`);
    UI mostra âncora destacada e `anchor_status` passa `confirming`→`fixed`;
  - objeto **movido** na frente → correção **rejeitada**; UI mostra `confirming` resetando
    ou `ambiguous` (se 2 objetos), com o clutter pintado;
  - na UI em RECORD: gravar um waypoint, clicar nele, clicar outra detecção → o cone do
    waypoint troca; "limpar cone" → waypoint fica sem âncora.
- **Campo:** gravar percurso, repetir autônomo, medir erro de chegada por waypoint
  **com e sem** `enable_cone_pose_fix` — quantificar o ganho real.

## 7. Rollout
- Correção de pose atrás de `enable_cone_pose_fix` (default `true`, gates conservadores).
- Permite A/B em campo (ligar/desligar por parâmetro de launch sem recompilar).
- `trekking.launch.py` ganha o arg correspondente (consistente com os demais).
- Observabilidade e `set_cone` não têm flag — são sempre úteis e de baixo risco.

## 8. Custo estimado
- **Correção de pose:** ~15–25 linhas em 2 nós (`trekking_runner`, `pose_estimator`) +
  6 parâmetros + módulo puro testável. O grosso (detecção de cone, match posição+bearing,
  transform p/ `odom`) **já existe**.
- **Observabilidade:** ~4 campos no state JSON + 1 propriedade no confirmador + render
  no `trekking.js` (cores/badge sobre o loop de desenho que já existe).
- **Correção de cone na gravação:** 1 comando no runner (~12 linhas) + 2 entradas de
  whitelist no `app.py` + hit-test/seleção no `trekking.js`. `trekking_service.py` intacto.
- Sem firmware, sem reflash.
