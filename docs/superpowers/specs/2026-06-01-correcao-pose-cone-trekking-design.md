# Design — Correção de pose por cone-âncora no trekking (teach-and-repeat)

**Data:** 2026-06-01
**Status:** design aprovado em conversa; aguardando revisão do spec antes do plano de implementação.

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

**Arquivos envolvidos (já existentes):**
- `ros2_packages/robot_nav/robot_nav/trekking_runner.py` — `_control_tick`, `_find_matching_cone`, `locked_cone`.
- `ros2_packages/robot_nav/robot_nav/pose_estimator.py` — dono de `x/y/yaw`, `_lock`, publica `/trekking/pose`.
- `ros2_packages/robot_nav/robot_nav/cone_detector.py` — `/scan` → `/trekking/cones` (PoseArray em `odom`, largura no `orientation.x`).

## 2. Objetivo

Estender o snap-to-cone para também **corrigir a POSE (x/y) de forma persistente**:
quando o runner confirma um cone gravado, empurrar `x/y` do `pose_estimator` pela deriva
medida (`cone_gravado − cone_observado`), de modo conservador. A correção passa a valer
para **o resto do trajeto**, não só para o waypoint atual.

### Não-objetivos (YAGNI)
- **Yaw:** continua só do IMU (BNO055 dá yaw absoluto bom; um cone só dá posição x/y).
- **SLAM / scan-matching** de scan inteiro (frágil em campo aberto — descartado).
- Usar feições que não sejam cones como landmark.
- Mexer em firmware, MEGA, controle de baixo nível, ou no caminho do alvo (snap-to-cone
  atual permanece **inalterado**; a correção de pose é **aditiva**).

## 3. Requisito crítico de segurança

**Juízes e/ou o operador estarão PERTO do robô** na gravação e no percurso autônomo.
Uma perna de pessoa tem largura de cone (~10–15 cm) e, pior, **se move**. O sistema
**não pode** confundir pessoa com cone e empurrar a pose para o lugar errado.

**Princípio de risco:** um pulo de pose por associação errada é **pior** que a deriva
suave que ele corrige. Portanto o viés é **na dúvida, NÃO corrige** — uma correção
perdida só mantém a (pequena) deriva; uma correção errada teleporta o robô.

## 4. Design

### 4.1 Interface (novo)
- **Tópico novo:** `/trekking/pose_fix` (`geometry_msgs/Vector3Stamped`) — delta de
  correção em `odom` (`vector.x`, `vector.y`; `z` não usado).
- `trekking_runner` **publica** `pose_fix` ao confirmar um cone-âncora.
- `pose_estimator` **assina** `pose_fix` e aplica à pose, com ganho + gates.

### 4.2 Fluxo de dados
1. `cone_detector` → `/trekking/cones` (já existe).
2. `runner._control_tick`: ao entrar no raio do cone esperado, `_find_matching_cone`
   (gate de posição `< cone_match_radius` + bearing `< cone_bearing_tol` — já existe).
3. **NOVO — confirmação antes de corrigir a pose** (não afeta o snap do alvo):
   - **Estabilidade temporal:** o candidato precisa casar na **mesma posição**
     (dentro de `cone_stable_eps`) por `cone_confirm_frames` ciclos seguidos. Cone
     parado confirma; pessoa se movendo reseta o contador → nunca confirma.
   - **Unicidade:** se houver **mais de um** candidato dentro do raio de match, é
     ambíguo (cone + pessoa, ou dois cones) → **não** corrige a pose nesse ciclo.
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
| trekking_runner | `cone_unique_radius` | 0,50 m | se >1 candidato aqui → ambíguo |
| trekking_runner | `enable_cone_pose_fix` | true | liga/desliga a correção de pose |
| pose_estimator | `pose_fix_gain` | 0,5 | fração do delta aplicada (suaviza) |
| pose_estimator | `pose_fix_max` | 0,6 m | acima disso, rejeita (cone errado) |

### 4.5 Resumo das travas de segurança (camadas)
1. **Largura** — `cone_detector` já filtra largura (torso de pessoa é largo demais).
2. **Posição + bearing** — `_find_matching_cone` (já existe).
3. **Estabilidade temporal** (NOVO) — mata pessoa se mexendo.
4. **Unicidade** (NOVO) — mata ambiguidade cone+pessoa / dois cones.
5. **Magnitude** (`pose_fix_max`) — mata teleporte por associação errada.
6. **Ganho parcial + warn** — mesmo aceita, aplica suave e deixa rastro no log.

## 5. Tratamento de erro / fallback
- Sem cone, ou não confirmado, ou rejeitado → **nenhum** `pose_fix` publicado →
  comportamento **idêntico** ao de hoje. Zero risco de regressão fora do caminho novo.
- `enable_cone_pose_fix=false` desliga o recurso inteiro (volta ao snap-só-do-alvo).
- Escrita concorrente de `x/y` protegida pelo `_lock` do `pose_estimator`.

## 6. Testes
- **Unit (sem ROS):** cálculo do delta; gate de magnitude (rejeita > max); confirmação
  temporal (sequência estável confirma, sequência móvel não); unicidade (2 candidatos → skip).
- **Bancada (rodas no ar / LiDAR na mesa):**
  - sem cone visível → pose intacta (não publica `pose_fix`);
  - objeto fixo (cone) a offset conhecido → pose corrige pro valor esperado (± `gain`);
  - objeto **movido** na frente (simula pessoa) → correção **rejeitada** (estabilidade/unicidade).
- **Campo:** gravar percurso, repetir autônomo, medir erro de chegada por waypoint
  **com e sem** `enable_cone_pose_fix` — quantificar o ganho real.

## 7. Rollout
- Recurso atrás de `enable_cone_pose_fix` (default `true`, gates conservadores).
- Permite A/B em campo (ligar/desligar por parâmetro de launch sem recompilar).
- `trekking.launch.py` ganha o arg correspondente (consistente com os demais).

## 8. Custo estimado
~15–25 linhas em 2 nós (`trekking_runner`, `pose_estimator`) + 6 parâmetros + testes.
O grosso (detecção de cone, match posição+bearing, transform p/ `odom`) **já existe**.
Sem firmware, sem reflash.
