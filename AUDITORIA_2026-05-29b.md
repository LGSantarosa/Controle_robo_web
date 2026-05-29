# Auditoria do projeto Controle_robo_web — 2026-05-29 (2ª passada do dia)

Sexta passada no total. A [AUDITORIA_2026-05-29.md](./AUDITORIA_2026-05-29.md) (mesma
data, mais cedo) já foi **aplicada e commitada** (`1d455fb`): anel WS2812 comentado
(A1), flow publica amostra nula em vez de suprimir (A2), limpezas M1–M4, e os
pendentes da 05-27 (M3/M4/M6). Esta passada é uma **leitura nova e independente**
do código de baixo nível (firmware, ponte serial, fusão de rodas, web/JS) caçando
bugs e inconsistências que as 5 passadas anteriores não pegaram.

Auditorias anteriores:
- [AUDITORIA_2026-05-14.md](./AUDITORIA_2026-05-14.md) — 1ª (D1 collision_monitor)
- [AUDITORIA_2026-05-18.md](./AUDITORIA_2026-05-18.md) — 2ª (~95 % aplicada)
- [AUDITORIA_2026-05-26.md](./AUDITORIA_2026-05-26.md) — 3ª pós-headless (100 %)
- [AUDITORIA_2026-05-27.md](./AUDITORIA_2026-05-27.md) — 4ª
- [AUDITORIA_2026-05-29.md](./AUDITORIA_2026-05-29.md) — 5ª (anel + flow A2), aplicada em `1d455fb`

> Severidades: 🔴 crítica (bloqueia uso real / quebra dados) — 🟠 alta (bug funcional)
> — 🟡 média (qualidade/robustez/latente) — 🟢 baixa (cosmético/refator).
>
> **Estado git:** branch `main`, HEAD `1d455fb`, working tree limpo no início.
>
> **Auditoria ≠ execução:** registrar decisões, **não** aplicar fixes nesta sessão
> (`feedback_audit_handoff`). Antes de mexer no robô, sincronizar o git da Pi
> (`project_pi_git_desync`). Sem `Co-Authored-By Claude` nos commits.

---

## 🟠 ALTO — bug funcional latente (verificar na bancada antes de aceitar)

### A1 — A inversão da placa TRÁS é aplicada só no caminho de COMANDO, nunca no FEEDBACK → odometria das rodas é corrompida pela placa traseira

Este é o achado central desta passada. É **estrutural e visível no código**; o
impacto real depende de um detalhe de hardware que **não consigo medir daqui** —
por isso peço um teste de 30 s antes de tratar como confirmado (ver "Como verificar").

**O caminho do comando corrige a traseira:**
- `robot.launch.py:92` — `mega_bridge` sobe com `rear_invert_speed: True`
  ("Placa traseira tem motores invertidos E cabos L/R trocados").
- `mega_bridge.py:153-154` — vira `self._rear_speed_sign = -1`.
- `mega_bridge.py:247-254` (`_on_setpoint`) — aplica `_rear_speed_sign`/
  `_rear_steer_sign` **antes** de empacotar o `FT_SET_SPEED`. ✅ Correto.

**O caminho do feedback NÃO corrige nada:**
- `mega_bridge.py:338-354` (`_handle_state`) — publica `rpm_RL`/`rpm_RR` **crus**,
  exatamente como vieram do `speedL_meas`/`speedR_meas` da placa traseira. Nenhum
  `_rear_speed_sign`, nenhum swap de L/R.
- `odom_publisher.py:93-105` (`_set_wheel`) — só tem sinal **por lado**:
  `left_sign` vale pra `fl` **e** `rl`; `right_sign` pra `fr` **e** `rr`. Não
  existe sinal por **placa** (frente vs trás).
- `pose_estimator.py:211-218` (`_set_wheel`) — idêntico: sinal por lado, não por placa.

**Consequência.** A placa traseira foi fisicamente invertida (fase dos motores),
então a RPM que ela mede/reporta quando o robô anda **pra frente** sai com sinal
**oposto** à da placa da frente. Como nada desfaz isso no feedback:

```
odom_publisher._publish_odom:
  v_left  = (v_fl + v_rl) / 2     # v_fl ≈ +, v_rl ≈ −  → v_left  ≈ 0
  v_right = (v_fr + v_rr) / 2     # idem               → v_right ≈ 0
  linear  = (v_left + v_right)/2  ≈ 0   ← anda pra frente, odom diz "parado"
```

E o "cabos L/R trocados" da traseira também troca a contribuição RL↔RR no termo
`angular = (v_right − v_left)/wheel_base`. Resultado: **`/odom` (e o TF
`odom→base_link`) ficam errados** — afeta SLAM (mapa distorcido), AMCL e qualquer
EKF que consuma a odom no robô real.

**Por que pode ter passado batido nas 5 auditorias e em campo:**
- No **trekking**, a pose vem do `pose_estimator` com **flow+IMU dominando** quando
  o flow está bom (α alto); a velocidade das rodas é só *fallback*. O bug fica
  mascarado — até o flow cair (manobra/EMI, justo o caso de A2 da passada anterior),
  quando α→0 e o `pose_estimator` passa a integrar `vx_wheel` corrompido.
- SLAM/Nav2 com odom-de-roda provavelmente só foram exercitados a fundo na
  **simulação** (`sim.launch.py`/`worlds/`), onde o plugin do Gazebo não tem a
  inversão física — então o sintoma não aparece no sim.

**Como verificar (bancada, ~30 s, rodas no ar):**
```bash
# com o robot.launch.py rodando e empurrando "pra frente" pelo PS4:
ros2 topic echo /hoverboard/front/left/velocity   # esperado: mesmo sinal de...
ros2 topic echo /hoverboard/rear/left/velocity    # ...este. Se sinais OPOSTOS → bug confirmado.
```
Se as RPMs de frente e trás saírem com **sinais opostos** dirigindo reto, A1 é real.

**Fix recomendado (quando confirmado):** simetria com o comando — o `mega_bridge`
deve publicar o feedback **já no referencial do chassi**, aplicando a inversa da
mesma correção que aplica no `_on_setpoint`. Em `_handle_state`, antes de publicar:
```python
rpm_RL *= self._rear_speed_sign      # desfaz a inversão de fase da traseira
rpm_RR *= self._rear_speed_sign
# se os cabos L/R da traseira estão trocados, trocar também RL↔RR aqui
# (ou expor rear_swap_lr como parâmetro).
```
Assim o `odom_publisher`/`pose_estimator` recebem as 4 RPMs num referencial
consistente e o sinal por-lado volta a bastar. **Requer só reflash do bridge ROS
(Python), não da MEGA.**

---

## 🟡 MÉDIOS — qualidade / robustez / inconsistência latente

### M1 — `pose_estimator` não recebe `left/right_wheel_sign` do `trekking.launch.py`; diverge do `odom_publisher` se a polaridade for calibrada

- `robot.launch.py:49-56,104-105,117-118` — `left_wheel_sign`/`right_wheel_sign`
  são args do launch e chegam ao `odom_publisher` **e** ao `cmd_vel_to_wheels`.
- `trekking.launch.py:39-55` — o `pose_estimator` recebe só `flow_height`,
  `flow_swap_xy`, `flow_x_sign`. **Não** recebe os sinais de roda.
- `pose_estimator.py:59-60,90-91` — declara `left/right_wheel_sign` (default 1.0)
  e os usa em `_set_wheel`.

Hoje os defaults são todos `1.0`, então não há divergência **viva**. Mas o
`cmd_vel_to_wheels.py:17-20` documenta explicitamente que esses sinais "devem casar"
entre os nós, e o `robot.launch.py` permite trocá-los por argumento. No dia em que
alguém rodar `ros2 launch ... left_wheel_sign:=-1.0`, o `odom_publisher` inverte
mas o `pose_estimator` **não** — a pose fundida do trekking passa a divergir da
odom silenciosamente. Mesma classe de bug que o A1, em versão latente.

**Fix:** passar `left_wheel_sign`/`right_wheel_sign`/`wheel_radius` do
`trekking.launch.py` pro `pose_estimator` (mesmos `LaunchConfiguration` do
`robot.launch.py`), ou centralizar a calibração de roda num YAML único que os
três nós carregam.

### M2 — `map_service`: estado do goal Nav2 escrito pelos callbacks do executor sem o `_wp_lock` que o comentário diz protegê-lo

- `map_service.py:150-165` — comentário: "`_wp_lock` protege ... e o handle do
  goal corrente".
- `map_service.py:411-435` (`_on_goal_response`/`_on_goal_result`) — escrevem
  `self._wp_goal_status` e `self._wp_goal_handle` **sem** pegar `_wp_lock`
  (rodam na thread do executor ROS).
- `map_service.py:298-301` (`stop_waypoints`) — lê `self._wp_goal_handle` **com**
  `_wp_lock`; `_wp_runner` (`:437+`) lê os dois **sem** lock.

Na prática o GIL torna a atribuição de uma referência atômica, então quase nunca
quebra. Mas a proteção é assimétrica: um lado pega o lock, o outro não — o
`stop_waypoints` pode ler `_wp_goal_handle` no exato instante em que
`_on_goal_result` o zera, perdendo o `cancel_goal_async()` de um goal recém-
terminado (inofensivo aqui, mas é exatamente o tipo de race que o lock existe pra
matar). Ou o comentário promete demais, ou falta `with self._wp_lock:` nos dois
callbacks. **Fix:** alinhar — proteger as escritas dos callbacks com o lock, ou
corrigir o comentário pra dizer "single-assignment, dependente do GIL".

---

## 🟢 BAIXOS — cosméticos / dead code / confirmações

### B1 — `nav_metrics._on_odom`: variável `dt` calculada e nunca usada

- `nav_metrics.py:272` — `dt = now - self._last_odom_ts` dentro do bloco de
  acumulação, mas `dt` não aparece em nenhuma linha seguinte (a distância vem do
  `hypot` de pose, a velocidade do `twist`). Sobra de refator. Remover.

### B2 — `pose_estimator` declara/lê `wheel_base` mas nunca usa

- `pose_estimator.py:57,88` — `wheel_base` é declarado e lido pra `self.wheel_base`,
  mas o nó tira `yaw_rate` do IMU e nunca calcula `angular` das rodas, então
  `self.wheel_base` é morto. Inofensivo; remover ou anotar "reservado".

### B3 — `cone_detector` publica quaternion não-unitário nos cones

- `cone_detector.py:173-174` — `p.orientation.x = w` (largura) e `p.orientation.w = 1.0`.
  Norma = √(w²+1) ≠ 1. O `trekking_runner` lê `orientation.x` só como número
  (largura, nunca como rotação), então funciona — mas o rviz e qualquer consumidor
  TF-aware renderizam errado. Já documentado como "uso interno" no header do nó.
  Sem ação; anotação. Se algum dia for pra rviz, mover a largura pra um campo que
  não seja o quaternion.

### B4 — Reconfirmações (sem ação)

Lidos por inteiro nesta passada, sólidos:
- **Protocolo serial** (`protocol.cpp`/`.h` ↔ `mega_bridge._Decoder`): checksum
  XOR8 idêntico nos dois lados, resync `0xAA 0xAA 0x55` tratado em ambos, guarda de
  `len > MAX_PAYLOAD`, fila RX com drop-do-mais-novo. Coerente.
- **`hoverboard.cpp`**: parser de feedback com checksum próprio e `stale()` a 200 ms;
  `txState` zera RPM/bateria de placa muda. OK (a ressalva é o **referencial** do
  feedback da traseira — A1, não o parsing).
- **`sensors_imu.cpp`**: re-init a cada 2 s, descarta quat all-zero, normaliza
  quat, converte giro °/s→rad/s. Bem feito.
- **`sensors_flow.cpp` + `txFlow`**: filtro de EMI (FLOW_MAX_COUNT/SQUAL) + A2
  (amostra nula em vez de suprimida) aplicados e coerentes.
- **`cmd_vel_to_wheels`**: guarda NaN/Inf, saturação por pico preservando a razão
  L/R. OK.
- **`app.py`/`robot_controller.py`**: `WEB_TELEOP` off como no-op em profundidade,
  `force_stop` no disconnect, validação de XY/yaw, `_TREKKING_CMDS`/`_TREKKING_KWARGS`
  como allowlist. OK.
- **`client.js`/`trekking.js`**: `isTypingInField` evita comandar o robô ao digitar
  nome de rota; trail/auto-fit do canvas corretos. OK.

---

## Decisões pendentes (consultar usuário)

- **D2026-05-29b-A** — A1 (feedback da traseira): rodar a verificação de bancada.
  Se confirmado, corrigir no `mega_bridge._handle_state` (recomendado) — é o ponto
  único onde o referencial do chassi pode ser restabelecido pras 4 rodas. Decidir
  também se os cabos L/R da traseira estão trocados (→ precisa swap RL↔RR no feedback).
- **D2026-05-29b-B** — M1: passar os sinais de roda pro `pose_estimator` no
  `trekking.launch.py`, ou migrar a calibração de roda pra um YAML compartilhado
  pelos 3 nós (mais robusto, evita a classe inteira de drift).

## Checklist sugerido (ordem)
1. **A1** — verificar sinais das RPMs na bancada → se opostos, corrigir feedback no `mega_bridge`. (só Python, sem reflash da MEGA)
2. **M1** — propagar sinais de roda pro `pose_estimator` (ou YAML único).
3. **M2** — alinhar lock/comentário do estado de goal no `map_service`.
4. **B1/B2** — remover `dt` morto e `wheel_base` morto. **B3/B4** — anotação, sem fix.

## Notas finais
- **Sem 🔴 confirmado.** O único achado de impacto é o **A1** (odometria de roda da
  placa traseira), e ele é **condicional** a um detalhe de HW que precisa de 30 s de
  bancada pra confirmar — está marcado 🟠, não 🔴, por honestidade: se o feedback da
  traseira não for sign-invertido, A1 cai pra não-issue.
- O tema desta passada é **simetria comando↔feedback** e **consistência de
  parâmetros entre launches**: A1 e M1 são a mesma doença (uma correção/calibração
  aplicada num caminho e esquecida no espelho). Centralizar a calibração de roda
  num YAML mataria as duas de uma vez.
- A stack continua sólida no resto (protocolo, sensores, fusão de flow, web monitor,
  métricas Nav2) — confirmado por leitura linha-a-linha, ver B4.

---

# Execução / verificação em bancada — 2026-05-30

A1 foi **confirmado no robô** (rodas no ar) e o fix evoluiu durante o teste.
Resumo do que se descobriu medindo + observando visualmente:

## Diagnóstico (medido)
- **Andando reto pra frente**, as 4 RPMs cruas saem `(FL,FR,RL,RR)=(+,−,−,+)`:
  espelho L/R intrínseco do hoverboard (roda direita lê invertida) + inversão
  da placa traseira. O modelo por-lado do odom (`left/right_sign`) não consegue
  representar isso → `v_left`/`v_right` cancelavam → odom achava o robô parado
  andando reto. **A1 confirmado.**
- **No giro**, o feedback da traseira saía **invertido vs físico**: durante um giro
  à esquerda FISICAMENTE correto (confirmado a olho: as 2 da esquerda pra trás,
  as 2 da direita pra frente), a traseira publicava o oposto → odom angular
  cancelava. Causa: os **cabos L/R da placa traseira estão trocados** (o
  comentário antigo do `robot.launch.py` estava certo quanto ao swap). Andando
  reto isso é invisível (RL==RR); só aparece no giro.

## Comando (lado de produção): estava CERTO
- `rear_invert_speed=True`, `rear_invert_steer=False` → robô anda reto e **gira
  certo** (verificado a olho). NÃO mexer. A tentativa de `rear_invert_steer=True`
  foi descartada (fazia a traseira contra-girar — vista a olho).

## Fix aplicado (só `mega_bridge.py`, sem reflash da MEGA)
Normalização do feedback no `_handle_state` via `_fb_map` (tópico ← campo+sinal):
```
front/left  = +FL     front/right = -FR
rear/left   = +RR     rear/right  = -RL   (swap L↔R: cabos da traseira trocados)
```
Isto acerta translação **e** rotação. (A 1ª versão usava só sinal por roda, sem
o swap: acertava reto mas cancelava o angular no giro.)

## Verificação final (rodas no ar, com odom_publisher)
| comando | rodas publicadas | `/odom` |
|---|---|---|
| frente | 4 todas + | linear **+**, angular ~0 |
| giro esquerda | FL/RL −, FR/RR + | angular **+0.89** (CCW) |
| giro direita  | FL/RL +, FR/RR − | angular **−1.70** (CW) |

Tudo coerente com o físico observado. `odom_publisher` e `pose_estimator` (que
leem `/hoverboard/*/velocity`) ficam corretos sem nenhuma mudança neles — o fix
vive num único ponto. Nota: `pose_estimator` tira o yaw do IMU, então o angular
do trekking nunca dependeu deste feedback; quem mais ganha é o `/odom` (SLAM/Nav2).

## Pendências
- **Git:** push do dev pro GitHub precisa da credencial do usuário (ambiente do
  Claude não tem). Depois do push, sincronizar a Pi (`git reset --hard origin/main`;
  a Pi tem o `mega_bridge.py` via scp, idêntico ao commit).
- **Tuning (opcional):** o `BASE_ANGULAR_SPEED=6.0` no `robot_controller.py` foi
  inflado "porque o robô não girava no eixo". Agora que o giro/odom estão certos,
  revisar se 6.0 ainda faz sentido ou se dá pra baixar.
