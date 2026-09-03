# Plano do robô REAL — relê → BNO055 → o pacote de hoje

**Data:** 2026-09-03 · **Branch:** `arena-galpao` · **Prazo:** 05/09
**Para revisão do Codex.** Tudo o que segue foi validado **só no sim**. O robô real
não rodou nada disto.

Ordem definida pelo dono: **1) relê · 2) IMU BNO055 · 3) o que fizemos hoje.**

---

## ⚠️ 0. Antes de ligar o robô — 4 coisas que mordem

**(a) `--arena` desliga o `motion_guard` no REAL também.** O perfil passa
`motion_guard:=false` (decisão de 08-31, §2B.8: a vigília de pessoa disparava em cima
de cone e travava o robô 26-52 s). No sim é inofensivo; **no robô físico significa
subir sem vigia de pessoa** → pista controlada e E-STOP na mão. Está no item `2j` do
§6 do diário.

**(b) Deploy é por git, nunca `scp`.** Na Pi: `git fetch && git reset --hard
origin/<branch>` + `colcon build --packages-select robot_nav`. Repo da Pi:
`~/workspace/Controle_robo_web`. O `face_app` roda **separado** e precisa ser
reiniciado no deploy.

**(c) A branch é `arena-galpao`, não `main`.** Todo o trabalho de hoje está nela.

**(d) 2 testes quebrados no working tree**, herdados de antes desta sessão
(`test_rotating_is_proportional_slows_near_target`, `test_restage_when_aligned_but_wont_fit`).
Conferido que **já falhavam em `3c2b051`**. Não são regressão de hoje, mas estão lá.

---

## 1. Relê — o que existe e o que testar

### Encanamento (todo implementado, nada validado no robô)

| camada | onde |
|---|---|
| tópicos | `/light/cmd` (Bool) = relê · `/light/marker` (Bool) = LED de marco |
| nó | `ros2_packages/robot_nav/robot_nav/mega_bridge.py:264-265` |
| frame serial | `FT_RELAY`, 2 bytes: `[light, marker]` (`mega_bridge.py:342`) |
| firmware | `firmware/mega_bridge/src/main.cpp:99` → `io_signals::setRelay/setMarkerLed` |
| **pinos** | **`PIN_RELAY = 7`, `PIN_LED = 8`** (`firmware/mega_bridge/include/io_signals.h:6-7`) |
| acionamento automático | `controle_web/map_service.py:1028` publica em `/light/cmd` ao concluir waypoint (commit `8d8c99e`, do próprio dono, hoje 10:58) |
| log | `controle_web/logs/light_events.log` |

### Teste mínimo sugerido (bancada, robô sem tração)

1. `ros2 topic pub --once /light/cmd std_msgs/Bool "{data: true}"` → relê fecha
2. idem `false` → abre
3. idem `/light/marker` → LED do pino 8
4. Só então a volta, conferindo que a luz acende sozinha a cada waypoint concluído
   (o log da volta de hoje mostra `luz do objetivo acionada (waypoint N/7)` nos
   waypoints 1, 3, 5, 6 e 7)

**Pergunta pro Codex:** o `_send_relay` manda os **dois** estados em todo frame, e
`/light/cmd` e `/light/marker` são subscritos separados. Se só um chegar, o outro é
reenviado com o valor que o nó tem em memória. Isso é seguro no boot, antes de
qualquer publicação? (`mega_bridge.py:255` tem um comentário sobre isso — vale
conferir se o default é o desejado para um **relê**, que pode estar ligado a algo de
potência.)

---

## 2. IMU BNO055 — já está no código, falta validar

**Não é trabalho novo.** O `mega_bridge` já a trata como IMU #2:

| | |
|---|---|
| I²C | `0x28/0x29` |
| tópicos | `/imu2/data` (com `orientation`) · `/imu2/mag` (tesla) |
| frame | `FT_IMU2 = 0x85` — quat + gyro + accel + mag + calib |
| quaternion | LSB fixo `1/16384` (2^14) — `mega_bridge.py:63` |
| calibração | `sys/gyro/accel/mag`, 0..3 cada, entra no `/system/health` (`:235`, `:244`) |
| detecção | automática no I²C da MEGA (`:455`) |

**O que a BNO055 resolve:** yaw **absoluto**. A MPU6050 de hoje (validada em campo
02/07, `yaw_source=imu`, 51 Hz) dá yaw **relativo**, que deriva. Há um BO aberto
(`project_imu_plausibility_gate_bo`): IMU com mau contato fazia a pose girar com o
robô parado, e não existe gate de plausibilidade.

### Teste mínimo sugerido

1. `ros2 topic echo /imu2/data --once` → confirmar que **existe** (detecção I²C ok)
2. `ros2 topic echo /system/health` → ler os 4 números de calibração; **girar o robô
   em 8** até `mag` e `sys` chegarem a 3
3. Girar o robô 360° no lugar e conferir que o yaw do quaternion **fecha** (volta ao
   valor inicial) — é o teste que a MPU6050 não passa
4. Comparar `/imu2/data` com `/imu/data` (MPU6050) parados por 60 s: quanto cada uma
   deriva

**Perguntas pro Codex:**
- A fusão hoje usa qual IMU? O `test_fused_odom.py` cita BNO055 — a troca é por
  parâmetro ou exige código?
- Faz sentido usar a BNO055 **só** como referência absoluta de yaw (correção lenta) e
  manter a MPU6050 na malha rápida, ou trocar de vez?
- O gate de plausibilidade do BO acima deveria entrar **junto** com a BNO055, já que
  passa a haver duas fontes para comparar?

---

## 3. O pacote de hoje — o que vai pro real, e o que ele muda

Tudo abaixo tem teste e foi medido no sim. **Zero validação no robô.**

### 3.1 `door_crossing`: `exit_margin` 0,50 → 0,60

A porta da arena é feita de 2 cones de R=0,17 → **0,34 m de profundidade física**.
Soltando em 0,50 sobravam 8 cm atrás da traseira; um pivô precisa de meia-diagonal
0,354 + cone 0,17 = **0,524 m**. Faltavam 2,4 cm, e o robô girou 180° ali dentro,
arrastando os dois cones.

⚠️ **No real, a "porta" pode não ser cone.** Se o batente for parede fina, 0,60 é
folgado; se for mais fundo, é curto. O modelo **não tem profundidade declarável** —
item `2o` do §6, com plano em `docs/2026-09-03-vao-com-profundidade-plano.md`,
parqueado por decisão do dono.

### 3.2 `path_follower`: não acumula estado enquanto o `door_crossing` dirige

Assina `/door_zone`; nos estados em que a porta conduz, zera `_aim_filt`,
`_turn_target`, `_prev_yaw`, `_yaw_rate` e devolve `preempted`.
**Falha segura:** se o `/door_zone` travar sujo, o pior caso é o robô **parar**.

### 3.3 `path_follower`: saída reta de 0,8 m após a travessia

`wz = 0` forçado por 0,8 m / 4 s / até `front_clear < 0,5 m`. Impede o pivô de 180°
logo na saída do vão. **Incondicional de propósito.**

⚠️ **Achado consistente em 4 corridas:** um dos episódios sempre termina pela
**guarda de folga**, não pela distância — sinal de que a janela arma também depois de
um **abort** do `door_crossing`, não só de travessia concluída. O `/door_zone` publica
`idle` nos dois casos. Item `2p`, **não consertado**.

### 3.4 `path_follower`: guard estreito (o conserto do travamento) — **o mais crítico**

Era: `if pose is None or not goal_active or not path or len(path) < 2:` → `idle`.
O plano global **encolhe** conforme o robô converge (`n=4 → 3 → 2 → 1`), e ao colapsar
matava o `goal_turn` no meio. O robô ficava fora do `yaw_goal_tolerance`, o Nav2 nunca
fechava o goal, o app nunca mandava o próximo: **travava para sempre**. 3 corridas
seguidas.

Agora: `len(path) < 2` só barra o **carrot**; a fase de chegada roda com 1 pose.
+ `idle_reason` e log `FOLLOW_IDLE motivo=...`.

**Prova de que foi exercitado** (`docs/baselines/2026-09-03-arena-PROVA-conserto-guard-exercitado/`):
plano caiu a 1 pose com **44,1° de erro de yaw restante**, o giro continuou 22 ticks
e o goal fechou 1,16 s depois. As 3 corridas travadas morreram nesse mesmo ponto com
32,2° / 23,7° / 38,0°.

⚠️ **Ressalva:** com plano de 1 pose, o `goal_yaw` vem da orientação **daquela única
pose** (`_on_plan`, `path_follower.py:704`). Estava certo nas corridas observadas,
mas não é garantia.

### 3.5 Portas 3 e 4 removidas do `doors.json`

Por ordem do dono. ⚠️ Elas **continuam existindo no mundo e no mapa**, e a rota
continua com 5 waypoints — o robô ainda passa por elas, com Nav2 puro.

---

## 4. O que segue ABERTO e vale a opinião do Codex

| item | o quê |
|---|---|
| `2q` | Preparação da porta 2 custa 20-25 s no limite-ciclo `staging ↔ rotating`. **Causa provável medida:** o Nav2 fecha o goal com `yaw_goal_tolerance` = 20,05° enquanto o `path_follower` exige 6° — **5 de 7 goals fecharam com 14,3° a 19,9° de erro**. O robô entra na aproximação torto. **Falta a decisão: quem é o dono do yaw final perto da porta?** Afrouxar a tolerância é curativo e foi rejeitado por mim e pelo Codex até essa decisão |
| `2p` | Janela de saída reta arma depois de *abort*, não só de travessia |
| `2o` | Porta é uma LINHA; profundidade não declarável |
| 8 | `start/goal is an obstacle` — plano nasce apontando pra trás. É a raiz dos BOs de hoje; os consertos só impedem que ele custe cones |
| — | 2 testes quebrados herdados |

---

## 5. Números do sim, para comparar com o real

Última volta (16:16), **7/7 goals, 222,3 s**:

| porta | total | preparação | travessia | tentativas |
|---|---|---|---|---|
| 1 | 8,1 s | 1,4 s | 6,8 s | 1 |
| 2 | 31,2 s | 24,8 s | 6,4 s | 3 |

`exit_straight`: 3 episódios, `wz = 0` nos três (0,79 / 0,34 / 0,80 m).

Baselines em `docs/baselines/2026-09-03-*`. Narrativa completa no `DIARIO_ARENA.md`,
§2H.23 a §2H.41.
