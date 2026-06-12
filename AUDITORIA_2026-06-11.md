# Auditoria do projeto Controle_robo_web — 2026-06-11 (7ª passada)

Foco desta passada (pedido do usuário): **desempenho + qualidade de código**.
Leitura completa dos nós Python do `robot_nav`, do firmware da MEGA, do stack
web (`controle_web/`) e do `launch.sh`. Os pacotes vendorizados
(`teb_local_planner`, `costmap_converter`) ficaram fora — não são código nosso.
**Tuning do Nav2 ficou fora de propósito**: a saga da PORTA está no meio
(`7b0f3c3` ainda não validado em campo) e o próximo passo estrutural já está
decidido (REMAPEAR com TF correto). Mexer em params agora só contaminaria o teste.

Auditorias anteriores: 05-14, 05-18, 05-26, 05-27, 05-29 (aplicada `1d455fb`),
05-29b (A1 do feedback aplicado em `7115b09`).

> Severidades: 🔴 crítica — 🟠 alta — 🟡 média — 🟢 baixa.
> Prefixo **P** = desempenho, **B** = bug/robustez, **L** = limpeza.
>
> **Estado git:** branch `main`, HEAD `7b0f3c3`, working tree limpo.
> **Baseline de testes:** 70 passed (rodados contra a árvore fonte via
> `PYTHONPATH` — ver L3: o `install/` do PC dev está velho).
>
> **Auditoria ≠ execução** (`feedback_audit_handoff`): aplicar em sessão
> separada. Robô pode ficar **DESLIGADO** pra implementar; só liga pra validar
> (medida de CPU + smoke teleop). Deploy na Pi = `git fetch && reset --hard` +
> **COLCON build robot_nav** (mudanças são em .py instalados). **Sem reflash da
> MEGA** (nenhum achado toca firmware). Sem `Co-Authored-By` nos commits.

---

## Sumário executivo

| # | Sev | O quê | Ganho esperado |
|---|-----|-------|----------------|
| P1 | 🟠 | `EventsExecutor` (Jazzy) nos nós Python | Maior alavanca de CPU da Pi: ataca a causa raiz MEDIDA (executor rclpy remontando wait-set em Python puro) |
| P2 | 🟡 | Vetorizar o `/scan` do unstuck_supervisor (numpy) | ~9000 iterações Python/s com trig → 2 passes numpy |
| P3 | 🟡 | Freshness por `time.monotonic()` no pose_estimator | corta ~200 criações/s de objetos `Time` via rcl |
| P4 | 🟡 | QoS sensor_data em `/hoverboard/wheel_velocities` | menos contabilidade RELIABLE a 50 Hz no DDS |
| B1 | 🟠 | `KNOWN_NODE_PATTERNS` do launch.sh não mata órfão do unstuck_supervisor/twist_mux/collision_monitor | órfão de crash pode dar RÉ fantasma (prio 30!) na sessão seguinte |
| B2 | 🟡 | Salto de tempo no `_tick` não drena o acumulador do flow | janela de flow perdida/velocidade inflada mascarada pelo gate |
| B3 | 🟡 | Log de boot do unstuck mente ("giro desativado") | diagnóstico de campo errado |
| B4–B6, L1–L4 | 🟢 | limpezas diversas | legibilidade/robustez |

Ordem sugerida de aplicação: **B1 → P1 → B2/B3 (caronas do mesmo build) → P2 → P3/P4 → 🟢**.
B1 é 5 linhas de shell e fecha um buraco de segurança; P1 é a tese central da passada.

---

## 🟠 P1 — Trocar o executor dos nós Python pelo `EventsExecutor` (a causa raiz medida do CPU, atacada na raiz)

**Contexto medido (2026-06-09, no robô):** a campanha de CPU provou por
`/proc` (utime vs stime, por-thread) que mega_bridge e pose_estimator queimavam
~70-75% de um core CADA **em user-space na thread do executor rclpy** — serial
e DDS ociosos. O fix da época (lote de 4→1 tópico + drain a 50 Hz) derrubou o
mega_bridge pra ~29,5%, mas o **pose_estimator continua em ~62%**: ele acorda
~250×/s (tick 50 Hz + IMU 50 + rodas 50 + flow 100) e, a cada acordada, o
`SingleThreadedExecutor` do rclpy **remonta o wait-set sobre ~12 entidades em
Python puro**. É exatamente o overhead que o lote não elimina.

**A alavanca:** o Jazzy traz o `EventsExecutor` (`rclpy.experimental`) — fila de
eventos implementada em C++ (pybind), **sem remontagem de wait-set por acordada**.
Verificado disponível no rclpy do Jazzy desta máquina:

```python
from rclpy.experimental import EventsExecutor  # OK no Jazzy
```

**Proposta** — helper único em `robot_nav/utils.py`, usado por TODOS os mains
(`pose_estimator`, `mega_bridge`, `cmd_vel_to_wheels`, `cone_detector`,
`trekking_runner`, `unstuck_supervisor`):

```python
def spin_node(node):
    """EventsExecutor (C++, sem wait-set por acordada) com fallback pro spin
    clássico — rclpy.experimental pode mudar de lugar em distro futura."""
    try:
        from rclpy.experimental import EventsExecutor
    except ImportError:
        import rclpy
        rclpy.spin(node)
        return
    ex = EventsExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        ex.shutdown()
```

e em cada `main()`: `rclpy.spin(node)` → `spin_node(node)` (mantendo o
`try/except KeyboardInterrupt` existente).

**Riscos e mitigação:**
- É API `experimental` — por isso o fallback por ImportError, e por isso NÃO
  proponho tocar nos bridges do Flask (map_service/power_monitor/nav_metrics
  usam `spin_once` em thread própria; o EventsExecutor não tem `spin_once`
  equivalente estável — deixá-los como estão).
- O mega_bridge publica só de dentro de callbacks do executor (timer
  `_drain_rx`) — compatível. A thread `mega_rx` não toca o executor.
- Timers + subscriptions são o caso suportado e testado do EventsExecutor.

**Validação (robô LIGADO, ~10 min):** mesmo método de 2026-06-09 — CPU
por-thread via `/proc` em teleop; conferir `ros2 topic hz /odom` (50 Hz firme)
e TF fluindo. Critério de aceite: pose_estimator caindo de ~62% pra
materialmente menos (qualquer coisa ≥20 pts já paga o risco); rollback = 1
linha por nó (voltar ao `rclpy.spin`).

---

## 🟠 B1 — launch.sh: lista de órfãos não cobre unstuck_supervisor, twist_mux, joy/teleop nem collision_monitor → "ré fantasma" possível após crash

`launch.sh:224-243` (`KNOWN_NODE_PATTERNS`) mata órfãos de execuções
anteriores no boot e no cleanup. A lista parou no tempo: cobre os nós antigos
e o stack Nav2 de 2026-05, mas **não** cobre o que entrou depois:

- `robot_nav/unstuck_supervisor` (sobe no `nav2.launch.py:128-161`)
- `twist_mux` (sobe no `robot.launch.py:189` E no `sim.launch.py:116`)
- `joy_node` / `teleop_node` (robot.launch.py:164-186)
- `collision_monitor` (nav2.launch.py:117-121 — a lista tem `nav2_behaviors`,
  `nav2_amcl` etc., mas o executável do collision monitor não está lá)

**Por que 🟠 e não 🟢:** o cleanup normal (Ctrl+C) mata por árvore de PID e
funciona. O buraco é o **crash** (SIGKILL, queda de energia parcial, OOM): na
relança, um `unstuck_supervisor` órfão da sessão anterior continua assinando
`/odom`+status e **publicando `unstuck_vel` (prioridade 30 do twist_mux, ACIMA
do nav_vel)** — uma ré comandada por um processo que ninguém sabe que existe.
Dois `twist_mux` simultâneos publicando `/cmd_vel` é o mesmo gênero de bug. O
robô bateu de ré em 2026-06-11 por causa MUITO mais sutil que isso; não vale
deixar esse vetor aberto.

**Fix (5 linhas):** adicionar à lista:

```bash
    "robot_nav/unstuck_supervisor"
    "twist_mux"
    "joy_node"
    "teleop_node"
    "collision_monitor"
```

Nota: os padrões casam com o cmdline (`install/robot_nav/lib/robot_nav/...`),
mesmo esquema dos existentes. `pkill -9 -f twist_mux` na Pi não tem
falso-positivo (nada mais roda lá com esse nome).

---

## 🟡 P2 — unstuck_supervisor: processamento do `/scan` em Python puro a 10 Hz → vetorizar com numpy

`unstuck_supervisor.py:405-412` (`_on_scan`) roda **duas** varreduras Python
sobre o scan inteiro (~450 pontos do LD06) a ~10 Hz:

- `rear_min_gap` (linhas 46-72): `math.cos`/`math.sin` + comparações **por ponto**;
- `freer_side` (linhas 75-92): `_norm_angle` (atan2+sin+cos!) **por ponto**.

São ~9.000 iterações/s com trigonometria em Python — na Pi isso é CPU de
verdade e jitter no callback. O `cone_detector.py:104-128` já mostra o padrão
correto no mesmo repo (asarray + máscaras).

**Proposta:** versões vetorizadas das duas funções puras (mesma assinatura,
aceitam list ou ndarray — os 60 testes existentes continuam passando):

```python
def rear_min_gap(ranges, angle_min, angle_increment, lidar_x, tail_x, half_width):
    if angle_increment == 0.0:
        return math.inf
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + np.arange(r.size) * angle_increment
    ok = np.isfinite(r) & (r > 0.0)
    x = lidar_x + r * np.cos(a)
    y = r * np.sin(a)
    sel = ok & (x < tail_x) & (np.abs(y) <= half_width)
    return float((tail_x - x[sel]).min()) if sel.any() else math.inf
```

(`freer_side` análogo: wrap vetorizado `np.arctan2(np.sin(a), np.cos(a))`,
mínimos por máscara de setor). Cuidado de fidelidade: manter o descarte de
`r <= 0` e não-finito idêntico; `None` dentro de ranges não existe em
`LaserScan` real (o parâmetro era defensivo) — cobrir com `np.asarray(...,
dtype=float)` que vira NaN e cai no `isfinite`. Remover também o
`list(msg.ranges)` (cópia desnecessária; `np.asarray` já resolve).

**Validação:** pytest do pacote (os testes chamam as funções puras com listas)
+ olhar o `vao_re` nos logs de transição do unstuck num teste de bancada.

---

## 🟡 B2 — pose_estimator: salto de tempo no `_tick` não drena o acumulador do flow

`pose_estimator.py:342-344`: quando `dt <= 0 or dt > 0.5` o tick retorna cedo —
correto pra não integrar lixo. **Mas** `_flow_dx_accum/_flow_dy_accum`
(alimentados pelo `_on_flow` a 100 Hz) **não são drenados**: o deslocamento da
janela perdida fica acumulado e o próximo tick divide tudo por `dt≈0,02 s` →
velocidade ~25× a real. Hoje o `flow_plausible` (gate de 0,8 m/s) descarta a
amostra, então o efeito prático é "perde a janela de flow" — mas isso é o gate
de EMI **mascarando** um bug aritmético; se um dia o limiar subir, vira
teleporte de pose.

**Fix (2 linhas):** zerar os acumuladores no early-return:

```python
if dt <= 0.0 or dt > 0.5:
    with self._lock:
        self._flow_dx_accum = 0.0
        self._flow_dy_accum = 0.0
    return
```

Carona: adicionar um caso no `test_fused_odom.py` documentando o contrato
("após salto de tempo, deslocamento acumulado é descartado, não re-integrado").

---

## 🟡 B3 — unstuck_supervisor: log de boot diz "giro desativado", mas o giro escalonado está ATIVO

`unstuck_supervisor.py:390-393` imprime `"... -> ré 0.30m; giro desativado"`.
O giro **existe e está ligado**: `_SPINNING` é alcançável via `escalated`
(`escalate_after=2` no `nav2.launch.py:139` — mais agressivo que o default 3,
aliás). A string é fóssil da versão de 2026-06-10 anterior à escalada. Num
incidente de campo ("por que o robô girou sozinho?!") esse log aponta pra
direção errada.

**Fix:** refletir a config real, ex.:
`"unstuck ativo: parado %.0fs → ré %.2fm; %dª no mesmo ponto → +giro %.0f° (lado livre)"`.

---

## 🟡 P3 — pose_estimator: freshness com `get_clock().now()` em todo callback → trocar por `time.monotonic()`

`pose_estimator.py:262,271,335`: `_on_imu` (50 Hz), `_on_flow` (100 Hz) e
`_on_wheels` (50 Hz) chamam `self.get_clock().now()` só pra carimbar
*freshness* — ~200 criações/s de `rclpy.time.Time` atravessando o rcl, mais o
`now - wall` virando objetos `Duration` no tick. Freshness é relógio de parede
local; `time.monotonic()` (float, imune a NTP) é mais barato e mais correto pra
isso. O nó nunca roda com `use_sim_time` (robô real), e o stamp das mensagens
PUBLICADAS continua vindo do clock ROS — só a contagem de idade muda.

**Mudança:** `_last_imu_wall/_last_wheel_wall/_last_flow_wall` viram floats de
`time.monotonic()`; as contas `*_age` no `_tick` viram subtração de float.
Mesma troca vale pro `unstuck_supervisor` (`_on_scan`/`_tick` usam
`get_clock().now().nanoseconds*1e-9` como walltime) e pro throttle do
`mega_bridge._drain_rx` (já usa `time.time()` — trocar por monotonic é carona).

Ganho individual é pequeno; somado ao P1 é a faxina que mantém os callbacks
"só guardam valor" de verdade. Risco ~zero, testes cobrem o núcleo puro.

---

## 🟡 P4 — `/hoverboard/wheel_velocities` em RELIABLE a 50 Hz → QoS sensor_data (mudar pub + TODOS os subs juntos)

`mega_bridge.py:203-204` publica as rodas com `qos_cmd` (RELIABLE depth 10). O
próprio arquivo justifica BEST_EFFORT pra IMU/flow nas linhas 186-190 ("sob
jitter, RELIABLE força reenvio e empilha latência") — e o argumento vale
igualzinho pras rodas: é stream de velocidade a 50 Hz, amostra perdida não
importa (o tick integra o que chegar; `wheel_timeout` já cobre gaps), e
latência empilhada é PIOR que perda (odometria atrasada).

**Atenção (a pegadinha que já mordeu o projeto):** QoS incompatível = silêncio
total — foi exatamente o bug "robô sem IMU" de 2026-06-05. Mudar **no mesmo
commit**: publisher (`mega_bridge.py:203`) e os 3 subscribers:
- `pose_estimator.py:233` (sub com depth 10 → `qos_profile_sensor_data`)
- `power_monitor.py:241`
- `odom_publisher.py:70` (deprecado, mas se ficar no repo tem que casar — ver L2)

**Validação:** `ros2 topic hz /hoverboard/wheel_velocities` (50 Hz) + chip de
tensão da UI vivo (power_monitor recebendo) + `/odom` andando.

---

## 🟢 Baixas (B4-B6) e limpezas (L1-L4)

### B4 — mega_bridge: shadowing de `raw` no `_handle_state`
`mega_bridge.py:387` cria `raw` (dict dos RPMs) e `:402` re-usa `raw` como
variável de loop das baterias (int). Funciona por ordem de execução, mas é
armadilha de manutenção. Renomear o do loop pra `volts`.

### B5 — power_monitor: detector usa `time.time()` (parede)
`power_monitor.py:277` alimenta o `PowerEventDetector` com walltime. Na Pi o
relógio **salta** quando o NTP sincroniza pós-boot (WiFi demora) → janela de
sag de 1 s atravessada por um salto de minutos pode cuspir `sag_*` falso no
CSV exatamente na fase "liguei o robô, ainda nem dirigi". Passar
`time.monotonic()` pro detector e manter `time.time()` SÓ na coluna `ts` do
CSV (correlação com logs ROS). Os testes do detector já usam tempos sintéticos
— só muda a fonte no `_on_tick`.

### B6 — app.py: `_TREKKING_KWARGS` aceita kwargs que o runner ignora
`app.py:623-624` permite `v_max`, `kp_heading`, `kd_heading`, mas o
`trekking_runner._on_cmd` não trata nenhum dos três (parâmetros só via launch).
Whitelist mentindo pra quem lê. Remover os três (ou implementar — remover é
melhor: tuning ao vivo por websocket é convite a acidente).

### L1 — trekking_runner: `_state_tick` publica PoseArray de waypoints sem assinante
`trekking_runner.py:584-596` monta/publica `/trekking/waypoints` a 10 Hz. No
modo trekking real ninguém assina (a UI usa o JSON do `/trekking/state`; era
pra rviz). Gate `get_subscription_count() > 0` — padrão já usado no
pose_estimator linhas 510-528. (O `/trekking/state` em si TEM assinante sempre
— o TrekkingBridge — então não ganha gate.)

### L2 — odom_publisher: nó morto no repo
`odom_publisher.py` está fora de todos os launches desde que o pose_estimator
virou dono do TF (docstring linha 8 já avisa). Manter custa: todo achado de QoS/
geometria precisa ser espelhado nele (ver P4). **Deletar** (git history guarda)
+ tirar o entry point do `setup.py` + tirar `robot_nav/odom_publisher` do
`KNOWN_NODE_PATTERNS` quando aplicar B1. Alternativa conservadora: mover pra
`tools/`.

### L3 — `install/` do PC dev está velho (testes não acham `unstuck_supervisor`)
`pytest` com o `install/setup.bash` do dev falha na coleta
(`No module named 'robot_nav.unstuck_supervisor'`) — o build é anterior ao nó.
Não afeta a Pi (lá o colcon rodou). Rodar `colcon build --packages-select
robot_nav` no dev, ou padronizar rodar pytest com `PYTHONPATH` da árvore fonte
(como esta auditoria fez). Vale uma linha no README de dev.

### L4 — cone_detector: cache do vetor de ângulos
`cone_detector.py:110` recomputa `angle_min + arange(n)*increment` a cada scan
(~10 Hz). Tamanho/params do LD06 não mudam em runtime — cachear por
`(n, angle_min, angle_increment)`. Micro, mas é o tipo de alocação que o
GC da Pi agradece. Só aplicar se passar perto; não justifica commit próprio.

---

## O que esta passada NÃO recomenda (e por quê)

- **Re-tunar Nav2 / collision monitor / inflação** — campo aberto da PORTA
  (`7b0f3c3` não validado) + decisão estrutural já tomada (remapear). Qualquer
  mexida agora contamina o experimento.
- **Vetorizar o `_Decoder.feed` do mega_bridge** (decode byte-a-byte em
  Python, thread `mega_rx`) — dá pra acelerar com `bytes.find()` no resync e
  fatiamento, mas a thread já caiu pra ~29,5% e o protocolo tem casos de
  borda (resync 0xAA 0xAA 0x55) cobertos a dedo. Custo/risco não paga antes
  de P1; reavaliar com números pós-EventsExecutor.
- **Mexer no firmware** — está estável (watchdog + Wire timeout validados,
  flash conferido por readback). Não acordar cachorro que dorme.

## Checklist de aplicação (sessão futura)

> **APLICADO em 2026-06-12** — commits `1c5e9a9` (B1), `e2d6edc` (P1+B2+B3),
> `6902248` (P2), `3e7aac0` (P3), `e483dae` (L2), `864de50` (P4),
> `131b07a` (B4/B5/B6+L1/L3/L4). Falta só deploy na Pi + validação de campo.

1. [x] B1 (launch.sh) — sem build, sem reflash. Commit isolado.
2. [x] P1 (EventsExecutor + helper em utils.py) + B2 + B3 no mesmo build.
3. [x] `pytest` (61 robot_nav + 15 controle_web) verde no dev via PYTHONPATH
   (L3 resolvido: install/ rebuiltado + nota no README).
4. [x] Deploy Pi 2026-06-12: reset --hard pra `18723bd` + COLCON OK; smoke na
   Pi (EventsExecutor importa, 61 testes verdes na própria Pi); executável
   órfão do odom_publisher removido do install/ (colcon não limpa sozinho).
5. [x] **VALIDADO no robô 2026-06-12 (teleop):** CPU por-thread em janela de
   60 s — pose_estimator **62% → 42,7%** (main 40,1% + DDS 1,6%; −19 pts),
   mega_bridge **29,5% → 11,6%** (−18 pts), loadavg 1,45/4 cores.
   `/odom` 49,9 Hz e `/hoverboard/wheel_velocities` 51 Hz (P4 sem silêncio);
   power_monitor recebendo rodas com o QoS novo (CSV 10 Hz, 42,2/41,6 V,
   meas_* preenchidos dirigindo). PENDENTE só o log de boot do unstuck (B3),
   que exige modo nav2 — conferir na próxima sessão de nav2.
6. [x] P2/P3/P4 aplicados (mesma leva); validação de campo é o item 5.
